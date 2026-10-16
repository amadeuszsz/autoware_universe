// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/ptv3/ros_utils.hpp"

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::ptv3
{
namespace test
{
using autoware_perception_msgs::msg::ObjectClassification;

TEST(RosUtilsTest, MapsDetectionClassNames)
{
  EXPECT_EQ(get_classification_type("CAR"), ObjectClassification::CAR);
  EXPECT_EQ(get_classification_type("MOTORBIKE"), ObjectClassification::MOTORCYCLE);
  EXPECT_EQ(get_classification_type("TRAFFIC_CONE"), ObjectClassification::HAZARD);
  EXPECT_EQ(get_classification_type("UNKNOWN_CLASS"), ObjectClassification::UNKNOWN);
}

TEST(RosUtilsTest, ConvertsBox3DToDetectedObject)
{
  constexpr float pi = 3.14159265358979323846F;
  const Box3D box{1, 0.75F, 1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 0.5F * pi, 1.0F, 2.0F};
  autoware_perception_msgs::msg::DetectedObject object;

  box3d_to_detected_object(box, {"CAR", "PEDESTRIAN"}, object);

  ASSERT_EQ(object.classification.size(), 1U);
  EXPECT_EQ(object.classification.front().label, ObjectClassification::PEDESTRIAN);
  EXPECT_FLOAT_EQ(object.existence_probability, 0.75F);
  EXPECT_FLOAT_EQ(object.kinematics.pose_with_covariance.pose.position.x, 1.0F);
  EXPECT_FLOAT_EQ(object.shape.dimensions.x, 4.0F);
  EXPECT_TRUE(object.kinematics.has_twist);
  EXPECT_NEAR(object.kinematics.twist_with_covariance.twist.linear.x, 2.0F, 1e-5F);
  EXPECT_NEAR(object.kinematics.twist_with_covariance.twist.linear.y, -1.0F, 1e-5F);
}

TEST(RosUtilsTest, ConvertsUnknownBoxLabel)
{
  const Box3D box{99, 0.5F, 0.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 0.0F, 1.0F, 1.0F};
  autoware_perception_msgs::msg::DetectedObject object;

  box3d_to_detected_object(box, {"CAR"}, object);

  ASSERT_EQ(object.classification.size(), 1U);
  EXPECT_EQ(object.classification.front().label, ObjectClassification::UNKNOWN);
}

TEST(RosUtilsTest, TrackIdentifiersKeepTheSessionAndTheTrackApart)
{
  const auto uuid = track_uuid(0x0807060504030201ULL, 0x1817161514131211LL);
  for (std::size_t byte = 0; byte < 8; ++byte) {
    EXPECT_EQ(uuid.uuid[byte], 0x11 + byte);
    EXPECT_EQ(uuid.uuid[8 + byte], byte + 1);
  }
  EXPECT_NE(track_uuid(1, 7).uuid, track_uuid(2, 7).uuid);
  EXPECT_NE(track_uuid(1, 7).uuid, track_uuid(1, 8).uuid);
}

TEST(RosUtilsTest, TrackIdentifiersOfASessionDifferInTheirFirstBytes)
{
  const std::uint64_t session_id = 0x7b73a1b2c3d4e5f6ULL;
  const auto first = track_uuid(session_id, 1);
  const auto second = track_uuid(session_id, 2);
  const auto far = track_uuid(session_id, 257);
  EXPECT_NE(first.uuid[0], second.uuid[0]);
  EXPECT_NE(std::make_pair(first.uuid[0], first.uuid[1]), std::make_pair(far.uuid[0], far.uuid[1]));
}

TEST(RosUtilsTest, ConvertsADetectedObjectIntoATrackedObject)
{
  const Box3D box{0, 0.75F, 1.0F, 2.0F, 3.0F, 4.0F, 2.0F, 1.5F, 0.0F, 3.0F, 0.0F};
  autoware_perception_msgs::msg::DetectedObject detected_object;
  box3d_to_detected_object(box, {"CAR"}, detected_object);

  const auto object_id = track_uuid(1, 7);
  const auto tracked_object = detected_object_to_tracked_object(detected_object, object_id);

  EXPECT_EQ(tracked_object.object_id, object_id);
  EXPECT_FLOAT_EQ(tracked_object.existence_probability, 0.75F);
  EXPECT_EQ(tracked_object.classification, detected_object.classification);
  EXPECT_EQ(
    tracked_object.kinematics.pose_with_covariance,
    detected_object.kinematics.pose_with_covariance);
  EXPECT_NEAR(tracked_object.kinematics.twist_with_covariance.twist.linear.x, 3.0, 1e-6);
  EXPECT_EQ(tracked_object.shape, detected_object.shape);
}

class ClassificationMappingTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  static rclcpp::Node::SharedPtr makeNode(const std::vector<rclcpp::Parameter> & overrides)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    return std::make_shared<rclcpp::Node>("class_mapping_test_node", options);
  }
};

TEST_F(ClassificationMappingTest, ResolvesMappingKeyedByClassName)
{
  const auto node = makeNode(
    {{"segmentation3d.class_mapping.car", "CAR"},
     {"segmentation3d.class_mapping.traffic_cone", "HAZARD"}});

  const auto class_mapping = declare_class_mapping(
    *node, {"car", "traffic_cone"}, rcl_interfaces::msg::ParameterDescriptor{});

  ASSERT_EQ(class_mapping.size(), 2U);
  EXPECT_EQ(class_mapping.at("car"), "CAR");
  EXPECT_EQ(class_mapping.at("traffic_cone"), "HAZARD");
}

TEST_F(ClassificationMappingTest, SkipsMappingEntriesNotInClassNames)
{
  const auto node = makeNode(
    {{"segmentation3d.class_mapping.car", "CAR"},
     {"segmentation3d.class_mapping.vegetation", "VEGETATION"}});

  const auto class_mapping =
    declare_class_mapping(*node, {"car"}, rcl_interfaces::msg::ParameterDescriptor{});

  ASSERT_EQ(class_mapping.size(), 1U);
  EXPECT_EQ(class_mapping.at("car"), "CAR");
  EXPECT_EQ(class_mapping.count("vegetation"), 0U);
}

TEST_F(ClassificationMappingTest, ThrowsWhenClassIsNotMapped)
{
  const auto node = makeNode({{"segmentation3d.class_mapping.car", "CAR"}});

  EXPECT_THROW(
    declare_class_mapping(*node, {"car", "truck"}, rcl_interfaces::msg::ParameterDescriptor{}),
    std::runtime_error);
}

}  // namespace test
}  // namespace autoware::ptv3
