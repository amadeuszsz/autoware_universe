// Copyright 2025 TIER IV, Inc.
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

#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::ptv3
{

using Label = autoware_perception_msgs::msg::ObjectClassification;

void box3d_to_detected_object(
  const Box3D & box3d, const std::vector<std::string> & class_names,
  autoware_perception_msgs::msg::DetectedObject & obj)
{
  obj.existence_probability = box3d.score;

  Label classification;
  classification.probability = 1.0F;
  if (box3d.label >= 0 && static_cast<std::size_t>(box3d.label) < class_names.size()) {
    classification.label = get_classification_type(class_names[box3d.label]);
  } else {
    classification.label = Label::UNKNOWN;
  }

  if (autoware::object_recognition_utils::isCarLikeVehicle(classification.label)) {
    obj.kinematics.orientation_availability =
      autoware_perception_msgs::msg::DetectedObjectKinematics::SIGN_UNKNOWN;
  }

  obj.classification.emplace_back(classification);

  const float yaw = box3d.yaw;
  obj.kinematics.pose_with_covariance.pose.position =
    autoware_utils::create_point(box3d.x, box3d.y, box3d.z);
  obj.kinematics.pose_with_covariance.pose.orientation =
    autoware_utils::create_quaternion_from_yaw(yaw);
  obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  obj.shape.dimensions =
    autoware_utils::create_translation(box3d.length, box3d.width, box3d.height);

  geometry_msgs::msg::Twist twist;
  twist.linear.x = std::cos(yaw) * box3d.vel_x + std::sin(yaw) * box3d.vel_y;
  twist.linear.y = -std::sin(yaw) * box3d.vel_x + std::cos(yaw) * box3d.vel_y;
  obj.kinematics.twist_with_covariance.twist = twist;
  obj.kinematics.has_twist = true;
}

unique_identifier_msgs::msg::UUID track_uuid(
  const std::uint64_t session_id, const std::int64_t track_id)
{
  unique_identifier_msgs::msg::UUID uuid;
  const auto track = static_cast<std::uint64_t>(track_id);
  for (std::size_t byte = 0; byte < 8; ++byte) {
    const auto shift = 8 * byte;
    uuid.uuid[byte] = static_cast<std::uint8_t>(track >> shift);
    uuid.uuid[8 + byte] = static_cast<std::uint8_t>(session_id >> shift);
  }
  return uuid;
}

autoware_perception_msgs::msg::TrackedObject detected_object_to_tracked_object(
  const autoware_perception_msgs::msg::DetectedObject & detected_object,
  const unique_identifier_msgs::msg::UUID & object_id)
{
  autoware_perception_msgs::msg::TrackedObject tracked_object;
  tracked_object.object_id = object_id;
  tracked_object.existence_probability = detected_object.existence_probability;
  tracked_object.classification = detected_object.classification;
  tracked_object.kinematics.pose_with_covariance = detected_object.kinematics.pose_with_covariance;
  tracked_object.kinematics.twist_with_covariance =
    detected_object.kinematics.twist_with_covariance;
  tracked_object.kinematics.orientation_availability =
    detected_object.kinematics.orientation_availability;
  tracked_object.kinematics.is_stationary = false;
  tracked_object.shape = detected_object.shape;
  return tracked_object;
}

std::uint8_t get_classification_type(const std::string & class_name)
{
  if (class_name == "CAR") {
    return Label::CAR;
  }
  if (class_name == "TRUCK") {
    return Label::TRUCK;
  }
  if (class_name == "BUS") {
    return Label::BUS;
  }
  if (class_name == "TRAILER") {
    return Label::TRAILER;
  }
  if (class_name == "MOTORBIKE" || class_name == "MOTORCYCLE") {
    return Label::MOTORCYCLE;
  }
  if (class_name == "BICYCLE") {
    return Label::BICYCLE;
  }
  if (class_name == "PEDESTRIAN") {
    return Label::PEDESTRIAN;
  }
  if (class_name == "ANIMAL") {
    return Label::ANIMAL;
  }
  if (class_name == "TRAFFIC_CONE") {
    return Label::HAZARD;
  }
  if (class_name == "BARRIER") {
    return Label::HAZARD;
  }
  if (class_name == "DEBRIS") {
    return Label::HAZARD;
  }
  // Autoware has no train label, a train is published as an unknown object with its box.
  if (class_name == "TRAIN") {
    return Label::UNKNOWN;
  }
  return Label::UNKNOWN;
}

std::unordered_map<std::string, std::string> declare_class_mapping(
  rclcpp::Node & node, const std::vector<std::string> & class_names,
  const rcl_interfaces::msg::ParameterDescriptor & descriptor)
{
  std::unordered_map<std::string, std::string> class_mapping;
  class_mapping.reserve(class_names.size());

  // The mapping keys live in ptv3.param.yaml while class_names comes from the ml_package parameter
  // file, so report which one is out of sync when an entry is missing.
  std::optional<std::string> missing_classes;
  for (const auto & class_name : class_names) {
    const std::string param_name = "segmentation3d.class_mapping." + class_name;
    const auto mapped_class =
      node.declare_parameter<std::string>(param_name, std::string{}, descriptor);
    if (!mapped_class.empty()) {
      class_mapping.emplace(class_name, mapped_class);
      continue;
    }

    if (missing_classes) {
      *missing_classes += ", " + class_name;
    } else {
      missing_classes = class_name;
    }
  }

  if (missing_classes) {
    throw std::runtime_error(
      "segmentation3d.class_mapping is missing entries for segmentation3d.class_names: [" +
      *missing_classes + "].");
  }
  return class_mapping;
}

}  // namespace autoware::ptv3
