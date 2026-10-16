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

#include "autoware/ptv3/postprocess/detection3d_postprocess.hpp"

#include "ptv3_test_fixture.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace autoware::ptv3
{
namespace test
{

constexpr std::size_t num_classes{2};
constexpr std::size_t num_anchor{4};

// CAR and PEDESTRIAN with thresholds per distance bin, pedestrians alone in a circle NMS group.
PTv3Config makePostprocessConfig()
{
  PTv3ConfigParams params;
  params.use_seg3d_head = false;
  params.use_det3d_head = true;
  params.voxels_num = {1, 4, 8};
  params.pooled_voxels_num_max = {8, 8, 8, 8};
  params.point_cloud_range = {0.0F, 0.0F, 0.0F, 16.0F, 16.0F, 4.0F};
  params.voxel_size = {1.0F, 1.0F, 1.0F};
  params.pooling_strides = {2, 2, 2, 2};
  params.enc_channels = {8, 16, 32, 64, 128};
  params.patch_sizes = {4, 4, 4, 4, 4};
  params.distance_bin_upper_limits = {10.0F, 20.0F};
  params.detection_score_thresholds = {0.1F, 0.2F, 0.3F, 0.4F};
  params.num_anchor = num_anchor;
  params.num_temp_instances = 2;
  params.circle_nms_class_groups = {-1, 0};
  params.circle_nms_radii = {0.5F};
  return makeConfig(params);
}

Box3D makeBox(const int label, const float score, const float x, const float y)
{
  Box3D box;
  box.label = label;
  box.score = score;
  box.x = x;
  box.y = y;
  return box;
}

float logit(const float probability)
{
  return std::log(probability / (1.0F - probability));
}

TEST(Detection3DPostprocessTest, DecodesEveryQueryIntoTheBoxOfItsBestClass)
{
  const Detection3DPostprocess postprocess(makePostprocessConfig());
  std::vector<float> classification(num_anchor * num_classes, logit(0.01F));
  std::vector<float> prediction(num_anchor * Detection3DPostprocess::anchor_dim, 0.0F);
  std::vector<float> quality(num_anchor * Detection3DPostprocess::quality_dim, 0.0F);
  classification[2 * num_classes + 0] = logit(0.3F);
  classification[2 * num_classes + 1] = logit(0.8F);
  quality[2 * Detection3DPostprocess::quality_dim] = logit(0.5F);
  float * state = prediction.data() + 2 * Detection3DPostprocess::anchor_dim;
  const std::vector<float> anchor_state{
    1.0F,           2.0F,           0.5F,           std::log(4.0F), std::log(2.0F),
    std::log(1.5F), std::sin(0.3F), std::cos(0.3F), 1.5F,           -0.5F};
  std::copy(anchor_state.begin(), anchor_state.end(), state);

  const auto boxes =
    postprocess.decodeQueries(classification.data(), prediction.data(), quality.data());

  ASSERT_EQ(boxes.size(), num_anchor);
  const auto & box = boxes[2];
  EXPECT_EQ(box.label, 1);
  EXPECT_NEAR(box.score, 0.4F, 1e-5F);
  EXPECT_FLOAT_EQ(box.x, 1.0F);
  EXPECT_FLOAT_EQ(box.y, 2.0F);
  EXPECT_FLOAT_EQ(box.z, 0.5F);
  EXPECT_NEAR(box.length, 4.0F, 1e-5F);
  EXPECT_NEAR(box.width, 2.0F, 1e-5F);
  EXPECT_NEAR(box.height, 1.5F, 1e-5F);
  EXPECT_NEAR(box.yaw, 0.3F, 1e-5F);
  EXPECT_FLOAT_EQ(box.vel_x, 1.5F);
  EXPECT_FLOAT_EQ(box.vel_y, -0.5F);
  EXPECT_EQ(box.track_id, -1);
}

TEST(Detection3DPostprocessTest, KeepsTheBoxesReachingTheThresholdOfTheirDistance)
{
  const Detection3DPostprocess postprocess(makePostprocessConfig());

  const auto boxes = postprocess.select({
    makeBox(0, 0.15F, 5.0F, 0.0F),   // first bin, car threshold 0.1
    makeBox(0, 0.25F, 15.0F, 0.0F),  // second bin, car threshold 0.3
    makeBox(1, 0.35F, 0.0F, 6.0F),   // first bin, pedestrian threshold 0.2
    makeBox(0, 0.95F, 25.0F, 0.0F),  // past the last bin
  });

  ASSERT_EQ(boxes.size(), 2U);
  EXPECT_FLOAT_EQ(boxes[0].score, 0.35F);
  EXPECT_FLOAT_EQ(boxes[1].score, 0.15F);
}

TEST(Detection3DPostprocessTest, SuppressesOnlyWithinACircleNmsGroup)
{
  const Detection3DPostprocess postprocess(makePostprocessConfig());

  const auto boxes = postprocess.select({
    makeBox(1, 0.6F, 1.0F, 1.0F),
    makeBox(1, 0.9F, 1.3F, 1.0F),  // suppresses the pedestrian 0.3 m away
    makeBox(1, 0.5F, 2.0F, 1.0F),  // 0.7 m away from the better pedestrian
    makeBox(0, 0.8F, 1.2F, 1.0F),  // cars are never suppressed
    makeBox(0, 0.7F, 1.1F, 1.0F),
  });

  ASSERT_EQ(boxes.size(), 4U);
  EXPECT_FLOAT_EQ(boxes[0].score, 0.9F);
  EXPECT_FLOAT_EQ(boxes[1].score, 0.8F);
  EXPECT_FLOAT_EQ(boxes[2].score, 0.7F);
  EXPECT_FLOAT_EQ(boxes[3].score, 0.5F);
}

}  // namespace test
}  // namespace autoware::ptv3
