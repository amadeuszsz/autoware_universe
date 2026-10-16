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

#include "autoware/ptv3/ptv3_config.hpp"

#include <autoware/point_types/types.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::ptv3
{
namespace test
{

// Detection head of two classes, the second one alone in a circle NMS group.
struct DetectionHeadParams
{
  std::vector<float> distance_bin_upper_limits{10.0F, 20.0F};
  std::vector<float> detection_score_thresholds{0.1F, 0.2F, 0.3F, 0.4F};
  std::int64_t num_anchor{8};
  std::int64_t num_temp_instances{4};
  std::int64_t embed_dims{16};
  float default_time_interval{0.1F};
  float max_time_interval{2.0F};
  float track_birth_score{0.2F};
  std::vector<std::int64_t> circle_nms_class_groups{-1, 0};
  std::vector<float> circle_nms_radii{0.5F};
};

PTv3Config makeDetectionConfig(
  const std::vector<float> & point_cloud_range = {0.0F, 0.0F, 0.0F, 16.0F, 16.0F, 4.0F},
  const std::vector<float> & voxel_size = {1.0F, 1.0F, 1.0F},
  const std::vector<std::int64_t> & voxels_num = {1, 4, 8},
  const std::vector<std::int64_t> & pooled_voxels_num_max = {8, 8, 8, 8},
  const DetectionHeadParams & head = {})
{
  return PTv3Config(
    false, true, "", 8, voxels_num, pooled_voxels_num_max, point_cloud_range, voxel_size, 2, {}, {},
    {"z", "z-trans"}, {2, 2, 2, 2}, {8, 16, 32, 64, 128}, {4, 4, 4, 4, 4}, {}, {}, "", false, "",
    {}, {"CAR", "PEDESTRIAN"}, head.distance_bin_upper_limits, head.detection_score_thresholds,
    head.num_anchor, head.num_temp_instances, head.embed_dims, head.default_time_interval,
    head.max_time_interval, head.track_birth_score, head.circle_nms_class_groups,
    head.circle_nms_radii);
}

PTv3Config makeDetectionConfig(const DetectionHeadParams & head)
{
  return makeDetectionConfig(
    {0.0F, 0.0F, 0.0F, 16.0F, 16.0F, 4.0F}, {1.0F, 1.0F, 1.0F}, {1, 4, 8}, {8, 8, 8, 8}, head);
}

// Segmentation-only config over three levels (two pooling stages), so the attention window can
// differ per level.
PTv3Config makeWindowedSegmentationConfig(const std::vector<std::int64_t> & patch_sizes = {4, 4, 4})
{
  return PTv3Config(
    true, false, "", 8, {1, 4, 8}, {8, 8}, {-1.0F, -1.0F, -1.0F, 3.0F, 3.0F, 3.0F},
    {1.0F, 1.0F, 1.0F}, 2, {"noise"}, {{"noise", "NOISE"}}, {"z", "z-trans"}, {2, 2}, {8, 16, 32},
    patch_sizes, {0, 0, 0}, {}, "xyzi", false, "partial", {0, 0});
}

TEST(PTv3ConfigTest, RejectsPatchSizesThatDoNotCoverEveryStage)
{
  EXPECT_THROW(makeWindowedSegmentationConfig({4, 4}), std::runtime_error);
  EXPECT_THROW(makeWindowedSegmentationConfig({4, 0, 4}), std::runtime_error);
}

TEST(PTv3ConfigTest, PadsVoxelCountsToWholeAttentionWindows)
{
  const auto config = makeWindowedSegmentationConfig();
  EXPECT_EQ(config.padded_voxel_count(0, 0), 0);
  EXPECT_EQ(config.padded_voxel_count(1, 0), 4);
  EXPECT_EQ(config.padded_voxel_count(4, 0), 4);
  EXPECT_EQ(config.padded_voxel_count(5, 1), 8);
  EXPECT_EQ(config.padded_voxel_count(9, 2), 12);
}

// Segmentation-only config exercising segmentation3d.class_mapping resolution.
PTv3Config makeSegmentationConfig(
  const std::vector<std::string> & segmentation_class_names,
  const std::unordered_map<std::string, std::string> & segmentation_class_mapping)
{
  std::vector<std::int64_t> palette(segmentation_class_names.size() * 3, 0);
  return PTv3Config(
    true, false, "", 8, {1, 4, 8}, {8, 8}, {-1.0F, -1.0F, -1.0F, 3.0F, 3.0F, 3.0F},
    {1.0F, 1.0F, 1.0F}, 2, segmentation_class_names, segmentation_class_mapping, {"z", "z-trans"},
    {2, 2}, {8, 16, 32}, {4, 4, 4}, palette, {}, "xyzi", false, "partial", {0, 0});
}

TEST(PTv3ConfigTest, AcceptsTheInstanceMemoryOfTheDetectionHead)
{
  const auto config = makeDetectionConfig();
  EXPECT_EQ(config.num_anchor_, 8U);
  EXPECT_EQ(config.num_temp_instances_, 4U);
  EXPECT_EQ(config.embed_dims_, 16U);
  EXPECT_EQ(config.circle_nms_class_groups_, (std::vector<std::int64_t>{-1, 0}));
}

TEST(PTv3ConfigTest, RejectsAnInstanceMemoryOutsideTheQueries)
{
  const auto rejects = [](const auto & modify) {
    DetectionHeadParams head;
    modify(head);
    EXPECT_THROW(makeDetectionConfig(head), std::runtime_error);
  };
  rejects([](DetectionHeadParams & head) { head.num_temp_instances = 0; });
  rejects([](DetectionHeadParams & head) { head.num_temp_instances = head.num_anchor; });
  rejects([](DetectionHeadParams & head) { head.embed_dims = 0; });
  rejects([](DetectionHeadParams & head) { head.default_time_interval = 0.0F; });
  rejects([](DetectionHeadParams & head) { head.max_time_interval = 0.05F; });
  rejects([](DetectionHeadParams & head) { head.track_birth_score = 1.5F; });
}

TEST(PTv3ConfigTest, RejectsMalformedCircleNmsGroups)
{
  const auto rejects = [](const auto & modify) {
    DetectionHeadParams head;
    modify(head);
    EXPECT_THROW(makeDetectionConfig(head), std::runtime_error);
  };
  rejects([](DetectionHeadParams & head) { head.circle_nms_class_groups = {0}; });
  rejects([](DetectionHeadParams & head) { head.circle_nms_class_groups = {-1, 1}; });
  rejects([](DetectionHeadParams & head) { head.circle_nms_class_groups = {-2, 0}; });
  rejects([](DetectionHeadParams & head) { head.circle_nms_radii = {0.0F}; });
  rejects([](DetectionHeadParams & head) { head.circle_nms_radii = {0.5F, 1.0F}; });
}

TEST(PTv3ConfigTest, RejectsInvalidDetectionThresholdTables)
{
  const auto rejects = [](const auto & modify) {
    DetectionHeadParams head;
    modify(head);
    EXPECT_THROW(makeDetectionConfig(head), std::runtime_error);
  };
  rejects([](DetectionHeadParams & head) { head.distance_bin_upper_limits = {20.0F, 10.0F}; });
  rejects([](DetectionHeadParams & head) {
    head.distance_bin_upper_limits = {10.0F};
    head.detection_score_thresholds = {0.1F};
  });
  rejects(
    [](DetectionHeadParams & head) { head.detection_score_thresholds = {0.1F, 0.2F, 0.3F, 1.4F}; });
}

TEST(PTv3ConfigTest, BuildsClassificationLutInClassNameOrder)
{
  using autoware::point_types::PointCloudClassification;

  // Keep the mapping order different from class_names to verify that the LUT follows class_names.
  const auto config = makeSegmentationConfig(
    {"car", "traffic_cone", "drivable_flat"},
    {{"drivable_flat", "FLAT_SURFACE"}, {"car", "CAR"}, {"traffic_cone", "HAZARD"}});

  ASSERT_EQ(config.class_id_to_classification_.size(), 3U);
  EXPECT_EQ(
    config.class_id_to_classification_[0],
    static_cast<std::uint8_t>(PointCloudClassification::CAR));
  EXPECT_EQ(
    config.class_id_to_classification_[1],
    static_cast<std::uint8_t>(PointCloudClassification::HAZARD));
  EXPECT_EQ(
    config.class_id_to_classification_[2],
    static_cast<std::uint8_t>(PointCloudClassification::FLAT_SURFACE));
}

TEST(PTv3ConfigTest, HonorsClassificationMappingOverride)
{
  using autoware::point_types::PointCloudClassification;

  // traffic_cone used to be hard-coded to HAZARD; the parameter must be able to override it.
  const auto config = makeSegmentationConfig({"traffic_cone"}, {{"traffic_cone", "STRUCTURE"}});

  ASSERT_EQ(config.class_id_to_classification_.size(), 1U);
  EXPECT_EQ(
    config.class_id_to_classification_[0],
    static_cast<std::uint8_t>(PointCloudClassification::STRUCTURE));
}

TEST(PTv3ConfigTest, AcceptsClassificationNamesRegardlessOfCase)
{
  using autoware::point_types::PointCloudClassification;

  const auto config = makeSegmentationConfig(
    {"drivable_flat", "noise"}, {{"drivable_flat", "flat_surface"}, {"noise", "Noise"}});

  ASSERT_EQ(config.class_id_to_classification_.size(), 2U);
  EXPECT_EQ(
    config.class_id_to_classification_[0],
    static_cast<std::uint8_t>(PointCloudClassification::FLAT_SURFACE));
  EXPECT_EQ(
    config.class_id_to_classification_[1],
    static_cast<std::uint8_t>(PointCloudClassification::NOISE));
}

TEST(PTv3ConfigTest, RejectsUnknownClassificationName)
{
  EXPECT_THROW(makeSegmentationConfig({"car"}, {{"car", "UNKNOWN"}}), std::invalid_argument);
}

TEST(PTv3ConfigTest, RejectsClassNameMissingFromMapping)
{
  EXPECT_THROW(makeSegmentationConfig({"car", "truck"}, {{"car", "CAR"}}), std::runtime_error);
}

TEST(PTv3ConfigTest, IgnoresMappingEntriesForClassesTheModelDoesNotOutput)
{
  using autoware::point_types::PointCloudClassification;

  // The mapping may cover more classes than class_names, e.g. one param file shared across model
  // variants that output different class subsets.
  const auto config = makeSegmentationConfig(
    {"car"}, {{"car", "CAR"}, {"truck", "TRUCK"}, {"vegetation", "VEGETATION"}});

  ASSERT_EQ(config.class_id_to_classification_.size(), 1U);
  EXPECT_EQ(
    config.class_id_to_classification_[0],
    static_cast<std::uint8_t>(PointCloudClassification::CAR));
}

// Cells count from the lower range border, so [0.5, 16.5) holds 16 unit cells like [0, 16).
TEST(PTv3ConfigTest, SerializationDepthCountsCellsFromTheLowerBorder)
{
  const auto aligned = makeDetectionConfig();
  EXPECT_EQ(aligned.serialization_depth_, 4);

  const auto unaligned = makeDetectionConfig({0.5F, 0.5F, 0.5F, 16.5F, 16.5F, 4.5F});
  EXPECT_EQ(unaligned.grid_x_size_, 16);
  EXPECT_EQ(unaligned.grid_z_size_, 4);
  EXPECT_EQ(unaligned.serialization_depth_, 4);
}

// A range whose height is no multiple of the voxel size from the origin still holds
// round(extent / size) cells: 7.68 m of 0.12 m voxels from -2.3 m are 64 cells.
TEST(PTv3ConfigTest, GridHoldsTheTrainingVoxelizerCells)
{
  const auto config = makeDetectionConfig(
    {-122.88F, -122.88F, -2.3F, 122.88F, 122.88F, 5.38F}, {0.12F, 0.12F, 0.12F}, {1, 4, 8});
  EXPECT_EQ(config.grid_x_size_, 2048);
  EXPECT_EQ(config.grid_y_size_, 2048);
  EXPECT_EQ(config.grid_z_size_, 64);
}

// The per-stage voxel bound follows the cells: 16 x 16 x 4 at stage 0, halved per stride-2 stage.
TEST(PTv3ConfigTest, StageVoxelCapacityFollowsTheGridCells)
{
  const auto config = makeDetectionConfig(
    {0.5F, 0.5F, 0.5F, 16.5F, 16.5F, 4.5F}, {1.0F, 1.0F, 1.0F}, {1, 1024, 4096},
    {4096, 4096, 4096, 4096});
  EXPECT_EQ(config.stage_voxel_capacity(0), 16 * 16 * 4);
  EXPECT_EQ(config.stage_voxel_capacity(1), 8 * 8 * 2);
  EXPECT_EQ(config.stage_voxel_capacity(4), 1 * 1 * 1);
}

// 16 x 16 x 4 grid: cell counts 1024, 128, 16, 4, 1 per level.
TEST(PTv3ConfigTest, StageProfilesFollowThePooledMaximums)
{
  using Counts = std::array<std::int64_t, 3>;
  const auto config = makeDetectionConfig(
    {0.0F, 0.0F, 0.0F, 16.0F, 16.0F, 4.0F}, {1.0F, 1.0F, 1.0F}, {16, 256, 1024}, {64, 8, 4, 4});
  EXPECT_EQ(config.stage_profile_counts(0), (Counts{16, 256, 1024}));
  EXPECT_EQ(config.stage_profile_counts(1), (Counts{1, 64, 64}));
  EXPECT_EQ(config.stage_profile_counts(2), (Counts{1, 8, 8}));
  EXPECT_EQ(config.stage_profile_counts(3), (Counts{1, 4, 4}));
  EXPECT_EQ(config.stage_profile_counts(4), (Counts{1, 1, 1}));
}

TEST(PTv3ConfigTest, StageProfileCountsClampToTheGridCapacity)
{
  const auto config = makeDetectionConfig(
    {0.0F, 0.0F, 0.0F, 16.0F, 16.0F, 4.0F}, {1.0F, 1.0F, 1.0F}, {2048, 3072, 4096},
    {4096, 4096, 4096, 4096});
  EXPECT_EQ(config.stage_profile_counts(0), (std::array<std::int64_t, 3>{1024, 1024, 1024}));
}

TEST(PTv3ConfigTest, RejectsMalformedPooledVoxelsNumMax)
{
  const auto rejects = [](const std::vector<std::int64_t> & pooled_voxels_num_max) {
    EXPECT_THROW(
      makeDetectionConfig(
        {0.0F, 0.0F, 0.0F, 16.0F, 16.0F, 4.0F}, {1.0F, 1.0F, 1.0F}, {1, 4, 8},
        pooled_voxels_num_max),
      std::runtime_error);
  };
  rejects({8, 8, 8});        // one entry per pooling stage (four here)
  rejects({8, 8, 8, 8, 8});  //
  rejects({8, 8, 0, 8});     // positive
  rejects({9, 8, 8, 8});     // at most the input level's maximum (8)
  rejects({8, 4, 8, 8});     // non-increasing
}

// Borders that are voxel-aligned in decimal but not exactly representable in binary (neither 102.4
// nor 0.1 is a binary float) must not gain a spurious extra coordinate from rounding.
TEST(PTv3ConfigTest, SerializationDepthStaysExactForBase10AlignedRanges)
{
  const auto config =
    makeDetectionConfig({-102.4F, -102.4F, -0.4F, 102.4F, 102.4F, 0.4F}, {0.1F, 0.1F, 0.1F});
  EXPECT_EQ(config.serialization_depth_, 11);
}

}  // namespace test
}  // namespace autoware::ptv3
