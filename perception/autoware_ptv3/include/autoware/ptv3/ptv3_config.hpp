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

#ifndef AUTOWARE__PTV3__PTV3_CONFIG_HPP_
#define AUTOWARE__PTV3__PTV3_CONFIG_HPP_

#include <autoware/point_types/types.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::ptv3
{

enum class SourceReconstruction {
  NONE,
  PARTIAL,
  FULL,
};

class PTv3Config
{
public:
  PTv3Config(
    const bool use_seg3d_head, const bool use_det3d_head, const std::string & plugins_path,
    const std::int64_t cloud_capacity, const std::vector<std::int64_t> & voxels_num,
    const std::vector<std::int64_t> & pooled_voxels_num_max,
    const std::vector<float> & point_cloud_range, const std::vector<float> & voxel_size,
    const std::int64_t max_points_per_voxel,
    const std::vector<std::string> & segmentation_class_names = {},
    const std::unordered_map<std::string, std::string> & segmentation_class_mapping = {},
    const std::vector<std::string> & serialization_orders = {},
    const std::vector<std::int64_t> & pooling_strides = {},
    const std::vector<std::int64_t> & enc_channels = {},
    const std::vector<std::int64_t> & patch_sizes = {},
    const std::vector<std::int64_t> & palette = {},
    const std::vector<std::string> & filter_classes = {},
    const std::string & filter_output_format = {}, const bool filter_apply_to_segmentation = {},
    const std::string & source_reconstruction = {},
    const std::vector<std::int64_t> & dec_depths = {},
    const std::vector<std::string> & detection_class_names = {},
    const std::vector<float> & distance_bin_upper_limits = {},
    const std::vector<float> & detection_score_thresholds = {}, const std::int64_t num_anchor = {},
    const std::int64_t num_temp_instances = {}, const std::int64_t embed_dims = {},
    const float default_time_interval = {}, const float max_time_interval = {},
    const float track_birth_score = {},
    const std::vector<std::int64_t> & circle_nms_class_groups = {},
    const std::vector<float> & circle_nms_radii = {})
  : use_seg3d_head_(use_seg3d_head), use_det3d_head_(use_det3d_head)
  {
    plugins_path_ = plugins_path;

    cloud_capacity_ = cloud_capacity;

    if (!use_seg3d_head_ && !use_det3d_head_) {
      throw std::runtime_error(
        "At least one of segmentation3d.use_head or detection3d.use_head must be true.");
    }

    if (voxels_num.size() == 3) {
      min_num_voxels_ = voxels_num[0];
      max_num_voxels_ = voxels_num[2];

      voxels_num_[0] = voxels_num[0];
      voxels_num_[1] = voxels_num[1];
      voxels_num_[2] = voxels_num[2];
    }
    if (point_cloud_range.size() == 6) {
      min_x_range_ = point_cloud_range[0];
      min_y_range_ = point_cloud_range[1];
      min_z_range_ = point_cloud_range[2];
      max_x_range_ = point_cloud_range[3];
      max_y_range_ = point_cloud_range[4];
      max_z_range_ = point_cloud_range[5];
    }
    if (voxel_size.size() == 3) {
      voxel_x_size_ = voxel_size[0];
      voxel_y_size_ = voxel_size[1];
      voxel_z_size_ = voxel_size[2];
    }
    if (max_points_per_voxel <= 0) {
      throw std::runtime_error("max_points_per_voxel must be positive.");
    }
    max_points_per_voxel_ = max_points_per_voxel;

    // Cells per axis of the training voxelizer grid: cells count from the lower range border and
    // the range holds round(extent / size) of them, the crop drops the rows past the last one.
    const auto grid_cells = [](const float min_range, const float max_range, const float size) {
      return static_cast<std::int64_t>(std::lround((max_range - min_range) / size));
    };
    grid_x_size_ = grid_cells(min_x_range_, max_x_range_, voxel_x_size_);
    grid_y_size_ = grid_cells(min_y_range_, max_y_range_, voxel_y_size_);
    grid_z_size_ = grid_cells(min_z_range_, max_z_range_, voxel_z_size_);
    const auto max_grid_size = std::max({grid_x_size_, grid_y_size_, grid_z_size_});
    serialization_depth_ =
      static_cast<std::int32_t>(std::ceil(std::log2(static_cast<float>(max_grid_size))));
    auto max_voxels_depth =
      static_cast<std::int32_t>(std::ceil(std::log2(static_cast<float>(max_num_voxels_))));
    if (serialization_depth_ * 3 + max_voxels_depth >= 64) {
      throw std::runtime_error("Serialization depth is too large");
    }

    serialization_orders_ = validate_serialization_orders(serialization_orders);
    pooling_strides_ = validate_pooling_strides(pooling_strides);
    pooled_voxels_num_max_ = validate_pooled_voxels_num_max(
      pooled_voxels_num_max, max_num_voxels_, pooling_strides_.size());
    enc_channels_ = validate_enc_channels(enc_channels, pooling_strides_.size() + 1);
    patch_sizes_ = validate_patch_sizes(patch_sizes, pooling_strides_.size() + 1);

    if (use_seg3d_head_) {
      segmentation_class_names_ = segmentation_class_names;
      colors_rgb_ = make_palette(segmentation_class_names_, palette);
      class_id_to_classification_ =
        make_class_id_to_classification(segmentation_class_names_, segmentation_class_mapping);
      for (auto & class_name : segmentation_class_names_) {
        std::transform(
          class_name.begin(), class_name.end(), class_name.begin(),
          [](unsigned char c) { return std::tolower(c); });
      }
      filter_class_indices_ = make_filter_class_indices(segmentation_class_names_, filter_classes);
      filter_output_format_ = filter_output_format;
      filter_apply_to_segmentation_ = filter_apply_to_segmentation;
      source_reconstruction_ = parse_source_reconstruction(source_reconstruction);

      // dec_depths drives the seg-head engine input set: block stages consume their
      if (dec_depths.size() != pooling_strides_.size()) {
        throw std::runtime_error(
          "dec_depths must contain one entry per decoder stage (pooling_strides size = " +
          std::to_string(pooling_strides_.size()) + "), got " + std::to_string(dec_depths.size()) +
          ".");
      }
      for (std::size_t stage = 0; stage < dec_depths.size(); ++stage) {
        if (dec_depths[stage] < 0) {
          throw std::runtime_error("dec_depths entries must be non-negative.");
        }
      }
      dec_depths_ = dec_depths;
    }

    if (use_det3d_head_) {
      if (detection_class_names.empty()) {
        throw std::runtime_error("detection_class_names must not be empty when use_det3d_head.");
      }
      if (distance_bin_upper_limits.empty()) {
        throw std::runtime_error("distance_bin_upper_limits must not be empty.");
      }
      if (!std::is_sorted(distance_bin_upper_limits.begin(), distance_bin_upper_limits.end())) {
        throw std::runtime_error("distance_bin_upper_limits must be sorted.");
      }
      if (distance_bin_upper_limits.front() <= 0.0F) {
        throw std::runtime_error("distance_bin_upper_limits values must be positive.");
      }
      if (
        detection_score_thresholds.size() !=
        distance_bin_upper_limits.size() * detection_class_names.size()) {
        throw std::runtime_error(
          "detection_score_thresholds size must match distance bins x detection classes.");
      }
      if (!std::all_of(
            detection_score_thresholds.begin(), detection_score_thresholds.end(),
            [](float value) { return value >= 0.0F && value <= 1.0F; })) {
        throw std::runtime_error("detection_score_thresholds values must be between 0 and 1.");
      }
      if (num_anchor < 1 || embed_dims < 1) {
        throw std::runtime_error("num_anchor and embed_dims must be positive.");
      }
      if (num_temp_instances < 1 || num_temp_instances >= num_anchor) {
        throw std::runtime_error("num_temp_instances must be in [1, num_anchor).");
      }
      if (default_time_interval <= 0.0F || max_time_interval < default_time_interval) {
        throw std::runtime_error("The time intervals must satisfy 0 < default <= max.");
      }
      if (track_birth_score < 0.0F || track_birth_score > 1.0F) {
        throw std::runtime_error("track_birth_score must be between 0 and 1.");
      }

      detection_class_names_ = detection_class_names;
      distance_bin_upper_limits_ = distance_bin_upper_limits;
      detection_score_thresholds_ = detection_score_thresholds;
      num_anchor_ = static_cast<std::size_t>(num_anchor);
      num_temp_instances_ = static_cast<std::size_t>(num_temp_instances);
      embed_dims_ = static_cast<std::size_t>(embed_dims);
      default_time_interval_ = default_time_interval;
      max_time_interval_ = max_time_interval;
      track_birth_score_ = track_birth_score;
      circle_nms_class_groups_ = validate_circle_nms_groups(
        circle_nms_class_groups, circle_nms_radii, detection_class_names_.size());
      circle_nms_radii_ = circle_nms_radii;
    }
  }

  static SourceReconstruction parse_source_reconstruction(const std::string & value)
  {
    if (value == "none") {
      return SourceReconstruction::NONE;
    }
    if (value == "partial") {
      return SourceReconstruction::PARTIAL;
    }
    if (value == "full") {
      return SourceReconstruction::FULL;
    }
    throw std::runtime_error("source_reconstruction must be one of: 'none', 'partial', or 'full'.");
  }

  static std::vector<std::uint32_t> make_filter_class_indices(
    const std::vector<std::string> & class_names, const std::vector<std::string> & filter_classes)
  {
    std::vector<std::uint32_t> indices;
    for (const auto & filter_class : filter_classes) {
      auto it = std::find(class_names.begin(), class_names.end(), filter_class);
      if (it == class_names.end()) {
        throw std::runtime_error("Filter class '" + filter_class + "' not found in class names.");
      }
      indices.push_back(static_cast<std::uint32_t>(std::distance(class_names.begin(), it)));
    }
    return indices;
  }

  /**
   * @brief Build the lookup table from model class id (index into class_names) to
   * PointCloudClassification.
   * @details The model output label index is determined by the class_names order, so the table is
   * built by looking each class name up in class_mapping. Entries in class_mapping whose key is not
   * in class_names are ignored, so the map may cover more classes than the loaded model outputs.
   * @param class_names Segmentation class names, indexed by model output label.
   * @param class_mapping Class name to PointCloudClassification name.
   * @return Lookup table with one entry per class name.
   * @throws std::runtime_error If a class name has no entry in class_mapping.
   * @throws std::invalid_argument If a mapped value is not a PointCloudClassification name.
   */
  static std::vector<std::uint8_t> make_class_id_to_classification(
    const std::vector<std::string> & class_names,
    const std::unordered_map<std::string, std::string> & class_mapping)
  {
    std::vector<std::uint8_t> lut;
    lut.reserve(class_names.size());
    for (const auto & class_name : class_names) {
      const auto it = class_mapping.find(class_name);
      if (it == class_mapping.end()) {
        throw std::runtime_error("class_mapping has no entry for class name '" + class_name + "'.");
      }
      lut.push_back(
        static_cast<std::uint8_t>(autoware::point_types::to_pointcloud_classification(it->second)));
    }
    return lut;
  }

  static std::vector<float> make_palette(
    const std::vector<std::string> & class_names, const std::vector<std::int64_t> & palette)
  {
    if (palette.size() % 3 != 0) {
      throw std::runtime_error("Palette size must be a multiple of 3.");
    }
    if (palette.size() != class_names.size() * 3) {
      throw std::runtime_error("Palette size does not match class names size.");
    }

    std::vector<float> colors;
    colors.reserve(class_names.size());
    for (size_t i = 0; i < palette.size(); i += 3) {
      const auto r = palette[i];
      const auto g = palette[i + 1];
      const auto b = palette[i + 2];
      if (r < 0 || r > 255 || g < 0 || g > 255 || b < 0 || b > 255) {
        throw std::runtime_error("Color values must be within 0-255 range.");
      }

      const std::uint32_t rgb = (static_cast<std::uint32_t>(r) << 16u) |
                                (static_cast<std::uint32_t>(g) << 8u) |
                                static_cast<std::uint32_t>(b);
      float rgb_float = 0.0f;
      memcpy(&rgb_float, &rgb, sizeof(rgb_float));
      colors.push_back(rgb_float);
    }
    return colors;
  }

  // Every class names its circle NMS group, -1 for none, and every group has a positive radius.
  static std::vector<std::int64_t> validate_circle_nms_groups(
    const std::vector<std::int64_t> & class_groups, const std::vector<float> & radii,
    const std::size_t num_classes)
  {
    if (class_groups.size() != num_classes) {
      throw std::runtime_error("circle_nms class_groups must have one entry per detection class.");
    }
    const auto num_groups = static_cast<std::int64_t>(radii.size());
    for (const auto group : class_groups) {
      if (group < -1 || group >= num_groups) {
        throw std::runtime_error("circle_nms class_groups entries must be -1 or a radii index.");
      }
    }
    for (std::int64_t group = 0; group < num_groups; ++group) {
      if (radii[group] <= 0.0F) {
        throw std::runtime_error("circle_nms radii must be positive.");
      }
      if (std::find(class_groups.begin(), class_groups.end(), group) == class_groups.end()) {
        throw std::runtime_error("Every circle_nms group must hold at least one class.");
      }
    }
    return class_groups;
  }

  static std::vector<std::string> validate_serialization_orders(
    const std::vector<std::string> & serialization_orders)
  {
    if (serialization_orders.empty()) {
      throw std::runtime_error("serialization_orders must not be empty.");
    }
    if (
      serialization_orders.size() != 2 || serialization_orders[0] != "z" ||
      serialization_orders[1] != "z-trans") {
      throw std::runtime_error(
        "The current PTv3 preprocessing path supports serialization_orders: ['z', 'z-trans'].");
    }
    return serialization_orders;
  }

  static std::vector<std::int64_t> validate_pooling_strides(
    const std::vector<std::int64_t> & pooling_strides)
  {
    if (pooling_strides.empty()) {
      throw std::runtime_error("pooling_strides must not be empty.");
    }
    for (const auto stride : pooling_strides) {
      if (stride < 1 || (stride & (stride - 1)) != 0) {
        throw std::runtime_error("Each pooling stride must be a positive power of two.");
      }
    }
    return pooling_strides;
  }

  static std::vector<std::int64_t> validate_enc_channels(
    const std::vector<std::int64_t> & enc_channels, const std::size_t expected_size)
  {
    if (enc_channels.size() != expected_size) {
      throw std::runtime_error(
        "enc_channels must contain one entry per encoder stage (pooling_strides size + 1 = " +
        std::to_string(expected_size) + "), got " + std::to_string(enc_channels.size()) + ".");
    }
    for (const auto channels : enc_channels) {
      if (channels < 1) {
        throw std::runtime_error("Each enc_channels entry must be positive.");
      }
    }
    return enc_channels;
  }

  static std::vector<std::int64_t> validate_patch_sizes(
    const std::vector<std::int64_t> & patch_sizes, const std::size_t expected_size)
  {
    if (patch_sizes.size() != expected_size) {
      throw std::runtime_error(
        "patch_sizes must contain one entry per encoder stage (pooling_strides size + 1 = " +
        std::to_string(expected_size) + "), got " + std::to_string(patch_sizes.size()) + ".");
    }
    for (const auto patch_size : patch_sizes) {
      if (patch_size < 1) {
        throw std::runtime_error("Each patch_sizes entry must be positive.");
      }
    }
    return patch_sizes;
  }

  // Extent of a stage's patch_order input for `count` voxels: rounded up to whole windows.
  [[nodiscard]] std::int64_t padded_voxel_count(
    const std::int64_t count, const std::size_t stage_index) const
  {
    const auto patch_size = patch_sizes_[stage_index];
    return (count + patch_size - 1) / patch_size * patch_size;
  }

  // One entry per pooled stage, positive and non-increasing from the input level's maximum.
  static std::vector<std::int64_t> validate_pooled_voxels_num_max(
    const std::vector<std::int64_t> & pooled_voxels_num_max, const std::int64_t max_num_voxels,
    const std::size_t num_pooling_stages)
  {
    if (pooled_voxels_num_max.size() != num_pooling_stages) {
      throw std::runtime_error(
        "pooled_voxels_num_max must contain one entry per pooling stage (pooling_strides size = " +
        std::to_string(num_pooling_stages) + "), got " +
        std::to_string(pooled_voxels_num_max.size()) + ".");
    }
    auto previous = max_num_voxels;
    for (const auto max : pooled_voxels_num_max) {
      if (max < 1 || max > previous) {
        throw std::runtime_error(
          "Each pooled_voxels_num_max entry must be positive and at most the previous stage's "
          "maximum (" +
          std::to_string(previous) + "), got " + std::to_string(max) + ".");
      }
      previous = max;
    }
    return pooled_voxels_num_max;
  }

  // Voxel-count bound of one encoder stage: the smaller of its configured maximum (voxels_num[2],
  // then pooled_voxels_num_max) and its grid cell count. Sizes buffers and profiles.
  [[nodiscard]] std::int64_t stage_voxel_capacity(const std::size_t stage_index) const
  {
    std::int64_t cumulative_depth = 0;
    for (std::size_t stage = 0; stage < stage_index; ++stage) {
      for (auto value = pooling_strides_[stage]; value > 1; value >>= 1) {
        ++cumulative_depth;
      }
    }
    const auto ceil_shift = [cumulative_depth](const std::int64_t size) {
      return (size + (std::int64_t{1} << cumulative_depth) - 1) >> cumulative_depth;
    };
    const auto grid_cells =
      ceil_shift(grid_x_size_) * ceil_shift(grid_y_size_) * ceil_shift(grid_z_size_);
    const auto configured_max =
      stage_index == 0 ? max_num_voxels_ : pooled_voxels_num_max_.at(stage_index - 1);
    return std::min(configured_max, grid_cells);
  }

  // [min, opt, max] TensorRT profile of one encoder stage: voxels_num for the input level; pooled
  // levels use min 1, opt halved per stage, and the stage capacity as max.
  [[nodiscard]] std::array<std::int64_t, 3> stage_profile_counts(
    const std::size_t stage_index) const
  {
    const std::int64_t max_count = stage_voxel_capacity(stage_index);
    const std::int64_t min_count =
      std::min(stage_index == 0 ? min_num_voxels_ : std::int64_t{1}, max_count);
    const std::int64_t opt_count = std::clamp(voxels_num_[1] >> stage_index, min_count, max_count);
    return {min_count, opt_count, max_count};
  }

  // stage_profile_counts padded to whole attention windows: the profile of a level's patch_order.
  [[nodiscard]] std::array<std::int64_t, 3> padded_stage_profile_counts(
    const std::size_t stage_index) const
  {
    const auto counts = stage_profile_counts(stage_index);
    return {
      padded_voxel_count(counts[0], stage_index), padded_voxel_count(counts[1], stage_index),
      padded_voxel_count(counts[2], stage_index)};
  }

  // CUDA parameters
  const std::uint32_t threads_per_block_{256};  // threads number for a block

  // TensorRT parameters
  std::string plugins_path_;

  // Head selection
  bool use_seg3d_head_;
  bool use_det3d_head_;

  // Preprocess parameters
  std::int32_t serialization_depth_{};

  ///// NETWORK PARAMETERS /////

  // Encoder
  std::vector<std::string> segmentation_class_names_;
  std::vector<std::string> serialization_orders_;
  std::vector<std::int64_t> pooling_strides_;
  std::vector<std::int64_t> enc_channels_;  // per encoder stage, finest to deepest
  std::vector<std::int64_t> patch_sizes_;   // attention window per encoder stage, finest to deepest

  // Segmentation head
  std::vector<std::int64_t> dec_depths_;  // decoder block counts per stage
  std::vector<float> colors_rgb_;
  // class id (index into segmentation_class_names_) -> PointCloudClassification
  std::vector<std::uint8_t> class_id_to_classification_;
  std::vector<std::uint32_t> filter_class_indices_;
  std::string filter_output_format_;
  bool filter_apply_to_segmentation_{};
  SourceReconstruction source_reconstruction_{SourceReconstruction::NONE};

  // Detection head
  std::vector<std::string> detection_class_names_;
  std::vector<float> distance_bin_upper_limits_;
  std::vector<float> detection_score_thresholds_;      // [distance bin][class]
  std::size_t num_anchor_{};                           // instances decoded per frame
  std::size_t num_temp_instances_{};                   // instances carried to the next frame
  std::size_t embed_dims_{};                           // instance feature width
  float default_time_interval_{};                      // frame gap fed when a stream starts
  float max_time_interval_{};                          // largest frame gap that keeps the memory
  float track_birth_score_{};                          // score from which a query starts a track
  std::vector<std::int64_t> circle_nms_class_groups_;  // per class, -1 when not suppressed
  std::vector<float> circle_nms_radii_;                // per group, in meters

  // Common network parameters
  std::int64_t cloud_capacity_{};  // capacity of one lidar frame
  std::int64_t min_num_voxels_{};
  std::int64_t max_num_voxels_{};
  std::int64_t max_points_per_voxel_{};  // padded voxel slots, matches the training voxelizer
  const std::int64_t num_point_feature_size_{4};  // x, y, z, intensity

  // Pointcloud range in meters
  float min_x_range_{};
  float max_x_range_{};
  float min_y_range_{};
  float max_y_range_{};
  float min_z_range_{};
  float max_z_range_{};

  // Voxel size in meters
  float voxel_x_size_{};
  float voxel_y_size_{};
  float voxel_z_size_{};

  // Grid size (cells the device grid mapping can emit, see the constructor)
  std::int64_t grid_x_size_{};
  std::int64_t grid_y_size_{};
  std::int64_t grid_z_size_{};

  ///// RUNTIME DIMENSIONS /////
  std::array<std::int64_t, 3> voxels_num_{};
  std::vector<std::int64_t> pooled_voxels_num_max_;  // one per pooling stage
};

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__PTV3_CONFIG_HPP_
