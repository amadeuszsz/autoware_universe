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

#include "autoware/ptv3/ptv3_trt.hpp"

#include "autoware/ptv3/preprocess/point_feature_kernel.hpp"
#include "autoware/ptv3/preprocess/point_type.hpp"
#include "autoware/ptv3/preprocess/preprocess_kernel.hpp"
#include "autoware/ptv3/ptv3_config.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/cuda_utils/cuda_utils.hpp>
#include <autoware/point_types/memory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::ptv3
{
namespace
{

std::int64_t poolingDepth(const std::int64_t stride)
{
  std::int64_t depth = 0;
  for (auto value = stride; value > 1; value >>= 1) {
    ++depth;
  }
  return depth;
}

std::string stageFeatureName(const std::size_t stage_index)
{
  return "point_feat_" + std::to_string(stage_index);
}

std::string poolingClusterName(const std::size_t stage_index)
{
  return "pooling_cluster_" + std::to_string(stage_index);
}

std::string stageGridCoordName(const std::size_t stage_index)
{
  return "point_grid_coord_" + std::to_string(stage_index);
}

CloudFormat detectCloudFormat(const cuda_blackboard::CudaPointCloud2 & cloud)
{
  const auto & fields = cloud.fields;
  const auto num_fields = fields.size();

  if (num_fields == 10 && point_types::is_data_layout_compatible_with_point_xyzircaedt(fields)) {
    return CloudFormat::XYZIRCAEDT;
  }
  if (num_fields == 9 && point_types::is_data_layout_compatible_with_point_xyziradrt(fields)) {
    return CloudFormat::XYZIRADRT;
  }
  if (num_fields == 6 && point_types::is_data_layout_compatible_with_point_xyzirc(fields)) {
    return CloudFormat::XYZIRC;
  }
  if (num_fields == 4 && point_types::is_data_layout_compatible_with_point_xyzi(fields)) {
    return CloudFormat::XYZI;
  }

  return CloudFormat::UNKNOWN;
}

}  // namespace

PTv3TRT::PTv3TRT(
  const tensorrt_common::TrtCommonConfig & encoder_trt_config,
  const std::optional<tensorrt_common::TrtCommonConfig> & seg3d_head_trt_config,
  const std::optional<tensorrt_common::TrtCommonConfig> & det3d_head_trt_config,
  const PTv3Config & config)
: config_(config)
{
  stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
  stop_watch_ptr_->tic("processing/inner");

  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));

  createPointFields();
  initPtr();
  initEncoderTrt(encoder_trt_config);
  if (config_.use_seg3d_head_) {
    if (!seg3d_head_trt_config.has_value()) {
      throw std::runtime_error("seg3d_head_trt_config is required when segmentation3d.use_head.");
    }
    initSeg3dHeadTrt(*seg3d_head_trt_config);
  }
  if (config_.use_det3d_head_) {
    if (!det3d_head_trt_config.has_value()) {
      throw std::runtime_error("det3d_head_trt_config is required when detection3d.use_head.");
    }
    initDetection3DHeadTrt(*det3d_head_trt_config);
  }
}

void PTv3TRT::setPublishSegmentedPointcloud(
  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func)
{
  publish_segmented_pointcloud_ = std::move(func);
}

void PTv3TRT::setPublishVisualizationPointcloud(
  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func)
{
  publish_visualization_pointcloud_ = std::move(func);
}

void PTv3TRT::setPublishFilteredPointcloud(
  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func)
{
  publish_filtered_pointcloud_ = std::move(func);
}

void PTv3TRT::allocateSegOutputMessages()
{
  if (!config_.use_seg3d_head_) {
    return;
  }

  const auto output_capacity = config_.source_reconstruction_ != SourceReconstruction::NONE
                                 ? config_.cloud_capacity_
                                 : config_.max_num_voxels_;
  if (segmented_points_msg_ptr_ == nullptr) {
    segmented_points_msg_ptr_ = std::make_unique<cuda_blackboard::CudaPointCloud2>();
    segmented_points_msg_ptr_->height = 1;
    segmented_points_msg_ptr_->width = output_capacity;
    segmented_points_msg_ptr_->fields = segmented_pointcloud_fields_;
    segmented_points_msg_ptr_->is_bigendian = false;
    segmented_points_msg_ptr_->is_dense = true;
    segmented_points_msg_ptr_->point_step =
      static_cast<std::uint32_t>(sizeof(point_types::PointXYZCPE));
    segmented_points_msg_ptr_->data = cuda_blackboard::make_unique<std::uint8_t[]>(
      output_capacity * segmented_points_msg_ptr_->point_step);
  }

  if (visualization_points_msg_ptr_ == nullptr) {
    visualization_points_msg_ptr_ = std::make_unique<cuda_blackboard::CudaPointCloud2>();
    visualization_points_msg_ptr_->height = 1;
    visualization_points_msg_ptr_->width = output_capacity;
    visualization_points_msg_ptr_->fields = visualization_pointcloud_fields_;
    visualization_points_msg_ptr_->is_bigendian = false;
    visualization_points_msg_ptr_->is_dense = true;
    visualization_points_msg_ptr_->point_step =
      static_cast<std::uint32_t>(visualization_pointcloud_fields_.size() * sizeof(float));
    visualization_points_msg_ptr_->data = cuda_blackboard::make_unique<std::uint8_t[]>(
      output_capacity * visualization_points_msg_ptr_->point_step);
  }

  if (filtered_points_msg_ptr_ == nullptr && filtered_output_format_ != CloudFormat::UNKNOWN) {
    filtered_points_msg_ptr_ = std::make_unique<cuda_blackboard::CudaPointCloud2>();
    filtered_points_msg_ptr_->height = 1;
    filtered_points_msg_ptr_->width = output_capacity;
    filtered_points_msg_ptr_->fields = filtered_pointcloud_fields_;
    filtered_points_msg_ptr_->is_bigendian = false;
    filtered_points_msg_ptr_->is_dense = true;
    filtered_points_msg_ptr_->point_step =
      static_cast<std::uint32_t>(get_point_step(filtered_output_format_));
    filtered_points_msg_ptr_->data = cuda_blackboard::make_unique<std::uint8_t[]>(
      output_capacity * filtered_points_msg_ptr_->point_step);
  }
}

PTv3TRT::~PTv3TRT()
{
  if (stream_) {
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
    CHECK_CUDA_ERROR(cudaStreamDestroy(stream_));
  }
}

void PTv3TRT::initPtr()
{
  point_features_d_ = autoware::cuda_utils::make_unique<float[]>(
    config_.cloud_capacity_ * config_.num_point_feature_size_);
  voxels_d_ = autoware::cuda_utils::make_unique<float[]>(
    config_.max_num_voxels_ * config_.max_points_per_voxel_ * config_.num_point_feature_size_);
  num_points_per_voxel_d_ =
    autoware::cuda_utils::make_unique<std::int32_t[]>(config_.max_num_voxels_);
  grid_coord_d_ = autoware::cuda_utils::make_unique<std::int32_t[]>(config_.max_num_voxels_ * 3);
  serialized_code_d_ =
    autoware::cuda_utils::make_unique<std::int64_t[]>(config_.max_num_voxels_ * 2);

  // Encoder outputs shared with all the heads: one feature buffer per stage.
  stage_feat_d_.clear();
  stage_feat_d_.reserve(config_.enc_channels_.size());
  for (std::size_t stage = 0; stage < config_.enc_channels_.size(); ++stage) {
    stage_feat_d_.push_back(
      autoware::cuda_utils::make_unique<float[]>(
        config_.stage_voxel_capacity(stage) * config_.enc_channels_[stage]));
  }

  if (config_.use_seg3d_head_) {
    pred_labels_d_ = autoware::cuda_utils::make_unique<std::int64_t[]>(config_.max_num_voxels_);
    pred_probs_d_ = autoware::cuda_utils::make_unique<float[]>(
      config_.max_num_voxels_ * config_.segmentation_class_names_.size());
    if (config_.source_reconstruction_ == SourceReconstruction::PARTIAL) {
      cropped_source_points_d_ = autoware::cuda_utils::make_unique<std::uint8_t[]>(
        config_.cloud_capacity_ * sizeof(CloudPointTypeXYZIRCAEDT));
    }
    if (config_.source_reconstruction_ != SourceReconstruction::NONE) {
      inverse_map_d_ = autoware::cuda_utils::make_unique<std::int64_t[]>(config_.cloud_capacity_);
      reconstructed_labels_d_ =
        autoware::cuda_utils::make_unique<std::int64_t[]>(config_.cloud_capacity_);
      reconstructed_probs_d_ = autoware::cuda_utils::make_unique<float[]>(
        config_.cloud_capacity_ * config_.segmentation_class_names_.size());
    }
  }

  // Detection head output buffers and the instance memory.
  if (config_.use_det3d_head_) {
    const auto classification_size = config_.num_anchor_ * config_.detection_class_names_.size();
    const auto prediction_size = config_.num_anchor_ * Detection3DPostprocess::anchor_dim;
    const auto quality_size = config_.num_anchor_ * Detection3DPostprocess::quality_dim;
    classification_d_ = autoware::cuda_utils::make_unique<float[]>(classification_size);
    prediction_d_ = autoware::cuda_utils::make_unique<float[]>(prediction_size);
    quality_d_ = autoware::cuda_utils::make_unique<float[]>(quality_size);
    state_query_index_d_ =
      autoware::cuda_utils::make_unique<std::int64_t[]>(config_.num_temp_instances_);
    classification_h_ =
      autoware::cuda_utils::make_unique_host<float[]>(classification_size, cudaHostAllocDefault);
    prediction_h_ =
      autoware::cuda_utils::make_unique_host<float[]>(prediction_size, cudaHostAllocDefault);
    quality_h_ =
      autoware::cuda_utils::make_unique_host<float[]>(quality_size, cudaHostAllocDefault);
    state_query_index_h_ = autoware::cuda_utils::make_unique_host<std::int64_t[]>(
      config_.num_temp_instances_, cudaHostAllocDefault);
    detection3d_post_ptr_ = std::make_unique<Detection3DPostprocess>(config_);
    detection3d_state_ptr_ = std::make_unique<Detection3DTemporalState>(config_, stream_);
    instance_tracker_ptr_ = std::make_unique<InstanceTracker>(
      config_.num_anchor_, config_.num_temp_instances_, config_.track_birth_score_);
  }

  pre_ptr_ = std::make_unique<PreprocessCuda>(config_, stream_);
  if (config_.use_seg3d_head_) {
    post_ptr_ = std::make_unique<PostprocessCuda>(config_, stream_);
  }

  allocateSegOutputMessages();
  allocateSerializedPoolingBuffers();
}

void PTv3TRT::allocateSerializedPoolingBuffers()
{
  serialized_pooling_stages_d_.clear();
  serialized_pooling_stages_d_.reserve(config_.pooling_strides_.size());
  const auto max_num_voxels = static_cast<std::size_t>(config_.max_num_voxels_);
  const auto num_orders = config_.serialization_orders_.size();

  for (std::size_t stage_index = 0; stage_index < config_.pooling_strides_.size(); ++stage_index) {
    SerializedPoolingDeviceStage stage;
    stage.indices = autoware::cuda_utils::make_unique<std::int64_t[]>(max_num_voxels);
    stage.indptr = autoware::cuda_utils::make_unique<std::int64_t[]>(max_num_voxels + 1);
    stage.head_indices = autoware::cuda_utils::make_unique<std::int64_t[]>(max_num_voxels);
    stage.cluster = autoware::cuda_utils::make_unique<std::int64_t[]>(max_num_voxels);
    stage.grid_coord = autoware::cuda_utils::make_unique<std::int32_t[]>(max_num_voxels * 3);
    stage.serialized_code =
      autoware::cuda_utils::make_unique<std::int64_t[]>(max_num_voxels * num_orders);
    stage.serialized_order =
      autoware::cuda_utils::make_unique<std::int64_t[]>(max_num_voxels * num_orders);
    stage.serialized_inverse =
      autoware::cuda_utils::make_unique<std::int64_t[]>(max_num_voxels * num_orders);
    stage.patch_order = autoware::cuda_utils::make_unique<std::int64_t[]>(
      config_.padded_voxel_count(config_.max_num_voxels_, stage_index + 1) * num_orders);
    serialized_pooling_stages_d_.push_back(std::move(stage));
  }

  serialized_pooling_num_voxels_d_ =
    autoware::cuda_utils::make_unique<std::int64_t[]>(config_.pooling_strides_.size() + 1);
  serialized_pooling_num_voxels_ = autoware::cuda_utils::make_unique_host<std::int64_t[]>(
    config_.pooling_strides_.size() + 1, cudaHostAllocDefault);
  std::fill_n(serialized_pooling_num_voxels_.get(), config_.pooling_strides_.size() + 1, 0);
  serialized_pooling_depths_.resize(config_.pooling_strides_.size());
  for (std::size_t stage_index = 0; stage_index < config_.pooling_strides_.size(); ++stage_index) {
    serialized_pooling_depths_[stage_index] = poolingDepth(config_.pooling_strides_[stage_index]);
  }
}

void PTv3TRT::createPointFields()
{
  segmented_pointcloud_fields_ = point_cloud_msg_wrapper::generate_fields_from_point<
    point_types::PointXYZCPE, point_types::PointXYZCPEFieldGenerator>();

  auto make_point_field = [](const std::string & name, int offset, int datatype, int count) {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = count;
    return field;
  };

  visualization_pointcloud_fields_.push_back(
    make_point_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1));
  visualization_pointcloud_fields_.push_back(
    make_point_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1));
  visualization_pointcloud_fields_.push_back(
    make_point_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1));
  visualization_pointcloud_fields_.push_back(
    make_point_field("rgb", 12, sensor_msgs::msg::PointField::FLOAT32, 1));
}

void PTv3TRT::initEncoderTrt(const tensorrt_common::TrtCommonConfig & trt_config)
{
  // Models like LitePT declare only a subset of the PTv3 inputs, so check the
  // network inputs and only wire up what's declared.
  encoder_trt_ptr_ = std::make_unique<autoware::tensorrt_common::TrtCommon>(
    trt_config, std::make_shared<autoware::tensorrt_common::Profiler>(),
    std::vector<std::string>{config_.plugins_path_});

  std::vector<autoware::tensorrt_common::NetworkIO> network_io;
  std::vector<autoware::tensorrt_common::ProfileDims> profile_dims;

  // Everything except the geometry and the point features is optional: PTv3 variants such
  // as LitePT only need a subset of per-stage inputs.
  constexpr bool kOptional = true;
  const auto add_io = [&network_io, &profile_dims](
                        const std::string & name, const nvinfer1::Dims & io_dims,
                        const nvinfer1::Dims & min_dims, const nvinfer1::Dims & opt_dims,
                        const nvinfer1::Dims & max_dims,
                        const std::optional<nvinfer1::DataType> data_type = std::nullopt,
                        const bool is_optional = false) {
    network_io.emplace_back(name, io_dims, data_type, is_optional);
    profile_dims.emplace_back(name, min_dims, opt_dims, max_dims, is_optional);
  };

  const std::int64_t num_orders = static_cast<std::int64_t>(config_.serialization_orders_.size());

  // Inputs: padded voxel points and their counts (the graph averages them per voxel), grid
  // coordinates and the input level's serialization order.
  const auto input_counts = config_.stage_profile_counts(0);
  const auto max_points_per_voxel = config_.max_points_per_voxel_;
  const auto num_features = config_.num_point_feature_size_;
  add_io(
    "voxels", nvinfer1::Dims{3, {-1, max_points_per_voxel, num_features}},
    nvinfer1::Dims{3, {input_counts[0], max_points_per_voxel, num_features}},
    nvinfer1::Dims{3, {input_counts[1], max_points_per_voxel, num_features}},
    nvinfer1::Dims{3, {input_counts[2], max_points_per_voxel, num_features}},
    nvinfer1::DataType::kFLOAT);
  add_io(
    "num_points_per_voxel", nvinfer1::Dims{1, {-1}}, nvinfer1::Dims{1, {input_counts[0]}},
    nvinfer1::Dims{1, {input_counts[1]}}, nvinfer1::Dims{1, {input_counts[2]}},
    nvinfer1::DataType::kINT32);
  add_io(
    "grid_coord", nvinfer1::Dims{2, {-1, 3}}, nvinfer1::Dims{2, {input_counts[0], 3}},
    nvinfer1::Dims{2, {input_counts[1], 3}}, nvinfer1::Dims{2, {input_counts[2], 3}},
    nvinfer1::DataType::kINT32);
  // The input level's attention gathers through `patch_order` (its orders padded to whole windows)
  // and un-permutes through `serialized_inverse`; serialized_code stays a host-side buffer for
  // chaining the pooling stages. Only consumed when the finest stage attends; a convolution-only
  // stage 0 reads neither.
  {
    const auto padded = config_.padded_stage_profile_counts(0);
    add_io(
      "patch_order", nvinfer1::Dims{2, {num_orders, -1}},
      nvinfer1::Dims{2, {num_orders, padded[0]}}, nvinfer1::Dims{2, {num_orders, padded[1]}},
      nvinfer1::Dims{2, {num_orders, padded[2]}}, nvinfer1::DataType::kINT64, kOptional);
  }
  add_io(
    "serialized_inverse", nvinfer1::Dims{2, {num_orders, -1}},
    nvinfer1::Dims{2, {num_orders, input_counts[0]}},
    nvinfer1::Dims{2, {num_orders, input_counts[1]}},
    nvinfer1::Dims{2, {num_orders, input_counts[2]}}, nvinfer1::DataType::kINT64, kOptional);

  // Serialized pooling metadata inputs are precomputed on device each frame and fed to the
  // engine. Cluster tensors are computed too but consumed only by the head engines
  // (as pooling_cluster_i); the encoder graph does not take them.
  // In the exported ONNX, indices drive native Gather, indptr drives SegmentCSR, and the remaining
  // per-stage tensors feed the following PTv3 serialization steps. Their extents are
  // data-dependent, so they are declared dynamic and bounded by the profile of the stage they
  // read (indices) or produce (the pooled tensors).
  for (std::size_t stage = 0; stage < config_.pooling_strides_.size(); ++stage) {
    const auto prefix = "serialized_pooling_" + std::to_string(stage) + "_";
    const auto in_counts = config_.stage_profile_counts(stage);
    const auto out_counts = config_.stage_profile_counts(stage + 1);
    // Input-count-sized tensors.
    add_io(
      prefix + "indices", nvinfer1::Dims{1, {-1}}, nvinfer1::Dims{1, {in_counts[0]}},
      nvinfer1::Dims{1, {in_counts[1]}}, nvinfer1::Dims{1, {in_counts[2]}}, std::nullopt,
      kOptional);
    // Output-count-sized (pooled) tensors.
    add_io(
      prefix + "indptr", nvinfer1::Dims{1, {-1}}, nvinfer1::Dims{1, {out_counts[0] + 1}},
      nvinfer1::Dims{1, {out_counts[1] + 1}}, nvinfer1::Dims{1, {out_counts[2] + 1}}, std::nullopt,
      kOptional);
    add_io(
      prefix + "head_indices", nvinfer1::Dims{1, {-1}}, nvinfer1::Dims{1, {out_counts[0]}},
      nvinfer1::Dims{1, {out_counts[1]}}, nvinfer1::Dims{1, {out_counts[2]}}, std::nullopt,
      kOptional);
    add_io(
      prefix + "grid_coord", nvinfer1::Dims{2, {-1, 3}}, nvinfer1::Dims{2, {out_counts[0], 3}},
      nvinfer1::Dims{2, {out_counts[1], 3}}, nvinfer1::Dims{2, {out_counts[2], 3}},
      nvinfer1::DataType::kINT32, kOptional);
    {
      const auto padded = config_.padded_stage_profile_counts(stage + 1);
      add_io(
        prefix + "patch_order", nvinfer1::Dims{2, {num_orders, -1}},
        nvinfer1::Dims{2, {num_orders, padded[0]}}, nvinfer1::Dims{2, {num_orders, padded[1]}},
        nvinfer1::Dims{2, {num_orders, padded[2]}}, std::nullopt, kOptional);
    }
    add_io(
      prefix + "serialized_inverse", nvinfer1::Dims{2, {num_orders, -1}},
      nvinfer1::Dims{2, {num_orders, out_counts[0]}},
      nvinfer1::Dims{2, {num_orders, out_counts[1]}},
      nvinfer1::Dims{2, {num_orders, out_counts[2]}}, std::nullopt, kOptional);
  }

  // Outputs: per-encoder-stage point features point_feat_i [N_i, enc_channels[i]],
  // finest to deepest.
  for (std::size_t stage = 0; stage < config_.enc_channels_.size(); ++stage) {
    network_io.emplace_back(
      stageFeatureName(stage), nvinfer1::Dims{2, {-1, config_.enc_channels_[stage]}},
      nvinfer1::DataType::kFLOAT);
  }

  if (!encoder_trt_ptr_->setup(
        std::make_unique<std::vector<autoware::tensorrt_common::ProfileDims>>(profile_dims),
        std::make_unique<std::vector<autoware::tensorrt_common::NetworkIO>>(network_io))) {
    throw std::runtime_error("Failed to setup encoder TRT engine.");
  }

  // Binding a tensor the artifact omitted is a no-op, since it was offered as optional.
  encoder_trt_ptr_->setTensorAddress("voxels", voxels_d_.get());
  encoder_trt_ptr_->setTensorAddress("num_points_per_voxel", num_points_per_voxel_d_.get());
  encoder_trt_ptr_->setTensorAddress("grid_coord", grid_coord_d_.get());
  encoder_trt_ptr_->setTensorAddress("patch_order", pre_ptr_->inputLevelPatchOrder());
  encoder_trt_ptr_->setTensorAddress("serialized_inverse", pre_ptr_->inputLevelSerializedInverse());
  for (std::size_t stage = 0; stage < stage_feat_d_.size(); ++stage) {
    encoder_trt_ptr_->setTensorAddress(stageFeatureName(stage).c_str(), stage_feat_d_[stage].get());
  }
  bindSerializedPoolingAddresses();
}

void PTv3TRT::initSeg3dHeadTrt(const tensorrt_common::TrtCommonConfig & trt_config)
{
  std::vector<autoware::tensorrt_common::NetworkIO> network_io;
  std::vector<autoware::tensorrt_common::ProfileDims> profile_dims;

  // Inputs: per-encoder-stage features plus the per-pooling cluster tensors that drive the
  // segmentation decoder's unpooling. Cluster s maps every stage-s voxel to its pooled voxel,
  // so it is stage-s-count-sized.
  const auto stage_count = config_.enc_channels_.size();
  for (std::size_t stage = 0; stage < stage_count; ++stage) {
    const auto counts = config_.stage_profile_counts(stage);
    const auto channels = config_.enc_channels_[stage];
    network_io.emplace_back(
      stageFeatureName(stage), nvinfer1::Dims{2, {-1, channels}}, nvinfer1::DataType::kFLOAT);
    profile_dims.emplace_back(
      stageFeatureName(stage), nvinfer1::Dims{2, {counts[0], channels}},
      nvinfer1::Dims{2, {counts[1], channels}}, nvinfer1::Dims{2, {counts[2], channels}});
  }
  for (std::size_t stage = 0; stage + 1 < stage_count; ++stage) {
    const auto counts = config_.stage_profile_counts(stage);
    network_io.emplace_back(
      poolingClusterName(stage), nvinfer1::Dims{1, {-1}}, nvinfer1::DataType::kINT64);
    profile_dims.emplace_back(
      poolingClusterName(stage), nvinfer1::Dims{1, {counts[0]}}, nvinfer1::Dims{1, {counts[1]}},
      nvinfer1::Dims{1, {counts[2]}});
  }

  // Stages with decoder blocks additionally consume their serialization metadata
  const auto num_orders = static_cast<std::int64_t>(config_.serialization_orders_.size());
  for (std::size_t stage = 0; stage + 1 < stage_count; ++stage) {
    if (config_.dec_depths_[stage] == 0) {
      continue;
    }
    const auto counts = config_.stage_profile_counts(stage);
    const auto padded = config_.padded_stage_profile_counts(stage);
    // Stage 0 reads the input level's own tensors; deeper stages the pooling metadata of the
    // stage that produced them - the same names and tensors the encoder consumes.
    const auto prefix =
      stage == 0 ? std::string() : "serialized_pooling_" + std::to_string(stage - 1) + "_";
    network_io.emplace_back(
      prefix + "patch_order", nvinfer1::Dims{2, {num_orders, -1}}, nvinfer1::DataType::kINT64);
    profile_dims.emplace_back(
      prefix + "patch_order", nvinfer1::Dims{2, {num_orders, padded[0]}},
      nvinfer1::Dims{2, {num_orders, padded[1]}}, nvinfer1::Dims{2, {num_orders, padded[2]}});
    network_io.emplace_back(
      prefix + "serialized_inverse", nvinfer1::Dims{2, {num_orders, -1}},
      nvinfer1::DataType::kINT64);
    profile_dims.emplace_back(
      prefix + "serialized_inverse", nvinfer1::Dims{2, {num_orders, counts[0]}},
      nvinfer1::Dims{2, {num_orders, counts[1]}}, nvinfer1::Dims{2, {num_orders, counts[2]}});
    network_io.emplace_back(
      prefix + "grid_coord", nvinfer1::Dims{2, {-1, 3}}, nvinfer1::DataType::kINT32);
    profile_dims.emplace_back(
      prefix + "grid_coord", nvinfer1::Dims{2, {counts[0], 3}}, nvinfer1::Dims{2, {counts[1], 3}},
      nvinfer1::Dims{2, {counts[2], 3}});
  }

  network_io.emplace_back("pred_labels", nvinfer1::Dims{1, {-1}}, nvinfer1::DataType::kINT64);
  network_io.emplace_back(
    "pred_probs",
    nvinfer1::Dims{2, {-1, static_cast<std::int64_t>(config_.segmentation_class_names_.size())}},
    nvinfer1::DataType::kFLOAT);

  seg3d_head_trt_ptr_ = std::make_unique<autoware::tensorrt_common::TrtCommon>(
    trt_config, std::make_shared<autoware::tensorrt_common::Profiler>(),
    std::vector<std::string>{config_.plugins_path_});

  if (!seg3d_head_trt_ptr_->setup(
        std::make_unique<std::vector<autoware::tensorrt_common::ProfileDims>>(profile_dims),
        std::make_unique<std::vector<autoware::tensorrt_common::NetworkIO>>(network_io))) {
    throw std::runtime_error("Failed to setup seg3d_head TRT engine.");
  }

  // Feature buffers are encoder outputs; cluster buffers are preprocessing outputs consumed
  // by the head engines. All addresses are stable (allocated once).
  for (std::size_t stage = 0; stage < stage_count; ++stage) {
    seg3d_head_trt_ptr_->setTensorAddress(
      stageFeatureName(stage).c_str(), stage_feat_d_[stage].get());
  }
  for (std::size_t stage = 0; stage + 1 < stage_count; ++stage) {
    seg3d_head_trt_ptr_->setTensorAddress(
      poolingClusterName(stage).c_str(), serialized_pooling_stages_d_[stage].cluster.get());
  }
  for (std::size_t stage = 0; stage + 1 < stage_count; ++stage) {
    if (config_.dec_depths_[stage] == 0) {
      continue;
    }
    if (stage == 0) {
      seg3d_head_trt_ptr_->setTensorAddress("patch_order", pre_ptr_->inputLevelPatchOrder());
      seg3d_head_trt_ptr_->setTensorAddress(
        "serialized_inverse", pre_ptr_->inputLevelSerializedInverse());
      seg3d_head_trt_ptr_->setTensorAddress("grid_coord", grid_coord_d_.get());
      continue;
    }
    const auto prefix = "serialized_pooling_" + std::to_string(stage - 1) + "_";
    auto & buffers = serialized_pooling_stages_d_[stage - 1];
    seg3d_head_trt_ptr_->setTensorAddress(
      (prefix + "patch_order").c_str(), buffers.patch_order.get());
    seg3d_head_trt_ptr_->setTensorAddress(
      (prefix + "serialized_inverse").c_str(), buffers.serialized_inverse.get());
    seg3d_head_trt_ptr_->setTensorAddress(
      (prefix + "grid_coord").c_str(), buffers.grid_coord.get());
  }
  seg3d_head_trt_ptr_->setTensorAddress("pred_labels", pred_labels_d_.get());
  seg3d_head_trt_ptr_->setTensorAddress("pred_probs", pred_probs_d_.get());
}

void PTv3TRT::bindSerializedPoolingAddresses()
{
  // Metadata buffers are allocated once and never reallocated, so binding them here is enough.
  // Per-stage serialized_code only chains the pooling stages host-side and is not an engine input.
  // Metadata is generated for every stage, but a gated encoder reads only the part it declares.
  for (std::size_t stage = 0; stage < serialized_pooling_stages_d_.size(); ++stage) {
    const auto prefix = "serialized_pooling_" + std::to_string(stage) + "_";
    auto & buffers = serialized_pooling_stages_d_[stage];
    const auto bind = [this, &prefix](const std::string & field, void * data) {
      encoder_trt_ptr_->setTensorAddress((prefix + field).c_str(), data);
    };
    bind("indices", buffers.indices.get());
    bind("indptr", buffers.indptr.get());
    bind("head_indices", buffers.head_indices.get());
    bind("grid_coord", buffers.grid_coord.get());
    bind("patch_order", buffers.patch_order.get());
    bind("serialized_inverse", buffers.serialized_inverse.get());
  }
}

void PTv3TRT::precomputeSerializedPoolingMetadata()
{
  if (config_.pooling_strides_.empty()) {
    return;
  }

  std::vector<SerializedPoolingDeviceStageView> stage_views;
  stage_views.reserve(serialized_pooling_stages_d_.size());
  for (auto & stage : serialized_pooling_stages_d_) {
    stage_views.push_back(
      SerializedPoolingDeviceStageView{
        stage.indices.get(), stage.indptr.get(), stage.head_indices.get(), stage.cluster.get(),
        stage.grid_coord.get(), stage.serialized_code.get(), stage.serialized_order.get(),
        stage.serialized_inverse.get(), stage.patch_order.get()});
  }

  pre_ptr_->generateSerializedPoolingMetadata(
    grid_coord_d_.get(), serialized_code_d_.get(), num_voxels_, stage_views,
    serialized_pooling_num_voxels_d_.get(), serialized_pooling_num_voxels_.get());

  if (serialized_pooling_num_voxels_[0] < num_voxels_) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("ptv3"),
      "A pooled level exceeded encoder.pooled_voxels_num_max; the input was truncated from "
        << num_voxels_ << " to " << serialized_pooling_num_voxels_[0] << " voxels.");
    num_voxels_ = serialized_pooling_num_voxels_[0];
  }
}

bool PTv3TRT::setSerializedPoolingInputShapes()
{
  bool success = true;
  const auto num_orders = static_cast<std::int64_t>(config_.serialization_orders_.size());

  // serialized_pooling_num_voxels_[s] is the input count of stage s (entry 0 is num_voxels_).
  // [s + 1] is its pooled output count, which sets the SegmentCSR output shape and the shape of
  // the metadata consumed by later PTv3 blocks.
  for (std::size_t stage = 0; stage < serialized_pooling_stages_d_.size(); ++stage) {
    const auto prefix = "serialized_pooling_" + std::to_string(stage) + "_";
    const auto in_count = serialized_pooling_num_voxels_[stage];
    const auto out_count = serialized_pooling_num_voxels_[stage + 1];
    const auto set_shape = [this, &prefix, &success](
                             const std::string & field, const nvinfer1::Dims & dims) {
      success &= encoder_trt_ptr_->setInputShape((prefix + field).c_str(), dims);
    };
    set_shape("indices", nvinfer1::Dims{1, {in_count}});
    set_shape("indptr", nvinfer1::Dims{1, {out_count + 1}});
    set_shape("head_indices", nvinfer1::Dims{1, {out_count}});
    set_shape("grid_coord", nvinfer1::Dims{2, {out_count, 3}});
    set_shape(
      "patch_order",
      nvinfer1::Dims{2, {num_orders, config_.padded_voxel_count(out_count, stage + 1)}});
    set_shape("serialized_inverse", nvinfer1::Dims{2, {num_orders, out_count}});
  }

  return success;
}

void PTv3TRT::initDetection3DHeadTrt(const tensorrt_common::TrtCommonConfig & trt_config)
{
  std::vector<autoware::tensorrt_common::NetworkIO> network_io;
  std::vector<autoware::tensorrt_common::ProfileDims> profile_dims;

  // Inputs: the features and voxel coordinates of the encoder stages the head samples, plus the
  // pooling clusters that link every stage to the next one. Every stage is offered as optional,
  // the model declares the stages it reads.
  const auto add_stage_io = [&network_io, &profile_dims](
                              const std::string & name, const nvinfer1::Dims & io_dims,
                              const nvinfer1::Dims & min_dims, const nvinfer1::Dims & opt_dims,
                              const nvinfer1::Dims & max_dims, const nvinfer1::DataType data_type) {
    network_io.emplace_back(name, io_dims, data_type, true);
    profile_dims.emplace_back(name, min_dims, opt_dims, max_dims, true);
  };
  const auto stage_count = config_.enc_channels_.size();
  for (std::size_t stage = 0; stage < stage_count; ++stage) {
    const auto counts = config_.stage_profile_counts(stage);
    const auto channels = config_.enc_channels_[stage];
    add_stage_io(
      stageFeatureName(stage), nvinfer1::Dims{2, {-1, channels}},
      nvinfer1::Dims{2, {counts[0], channels}}, nvinfer1::Dims{2, {counts[1], channels}},
      nvinfer1::Dims{2, {counts[2], channels}}, nvinfer1::DataType::kFLOAT);
    add_stage_io(
      stageGridCoordName(stage), nvinfer1::Dims{2, {-1, 3}}, nvinfer1::Dims{2, {counts[0], 3}},
      nvinfer1::Dims{2, {counts[1], 3}}, nvinfer1::Dims{2, {counts[2], 3}},
      nvinfer1::DataType::kINT32);
  }
  for (std::size_t stage = 0; stage + 1 < stage_count; ++stage) {
    const auto counts = config_.stage_profile_counts(stage);
    add_stage_io(
      poolingClusterName(stage), nvinfer1::Dims{1, {-1}}, nvinfer1::Dims{1, {counts[0]}},
      nvinfer1::Dims{1, {counts[1]}}, nvinfer1::Dims{1, {counts[2]}}, nvinfer1::DataType::kINT64);
  }

  // The instance memory of the last frame and how the incoming frame relates to it
  const auto num_classes = static_cast<std::int64_t>(config_.detection_class_names_.size());
  const auto num_anchor = static_cast<std::int64_t>(config_.num_anchor_);
  const auto num_temp = static_cast<std::int64_t>(config_.num_temp_instances_);
  const auto embed_dims = static_cast<std::int64_t>(config_.embed_dims_);
  const auto anchor_dim = static_cast<std::int64_t>(Detection3DPostprocess::anchor_dim);
  const auto quality_dim = static_cast<std::int64_t>(Detection3DPostprocess::quality_dim);
  const auto add_static_io = [&network_io](
                               const char * name, const nvinfer1::Dims & dims,
                               const nvinfer1::DataType data_type) {
    network_io.emplace_back(name, dims, data_type);
  };
  add_static_io(
    "temp_instance_feature", nvinfer1::Dims{3, {1, num_temp, embed_dims}},
    nvinfer1::DataType::kFLOAT);
  add_static_io(
    "temp_anchor", nvinfer1::Dims{3, {1, num_temp, anchor_dim}}, nvinfer1::DataType::kFLOAT);
  add_static_io("temp_confidence", nvinfer1::Dims{2, {1, num_temp}}, nvinfer1::DataType::kFLOAT);
  add_static_io("temp_valid", nvinfer1::Dims{1, {1}}, nvinfer1::DataType::kFLOAT);
  add_static_io("time_interval", nvinfer1::Dims{1, {1}}, nvinfer1::DataType::kFLOAT);
  add_static_io("prev2current", nvinfer1::Dims{3, {1, 4, 4}}, nvinfer1::DataType::kFLOAT);

  // Outputs: the last decoder layer of every query and the memory of the next frame
  add_static_io(
    "classification", nvinfer1::Dims{3, {1, num_anchor, num_classes}}, nvinfer1::DataType::kFLOAT);
  add_static_io(
    "prediction", nvinfer1::Dims{3, {1, num_anchor, anchor_dim}}, nvinfer1::DataType::kFLOAT);
  add_static_io(
    "quality", nvinfer1::Dims{3, {1, num_anchor, quality_dim}}, nvinfer1::DataType::kFLOAT);
  add_static_io(
    "temp_instance_feature_out", nvinfer1::Dims{3, {1, num_temp, embed_dims}},
    nvinfer1::DataType::kFLOAT);
  add_static_io(
    "temp_anchor_out", nvinfer1::Dims{3, {1, num_temp, anchor_dim}}, nvinfer1::DataType::kFLOAT);
  add_static_io(
    "temp_confidence_out", nvinfer1::Dims{2, {1, num_temp}}, nvinfer1::DataType::kFLOAT);
  add_static_io(
    "temp_query_index_out", nvinfer1::Dims{2, {1, num_temp}}, nvinfer1::DataType::kINT64);

  detection3d_head_trt_ptr_ = std::make_unique<autoware::tensorrt_common::TrtCommon>(
    trt_config, std::make_shared<autoware::tensorrt_common::Profiler>(),
    std::vector<std::string>{config_.plugins_path_});

  if (!detection3d_head_trt_ptr_->setup(
        std::make_unique<std::vector<autoware::tensorrt_common::ProfileDims>>(profile_dims),
        std::make_unique<std::vector<autoware::tensorrt_common::NetworkIO>>(network_io))) {
    throw std::runtime_error("Failed to setup Detection3D head TRT engine.");
  }

  const auto bind = [this](const char * name, void * data) {
    detection3d_head_trt_ptr_->setTensorAddress(name, data);
  };
  // The voxel coordinates of stage 0 are the encoder input ones, every deeper stage has the
  // grid_coord output of the pooling that produced it.
  for (std::size_t stage = 0; stage < stage_count; ++stage) {
    bind(stageFeatureName(stage).c_str(), stage_feat_d_[stage].get());
    bind(
      stageGridCoordName(stage).c_str(),
      stage == 0 ? grid_coord_d_.get() : serialized_pooling_stages_d_[stage - 1].grid_coord.get());
  }
  for (std::size_t stage = 0; stage + 1 < stage_count; ++stage) {
    bind(poolingClusterName(stage).c_str(), serialized_pooling_stages_d_[stage].cluster.get());
  }
  bind("temp_instance_feature", detection3d_state_ptr_->instanceFeatureIn());
  bind("temp_anchor", detection3d_state_ptr_->anchorIn());
  bind("temp_confidence", detection3d_state_ptr_->confidenceIn());
  bind("temp_valid", detection3d_state_ptr_->validIn());
  bind("time_interval", detection3d_state_ptr_->timeIntervalIn());
  bind("prev2current", detection3d_state_ptr_->prev2currentIn());
  bind("classification", classification_d_.get());
  bind("prediction", prediction_d_.get());
  bind("quality", quality_d_.get());
  bind("temp_instance_feature_out", detection3d_state_ptr_->instanceFeatureOut());
  bind("temp_anchor_out", detection3d_state_ptr_->anchorOut());
  bind("temp_confidence_out", detection3d_state_ptr_->confidenceOut());
  bind("temp_query_index_out", state_query_index_d_.get());
}

bool PTv3TRT::infer(
  const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr,
  const StampedEgoPose & ego_pose, bool should_publish_segmented_pointcloud,
  bool should_publish_visualization_pointcloud, bool should_publish_filtered_pointcloud,
  bool should_detect_objects, std::optional<std::vector<Box3D>> & det_boxes3d,
  std::unordered_map<std::string, double> & proc_timing)
{
  det_boxes3d.reset();

  const bool should_run_seg3d =
    config_.use_seg3d_head_ &&
    (should_publish_segmented_pointcloud || should_publish_visualization_pointcloud ||
     should_publish_filtered_pointcloud);
  const bool should_run_det3d = config_.use_det3d_head_ && should_detect_objects;

  // The detection memory inputs of the frame are staged ahead of the encoder
  bool det_continues_stream = false;
  if (should_run_det3d) {
    det_continues_stream = detection3d_state_ptr_->beginFrame(ego_pose);
  }

  stop_watch_ptr_->toc("processing/inner", true);
  if (!preProcess(msg_ptr, should_run_seg3d)) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Pre-process failed. Skipping inference.");
    return false;
  }
  proc_timing.emplace(
    "debug/processing_time/preprocess_ms", stop_watch_ptr_->toc("processing/inner", true));

  if (!inferenceEncoder()) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Encoder inference failed.");
    return false;
  }

  bool seg_ok = !should_run_seg3d;
  bool det_ok = !should_run_det3d;
  bool seg_post_ok = !should_run_seg3d;
  bool det_post_ok = !should_run_det3d;

  if (should_run_seg3d) {
    seg_ok = inferenceSeg3dHead();
    if (!seg_ok) {
      RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Seg head inference failed.");
    }
  }
  if (should_run_det3d) {
    det_ok = inferenceDetection3DHead();
    if (det_ok) {
      copyDetection3DOutputsToHost();
      detection3d_state_ptr_->commitFrame();
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Det head inference failed.");
    }
  }

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
  proc_timing.emplace(
    "debug/processing_time/inference_ms", stop_watch_ptr_->toc("processing/inner", true));

  if (seg_ok && should_run_seg3d) {
    if (
      postProcess(
        msg_ptr, should_publish_segmented_pointcloud, should_publish_visualization_pointcloud,
        should_publish_filtered_pointcloud)) {
      seg_post_ok = true;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Seg post-process failed.");
    }
  }

  if (det_ok && should_run_det3d) {
    std::vector<Box3D> detected_boxes;
    postProcessDetection3D(det_continues_stream, detected_boxes);
    det_boxes3d = std::move(detected_boxes);
    det_post_ok = true;
  }

  proc_timing.emplace(
    "debug/processing_time/postprocess_ms", stop_watch_ptr_->toc("processing/inner", true));

  return (should_run_seg3d && seg_ok && seg_post_ok) || (should_run_det3d && det_ok && det_post_ok);
}

bool PTv3TRT::preProcess(
  const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr,
  const bool should_run_seg3d)
{
  using autoware::cuda_utils::clear_async;

  frame_format_ = detectCloudFormat(*msg_ptr);
  if (frame_format_ == CloudFormat::UNKNOWN) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("ptv3"),
      "Skipping a frame with an unsupported point cloud type. Expected one of: XYZIRCAEDT (10 "
      "fields), XYZIRADRT (9 fields), XYZIRC (6 fields), or XYZI (4 fields).");
    return false;
  }

  // The reconstruction buffers and output messages are sized per frame
  num_frame_points_ = static_cast<std::int64_t>(msg_ptr->height) * msg_ptr->width;
  if (num_frame_points_ > config_.cloud_capacity_) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("ptv3"),
      "Skipping a frame of " << num_frame_points_ << " points; it exceeds the cloud capacity ("
                             << config_.cloud_capacity_ << "). Increase cloud_capacity.");
    return false;
  }
  if (num_frame_points_ == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Empty pointcloud. Skipping inference.");
    return false;
  }

  generatePointFeaturesLaunch(
    msg_ptr->data.get(), frame_format_, static_cast<std::size_t>(num_frame_points_),
    point_features_d_.get(), config_.threads_per_block_, stream_);
  CHECK_CUDA_ERROR(cudaPeekAtLastError());

  std::call_once(init_cloud_, [this]() {
    input_format_ = frame_format_;

    const auto requested_output_format = parse_cloud_format_string(config_.filter_output_format_);
    filtered_output_format_ =
      config_.filter_output_format_.empty() ? input_format_ : requested_output_format;
    if (
      filtered_output_format_ == CloudFormat::UNKNOWN ||
      !can_convert_format(input_format_, filtered_output_format_)) {
      throw std::runtime_error(
        "filter.output_format='" + config_.filter_output_format_ +
        "' is not compatible with input format '" + std::string(to_string(input_format_)) + "'.");
    }

    auto append_field = [this](const std::string & name, int offset, int datatype, int count) {
      sensor_msgs::msg::PointField field;
      field.name = name;
      field.offset = offset;
      field.datatype = datatype;
      field.count = count;
      filtered_pointcloud_fields_.push_back(field);
    };
    filtered_pointcloud_fields_.clear();
    switch (filtered_output_format_) {
      case CloudFormat::XYZIRCAEDT:
        append_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("intensity", 12, sensor_msgs::msg::PointField::UINT8, 1);
        append_field("return_type", 13, sensor_msgs::msg::PointField::UINT8, 1);
        append_field("channel", 14, sensor_msgs::msg::PointField::UINT16, 1);
        append_field("azimuth", 16, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("elevation", 20, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("distance", 24, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("time_stamp", 28, sensor_msgs::msg::PointField::UINT32, 1);
        break;
      case CloudFormat::XYZIRADRT:
        append_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("intensity", 12, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("ring", 16, sensor_msgs::msg::PointField::UINT16, 1);
        append_field("azimuth", 18, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("distance", 22, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("return_type", 26, sensor_msgs::msg::PointField::UINT8, 1);
        append_field("time_stamp", 27, sensor_msgs::msg::PointField::FLOAT64, 1);
        break;
      case CloudFormat::XYZIRC:
        append_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("intensity", 12, sensor_msgs::msg::PointField::UINT8, 1);
        append_field("return_type", 13, sensor_msgs::msg::PointField::UINT8, 1);
        append_field("channel", 14, sensor_msgs::msg::PointField::UINT16, 1);
        break;
      case CloudFormat::XYZI:
        append_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("intensity", 12, sensor_msgs::msg::PointField::FLOAT32, 1);
        break;
      default:
        throw std::runtime_error("Unsupported filtered point cloud format.");
    }

    RCLCPP_INFO(
      rclcpp::get_logger("ptv3"),
      "Detected input format with %zu fields and point step %zu bytes; filtered output format '%s'",
      get_num_fields(input_format_), get_point_step(input_format_),
      to_string(filtered_output_format_));
  });
  allocateSegOutputMessages();

  clear_async(
    num_points_per_voxel_d_.get(), static_cast<std::size_t>(config_.max_num_voxels_), stream_);
  clear_async(grid_coord_d_.get(), static_cast<std::size_t>(config_.max_num_voxels_) * 3, stream_);
  clear_async(
    serialized_code_d_.get(), static_cast<std::size_t>(config_.max_num_voxels_) * 2, stream_);
  if (should_run_seg3d) {
    clear_async(pred_labels_d_.get(), static_cast<std::size_t>(config_.max_num_voxels_), stream_);
    clear_async(
      pred_probs_d_.get(),
      static_cast<std::size_t>(config_.max_num_voxels_) * config_.segmentation_class_names_.size(),
      stream_);
    if (config_.source_reconstruction_ == SourceReconstruction::PARTIAL) {
      clear_async(
        cropped_source_points_d_.get(),
        static_cast<std::size_t>(config_.cloud_capacity_) * sizeof(CloudPointTypeXYZIRCAEDT),
        stream_);
    }
    if (config_.source_reconstruction_ != SourceReconstruction::NONE) {
      clear_async(inverse_map_d_.get(), static_cast<std::size_t>(config_.cloud_capacity_), stream_);
      clear_async(
        reconstructed_labels_d_.get(), static_cast<std::size_t>(config_.cloud_capacity_), stream_);
      clear_async(
        reconstructed_probs_d_.get(),
        static_cast<std::size_t>(config_.cloud_capacity_) *
          config_.segmentation_class_names_.size(),
        stream_);
    }
  }

  std::size_t num_cropped_points = 0;
  const bool should_reconstruct_source =
    should_run_seg3d && config_.source_reconstruction_ != SourceReconstruction::NONE;
  num_voxels_ = pre_ptr_->generateVoxels(
    point_features_d_.get(), static_cast<std::size_t>(num_frame_points_), voxels_d_.get(),
    num_points_per_voxel_d_.get(), grid_coord_d_.get(), serialized_code_d_.get(),
    should_reconstruct_source ? inverse_map_d_.get() : nullptr, &num_cropped_points);
  num_cropped_points_ = static_cast<std::int64_t>(num_cropped_points);

  if (should_run_seg3d && config_.source_reconstruction_ == SourceReconstruction::PARTIAL) {
    pre_ptr_->extractSourcePoints(
      msg_ptr->data.get(), frame_format_, static_cast<std::size_t>(num_frame_points_),
      cropped_source_points_d_.get());
  }

  if (num_voxels_ > config_.max_num_voxels_) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("ptv3"), "Actual number of voxels ("
                                    << num_voxels_
                                    << ") is over the limit for the actual optimization profile ("
                                    << config_.max_num_voxels_ << "). Clipping to the limit.");
    num_voxels_ = config_.max_num_voxels_;
  }

  precomputeSerializedPoolingMetadata();
  if (num_voxels_ < config_.min_num_voxels_) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("ptv3"), "Too few voxels (" << num_voxels_
                                                     << ") for the actual optimization profile ("
                                                     << config_.min_num_voxels_ << ")");
    return false;
  }

  encoder_trt_ptr_->setInputShape(
    "voxels", nvinfer1::Dims{
                3, {num_voxels_, config_.max_points_per_voxel_, config_.num_point_feature_size_}});
  encoder_trt_ptr_->setInputShape("num_points_per_voxel", nvinfer1::Dims{1, {num_voxels_}});
  encoder_trt_ptr_->setInputShape("grid_coord", nvinfer1::Dims{2, {num_voxels_, 3}});
  const auto num_orders = static_cast<std::int64_t>(config_.serialization_orders_.size());
  if (!encoder_trt_ptr_->setInputShape(
        "patch_order",
        nvinfer1::Dims{2, {num_orders, config_.padded_voxel_count(num_voxels_, 0)}})) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Failed to set patch_order input shape.");
    return false;
  }
  if (!encoder_trt_ptr_->setInputShape(
        "serialized_inverse", nvinfer1::Dims{2, {num_orders, num_voxels_}})) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Failed to set serialized_inverse input shape.");
    return false;
  }

  if (!setSerializedPoolingInputShapes()) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Failed to set serialized pooling input shapes.");
    return false;
  }

  return true;
}

bool PTv3TRT::inferenceEncoder()
{
  if (!encoder_trt_ptr_->enqueueV3(stream_)) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Fail to enqueue encoder.");
    return false;
  }
  return true;
}

bool PTv3TRT::inferenceSeg3dHead()
{
  // serialized_pooling_num_voxels_[i] holds every encoder stage count (entry 0 is num_voxels_).
  bool success = true;
  const auto stage_count = config_.enc_channels_.size();
  const auto num_orders = static_cast<std::int64_t>(config_.serialization_orders_.size());
  for (std::size_t stage = 0; stage < stage_count; ++stage) {
    success &= seg3d_head_trt_ptr_->setInputShape(
      stageFeatureName(stage).c_str(),
      nvinfer1::Dims{2, {serialized_pooling_num_voxels_[stage], config_.enc_channels_[stage]}});
  }
  for (std::size_t stage = 0; stage + 1 < stage_count; ++stage) {
    success &= seg3d_head_trt_ptr_->setInputShape(
      poolingClusterName(stage).c_str(),
      nvinfer1::Dims{1, {serialized_pooling_num_voxels_[stage]}});
  }
  for (std::size_t stage = 0; stage + 1 < stage_count; ++stage) {
    if (config_.dec_depths_[stage] == 0) {
      continue;
    }
    const auto stage_count_voxels = serialized_pooling_num_voxels_[stage];
    const auto prefix =
      stage == 0 ? std::string() : "serialized_pooling_" + std::to_string(stage - 1) + "_";
    success &= seg3d_head_trt_ptr_->setInputShape(
      (prefix + "patch_order").c_str(),
      nvinfer1::Dims{2, {num_orders, config_.padded_voxel_count(stage_count_voxels, stage)}});
    success &= seg3d_head_trt_ptr_->setInputShape(
      (prefix + "serialized_inverse").c_str(), nvinfer1::Dims{2, {num_orders, stage_count_voxels}});
    success &= seg3d_head_trt_ptr_->setInputShape(
      (prefix + "grid_coord").c_str(), nvinfer1::Dims{2, {stage_count_voxels, 3}});
  }
  if (!success) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Failed to set seg3d head input shapes.");
    return false;
  }
  if (!seg3d_head_trt_ptr_->enqueueV3(stream_)) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Fail to enqueue seg3d head.");
    return false;
  }
  return true;
}

bool PTv3TRT::inferenceDetection3DHead()
{
  // serialized_pooling_num_voxels_[s] is the voxel count of encoder stage s. The engine skips the
  // shapes of the stages the model does not read.
  const auto stage_count = config_.enc_channels_.size();
  bool success = true;
  for (std::size_t stage = 0; stage < stage_count; ++stage) {
    const auto count = serialized_pooling_num_voxels_[stage];
    success &= detection3d_head_trt_ptr_->setInputShape(
      stageFeatureName(stage).c_str(), nvinfer1::Dims{2, {count, config_.enc_channels_[stage]}});
    success &= detection3d_head_trt_ptr_->setInputShape(
      stageGridCoordName(stage).c_str(), nvinfer1::Dims{2, {count, 3}});
  }
  for (std::size_t stage = 0; stage + 1 < stage_count; ++stage) {
    success &= detection3d_head_trt_ptr_->setInputShape(
      poolingClusterName(stage).c_str(),
      nvinfer1::Dims{1, {serialized_pooling_num_voxels_[stage]}});
  }
  if (!success) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Failed to set Detection3D head input shapes.");
    return false;
  }
  if (!detection3d_head_trt_ptr_->enqueueV3(stream_)) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Fail to enqueue Detection3D head.");
    return false;
  }
  return true;
}

bool PTv3TRT::postProcess(
  const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr,
  bool should_publish_segmented_pointcloud, bool should_publish_visualization_pointcloud,
  bool should_publish_filtered_pointcloud)
{
  const auto & header = msg_ptr->header;
  if (config_.source_reconstruction_ == SourceReconstruction::PARTIAL) {
    post_ptr_->reconstructPartial(
      inverse_map_d_.get(), pred_labels_d_.get(), pred_probs_d_.get(),
      reconstructed_labels_d_.get(), reconstructed_probs_d_.get(),
      config_.segmentation_class_names_.size(), num_cropped_points_, num_voxels_);
  }
  if (config_.source_reconstruction_ == SourceReconstruction::FULL) {
    post_ptr_->reconstructFull(
      pre_ptr_->cropMask(), pre_ptr_->cropIndices(), inverse_map_d_.get(), pred_labels_d_.get(),
      pred_probs_d_.get(), reconstructed_labels_d_.get(), reconstructed_probs_d_.get(),
      config_.segmentation_class_names_.size(), num_frame_points_, num_voxels_);
  }

  // Without reconstruction the outputs sit at voxel level, positioned at each voxel's first
  // point (slot 0 of the padded voxel), hence the padded row stride.
  const auto source_features =
    config_.source_reconstruction_ == SourceReconstruction::FULL      ? point_features_d_.get()
    : config_.source_reconstruction_ == SourceReconstruction::PARTIAL ? pre_ptr_->croppedFeatures()
                                                                      : voxels_d_.get();
  const auto source_feature_stride =
    config_.source_reconstruction_ == SourceReconstruction::NONE
      ? config_.max_points_per_voxel_ * config_.num_point_feature_size_
      : config_.num_point_feature_size_;
  const auto source_labels = config_.source_reconstruction_ != SourceReconstruction::NONE
                               ? reconstructed_labels_d_.get()
                               : pred_labels_d_.get();
  const auto source_probs = config_.source_reconstruction_ != SourceReconstruction::NONE
                              ? reconstructed_probs_d_.get()
                              : pred_probs_d_.get();
  const auto voxel_mapping = config_.source_reconstruction_ == SourceReconstruction::NONE
                               ? pre_ptr_->voxelPointMapping()
                               : VoxelPointMapping{};
  const void * source_points = config_.source_reconstruction_ == SourceReconstruction::PARTIAL
                                 ? cropped_source_points_d_.get()
                                 : msg_ptr->data.get();
  const auto num_source_output_points =
    config_.source_reconstruction_ == SourceReconstruction::FULL      ? num_frame_points_
    : config_.source_reconstruction_ == SourceReconstruction::PARTIAL ? num_cropped_points_
                                                                      : num_voxels_;

  if (should_publish_segmented_pointcloud) {
    const auto num_segmented_points = post_ptr_->createSegmentationPointcloud(
      source_features, source_feature_stride, source_labels, source_probs,
      reinterpret_cast<point_types::PointXYZCPE *>(segmented_points_msg_ptr_->data.get()),
      config_.segmentation_class_names_.size(), num_source_output_points);
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

    segmented_points_msg_ptr_->header = header;
    segmented_points_msg_ptr_->width = static_cast<std::uint32_t>(num_segmented_points);
    segmented_points_msg_ptr_->row_step =
      segmented_points_msg_ptr_->width * segmented_points_msg_ptr_->point_step;
    publish_segmented_pointcloud_(std::move(segmented_points_msg_ptr_));
    segmented_points_msg_ptr_ = nullptr;
  }

  // Visualization pointcloud
  if (should_publish_visualization_pointcloud) {
    post_ptr_->createVisualizationPointcloud(
      source_features, source_feature_stride, source_labels,
      reinterpret_cast<float *>(visualization_points_msg_ptr_->data.get()),
      config_.segmentation_class_names_.size(), num_source_output_points);
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
    visualization_points_msg_ptr_->header = header;
    visualization_points_msg_ptr_->width = static_cast<std::uint32_t>(num_source_output_points);
    visualization_points_msg_ptr_->row_step =
      visualization_points_msg_ptr_->width * visualization_points_msg_ptr_->point_step;
    publish_visualization_pointcloud_(std::move(visualization_points_msg_ptr_));
    visualization_points_msg_ptr_ = nullptr;
  }

  if (should_publish_filtered_pointcloud) {
    const auto num_filtered_points = post_ptr_->createFilteredPointcloud(
      source_points, frame_format_, filtered_output_format_, source_probs,
      filtered_points_msg_ptr_->data.get(), config_.segmentation_class_names_.size(),
      num_source_output_points, voxel_mapping);
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

    filtered_points_msg_ptr_->header = header;
    filtered_points_msg_ptr_->width = num_filtered_points;
    filtered_points_msg_ptr_->row_step =
      filtered_points_msg_ptr_->width * filtered_points_msg_ptr_->point_step;
    publish_filtered_pointcloud_(std::move(filtered_points_msg_ptr_));
    filtered_points_msg_ptr_ = nullptr;
  }

  allocateSegOutputMessages();
  return true;
}

void PTv3TRT::copyDetection3DOutputsToHost()
{
  const auto copy = [this](auto * host, const auto * device, const std::size_t count) {
    CHECK_CUDA_ERROR(
      cudaMemcpyAsync(host, device, count * sizeof(*host), cudaMemcpyDeviceToHost, stream_));
  };
  copy(
    classification_h_.get(), classification_d_.get(),
    config_.num_anchor_ * config_.detection_class_names_.size());
  copy(
    prediction_h_.get(), prediction_d_.get(),
    config_.num_anchor_ * Detection3DPostprocess::anchor_dim);
  copy(
    quality_h_.get(), quality_d_.get(), config_.num_anchor_ * Detection3DPostprocess::quality_dim);
  copy(state_query_index_h_.get(), state_query_index_d_.get(), config_.num_temp_instances_);
}

void PTv3TRT::postProcessDetection3D(
  const bool continues_stream, std::vector<Box3D> & detection_boxes)
{
  auto queries = detection3d_post_ptr_->decodeQueries(
    classification_h_.get(), prediction_h_.get(), quality_h_.get());
  instance_tracker_ptr_->assign(continues_stream, queries);
  instance_tracker_ptr_->commit(state_query_index_h_.get());
  detection_boxes = detection3d_post_ptr_->select(std::move(queries));
}

}  //  namespace autoware::ptv3
