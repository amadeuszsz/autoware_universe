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

#ifndef AUTOWARE__PTV3__PTV3_TRT_HPP_
#define AUTOWARE__PTV3__PTV3_TRT_HPP_

#include "autoware/ptv3/detection3d_temporal_state.hpp"
#include "autoware/ptv3/postprocess/detection3d_postprocess.hpp"
#include "autoware/ptv3/postprocess/instance_tracker.hpp"
#include "autoware/ptv3/postprocess/postprocess_kernel.hpp"
#include "autoware/ptv3/preprocess/point_type.hpp"
#include "autoware/ptv3/preprocess/preprocess_kernel.hpp"
#include "autoware/ptv3/utils.hpp"
#include "autoware/ptv3/visibility_control.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::ptv3
{

using autoware::cuda_utils::CudaUniquePtr;
using autoware::cuda_utils::CudaUniquePtrHost;

class PTV3_PUBLIC PTv3TRT
{
public:
  explicit PTv3TRT(
    const tensorrt_common::TrtCommonConfig & encoder_trt_config,
    const std::optional<tensorrt_common::TrtCommonConfig> & seg3d_head_trt_config,
    const std::optional<tensorrt_common::TrtCommonConfig> & det3d_head_trt_config,
    const PTv3Config & config);
  virtual ~PTv3TRT();

  /// Run one frame. The ego pose places the frame in its stream and carries the detection memory.
  /// Detected boxes hold the track of their instance, -1 for an instance without a track.
  // cSpell:ignore probs
  bool infer(
    const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr,
    const StampedEgoPose & ego_pose, bool should_publish_segmented_pointcloud,
    bool should_publish_visualization_pointcloud, bool should_publish_filtered_pointcloud,
    bool should_detect_objects, std::optional<std::vector<Box3D>> & det_boxes3d,
    std::unordered_map<std::string, double> & proc_timing);

  void setPublishSegmentedPointcloud(
    std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func);
  void setPublishVisualizationPointcloud(
    std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func);
  void setPublishFilteredPointcloud(
    std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func);

  /// CUDA stream used for pointcloud processing and inference.
  /// Enables stream-ordered producer/consumer lifetime handling.
  cudaStream_t stream() const { return stream_; }

protected:
  void initPtr();
  void initEncoderTrt(const tensorrt_common::TrtCommonConfig & trt_config);
  void initSeg3dHeadTrt(const tensorrt_common::TrtCommonConfig & trt_config);
  void initDetection3DHeadTrt(const tensorrt_common::TrtCommonConfig & trt_config);
  void createPointFields();
  void allocateSegOutputMessages();
  void allocateSerializedPoolingBuffers();
  void bindSerializedPoolingAddresses();
  void precomputeSerializedPoolingMetadata();
  bool setSerializedPoolingInputShapes();

  bool preProcess(
    const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr, bool should_run_seg3d);

  bool inferenceEncoder();
  bool inferenceSeg3dHead();
  bool inferenceDetection3DHead();
  void copyDetection3DOutputsToHost();

  bool postProcess(
    const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr,
    bool should_publish_segmented_pointcloud, bool should_publish_visualization_pointcloud,
    bool should_publish_filtered_pointcloud);

  void postProcessDetection3D(bool continues_stream, std::vector<Box3D> & detection_boxes);

  // The encoder is always present. The heads are loaded only when enabled.
  std::unique_ptr<autoware::tensorrt_common::TrtCommon> encoder_trt_ptr_{nullptr};
  std::unique_ptr<autoware::tensorrt_common::TrtCommon> seg3d_head_trt_ptr_{nullptr};
  std::unique_ptr<autoware::tensorrt_common::TrtCommon> detection3d_head_trt_ptr_{nullptr};
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_{nullptr};
  std::unique_ptr<PreprocessCuda> pre_ptr_{nullptr};
  std::unique_ptr<PostprocessCuda> post_ptr_{nullptr};
  std::unique_ptr<Detection3DPostprocess> detection3d_post_ptr_{nullptr};
  std::unique_ptr<Detection3DTemporalState> detection3d_state_ptr_{nullptr};
  std::unique_ptr<InstanceTracker> instance_tracker_ptr_{nullptr};
  cudaStream_t stream_{nullptr};

  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)>
    publish_segmented_pointcloud_{nullptr};
  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)>
    publish_visualization_pointcloud_{nullptr};
  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)>
    publish_filtered_pointcloud_{nullptr};

  std::vector<sensor_msgs::msg::PointField> segmented_pointcloud_fields_;
  std::vector<sensor_msgs::msg::PointField> visualization_pointcloud_fields_;
  std::vector<sensor_msgs::msg::PointField> filtered_pointcloud_fields_;

  std::unique_ptr<cuda_blackboard::CudaPointCloud2> segmented_points_msg_ptr_{nullptr};
  std::unique_ptr<cuda_blackboard::CudaPointCloud2> visualization_points_msg_ptr_{nullptr};
  std::unique_ptr<cuda_blackboard::CudaPointCloud2> filtered_points_msg_ptr_{nullptr};

  PTv3Config config_;
  std::once_flag init_cloud_;
  CloudFormat input_format_{CloudFormat::UNKNOWN};
  CloudFormat filtered_output_format_{CloudFormat::UNKNOWN};

  struct SerializedPoolingDeviceStage
  {
    CudaUniquePtr<std::int64_t[]> indices{nullptr};
    CudaUniquePtr<std::int64_t[]> indptr{nullptr};
    CudaUniquePtr<std::int64_t[]> head_indices{nullptr};
    CudaUniquePtr<std::int64_t[]> cluster{nullptr};
    CudaUniquePtr<std::int32_t[]> grid_coord{nullptr};
    CudaUniquePtr<std::int64_t[]> serialized_code{nullptr};
    CudaUniquePtr<std::int64_t[]> serialized_order{nullptr};
    CudaUniquePtr<std::int64_t[]> serialized_inverse{nullptr};
    CudaUniquePtr<std::int64_t[]> patch_order{nullptr};
  };

  std::vector<SerializedPoolingDeviceStage> serialized_pooling_stages_d_;
  CudaUniquePtr<std::int64_t[]> serialized_pooling_num_voxels_d_{nullptr};
  CudaUniquePtrHost<std::int64_t[]> serialized_pooling_num_voxels_;
  std::vector<std::int64_t> serialized_pooling_depths_;

  // Preprocess outputs of the frame: its point layout, point count and (x, y, z, intensity) rows
  CloudFormat frame_format_{CloudFormat::UNKNOWN};
  std::int64_t num_frame_points_{0};
  CudaUniquePtr<float[]> point_features_d_{nullptr};
  std::int64_t num_voxels_{0};
  std::int64_t num_cropped_points_{0};  // only for partial

  CudaUniquePtr<std::uint8_t[]> cropped_source_points_d_{nullptr};  // only for partial
  CudaUniquePtr<std::int64_t[]> inverse_map_d_{nullptr};            // only for partial and full
  CudaUniquePtr<std::int64_t[]> reconstructed_labels_d_{nullptr};   // only for partial and full
  CudaUniquePtr<float[]> reconstructed_probs_d_{nullptr};           // only for partial and full
  // Encoder inputs: padded voxel points [num_voxels, max_points_per_voxel, 4], their valid point
  // counts, grid coordinates and serialized codes.
  CudaUniquePtr<float[]> voxels_d_{nullptr};
  CudaUniquePtr<std::int32_t[]> num_points_per_voxel_d_{nullptr};
  CudaUniquePtr<std::int32_t[]> grid_coord_d_{nullptr};
  CudaUniquePtr<std::int64_t[]> serialized_code_d_{nullptr};

  // Encoder outputs shared with all the heads: per-stage point features,
  // finest to deepest, sized by each stage's geometric voxel capacity.
  std::vector<CudaUniquePtr<float[]>> stage_feat_d_;

  // Segmentation head outputs
  CudaUniquePtr<std::int64_t[]> pred_labels_d_{nullptr};
  CudaUniquePtr<float[]> pred_probs_d_{nullptr};

  // Detection3D head outputs of every query and the query each temporal slot was selected from,
  // decoded on the host. The memory tensors live in detection3d_state_ptr_.
  CudaUniquePtr<float[]> classification_d_{nullptr};
  CudaUniquePtr<float[]> prediction_d_{nullptr};
  CudaUniquePtr<float[]> quality_d_{nullptr};
  CudaUniquePtr<std::int64_t[]> state_query_index_d_{nullptr};
  CudaUniquePtrHost<float[]> classification_h_{nullptr};
  CudaUniquePtrHost<float[]> prediction_h_{nullptr};
  CudaUniquePtrHost<float[]> quality_h_{nullptr};
  CudaUniquePtrHost<std::int64_t[]> state_query_index_h_{nullptr};
};

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__PTV3_TRT_HPP_
