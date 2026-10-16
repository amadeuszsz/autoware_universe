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

#ifndef AUTOWARE__PTV3__DETECTION3D_TEMPORAL_STATE_HPP_
#define AUTOWARE__PTV3__DETECTION3D_TEMPORAL_STATE_HPP_

#include "autoware/ptv3/ptv3_config.hpp"

#include <Eigen/Geometry>
#include <autoware/cuda_utils/cuda_unique_ptr.hpp>

#include <cuda_runtime_api.h>

#include <cstddef>
#include <cstdint>
#include <optional>

namespace autoware::ptv3
{

/// Pose of the lidar frame of one point cloud in the world frame, at the cloud stamp.
struct StampedEgoPose
{
  double stamp_seconds;
  Eigen::Isometry3d lidar_to_world;
};

/// Carry the Sparse4D instance memory across detection engine invocations.
///
/// The engine takes the memory of the last frame (`temp_instance_feature`, `temp_anchor`,
/// `temp_confidence`) with the continuity gate (`temp_valid`), the frame gap (`time_interval`)
/// and the transform from the last frame into the current one (`prev2current`), and returns the
/// memory of the next frame (`*_out`). The anchor warp and the confidence decay run in the graph,
/// so this class only derives the per-frame inputs from the ego poses and feeds the outputs back.
///
/// The first frame and a frame gap outside (0, max_time_interval] start a new stream: the gate
/// closes and the graph discards the memory.
class Detection3DTemporalState
{
public:
  Detection3DTemporalState(const PTv3Config & config, cudaStream_t stream);

  /// Upload the gate, the frame gap and the transform of the incoming frame. Enqueue before the
  /// detection engine. Returns whether the frame continues the stream of the last committed one.
  bool beginFrame(const StampedEgoPose & ego_pose);

  /// Feed the memory outputs back as the next inputs and keep the frame pose. Enqueue after the
  /// detection engine on the same stream. A frame never committed leaves the memory untouched.
  void commitFrame();

  [[nodiscard]] float * instanceFeatureIn() const { return instance_feature_in_d_.get(); }
  [[nodiscard]] float * anchorIn() const { return anchor_in_d_.get(); }
  [[nodiscard]] float * confidenceIn() const { return confidence_in_d_.get(); }
  [[nodiscard]] float * validIn() const { return valid_d_.get(); }
  [[nodiscard]] float * timeIntervalIn() const { return time_interval_d_.get(); }
  [[nodiscard]] float * prev2currentIn() const { return prev2current_d_.get(); }
  [[nodiscard]] float * instanceFeatureOut() const { return instance_feature_out_d_.get(); }
  [[nodiscard]] float * anchorOut() const { return anchor_out_d_.get(); }
  [[nodiscard]] float * confidenceOut() const { return confidence_out_d_.get(); }

  /// Anchor state channels: [x, y, z, ln l, ln w, ln h, sin yaw, cos yaw, vx, vy].
  static constexpr std::size_t anchor_dim{10};

private:
  std::size_t num_temp_instances_;
  std::size_t embed_dims_;
  float default_time_interval_;
  float max_time_interval_;
  cudaStream_t stream_;

  autoware::cuda_utils::CudaUniquePtr<float[]> instance_feature_in_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> anchor_in_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> confidence_in_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> valid_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> time_interval_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> prev2current_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> instance_feature_out_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> anchor_out_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> confidence_out_d_;

  // Pinned staging of the per-frame inputs: gate, frame gap and the 4x4 transform, row-major.
  autoware::cuda_utils::CudaUniquePtrHost<float[]> frame_inputs_h_;

  std::optional<StampedEgoPose> committed_pose_;
  std::optional<StampedEgoPose> pending_pose_;
};

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__DETECTION3D_TEMPORAL_STATE_HPP_
