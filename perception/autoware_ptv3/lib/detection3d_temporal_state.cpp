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

#include "autoware/ptv3/detection3d_temporal_state.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/cuda_utils/cuda_utils.hpp>

#include <cstddef>

namespace autoware::ptv3
{

namespace
{
constexpr std::size_t transform_size{16};
// Per-frame input layout of the pinned staging buffer
constexpr std::size_t valid_offset{0};
constexpr std::size_t time_interval_offset{1};
constexpr std::size_t transform_offset{2};
}  // namespace

Detection3DTemporalState::Detection3DTemporalState(const PTv3Config & config, cudaStream_t stream)
: num_temp_instances_(config.num_temp_instances_),
  embed_dims_(config.embed_dims_),
  default_time_interval_(config.default_time_interval_),
  max_time_interval_(config.max_time_interval_),
  stream_(stream)
{
  const auto feature_size = num_temp_instances_ * embed_dims_;
  const auto anchor_size = num_temp_instances_ * anchor_dim;
  instance_feature_in_d_ = autoware::cuda_utils::make_unique<float[]>(feature_size);
  anchor_in_d_ = autoware::cuda_utils::make_unique<float[]>(anchor_size);
  confidence_in_d_ = autoware::cuda_utils::make_unique<float[]>(num_temp_instances_);
  valid_d_ = autoware::cuda_utils::make_unique<float[]>(1);
  time_interval_d_ = autoware::cuda_utils::make_unique<float[]>(1);
  prev2current_d_ = autoware::cuda_utils::make_unique<float[]>(transform_size);
  instance_feature_out_d_ = autoware::cuda_utils::make_unique<float[]>(feature_size);
  anchor_out_d_ = autoware::cuda_utils::make_unique<float[]>(anchor_size);
  confidence_out_d_ = autoware::cuda_utils::make_unique<float[]>(num_temp_instances_);
  frame_inputs_h_ = autoware::cuda_utils::make_unique_host<float[]>(
    transform_offset + transform_size, cudaHostAllocDefault);

  // The gate of the first frame is closed, the zeros only keep the inputs defined
  autoware::cuda_utils::clear_async(instance_feature_in_d_.get(), feature_size, stream_);
  autoware::cuda_utils::clear_async(anchor_in_d_.get(), anchor_size, stream_);
  autoware::cuda_utils::clear_async(confidence_in_d_.get(), num_temp_instances_, stream_);
}

bool Detection3DTemporalState::beginFrame(const StampedEgoPose & ego_pose)
{
  // The staging buffer may still feed the copies of the last frame
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
  pending_pose_ = ego_pose;

  const double gap =
    committed_pose_.has_value() ? ego_pose.stamp_seconds - committed_pose_->stamp_seconds : 0.0;
  const bool continues = gap > 0.0 && gap <= static_cast<double>(max_time_interval_);
  const Eigen::Isometry3d prev2current =
    continues ? ego_pose.lidar_to_world.inverse() * committed_pose_->lidar_to_world
              : Eigen::Isometry3d::Identity();

  frame_inputs_h_[valid_offset] = continues ? 1.0F : 0.0F;
  frame_inputs_h_[time_interval_offset] =
    continues ? static_cast<float>(gap) : default_time_interval_;
  const Eigen::Matrix4f prev2current_matrix = prev2current.matrix().cast<float>();
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      frame_inputs_h_[transform_offset + row * 4 + col] = prev2current_matrix(row, col);
    }
  }

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    valid_d_.get(), frame_inputs_h_.get() + valid_offset, sizeof(float), cudaMemcpyHostToDevice,
    stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    time_interval_d_.get(), frame_inputs_h_.get() + time_interval_offset, sizeof(float),
    cudaMemcpyHostToDevice, stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    prev2current_d_.get(), frame_inputs_h_.get() + transform_offset, transform_size * sizeof(float),
    cudaMemcpyHostToDevice, stream_));
  return continues;
}

void Detection3DTemporalState::commitFrame()
{
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    instance_feature_in_d_.get(), instance_feature_out_d_.get(),
    num_temp_instances_ * embed_dims_ * sizeof(float), cudaMemcpyDeviceToDevice, stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    anchor_in_d_.get(), anchor_out_d_.get(), num_temp_instances_ * anchor_dim * sizeof(float),
    cudaMemcpyDeviceToDevice, stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    confidence_in_d_.get(), confidence_out_d_.get(), num_temp_instances_ * sizeof(float),
    cudaMemcpyDeviceToDevice, stream_));
  committed_pose_ = pending_pose_;
}

}  // namespace autoware::ptv3
