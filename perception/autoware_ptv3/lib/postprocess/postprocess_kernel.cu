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

#include "autoware/ptv3/postprocess/postprocess_kernel.hpp"
#include "autoware/ptv3/utils.hpp"

#include <thrust/device_ptr.h>
#include <thrust/execution_policy.h>
#include <thrust/scan.h>

namespace autoware::ptv3
{

__global__ void paintPointcloudKernel(
  const float * input_features, const float * colors, const std::int64_t * labels,
  float4 * output_points, std::size_t num_points)
{
  const auto idx = static_cast<std::uint32_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (idx >= num_points) {
    return;
  }

  constexpr int kFeatDim = 9;
  const auto label = labels[idx];
  const auto color = colors[label];

  output_points[idx] = make_float4(
    input_features[kFeatDim * idx + 0], input_features[kFeatDim * idx + 1],
    input_features[kFeatDim * idx + 2], color);
}

// cSpell:ignore Probs probs
__global__ void createProbsPointcloudKernel(
  const float * input_features, const float * pred_probs, float * output_points,
  std::size_t num_classes, std::size_t num_points)
{
  const auto idx = static_cast<std::uint32_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (idx >= num_points) {
    return;
  }

  constexpr int kFeatDim = 9;
  const float * point_probs = &pred_probs[num_classes * idx];
  float * output_point = &output_points[(3 + num_classes) * idx];

  output_point[0] = input_features[kFeatDim * idx + 0];
  output_point[1] = input_features[kFeatDim * idx + 1];
  output_point[2] = input_features[kFeatDim * idx + 2];

  for (std::size_t i = 0; i < num_classes; ++i) {
    output_point[3 + i] = point_probs[i];
  }
}

__global__ void computeGroundSegmentationMask(
  const std::int64_t * pred_labels, const float * pred_probs, std::uint32_t * not_ground_mask,
  const int ground_label, const float ground_prob_threshold, std::size_t num_classes,
  std::size_t num_points)
{
  const auto idx = static_cast<std::uint32_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (idx >= num_points) {
    return;
  }

  const std::int64_t label = pred_labels[idx];
  const float ground_prob = pred_probs[num_classes * idx + ground_label];

  not_ground_mask[idx] = (label != ground_label) && (ground_prob < ground_prob_threshold);
}

__global__ void scatterNonGroundPointsKernel(
  const float * __restrict__ input_features, const std::uint32_t * __restrict__ mask,
  const std::uint32_t * __restrict__ indices, float4 * __restrict__ output_points,
  std::size_t num_points)
{
  const auto idx = static_cast<std::uint32_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (idx >= num_points) {
    return;
  }

  if (mask[idx] != 0) {
    constexpr int kFeatDim = 9;
    const std::uint32_t out_idx = indices[idx] - 1;
    output_points[out_idx] = make_float4(
      input_features[kFeatDim * idx + 0], input_features[kFeatDim * idx + 1],
      input_features[kFeatDim * idx + 2], 0.0f);
  }
}

PostprocessCuda::PostprocessCuda(const PTv3Config & config, cudaStream_t stream)
: config_(config), stream_(stream)
{
  ground_mask_d_ = autoware::cuda_utils::make_unique<std::uint32_t[]>(config_.max_num_voxels_);
  ground_indices_d_ = autoware::cuda_utils::make_unique<std::uint32_t[]>(config_.max_num_voxels_);

  color_map_d_ = autoware::cuda_utils::make_unique<float[]>(config_.colors_rgb_.size());
  cudaMemcpyAsync(
    color_map_d_.get(), config_.colors_rgb_.data(), config_.colors_rgb_.size() * sizeof(float),
    cudaMemcpyHostToDevice, stream_);

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
}

void PostprocessCuda::paintPointcloud(
  const float * input_features, const std::int64_t * labels, float * output_points,
  std::size_t num_points)
{
  auto num_blocks = divup(num_points, config_.threads_per_block_);

  paintPointcloudKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
    input_features, color_map_d_.get(), labels, reinterpret_cast<float4 *>(output_points),
    num_points);

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
}

void PostprocessCuda::createProbsPointcloud(
  const float * input_features, const float * pred_probs, float * output_points,
  std::size_t num_classes, std::size_t num_points)
{
  auto num_blocks = divup(num_points, config_.threads_per_block_);

  createProbsPointcloudKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
    input_features, pred_probs, output_points, num_classes, num_points);

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
}

std::size_t PostprocessCuda::createGroundSegmentedPointcloud(
  const float * input_features, const std::int64_t * pred_labels, const float * pred_probs,
  float * output_points, const int ground_label, const float ground_prob_threshold,
  std::size_t num_classes, std::size_t num_points)
{
  auto num_blocks = divup(num_points, config_.threads_per_block_);

  computeGroundSegmentationMask<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
    pred_labels, pred_probs, ground_mask_d_.get(), ground_label, ground_prob_threshold, num_classes,
    num_points);

  auto policy = thrust::cuda::par.on(stream_);

  thrust::inclusive_scan(
    policy, ground_mask_d_.get(), ground_mask_d_.get() + num_points, ground_indices_d_.get());

  std::uint32_t num_non_ground_points;
  cudaMemcpyAsync(
    &num_non_ground_points, ground_indices_d_.get() + num_points - 1, sizeof(std::uint32_t),
    cudaMemcpyDeviceToHost, stream_);
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  if (num_non_ground_points == 0) {
    return 0;
  }

  scatterNonGroundPointsKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
    input_features, ground_mask_d_.get(), ground_indices_d_.get(),
    reinterpret_cast<float4 *>(output_points), num_points);

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  return num_non_ground_points;
}

}  // namespace autoware::ptv3
