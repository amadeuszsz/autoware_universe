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

#include "autoware/sparse_deformable_aggregation_ops/sparse_deformable_aggregation.hpp"

#include <cuda_fp16.h>

#include <cstdint>

namespace autoware::sparse_deformable_aggregation_ops
{
namespace
{

constexpr std::int32_t kCorners = 8;
constexpr std::int32_t kThreads = 256;

__device__ inline float to_float(float value)
{
  return value;
}

__device__ inline float to_float(half value)
{
  return __half2float(value);
}

template <typename T>
__device__ inline T from_float(float value);

template <>
__device__ inline float from_float<float>(float value)
{
  return value;
}

template <>
__device__ inline half from_float<half>(float value)
{
  return __float2half(value);
}

// One block per anchor. The block first resolves the voxel row and the interpolation weight of the
// 8 corners of every sample, then every thread accumulates one channel over all samples.
template <typename T>
__global__ void sparse_deformable_aggregation_kernel(
  const float * key_points, const float * weights, SparseLevels levels, float origin_x,
  float origin_y, float origin_z, std::int32_t num_points, std::int32_t num_groups,
  std::int32_t channels, float min_occupancy, T * output)
{
  extern __shared__ unsigned char shared[];
  const std::int32_t num_samples = levels.num_levels * num_points;
  auto * rows = reinterpret_cast<std::int32_t *>(shared);
  auto * corner_weights = reinterpret_cast<float *>(rows + num_samples * kCorners);
  auto * sample_weights = corner_weights + num_samples * kCorners;

  const std::int64_t anchor = blockIdx.x;
  for (std::int32_t i = threadIdx.x; i < num_samples * kCorners; i += blockDim.x) {
    const std::int32_t sample = i / kCorners;
    const std::int32_t corner = i % kCorners;
    const std::int32_t level = sample / num_points;
    const std::int32_t point = sample % num_points;
    const float * key_point = key_points + (anchor * num_points + point) * 3;
    const float voxel_size = levels.voxel_sizes[level];
    const float cx = (key_point[0] - origin_x) / voxel_size - 0.5f;
    const float cy = (key_point[1] - origin_y) / voxel_size - 0.5f;
    const float cz = (key_point[2] - origin_z) / voxel_size - 0.5f;
    const float bx = floorf(cx);
    const float by = floorf(cy);
    const float bz = floorf(cz);
    const std::int32_t shift_x = (corner >> 2) & 1;
    const std::int32_t shift_y = (corner >> 1) & 1;
    const std::int32_t shift_z = corner & 1;
    const float fx = cx - bx;
    const float fy = cy - by;
    const float fz = cz - bz;
    corner_weights[i] =
      (shift_x ? fx : 1.0f - fx) * (shift_y ? fy : 1.0f - fy) * (shift_z ? fz : 1.0f - fz);
    const std::int32_t x = static_cast<std::int32_t>(bx) + shift_x;
    const std::int32_t y = static_cast<std::int32_t>(by) + shift_y;
    const std::int32_t z = static_cast<std::int32_t>(bz) + shift_z;
    const std::int32_t size_x = levels.dims[level][0];
    const std::int32_t size_y = levels.dims[level][1];
    const std::int32_t size_z = levels.dims[level][2];
    const bool inside = x >= 0 && x < size_x && y >= 0 && y < size_y && z >= 0 && z < size_z;
    rows[i] = inside
                ? static_cast<std::int32_t>(
                    levels.grids[level][(static_cast<std::int64_t>(z) * size_y + y) * size_x + x])
                : 0;
  }
  const float * anchor_weights = weights + anchor * num_samples * num_groups;
  for (std::int32_t i = threadIdx.x; i < num_samples * num_groups; i += blockDim.x) {
    sample_weights[i] = anchor_weights[i];
  }
  __syncthreads();

  const std::int32_t group_channels = channels / num_groups;
  for (std::int32_t channel = threadIdx.x; channel < channels; channel += blockDim.x) {
    const std::int32_t group = channel / group_channels;
    float sum = 0.0f;
    float occupied = 0.0f;
    for (std::int32_t sample = 0; sample < num_samples; ++sample) {
      const T * features = static_cast<const T *>(levels.features[sample / num_points]);
      float sample_sum = 0.0f;
      float sample_occupied = 0.0f;
      for (std::int32_t corner = 0; corner < kCorners; ++corner) {
        const std::int32_t row = rows[sample * kCorners + corner];
        if (row != 0) {
          const float weight = corner_weights[sample * kCorners + corner];
          sample_sum +=
            weight * to_float(features[static_cast<std::int64_t>(row) * channels + channel]);
          sample_occupied += weight;
        }
      }
      const float weight = sample_weights[sample * num_groups + group];
      sum += weight * sample_sum;
      occupied += weight * sample_occupied;
    }
    output[anchor * channels + channel] = from_float<T>(sum / fmaxf(occupied, min_occupancy));
  }
}

}  // namespace

template <typename T>
cudaError_t sparse_deformable_aggregation_launch(
  const float * key_points, const float * weights, const SparseLevels & levels,
  const float grid_origin[3], std::int32_t num_anchors, std::int32_t num_points,
  std::int32_t num_groups, std::int32_t channels, float min_occupancy, T * output,
  cudaStream_t stream)
{
  const std::int32_t num_samples = levels.num_levels * num_points;
  const std::size_t shared_bytes = num_samples * kCorners * (sizeof(std::int32_t) + sizeof(float)) +
                                   num_samples * num_groups * sizeof(float);
  sparse_deformable_aggregation_kernel<T><<<num_anchors, kThreads, shared_bytes, stream>>>(
    key_points, weights, levels, grid_origin[0], grid_origin[1], grid_origin[2], num_points,
    num_groups, channels, min_occupancy, output);
  return cudaGetLastError();
}

template cudaError_t sparse_deformable_aggregation_launch<float>(
  const float *, const float *, const SparseLevels &, const float[3], std::int32_t, std::int32_t,
  std::int32_t, std::int32_t, float, float *, cudaStream_t);
template cudaError_t sparse_deformable_aggregation_launch<half>(
  const float *, const float *, const SparseLevels &, const float[3], std::int32_t, std::int32_t,
  std::int32_t, std::int32_t, float, half *, cudaStream_t);

}  // namespace autoware::sparse_deformable_aggregation_ops
