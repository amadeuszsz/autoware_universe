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

#include "autoware/ptv3/preprocess/point_feature_kernel.hpp"
#include "autoware/ptv3/preprocess/point_type.hpp"
#include "autoware/ptv3/utils.hpp"

#include <stdexcept>

namespace autoware::ptv3
{
namespace
{

// Intensity normalization must match training: the 8-bit intensity formats are
// scaled to [0, 1]; the float intensity formats are consumed raw.
__device__ float normalizedIntensity(const CloudPointTypeXYZIRCAEDT & point)
{
  return static_cast<float>(point.intensity) / 255.f;
}

__device__ float normalizedIntensity(const CloudPointTypeXYZIRC & point)
{
  return static_cast<float>(point.intensity) / 255.f;
}

__device__ float normalizedIntensity(const CloudPointTypeXYZIRADRT & point)
{
  return point.intensity;
}

__device__ float normalizedIntensity(const CloudPointTypeXYZI & point)
{
  return point.intensity;
}

constexpr std::size_t num_point_features{4};

template <typename PointT>
__global__ void generatePointFeaturesKernel(
  const PointT * __restrict__ input_points, std::size_t points_size,
  float * __restrict__ output_points)
{
  const auto idx = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (idx >= points_size) {
    return;
  }

  const PointT & input_point = input_points[idx];
  float * output_point = &output_points[idx * num_point_features];
  output_point[0] = input_point.x;
  output_point[1] = input_point.y;
  output_point[2] = input_point.z;
  output_point[3] = normalizedIntensity(input_point);
}

}  // namespace

void generatePointFeaturesLaunch(
  const void * input_data, CloudFormat input_format, std::size_t num_points, float * output_points,
  std::uint32_t threads_per_block, cudaStream_t stream)
{
  // A zero-sized grid is an invalid CUDA launch; an empty frame contributes nothing.
  if (num_points == 0) {
    return;
  }
  const auto num_blocks = divup(num_points, threads_per_block);
  switch (input_format) {
    case CloudFormat::XYZIRCAEDT:
      generatePointFeaturesKernel<<<num_blocks, threads_per_block, 0, stream>>>(
        static_cast<const CloudPointTypeXYZIRCAEDT *>(input_data), num_points, output_points);
      break;
    case CloudFormat::XYZIRADRT:
      generatePointFeaturesKernel<<<num_blocks, threads_per_block, 0, stream>>>(
        static_cast<const CloudPointTypeXYZIRADRT *>(input_data), num_points, output_points);
      break;
    case CloudFormat::XYZIRC:
      generatePointFeaturesKernel<<<num_blocks, threads_per_block, 0, stream>>>(
        static_cast<const CloudPointTypeXYZIRC *>(input_data), num_points, output_points);
      break;
    case CloudFormat::XYZI:
      generatePointFeaturesKernel<<<num_blocks, threads_per_block, 0, stream>>>(
        static_cast<const CloudPointTypeXYZI *>(input_data), num_points, output_points);
      break;
    default:
      throw std::runtime_error("Unsupported input point cloud format.");
  }
}

}  // namespace autoware::ptv3
