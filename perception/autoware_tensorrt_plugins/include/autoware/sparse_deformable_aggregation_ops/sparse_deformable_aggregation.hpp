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

#ifndef AUTOWARE__SPARSE_DEFORMABLE_AGGREGATION_OPS__SPARSE_DEFORMABLE_AGGREGATION_HPP_
#define AUTOWARE__SPARSE_DEFORMABLE_AGGREGATION_OPS__SPARSE_DEFORMABLE_AGGREGATION_HPP_

#include <cuda_runtime.h>

#include <cstdint>

namespace autoware::sparse_deformable_aggregation_ops
{

constexpr std::int32_t kMaxLevels = 4;

// Sampled sparse encoder stages: the features of every stage with a zero first row, and the dense
// grid that holds the feature row of every cell (0 for an empty cell).
struct SparseLevels
{
  const void * features[kMaxLevels];
  const std::int64_t * grids[kMaxLevels];
  std::int32_t dims[kMaxLevels][3];
  float voxel_sizes[kMaxLevels];
  std::int32_t num_levels;
};

// Aggregate the features of every anchor: every keypoint interpolates the 8 voxels around it on
// every level, the samples are summed with their weights and divided by the weight that landed
// on occupied voxels (at least min_occupancy).
template <typename T>
cudaError_t sparse_deformable_aggregation_launch(
  const float * key_points, const float * weights, const SparseLevels & levels,
  const float grid_origin[3], std::int32_t num_anchors, std::int32_t num_points,
  std::int32_t num_groups, std::int32_t channels, float min_occupancy, T * output,
  cudaStream_t stream);

}  // namespace autoware::sparse_deformable_aggregation_ops

#endif  // AUTOWARE__SPARSE_DEFORMABLE_AGGREGATION_OPS__SPARSE_DEFORMABLE_AGGREGATION_HPP_
