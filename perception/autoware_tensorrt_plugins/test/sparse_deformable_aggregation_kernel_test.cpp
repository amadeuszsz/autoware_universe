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
#include "test_utils.hpp"

#include <autoware/cuda_utils/cuda_gtest_utils.hpp>

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <random>
#include <vector>

namespace
{

using autoware::sparse_deformable_aggregation_ops::sparse_deformable_aggregation_launch;
using autoware::sparse_deformable_aggregation_ops::SparseLevels;
using autoware::tensorrt_plugins::test::copy_to_device;
using autoware::tensorrt_plugins::test::copy_to_host;
using autoware::tensorrt_plugins::test::CudaStreamGuard;
using autoware::tensorrt_plugins::test::DeviceBuffer;

// One sampled stage on the host: features with a zero first row and the dense grid of feature rows
struct HostLevel
{
  std::array<std::int32_t, 3> dims;
  float voxel_size;
  std::vector<float> features;
  std::vector<std::int64_t> grid;
};

HostLevel make_level(
  const std::array<std::int32_t, 3> & dims, const float voxel_size, const std::int32_t channels,
  const float occupied_fraction, std::mt19937 & rng)
{
  HostLevel level{dims, voxel_size, {}, {}};
  const auto cells = static_cast<std::size_t>(dims[0]) * dims[1] * dims[2];
  level.grid.assign(cells, 0);
  level.features.assign(static_cast<std::size_t>(channels), 0.0F);
  std::uniform_real_distribution<float> unit(0.0F, 1.0F);
  std::normal_distribution<float> normal(0.0F, 1.0F);
  std::int64_t rows = 0;
  for (std::size_t cell = 0; cell < cells; ++cell) {
    if (unit(rng) < occupied_fraction) {
      level.grid[cell] = ++rows;
      for (std::int32_t channel = 0; channel < channels; ++channel) {
        level.features.push_back(normal(rng));
      }
    }
  }
  return level;
}

// Same math as the training head: trilinear corners, zeros on empty cells, the weighted sum divided
// by the weight on occupied voxels.
std::vector<float> reference(
  const std::vector<float> & key_points, const std::vector<float> & weights,
  const std::vector<HostLevel> & levels, const std::array<float, 3> & origin,
  const std::int32_t num_anchors, const std::int32_t num_points, const std::int32_t num_groups,
  const std::int32_t channels, const float min_occupancy)
{
  const auto num_levels = static_cast<std::int32_t>(levels.size());
  const std::int32_t num_samples = num_levels * num_points;
  const std::int32_t group_channels = channels / num_groups;
  std::vector<float> output(static_cast<std::size_t>(num_anchors) * channels, 0.0F);
  for (std::int32_t anchor = 0; anchor < num_anchors; ++anchor) {
    std::vector<float> sums(static_cast<std::size_t>(channels), 0.0F);
    std::vector<float> occupied(static_cast<std::size_t>(num_groups), 0.0F);
    for (std::int32_t sample = 0; sample < num_samples; ++sample) {
      const auto & level = levels[sample / num_points];
      const float * point = &key_points[(anchor * num_points + sample % num_points) * 3];
      std::array<float, 3> base{};
      std::array<float, 3> fraction{};
      for (std::int32_t axis = 0; axis < 3; ++axis) {
        const float continuous = (point[axis] - origin[axis]) / level.voxel_size - 0.5F;
        base[axis] = std::floor(continuous);
        fraction[axis] = continuous - base[axis];
      }
      for (std::int32_t corner = 0; corner < 8; ++corner) {
        const std::array<std::int32_t, 3> shift{(corner >> 2) & 1, (corner >> 1) & 1, corner & 1};
        float corner_weight = 1.0F;
        bool inside = true;
        std::array<std::int32_t, 3> cell{};
        for (std::int32_t axis = 0; axis < 3; ++axis) {
          corner_weight *= shift[axis] ? fraction[axis] : 1.0F - fraction[axis];
          cell[axis] = static_cast<std::int32_t>(base[axis]) + shift[axis];
          inside &= cell[axis] >= 0 && cell[axis] < level.dims[axis];
        }
        const std::int64_t row =
          inside
            ? level.grid
                [(static_cast<std::size_t>(cell[2]) * level.dims[1] + cell[1]) * level.dims[0] +
                 cell[0]]
            : 0;
        if (row == 0) {
          continue;
        }
        for (std::int32_t channel = 0; channel < channels; ++channel) {
          const float weight =
            weights[(anchor * num_samples + sample) * num_groups + channel / group_channels];
          sums[channel] += weight * corner_weight * level.features[row * channels + channel];
        }
        for (std::int32_t group = 0; group < num_groups; ++group) {
          occupied[group] +=
            weights[(anchor * num_samples + sample) * num_groups + group] * corner_weight;
        }
      }
    }
    for (std::int32_t channel = 0; channel < channels; ++channel) {
      output[anchor * channels + channel] =
        sums[channel] / std::max(occupied[channel / group_channels], min_occupancy);
    }
  }
  return output;
}

TEST(SparseDeformableAggregationKernelTest, MatchesCpuReference)
{
  SKIP_TEST_IF_CUDA_UNAVAILABLE();

  constexpr std::int32_t num_anchors = 37;
  constexpr std::int32_t num_points = 5;
  constexpr std::int32_t num_groups = 4;
  constexpr std::int32_t channels = 32;
  constexpr float min_occupancy = 0.05F;
  const std::array<float, 3> origin{-3.2F, -3.2F, -1.0F};
  std::mt19937 rng(7);

  std::vector<HostLevel> levels;
  levels.push_back(make_level({16, 16, 4}, 0.4F, channels, 0.3F, rng));
  levels.push_back(make_level({8, 8, 2}, 0.8F, channels, 0.5F, rng));
  const auto num_levels = static_cast<std::int32_t>(levels.size());

  // Keypoints spread past the grid border so that some corners fall outside
  std::uniform_real_distribution<float> xy(-3.6F, 3.6F);
  std::uniform_real_distribution<float> z(-1.2F, 0.8F);
  std::vector<float> key_points;
  for (std::int32_t i = 0; i < num_anchors * num_points; ++i) {
    key_points.insert(key_points.end(), {xy(rng), xy(rng), z(rng)});
  }
  std::uniform_real_distribution<float> unit(0.0F, 1.0F);
  std::vector<float> weights(
    static_cast<std::size_t>(num_anchors) * num_levels * num_points * num_groups);
  std::generate(weights.begin(), weights.end(), [&] { return unit(rng); });

  CudaStreamGuard stream;
  DeviceBuffer<float> key_points_d(key_points.size());
  DeviceBuffer<float> weights_d(weights.size());
  DeviceBuffer<float> output_d(static_cast<std::size_t>(num_anchors) * channels);
  copy_to_device(key_points_d.get(), key_points);
  copy_to_device(weights_d.get(), weights);

  std::vector<std::unique_ptr<DeviceBuffer<float>>> features_d;
  std::vector<std::unique_ptr<DeviceBuffer<std::int64_t>>> grids_d;
  SparseLevels sparse_levels{};
  sparse_levels.num_levels = num_levels;
  for (std::int32_t index = 0; index < num_levels; ++index) {
    const auto & level = levels[index];
    features_d.push_back(std::make_unique<DeviceBuffer<float>>(level.features.size()));
    grids_d.push_back(std::make_unique<DeviceBuffer<std::int64_t>>(level.grid.size()));
    copy_to_device(features_d.back()->get(), level.features);
    copy_to_device(grids_d.back()->get(), level.grid);
    sparse_levels.features[index] = features_d.back()->get();
    sparse_levels.grids[index] = grids_d.back()->get();
    std::copy(level.dims.begin(), level.dims.end(), sparse_levels.dims[index]);
    sparse_levels.voxel_sizes[index] = level.voxel_size;
  }

  ASSERT_EQ(
    sparse_deformable_aggregation_launch<float>(
      key_points_d.get(), weights_d.get(), sparse_levels, origin.data(), num_anchors, num_points,
      num_groups, channels, min_occupancy, output_d.get(), stream.get()),
    cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(stream.get()), cudaSuccess);

  const auto expected = reference(
    key_points, weights, levels, origin, num_anchors, num_points, num_groups, channels,
    min_occupancy);
  const auto actual = copy_to_host(output_d.get(), expected.size());
  for (std::size_t index = 0; index < expected.size(); ++index) {
    EXPECT_NEAR(actual[index], expected[index], 1e-4F * (1.0F + std::abs(expected[index])))
      << "index=" << index;
  }
}

}  // namespace
