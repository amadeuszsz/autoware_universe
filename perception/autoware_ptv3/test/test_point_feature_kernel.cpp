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
#include "ptv3_test_fixture.hpp"

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <vector>

namespace autoware::ptv3
{
namespace test
{

constexpr std::size_t kNumFeatures = 4;
constexpr std::uint32_t kThreadsPerBlock = 256;

class PointFeatureKernelTest : public PTv3CudaTest
{
protected:
  std::vector<float> runPointFeatures(
    const void * input_points_d, const CloudFormat format, const std::size_t num_points)
  {
    // One sentinel past the rows shows that nothing is written beyond them.
    auto output_d = makeDeviceBuffer<float>(num_points * kNumFeatures + 1);
    copyToDevice(output_d.get(), std::vector<float>(num_points * kNumFeatures + 1, kSentinel));

    generatePointFeaturesLaunch(
      input_points_d, format, num_points, output_d.get(), kThreadsPerBlock, stream_);
    EXPECT_EQ(cudaStreamSynchronize(stream_), cudaSuccess);

    return copyToHost(output_d.get(), num_points * kNumFeatures + 1);
  }

  static constexpr float kSentinel = 42.0F;
};

TEST_F(PointFeatureKernelTest, WritesPackedRowsWithRawFloatIntensity)
{
  const std::vector<CloudPointTypeXYZI> points{
    {1.0F, 0.0F, 0.0F, 10.0F},
    {0.0F, 2.0F, -1.0F, 20.0F},
  };
  auto points_d = makeDeviceBuffer<CloudPointTypeXYZI>(points.size());
  copyToDevice(points_d.get(), points);

  const auto output = runPointFeatures(points_d.get(), CloudFormat::XYZI, points.size());

  // XYZI carries float intensity, consumed raw.
  const std::vector<float> expected{
    1.0F,      0.0F, 0.0F,  10.0F,  //
    0.0F,      2.0F, -1.0F, 20.0F,  //
    kSentinel,
  };
  ASSERT_EQ(output.size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_FLOAT_EQ(output[i], expected[i]) << "at index " << i;
  }
}

TEST_F(PointFeatureKernelTest, NormalizesEightBitIntensity)
{
  const std::vector<CloudPointTypeXYZIRC> points{
    {1.0F, 0.0F, 0.0F, 255U, 0U, 0U},
    {0.0F, 2.0F, 0.0F, 51U, 0U, 0U},
  };
  auto points_d = makeDeviceBuffer<CloudPointTypeXYZIRC>(points.size());
  copyToDevice(points_d.get(), points);

  const auto output = runPointFeatures(points_d.get(), CloudFormat::XYZIRC, points.size());

  // XYZIRC carries 8-bit intensity, normalized to [0, 1] like the training strength.
  EXPECT_FLOAT_EQ(output[3], 1.0F);
  EXPECT_FLOAT_EQ(output[kNumFeatures + 1], 2.0F);
  EXPECT_FLOAT_EQ(output[kNumFeatures + 3], 0.2F);
}

TEST_F(PointFeatureKernelTest, SkipsEmptyFrames)
{
  generatePointFeaturesLaunch(nullptr, CloudFormat::XYZI, 0, nullptr, kThreadsPerBlock, stream_);

  EXPECT_EQ(cudaStreamSynchronize(stream_), cudaSuccess);
  EXPECT_EQ(cudaGetLastError(), cudaSuccess);
}

}  // namespace test
}  // namespace autoware::ptv3
