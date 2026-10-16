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

#include "ptv3_test_fixture.hpp"

#include <Eigen/Geometry>

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

namespace autoware::ptv3
{
namespace test
{

constexpr std::size_t num_temp_instances{4};
constexpr std::size_t embed_dims{16};

PTv3Config makeTemporalConfig()
{
  PTv3ConfigParams params;
  params.use_seg3d_head = false;
  params.use_det3d_head = true;
  params.num_anchor = 8;
  params.num_temp_instances = num_temp_instances;
  params.embed_dims = embed_dims;
  params.default_time_interval = 0.1F;
  params.max_time_interval = 2.0F;
  return makeConfig(params);
}

StampedEgoPose makePose(const double stamp_seconds, const double x, const double yaw)
{
  Eigen::Isometry3d lidar_to_world = Eigen::Isometry3d::Identity();
  lidar_to_world.translate(Eigen::Vector3d(x, 0.0, 0.0));
  lidar_to_world.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  return StampedEgoPose{stamp_seconds, lidar_to_world};
}

class Detection3DTemporalStateTest : public PTv3CudaTest
{
protected:
  // The gate, the frame gap and the row-major transform the engine reads for the frame
  struct FrameInputs
  {
    float valid;
    float time_interval;
    std::vector<float> prev2current;
  };

  FrameInputs readFrameInputs(const Detection3DTemporalState & state)
  {
    EXPECT_EQ(cudaStreamSynchronize(stream_), cudaSuccess);
    return FrameInputs{
      copyToHost(state.validIn(), 1).front(), copyToHost(state.timeIntervalIn(), 1).front(),
      copyToHost(state.prev2currentIn(), 16)};
  }

  PTv3Config config_{makeTemporalConfig()};
};

TEST_F(Detection3DTemporalStateTest, TheFirstFrameStartsAStream)
{
  Detection3DTemporalState state(config_, stream_);

  EXPECT_FALSE(state.beginFrame(makePose(10.0, 0.0, 0.0)));

  const auto inputs = readFrameInputs(state);
  EXPECT_FLOAT_EQ(inputs.valid, 0.0F);
  EXPECT_FLOAT_EQ(inputs.time_interval, 0.1F);
  for (std::size_t index = 0; index < 16; ++index) {
    EXPECT_FLOAT_EQ(inputs.prev2current[index], index % 5 == 0 ? 1.0F : 0.0F);
  }
}

TEST_F(Detection3DTemporalStateTest, AContinuingFrameReadsTheEgoMotionSinceTheLastCommit)
{
  Detection3DTemporalState state(config_, stream_);
  state.beginFrame(makePose(10.0, 0.0, 0.0));
  state.commitFrame();

  // The ego drove 2 m forward and turned left by 90 degrees: the last frame origin lies 2 m on
  // the left of the new frame
  EXPECT_TRUE(state.beginFrame(makePose(10.25, 2.0, M_PI / 2.0)));

  const auto inputs = readFrameInputs(state);
  EXPECT_FLOAT_EQ(inputs.valid, 1.0F);
  EXPECT_FLOAT_EQ(inputs.time_interval, 0.25F);
  const auto & transform = inputs.prev2current;
  EXPECT_NEAR(transform[0 * 4 + 3], 0.0F, 1e-6F);
  EXPECT_NEAR(transform[1 * 4 + 3], 2.0F, 1e-6F);
  EXPECT_NEAR(transform[0 * 4 + 1], 1.0F, 1e-6F);
  EXPECT_NEAR(transform[1 * 4 + 0], -1.0F, 1e-6F);
}

TEST_F(Detection3DTemporalStateTest, AGapOutsideTheMemoryStartsAStream)
{
  Detection3DTemporalState state(config_, stream_);
  state.beginFrame(makePose(10.0, 0.0, 0.0));
  state.commitFrame();

  EXPECT_FALSE(state.beginFrame(makePose(12.5, 0.0, 0.0)));
  EXPECT_FLOAT_EQ(readFrameInputs(state).time_interval, 0.1F);
  EXPECT_FALSE(state.beginFrame(makePose(9.9, 0.0, 0.0)));
  EXPECT_FLOAT_EQ(readFrameInputs(state).valid, 0.0F);
}

TEST_F(Detection3DTemporalStateTest, AnUncommittedFrameLeavesTheLastCommitInPlace)
{
  Detection3DTemporalState state(config_, stream_);
  state.beginFrame(makePose(10.0, 0.0, 0.0));
  state.commitFrame();
  state.beginFrame(makePose(10.1, 0.0, 0.0));

  EXPECT_TRUE(state.beginFrame(makePose(10.2, 0.0, 0.0)));
  EXPECT_NEAR(readFrameInputs(state).time_interval, 0.2F, 1e-6F);
}

TEST_F(Detection3DTemporalStateTest, ACommitFeedsTheMemoryOutputsBack)
{
  Detection3DTemporalState state(config_, stream_);
  const std::vector<float> feature(num_temp_instances * embed_dims, 1.5F);
  const std::vector<float> anchor(num_temp_instances * Detection3DTemporalState::anchor_dim, 2.5F);
  const std::vector<float> confidence(num_temp_instances, 0.75F);
  state.beginFrame(makePose(10.0, 0.0, 0.0));
  ASSERT_EQ(cudaStreamSynchronize(stream_), cudaSuccess);
  copyToDevice(state.instanceFeatureOut(), feature);
  copyToDevice(state.anchorOut(), anchor);
  copyToDevice(state.confidenceOut(), confidence);

  state.commitFrame();
  ASSERT_EQ(cudaStreamSynchronize(stream_), cudaSuccess);

  EXPECT_EQ(copyToHost(state.instanceFeatureIn(), feature.size()), feature);
  EXPECT_EQ(copyToHost(state.anchorIn(), anchor.size()), anchor);
  EXPECT_EQ(copyToHost(state.confidenceIn(), confidence.size()), confidence);
}

}  // namespace test
}  // namespace autoware::ptv3
