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

#include "autoware/ptv3/postprocess/instance_tracker.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace autoware::ptv3
{
namespace test
{

// Queries of the given scores, without a track
std::vector<Box3D> makeQueries(const std::vector<float> & scores)
{
  std::vector<Box3D> queries(scores.size());
  for (std::size_t query = 0; query < scores.size(); ++query) {
    queries[query].score = scores[query];
  }
  return queries;
}

std::vector<std::int64_t> trackIds(const std::vector<Box3D> & queries)
{
  std::vector<std::int64_t> ids;
  for (const auto & query : queries) {
    ids.push_back(query.track_id);
  }
  return ids;
}

TEST(InstanceTrackerTest, AConfidentQueryStartsATrack)
{
  InstanceTracker tracker(4, 2, 0.5F);
  auto queries = makeQueries({0.6F, 0.1F, 0.7F, 0.4F});

  tracker.assign(true, queries);

  EXPECT_EQ(trackIds(queries), (std::vector<std::int64_t>{0, -1, 1, -1}));
}

TEST(InstanceTrackerTest, ASlotKeepsTheTrackOfTheQueryItWasSelectedFrom)
{
  InstanceTracker tracker(4, 2, 0.5F);
  auto first = makeQueries({0.6F, 0.1F, 0.7F, 0.4F});
  tracker.assign(true, first);
  const std::vector<std::int64_t> state_query_index{2, 0};
  tracker.commit(state_query_index.data());

  // The slots keep their tracks at any score, a fresh query starts the next track
  auto second = makeQueries({0.1F, 0.1F, 0.9F, 0.1F});
  tracker.assign(true, second);

  EXPECT_EQ(trackIds(second), (std::vector<std::int64_t>{1, 0, 2, -1}));
}

TEST(InstanceTrackerTest, AStreamRestartForgetsEveryTrack)
{
  InstanceTracker tracker(4, 2, 0.5F);
  auto first = makeQueries({0.6F, 0.6F, 0.1F, 0.1F});
  tracker.assign(true, first);
  const std::vector<std::int64_t> state_query_index{0, 1};
  tracker.commit(state_query_index.data());

  auto second = makeQueries({0.1F, 0.6F, 0.1F, 0.1F});
  tracker.assign(false, second);

  EXPECT_EQ(trackIds(second), (std::vector<std::int64_t>{-1, 2, -1, -1}));
}

TEST(InstanceTrackerTest, RejectsQueriesTheMemoryDoesNotHold)
{
  InstanceTracker tracker(4, 2, 0.5F);
  auto queries = makeQueries({0.1F, 0.1F, 0.1F});
  EXPECT_THROW(tracker.assign(true, queries), std::runtime_error);

  auto all_queries = makeQueries({0.1F, 0.1F, 0.1F, 0.1F});
  tracker.assign(true, all_queries);
  const std::vector<std::int64_t> state_query_index{0, 4};
  EXPECT_THROW(tracker.commit(state_query_index.data()), std::runtime_error);
}

}  // namespace test
}  // namespace autoware::ptv3
