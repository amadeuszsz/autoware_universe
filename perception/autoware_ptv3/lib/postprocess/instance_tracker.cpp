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

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::ptv3
{

InstanceTracker::InstanceTracker(
  const std::size_t num_anchor, const std::size_t num_temp_instances, const float track_birth_score)
: num_anchor_(num_anchor),
  track_birth_score_(track_birth_score),
  slot_track_ids_(num_temp_instances, -1),
  query_track_ids_(num_anchor, -1)
{
  if (num_temp_instances == 0 || num_temp_instances >= num_anchor) {
    throw std::runtime_error("InstanceTracker needs 0 < num_temp_instances < num_anchor.");
  }
}

void InstanceTracker::assign(const bool continues, std::vector<Box3D> & queries)
{
  if (queries.size() != num_anchor_) {
    throw std::runtime_error(
      "InstanceTracker expects " + std::to_string(num_anchor_) + " queries, got " +
      std::to_string(queries.size()) + ".");
  }
  if (!continues) {
    std::fill(slot_track_ids_.begin(), slot_track_ids_.end(), -1);
  }
  std::fill(query_track_ids_.begin(), query_track_ids_.end(), -1);
  std::copy(slot_track_ids_.begin(), slot_track_ids_.end(), query_track_ids_.begin());
  for (std::size_t query = 0; query < num_anchor_; ++query) {
    if (query_track_ids_[query] < 0 && queries[query].score >= track_birth_score_) {
      query_track_ids_[query] = next_track_id_++;
    }
    queries[query].track_id = query_track_ids_[query];
  }
}

void InstanceTracker::commit(const std::int64_t * state_query_index)
{
  for (std::size_t slot = 0; slot < slot_track_ids_.size(); ++slot) {
    const auto query = state_query_index[slot];
    if (query < 0 || static_cast<std::size_t>(query) >= num_anchor_) {
      throw std::runtime_error(
        "The detection head selected query " + std::to_string(query) + " for a temporal slot.");
    }
    slot_track_ids_[slot] = query_track_ids_[query];
  }
}

}  // namespace autoware::ptv3
