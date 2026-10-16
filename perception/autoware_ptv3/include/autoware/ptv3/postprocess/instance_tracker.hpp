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

#ifndef AUTOWARE__PTV3__POSTPROCESS__INSTANCE_TRACKER_HPP_
#define AUTOWARE__PTV3__POSTPROCESS__INSTANCE_TRACKER_HPP_

#include "autoware/ptv3/utils.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace autoware::ptv3
{

/**
 * @brief Track ids of the Sparse4D instance memory.
 *
 * The temporal slots lead the queries of a frame, each slot keeps the track of the query it was
 * selected from in the previous frame. A query without a track starts a new one once its score
 * reaches the birth score. A stream restart forgets every track.
 */
class InstanceTracker
{
public:
  InstanceTracker(std::size_t num_anchor, std::size_t num_temp_instances, float track_birth_score);

  /**
   * @brief Give every query the track of its slot or a new track when it is confident enough.
   *
   * @param continues Whether the frame continues the stream of the last committed frame.
   * @param queries Boxes of every query in query order, their track ids are written.
   */
  void assign(bool continues, std::vector<Box3D> & queries);

  /**
   * @brief Roll the tracks of the last assigned frame into the temporal slots.
   *
   * @param state_query_index Query every temporal slot was selected from [num_temp_instances].
   */
  void commit(const std::int64_t * state_query_index);

private:
  std::size_t num_anchor_;
  float track_birth_score_;
  std::vector<std::int64_t> slot_track_ids_;
  std::vector<std::int64_t> query_track_ids_;
  std::int64_t next_track_id_{0};
};

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__POSTPROCESS__INSTANCE_TRACKER_HPP_
