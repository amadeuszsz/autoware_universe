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

#ifndef AUTOWARE__PTV3__POSTPROCESS__DETECTION3D_POSTPROCESS_HPP_
#define AUTOWARE__PTV3__POSTPROCESS__DETECTION3D_POSTPROCESS_HPP_

#include "autoware/ptv3/ptv3_config.hpp"
#include "autoware/ptv3/utils.hpp"

#include <cstddef>
#include <vector>

namespace autoware::ptv3
{

/**
 * @brief Host decoding of the Sparse4D detection head outputs.
 *
 * Every query is one instance. Its label is its best class and its score is that class
 * probability times the query centerness.
 */
class Detection3DPostprocess
{
public:
  /// Anchor state channels: [x, y, z, ln l, ln w, ln h, sin yaw, cos yaw, vx, vy].
  static constexpr std::size_t anchor_dim{10};
  /// Quality channels: [centerness, yawness] logits.
  static constexpr std::size_t quality_dim{2};

  explicit Detection3DPostprocess(const PTv3Config & config);

  /**
   * @brief Decode every query into the box of its best class.
   *
   * @param classification Class logits [num_anchor, num_classes].
   * @param prediction Anchor states [num_anchor, anchor_dim].
   * @param quality Quality logits [num_anchor, quality_dim].
   * @return One box per query in query order, without a track.
   */
  [[nodiscard]] std::vector<Box3D> decodeQueries(
    const float * classification, const float * prediction, const float * quality) const;

  /**
   * @brief Keep the boxes reaching the score threshold of their class and distance, then
   * suppress the boxes of every circle NMS group closer than its radius to a better box.
   *
   * @param boxes Decoded boxes.
   * @return Kept boxes by descending score.
   */
  [[nodiscard]] std::vector<Box3D> select(std::vector<Box3D> boxes) const;

private:
  /// Score threshold of the box class at the box distance, above every score past the last bin.
  [[nodiscard]] float scoreThreshold(const Box3D & box) const;

  PTv3Config config_;
};

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__POSTPROCESS__DETECTION3D_POSTPROCESS_HPP_
