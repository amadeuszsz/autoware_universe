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

#include "autoware/ptv3/postprocess/detection3d_postprocess.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace autoware::ptv3
{

namespace
{

float sigmoid(const float logit)
{
  return 1.0F / (1.0F + std::exp(-logit));
}

}  // namespace

Detection3DPostprocess::Detection3DPostprocess(const PTv3Config & config) : config_(config)
{
}

std::vector<Box3D> Detection3DPostprocess::decodeQueries(
  const float * classification, const float * prediction, const float * quality) const
{
  const auto num_classes = config_.detection_class_names_.size();
  std::vector<Box3D> boxes(config_.num_anchor_);
  for (std::size_t query = 0; query < config_.num_anchor_; ++query) {
    // The sigmoid is monotonic, so the best logit is the best class probability
    const float * logits = classification + query * num_classes;
    const auto best_class = std::max_element(logits, logits + num_classes) - logits;
    const float * state = prediction + query * anchor_dim;

    Box3D & box = boxes[query];
    box.label = static_cast<int>(best_class);
    box.score = sigmoid(logits[best_class]) * sigmoid(quality[query * quality_dim]);
    box.x = state[0];
    box.y = state[1];
    box.z = state[2];
    box.length = std::exp(state[3]);
    box.width = std::exp(state[4]);
    box.height = std::exp(state[5]);
    box.yaw = std::atan2(state[6], state[7]);
    box.vel_x = state[8];
    box.vel_y = state[9];
  }
  return boxes;
}

float Detection3DPostprocess::scoreThreshold(const Box3D & box) const
{
  const auto num_classes = config_.detection_class_names_.size();
  const float distance = std::hypot(box.x, box.y);
  const auto & limits = config_.distance_bin_upper_limits_;
  const auto bin = std::upper_bound(limits.begin(), limits.end(), distance) - limits.begin();
  if (static_cast<std::size_t>(bin) == limits.size()) {
    return std::numeric_limits<float>::infinity();
  }
  return config_.detection_score_thresholds_[bin * num_classes + box.label];
}

std::vector<Box3D> Detection3DPostprocess::select(std::vector<Box3D> boxes) const
{
  boxes.erase(
    std::remove_if(
      boxes.begin(), boxes.end(),
      [this](const Box3D & box) { return box.score < scoreThreshold(box); }),
    boxes.end());
  std::stable_sort(boxes.begin(), boxes.end(), [](const Box3D & lhs, const Box3D & rhs) {
    return lhs.score > rhs.score;
  });

  // Greedy circle NMS within every group, a box is checked against the better boxes kept so far
  std::vector<Box3D> kept;
  kept.reserve(boxes.size());
  for (auto & box : boxes) {
    const auto group = config_.circle_nms_class_groups_[box.label];
    const bool suppressed =
      group >= 0 && std::any_of(kept.begin(), kept.end(), [this, &box, group](const Box3D & other) {
        return config_.circle_nms_class_groups_[other.label] == group &&
               std::hypot(box.x - other.x, box.y - other.y) <= config_.circle_nms_radii_[group];
      });
    if (!suppressed) {
      kept.push_back(std::move(box));
    }
  }
  return kept;
}

}  // namespace autoware::ptv3
