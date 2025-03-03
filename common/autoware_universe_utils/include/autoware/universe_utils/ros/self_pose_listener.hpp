// Copyright 2020 Tier IV, Inc.
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

#ifndef AUTOWARE__UNIVERSE_UTILS__ROS__SELF_POSE_LISTENER_HPP_
#define AUTOWARE__UNIVERSE_UTILS__ROS__SELF_POSE_LISTENER_HPP_

#include "autoware/universe_utils/geometry/geometry.hpp"
#include "managed_transform_buffer/managed_transform_buffer.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace autoware::universe_utils
{
class SelfPoseListener
{
public:
  explicit SelfPoseListener(rclcpp::Node * const node) : node_(node), managed_tf_buffer_() {}

  void waitForFirstPose()
  {
    while (rclcpp::ok()) {
      if (getCurrentPose()) {
        return;
      }
      RCLCPP_INFO(node_->get_logger(), "waiting for self pose...");
      rclcpp::Rate(0.2).sleep();
    }
  }

  geometry_msgs::msg::PoseStamped::ConstSharedPtr getCurrentPose()
  {
    const auto tf = managed_tf_buffer_.getLatestTransform<geometry_msgs::msg::TransformStamped>(
      "map", "base_link", node_->get_logger());
    if (!tf) {
      return {};
    }

    return std::make_shared<const geometry_msgs::msg::PoseStamped>(transform2pose(*tf));
  }

private:
  rclcpp::Node * const node_;
  managed_transform_buffer::ManagedTransformBuffer managed_tf_buffer_;
};
}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__ROS__SELF_POSE_LISTENER_HPP_
