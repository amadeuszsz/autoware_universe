// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__OBJECT_RECOGNITION_UTILS__TRANSFORM_HPP_
#define AUTOWARE__OBJECT_RECOGNITION_UTILS__TRANSFORM_HPP_

#include <managed_transform_buffer/managed_transform_buffer.hpp>
#include <pcl_ros/transforms.hpp>

#include <geometry_msgs/msg/transform.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <boost/optional.hpp>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <string>

namespace detail
{
[[maybe_unused]] inline boost::optional<geometry_msgs::msg::Transform> getTransform(
  managed_transform_buffer::ManagedTransformBuffer & managed_tf_buffer,
  const std::string & source_frame_id, const std::string & target_frame_id,
  const rclcpp::Time & time)
{
  const auto transform_opt = managed_tf_buffer.getTransform<geometry_msgs::msg::TransformStamped>(
    target_frame_id, source_frame_id, time, rclcpp::Duration::from_seconds(0.5),
    rclcpp::get_logger("object_recognition_utils"));
  if (!transform_opt) return boost::none;
  return transform_opt->transform;
}

[[maybe_unused]] inline boost::optional<Eigen::Matrix4f> getTransformMatrix(
  const std::string & in_target_frame, const std_msgs::msg::Header & in_cloud_header,
  managed_transform_buffer::ManagedTransformBuffer & managed_tf_buffer)
{
  const auto transform_opt = managed_tf_buffer.getTransform<Eigen::Matrix4f>(
    in_target_frame, in_cloud_header.frame_id, in_cloud_header.stamp,
    rclcpp::Duration::from_seconds(1.0), rclcpp::get_logger("object_recognition_utils"));
  if (!transform_opt) return boost::none;
  return *transform_opt;
}
}  // namespace detail

namespace autoware::object_recognition_utils
{
template <class T>
bool transformObjects(
  const T & input_msg, const std::string & target_frame_id,
  managed_transform_buffer::ManagedTransformBuffer & managed_tf_buffer, T & output_msg)
{
  output_msg = input_msg;

  // transform to world coordinate
  if (input_msg.header.frame_id != target_frame_id) {
    output_msg.header.frame_id = target_frame_id;
    tf2::Transform tf_target2objects_world;
    tf2::Transform tf_target2objects;
    tf2::Transform tf_objects_world2objects;
    {
      const auto ros_target2objects_world = detail::getTransform(
        managed_tf_buffer, input_msg.header.frame_id, target_frame_id, input_msg.header.stamp);
      if (!ros_target2objects_world) {
        return false;
      }
      tf2::fromMsg(*ros_target2objects_world, tf_target2objects_world);
    }
    for (auto & object : output_msg.objects) {
      auto & pose_with_cov = object.kinematics.pose_with_covariance;
      tf2::fromMsg(pose_with_cov.pose, tf_objects_world2objects);
      tf_target2objects = tf_target2objects_world * tf_objects_world2objects;
      // transform pose, frame difference and object pose
      tf2::toMsg(tf_target2objects, pose_with_cov.pose);
      // transform covariance, only the frame difference
      pose_with_cov.covariance =
        tf2::transformCovariance(pose_with_cov.covariance, tf_target2objects_world);
    }
  }
  return true;
}
template <class T>
bool transformObjectsWithFeature(
  const T & input_msg, const std::string & target_frame_id,
  managed_transform_buffer::ManagedTransformBuffer & managed_tf_buffer, T & output_msg)
{
  output_msg = input_msg;
  if (input_msg.header.frame_id != target_frame_id) {
    output_msg.header.frame_id = target_frame_id;
    tf2::Transform tf_target2objects_world;
    tf2::Transform tf_target2objects;
    tf2::Transform tf_objects_world2objects;
    const auto ros_target2objects_world = detail::getTransform(
      managed_tf_buffer, input_msg.header.frame_id, target_frame_id, input_msg.header.stamp);
    if (!ros_target2objects_world) {
      return false;
    }
    const auto tf_matrix =
      detail::getTransformMatrix(target_frame_id, input_msg.header, managed_tf_buffer);
    if (!tf_matrix) {
      RCLCPP_WARN(
        rclcpp::get_logger("object_recognition_utils:"), "failed to get transformed matrix");
      return false;
    }
    for (auto & feature_object : output_msg.feature_objects) {
      // transform object
      tf2::fromMsg(
        feature_object.object.kinematics.pose_with_covariance.pose, tf_objects_world2objects);
      tf_target2objects = tf_target2objects_world * tf_objects_world2objects;
      tf2::toMsg(tf_target2objects, feature_object.object.kinematics.pose_with_covariance.pose);

      // transform cluster
      sensor_msgs::msg::PointCloud2 transformed_cluster;
      pcl_ros::transformPointCloud(*tf_matrix, feature_object.feature.cluster, transformed_cluster);
      transformed_cluster.header.frame_id = target_frame_id;
      feature_object.feature.cluster = transformed_cluster;
    }
    output_msg.header.frame_id = target_frame_id;
    return true;
  }
  return true;
}
}  // namespace autoware::object_recognition_utils

#endif  // AUTOWARE__OBJECT_RECOGNITION_UTILS__TRANSFORM_HPP_
