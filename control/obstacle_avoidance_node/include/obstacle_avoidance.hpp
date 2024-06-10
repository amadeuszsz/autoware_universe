
// #include <rclcpp/rclcpp.hpp>

// #include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
// #include <autoware_auto_planning_msgs/msg/trajectory.hpp>
// #include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
// #include <geometry_msgs/msg/pose.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <autoware_auto_planning_msgs/msg/trajectory.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <tf2_ros/transform_listener.hpp>
// #include <tf2_ros/message_filtering.h>
// #include <tf2_geometry_msgs/msg.h>
// #include <cmath>
// #include <algorithm>

#include <memory>
#ifndef OBSTACLE_AVOIDANCE_HPP
#define OBSTACLE_AVOIDANCE_HPP

#include "rclcpp/rclcpp.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/quaternion.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


#include <vector>
#include <cmath>

namespace obstacle_avoidance {
class ObstacleAvoidance : public rclcpp::Node
{
public:
    explicit ObstacleAvoidance(const rclcpp::NodeOptions& options);

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void trajectory_callback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg);
    void obstacle_detected_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void filtered_lidar_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    void modify_trajectory(autoware_auto_planning_msgs::msg::Trajectory::SharedPtr& trajectory, const std::vector<double>& potentials, double threshold, const std::vector<std::pair<int, int>>& obstacle_positions);
    std::vector<std::pair<int, int>> ranges_to_positions(const std::vector<float>& ranges, float angle_min, float angle_increment);
    std::vector<double> compute_potential_field(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr& trajectory, const std::vector<std::pair<int, int>>& obstacles, double repulse_factor);
    float getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q);
    void publishPotentialField(const std::vector<double>& potentials, const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr& trajectory);


    rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr subscription_;
    rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_detected_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr filtered_lidar_scan_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
   

    bool obstacle_detected = false;
    bool trajectory_modified = false;
    geometry_msgs::msg::PoseStamped::SharedPtr last_pose_modified;
    sensor_msgs::msg::LaserScan::SharedPtr last_filtered_scan_;
    geometry_msgs::msg::PoseStamped::SharedPtr last_pose_;
    autoware_auto_planning_msgs::msg::Trajectory published_trajectory;
    const double lidar_offset_x = 0.35;
    const double lidar_offset_y = 0.0;
    
};
} // namespace obstacle_avoidance
#endif // OBSTACLE_AVOIDANCE_HPP
