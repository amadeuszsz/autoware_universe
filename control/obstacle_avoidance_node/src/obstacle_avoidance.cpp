//  Copyright 2022 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "obstacle_avoidance.hpp"

// #include "ObstacleAvoidance.hpp"
namespace obstacle_avoidance {

using namespace std;
using namespace rclcpp;
using autoware_auto_planning_msgs::msg::Trajectory;
using sensor_msgs::msg::LaserScan;
using std_msgs::msg::Bool;

ObstacleAvoidance::ObstacleAvoidance(const rclcpp::NodeOptions& options)
: Node("obstacle_avoidance", options)
{
    subscription_ = this->create_subscription<Trajectory>(
        "/planning/racing_planner/trajectory", 10,
        bind(&ObstacleAvoidance::trajectory_callback, this, placeholders::_1));

    publisher_ = this->create_publisher<Trajectory>(
        "/planning/racing_planner/avoidance/trajectory", 10);

    laser_subscription_ = this->create_subscription<LaserScan>(
        "/sensing/lidar/scan", 10,
        bind(&ObstacleAvoidance::lidar_callback, this, placeholders::_1));

    obstacle_detected_subscription_ = this->create_subscription<Bool>(
        "/obstackle_detected", 10,
        bind(&ObstacleAvoidance::obstacle_detected_callback, this, placeholders::_1));

    filtered_lidar_scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/filtered_lidar_scan", 10,
        std::bind(&ObstacleAvoidance::filtered_lidar_scan_callback, this, std::placeholders::_1));

    pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/localization/cartographer/pose", 10,
        std::bind(&ObstacleAvoidance::pose_callback, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "visualization_marker_array", 10);


}

void ObstacleAvoidance::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    last_pose_ = msg;  //  ostatnia pozycja robota
}

void ObstacleAvoidance::lidar_callback(const LaserScan::SharedPtr msg)
{
    // vector<pair<int, int>> obstacle_positions = ranges_to_positions(msg->ranges, msg->angle_min, msg->angle_increment);

    int grid_size[2] = {30, 30};
    // pair<int, int> goal = {25, 25};
    // int obstacle_charge = 100;
    // int goal_charge = -200;

    // auto potential_field = compute_potential_field(grid_size, goal, obstacle_positions, obstacle_charge, goal_charge);
}

void ObstacleAvoidance::filtered_lidar_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    last_filtered_scan_ = msg; 
}

void ObstacleAvoidance::obstacle_detected_callback(const Bool::SharedPtr msg)
{
    // RCLCPP_INFO_STREAM(this->get_logger(), "Obstacle msg = " << msg->data );
    // if (msg->data) {
        
    obstacle_detected = msg->data;
        // sprawdzmy czy jest na tracectory 
    // }
}

void ObstacleAvoidance::trajectory_callback(const Trajectory::SharedPtr msg)
{
    // if obstacle detected to sprawdzamy czy jest na trajectory 
    if (!obstacle_detected){
        // RCLCPP_INFO_STREAM(this->get_logger(), "Publishing trajectory"); 

        publisher_->publish(*msg);
    }
    else {
        // RCLCPP_INFO_STREAM(this->get_logger(), "obstackle detected!!!!!!!!!!1");
        if (!last_filtered_scan_) {
            // RCLCPP_INFO_STREAM(this->get_logger(), "No filtered scan available");
            return;
        }

        bool obstacle_on_trajectory = false;
        // Pobranie orientacji robota (yaw)
        float robot_yaw = getYawFromQuaternion(last_pose_->pose.orientation);
        float robot_x = last_pose_->pose.position.x;
        float robot_y = last_pose_->pose.position.y;

        // przekształcenie danych z LaserScan na współrzędne
        // last_pose_
        for (size_t i = 0; i < last_filtered_scan_->ranges.size(); i++) {
            double range = last_filtered_scan_->ranges[i];
            if (range < last_filtered_scan_->range_max && range > last_filtered_scan_->range_min) {
                double angle = last_filtered_scan_->angle_min + i * last_filtered_scan_->angle_increment + robot_yaw;
                double local_x = range * cos(angle);
                double local_y = range * sin(angle);

                // Transformacja do globalnych współrzędnych
                double x = robot_x + local_x + lidar_offset_x * cos(robot_yaw) - lidar_offset_y * sin(robot_yaw);
                double y = robot_y + local_y + lidar_offset_x * sin(robot_yaw) + lidar_offset_y * cos(robot_yaw);

                double distance_to_robot = std::sqrt(std::pow(x - robot_x, 2) + std::pow(y - robot_y, 2));
                if (distance_to_robot > 2.5){
                    continue;
                }

                // Sprawdzamy czy punkt (x, y) jest blisko jakiegokolwiek punktu trajektorii
                for (const auto& point : msg->points) {
                    double distance = std::sqrt(std::pow(point.pose.position.x - x, 2) + std::pow(point.pose.position.y - y, 2));

                    if (distance < 0.30) {
                        // RCLCPP_INFO_STREAM(this->get_logger(), "distance " << distance);

                        obstacle_on_trajectory = true;
                        break;
                    }
                }
                if (obstacle_on_trajectory) break;
                
            }
        }
        if (obstacle_on_trajectory) {
            // RCLCPP_INFO_STREAM(this->get_logger(), "ON TRAJECTORY NEED TO CHENGE!!!!!");
            vector<pair<int, int>> obstacle_positions = ranges_to_positions(last_filtered_scan_->ranges, last_filtered_scan_->angle_min, last_filtered_scan_->angle_increment);

            // int grid_size[2] = {30, 30};
            // int obstacle_charge = 100;
            // int goal_charge = -200;

             // Oblicz cel 3 metry przed aktualną pozycją
            // double goal_x = last_pose_->pose.position.x + 3 * cos(tf2::getYaw(last_pose_->pose.orientation));
            // double goal_y = last_pose_->pose.position.y + 3 * sin(tf2::getYaw(last_pose_->pose.orientation));
            // std::pair<double, double> closest_point;
            // double min_distance = std::numeric_limits<double>::max();

            // for (const auto& point : msg->points) {
            //     double dx = point.pose.position.x - goal_x;
            //     double dy = point.pose.position.y - goal_y;
            //     double distance = std::sqrt(dx * dx + dy * dy);

            //     if (distance < min_distance) {
            //         min_distance = distance;
            //         closest_point.first = point.pose.position.x;
            //         closest_point.second = point.pose.position.y;
            //     }
            // }

            double repulse_factor = 5.0;
            double threshold = 5.0;

            auto new_trajectory = std::make_shared<autoware_auto_planning_msgs::msg::Trajectory>(*msg);
            auto potentials = compute_potential_field(msg, obstacle_positions, repulse_factor);

            if (trajectory_modified){
                double dx = last_pose_modified->pose.position.x - last_pose_->pose.position.x;
                double dy = last_pose_modified->pose.position.y - last_pose_->pose.position.y;
                double diffrence_pose = std::sqrt(dx * dx + dy * dy);
                if (diffrence_pose > 3.0){
                    trajectory_modified = false;
                }
            }
            
             

            
            if (!trajectory_modified) {
                modify_trajectory(new_trajectory, potentials, threshold, obstacle_positions);
                published_trajectory = *new_trajectory;
            }
            

            publisher_->publish(published_trajectory); 
            // auto potential_field = compute_potential_field(grid_size, closest_point, obstacle_positions, obstacle_charge, goal_charge);

            // auto new_trajectory = create_new_trajectory(potential_field, {static_cast<int>(last_pose_->pose.position.x), static_cast<int>(last_pose_->pose.position.y)}, goal);
            
            // publisher_->publish(new_trajectory);  // Publikacja nowej trajektorii
        }
        else{
            // RCLCPP_INFO_STREAM(this->get_logger(), "Publishing trajectory"); 
            publisher_->publish(*msg);
        }
            
        
    }

}

float ObstacleAvoidance::getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
    tf2::Quaternion tfq;
    tf2::fromMsg(q, tfq);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tfq).getRPY(roll, pitch, yaw);
    return static_cast<float>(yaw);
}

vector<pair<int, int>> ObstacleAvoidance::ranges_to_positions(const vector<float>& ranges, float angle_min, float angle_increment)
{
    vector<pair<int, int>> positions;
    // last_pose_
    float robot_yaw = getYawFromQuaternion(last_pose_->pose.orientation); // Pobranie orientacji robota (yaw)
    float robot_x = last_pose_->pose.position.x; // Pobranie globalnej pozycji x robota
    float robot_y = last_pose_->pose.position.y; // Pobranie globalnej pozycji y robota

    float angle = angle_min + robot_yaw; // Kąt początkowy uwzględniający orientację robota

    for (auto range : ranges) {
        if (range < 30) { // Zakładamy, że 30 to maksymalny zasięg, który nas interesuje
            float local_x = range * cos(angle);
            float local_y = range * sin(angle);
            // Transformacja lokalnych współrzędnych LIDAR do globalnych współrzędnych mapy
            int global_x = static_cast<int>(robot_x + local_x + lidar_offset_x * cos(robot_yaw) - lidar_offset_y * sin(robot_yaw));
            int global_y = static_cast<int>(robot_y + local_y + lidar_offset_x * sin(robot_yaw) + lidar_offset_y * cos(robot_yaw));
            positions.emplace_back(global_x, global_y);
        }
        angle += angle_increment; // Aktualizacja kąta dla następnego pomiaru
    }

    return positions;
}

std::vector<double> ObstacleAvoidance::compute_potential_field(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr& trajectory, const std::vector<std::pair<int, int>>& obstacles, double repulse_factor) {
    std::vector<double> potentials(trajectory->points.size(), 0.0);
    for (size_t i = 0; i < trajectory->points.size(); i++) {
        auto& tp = trajectory->points[i].pose.position;
        double min_obstacle_distance = std::numeric_limits<double>::max();
        
        for (auto& obs : obstacles) {
            double dx = tp.x - obs.first;
            double dy = tp.y - obs.second;
            double distance = std::sqrt(dx * dx + dy * dy);
            min_obstacle_distance = std::min(min_obstacle_distance, distance);
        }

        potentials[i] = repulse_factor / (min_obstacle_distance + 1e-5);
    }
    publishPotentialField(potentials, trajectory);
    return potentials;
}

// void ObstacleAvoidance::modify_trajectory(autoware_auto_planning_msgs::msg::Trajectory::SharedPtr& trajectory, const std::vector<double>& potentials, double threshold, const std::vector<std::pair<int, int>>& obstacle_positions) {
//     if (trajectory->points.size() < 2) return;

//     double scale_factor = 0.3;  // Współczynnik mocy odpychania
//     double smoothing_factor = 0.5; // Intensywność wygładzania

//     std::vector<geometry_msgs::msg::Point> new_positions(trajectory->points.size());


//     for (size_t i = 0; i < trajectory->points.size(); i++) {
//         if (potentials[i] > threshold && i > 0 && i < trajectory->points.size() - 1) {
//             auto& current_point = trajectory->points[i].pose.position;
//             auto& prev_point = trajectory->points[i - 1].pose.position;
//             auto& next_point = trajectory->points[i + 1].pose.position;

//             // Logika przesunięcia z powodu przeszkód (jak wcześniej)

//             double vec_x = (prev_point.x - current_point.x) + (next_point.x - current_point.x);
//             double vec_y = (prev_point.y - current_point.y) + (next_point.y - current_point.y);

//             double length = std::sqrt(vec_x * vec_x + vec_y * vec_y);
//             if (length > 0) {
//                 vec_x /= length;
//                 vec_y /= length;
//             }

//             current_point.x += scale_factor * vec_x;
//             current_point.y += scale_factor * vec_y;
//         }

//         // Przechowywanie nowych pozycji
//         new_positions[i] = trajectory->points[i].pose.position;
//     }

//     // Wygładzanie trajektorii
//     for (size_t i = 1; i < trajectory->points.size() - 1; i++) {
//         new_positions[i].x = smoothing_factor * new_positions[i].x + (1 - smoothing_factor) * (new_positions[i - 1].x + new_positions[i + 1].x) / 2;
//         new_positions[i].y = smoothing_factor * new_positions[i].y + (1 - smoothing_factor) * (new_positions[i - 1].y + new_positions[i + 1].y) / 2;
//     }

//     // Aktualizacja trajektorii
//     for (size_t i = 0; i < trajectory->points.size(); i++) {
//         trajectory->points[i].pose.position = new_positions[i];
//     }
// }

void ObstacleAvoidance::modify_trajectory(autoware_auto_planning_msgs::msg::Trajectory::SharedPtr& trajectory, const std::vector<double>& potentials, double threshold, const std::vector<std::pair<int, int>>& obstacle_positions) {
    if (trajectory->points.size() < 2) return;

    double scale_factor = 0.3;  // Współczynnik mocy odpychania
    double smoothing_factor = 0.5; // Intensywność wygładzania

    std::vector<geometry_msgs::msg::Point> new_positions(trajectory->points.size());

    for (size_t i = 0; i < trajectory->points.size(); i++) {
        if (potentials[i] > threshold) {
            auto& current_point = trajectory->points[i].pose.position;

            // Znajdowanie najbliższej przeszkody
            std::pair<int, int> closest_obstacle;
            double min_distance = std::numeric_limits<double>::max();

            for (const auto& obstacle : obstacle_positions) {
                double distance = std::sqrt(std::pow(current_point.x - obstacle.first, 2) + std::pow(current_point.y - obstacle.second, 2));
                if (distance < min_distance) {
                    min_distance = distance;
                    closest_obstacle = obstacle;
                }
            }

            // Obliczanie wektora odpychania od najbliższej przeszkody
            double vec_x = current_point.x - closest_obstacle.first;
            double vec_y = current_point.y - closest_obstacle.second;
            double length = std::sqrt(vec_x * vec_x + vec_y * vec_y);
            if (length > 0) {
                vec_x /= length;
                vec_y /= length;
            }

            current_point.x += scale_factor * vec_x;
            current_point.y += scale_factor * vec_y;
        }
        // Przechowywanie nowych pozycji
        new_positions[i] = trajectory->points[i].pose.position;
    }

    // Wygładzanie trajektorii
    for (size_t i = 1; i < trajectory->points.size() - 1; i++) {
        new_positions[i].x = smoothing_factor * new_positions[i].x + (1 - smoothing_factor) * (new_positions[i - 1].x + new_positions[i + 1].x) / 2;
        new_positions[i].y = smoothing_factor * new_positions[i].y + (1 - smoothing_factor) * (new_positions[i - 1].y + new_positions[i + 1].y) / 2;
    }

    // Aktualizacja trajektorii
    for (size_t i = 0; i < trajectory->points.size(); i++) {
        trajectory->points[i].pose.position = new_positions[i];
    }
    trajectory_modified = true;
    last_pose_modified = last_pose_;
}


void ObstacleAvoidance::publishPotentialField(const std::vector<double>& potentials, const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr& trajectory) {
    visualization_msgs::msg::MarkerArray marker_array;
    for (size_t i = 0; i < trajectory->points.size(); i++) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "potential_field";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = trajectory->points[i].pose.position.x;
        marker.pose.position.y = trajectory->points[i].pose.position.y;
        marker.pose.position.z = 0.5;  // Wysokość nad ziemią
        marker.scale.x = 0.2;  // Rozmiar sfery
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0;  // Nieprzezroczystość
        marker.color.r = 1.0 - potentials[i];  // Czerwień maleje wraz z potencjałem
        marker.color.g = potentials[i];  // Zielony wzrasta wraz z potencjałem
        marker.color.b = 0.0;  // Brak niebieskiego

        marker_array.markers.push_back(marker);
    }

    marker_pub_->publish(marker_array);
}

// std::vector<std::vector<float>> ObstacleAvoidance::compute_potential_field(const std::array<int, 2>& grid_size, const std::pair<int, int>& goal, const std::vector<std::pair<int, int>>& obstacles, int obstacle_charge, int goal_charge) {
//     std::vector<std::vector<float>> field(grid_size[1], std::vector<float>(grid_size[0], 0));

//     //  przyciąganie do celu
//     for (int y = 0; y < grid_size[1]; ++y) {
//         for (int x = 0; x < grid_size[0]; ++x) {
//             double dx = x - goal.first;
//             double dy = y - goal.second;
//             double distance = sqrt(dx * dx + dy * dy);
//             field[y][x] -= goal_charge / (distance + 1); 
//         }
//     }

//     //  odpychanie od przeszkód
//     for (const auto& obs : obstacles) {
//         for (int y = 0; y < grid_size[1]; ++y) {
//             for (int x = 0; x < grid_size[0]; ++x) {
//                 double dx = x - obs.first;
//                 double dy = y - obs.second;
//                 double distance = sqrt(dx * dx + dy * dy);
//                 field[y][x] += obstacle_charge / (distance + 1);
//             }
//         }
//     }

//     return field;
// }

// autoware_auto_planning_msgs::msg::Trajectory ObstacleAvoidance::create_new_trajectory(const std::vector<std::vector<float>>& potential_field, const std::pair<int, int>& start, const std::pair<int, int>& goal) {
//     autoware_auto_planning_msgs::msg::Trajectory new_trajectory;
//     std::pair<int, int> current = start;

//     while (current != goal) {
//         std::pair<int, int> next_step = current;
//         float min_potential = std::numeric_limits<float>::max();

//         for (int dy = -1; dy <= 1; ++dy) {
//             for (int dx = -1; dx <= 1; ++dx) {
//                 int nx = current.first + dx;
//                 int ny = current.second + dy;
//                 if (nx >= 0 && ny >= 0 && nx < potential_field[0].size() && ny < potential_field.size()) {
//                     if (potential_field[ny][nx] < min_potential) {
//                         min_potential = potential_field[ny][nx];
//                         next_step = {nx, ny};
//                     }
//                 }
//             }
//         }

//         // punkt do nowej trajektorii
//         autoware_auto_planning_msgs::msg::TrajectoryPoint point;
//         point.pose.position.x = next_step.first;
//         point.pose.position.y = next_step.second;
//         new_trajectory.points.push_back(point);
//         current = next_step;

//         if (current == goal) break; 
//     }

//     return new_trajectory;
// }



} // namespace obstacle_avoidance

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_avoidance::ObstacleAvoidance)
