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

#include "trajectory_follower_node/simple_trajectory_follower.hpp"

#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/pose_deviation.hpp>

#include <algorithm>

namespace simple_trajectory_follower
{

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

SimpleTrajectoryFollower::SimpleTrajectoryFollower(const rclcpp::NodeOptions & options)
: Node("simple_trajectory_follower", options)
{
  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1); //do sterowania prędkością boczną i wzdłużną  

  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", 1, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });//subskrybuje temat, odebraną wiadomość przypisuje do zmiennej odometry_
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", 1, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });

  use_external_target_vel_ = declare_parameter<bool>("use_external_target_vel"); // tworzy zmienną use_external_target_vel było zdefiniowane w pliku hpp
  external_target_vel_ = declare_parameter<float>("external_target_vel");
  lateral_deviation_ = declare_parameter<float>("lateral_deviation");

  using namespace std::literals::chrono_literals;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 30ms, std::bind(&SimpleTrajectoryFollower::onTimer, this));// wyołanie funkcji SimpleTrajectoryFollower::onTimer co jakiś czas 
}

void SimpleTrajectoryFollower::onTimer()
{
  if (!checkData()) {
    RCLCPP_INFO(get_logger(), "data not ready");
    return;
  }// sprawdza czy odebrał subkrubowane (trajectory_, odometry_) dane oraz czy nie są puste jeśli są puse lb ich nie odebrał przerywa wywołaną funkcję 

  updateClosest();//funckcja znajdującą punkt z trajektori który znajdję się najbliżej aktyalnej pozycji closest_traj_point_

  AckermannControlCommand cmd; //tworzy zmienną cmd typu AckermannControlCommand (służy do kontrolownia skrętu kół )
  cmd.stamp = cmd.lateral.stamp = cmd.longitudinal.stamp = get_clock()->now();//ustawia aktualny czas dla prędkości kontowej oraz liniowej
  cmd.lateral.steering_tire_angle = static_cast<float>(calcSteerCmd());//calcSteerCmd()-funkcja zwraca kąt skrętu kuł, przypisanie wartości skrętu kół do zmiennej cmd 
  cmd.longitudinal.speed = use_external_target_vel_ ? static_cast<float>(external_target_vel_)
                                                    : closest_traj_point_.longitudinal_velocity_mps;//use_external_target_vel_-zmienn kontrolna jeśli ma wrtość true wartość prędkosci liniowej będzie równa zmiennej external_target_vel_ jeśli nie chyba wpisujemy prędkość deflautową trajektori ??
  cmd.longitudinal.acceleration = static_cast<float>(calcAccCmd());//przypisanie wartości przyspieszenia 
  pub_cmd_->publish(cmd);
}

void SimpleTrajectoryFollower::updateClosest()
{
  const auto closest = findNearestIndex(trajectory_->points, odometry_->pose.pose.position);// funkcja która znajduje index elementu w senkwencji (w naszym przypadku trajectory_->points) który znajduję się najbliżej danej wartości (pose.pose.position)
  closest_traj_point_ = trajectory_->points.at(closest);// wspułżądnę najbliższego punktu w stosunku do punktu pose.pose.position 
}

double SimpleTrajectoryFollower::calcSteerCmd()
{
  const auto lat_err =//zmienna zawiera błąd odchylenia bocznego lub zmianę 
    calcLateralDeviation(closest_traj_point_.pose, odometry_->pose.pose.position) -
    lateral_deviation_;//funkcja służy do obliczania odchylenia bocznego pojazdu od zdefiniowanej trajektorii ruchu
  const auto yaw_err = calcYawDeviation(closest_traj_point_.pose, odometry_->pose.pose);//obliczania odchylenia kątowego pojazdu od zdefiniowanej trajektorii ruchu

  // linearized pure_pursuit control
  constexpr auto wheel_base = 4.0;
  constexpr auto lookahead_time = 3.0;
  constexpr auto min_lookahead = 3.0;
  const auto lookahead = min_lookahead + lookahead_time * std::abs(odometry_->twist.twist.linear.x);//twist.twist.linear.x aktualana prędkość luniowa wzdłużosi x
  const auto kp = 2.0 * wheel_base / (lookahead * lookahead);
  const auto kd = 2.0 * wheel_base / lookahead; //obliczenie paramtrów sterrowinka pd 

  constexpr auto steer_lim = 0.6;

  const auto steer = std::clamp(-kp * lat_err - kd * yaw_err, -steer_lim, steer_lim);// oblicza wartoś skrętu kół ,najpradopodobniej
  RCLCPP_DEBUG(
    get_logger(), "kp = %f, lat_err = %f, kd - %f, yaw_err = %f, steer = %f", kp, lat_err, kd,
    yaw_err, steer);//wypisanie w konsoli 
  return steer;
}

double SimpleTrajectoryFollower::calcAccCmd()
{
  const auto traj_vel = static_cast<double>(closest_traj_point_.longitudinal_velocity_mps);//prędkość wzdłużna trajektori 
  const auto ego_vel = odometry_->twist.twist.linear.x;//aktualna prędkość 
  const auto target_vel = use_external_target_vel_ ? external_target_vel_ : traj_vel; //wybur p©ędkości w zależnosci od zmiennej use_external_target_vel_
  const auto vel_err = ego_vel - target_vel;//obliczenie błędu względem prędkości aktualnej i zadanej 

  // P feedback
  constexpr auto kp = 0.5;
  constexpr auto acc_lim = 2.0;

  const auto acc = std::clamp(-kp * vel_err, -acc_lim, acc_lim);//wyznaczenie wartości przyspieszenie przy urzuciu pd 
  RCLCPP_DEBUG(get_logger(), "vel_err = %f, acc = %f", vel_err, acc);
  return acc; 
}

bool SimpleTrajectoryFollower::checkData()
{
  return (trajectory_ && odometry_);
}

}  // namespace simple_trajectory_follower

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(simple_trajectory_follower::SimpleTrajectoryFollower)
