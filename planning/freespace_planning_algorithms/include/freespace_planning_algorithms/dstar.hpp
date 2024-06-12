// Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#ifndef FREESPACE_PLANNING_ALGORITHMS__DSTAR_HPP_
#define FREESPACE_PLANNING_ALGORITHMS__DSTAR_HPP_

#include "freespace_planning_algorithms/abstract_algorithm.hpp"
#include "freespace_planning_algorithms/reeds_shepp.hpp"
#include "freespace_planning_algorithms/astar_search.hpp"


#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/header.hpp>

#include <cmath>
#include <functional>
#include <iostream>
#include <queue>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace freespace_planning_algorithms
{
enum class DNodeStatus : uint8_t { NEW, OPEN, CLOSED };

double calcReedsSheppDistance(
  const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2, double radius);



struct DstarParam
{
  // base configs
  bool only_behind_solutions;  // solutions should be behind the goal
  bool use_back;               // backward search
  double distance_heuristic_weight;  
};

struct Cost {
  double total_cost;
  double min_cost;

  bool operator<(const Cost& other) const {
    if (total_cost != other.total_cost) {
      return total_cost < other.total_cost;
    }
    return min_cost < other.min_cost;
  }
};

struct DstarNode
{
  DNodeStatus status = DNodeStatus::NEW;  // node status
  double x;                              // x
  double y;                              // y
  double theta;                          // theta
  double g = 0;                         // predicted cost
  double h = 0;                         // heuristic cost
  double rhs = 0;                       // actual cost
  IndexXYT index;
  DstarNode * parent = nullptr;          // parent node
  double distance = 0;
  Cost cost = {0, 0};
};

struct DstarNodeUpdate
{
  double shift_x;
  double shift_y;
  double shift_theta;
  double distance;
  bool is_curve;
  bool is_back;

  DstarNodeUpdate rotated(const double theta) const
  {
    DstarNodeUpdate result = *this;
    result.shift_x = std::cos(theta) * this->shift_x - std::sin(theta) * this->shift_y;
    result.shift_y = std::sin(theta) * this->shift_x + std::cos(theta) * this->shift_y;
    return result;
  }

  DstarNodeUpdate flipped() const
  {
    DstarNodeUpdate result = *this;
    result.shift_y = -result.shift_y;
    result.shift_theta = -result.shift_theta;
    return result;
  }

  DstarNodeUpdate reversed() const
  {
    DstarNodeUpdate result = *this;
    result.shift_x = -result.shift_x;
    result.shift_theta = -result.shift_theta;
    result.is_back = !result.is_back;
    return result;
  }
};

struct DNodeComparison
{
  bool operator()(const DstarNode * lhs, const DstarNode * rhs) const
  {
    if (!lhs || !rhs) {
      throw std::runtime_error("Null pointer in DNodeComparison");
    }

    Cost lhs_key = lhs->cost;
    Cost rhs_key = rhs->cost;

    if (lhs_key.total_cost != rhs_key.total_cost) {
      return lhs_key.total_cost > rhs_key.total_cost;
    }
    return lhs_key.min_cost > rhs_key.min_cost;
  }
};

class DstarSearch : public AbstractPlanningAlgorithm
{
public:
  using TransitionTable = std::vector<std::vector<DstarNodeUpdate>>;
  DstarSearch(
    const PlannerCommonParam & planner_common_param, const VehicleShape & collision_vehicle_shape,
    const DstarParam & dstar_param);

  DstarSearch(
    const PlannerCommonParam & planner_common_param, const VehicleShape & collision_vehicle_shape,
    rclcpp::Node & node)
  : DstarSearch(
      planner_common_param, collision_vehicle_shape,
      DstarParam{
        node.declare_parameter<bool>("dstar.only_behind_solutions"),
        node.declare_parameter<bool>("dstar.use_back"),
        node.declare_parameter<double>("dstar.distance_heuristic_weight")})
  {
  }
  
  void setYaw(geometry_msgs::msg::Quaternion * orientation, const double yaw);
  void setMap(const nav_msgs::msg::OccupancyGrid & costmap) override;
  bool makePlan(
    const geometry_msgs::msg::Pose & start_pose,
    const geometry_msgs::msg::Pose & goal_pose) override;

  void clearNodesDstar();
  const visualization_msgs::msg::MarkerArray & getMarkerArray() const { return marker_array_; }

  inline int getKey(const IndexXYT & index) const
  {
    return (index.theta + (index.y * x_scale_ + index.x) * y_scale_);
  }


private:
  bool search();
  void UpdateVertex();
  bool setStartNode();
  bool setGoalNode();
  double estimateCost(const geometry_msgs::msg::Pose & pose) const;
  void UpdateVertex(DstarNode* u);
  void ComputeShortestPath();
  double getCost(DstarNode* u);
  double heuristic(DstarNode *a, DstarNode *b);
  geometry_msgs::msg::Pose node2pose(const DstarNode & node) const;


  DstarNode * getDNodeRef(const IndexXYT & index)
  {
    return &(graph_.emplace(getKey(index), DstarNode()).first->second);
  }

  Cost CalculateKey(DstarNode * node) const {
    node->cost = {std::min(node->g, node->rhs) + node->h + k_m_ , std::min(node->g, node->rhs)};
    return node->cost;
  }

  std::vector<DstarNode*> getNeighbours(DstarNode* u);
  std::vector<DstarNode*> getPredecessors(DstarNode* u);
  std::vector<DstarNode*> getSuccessors(DstarNode* u);
  std::vector<DstarNode*> scanGraphForChanges();
  // Algorithm specific param
  DstarParam dstar_param_;
  TransitionTable transition_table_;

  std::unordered_map<uint, DstarNode> graph_;
  std::priority_queue<DstarNode *, std::vector<DstarNode *>, DNodeComparison> openlist_;

  DstarNode * goal_node_;
  DstarNode * start_node_;

  bool use_reeds_shepp_;

  int x_scale_;
  int y_scale_;
  double k_m_ = 0;
  std::vector<std::vector<int>> grid_;

};
}  // namespace freespace_planning_algorithms

#endif  // FREESPACE_PLANNING_ALGORITHMS__DSTAR_HPP_
