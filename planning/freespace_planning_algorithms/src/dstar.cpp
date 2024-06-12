// Copyright 2024 Andrzej_Norbert_Jeremiasz
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "freespace_planning_algorithms/dstar.hpp"
#include "freespace_planning_algorithms/astar_search.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>
#include <tier4_autoware_utils/math/normalization.hpp>

#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <vector>
#include <string>


namespace freespace_planning_algorithms
{ 

DstarSearch::TransitionTable createDstarTransitionTable(
  const double minimum_turning_radius, const double maximum_turning_radius,
  const int turning_radius_size, const double theta_size, const bool use_back)
{
  // Vehicle moving for each angle
  DstarSearch::TransitionTable transition_table;
  transition_table.resize(theta_size);

  const double dtheta = 2.0 * M_PI / theta_size;

  // Minimum moving distance with one state update
  // arc  = r * theta
  const auto & R_min = minimum_turning_radius;
  const auto & R_max = maximum_turning_radius;
  const double step_min = R_min * dtheta;
  const double dR = (R_max - R_min) / std::max(turning_radius_size - 1, 1);

  // NodeUpdate actions
  std::vector<DstarNodeUpdate> forward_node_candidates;
  const DstarNodeUpdate forward_straight{-step_min, 0.0, 0.0, step_min, false, false};
  forward_node_candidates.push_back(forward_straight);
  for (int i = 0; i < turning_radius_size; ++i) {
    double R = R_min + i * dR;
    double step = R * dtheta;
    const DstarNodeUpdate forward_left{
      -R * sin(dtheta), R * (1 - cos(dtheta)), -dtheta, step, true, false};
    const DstarNodeUpdate forward_right = forward_left.flipped();
    forward_node_candidates.push_back(forward_left);
    forward_node_candidates.push_back(forward_right);
  }

  for (int i = 0; i < theta_size; i++) {
    const double theta = dtheta * i;

    for (const auto & nu : forward_node_candidates) {
      transition_table[i].push_back(nu.rotated(theta));
    }

    if (use_back) {
      for (const auto & nu : forward_node_candidates) {
        transition_table[i].push_back(nu.reversed().rotated(theta));
      }
    }
  }

  return transition_table;
}


void dsetYaw(geometry_msgs::msg::Quaternion * orientation, const double yaw)
{
  *orientation = tier4_autoware_utils::createQuaternionFromYaw(yaw);
}

DstarSearch::DstarSearch(
  const PlannerCommonParam & planner_common_param, const VehicleShape & collision_vehicle_shape,
  const DstarParam & dstar_param)
: AbstractPlanningAlgorithm(planner_common_param, collision_vehicle_shape),
  dstar_param_(dstar_param),
  goal_node_(nullptr),
  use_reeds_shepp_(true)
{
  transition_table_ = createDstarTransitionTable(
  planner_common_param_.minimum_turning_radius, planner_common_param_.maximum_turning_radius,
  planner_common_param_.turning_radius_size, planner_common_param_.theta_size,
  dstar_param_.use_back);

  y_scale_ = planner_common_param.theta_size;
}


void DstarSearch::clearNodesDstar()
{
  openlist_ = std::priority_queue<DstarNode *, std::vector<DstarNode *>, DNodeComparison>();
  graph_ = std::unordered_map<uint, DstarNode>();
}

void DstarSearch::setMap(const nav_msgs::msg::OccupancyGrid & costmap)
{
  AbstractPlanningAlgorithm::setMap(costmap);


  x_scale_ = costmap_.info.height;
  graph_.reserve(100000);
}

bool DstarSearch::makePlan(
  const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & goal_pose)
{
  start_pose_ = global2local(costmap_, start_pose);
  goal_pose_ = global2local(costmap_, goal_pose);

  if (!setStartNode()) {
    return false;
  }

  if (!setGoalNode()) {
    return false;
  }

  return search();
  return false;
}

bool DstarSearch::setStartNode()
{
  const auto index = pose2index(costmap_, start_pose_, planner_common_param_.theta_size);

  if (detectCollision(index)) {
    return false;
  }

  // Set start node
  DstarNode * start_node = getDNodeRef(index);
  start_node->x = start_pose_.position.x;
  start_node->y = start_pose_.position.y;
  start_node->theta = 2.0 * M_PI / planner_common_param_.theta_size * index.theta;
  start_node->g = std::numeric_limits<double>::infinity();
  start_node->h = 0;
  start_node->rhs = std::numeric_limits<double>::infinity();
  start_node->status = DNodeStatus::OPEN;
  start_node->parent = nullptr;

  start_node_ = start_node;
  return true;
}

bool DstarSearch::setGoalNode()
{
    const auto index = pose2index(costmap_, goal_pose_, planner_common_param_.theta_size);

    if (detectCollision(index)) {
      return false;
    }

    DstarNode * goal_node = getDNodeRef(index);
    goal_node->x = goal_pose_.position.x;
    goal_node->y = goal_pose_.position.y;
    goal_node->theta = 2.0 * M_PI / planner_common_param_.theta_size * index.theta;
    goal_node->g = std::numeric_limits<double>::infinity();
    goal_node->h = estimateCost(start_pose_); 
    goal_node->rhs = 0;
    goal_node->status = DNodeStatus::NEW;
    goal_node->parent = nullptr;

    goal_node_ = goal_node;
    openlist_.push(goal_node_);

    return true;
}


double DstarSearch::estimateCost(const geometry_msgs::msg::Pose & pose) const
{
  double total_cost = 0.0;
  if (use_reeds_shepp_) {
    const double radius = (planner_common_param_.minimum_turning_radius +
                           planner_common_param_.maximum_turning_radius) *
                          0.5;
    total_cost +=
      calcReedsSheppDistance(pose, start_pose_, radius) * dstar_param_.distance_heuristic_weight;
  } else {
    total_cost += tier4_autoware_utils::calcDistance2d(pose, start_pose_) *
                  dstar_param_.distance_heuristic_weight;
  }
  return total_cost;
}

double DstarSearch::getCost( DstarNode* u) {
    return u->distance;
}


void DstarSearch::UpdateVertex(DstarNode* u)
{
  if (u->g != u->rhs) {
    if (u->status == DNodeStatus::OPEN) {
      u->status = DNodeStatus::NEW;  
      openlist_.push(u);  // Reinsert with updated key
      u->status = DNodeStatus::OPEN;

    } else {
      // Insert the node into the open list with the new key
      u->status = DNodeStatus::OPEN;
      openlist_.push(u);
    }
  } else if (u->status == DNodeStatus::OPEN) {
    // Remove the node from the open list
    u->status = DNodeStatus::NEW;  
  }
}


std::vector<DstarNode*> DstarSearch::getNeighbours(DstarNode* u) {
  std::vector<DstarNode*> neighbours;
  const auto index_theta = static_cast<size_t>(discretizeAngle(u->theta, planner_common_param_.theta_size));  
  if (index_theta >= transition_table_.size()) {
    return neighbours;
  }

  for (const auto & transition : transition_table_[index_theta]) {
    // Calculate next pose
    geometry_msgs::msg::Pose next_pose;
    next_pose.position.x = u->x + transition.shift_x;
    next_pose.position.y = u->y + transition.shift_y;
    dsetYaw(&next_pose.orientation, u->theta + transition.shift_theta);
    const auto neighbour_index = pose2index(costmap_, next_pose, planner_common_param_.theta_size);

    if (!detectCollision(neighbour_index)) {
      DstarNode * neighbour = getDNodeRef(neighbour_index);
        neighbour->x = next_pose.position.x;
        neighbour->y = next_pose.position.y;
        neighbour->theta = tf2::getYaw(next_pose.orientation);
        neighbour->distance = transition.distance;
        neighbours.push_back(neighbour);
    }
  }

  return neighbours;
}

std::vector<DstarNode*> DstarSearch::getPredecessors(DstarNode* u)  {
  return getNeighbours(u);
}

std::vector<DstarNode*> DstarSearch::getSuccessors(DstarNode* u)  {
  return getNeighbours(u);
}


void DstarSearch::ComputeShortestPath() {
    while (!openlist_.empty() && (CalculateKey(openlist_.top()) < CalculateKey(start_node_) || start_node_->rhs != start_node_->g)) {
        DstarNode* u = openlist_.top();
        openlist_.pop();
        Cost k_old = u->cost;
        Cost k_new = CalculateKey(u);

        if (k_old < k_new) {
            openlist_.push(u);
        } else if (u->g > u->rhs) {
            u->g = u->rhs;
            u->status = DNodeStatus::CLOSED;
            for (DstarNode* s : getPredecessors(u)) {
                if (s != goal_node_) {
                    s->rhs = std::min(s->rhs, getCost(u) + u->g);
                }
                UpdateVertex(s);
            }
        } else {
            double g_old = u->g;
            u->g = std::numeric_limits<double>::infinity();

            std::vector<DstarNode*> nodes_to_update = getPredecessors(u);
            nodes_to_update.push_back(u);
            for (DstarNode* s : nodes_to_update) {
                if (s->rhs == getCost(u) + g_old) {
                    if (s != goal_node_) {
                        s->rhs = std::numeric_limits<double>::infinity();
                        for (DstarNode* succ : getSuccessors(s)) {
                            s->rhs = std::min(s->rhs, getCost(succ) + succ->g);
                        }
                    }
                }
                UpdateVertex(s);
            }
        }
    }
}

double DstarSearch::heuristic(DstarNode *a, DstarNode *b)
{
    const auto a_x = a->x;
    const auto a_y = a->y;
    const auto a_theta = a->theta;
    const auto b_x = b->x;
    const auto b_y = b->y;
    const auto b_theta = b->theta;
    geometry_msgs::msg::Pose pose_a;
    pose_a.position.x = a_x;
    pose_a.position.y = a_y;
    pose_a.orientation = tier4_autoware_utils::createQuaternionFromYaw(a_theta);

    geometry_msgs::msg::Pose pose_b;
    pose_b.position.x = b_x;
    pose_b.position.y = b_y;
    pose_b.orientation = tier4_autoware_utils::createQuaternionFromYaw(b_theta);
    // Heuristic function
    return estimateCost(pose_a) - estimateCost(pose_b);
}

std::vector<DstarNode*> DstarSearch::scanGraphForChanges() {
    std::vector<DstarNode*> nodes_with_collisions;

    for (auto &entry : graph_) {
        DstarNode &node = entry.second;
        IndexXYT index = pose2index(costmap_, node2pose(node), planner_common_param_.theta_size);

        if (detectCollision(index)) {
            nodes_with_collisions.push_back(&node);
        }
    }

    return nodes_with_collisions;
}

bool DstarSearch::search() {
    DstarNode* s_last = start_node_;
    ComputeShortestPath();

    while (start_node_ != goal_node_) {
        if (start_node_->g == std::numeric_limits<double>::infinity()) {
            // If g(s_start) is infinity, there is no known path
            return false;
        }

        // Find the node s' that minimizes c(s_start, s') + g(s')
        DstarNode* next_node = nullptr;
        double min_cost = std::numeric_limits<double>::infinity();

        for (DstarNode* s_prime : getSuccessors(start_node_)) {
            double cost = getCost(s_prime) + s_prime->g;
            if (cost < min_cost) {
                min_cost = cost;
                next_node = s_prime;
            }
        }
        if (next_node == nullptr) {
            return false; // No valid successor found
        }

        // Move to the next node
        start_node_ = next_node;

        // Scan graph for changed edge costs (this would typically involve sensor updates)
        std::vector<DstarNode*> changed_nodes = scanGraphForChanges();
        if (!changed_nodes.empty()) {
          // If any edge costs changed
          k_m_ += heuristic(s_last, start_node_);
          s_last = start_node_;
          for (const auto& node : changed_nodes) {
            UpdateVertex(node);
          }
          ComputeShortestPath();
        }

    }

    return true;
}
geometry_msgs::msg::Pose DstarSearch::node2pose(const DstarNode & node) const
{
  geometry_msgs::msg::Pose pose_local;

  pose_local.position.x = node.x;
  pose_local.position.y = node.y;
  pose_local.position.z = goal_pose_.position.z;
  pose_local.orientation = tier4_autoware_utils::createQuaternionFromYaw(node.theta);

  return pose_local;
}


}  // namespace freespace_planning_algorithms
