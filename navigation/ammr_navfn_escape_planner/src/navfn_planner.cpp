// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2018 Simbe Robotics
// Copyright (c) 2019 Samsung Research America
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

// Navigation Strategy based on:
// Brock, O. and Oussama K. (1999). High-Speed Navigation Using
// the Global Dynamic Window Approach. IEEE.
// https://cs.stanford.edu/group/manips/publications/pdfs/Brock_1999_ICRA.pdf

// #define BENCHMARK_TESTING

#include "ammr_navfn_escape_planner/navfn_planner.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "ammr_navfn_escape_planner/navfn.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;  // NOLINT
using nav2_util::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace ammr_navfn_escape_planner
{

NavfnPlanner::NavfnPlanner()
: tf_(nullptr), costmap_(nullptr)
{
}

NavfnPlanner::~NavfnPlanner()
{
  RCLCPP_INFO(
    logger_, "Destroying plugin %s of type NavfnPlanner",
    name_.c_str());
}

void
NavfnPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  tf_ = tf;
  name_ = name;
  costmap_ = costmap_ros->getCostmap();
  costmap_ros_ = costmap_ros;
  global_frame_ = costmap_ros->getGlobalFrameID();

  node_ = parent;
  auto node = parent.lock();
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  RCLCPP_INFO(
    logger_, "Configuring plugin %s of type NavfnPlanner",
    name_.c_str());

  // Initialize parameters
  // Declare this plugin's parameters
  declare_parameter_if_not_declared(node, name + ".tolerance", rclcpp::ParameterValue(0.5));
  node->get_parameter(name + ".tolerance", tolerance_);
  declare_parameter_if_not_declared(node, name + ".use_astar", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".use_astar", use_astar_);
  declare_parameter_if_not_declared(node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".allow_unknown", allow_unknown_);
  declare_parameter_if_not_declared(
    node, name + ".use_final_approach_orientation", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".use_final_approach_orientation", use_final_approach_orientation_);
  // [10-05 D17] escape: 출발 자세에서 제자리 회전이 막혀 있으면 지금 방향(앞/뒤)으로 곧장 빠진 뒤 NavFn 경로를 잇는다
  declare_parameter_if_not_declared(node, name + ".escape_enable", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".escape_enable", escape_enable_);
  declare_parameter_if_not_declared(node, name + ".escape_max_dist", rclcpp::ParameterValue(1.5));
  node->get_parameter(name + ".escape_max_dist", escape_max_dist_);
  declare_parameter_if_not_declared(node, name + ".escape_step", rclcpp::ParameterValue(0.05));
  node->get_parameter(name + ".escape_step", escape_step_);
  escape_step_ = std::max(0.01, escape_step_);
  escape_max_dist_ = std::max(0.0, escape_max_dist_);
  declare_parameter_if_not_declared(node, name + ".escape_margin", rclcpp::ParameterValue(0.15));
  node->get_parameter(name + ".escape_margin", escape_margin_);
  declare_parameter_if_not_declared(node, name + ".escape_min_straight", rclcpp::ParameterValue(0.5));
  node->get_parameter(name + ".escape_min_straight", escape_min_straight_);
  declare_parameter_if_not_declared(node, name + ".escape_edge_inset", rclcpp::ParameterValue(-1.0));
  node->get_parameter(name + ".escape_edge_inset", escape_edge_inset_);   // 음수 = 격자 1칸
  // [10-06 사용자] "abnormal 상황에서는 최소한의 이동으로 해결" — 내보내는 직선은 BT recovery 기동(0.45 m)과 같은 상한.
  //   회전 가능 지점 탐색(escape_max_dist)은 그대로 두고 출력 길이만 자른다 → 남은 거리는 다음 회복 주기가 다시 판단.
  declare_parameter_if_not_declared(node, name + ".escape_max_len", rclcpp::ParameterValue(-1.0));
  node->get_parameter(name + ".escape_max_len", escape_max_len_);

  // Create a planner based on the new costmap size
  planner_ = std::make_unique<NavFn>(
    costmap_->getSizeInCellsX(),
    costmap_->getSizeInCellsY());
}

void
NavfnPlanner::activate()
{
  RCLCPP_INFO(
    logger_, "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
  // Add callback for dynamic parameters
  auto node = node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&NavfnPlanner::dynamicParametersCallback, this, _1));
}

void
NavfnPlanner::deactivate()
{
  RCLCPP_INFO(
    logger_, "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
  auto node = node_.lock();
  if (dyn_params_handler_ && node) {
    node->remove_on_set_parameters_callback(dyn_params_handler_.get());
  }
  dyn_params_handler_.reset();
}

void
NavfnPlanner::cleanup()
{
  RCLCPP_INFO(
    logger_, "Cleaning up plugin %s of type NavfnPlanner",
    name_.c_str());
  planner_.reset();
}

nav_msgs::msg::Path NavfnPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
#ifdef BENCHMARK_TESTING
  steady_clock::time_point a = steady_clock::now();
#endif
  unsigned int mx_start, my_start, mx_goal, my_goal;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start)) {
    throw nav2_core::StartOutsideMapBounds(
            "Start Coordinates of(" + std::to_string(start.pose.position.x) + ", " +
            std::to_string(start.pose.position.y) + ") was outside bounds");
  }

  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)) {
    throw nav2_core::GoalOutsideMapBounds(
            "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
            std::to_string(goal.pose.position.y) + ") was outside bounds");
  }

  if (tolerance_ == 0 && costmap_->getCost(mx_goal, my_goal) == nav2_costmap_2d::LETHAL_OBSTACLE) {
    throw nav2_core::GoalOccupied(
            "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
            std::to_string(goal.pose.position.y) + ") was in lethal cost");
  }

  // Update planner based on the new costmap size
  if (isPlannerOutOfDate()) {
    planner_->setNavArr(
      costmap_->getSizeInCellsX(),
      costmap_->getSizeInCellsY());
  }

  nav_msgs::msg::Path path;

  // Corner case of the start(x,y) = goal(x,y)
  if (start.pose.position.x == goal.pose.position.x &&
    start.pose.position.y == goal.pose.position.y)
  {
    path.header.stamp = clock_->now();
    path.header.frame_id = global_frame_;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.z = 0.0;

    pose.pose = start.pose;
    // if we have a different start and goal orientation, set the unique path pose to the goal
    // orientation, unless use_final_approach_orientation=true where we need it to be the start
    // orientation to avoid movement from the local planner
    if (start.pose.orientation != goal.pose.orientation && !use_final_approach_orientation_) {
      pose.pose.orientation = goal.pose.orientation;
    }
    path.poses.push_back(pose);
    return path;
  }

  // [10-06 S3b 회귀 수정] 앞 구간에서 자른 escape 직선의 나머지 + 이 구간
  if (escape_enable_ && tryEscapeRest(start, goal, cancel_checker, path)) {
    return path;
  }
  escape_rest_.valid = false;

  // [10-05 D17] 회전 못 하는 출발 자세 → 지금 방향으로 먼저 빠지는 경로
  if (escape_enable_ && tryEscapePlan(start, goal, path)) {
    return path;
  }

  if (!makePlan(start.pose, goal.pose, tolerance_, cancel_checker, path)) {
    throw nav2_core::NoValidPathCouldBeFound(
            "Failed to create plan with tolerance of: " + std::to_string(tolerance_) );
  }


#ifdef BENCHMARK_TESTING
  steady_clock::time_point b = steady_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(b - a);
  std::cout << "It took " << time_span.count() * 1000 << std::endl;
#endif

  return path;
}

bool
NavfnPlanner::isPlannerOutOfDate()
{
  if (!planner_.get() ||
    planner_->nx != static_cast<int>(costmap_->getSizeInCellsX()) ||
    planner_->ny != static_cast<int>(costmap_->getSizeInCellsY()))
  {
    return true;
  }
  return false;
}

bool
NavfnPlanner::makePlan(
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal, double tolerance,
  std::function<bool()> cancel_checker,
  nav_msgs::msg::Path & plan)
{
  // clear the plan, just in case
  plan.poses.clear();

  plan.header.stamp = clock_->now();
  plan.header.frame_id = global_frame_;

  double wx = start.position.x;
  double wy = start.position.y;

  RCLCPP_DEBUG(
    logger_, "Making plan from (%.2f,%.2f) to (%.2f,%.2f)",
    start.position.x, start.position.y, goal.position.x, goal.position.y);

  unsigned int mx, my;
  worldToMap(wx, wy, mx, my);

  // clear the starting cell within the costmap because we know it can't be an obstacle
  clearRobotCell(mx, my);

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  // make sure to resize the underlying array that Navfn uses
  planner_->setNavArr(
    costmap_->getSizeInCellsX(),
    costmap_->getSizeInCellsY());

  planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

  lock.unlock();

  int map_start[2];
  map_start[0] = mx;
  map_start[1] = my;

  wx = goal.position.x;
  wy = goal.position.y;

  worldToMap(wx, wy, mx, my);
  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  planner_->setStart(map_goal);
  planner_->setGoal(map_start);
  if (use_astar_) {
    planner_->calcNavFnAstar(cancel_checker);
  } else {
    planner_->calcNavFnDijkstra(cancel_checker, true);
  }

  double resolution = costmap_->getResolution();
  geometry_msgs::msg::Pose p, best_pose;

  bool found_legal = false;

  p = goal;
  double potential = getPointPotential(p.position);
  if (potential < POT_HIGH) {
    // Goal is reachable by itself
    best_pose = p;
    found_legal = true;
  } else {
    // Goal is not reachable. Trying to find nearest to the goal
    // reachable point within its tolerance region
    double best_sdist = std::numeric_limits<double>::max();

    p.position.y = goal.position.y - tolerance;
    while (p.position.y <= goal.position.y + tolerance) {
      p.position.x = goal.position.x - tolerance;
      while (p.position.x <= goal.position.x + tolerance) {
        potential = getPointPotential(p.position);
        double sdist = squared_distance(p, goal);
        if (potential < POT_HIGH && sdist < best_sdist) {
          best_sdist = sdist;
          best_pose = p;
          found_legal = true;
        }
        p.position.x += resolution;
      }
      p.position.y += resolution;
    }
  }

  if (found_legal) {
    // extract the plan
    if (getPlanFromPotential(best_pose, plan)) {
      smoothApproachToGoal(best_pose, plan);

      // If use_final_approach_orientation=true, interpolate the last pose orientation from the
      // previous pose to set the orientation to the 'final approach' orientation of the robot so
      // it does not rotate.
      // And deal with corner case of plan of length 1
      if (use_final_approach_orientation_) {
        size_t plan_size = plan.poses.size();
        if (plan_size == 1) {
          plan.poses.back().pose.orientation = start.orientation;
        } else if (plan_size > 1) {
          double dx, dy, theta;
          auto last_pose = plan.poses.back().pose.position;
          auto approach_pose = plan.poses[plan_size - 2].pose.position;
          // Deal with the case of NavFn producing a path with two equal last poses
          if (std::abs(last_pose.x - approach_pose.x) < 0.0001 &&
            std::abs(last_pose.y - approach_pose.y) < 0.0001 && plan_size > 2)
          {
            approach_pose = plan.poses[plan_size - 3].pose.position;
          }
          dx = last_pose.x - approach_pose.x;
          dy = last_pose.y - approach_pose.y;
          theta = atan2(dy, dx);
          plan.poses.back().pose.orientation =
            nav2_util::geometry_utils::orientationAroundZAxis(theta);
        }
      }
    } else {
      RCLCPP_ERROR(
        logger_,
        "Failed to create a plan from potential when a legal"
        " potential was found. This shouldn't happen.");
    }
  }

  return !plan.poses.empty();
}

void
NavfnPlanner::smoothApproachToGoal(
  const geometry_msgs::msg::Pose & goal,
  nav_msgs::msg::Path & plan)
{
  if (plan.poses.size() >= 2) {
    auto second_to_last_pose = plan.poses.end()[-2];
    auto last_pose = plan.poses.back();
    // Replace the last pose of the computed path if it's actually further away
    // to the second to last pose than the goal pose.
    if (
      squared_distance(last_pose.pose, second_to_last_pose.pose) >
      squared_distance(goal, second_to_last_pose.pose))
    {
      plan.poses.back().pose = goal;
      return;
    }
    // Replace the last pose of the computed path if its position matches but orientation differs
    if (squared_distance(last_pose.pose, goal) < 1e-6) {
      plan.poses.back().pose = goal;
      return;
    }
  }
  geometry_msgs::msg::PoseStamped goal_copy;
  goal_copy.pose = goal;
  goal_copy.header = plan.header;
  plan.poses.push_back(goal_copy);
}


bool
NavfnPlanner::getPlanFromPotential(
  const geometry_msgs::msg::Pose & goal,
  nav_msgs::msg::Path & plan)
{
  // clear the plan, just in case
  plan.poses.clear();

  // Goal should be in global frame
  double wx = goal.position.x;
  double wy = goal.position.y;

  // the potential has already been computed, so we won't update our copy of the costmap
  unsigned int mx, my;
  worldToMap(wx, wy, mx, my);

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  planner_->setStart(map_goal);

  const int & max_cycles = (costmap_->getSizeInCellsX() >= costmap_->getSizeInCellsY()) ?
    (costmap_->getSizeInCellsX() * 4) : (costmap_->getSizeInCellsY() * 4);

  int path_len = planner_->calcPath(max_cycles);
  if (path_len == 0) {
    return false;
  }

  auto cost = planner_->getLastPathCost();
  RCLCPP_DEBUG(
    logger_,
    "Path found, %d steps, %f cost\n", path_len, cost);

  // extract the plan
  float * x = planner_->getPathX();
  float * y = planner_->getPathY();
  int len = planner_->getPathLen();

  // 방향 계산을 위한 변수 초기화
  geometry_msgs::msg::Quaternion last_orientation;
  last_orientation.w = 1.0; 

  // NavFn은 Goal -> Start 순서로 배열을 반환하므로 역순으로 순회
  for (int i = len - 1; i >= 0; --i) {
    // convert the plan to world coordinates
    double world_x, world_y;
    mapToWorld(x[i], y[i], world_x, world_y);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = plan.header;
    pose.pose.position.x = world_x;
    pose.pose.position.y = world_y;
    pose.pose.position.z = 0.0;

    // [Modified Logic Start]
    if (i == 0) {
        // 1. 마지막 점 (Goal)인 경우:
        // 경로의 마지막 점은 요청받은 Goal의 Orientation을 그대로 따릅니다.
        pose.pose.orientation = goal.orientation;
    } else {
        // 2. 중간 경로 점들인 경우:
        // 다음 점(i-1)을 바라보는 방향(Tangent)을 계산합니다.
        
        double next_world_x, next_world_y;
        mapToWorld(x[i - 1], y[i - 1], next_world_x, next_world_y);

        double dx = next_world_x - world_x;
        double dy = next_world_y - world_y;

        // 노이즈 방지를 위해 일정 거리 이상일 때만 방향 갱신
        if (std::hypot(dx, dy) > 1e-3) {
            double theta = std::atan2(dy, dx);
            pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta);
            last_orientation = pose.pose.orientation; // 계산된 방향 저장
        } else {
            // 제자리거나 거리가 너무 가까우면 이전 방향 유지
            pose.pose.orientation = last_orientation;
        }
    }
    // [Modified Logic End]

    plan.poses.push_back(pose);
  }

  return !plan.poses.empty();
}





// bool
// NavfnPlanner::getPlanFromPotential(
//   const geometry_msgs::msg::Pose & goal,
//   nav_msgs::msg::Path & plan)
// {
//   // clear the plan, just in case
//   plan.poses.clear();

//   // Goal should be in global frame
//   double wx = goal.position.x;
//   double wy = goal.position.y;

//   // the potential has already been computed, so we won't update our copy of the costmap
//   unsigned int mx, my;
//   worldToMap(wx, wy, mx, my);

//   int map_goal[2];
//   map_goal[0] = mx;
//   map_goal[1] = my;

//   planner_->setStart(map_goal);

//   const int & max_cycles = (costmap_->getSizeInCellsX() >= costmap_->getSizeInCellsY()) ?
//     (costmap_->getSizeInCellsX() * 4) : (costmap_->getSizeInCellsY() * 4);

//   int path_len = planner_->calcPath(max_cycles);
//   if (path_len == 0) {
//     return false;
//   }

//   auto cost = planner_->getLastPathCost();
//   RCLCPP_DEBUG(
//     logger_,
//     "Path found, %d steps, %f cost\n", path_len, cost);

//   // extract the plan
//   float * x = planner_->getPathX();
//   float * y = planner_->getPathY();
//   int len = planner_->getPathLen();

//   for (int i = len - 1; i >= 0; --i) {
//     // convert the plan to world coordinates
//     double world_x, world_y;
//     mapToWorld(x[i], y[i], world_x, world_y);

//     geometry_msgs::msg::PoseStamped pose;
//     pose.header = plan.header;
//     pose.pose.position.x = world_x;
//     pose.pose.position.y = world_y;
//     pose.pose.position.z = 0.0;
//     pose.pose.orientation.x = 0.0;
//     pose.pose.orientation.y = 0.0;
//     pose.pose.orientation.z = 0.0;
//     pose.pose.orientation.w = 1.0;
//     plan.poses.push_back(pose);
//   }

//   return !plan.poses.empty();
// }

double
NavfnPlanner::getPointPotential(const geometry_msgs::msg::Point & world_point)
{
  unsigned int mx, my;
  if (!worldToMap(world_point.x, world_point.y, mx, my)) {
    return std::numeric_limits<double>::max();
  }

  unsigned int index = my * planner_->nx + mx;
  return planner_->potarr[index];
}

// bool
// NavfnPlanner::validPointPotential(const geometry_msgs::msg::Point & world_point)
// {
//   return validPointPotential(world_point, tolerance_);
// }

// bool
// NavfnPlanner::validPointPotential(
//   const geometry_msgs::msg::Point & world_point, double tolerance)
// {
//   const double resolution = costmap_->getResolution();

//   geometry_msgs::msg::Point p = world_point;
//   double potential = getPointPotential(p);
//   if (potential < POT_HIGH) {
//     // world_point is reachable by itself
//     return true;
//   } else {
//     // world_point, is not reachable. Trying to find any
//     // reachable point within its tolerance region
//     p.y = world_point.y - tolerance;
//     while (p.y <= world_point.y + tolerance) {
//       p.x = world_point.x - tolerance;
//       while (p.x <= world_point.x + tolerance) {
//         potential = getPointPotential(p);
//         if (potential < POT_HIGH) {
//           return true;
//         }
//         p.x += resolution;
//       }
//       p.y += resolution;
//     }
//   }

//   return false;
// }

bool
NavfnPlanner::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
{
  if (wx < costmap_->getOriginX() || wy < costmap_->getOriginY()) {
    return false;
  }

  mx = static_cast<int>(
    std::round((wx - costmap_->getOriginX()) / costmap_->getResolution()));
  my = static_cast<int>(
    std::round((wy - costmap_->getOriginY()) / costmap_->getResolution()));

  if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY()) {
    return true;
  }

  RCLCPP_ERROR(
    logger_,
    "worldToMap failed: mx,my: %d,%d, size_x,size_y: %d,%d", mx, my,
    costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

  return false;
}

void
NavfnPlanner::mapToWorld(double mx, double my, double & wx, double & wy)
{
  wx = costmap_->getOriginX() + mx * costmap_->getResolution();
  wy = costmap_->getOriginY() + my * costmap_->getResolution();
}

void
NavfnPlanner::clearRobotCell(unsigned int mx, unsigned int my)
{
  // TODO(orduno): check usage of this function, might instead be a request to
  //               world_model / map server
  costmap_->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
}

// ----------------------------------------------------------------------------------------------
// [10-05 D17, sim 검증 10-03~04] escape (v2 — 리뷰 wf_3958ed18 반영)
//   현장 증상(B형): 옆 벽 3~8 cm 에서 정사각 차체(0.635x0.63, 제자리 회전에 옆 여유 0.134 m)는 돌 수 없다.
//   NavFn(점 로봇)은 벽에서 비스듬히 떨어지는 경로(48~160도 회전 필요)를 주고, MPPI 는 그 회전을 못 해 105.
//   출발 자세(= 로봇 현재 자세)에서 회전이 막혀 있으면 **직선 구간만** 돌려준다:
//     (a) goal 이 지금 방향 직선 위(옆 <= 5 cm, 방향 차 <= 20도)이고 그 직선이 비면 goal 까지 직선,
//     (b) 아니면 앞/뒤로 나가며 처음으로 제자리 회전이 가능한 지점 d 를 찾고, 그보다 margin 더(최소 min_straight)
//         비어 있는 만큼까지 직선. 다음 회복 주기가 회전 가능한 자세에서 평소대로 계획한다.
//         (NavFn 꼬리를 붙이면 MPPI 는 0.45 m 절단 alt_goal 만 보고 직선을 안 따른다 — 리뷰 high)
//   직선 진행 검사: unpadded footprint 의 **앞장서는 변**(전진=앞변, 후진=뒷변)만, 격자 크기 이하 간격으로.
//     옆변이 이미 벽에 닿은 상태(3~5 cm 격자 양자화)는 평행 이동으로 새로 닿는 칸이 없으므로 막지 않는다.
//   회전 가능: (padded) 외접원 안 모든 칸이 LETHAL·NO_INFORMATION 이 아님 (360도 회전이 쓸고 가는 정확한 영역).
//   검사 동안 costmap mutex 를 잡는다.
static double quatYaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

static bool cellBlocked(nav2_costmap_2d::Costmap2D * cm, double wx, double wy)
{
  unsigned int mx, my;
  if (!cm->worldToMap(wx, wy, mx, my)) {
    return true;
  }
  const unsigned char c = cm->getCost(mx, my);
  return c == nav2_costmap_2d::LETHAL_OBSTACLE || c == nav2_costmap_2d::NO_INFORMATION;
}

bool
NavfnPlanner::leadingEdgeFree(double x, double y, double yaw, double dir)
{
  const double ex = dir > 0 ? fp_front_ : fp_back_;
  const double res = costmap_->getResolution();
  // [10-03 S4] 앞장서는 변 양 끝을 escape_edge_inset(기본 격자 1칸)만큼 안쪽으로: 옆이 벽에 붙은 채 나란히 갈 때
  //   변 끝점이 격자 양자화로 벽 칸에 걸려 직진이 막히던 것 (옆 5 cm, S4 E1 3/8). 실제 충돌은 MPPI(local 0.02)가 막는다.
  const double inset = escape_edge_inset_ >= 0.0 ? escape_edge_inset_ : res;
  const double yl = fp_left_ - inset, yr = fp_right_ + inset;
  const int n = std::max(2, static_cast<int>(std::ceil((yl - yr) / (0.5 * res))));
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (int i = 0; i <= n; ++i) {
    const double ey = yr + (yl - yr) * i / n;
    if (cellBlocked(costmap_, x + c * ex - s * ey, y + s * ex + c * ey)) {
      return false;
    }
  }
  return true;
}

bool
NavfnPlanner::canRotate(double x, double y)
{
  double r = 0.0;
  for (const auto & p : costmap_ros_->getRobotFootprint()) {
    r = std::max(r, std::hypot(p.x, p.y));
  }
  const double res = costmap_->getResolution();
  for (double dx = -r; dx <= r + 1e-9; dx += res) {
    for (double dy = -r; dy <= r + 1e-9; dy += res) {
      if (dx * dx + dy * dy <= r * r && cellBlocked(costmap_, x + dx, y + dy)) {
        return false;
      }
    }
  }
  return true;
}

double
NavfnPlanner::sweepFree(double sx, double sy, double yaw, double dir, double max_dist)
{
  // 직선으로 비어 있는 거리 (격자 크기 이하 간격으로 앞장서는 변 검사)
  const double step = std::min(escape_step_, 0.5 * costmap_->getResolution());
  const double ux = std::cos(yaw), uy = std::sin(yaw);
  double ok = 0.0;
  for (double d = step; d <= max_dist + 1e-9; d += step) {
    if (!leadingEdgeFree(sx + dir * d * ux, sy + dir * d * uy, yaw, dir)) {
      break;
    }
    ok = d;
  }
  return ok;
}

bool
NavfnPlanner::isRobotStart(const geometry_msgs::msg::Pose & start)
{
  geometry_msgs::msg::PoseStamped robot;
  if (!costmap_ros_ || !costmap_ros_->getRobotPose(robot)) {
    return false;
  }
  const double dyaw = std::fabs(std::remainder(
      quatYaw(robot.pose.orientation) - quatYaw(start.orientation), 2.0 * M_PI));
  return std::hypot(robot.pose.position.x - start.position.x,
           robot.pose.position.y - start.position.y) < 0.10 && dyaw < 0.3;
}

void
NavfnPlanner::appendStraight(
  const geometry_msgs::msg::Pose & from, double dir, double dist, nav_msgs::msg::Path & path)
{
  const double yaw = quatYaw(from.orientation);
  const int n = std::max(1, static_cast<int>(std::ceil(dist / escape_step_)));
  for (int i = 0; i <= n; ++i) {
    const double d = dist * i / n;
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    p.pose.position.x = from.position.x + dir * d * std::cos(yaw);
    p.pose.position.y = from.position.y + dir * d * std::sin(yaw);
    p.pose.orientation = from.orientation;          // 후진 구간도 차체 방향은 그대로
    path.poses.push_back(p);
  }
}

void
NavfnPlanner::noteEscapeRest(
  const geometry_msgs::msg::Pose & from, double dir, double out_len, double full_len)
{
  escape_rest_.valid = full_len - out_len > 1e-3;
  if (!escape_rest_.valid) {
    return;
  }
  const double yaw = quatYaw(from.orientation);
  escape_rest_.x = from.position.x + dir * out_len * std::cos(yaw);
  escape_rest_.y = from.position.y + dir * out_len * std::sin(yaw);
  escape_rest_.yaw = yaw;
  escape_rest_.dir = dir;
  escape_rest_.rest = full_len - out_len;
  escape_rest_.stamp = clock_->now();
}

bool
NavfnPlanner::tryEscapeRest(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker, nav_msgs::msg::Path & path)
{
  if (!escape_rest_.valid) {
    return false;
  }
  // 바로 다음 구간 (같은 through-poses 계산) 만: 시작이 잘린 끝과 같고 (1 cm, 3°) 1 s 안
  const bool same = std::hypot(start.pose.position.x - escape_rest_.x, start.pose.position.y - escape_rest_.y) < 0.01 &&
    std::fabs(std::remainder(quatYaw(start.pose.orientation) - escape_rest_.yaw, 2.0 * M_PI)) < 0.05 &&
    (clock_->now() - escape_rest_.stamp).seconds() < 1.0;
  const EscapeRest r = escape_rest_;
  escape_rest_.valid = false;
  if (!same) {
    return false;
  }
  path.poses.clear();
  path.header.stamp = clock_->now();
  path.header.frame_id = global_frame_;
  appendStraight(start.pose, r.dir, r.rest, path);     // 자르기 전 직선의 끝까지
  geometry_msgs::msg::Pose mid = path.poses.back().pose;
  if (std::hypot(goal.pose.position.x - mid.position.x, goal.pose.position.y - mid.position.y) > 1e-3) {
    nav_msgs::msg::Path tail;
    if (!makePlan(mid, goal.pose, tolerance_, cancel_checker, tail)) {
      throw nav2_core::NoValidPathCouldBeFound(
              "Failed to create plan with tolerance of: " + std::to_string(tolerance_) + " (after escape rest)");
    }
    for (size_t i = tail.poses.empty() ? 0 : 1; i < tail.poses.size(); ++i) {
      tail.poses[i].header = path.header;
      path.poses.push_back(tail.poses[i]);
    }
  }
  RCLCPP_INFO(logger_, "[%s escape] next through-poses segment: rest of capped straight %.2f m prepended",
    name_.c_str(), r.rest);
  return true;
}

bool
NavfnPlanner::tryEscapePlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal,
  nav_msgs::msg::Path & path)
{
  if (!isRobotStart(start.pose)) {
    return false;                                   // through-poses 뒤쪽 구간 등 → 원래 NavFn
  }
  const double sx = start.pose.position.x, sy = start.pose.position.y;
  const double yaw = quatYaw(start.pose.orientation);
  const double ux = std::cos(yaw), uy = std::sin(yaw);
  // unpadded footprint 치수 (직사각형 가정)
  fp_front_ = fp_back_ = fp_left_ = fp_right_ = 0.0;
  for (const auto & p : costmap_ros_->getUnpaddedRobotFootprint()) {
    fp_front_ = std::max(fp_front_, static_cast<double>(p.x));
    fp_back_ = std::min(fp_back_, static_cast<double>(p.x));
    fp_left_ = std::max(fp_left_, static_cast<double>(p.y));
    fp_right_ = std::min(fp_right_, static_cast<double>(p.y));
  }

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  if (canRotate(sx, sy)) {
    return false;                                   // 회전 가능 → 원래 NavFn
  }
  path.poses.clear();
  path.header.stamp = clock_->now();
  path.header.frame_id = global_frame_;

  // (a) goal 이 지금 방향 직선 위
  const double gx = goal.pose.position.x - sx, gy = goal.pose.position.y - sy;
  const double lon = gx * ux + gy * uy, lat = -gx * uy + gy * ux;
  const double gyaw_err = std::fabs(std::remainder(quatYaw(goal.pose.orientation) - yaw, 2.0 * M_PI));
  if (std::fabs(lat) <= 0.05 && gyaw_err <= 20.0 * M_PI / 180.0 && std::fabs(lon) > 1e-3) {
    const double dir = lon >= 0 ? 1.0 : -1.0;
    // [10-06] 받아들일지는 예전처럼 goal 까지 **직선 전체**가 비었는지로 판단하고, 내보내는 길이만 자른다
    //   (앞 0.45 m 만 보면 그 너머가 막힌 직선도 받아들여 0.45 m 씩 들어갔다 되나오게 된다 — 검토 wf_3646d78a)
    const double len_a = escape_max_len_ > 0.0 ? std::min(std::fabs(lon), escape_max_len_) : std::fabs(lon);
    if (sweepFree(sx, sy, yaw, dir, std::fabs(lon)) >= std::fabs(lon) - 0.5 * costmap_->getResolution()) {
      appendStraight(start.pose, dir, len_a, path);
      noteEscapeRest(start.pose, dir, len_a, std::fabs(lon));
      RCLCPP_INFO(logger_, "[%s escape] start cannot rotate -> straight %s %.2f m to goal%s",
        name_.c_str(), dir > 0 ? "forward" : "backward", len_a,
        len_a < std::fabs(lon) - 1e-6 ? " (capped, goal further)" : "");
      return true;
    }
  }

  // (b) 앞/뒤로 처음 회전 가능한 지점 + margin
  double best_len = -1.0, best_d = -1.0, best_dir = 0.0, best_full = -1.0;
  for (double dir : {1.0, -1.0}) {
    const double free_d = sweepFree(sx, sy, yaw, dir, escape_max_dist_ + escape_margin_);
    double d_rot = -1.0;
    for (double d = escape_step_; d <= std::min(free_d, escape_max_dist_) + 1e-9; d += escape_step_) {
      if (canRotate(sx + dir * d * ux, sy + dir * d * uy)) {
        d_rot = d;
        break;
      }
    }
    if (d_rot < 0) {
      continue;
    }
    double len = std::min(free_d, std::max(d_rot + escape_margin_, escape_min_straight_));
    const double len_full = len;
    if (escape_max_len_ > 0.0) {
      len = std::min(len, escape_max_len_);       // [10-06] 출력 상한 — 회전 가능 지점이 더 멀면 다음 주기에 이어서
    }
    const bool better = best_d < 0 || d_rot < best_d - 1e-6 ||
      (std::fabs(d_rot - best_d) < 1e-6 && dir * lon > best_dir * lon);
    if (better) {
      best_d = d_rot;
      best_len = len;
      best_dir = dir;
      best_full = len_full;
    }
  }
  if (best_d < 0) {
    RCLCPP_WARN(logger_, "[%s escape] start cannot rotate and no straight escape within %.2f m -> plain NavFn",
      name_.c_str(), escape_max_dist_);
    return false;
  }
  appendStraight(start.pose, best_dir, best_len, path);
  noteEscapeRest(start.pose, best_dir, best_len, best_full);
  RCLCPP_INFO(logger_, "[%s escape] start cannot rotate -> straight %s %.2f m (rotatable at %.2f m), goal after next replan",
    name_.c_str(), best_dir > 0 ? "forward" : "backward", best_len, best_d);
  return true;
}

rcl_interfaces::msg::SetParametersResult
NavfnPlanner::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == name_ + ".tolerance") {
        tolerance_ = parameter.as_double();
      } else if (name == name_ + ".escape_max_dist") {
        escape_max_dist_ = std::max(0.0, parameter.as_double());
      } else if (name == name_ + ".escape_step") {
        escape_step_ = std::max(0.01, parameter.as_double());
      } else if (name == name_ + ".escape_margin") {
        escape_margin_ = std::max(0.0, parameter.as_double());
      } else if (name == name_ + ".escape_min_straight") {
        escape_min_straight_ = std::max(0.0, parameter.as_double());
      } else if (name == name_ + ".escape_edge_inset") {
        escape_edge_inset_ = parameter.as_double();
      } else if (name == name_ + ".escape_max_len") {
        escape_max_len_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == name_ + ".use_astar") {
        use_astar_ = parameter.as_bool();
      } else if (name == name_ + ".allow_unknown") {
        allow_unknown_ = parameter.as_bool();
      } else if (name == name_ + ".use_final_approach_orientation") {
        use_final_approach_orientation_ = parameter.as_bool();
      } else if (name == name_ + ".escape_enable") {
        escape_enable_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace ammr_navfn_escape_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ammr_navfn_escape_planner::NavfnPlanner, nav2_core::GlobalPlanner)
