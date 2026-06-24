/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024, Custom Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cmath>
#include <string>
#include <memory>
#include <limits>
#include <vector>

#include "nav2_stable_stopped_goal_checker/stable_stopped_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "angles/angles.h"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

using std::hypot;
using std::fabs;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{

StableStoppedGoalChecker::StableStoppedGoalChecker()
: x_goal_tolerance_(0.25),
  y_goal_tolerance_(0.25),
  yaw_goal_tolerance_(0.25),
  rot_stopped_velocity_(0.25),
  trans_stopped_velocity_(0.25),
  xy_stability_duration_(0.5),
  yaw_stability_duration_(0.5),
  stateful_(true),
  check_xy_(true),
  in_xy_tolerance_(false),
  in_yaw_tolerance_(false),
  path_topic_("/plan_truncated_short") // 기본값
{
}

void StableStoppedGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();
  clock_ = node->get_clock();

  // Declare parameters
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".x_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".y_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".trans_stopped_velocity", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".rot_stopped_velocity", rclcpp::ParameterValue(0.25));
  
  // New parameters
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".xy_stability_duration", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".yaw_stability_duration", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".stateful", rclcpp::ParameterValue(true));

// [추가] path_topic 파라미터 선언
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".path_topic", rclcpp::ParameterValue("/plan_truncated_short"));

  // Get parameters
  node->get_parameter(plugin_name + ".x_goal_tolerance", x_goal_tolerance_);
  node->get_parameter(plugin_name + ".y_goal_tolerance", y_goal_tolerance_);
  node->get_parameter(plugin_name + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node->get_parameter(plugin_name + ".trans_stopped_velocity", trans_stopped_velocity_);
  node->get_parameter(plugin_name + ".rot_stopped_velocity", rot_stopped_velocity_);
  node->get_parameter(plugin_name + ".xy_stability_duration", xy_stability_duration_);
  node->get_parameter(plugin_name + ".yaw_stability_duration", yaw_stability_duration_);
  node->get_parameter(plugin_name + ".stateful", stateful_);

// [추가] Path Subscription 생성 (TransientLocal QoS 사용)
  // rclcpp::QoS qos(rclcpp::KeepLast(1));
  // qos.transient_local();
  path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    path_topic_, 10,
    std::bind(&StableStoppedGoalChecker::pathCallback, this, _1));


  remaining_goals_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    "/remaining_goals", rclcpp::QoS(10),
    std::bind(&StableStoppedGoalChecker::remainingGoalsCallback, this, _1));

  
  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&StableStoppedGoalChecker::dynamicParametersCallback, this, _1));
}


void StableStoppedGoalChecker::reset()
{
  check_xy_ = true;
  in_xy_tolerance_ = false;
  in_yaw_tolerance_ = false;
}

void StableStoppedGoalChecker::remainingGoalsCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  remaining_goals_count_.store(static_cast<int>(msg->poses.size()));  // ⚠️ PoseArray/Goals면 접근자 변경
}


// [추가] Path Callback 구현
void StableStoppedGoalChecker::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  current_path_ = msg;
}

bool StableStoppedGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & velocity)
{

// // [추가] Path Length Check
//   {
//     std::lock_guard<std::mutex> lock(path_mutex_);
//     if (current_path_) {
//       // nav2_util을 사용하여 경로 길이 계산
//       double total_distance = nav2_util::geometry_utils::calculate_path_length(*current_path_);
      
//       // 남은 경로의 길이가 허용 오차의 2배보다 크면, 아직 도착하지 않은 것으로 간주
//       if (total_distance > 2.0 * x_goal_tolerance_) {
//         // 단, 이미 latch(XY 완료) 상태라면 거리 체크를 무시하고 Yaw 체크로 넘어갈 수 있도록
//         // 아래 로직이 필요할 수 있으나, 요구사항에 맞춰 "엄격하게" 리턴합니다.
//         // 만약 XY가 이미 맞았더라도 경로가 갑자기 길어지면(Replanning 등) 멈추지 않아야 합니다.
//         // 여기서는 요구사항대로 "if distance <= ... 일 때만 체크"를 역으로 적용하여
//         // "if distance > ... 이면 False 리턴"으로 구현합니다.
        
//         // 주의: XY가 이미 맞아서 Latch된 상태(check_xy_ == false)에서도 
//         // 경로가 다시 길어졌다면(새로운 계획) 체크를 재개해야 하는지는 정책에 따릅니다.
//         // 여기서는 단순하게 적용합니다.
//         return false; 
//       }
//     }
//     // current_path_가 아직 없으면(nullptr), 안전을 위해 기존 로직을 수행하거나 false를 리턴할 수 있습니다.
//     // 여기서는 path가 없으면 거리 체크를 패스하고 기존 로직으로 넘어갑니다.
//   }

  // [교체] 3-조건 AND 게이트 (remaining_goals==최종 && 직선거리 근접 && 경로 짧음)
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    const double total_distance = current_path_
      ? nav2_util::geometry_utils::calculate_path_length(*current_path_) : 0.0;
    const double straight_dist = std::hypot(
      query_pose.position.x - goal_pose.position.x,
      query_pose.position.y - goal_pose.position.y);

    const int  rgc = remaining_goals_count_.load();
    const bool is_final_goal = (rgc >= 0 && rgc <= 1);   // -1(미수신)은 차단

    constexpr double PATH_DIST_MAX = 2.8;                // 컨트롤러와 동일값 유지
    const bool gate_ok = is_final_goal
                       && (straight_dist <= x_goal_tolerance_ * 1.5)  // 코스 프리게이트
                       && (total_distance <= PATH_DIST_MAX);
    if (!gate_ok) {
      return false;
    }
  }
  // 이하 기존 dx/dy/yaw/velocity stateful 로직 그대로

  

  // 1. Calculate Errors
  double dx = fabs(query_pose.position.x - goal_pose.position.x);
  double dy = fabs(query_pose.position.y - goal_pose.position.y);
  double dyaw = fabs(angles::shortest_angular_distance(
    tf2::getYaw(query_pose.orientation),
    tf2::getYaw(goal_pose.orientation)));

  bool xy_ok = (dx <= x_goal_tolerance_) && (dy <= y_goal_tolerance_);
  bool yaw_ok = (dyaw <= yaw_goal_tolerance_);

  // 2. Logic based on 'stateful' parameter
  if (stateful_) {
    // === STATEFUL MODE: Check XY first, then Yaw ===
    
    if (check_xy_) {
      // Phase 1: Checking XY Stability
      if (xy_ok) {
        if (!in_xy_tolerance_) {
          first_xy_tolerance_time_ = clock_->now();
          in_xy_tolerance_ = true;
        }
        
        double time_in_xy = (clock_->now() - first_xy_tolerance_time_).seconds();
        
        // If XY is stable for duration, switch to Yaw phase
        if (time_in_xy >= xy_stability_duration_) {
          check_xy_ = false;
          in_xy_tolerance_ = false; // Reset for cleanliness (though not used anymore)
          in_yaw_tolerance_ = false; // Reset for next phase
          // Fall through to return false (next loop will check yaw)
        }
      } else {
        in_xy_tolerance_ = false;
      }
      return false; // Still working on XY or just finished XY
    } else {
      // Phase 2: Checking Yaw Stability (XY is already assumed done)
      if (yaw_ok) {
        if (!in_yaw_tolerance_) {
          first_yaw_tolerance_time_ = clock_->now();
          in_yaw_tolerance_ = true;
        }

        double time_in_yaw = (clock_->now() - first_yaw_tolerance_time_).seconds();

        if (time_in_yaw >= yaw_stability_duration_) {
          // Both phases passed, now check velocity
          return fabs(velocity.angular.z) <= rot_stopped_velocity_ &&
                 hypot(velocity.linear.x, velocity.linear.y) <= trans_stopped_velocity_;
        }
      } else {
        in_yaw_tolerance_ = false;
      }
      return false;
    }

  } else {
    // === NON-STATEFUL MODE: Check XY & Yaw Simultaneously ===
    // Use xy_stability_duration for the combined check as requested
    
    if (xy_ok && yaw_ok) {
      if (!in_xy_tolerance_) {
        // Reuse xy variables for the combined state
        first_xy_tolerance_time_ = clock_->now();
        in_xy_tolerance_ = true;
      }

      double time_in_combined = (clock_->now() - first_xy_tolerance_time_).seconds();

      if (time_in_combined >= xy_stability_duration_) {
        // Tolerances & Duration met, now check velocity
        return fabs(velocity.angular.z) <= rot_stopped_velocity_ &&
               hypot(velocity.linear.x, velocity.linear.y) <= trans_stopped_velocity_;
      }
    } else {
      in_xy_tolerance_ = false;
    }
    return false;
  }
}

bool StableStoppedGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  double invalid_field = std::numeric_limits<double>::lowest();

  pose_tolerance.position.x = x_goal_tolerance_;
  pose_tolerance.position.y = y_goal_tolerance_;
  pose_tolerance.position.z = invalid_field;
  pose_tolerance.orientation =
    nav2_util::geometry_utils::orientationAroundZAxis(yaw_goal_tolerance_);

  vel_tolerance.linear.x = trans_stopped_velocity_;
  vel_tolerance.linear.y = trans_stopped_velocity_;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = rot_stopped_velocity_;

  return true;
}

rcl_interfaces::msg::SetParametersResult
StableStoppedGoalChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".x_goal_tolerance") {
        x_goal_tolerance_ = parameter.as_double();
      } else if (name == plugin_name_ + ".y_goal_tolerance") {
        y_goal_tolerance_ = parameter.as_double();
      } else if (name == plugin_name_ + ".yaw_goal_tolerance") {
        yaw_goal_tolerance_ = parameter.as_double();
      } else if (name == plugin_name_ + ".trans_stopped_velocity") {
        trans_stopped_velocity_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rot_stopped_velocity") {
        rot_stopped_velocity_ = parameter.as_double();
      } else if (name == plugin_name_ + ".xy_stability_duration") {
        xy_stability_duration_ = parameter.as_double();
      } else if (name == plugin_name_ + ".yaw_stability_duration") {
        yaw_stability_duration_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".stateful") {
        stateful_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_controller

PLUGINLIB_EXPORT_CLASS(nav2_controller::StableStoppedGoalChecker, nav2_core::GoalChecker)
