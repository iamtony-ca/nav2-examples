/*
 * Copyright (c) 2024, Custom Robotics
 * ... (License 생략) ...
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
  path_tolerance_multiplier_(1.5), // 기본값 1.5배
  path_topic_("/plan"),            // 기본 토픽명
  check_xy_(true),
  in_xy_tolerance_(false),
  in_yaw_tolerance_(false)
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

  // 1. Declare Standard Parameters
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
  
  // 2. Declare Stability Parameters
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".xy_stability_duration", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".yaw_stability_duration", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".stateful", rclcpp::ParameterValue(true));

  // 3. Declare Path Check Parameters (NEW)
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".path_tolerance_multiplier", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".path_topic", rclcpp::ParameterValue("/plan"));

  // 4. Get Parameters
  node->get_parameter(plugin_name + ".x_goal_tolerance", x_goal_tolerance_);
  node->get_parameter(plugin_name + ".y_goal_tolerance", y_goal_tolerance_);
  node->get_parameter(plugin_name + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node->get_parameter(plugin_name + ".trans_stopped_velocity", trans_stopped_velocity_);
  node->get_parameter(plugin_name + ".rot_stopped_velocity", rot_stopped_velocity_);
  node->get_parameter(plugin_name + ".xy_stability_duration", xy_stability_duration_);
  node->get_parameter(plugin_name + ".yaw_stability_duration", yaw_stability_duration_);
  node->get_parameter(plugin_name + ".stateful", stateful_);
  node->get_parameter(plugin_name + ".path_tolerance_multiplier", path_tolerance_multiplier_);
  node->get_parameter(plugin_name + ".path_topic", path_topic_);

  // 5. Initialize Path Subscription
  rclcpp::QoS qos(1);
  qos.transient_local(); // Plan topics usually act like latched topics
  path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    path_topic_, qos,
    std::bind(&StableStoppedGoalChecker::pathCallback, this, _1));

  // 6. Dynamic Parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&StableStoppedGoalChecker::dynamicParametersCallback, this, _1));
}

void StableStoppedGoalChecker::reset()
{
  check_xy_ = true;
  in_xy_tolerance_ = false;
  in_yaw_tolerance_ = false;
}

void StableStoppedGoalChecker::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  current_path_ = msg;
}

bool StableStoppedGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & velocity)
{
  // 1. Calculate Errors (Euclidean & Yaw)
  double dx = fabs(query_pose.position.x - goal_pose.position.x);
  double dy = fabs(query_pose.position.y - goal_pose.position.y);
  double dyaw = fabs(angles::shortest_angular_distance(
    tf2::getYaw(query_pose.orientation),
    tf2::getYaw(goal_pose.orientation)));

  bool xy_ok = (dx <= x_goal_tolerance_) && (dy <= y_goal_tolerance_);
  bool yaw_ok = (dyaw <= yaw_goal_tolerance_);

  // ---------------------------------------------------------
  // [FIXED & OPTIMIZED] Global Path Distance Check Logic
  // ---------------------------------------------------------
  if (xy_ok) {
    std::lock_guard<std::mutex> lock(path_mutex_);
    
    // 경로 데이터가 유효할 때만 검사
    if (current_path_ && !current_path_->poses.empty()) {
      
      double max_tolerance = std::max(x_goal_tolerance_, y_goal_tolerance_);
      double dist_threshold = max_tolerance * path_tolerance_multiplier_;
      
      // 검색 범위: Tolerance보다 약간 여유 있게 잡음 (1.2배)
      double search_limit_sq = pow(max_tolerance * 1.2, 2);

      bool goal_confirmed_by_path = false;

      // [핵심 변경] 단순히 가까운 점을 찾는 게 아니라, 
      // "남은 거리가 짧은(도착으로 인정되는) 점"이 내 주변에 있는지 확인합니다.
      for (size_t i = 0; i < current_path_->poses.size(); ++i) {
        
        // 1. 현재 인덱스(i)의 점이 로봇과 가까운가?
        double p_dx = current_path_->poses[i].pose.position.x - query_pose.position.x;
        double p_dy = current_path_->poses[i].pose.position.y - query_pose.position.y;
        double dist_sq = p_dx * p_dx + p_dy * p_dy;

        if (dist_sq <= search_limit_sq) {
          // 2. 가깝다면, 이 점을 기준으로 했을 때 남은 경로가 짧은가? (Early Exit 적용)
          double accumulated_dist = 0.0;
          bool is_short_path = true;

          // i부터 끝까지 거리 계산 (임계값 넘으면 즉시 중단)
          for (size_t j = i; j < current_path_->poses.size() - 1; ++j) {
            accumulated_dist += nav2_util::geometry_utils::euclidean_distance(
              current_path_->poses[j], current_path_->poses[j+1]);
            
            if (accumulated_dist > dist_threshold) {
              is_short_path = false;
              break; // 너무 기니까 이 점은 Goal이 아님 (교차점 등일 수 있음)
            }
          }

          if (is_short_path) {
            // "내 주변에 있고" AND "남은 경로도 짧은" 점을 찾았습니다!
            // 즉, 우리는 교차점이 아니라 진짜 Goal에 와있는 것입니다.
            goal_confirmed_by_path = true;
            break; // 더 이상 검사할 필요 없이 성공 확정
          }
        }
      }

      // 내 주변의 모든 점을 검사했는데도, 남은 경로가 짧은 점이 하나도 없다면?
      // -> 우리는 Goal 위치(좌표)에는 있지만, 경로상으로는 아직 멀었다(교차점이다)는 뜻입니다.
      if (!goal_confirmed_by_path) {
        xy_ok = false;
      }
    }
  }
  // ---------------------------------------------------------

  // 2. Logic based on 'stateful' parameter
  if (stateful_) {
    // === STATEFUL MODE ===
    if (check_xy_) {
      // Phase 1: Checking XY
      if (xy_ok) {
        if (!in_xy_tolerance_) {
          first_xy_tolerance_time_ = clock_->now();
          in_xy_tolerance_ = true;
        }
        
        double time_in_xy = (clock_->now() - first_xy_tolerance_time_).seconds();
        
        if (time_in_xy >= xy_stability_duration_) {
          check_xy_ = false;
          in_xy_tolerance_ = false; 
          in_yaw_tolerance_ = false;
        }
      } else {
        in_xy_tolerance_ = false;
      }
      return false; // Still processing XY or just finished XY
    } else {
      // Phase 2: Checking Yaw
      if (yaw_ok) {
        if (!in_yaw_tolerance_) {
          first_yaw_tolerance_time_ = clock_->now();
          in_yaw_tolerance_ = true;
        }

        double time_in_yaw = (clock_->now() - first_yaw_tolerance_time_).seconds();

        if (time_in_yaw >= yaw_stability_duration_) {
          // Final check: Velocity
          return fabs(velocity.angular.z) <= rot_stopped_velocity_ &&
                 hypot(velocity.linear.x, velocity.linear.y) <= trans_stopped_velocity_;
        }
      } else {
        in_yaw_tolerance_ = false;
      }
      return false;
    }

  } else {
    // === NON-STATEFUL MODE ===
    if (xy_ok && yaw_ok) {
      if (!in_xy_tolerance_) {
        first_xy_tolerance_time_ = clock_->now();
        in_xy_tolerance_ = true;
      }

      double time_in_combined = (clock_->now() - first_xy_tolerance_time_).seconds();

      if (time_in_combined >= xy_stability_duration_) {
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
      } else if (name == plugin_name_ + ".path_tolerance_multiplier") {
        path_tolerance_multiplier_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".stateful") {
        stateful_ = parameter.as_bool();
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == plugin_name_ + ".path_topic") {
        // Topic name change requires restart usually, but we can try to update subs
        // For simplicity in this snippet, we just update the variable.
        path_topic_ = parameter.as_string();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_controller

PLUGINLIB_EXPORT_CLASS(nav2_controller::StableStoppedGoalChecker, nav2_core::GoalChecker)