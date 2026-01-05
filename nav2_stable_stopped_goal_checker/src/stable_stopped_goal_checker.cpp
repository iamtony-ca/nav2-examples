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
  // [NEW] Global Path Distance Check Logic (Fixed & Optimized)
  // ---------------------------------------------------------
  if (xy_ok) {
    std::lock_guard<std::mutex> lock(path_mutex_);
    
    // 경로 데이터가 수신된 상태에서만 검사
    if (current_path_ && !current_path_->poses.empty()) {
      
      // A. [수정됨] 현재 로봇 위치에서 가장 가까운 경로상의 인덱스 직접 찾기
      size_t closest_pose_idx = 0;
      double min_dist_sq = std::numeric_limits<double>::max();

      for (size_t i = 0; i < current_path_->poses.size(); ++i) {
        double p_dx = current_path_->poses[i].pose.position.x - query_pose.position.x;
        double p_dy = current_path_->poses[i].pose.position.y - query_pose.position.y;
        double dist_sq = p_dx * p_dx + p_dy * p_dy;

        if (dist_sq < min_dist_sq) {
          min_dist_sq = dist_sq;
          closest_pose_idx = i;
        }
      }

      // ------------------------------------------------------------------
      // [최적화] Early Exit Path Length Calculation
      // ------------------------------------------------------------------
      double max_tolerance = std::max(x_goal_tolerance_, y_goal_tolerance_);
      double dist_threshold = max_tolerance * path_tolerance_multiplier_;
      
      double accumulated_dist = 0.0;
      bool path_is_long = false;

      // closest_pose_idx 부터 경로 끝까지 순회
      for (size_t i = closest_pose_idx; i < current_path_->poses.size() - 1; ++i) {
        double d = nav2_util::geometry_utils::euclidean_distance(
          current_path_->poses[i], current_path_->poses[i + 1]);
        
        accumulated_dist += d;

        // [핵심] 누적 거리가 이미 임계값을 넘었다면? 더 계산할 필요 없이 "도착 아님" 판정
        if (accumulated_dist > dist_threshold) {
          path_is_long = true;
          break; // Loop 탈출! (연산량 절약)
        }
      }

      // C. 비교
      if (path_is_long) {
        // 남은 경로가 임계값보다 깁니다. (교차점 통과 중으로 판단)
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
      return false;
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