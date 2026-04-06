// Copyright (c) 2018 Intel Corporation, 2019 Samsung Research America
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

#include <cmath>
#include <thread>
#include <algorithm>
#include <memory>
#include <utility>

#include "ammr_behaviors/plugins/graceful_spin.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/node_utils.hpp"

using namespace std::chrono_literals;

namespace ammr_behaviors
{

GracefulSpin::GracefulSpin()
: TimedBehavior<SpinAction>(),
  feedback_(std::make_shared<SpinAction::Feedback>()),
  min_rotational_vel_(0.0),
  max_rotational_vel_(0.0),
  rotational_acc_lim_(0.0),
  cmd_yaw_(0.0),
  prev_yaw_(0.0),
  relative_yaw_(0.0),
  simulate_ahead_time_(0.0),
  in_place_collision_resolution_(0.1) // Default 0.1 rad (약 5.7도)
{
}

GracefulSpin::~GracefulSpin() = default;

void GracefulSpin::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node,
    "simulate_ahead_time", rclcpp::ParameterValue(2.0));
  node->get_parameter("simulate_ahead_time", simulate_ahead_time_);

  nav2_util::declare_parameter_if_not_declared(
    node,
    "max_rotational_vel", rclcpp::ParameterValue(1.0));
  node->get_parameter("max_rotational_vel", max_rotational_vel_);

  nav2_util::declare_parameter_if_not_declared(
    node,
    "min_rotational_vel", rclcpp::ParameterValue(0.4));
  node->get_parameter("min_rotational_vel", min_rotational_vel_);

  nav2_util::declare_parameter_if_not_declared(
    node,
    "rotational_acc_lim", rclcpp::ParameterValue(3.2));
  node->get_parameter("rotational_acc_lim", rotational_acc_lim_);

  // 공간 분할 파라미터 로드
  nav2_util::declare_parameter_if_not_declared(
    node,
    "in_place_collision_resolution", rclcpp::ParameterValue(0.1));
  node->get_parameter("in_place_collision_resolution", in_place_collision_resolution_);
}

ResultStatus GracefulSpin::onRun(const std::shared_ptr<const SpinActionGoal> command)
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, local_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    return ResultStatus{Status::FAILED, SpinActionResult::TF_ERROR};
  }

  prev_yaw_ = tf2::getYaw(current_pose.pose.orientation);
  relative_yaw_ = 0.0;

  cmd_yaw_ = command->target_yaw;
  RCLCPP_INFO(
    logger_, "Turning %0.2f for graceful spin behavior.",
    cmd_yaw_);

  command_time_allowance_ = command->time_allowance;
  end_time_ = this->clock_->now() + command_time_allowance_;

  return ResultStatus{Status::SUCCEEDED, SpinActionResult::NONE};
}

ResultStatus GracefulSpin::onCycleUpdate()
{
  rclcpp::Duration time_remaining = end_time_ - this->clock_->now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN(
      logger_,
      "Exceeded time allowance before reaching the Spin goal - Exiting Spin");
    return ResultStatus{Status::FAILED, SpinActionResult::TIMEOUT};
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, local_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    return ResultStatus{Status::FAILED, SpinActionResult::TF_ERROR};
  }

  const double current_yaw = tf2::getYaw(current_pose.pose.orientation);

  double delta_yaw = current_yaw - prev_yaw_;
  if (abs(delta_yaw) > M_PI) {
    delta_yaw = copysign(2 * M_PI - abs(delta_yaw), prev_yaw_);
  }

  relative_yaw_ += delta_yaw;
  prev_yaw_ = current_yaw;

  feedback_->angular_distance_traveled = static_cast<float>(relative_yaw_);
  action_server_->publish_feedback(feedback_);

  double remaining_yaw = abs(cmd_yaw_) - abs(relative_yaw_);
  if (remaining_yaw < 1e-6) {
    stopRobot();
    return ResultStatus{Status::SUCCEEDED, SpinActionResult::NONE};
  }

  double vel = sqrt(2 * rotational_acc_lim_ * remaining_yaw);
  vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

  auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_vel->header.frame_id = robot_base_frame_;
  cmd_vel->header.stamp = clock_->now();
  cmd_vel->twist.angular.z = copysign(vel, cmd_yaw_);

  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = current_pose.pose.position.x;
  pose2d.y = current_pose.pose.position.y;
  pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

  if (!isCollisionFree(relative_yaw_, cmd_vel->twist, pose2d)) {
    stopRobot();
    RCLCPP_WARN(logger_, "Collision Ahead - Exiting Spin");
    return ResultStatus{Status::FAILED, SpinActionResult::COLLISION_AHEAD};
  }

  vel_pub_->publish(std::move(cmd_vel));

  return ResultStatus{Status::RUNNING, SpinActionResult::NONE};
}

bool GracefulSpin::isCollisionFree(
  const double & relative_yaw,
  const geometry_msgs::msg::Twist & cmd_vel,
  geometry_msgs::msg::Pose2D & pose2d)
{
  // 1. 현재 속도를 유지했을 때 'simulate_ahead_time' 동안 회전할 수 있는 최대 각도
  double sim_angle_ahead = cmd_vel.angular.z * simulate_ahead_time_;

  // 2. 최종 목표까지 실제로 남은 각도 크기 (음수 방지를 위해 std::max 사용)
  double remaining_yaw_mag = std::max(0.0, std::abs(cmd_yaw_) - std::abs(relative_yaw));

  // 3. 검사해야 할 실제 각도: (시뮬레이션 각도 vs 남은 각도) 중 더 작은 것을 선택하되 부호 유지
  double actual_check_angle = std::copysign(
    std::min(std::abs(sim_angle_ahead), remaining_yaw_mag), 
    cmd_vel.angular.z);

  // 4. 공간 분할 해상도에 따라 검사 스텝 수 계산
  size_t num_steps = static_cast<size_t>(std::ceil(std::abs(actual_check_angle) / in_place_collision_resolution_));
  num_steps = std::max(static_cast<size_t>(1), num_steps);

  geometry_msgs::msg::Pose2D init_pose = pose2d;
  bool fetch_data = true; 

  // 5. 정해진 스텝 수만큼 각도를 쪼개어 가며 촘촘히 충돌 검사 수행
  for (size_t i = 1; i <= num_steps; ++i) {
    double step_ratio = static_cast<double>(i) / static_cast<double>(num_steps);
    double yaw_step = actual_check_angle * step_ratio;

    pose2d.theta = init_pose.theta + yaw_step;

    if (!local_collision_checker_->isCollisionFree(pose2d, fetch_data)) {
      return false; 
    }
    fetch_data = false;
  }

  return true; 
}

}  // namespace ammr_behaviors

#include "pluginlib/class_list_macros.hpp"
// ★ 플러그인으로 등록할 때는 새로 변경한 클래스 이름을 사용합니다.
PLUGINLIB_EXPORT_CLASS(ammr_behaviors::GracefulSpin, nav2_core::Behavior)