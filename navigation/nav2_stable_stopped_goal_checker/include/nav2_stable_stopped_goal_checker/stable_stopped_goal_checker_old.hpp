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

#ifndef NAV2_CONTROLLER__PLUGINS__STABLE_STOPPED_GOAL_CHECKER_HPP_
#define NAV2_CONTROLLER__PLUGINS__STABLE_STOPPED_GOAL_CHECKER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex> // [추가] Thread safety를 위해 필요


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/goal_checker.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "nav_msgs/msg/path.hpp" // [추가] Path 메시지 타입

namespace nav2_controller
{

/**
 * @class StableStoppedGoalChecker
 * @brief Checks if goal is reached with separate X/Y/Yaw tolerances, split stability durations, and stateful logic.
 */
class StableStoppedGoalChecker : public nav2_core::GoalChecker
{
public:
  StableStoppedGoalChecker();
  
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
    
  void reset() override;
  
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;
    
  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;


/**
   * @brief Graceful Controller가 XY 완료(Latch) 상태를 확인하기 위한 함수
   * @return true if XY check is passed and latched (now checking Yaw)
   */
  bool isXYLatched() const {
    // stateful 모드일 때 check_xy_가 false라면 XY 검사는 통과했다는 뜻
    return stateful_ && !check_xy_;
  }

  /**
   * @brief X축 허용 오차 Getter
   */
  double getXGoalTolerance() const {
    return x_goal_tolerance_;
  }

  /**
   * @brief Y축 허용 오차 Getter
   */
  double getYGoalTolerance() const {
    return y_goal_tolerance_;
  }





protected:
  // Tolerance parameters
  double x_goal_tolerance_;
  double y_goal_tolerance_;
  double yaw_goal_tolerance_;
  
  // Velocity parameters
  double rot_stopped_velocity_;
  double trans_stopped_velocity_;

  // Time stability parameters
  double xy_stability_duration_;
  double yaw_stability_duration_;

  // Logic control parameters
  bool stateful_;

  // State variables
  bool check_xy_; // If true, we are checking XY. If false (and stateful), we are checking Yaw.
  
  // Time tracking variables
  bool in_xy_tolerance_;
  bool in_yaw_tolerance_;
  rclcpp::Time first_xy_tolerance_time_;
  rclcpp::Time first_yaw_tolerance_time_;
  rclcpp::Clock::SharedPtr clock_;

// [추가] Path Subscription 관련 변수
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  nav_msgs::msg::Path::SharedPtr current_path_;
  std::mutex path_mutex_; 
  std::string path_topic_; // 파라미터로 받기 위해 추가


  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::string plugin_name_;

  /**
   * @brief Callback executed when a parameter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // [추가] Path Callback 함수
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__STABLE_STOPPED_GOAL_CHECKER_HPP_
