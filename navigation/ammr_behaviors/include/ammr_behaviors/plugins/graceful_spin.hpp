// Copyright (c) 2018 Intel Corporation
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

#ifndef AMMR_BEHAVIORS__PLUGINS__GRACEFUL_SPIN_HPP_
#define AMMR_BEHAVIORS__PLUGINS__GRACEFUL_SPIN_HPP_

#include <chrono>
#include <string>
#include <memory>

#include "ammr_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace ammr_behaviors
{
using SpinAction = nav2_msgs::action::Spin;

/**
 * @class ammr_behaviors::GracefulSpin
 * @brief An action server behavior for safely spinning in place using spatial resolution
 */
class GracefulSpin : public TimedBehavior<SpinAction>
{
  using CostmapInfoType = nav2_core::CostmapInfoType;

public:
  using SpinActionGoal = SpinAction::Goal;
  using SpinActionResult = SpinAction::Result;

  /**
   * @brief A constructor for ammr_behaviors::GracefulSpin
   */
  GracefulSpin();
  ~GracefulSpin();

  /**
   * @brief Initialization to run behavior
   * @param command Goal to execute
   * @return Status of behavior
   */
  ResultStatus onRun(const std::shared_ptr<const SpinActionGoal> command) override;

  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;

  /**
   * @brief Loop function to run behavior
   * @return Status of behavior
   */
  ResultStatus onCycleUpdate() override;

  /**
   * @brief Method to determine the required costmap info
   * @return costmap resources needed
   */
  CostmapInfoType getResourceInfo() override {return CostmapInfoType::LOCAL;}

protected:
  /**
   * @brief Check if pose is collision free using spatial resolution
   * @param distance Distance to check forward
   * @param cmd_vel current commanded velocity
   * @param pose2d Current pose
   * @return is collision free or not
   */
  bool isCollisionFree(
    const double & relative_yaw,
    const geometry_msgs::msg::Twist & cmd_vel,
    geometry_msgs::msg::Pose2D & pose2d);

  SpinAction::Feedback::SharedPtr feedback_;

  double min_rotational_vel_;
  double max_rotational_vel_;
  double rotational_acc_lim_;
  double cmd_yaw_;
  double prev_yaw_;
  double relative_yaw_;
  double simulate_ahead_time_;
  
  // 새로 추가된 파라미터 (공간 분할 해상도)
  double in_place_collision_resolution_;

  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;
};

}  // namespace ammr_behaviors

#endif  // AMMR_BEHAVIORS__PLUGINS__GRACEFUL_SPIN_HPP_