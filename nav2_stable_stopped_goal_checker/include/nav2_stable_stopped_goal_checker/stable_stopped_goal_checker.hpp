/*
 * Copyright (c) 2024, Custom Robotics
 * ... (License 생략) ...
 */

#ifndef NAV2_CONTROLLER__PLUGINS__STABLE_STOPPED_GOAL_CHECKER_HPP_
#define NAV2_CONTROLLER__PLUGINS__STABLE_STOPPED_GOAL_CHECKER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/goal_checker.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "nav_msgs/msg/path.hpp" // Path 메시지 추가

namespace nav2_controller
{

/**
 * @class StableStoppedGoalChecker
 * @brief Checks goal with separate X/Y tolerances, stability durations, and remaining path length check.
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

protected:
  // --- Parameters ---
  double x_goal_tolerance_;
  double y_goal_tolerance_;
  double yaw_goal_tolerance_;
  
  double rot_stopped_velocity_;
  double trans_stopped_velocity_;

  double xy_stability_duration_;
  double yaw_stability_duration_;

  bool stateful_;

  // Path check parameters
  double path_tolerance_multiplier_; // e.g. 1.5
  std::string path_topic_;           // e.g. "/plan"

  // --- State Variables ---
  bool check_xy_;
  bool in_xy_tolerance_;
  bool in_yaw_tolerance_;
  rclcpp::Time first_xy_tolerance_time_;
  rclcpp::Time first_yaw_tolerance_time_;
  rclcpp::Clock::SharedPtr clock_;

  // --- Path Subscription ---
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  nav_msgs::msg::Path::SharedPtr current_path_;
  std::mutex path_mutex_;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::string plugin_name_;

  /**
   * @brief Callback for path subscription
   */
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

  /**
   * @brief Dynamic Parameter Callback
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__STABLE_STOPPED_GOAL_CHECKER_HPP_