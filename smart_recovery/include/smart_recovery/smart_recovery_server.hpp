// smart_recovery_server.hpp
// =============================================================
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <atomic>

#include "robot_interfaces/action/smart_recovery.hpp"      // SmartRecovery.action
#include "robot_interfaces/srv/smart_recovery_path.hpp"    // SmartRecoveryPath.srv

class SmartRecoveryServer : public rclcpp::Node
{
  using SmartRecovery       = robot_interfaces::action::SmartRecovery;
  using GoalHandleRecovery  = rclcpp_action::ServerGoalHandle<SmartRecovery>;
  using SmartRecoveryPath   = robot_interfaces::srv::SmartRecoveryPath;

public:
  explicit SmartRecoveryServer(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions());

private:
  // Action callbacks
  rclcpp_action::GoalResponse   handle_goal(const rclcpp_action::GoalUUID & uuid,
                                            const std::shared_ptr<const SmartRecovery::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleRecovery> handle);
  void                          handle_accepted(const std::shared_ptr<GoalHandleRecovery> handle);
  void                          execute(const std::shared_ptr<GoalHandleRecovery> handle);

  // Topic callback
  void doneCallback(const std_msgs::msg::Bool::SharedPtr msg);

  // Members ---------------------------------------------------
  // Callback groups
  rclcpp::CallbackGroup::SharedPtr cb_action_;
  rclcpp::CallbackGroup::SharedPtr cb_service_client_;
  rclcpp::CallbackGroup::SharedPtr cb_topic_;

  // Interfaces
  rclcpp_action::Server<SmartRecovery>::SharedPtr      action_server_;
  rclcpp::Client<SmartRecoveryPath>::SharedPtr         planner_client_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr    path_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr done_sub_;

  // State flags
  std::atomic<bool> done_flag_{false};
  std::atomic<bool> cancel_requested_{false};
};