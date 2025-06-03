#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

class TaskManagement : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  TaskManagement();

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trigger_sub_;

  void trigger_callback(const std_msgs::msg::String::SharedPtr msg);
  void send_goal();
  void goal_response_callback(GoalHandleNav::SharedPtr goal_handle);
  void feedback_callback(GoalHandleNav::SharedPtr,
                         const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void result_callback(const GoalHandleNav::WrappedResult & result);
};
