// my_node.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <example_interfaces/action/fibonacci.hpp>

class MyNode : public rclcpp::Node
{
public:
  using Fibonacci = example_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;
  using GoalHandleFibonacciClient = rclcpp_action::ClientGoalHandle<Fibonacci>;

  MyNode();

private:
  // Callback Groups
  rclcpp::CallbackGroup::SharedPtr cb_group_service_;
  rclcpp::CallbackGroup::SharedPtr cb_group_service_client_;
  rclcpp::CallbackGroup::SharedPtr cb_group_action_;
  rclcpp::CallbackGroup::SharedPtr cb_group_action_client_;
  rclcpp::CallbackGroup::SharedPtr cb_group_timer_;
  rclcpp::CallbackGroup::SharedPtr cb_group_topic_;

  // // Service Server
  // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
  // void handle_service(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  //                     std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // // Service Client
  // rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr srv_client_;
  // void call_service();

  // // Action Server
  // rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
  // rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
  //                                         const std::shared_ptr<const Fibonacci::Goal> goal);
  // rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFibonacci> goal_handle);
  // void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle);
  // void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle);

  // // Action Client
  // rclcpp_action::Client<Fibonacci>::SharedPtr action_client_;
  // void send_goal();

  // Topic
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  void topic_callback(const std_msgs::msg::String::SharedPtr msg);
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

  // // Timer
  // rclcpp::TimerBase::SharedPtr timer_;
  // void timer_callback();
};