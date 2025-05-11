// run_if_flag_true_decorator.hpp
#pragma once

#include <behaviortree_cpp/decorator_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <mutex>

namespace amr_bt_nodes
{

class RunIfFlagTrueDecorator : public BT::DecoratorNode
{
public:
  RunIfFlagTrueDecorator(const std::string &name, const BT::NodeConfiguration &config);
  ~RunIfFlagTrueDecorator() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("flag_topic", "/mission_flag", "Topic name to subscribe for flag"),
      BT::InputPort<rclcpp::Node::SharedPtr>("node", "Shared ROS2 node")
    };
  }

  BT::NodeStatus tick() override;
  void halt() override;

private:
  void flagCallback(const std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flag_sub_;
  std::string flag_topic_;
  std::atomic_bool flag_ok_;
  std::mutex flag_mutex_;

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
};

}  // namespace amr_bt_nodes
