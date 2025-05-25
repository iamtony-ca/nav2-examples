// trigger_planner_decorator.hpp
#pragma once

#include <behaviortree_cpp/decorator_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <mutex>

namespace amr_bt_nodes
{

class TriggerPlannerDecorator : public BT::DecoratorNode
{
public:
  TriggerPlannerDecorator(const std::string & name, const BT::NodeConfiguration & config);
  ~TriggerPlannerDecorator() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<rclcpp::Node::SharedPtr>("node", "Shared ROS node"),
      BT::InputPort<std::string>("flag_topic", "/decor_flag", "Bool flag topic"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Target goal pose")
    };
  }

  BT::NodeStatus tick() override;
  void halt() override;

private:
  void flagCallback(const std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flag_sub_;
  std::string flag_topic_;

  std::mutex mutex_;
  bool last_flag_ = false;
  bool triggered_ = true;  // 최초 1회는 실행
  bool has_last_goal_ = false;
  geometry_msgs::msg::PoseStamped last_goal_;
};

class SharedExecutor
{
public:
  static void ensureExecutorStarted(rclcpp::Node::SharedPtr node)
  {
    static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
    static std::once_flag flag;
    std::call_once(flag, [&]() {
      executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
      executor->add_node(node);
      std::thread([]() {
        executor->spin();
      }).detach();
    });
  }
};

}  // namespace amr_bt_nodes