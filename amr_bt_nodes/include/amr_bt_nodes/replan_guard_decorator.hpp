// replan_guard_decorator.hpp
#pragma once

#include <behaviortree_cpp/decorator_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <atomic>
#include <string>
#include <vector>

namespace amr_bt_nodes
{

class ReplanGuardDecorator : public BT::DecoratorNode
{
public:
  ReplanGuardDecorator(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<rclcpp::Node::SharedPtr>("node", "Shared ROS node"),
      BT::InputPort<std::string>("flag_topic", "/replan_flag", "Bool flag topic"),
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
        "goals", "Target goals sequence"),
      // >> 변경된 부분: 기본값을 "normal"로 설정 <<
      BT::InputPort<std::string>(
        "bt_blackboard_status", "normal", "Status from the blackboard, e.g., 'recovery_mode'"),
      // Output Port: 자식 노드 실행 시, 블랙보드 상태를 다시 'normal'로 씀
      BT::OutputPort<std::string>(
        "bt_blackboard_status", "Set to 'normal' when triggered")
    };
  }

  BT::NodeStatus tick() override;
  void halt() override;

private:
  void flagCallback(const std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flag_sub_;
  
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  std::string flag_topic_;

  std::atomic<bool> triggered_{true};

  bool has_last_goals_{false};
  std::vector<geometry_msgs::msg::PoseStamped> last_goals_;
};

}  // namespace amr_bt_nodes