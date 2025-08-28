// trigger_planner_goals_decorator.hpp
#pragma once

#include <behaviortree_cpp/decorator_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <atomic>
#include <vector> // std::vector를 사용하기 위해 포함

namespace amr_bt_nodes
{

class TriggerPlannerGoalsDecorator : public BT::DecoratorNode
{
public:
  TriggerPlannerGoalsDecorator(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<rclcpp::Node::SharedPtr>("node", "Shared ROS node"),
      BT::InputPort<std::string>("flag_topic", "/replan_flag", "Bool flag topic"),
      // InputPort를 'goal'에서 'goals'로 변경하고, 타입을 vector로 수정
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
        "goals", "Target goals sequence for NavigateThroughPoses")
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

  std::atomic<bool> triggered_{true};  // 최초 1회는 실행
  
  // goals 시퀀스 비교를 위한 멤버 변수들
  bool has_last_goals_{false};
  std::vector<geometry_msgs::msg::PoseStamped> last_goals_;
};

}  // namespace amr_bt_nodes