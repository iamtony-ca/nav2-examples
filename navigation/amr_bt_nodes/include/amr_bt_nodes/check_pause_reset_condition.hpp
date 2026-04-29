// check_pause_reset_condition.hpp

#pragma once

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <atomic>

namespace amr_bt_nodes
{

class CheckPauseResetCondition : public BT::ConditionNode
{
public:
  CheckPauseResetCondition(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      // ROS 2 node 핸들
      BT::InputPort<rclcpp::Node::SharedPtr>("node", "Shared ROS2 node"),

      // 구독할 flag 토픽
      BT::InputPort<std::string>("flag_topic", "/mission_flag", "Topic name to subscribe for flag"),

      // 논리적 래치: true가 한번 들어오면 false가 들어오기 전까지 SUCCESS 유지
      BT::InputPort<bool>("latch", true, "If true, SUCCESS state is latched until flag becomes false"),

      // DDS 래치(transient_local) 사용 여부
      BT::InputPort<bool>("transient_local", true, "Use DDS transient_local (latched QoS)"),

      // ★ 블랙보드 리셋 트리거(단 하나의 변수): true면 즉시 latched 상태 초기화
      BT::InputPort<bool>("reset", false, "If true, reset internal latched state immediately")
    };
  }

  BT::NodeStatus tick() override;

  // // 트리/서브트리 정지 시 강제 초기화(취소/프리엠프/중단 등)
  // void halt() override
  // {
  //   BT::ConditionNode::halt();
  //   last_flag_.store(false, std::memory_order_relaxed);
  // }

private:
  void flagCallback(const std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flag_sub_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  std::string flag_topic_;
  bool latch_{true};
  bool transient_local_{true};

      // 클래스 멤버
  // bool prev_reset_ {false};

  std::atomic<bool> last_flag_{false}; // 최신 flag 상태 (thread-safe)
};

} // namespace amr_bt_nodes
