// include/amr_bt_nodes/bool_publisher_action.hpp

#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp" // String -> Bool 메시지 헤더로 변경

namespace amr_bt_nodes
{

/**
 * @brief Tick이 호출되면 그 즉시, 지정된 횟수만큼 boolean 메시지를 연속 발행하는 SyncActionNode
 */
class BoolPublisherAction : public BT::SyncActionNode
{
public:
  BoolPublisherAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  // SyncActionNode의 핵심. 모든 로직이 이 안에서 완료됩니다.
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_; // String -> Bool 타입으로 변경
};

} // namespace amr_bt_nodes