// include/amr_bt_nodes/error_code_publisher_action.hpp

#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp" // uint16 타입에 맞는 헤더로 변경

namespace amr_bt_nodes
{

/**
 * @brief Tick이 호출되면 그 즉시, 지정된 횟수만큼 에러 코드(uint16)를 발행하는 SyncActionNode
 */
class ErrorCodePublisherAction : public BT::SyncActionNode
{
public:
  ErrorCodePublisherAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  // 메시지 타입 변경: std_msgs::msg::UInt16
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr publisher_;
};

} // namespace amr_bt_nodes