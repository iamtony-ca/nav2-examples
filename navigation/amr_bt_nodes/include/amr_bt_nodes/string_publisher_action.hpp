// include/amr_bt_nodes/string_publisher_action.hpp

#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace amr_bt_nodes
{

/**
 * @brief Tick이 호출되면 그 즉시, 지정된 횟수만큼 문자열 메시지를 연속 발행하는 SyncActionNode
 */
class StringPublisherAction : public BT::SyncActionNode
{
public:
  StringPublisherAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  // SyncActionNode의 핵심. 모든 로직이 이 안에서 완료.
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

} // namespace amr_bt_nodes