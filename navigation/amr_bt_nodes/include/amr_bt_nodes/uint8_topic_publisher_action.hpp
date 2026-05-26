// include/amr_bt_nodes/uint8_topic_publisher_action.hpp

#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace amr_bt_nodes
{

/**
 * @brief Tick이 호출되면 지정된 uint8 값을 특정 토픽으로 N회 연속 발행하는 범용 SyncActionNode
 */
class Uint8TopicPublisherAction : public BT::SyncActionNode
{
public:
  Uint8TopicPublisherAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_;
};

} // namespace amr_bt_nodes