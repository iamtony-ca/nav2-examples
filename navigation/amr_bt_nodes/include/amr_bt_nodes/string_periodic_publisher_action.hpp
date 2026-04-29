// include/amr_bt_nodes/string_periodic_publisher_action.hpp

#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace amr_bt_nodes
{

/**
 * @brief 설정된 간격(interval)마다 N개의 메시지를 Burst 형태로 발행하는 SyncActionNode.
 * 이 작업은 명확한 종료 조건 없이 계속 반복됩니다.
 */
class StringPeriodicPublisherAction : public BT::SyncActionNode
{
public:
  StringPeriodicPublisherAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  // 마지막으로 'Burst' 발행이 일어난 시간을 기록
  rclcpp::Time last_burst_time_;
};

} // namespace amr_bt_nodes