#include "amr_bt_nodes/is_path_valid_custom_condition.hpp" 

#include <behaviortree_cpp/bt_factory.h>
#include "rclcpp/rclcpp.hpp"

namespace amr_bt_nodes
{

IsPathValidCustomCondition::IsPathValidCustomCondition(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{}

BT::PortsList IsPathValidCustomCondition::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("path", "Path to check for validity")
  };
}

BT::NodeStatus IsPathValidCustomCondition::tick()
{
  // getInput<T>()는 BT::Expected<T>를 반환합니다.
  auto path_res = getInput<nav_msgs::msg::Path>("path");

  // 1. 블랙보드에 'path' 키가 존재하지 않거나 타입이 맞지 않는 경우
  // 이 경우, 아직 경로가 없다고 판단하고 FAILURE를 반환합니다.
  if (!path_res) {
    // 예외를 던지지 않고 FAILURE를 반환하므로, BT는 Fallback 노드의 다음 자식을 실행하게 됩니다.
    return BT::NodeStatus::FAILURE;
  }

  // 2. 'path' 키는 존재하지만, 경로에 pose가 하나도 없는 경우 (경로가 비어있는 경우)
  if (path_res.value().poses.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  
  // 위의 모든 검사를 통과하면, 유효한 경로가 이미 존재하는 것입니다.
  return BT::NodeStatus::SUCCESS;
}

}  // namespace amr_bt_nodes

// Register this node
#include "behaviortree_cpp/bt_factory.h"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<amr_bt_nodes::IsPathValidCustomCondition>("IsPathValidCustomCondition");
}