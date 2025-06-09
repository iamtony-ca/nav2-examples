#include "amr_bt_nodes/is_path_valid_condition.hpp"
#include <behaviortree_cpp/bt_factory.h>

namespace amr_bt_nodes
{

IsPathValidCondition::IsPathValidCondition(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{}

BT::PortsList IsPathValidCondition::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("path", "Path to validate")
  };
}

BT::NodeStatus IsPathValidCondition::tick()
{
  nav_msgs::msg::Path path;
  if (!getInput("path", path)) {
    throw BT::RuntimeError("IsPathValidCondition: missing required input [path]");
  }

  if (!path.poses.empty()) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace amr_bt_nodes

// Register this node
#include "behaviortree_cpp/bt_factory.h"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<amr_bt_nodes::IsPathValidCondition>("IsPathValidCondition");
}