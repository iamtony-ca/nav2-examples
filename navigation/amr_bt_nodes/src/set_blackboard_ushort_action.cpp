// Copyright 2025 Your Name
// ... (License header) ...

#include "amr_bt_nodes/set_blackboard_ushort_action.hpp"

#include <string>

namespace amr_bt_nodes
{

SetBlackboardUShortAction::SetBlackboardUShortAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SetBlackboardUShortAction::providedPorts()
{
  return {
    // BehaviorTree.Cpp can convert string to unsigned short automatically
    BT::InputPort<unsigned short>("value", "Unsigned short value to write"),
    BT::InputPort<std::string>("output_key", "Name of the blackboard entry to write to")
  };
}

BT::NodeStatus SetBlackboardUShortAction::tick()
{
  std::string output_key;
  getInput("output_key", output_key);

  unsigned short value;
  getInput("value", value);

  config().blackboard->set<unsigned short>(output_key, value);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace amr_bt_nodes

#include "behaviortree_cpp/bt_factory.h"
// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<amr_bt_nodes::SetBlackboardUShortAction>("SetBlackboardUShort");
// }

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<amr_bt_nodes::SetBlackboardUShortAction>("SetBlackboardUShortAction");
}
