// Copyright 2025 Your Name
// ... (License header) ...

#ifndef AMR_BT_NODES__PLUGINS__ACTION__SET_BLACKBOARD_USHORT_ACTION_HPP_
#define AMR_BT_NODES__PLUGINS__ACTION__SET_BLACKBOARD_USHORT_ACTION_HPP_

#include <string>
#include "behaviortree_cpp/action_node.h"
#include <rclcpp/rclcpp.hpp>

namespace amr_bt_nodes
{
/**
 * @brief Sets an unsigned short (uint16_t) value on the blackboard.
 * This node can convert a literal string from the XML to an unsigned short.
 *
 * Example usage:
 * <SetBlackboardUShort output_key="error_code" value="305" />
 */
class SetBlackboardUShortAction : public BT::SyncActionNode
{
public:
  SetBlackboardUShortAction(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;
};
}  // namespace amr_bt_nodes

#endif  // AMR_BT_NODES__PLUGINS__ACTION__SET_BLACKBOARD_USHORT_ACTION_HPP_