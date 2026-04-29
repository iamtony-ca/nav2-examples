#pragma once

#include <behaviortree_cpp/condition_node.h>
#include <nav_msgs/msg/path.hpp>

namespace amr_bt_nodes
{

class IsPathValidCustomCondition : public BT::ConditionNode
{
public:
  IsPathValidCustomCondition(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace amr_bt_nodes