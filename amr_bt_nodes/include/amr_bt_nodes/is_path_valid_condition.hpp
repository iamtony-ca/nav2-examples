#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <nav_msgs/msg/path.hpp>

namespace amr_bt_nodes
{

class IsPathValidCondition : public BT::ConditionNode
{
public:
  IsPathValidCondition(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  // No member variables needed
};

}  // namespace amr_bt_nodes
