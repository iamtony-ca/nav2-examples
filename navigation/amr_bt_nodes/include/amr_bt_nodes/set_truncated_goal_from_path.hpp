#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>


namespace amr_bt_nodes
{

class SetTruncatedGoalFromPath : public BT::SyncActionNode
{
public:
  SetTruncatedGoalFromPath(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("short_path"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("alt_goal")
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace amr_bt_nodes
