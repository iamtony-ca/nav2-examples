#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"

namespace amr_bt_nodes
{

class PathPublisherAction : public BT::SyncActionNode
{
public:
  PathPublisherAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  
  // 마지막으로 토픽을 발행한 시간을 저장하기 위한 멤버 변수
  rclcpp::Time last_publish_time_; 
};

}  // namespace amr_bt_nodes