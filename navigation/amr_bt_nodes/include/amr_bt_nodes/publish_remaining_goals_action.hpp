// include/amr_bt_nodes/publish_remaining_path_action.hpp

#ifndef AMR_BT_NODES__PUBLISH_REMAINING_GOALS_ACTION_HPP_
#define AMR_BT_NODES__PUBLISH_REMAINING_GOALS_ACTION_HPP_

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace amr_bt_nodes
{

class PublishRemainingGoalsAction : public BT::SyncActionNode
{
public:
  typedef std::vector<geometry_msgs::msg::PoseStamped> Goals;

  PublishRemainingGoalsAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();

private:
  void initialize();
  
  BT::NodeStatus tick() override;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  rclcpp::Logger logger_{rclcpp::get_logger("PublishRemainingGoalsAction")}; // 로거 캐싱
  rclcpp::Time last_publish_time_;
  bool initialized_;
};

}  // namespace amr_bt_nodes

#endif  // AMR_BT_NODES__PUBLISH_REMAINING_GOALS_ACTION_HPP_