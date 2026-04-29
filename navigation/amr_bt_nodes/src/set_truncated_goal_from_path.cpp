#include "amr_bt_nodes/set_truncated_goal_from_path.hpp"

namespace amr_bt_nodes
{

BT::NodeStatus SetTruncatedGoalFromPath::tick()
{
  nav_msgs::msg::Path short_path;
  if (!getInput("short_path", short_path)) {
    RCLCPP_ERROR(rclcpp::get_logger("SetTruncatedGoalFromPath"), "Missing input [short_path]");
    return BT::NodeStatus::FAILURE;
  }

  if (short_path.poses.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("SetTruncatedGoalFromPath"), "short_path is empty");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped last_pose = short_path.poses.back();
  last_pose.header.frame_id = short_path.header.frame_id;
  setOutput("alt_goal", last_pose);
//   RCLCPP_INFO(rclcpp::get_logger("SetTruncatedGoalFromPath"),
//               "Set alt_goal from truncated path endpoint [%.2f, %.2f]",
//               last_pose.pose.position.x, last_pose.pose.position.y);
  RCLCPP_INFO(rclcpp::get_logger("SetTruncatedGoalFromPath"),
                "Set alt_goal from truncated path endpoint [%.2f, %.2f] with frame_id: %s",
                last_pose.pose.position.x,
                last_pose.pose.position.y,
                last_pose.header.frame_id.c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace amr_bt_nodes


#include "behaviortree_cpp/bt_factory.h"
// #include "amr_bt_nodes/set_truncated_goal_from_path.hpp"

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<amr_bt_nodes::SetTruncatedGoalFromPath>("SetTruncatedGoalFromPath");
// }

// ── BT plugin registration ---------------------------------------
extern "C"
void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<amr_bt_nodes::SetTruncatedGoalFromPath>("SetTruncatedGoalFromPath");
}
