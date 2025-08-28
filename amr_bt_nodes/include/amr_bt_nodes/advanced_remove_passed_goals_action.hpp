#pragma once

#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
// #include "nav2_behavior_tree/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace amr_bt_nodes
{

/**
 * @brief A BT node to remove passed goals from a list of goals based on progress along a path.
 *
 * This node implements a robust logic to determine if waypoints have been passed.
 * It operates on a dense path from a planner and uses a dynamic, distance-based
 * search window to find the robot's progress. It is resilient to path replanning,
 * large deviations from the path, and oscillations near the goal.
 */
class AdvancedRemovePassedGoalsAction : public BT::StatefulActionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::AdvancedRemovePassedGoalsAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  AdvancedRemovePassedGoalsAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts();

protected:
  /**
   * @brief The main execution routine.
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus onRunning() override;

  /**
   * @brief The routine called when the node is ticked for the first time.
   * @return BT::NodeStatus Status of the node after initialization
   */
  BT::NodeStatus onStart() override;

  /**
   * @brief The routine called when the node is halted.
   */
  void onHalted() override;

private:
  // ROS-related members
  rclcpp::Node::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // Parameters from BT XML
  double search_radius_;
  unsigned int hysteresis_indices_;
  double max_path_deviation_;
  std::string global_frame_;
  std::string robot_base_frame_;
  rclcpp::Duration transform_tolerance_;

  // Node state variables
  size_t last_path_hash_;
  std::map<size_t, size_t> waypoint_to_path_index_map_;
  size_t last_progress_index_;
  std::vector<geometry_msgs::msg::PoseStamped> initial_goals_;

  // Helper methods
  bool updateRobotPose(geometry_msgs::msg::PoseStamped & robot_pose);
  bool isPathUpdated(const nav_msgs::msg::Path & path);
  void createWaypointIndexMapping(
    const nav_msgs::msg::Path & path,
    const std::vector<geometry_msgs::msg::PoseStamped> & goals);
  size_t findCurrentProgressIndex(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & robot_pose);
};

}  // namespace amr_bt_nodes