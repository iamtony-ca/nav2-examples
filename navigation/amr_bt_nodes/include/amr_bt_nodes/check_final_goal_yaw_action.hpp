#ifndef AMR_BT_NODES__CHECK_FINAL_GOAL_YAW_ACTION_HPP_
#define AMR_BT_NODES__CHECK_FINAL_GOAL_YAW_ACTION_HPP_

#include <string>
#include <vector>
#include <cmath>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace amr_bt_nodes
{

/**
 * @brief Checks if the yaw difference between the input pose and the last goal
 * is within a tight threshold (0.2 degrees).
 */
class CheckFinalGoalYawAction : public BT::SyncActionNode
{
public:
  CheckFinalGoalYawAction(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
        "input_goals", "List of goals to check the last one"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "input_pose", "Current pose of the robot"),
      BT::OutputPort<double>(
        "result", "1.0 if within 0.2 degrees, 0.0 otherwise"),
      BT::OutputPort<double>(
        "difference_radian", "Yaw difference in radians (normalized)")
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace amr_bt_nodes

#endif  // AMR_BT_NODES__CHECK_FINAL_GOAL_YAW_ACTION_HPP_