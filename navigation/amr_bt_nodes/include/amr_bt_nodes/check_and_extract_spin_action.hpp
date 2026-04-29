#ifndef AMR_BT_NODES__CHECK_AND_EXTRACT_SPIN_ACTION_HPP_
#define AMR_BT_NODES__CHECK_AND_EXTRACT_SPIN_ACTION_HPP_

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
 * @brief Checks if the distance between the first two goals is within a tolerance.
 * If so, extracts the yaw difference and returns SUCCESS. Does NOT mutate the goals.
 */
class CheckAndExtractSpinAction : public BT::SyncActionNode
{
public:
  CheckAndExtractSpinAction(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
        "goals", "List of waypoints (goals)"),
      BT::InputPort<double>(
        "distance_tolerance", 0.05, "Distance threshold to consider as pure rotation"),
      BT::OutputPort<double>(
        "spin_angle", "Angle to spin in radians")
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace amr_bt_nodes

#endif  // AMR_BT_NODES__CHECK_AND_EXTRACT_SPIN_ACTION_HPP_
