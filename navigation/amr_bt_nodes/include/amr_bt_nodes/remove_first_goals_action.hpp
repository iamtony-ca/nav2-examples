#ifndef AMR_BT_NODES__REMOVE_FIRST_GOALS_ACTION_HPP_
#define AMR_BT_NODES__REMOVE_FIRST_GOALS_ACTION_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace amr_bt_nodes
{

using Goals = std::vector<geometry_msgs::msg::PoseStamped>;

class RemoveFirstGoalsAction : public BT::SyncActionNode
{
public:
  RemoveFirstGoalsAction(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Goals>("input_goals", "Original list of goals"),
      // [추가] 중복 판단을 위한 거리 임계값 (기본값 0.01m)
      BT::InputPort<double>("distance_threshold", 0.01, "Epsilon threshold to consider goals as same"),
      BT::OutputPort<Goals>("remaining_goals", "List of goals with one or two elements removed")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
};

}  // namespace amr_bt_nodes

#endif  // AMR_BT_NODES__REMOVE_FIRST_GOALS_ACTION_HPP_