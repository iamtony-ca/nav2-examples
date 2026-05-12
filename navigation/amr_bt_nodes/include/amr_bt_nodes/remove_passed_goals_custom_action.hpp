// Copyright (c) 2021 Samsung Research America
// ... (라이선스 생략) ...

#ifndef AMR_BT_NODES__REMOVE_PASSED_GOALS_CUSTOM_ACTION_HPP_
#define AMR_BT_NODES__REMOVE_PASSED_GOALS_CUSTOM_ACTION_HPP_

#include <vector>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "behaviortree_cpp/action_node.h"

#include "nav2_behavior_tree/bt_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace amr_bt_nodes
{

// 이름 변경 반영: RemovePassedGoalsCustomAction
class RemovePassedGoalsCustomAction : public BT::SyncActionNode
{
public:
  typedef std::vector<geometry_msgs::msg::PoseStamped> Goals;

  RemovePassedGoalsCustomAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void initialize();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Goals>("input_goals", "Original goals to remove viapoints from"),
      BT::OutputPort<Goals>("output_goals", "Goals with passed viapoints removed"),
      BT::InputPort<double>("radius", 0.5, "radius to goal for it to be considered for removal"),
      BT::InputPort<std::string>("global_frame", "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", "Robot base frame"),
    };
  }

private:
  BT::NodeStatus tick() override;

  double viapoint_achieved_radius_;
  double transform_tolerance_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string robot_base_frame_;
};

}  // namespace amr_bt_nodes

#endif  // AMR_BT_NODES__REMOVE_PASSED_GOALS_CUSTOM_ACTION_HPP_
