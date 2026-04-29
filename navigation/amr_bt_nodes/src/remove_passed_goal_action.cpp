// Copyright (c) 2021 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <memory>
#include <limits>

#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "amr_bt_nodes/remove_passed_goal_action.hpp"

namespace amr_bt_nodes
{

RemovePassedGoalAction::RemovePassedGoalAction(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf),
  viapoint_achieved_radius_(0.5)
{
}

void RemovePassedGoalAction::initialize()
{
  getInput("radius", viapoint_achieved_radius_);

  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node) {
    throw std::runtime_error("Failed to get 'node' from blackboard in RemovePassedGoalAction");
  }

  node->get_parameter("transform_tolerance", transform_tolerance_);

  robot_base_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node, "robot_base_frame", this);
}

inline BT::NodeStatus RemovePassedGoalAction::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  Goals goal_poses;
  getInput("input_goals", goal_poses);

  if (goal_poses.empty()) {
    setOutput("output_goals", goal_poses);
    return BT::NodeStatus::SUCCESS;
  }

  using namespace nav2_util::geometry_utils;  // NOLINT

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, goal_poses[0].header.frame_id, robot_base_frame_,
      transform_tolerance_))
  {
    return BT::NodeStatus::FAILURE;
  }


// 변경된 부분: while 루프를 if 문으로 대체하여 0번 인덱스 1회만 검사
  if (goal_poses.size() > 1) {
    double dist_to_goal = euclidean_distance(goal_poses[0].pose, current_pose.pose);

    if (dist_to_goal <= viapoint_achieved_radius_) {

      static auto logger = rclcpp::get_logger("RemovePassedGoalAction"); // 최초 1회만 생성됨
  
      RCLCPP_WARN(
            logger,
            "Removed first goal: %.2f, %.2f. Goals after removing first: %zu.",
            goal_poses[0].pose.position.x, goal_poses[0].pose.position.y, goal_poses.size());
      

            // overhead 줄이기 위해 logger를 static으로 선언하여 최초 1회만 생성되도록 변경
      //  RCLCPP_DEBUG(rclcpp::get_logger("RemovePassedGoalAction"), "Removed first goal: ...");     

      goal_poses.erase(goal_poses.begin());
    }
  }


  setOutput("output_goals", goal_poses);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace amr_bt_nodes


#include "behaviortree_cpp/bt_factory.h"


extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<amr_bt_nodes::RemovePassedGoalAction>("RemovePassedGoalAction");
}


