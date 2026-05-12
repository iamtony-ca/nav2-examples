// Copyright (c) 2021 Samsung Research America
// ... (라이선스 생략) ...

#include <string>
#include <memory>
#include <limits>

#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"

// 변경된 헤더 인클루드
#include "amr_bt_nodes/remove_passed_goals_custom_action.hpp"

namespace amr_bt_nodes
{

RemovePassedGoalsCustomAction::RemovePassedGoalsCustomAction(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf),
  viapoint_achieved_radius_(0.5)
{
}

void RemovePassedGoalsCustomAction::initialize()
{
  getInput("radius", viapoint_achieved_radius_);

  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node) {
    throw std::runtime_error("Failed to get 'node' from blackboard in RemovePassedGoalsCustomAction");
  }
  
  node->get_parameter("transform_tolerance", transform_tolerance_);

  robot_base_frame_ = nav2_behavior_tree::deconflictPortAndParamFrame<std::string>(
    node, "robot_base_frame", this);
}

inline BT::NodeStatus RemovePassedGoalsCustomAction::tick()
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

  using namespace nav2_util::geometry_utils;

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, goal_poses[0].header.frame_id, robot_base_frame_,
      transform_tolerance_))
  {
    return BT::NodeStatus::FAILURE;
  }

  bool is_removed = false;
  int removed_count = 0; // 로그 출력을 위해 몇 개나 지웠는지 카운트

  // 변경된 핵심 부분: while 문으로 순차 검사 후 break
  while (goal_poses.size() > 1) {
    double dist_to_goal = euclidean_distance(goal_poses[0].pose, current_pose.pose);

    // 조건 만족 시: 0번 인덱스 지우고 다음 루프(새로운 0번) 검사 진행
    if (dist_to_goal <= viapoint_achieved_radius_) {
      goal_poses.erase(goal_poses.begin());
      is_removed = true;
      removed_count++;
    } 
    // 조건 미만족 시: 즉시 while 루프 탈출
    else {
      break; 
    }
  }

  // 지운 기록이 있다면 한 번만 로그 출력 (성능 최적화)
  if (is_removed) {
    static auto logger = rclcpp::get_logger("RemovePassedGoalsCustomAction");
    RCLCPP_DEBUG(
          logger,
          "Removed %d goal(s). Next goal is at: %.2f, %.2f. Remaining goals: %zu.",
          removed_count, goal_poses[0].pose.position.x, goal_poses[0].pose.position.y, goal_poses.size());      
  }

  setOutput("output_goals", goal_poses);

  // 하나라도 지웠으면 SUCCESS, 하나도 안 지웠으면 FAILURE
  if (is_removed) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace amr_bt_nodes

#include "behaviortree_cpp/bt_factory.h"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  // XML에서 사용할 태그 이름 등록
  factory.registerNodeType<amr_bt_nodes::RemovePassedGoalsCustomAction>("RemovePassedGoalsCustomAction");
}
