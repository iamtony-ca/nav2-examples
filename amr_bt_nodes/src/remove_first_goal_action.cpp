#include "amr_bt_nodes/remove_first_goal_action.hpp"

#include <string>
#include <vector>

namespace amr_bt_nodes
{

RemoveFirstGoalAction::RemoveFirstGoalAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  config.blackboard->get<rclcpp::Node::SharedPtr>("node", node_);
  logger_ = node_->get_logger();
}

BT::NodeStatus RemoveFirstGoalAction::tick()
{
  // 1. 입력 포트("input_goals")로부터 목표 리스트를 가져옵니다.
  auto goals = getInput<std::vector<geometry_msgs::msg::PoseStamped>>("input_goals");

  if (!goals) {
    RCLCPP_ERROR(
      logger_, "Missing required input port [input_goals]: %s", goals.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  // 2. [수정된 핵심 로직] 목표 리스트의 크기를 확인합니다.
  if (goals.value().size() <= 1) {
    // 목표가 1개 이하로 남아있으면, 요청에 따라 FAILURE를 반환합니다.
    RCLCPP_WARN(
      logger_,
      "Cannot remove the first goal, as only %zu goal(s) remain. Returning FAILURE.",
      goals.value().size());
    return BT::NodeStatus::FAILURE;
  }

  // 3. 첫 번째 목표를 제외한 새로운 리스트를 생성합니다.
  std::vector<geometry_msgs::msg::PoseStamped> remaining_goals(
    goals.value().begin() + 1, goals.value().end());
  
  RCLCPP_DEBUG(
    logger_,
    "Original goals: %zu. New goals after removing first: %zu.",
    goals.value().size(), remaining_goals.size());

  // 4. 처리된 새로운 리스트를 출력 포트("remaining_goals")에 설정합니다.
  setOutput("remaining_goals", remaining_goals);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace amr_bt_nodes

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<amr_bt_nodes::RemoveFirstGoalAction>("RemoveFirstGoalAction");
}