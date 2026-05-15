#include "amr_bt_nodes/remove_first_goals_action.hpp"
#include <cmath> // std::hypot 사용

namespace amr_bt_nodes
{

RemoveFirstGoalsAction::RemoveFirstGoalsAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config),
  logger_(rclcpp::get_logger("RemoveFirstGoalsAction"))
{
  config.blackboard->get<rclcpp::Node::SharedPtr>("node", node_);
  logger_ = node_->get_logger();
}


BT::NodeStatus RemoveFirstGoalsAction::tick()
{
  auto goals_wrapper = getInput<Goals>("input_goals");
  if (!goals_wrapper) {
    RCLCPP_ERROR(logger_, "Missing input goals");
    return BT::NodeStatus::FAILURE;
  }
  const auto & goals = goals_wrapper.value();

  // 규칙 1: 처음부터 goals가 1개 이하면 삭제 없이 바로 FAILURE
  if (goals.size() <= 1) {
    RCLCPP_WARN(logger_, "Goal list size is %zu. Cannot remove to keep at least one goal.", goals.size());
    return BT::NodeStatus::FAILURE;
  }

  double epsilon;
  getInput("distance_threshold", epsilon);

  size_t remove_count = 1; // 기본은 1개 삭제

  // 0번과 1번 비교 로직
  const auto & p0 = goals[0].pose.position;
  const auto & p1 = goals[1].pose.position;
  double dist = std::hypot(p1.x - p0.x, p1.y - p0.y);

  if (dist < epsilon) {
    // 두 목표가 같은 경우
    if (goals.size() == 2) {
      // 규칙 2: 처음 goals가 2개인데 둘이 같으면, 다 지워져서 0개가 되므로 지우지 않음
      RCLCPP_WARN(logger_, "Two goals are identical but only 2 exist. Aborting remove to prevent empty list.");
      return BT::NodeStatus::FAILURE;
    } else {
      // 규칙 3: 3개 이상이면 둘 다 지워도 남은 게 있으므로 2개 삭제
      RCLCPP_INFO(logger_, "Goal 0 and 1 are identical. Removing both. (Remaining: %zu)", goals.size() - 2);
      remove_count = 2;
    }
  } else {
    // 두 목표가 다르면 0번만 삭제
    RCLCPP_DEBUG(logger_, "Goal 0 and 1 are different. Removing only Goal 0.");
    remove_count = 1;
  }

  // 최종 안전장치: 이럴 일은 없겠지만 결과물이 0개가 되는지 한 번 더 체크
  if (goals.size() <= remove_count) {
    return BT::NodeStatus::FAILURE;
  }

  // 데이터 추출 및 출력
  Goals remaining_goals(goals.begin() + remove_count, goals.end());
  setOutput("remaining_goals", remaining_goals);

  return BT::NodeStatus::SUCCESS;
}


}  // namespace amr_bt_nodes

#include "behaviortree_cpp/bt_factory.h"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<amr_bt_nodes::RemoveFirstGoalsAction>("RemoveFirstGoalsAction");
}