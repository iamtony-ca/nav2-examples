#include "amr_bt_nodes/check_final_goal_yaw_action.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace amr_bt_nodes
{

CheckFinalGoalYawAction::CheckFinalGoalYawAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::NodeStatus CheckFinalGoalYawAction::tick()
{
  std::vector<geometry_msgs::msg::PoseStamped> goals;
  geometry_msgs::msg::PoseStamped current_pose;

  // 1. Input Port 데이터 가져오기
  if (!getInput("input_goals", goals)) {
    RCLCPP_ERROR(rclcpp::get_logger("CheckFinalGoalYawAction"), "Missing required input [input_goals]");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("input_pose", current_pose)) {
    RCLCPP_ERROR(rclcpp::get_logger("CheckFinalGoalYawAction"), "Missing required input [input_pose]");
    return BT::NodeStatus::FAILURE;
  }

  if (goals.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("CheckFinalGoalYawAction"), "Input goals vector is empty");
    return BT::NodeStatus::FAILURE;
  }

  // 2. 마지막 Goal 가져오기
  const auto & final_goal = goals.back();

  // 3. Yaw 계산 (Helper Lambda)
  auto get_yaw = [](const geometry_msgs::msg::Pose & pose) -> double {
    tf2::Quaternion q(
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  };

  double goal_yaw = get_yaw(final_goal.pose);
  double current_yaw = get_yaw(current_pose.pose);

  // 4. 차이 계산 (Goal - Input)
  double diff = goal_yaw - current_yaw;

  // 5. 각도 정규화 (Normalize to -PI ~ +PI)
  // 3.14와 -3.14의 차이가 6.28이 아니라 0에 가깝게 계산되도록 함
  while (diff > M_PI) diff -= 2.0 * M_PI;
  while (diff < -M_PI) diff += 2.0 * M_PI;

  // 6. Output 세팅 (difference_radian)
  setOutput("difference_radian", diff);

  // 7. 조건 검사 (0.0 ~ 0.2 degree)
  // Radian to Degree 변환
  double diff_deg = std::abs(diff * (180.0 / M_PI));
  double result_value = 0.0;

  // degree 기준으로 0.2도 이하인지 확인
  if (diff_deg <= 0.2) {
    result_value = 1.0;
  } else {
    result_value = 0.0;
  }

  setOutput("result", result_value);

  // 디버깅용 로그 (필요시 주석 해제)
  /*
  RCLCPP_INFO(rclcpp::get_logger("CheckFinalGoalYawAction"), 
    "Goal Yaw: %.4f, Curr Yaw: %.4f, Diff(rad): %.4f, Diff(deg): %.4f, Result: %.1f", 
    goal_yaw, current_yaw, diff, diff_deg, result_value);
  */

  return BT::NodeStatus::SUCCESS;
}

}  // namespace amr_bt_nodes

// 플러그인 등록 매크로
#include "behaviortree_cpp/bt_factory.h"
// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<amr_bt_nodes::CheckFinalGoalYawAction>("CheckFinalGoalYawAction");
// }

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<amr_bt_nodes::CheckFinalGoalYawAction>("CheckFinalGoalYawAction");
}


