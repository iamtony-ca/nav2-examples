#include "amr_bt_nodes/check_and_extract_spin_action.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace amr_bt_nodes
{

CheckAndExtractSpinAction::CheckAndExtractSpinAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::NodeStatus CheckAndExtractSpinAction::tick()
{
  std::vector<geometry_msgs::msg::PoseStamped> goals;
  double distance_tolerance = 0.05;

  // 1. Input Port 데이터 가져오기
  if (!getInput("goals", goals)) {
    RCLCPP_ERROR(rclcpp::get_logger("CheckAndExtractSpinAction"), "Missing required input [goals]");
    return BT::NodeStatus::FAILURE;
  }

  getInput("distance_tolerance", distance_tolerance);

  // 2. 조건 검사: waypoint가 최소 2개 이상이어야 방향 비교가 가능
  if (goals.size() < 2) {
    return BT::NodeStatus::FAILURE;
  }

  const auto & wp0 = goals[0].pose.position;
  const auto & wp1 = goals[1].pose.position;

  // 3. Euclidean distance 계산
  double dx = wp1.x - wp0.x;
  double dy = wp1.y - wp0.y;
  double distance = std::hypot(dx, dy);

  // 4. 거리가 tolerance 이내인지 확인
  if (distance <= distance_tolerance) {
    // 5. Yaw 계산 (Helper Lambda)
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

    double yaw0 = get_yaw(goals[0].pose);
    double yaw1 = get_yaw(goals[1].pose);

    // 6. 차이 계산 (wp1 - wp0)
    double diff = yaw1 - yaw0;

    // 7. 각도 정규화 (Normalize to -PI ~ +PI)
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;

    // 8. Output 세팅 (spin_angle)
    setOutput("spin_angle", diff);

    // 디버깅용 로그
    RCLCPP_INFO(rclcpp::get_logger("CheckAndExtractSpinAction"), 
      "Pure rotation detected. Dist: %.4f, Yaw0: %.4f, Yaw1: %.4f, Spin Angle(rad): %.4f", 
      distance, yaw0, yaw1, diff);

    // goals 배열을 건드리지 않고 SUCCESS만 반환
    return BT::NodeStatus::SUCCESS;
  }

  // 거리가 멀면 일반 주행이므로 FAILURE 반환
  return BT::NodeStatus::FAILURE;
}

}  // namespace amr_bt_nodes

// 플러그인 등록
extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  // BT XML에서 <CheckAndExtractSpin ... /> 으로 호출할 수 있도록 이름 등록
  factory.registerNodeType<amr_bt_nodes::CheckAndExtractSpinAction>("CheckAndExtractSpin");
}
