#include <string>
#include <vector>
#include <memory>
#include <limits> // std::numeric_limits 사용을 위해 추가
#include "amr_bt_nodes/prune_passed_path_action.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h" // tf2::getYaw 사용을 위해 추가 (선택적이지만 좋은 습관)

#include "behaviortree_cpp/bt_factory.h"

namespace amr_bt_nodes
{

PrunePassedPathAction::PrunePassedPathAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList PrunePassedPathAction::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("input_path", "Path to prune"),
    BT::OutputPort<nav_msgs::msg::Path>("output_path", "Pruned path")
  };
}

BT::NodeStatus PrunePassedPathAction::tick()
{
  // Blackboard에서 필요한 데이터 가져오기
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  auto tf_buffer = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  // robot_base_frame은 nav2_bt_navigator의 파라미터로 설정되어 blackboard에 기본적으로 존재합니다.
  std::string robot_base_frame = "base_link"; 
  config().blackboard->get("robot_base_frame", robot_base_frame);


  nav_msgs::msg::Path path;
  if (!getInput("input_path", path) || path.poses.empty()) {
    RCLCPP_WARN(node->get_logger(), "Input path is empty or not available.");
    return BT::NodeStatus::FAILURE;
  }

  // 현재 로봇의 위치 가져오기
  geometry_msgs::msg::PoseStamped current_robot_pose;
  
  // ========================= 에러가 발생했던 부분 수정 =========================
  // nav2_util::getCurrentPose 함수에 올바른 타입의 인자를 전달합니다.
  // 타임아웃으로는 double 타입을 전달합니다.
  if (!nav2_util::getCurrentPose(
      current_robot_pose, *tf_buffer, path.header.frame_id, robot_base_frame, 0.5))
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to get current robot pose.");
    return BT::NodeStatus::FAILURE;
  }
  // ========================================================================

  // 경로 상에서 로봇과 가장 가까운 점 찾기
  auto& poses = path.poses;
  size_t closest_index = 0;
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < poses.size(); ++i) {
    double dist = std::hypot(
      poses[i].pose.position.x - current_robot_pose.pose.position.x,
      poses[i].pose.position.y - current_robot_pose.pose.position.y);
    if (dist < min_dist) {
      min_dist = dist;
      closest_index = i;
    }
  }

  // 새로운 경로 생성 (가장 가까운 점부터 끝까지)
  nav_msgs::msg::Path pruned_path;
  pruned_path.header = path.header;
  // .begin() + index를 사용하여 반복자(iterator)로 경로의 일부를 복사합니다.
  pruned_path.poses.assign(poses.begin() + closest_index, poses.end());
  
  // 결과를 Blackboard에 저장
  setOutput("output_path", pruned_path);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace amr_bt_nodes


// // 플러그인 등록 매크로
// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<amr_bt_nodes::PrunePassedPathAction>("PrunePassedPathAction");
//   // CheckBattery 노드도 같은 라이브러리에 있다면 여기에 함께 등록해야 합니다.
//   // factory.registerNodeType<amr_bt_nodes::CheckBatteryCondition>("CheckBattery");
// }


extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<amr_bt_nodes::PrunePassedPathAction>("PrunePassedPathAction");
} 
