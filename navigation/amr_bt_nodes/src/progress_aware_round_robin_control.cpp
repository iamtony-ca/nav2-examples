#include "amr_bt_nodes/progress_aware_round_robin_control.hpp"
#include <cmath>

namespace amr_bt_nodes
{

ProgressAwareRoundRobinControl::ProgressAwareRoundRobinControl(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ControlNode(name, config), last_tick_time_(0, 0, RCL_ROS_TIME)
{
}

void ProgressAwareRoundRobinControl::halt()
{
  ControlNode::halt();
  current_child_idx_ = 0;
  has_last_state_ = false; // 트리 리셋 시 상태도 초기화
}

BT::NodeStatus ProgressAwareRoundRobinControl::tick()
{
  const size_t num_children = children_nodes_.size();
  if (num_children == 0) {
    return BT::NodeStatus::SUCCESS;
  }

  // Nav2 블랙보드에서 ROS 노드와 TF 버퍼 가져오기
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  
  if (!config().blackboard->get("node", node) || 
      !config().blackboard->get("tf_buffer", tf_buffer)) 
  {
    RCLCPP_ERROR(
      node ? node->get_logger() : rclcpp::get_logger("ProgressAwareRoundRobinControl"),
      "[ProgressAwareRoundRobinControl] Nav2 Blackboard에서 'node' 또는 'tf_buffer'를 찾을 수 없습니다."
    );
    return BT::NodeStatus::FAILURE;
  }

  double timeout_sec = 10.0;
  double distance_thresh = 2.0;
  std::string global_frame = "odom";
  std::string robot_frame = "base_link";
  
  getInput("reset_timeout", timeout_sec);
  getInput("reset_distance", distance_thresh);
  getInput("global_frame", global_frame);
  getInput("robot_frame", robot_frame);

  auto now = node->now();
  geometry_msgs::msg::TransformStamped current_pose;
  bool got_pose = false;

  try {
    current_pose = tf_buffer->lookupTransform(
      global_frame, robot_frame, tf2::TimePointZero);
    got_pose = true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *node->get_clock(), 2000,
      "[ProgressAwareRoundRobinControl] TF Error: %s", ex.what()
    );
  }

  // === 시간 및 거리 기반 초기화 검사 ===
  if (has_last_state_ && got_pose) {
    double elapsed_time = (now - last_tick_time_).seconds();
    
    double dx = current_pose.transform.translation.x - last_pose_.transform.translation.x;
    double dy = current_pose.transform.translation.y - last_pose_.transform.translation.y;
    double moved_distance = std::hypot(dx, dy);

    if (elapsed_time >= timeout_sec && moved_distance >= distance_thresh) {
      RCLCPP_INFO(
        node->get_logger(),
        "[ProgressAwareRoundRobinControl] 정상 주행 감지 (경과시간: %.2f초, 이동거리: %.2fm). 리커버리 인덱스를 0으로 리셋합니다.",
        elapsed_time, moved_distance
      );
      current_child_idx_ = 0;
    }
  }

  last_tick_time_ = now;
  if (got_pose) {
    last_pose_ = current_pose;
    has_last_state_ = true;
  }

  // === 실행 로직 ===
  setStatus(BT::NodeStatus::RUNNING);

  while (current_child_idx_ < num_children) {
    TreeNode * child_node = children_nodes_[current_child_idx_];
    const BT::NodeStatus child_status = child_node->executeTick();

    switch (child_status) {
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      case BT::NodeStatus::SUCCESS:
        current_child_idx_++;
        if (current_child_idx_ == num_children) {
          current_child_idx_ = 0;
        }
        return BT::NodeStatus::SUCCESS;

      case BT::NodeStatus::FAILURE:
        {
          if (current_child_idx_ == num_children - 1) {
            current_child_idx_ = 0;
            return BT::NodeStatus::FAILURE;
          }
          current_child_idx_++;
        }
        break;

      default:  // SKIPPED
        if (current_child_idx_ == num_children - 1) {
            current_child_idx_ = 0;
            return BT::NodeStatus::FAILURE;
        }
        current_child_idx_++;
        break;
    }
  }

  halt();
  return BT::NodeStatus::FAILURE;
}

}  // namespace amr_bt_nodes


#include "behaviortree_cpp/bt_factory.h"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  // XML 태그 이름도 ProgressAwareRoundRobinControl으로 등록
  factory.registerNodeType<amr_bt_nodes::ProgressAwareRoundRobinControl>("ProgressAwareRoundRobinControl");
}
