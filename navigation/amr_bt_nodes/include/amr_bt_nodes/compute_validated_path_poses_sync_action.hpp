#ifndef AMR_BT_NODES__COMPUTE_VALIDATED_PATH_POSES_SYNC_ACTION_HPP_
#define AMR_BT_NODES__COMPUTE_VALIDATED_PATH_POSES_SYNC_ACTION_HPP_

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp/action_node.h"

namespace amr_bt_nodes
{

class ComputeValidatedPathPosesSyncAction : public BT::StatefulActionNode
{
public:
  using ActionType = nav2_msgs::action::ComputePathThroughPoses;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionType>;

  ComputeValidatedPathPosesSyncAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  bool initialize();
  bool performValidation();

  // actual 경로를 target(마지막 할당 waypoint)에 '처음' 가까워지는 통과 지점에서 절삭한다.
  // 같은 지점을 두 번 지나도 첫 번째 pass에서 컷하므로, ref 끝점(단일 통과)과의 대응이 일치한다.
  void truncateAtFirstWaypointPass(
    nav_msgs::msg::Path & path,
    const geometry_msgs::msg::Point & target,
    double reach_tol);

  // 경로 앞쪽 표본으로 점 간격(그리드 플래너 기준 ~ costmap resolution)을 추정한다. O(K).
  double estimatePathResolution(const nav_msgs::msg::Path & path) const;

  bool isPoseWithinDeviation(
    const geometry_msgs::msg::Point & p,
    const nav_msgs::msg::Path & ref_path,
    double max_dev);

  double pointToLineSegmentDistance(
    const geometry_msgs::msg::Point & p,
    const geometry_msgs::msg::Point & a,
    const geometry_msgs::msg::Point & b);

  // 검증에 실제로 쓰인 경로를 RViz 디버그용으로 발행한다.
  void publishDebugPath(
    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & pub,
    const nav_msgs::msg::Path & path,
    const std::string & fallback_frame = "map");

  std::string count_key_;   // 블랙보드에서 쓸 변수 이름 캐싱용
  int max_403_retries_;     // 허용되는 최대 403 횟수

  rclcpp::Time start_time_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("ComputeValidatedPathPosesSyncAction")};
  bool initialized_;

  // [핵심] 독립 콜백 그룹과 실행기 (Deadlock 방지)
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  rclcpp_action::Client<ActionType>::SharedPtr static_client_;
  rclcpp_action::Client<ActionType>::SharedPtr dynamic_client_;
  GoalHandle::SharedPtr static_goal_handle_;
  GoalHandle::SharedPtr dynamic_goal_handle_;

  std::mutex mutex_;
  std::atomic<bool> static_done_;
  std::atomic<bool> dynamic_done_;

  std::atomic<uint16_t> static_error_code_;
  std::atomic<uint16_t> dynamic_error_code_;

  nav_msgs::msg::Path static_path_;
  nav_msgs::msg::Path actual_path_;
  std::vector<geometry_msgs::msg::PoseStamped> local_goals_;

  double horizon_;
  double max_dev_;
  double step_dist_;
  double max_check_length_;

  // 디버그 시각화 퍼블리셔
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trunc_ref_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trunc_actual_pub_;

  // 순차적 실행을 위한 상태 머신
  enum class PlanningState {
    IDLE,
    WAITING_FOR_STATIC,
    WAITING_FOR_DYNAMIC,
    VALIDATING
  };

  PlanningState current_state_;
};

}  // namespace amr_bt_nodes

#endif  // AMR_BT_NODES__COMPUTE_VALIDATED_PATH_POSES_SYNC_ACTION_HPP_
