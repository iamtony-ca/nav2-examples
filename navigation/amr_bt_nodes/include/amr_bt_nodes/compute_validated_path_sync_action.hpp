#ifndef AMR_BT_NODES__COMPUTE_VALIDATED_PATH_SYNC_ACTION_HPP_
#define AMR_BT_NODES__COMPUTE_VALIDATED_PATH_SYNC_ACTION_HPP_

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

class ComputeValidatedPathSyncAction : public BT::StatefulActionNode
{
public:
  using ActionType = nav2_msgs::action::ComputePathThroughPoses;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionType>;

  ComputeValidatedPathSyncAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  bool initialize();
  bool performValidation();
  void truncatePathByEuclidean(nav_msgs::msg::Path & path, const geometry_msgs::msg::Point & start, double dist_limit);
  void truncatePathToGoal(nav_msgs::msg::Path & path, const geometry_msgs::msg::Point & target);
  bool isPoseWithinDeviation(const geometry_msgs::msg::Point & p, const nav_msgs::msg::Path & ref_path, double max_dev);
  double pointToLineSegmentDistance(const geometry_msgs::msg::Point & p, const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b);

  void publishDebugPath(
    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & pub,
    const nav_msgs::msg::Path & path,
    const std::string & fallback_frame = "map");

  std::string count_key_; // 블랙보드에서 쓸 변수 이름 캐싱용
  int max_403_retries_;          // 허용되는 최대 403 횟수

  std::atomic<uint64_t> cycle_id_{0};   // 콜백을 사이클에 묶기 위한 토큰

  // private 멤버에 추가
  rclcpp::Time start_time_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("ComputeValidatedPathSyncAction")};
  bool initialized_;

  // [핵심 추가] 독립 콜백 그룹과 실행기 (Deadlock 방지)
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  rclcpp_action::Client<ActionType>::SharedPtr static_client_;
  rclcpp_action::Client<ActionType>::SharedPtr dynamic_client_;
  GoalHandle::SharedPtr static_goal_handle_;
  GoalHandle::SharedPtr dynamic_goal_handle_;


  // [디버그] 검증에 실제로 쓰인 절삭 경로 시각화용 퍼블리셔
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trunc_ref_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trunc_actual_pub_;

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

  bool flag_path_empty{false};

  // [추가] 순차적 실행을 위한 상태 머신 Enum
  enum class PlanningState {
    IDLE,
    WAITING_FOR_STATIC,
    WAITING_FOR_DYNAMIC,
    VALIDATING
  };

  PlanningState current_state_; // 현재 상태 저장 변수

};

}  // namespace amr_bt_nodes

#endif  // AMR_BT_NODES__COMPUTE_VALIDATED_PATH_SYNC_ACTION_HPP_



