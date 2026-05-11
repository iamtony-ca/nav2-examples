#ifndef AMR_BT_NODES__COMPUTE_VALIDATED_PATH_HPP_
#define AMR_BT_NODES__COMPUTE_VALIDATED_PATH_HPP_

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

class ComputeValidatedPath : public BT::StatefulActionNode
{
public:
  using ActionType = nav2_msgs::action::ComputePathThroughPoses;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionType>;

  ComputeValidatedPath(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void initialize();
  bool performValidation();
  void truncatePathByEuclidean(nav_msgs::msg::Path & path, const geometry_msgs::msg::Point & start, double dist_limit);
  void truncatePathToGoal(nav_msgs::msg::Path & path, const geometry_msgs::msg::Point & target);
  bool isPoseWithinDeviation(const geometry_msgs::msg::Point & p, const nav_msgs::msg::Path & ref_path, double max_dev);
  double pointToLineSegmentDistance(const geometry_msgs::msg::Point & p, const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("ComputeValidatedPath")};
  bool initialized_;

  // [핵심 추가] 독립 콜백 그룹과 실행기 (Deadlock 방지)
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
};

}  // namespace amr_bt_nodes

#endif  // AMR_BT_NODES__COMPUTE_VALIDATED_PATH_HPP_

















// #ifndef AMR_BT_NODES__COMPUTE_VALIDATED_PATH_HPP_
// #define AMR_BT_NODES__COMPUTE_VALIDATED_PATH_HPP_

// #include <string>
// #include <vector>
// #include <memory>
// #include <mutex>
// #include <atomic>

// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include "nav2_msgs/action/compute_path_through_poses.hpp"
// #include "nav_msgs/msg/path.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "behaviortree_cpp/action_node.h"

// namespace amr_bt_nodes
// {

// class ComputeValidatedPath : public BT::StatefulActionNode
// {
// public:
//   using ActionType = nav2_msgs::action::ComputePathThroughPoses;
//   using GoalHandle = rclcpp_action::ClientGoalHandle<ActionType>;

//   ComputeValidatedPath(
//     const std::string & xml_tag_name,
//     const BT::NodeConfiguration & conf);

//   static BT::PortsList providedPorts();

//   BT::NodeStatus onStart() override;
//   BT::NodeStatus onRunning() override;
//   void onHalted() override;

// private:
//   void initialize();
//   bool performValidation();
//   void truncatePathByEuclidean(nav_msgs::msg::Path & path, const geometry_msgs::msg::Point & start, double dist_limit);
//   void truncatePathToGoal(nav_msgs::msg::Path & path, const geometry_msgs::msg::Point & target);
//   bool isPoseWithinDeviation(const geometry_msgs::msg::Point & p, const nav_msgs::msg::Path & ref_path, double max_dev);
//   double pointToLineSegmentDistance(const geometry_msgs::msg::Point & p, const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b);

//   // 캐싱 및 초기화 관련 멤버
//   rclcpp::Node::SharedPtr node_;
//   rclcpp::Logger logger_{rclcpp::get_logger("ComputeValidatedPath")};
//   bool initialized_;

//   // ROS 2 Action 관련 멤버
//   rclcpp_action::Client<ActionType>::SharedPtr static_client_;
//   rclcpp_action::Client<ActionType>::SharedPtr dynamic_client_;
//   GoalHandle::SharedPtr static_goal_handle_;
//   GoalHandle::SharedPtr dynamic_goal_handle_;

//   // 비동기 스레드 상태 공유를 위한 변수
//   std::mutex mutex_;
//   std::atomic<bool> static_done_;
//   std::atomic<bool> dynamic_done_;
  
//   // 에러 코드 개별 추적용
//   std::atomic<uint16_t> static_error_code_;
//   std::atomic<uint16_t> dynamic_error_code_;
  
//   nav_msgs::msg::Path static_path_;
//   nav_msgs::msg::Path actual_path_;
//   std::vector<geometry_msgs::msg::PoseStamped> local_goals_;

//   // 파라미터 캐시
//   double horizon_;
//   double max_dev_;
//   double step_dist_;
//   double max_check_length_;
// };

// }  // namespace amr_bt_nodes

// #endif  // AMR_BT_NODES__COMPUTE_VALIDATED_PATH_HPP_
