#ifndef AMR_BT_NODES__PROGRESS_AWARE_ROUND_ROBIN_HPP_
#define AMR_BT_NODES__PROGRESS_AWARE_ROUND_ROBIN_HPP_

#include <string>
#include <memory>
#include "behaviortree_cpp/control_node.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace amr_bt_nodes
{

class ProgressAwareRoundRobin : public BT::ControlNode
{
public:
  ProgressAwareRoundRobin(const std::string & name, const BT::NodeConfiguration & config);

  ~ProgressAwareRoundRobin() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("reset_timeout", 10.0, "인덱스 초기화를 위한 최소 경과 시간 (초)"),
      BT::InputPort<double>("reset_distance", 2.0, "인덱스 초기화를 위한 최소 이동 거리 (미터)"),
      BT::InputPort<std::string>("global_frame", "odom", "기준 글로벌 프레임 (odom 권장)"),
      BT::InputPort<std::string>("robot_frame", "base_link", "로봇 베이스 프레임")
    };
  }

  void halt() override;

private:
  BT::NodeStatus tick() override;

  size_t current_child_idx_ = 0;

  // 초기화 판단을 위한 상태 변수
  rclcpp::Time last_tick_time_;
  geometry_msgs::msg::TransformStamped last_pose_;
  bool has_last_state_ = false;
};

}  // namespace amr_bt_nodes

#endif  // AMR_BT_NODES__PROGRESS_AWARE_ROUND_ROBIN_HPP_
