#ifndef VELOCITY_MODIFIER__VELOCITY_MODIFIER_NODE_HPP_
#define VELOCITY_MODIFIER__VELOCITY_MODIFIER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <robot_interfaces/msg/modifier_control.hpp>
#include <std_msgs/msg/string.hpp>

#include <mutex> // <atomic> 대신 <mutex>를 포함
#include <limits>
#include <memory>
#include <cmath>

namespace velocity_modifier
{

class VelocityModifierNode : public rclcpp::Node
{
public:
  using ModifierControl = robot_interfaces::msg::ModifierControl;
  using String = std_msgs::msg::String;

  explicit VelocityModifierNode(const rclcpp::NodeOptions & options);

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void controlCallback(const ModifierControl::SharedPtr msg);
  void recoveryModeCallback(const String::SharedPtr msg);

  rclcpp::CallbackGroup::SharedPtr cb_group_cmd_vel_;
  rclcpp::CallbackGroup::SharedPtr cb_group_control_;
  rclcpp::CallbackGroup::SharedPtr cb_group_recovery_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<ModifierControl>::SharedPtr control_sub_;
  rclcpp::Subscription<String>::SharedPtr recovery_mode_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr adjusted_cmd_vel_pub_;

  // 데이터 보호를 위한 뮤텍스
  std::mutex data_mutex_;

  // 일반 멤버 변수
  double speed_limit_linear_;
  double speed_limit_angular_;
  double speed_scale_;

  
  double min_abs_linear_vel_;
  double min_abs_angular_vel_;
  
  // 비율 보정 시 적용될 상한선 
  double ratio_scaling_max_linear_vel_;
  double ratio_scaling_max_angular_vel_;
  
  bool recovery_mode_ = false;

};

}  // namespace velocity_modifier

#endif  // VELOCITY_MODIFIER__VELOCITY_MODIFIER_NODE_HPP_
