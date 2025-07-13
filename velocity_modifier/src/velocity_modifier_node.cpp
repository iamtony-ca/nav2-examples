#include "velocity_modifier/velocity_modifier_node.hpp"

#include <algorithm> // For std::clamp

namespace velocity_modifier
{

VelocityModifierNode::VelocityModifierNode(const rclcpp::NodeOptions & options)
: Node("velocity_modifier_node", options),
  speed_limit_linear_(std::numeric_limits<double>::max()),
  speed_limit_angular_(std::numeric_limits<double>::max()),
  speed_scale_(1.0)
{
  RCLCPP_INFO(this->get_logger(), "Velocity Modifier Node is initializing...");

  cb_group_cmd_vel_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  cb_group_control_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  adjusted_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  auto sub_cmd_vel_opt = rclcpp::SubscriptionOptions();
  sub_cmd_vel_opt.callback_group = cb_group_cmd_vel_;
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_adjusted", 10,
    std::bind(&VelocityModifierNode::cmdVelCallback, this, std::placeholders::_1),
    sub_cmd_vel_opt);

  auto sub_control_opt = rclcpp::SubscriptionOptions();
  sub_control_opt.callback_group = cb_group_control_;
  rclcpp::QoS qos_control(10);
  qos_control.transient_local();
  control_sub_ = this->create_subscription<ModifierControl>(
    "velocity_modifier/control", qos_control,
    std::bind(&VelocityModifierNode::controlCallback, this, std::placeholders::_1),
    sub_control_opt);
  
  RCLCPP_INFO(this->get_logger(), "Node has been started successfully.");
}

void VelocityModifierNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // lock_guard를 통해 데이터 읽기 전 잠금
  const std::lock_guard<std::mutex> lock(data_mutex_);
  
  auto adjusted_vel = std::make_unique<geometry_msgs::msg::Twist>(*msg);

  // 이제 안전하게 공유 변수 접근 가능
  adjusted_vel->linear.x *= speed_scale_;
  adjusted_vel->angular.z *= speed_scale_;

  adjusted_vel->linear.x = std::clamp(
    adjusted_vel->linear.x, -speed_limit_linear_, speed_limit_linear_);
  adjusted_vel->angular.z = std::clamp(
    adjusted_vel->angular.z, -speed_limit_angular_, speed_limit_angular_);

  adjusted_cmd_vel_pub_->publish(std::move(adjusted_vel));
}

void VelocityModifierNode::controlCallback(const ModifierControl::SharedPtr msg)
{
  // lock_guard를 통해 데이터 쓰기 전 잠금
  const std::lock_guard<std::mutex> lock(data_mutex_);

  switch (msg->command_type) {
    case ModifierControl::TYPE_SPEED_LIMIT:
      // 잠금 상태에서 모든 관련 변수를 원자적으로 변경
      speed_limit_linear_ = msg->linear_value;
      speed_limit_angular_ = msg->angular_value;
      speed_scale_ = 1.0;
      RCLCPP_INFO(
        this->get_logger(), "Set speed limit -> Linear: %.2f m/s, Angular: %.2f rad/s",
        speed_limit_linear_, speed_limit_angular_);
      break;

    case ModifierControl::TYPE_SPEED_SCALE:
      // 잠금 상태에서 모든 관련 변수를 원자적으로 변경
      speed_scale_ = msg->linear_value;
      speed_limit_linear_ = std::numeric_limits<double>::max();
      speed_limit_angular_ = std::numeric_limits<double>::max();
      RCLCPP_INFO(this->get_logger(), "Set speed scale to: %.2f", speed_scale_);
      break;

    default:
      RCLCPP_WARN(
        this->get_logger(), "Received control command with unknown type: %d", msg->command_type);
      break;
  }
}

}  // namespace velocity_modifier