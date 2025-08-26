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

  // 최소 속도 파라미터
  this->declare_parameter<double>("min_abs_linear_vel", 0.05);
  this->declare_parameter<double>("min_abs_angular_vel", 0.05);
  this->get_parameter("min_abs_linear_vel", min_abs_linear_vel_);
  this->get_parameter("min_abs_angular_vel", min_abs_angular_vel_);

  // 비율 보정 시 적용될 상한선 파라미터 
  this->declare_parameter<double>("ratio_scaling_max_linear_vel", 0.35);
  this->declare_parameter<double>("ratio_scaling_max_angular_vel", 0.25);
  this->get_parameter("ratio_scaling_max_linear_vel", ratio_scaling_max_linear_vel_);
  this->get_parameter("ratio_scaling_max_angular_vel", ratio_scaling_max_angular_vel_);
  
  RCLCPP_INFO(
    this->get_logger(), "Min absolute linear velocity: %.3f m/s", min_abs_linear_vel_);
  RCLCPP_INFO(
    this->get_logger(), "Min absolute angular velocity: %.3f rad/s", min_abs_angular_vel_);
  RCLCPP_INFO(
    this->get_logger(), "Ratio scaling max linear velocity: %.3f m/s", ratio_scaling_max_linear_vel_);
  RCLCPP_INFO(
    this->get_logger(), "Ratio scaling max angular velocity: %.3f rad/s", ratio_scaling_max_angular_vel_);

  
  cb_group_cmd_vel_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  cb_group_control_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  cb_group_recovery_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

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
  
  auto sub_recovery_opt = rclcpp::SubscriptionOptions();
  sub_recovery_opt.callback_group = cb_group_recovery_;
  // 상태 토픽은 마지막 메시지를 유지하는 것이 좋으므로 transient_local QoS 사용
  // rclcpp::QoS qos_recovery(10);
  // qos_recovery.transient_local(); 
  recovery_mode_sub_ = this->create_subscription<String>(
    // "/bt_recovery_mode", qos_recovery,
    "/bt_recovery_mode", 10,
    std::bind(&VelocityModifierNode::recoveryModeCallback, this, std::placeholders::_1),
    sub_recovery_opt);

  RCLCPP_INFO(this->get_logger(), "Node has been started successfully.");
}



void VelocityModifierNode::recoveryModeCallback(const String::SharedPtr msg)
{
  // lock_guard를 통해 공유 변수인 recovery_mode_를 안전하게 수정
  const std::lock_guard<std::mutex> lock(data_mutex_);
  
  if (msg->data == "start") {
    if (!recovery_mode_) {
      recovery_mode_ = true;
      RCLCPP_INFO(this->get_logger(), "Recovery mode ENABLED. Low-speed correction is active.");
    }
  } else if (msg->data == "finish") {
    if (recovery_mode_) {
      recovery_mode_ = false;
      RCLCPP_INFO(this->get_logger(), "Recovery mode DISABLED. Low-speed correction is inactive.");
    }
  } else {
    RCLCPP_DEBUG(
      this->get_logger(), "Received unknown command on /bt_recovery_mode: '%s'", msg->data.c_str());
  }
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

  // 3. 저속 보정 로직 
  if (recovery_mode_) {
    const double vx = adjusted_vel->linear.x;
    const double wz = adjusted_vel->angular.z;
    const double abs_vx = std::abs(vx);
    const double abs_wz = std::abs(wz);
    const double epsilon = 1e-9;

    // 조건: 정지 명령이 아니면서, 선속도 또는 각속도 중 하나라도 최소 임계값보다 작은 경우
    bool is_nonzero_and_too_slow = (abs_vx > epsilon || abs_wz > epsilon) &&
                                  ((abs_vx > epsilon && abs_vx < min_abs_linear_vel_) ||
                                    (abs_wz > epsilon && abs_wz < min_abs_angular_vel_));

    if (is_nonzero_and_too_slow) {
      RCLCPP_DEBUG(this->get_logger(), "Command is too slow, applying unified scaling.");

      // 각 축을 최소 속도까지 증폭시키는 데 필요한 배율을 계산
      double s_linear = 1.0;
      if (abs_vx > epsilon) {
        s_linear = min_abs_linear_vel_ / abs_vx;
      }

      double s_angular = 1.0;
      if (abs_wz > epsilon) {
        s_angular = min_abs_angular_vel_ / abs_wz;
      }

      // 두 배율 중 더 큰 값을 최종 배율로 선택
      // 이렇게 하면 최소한 하나의 축은 최소 임계값에 도달하거나 넘어서게 됨
      double scale = std::max(s_linear, s_angular);

      // 원래 속도에 최종 배율을 곱하여 비율을 유지한 채 증폭
      double new_vx = vx * scale;
      double new_wz = wz * scale;

      // 이 로직으로 계산된 값에 대해서만 특별 상한선 적용
      adjusted_vel->linear.x = std::clamp(
        new_vx, -ratio_scaling_max_linear_vel_, ratio_scaling_max_linear_vel_);
      adjusted_vel->angular.z = std::clamp(
        new_wz, -ratio_scaling_max_angular_vel_, ratio_scaling_max_angular_vel_);
    }
  } // End of if (recovery_mode_)

  
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
