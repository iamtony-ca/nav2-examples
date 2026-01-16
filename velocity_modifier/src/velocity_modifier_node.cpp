#include "velocity_modifier/velocity_modifier_node.hpp"

#include <algorithm> // For std::clamp

namespace velocity_modifier
{

VelocityModifierNode::VelocityModifierNode(const rclcpp::NodeOptions & options)
: Node("velocity_modifier_node", options)
{
  RCLCPP_INFO(this->get_logger(), "Velocity Modifier Node is initializing...");

  // 기존 파라미터들
  this->declare_parameter<double>("min_abs_linear_vel", 0.05);
  this->declare_parameter<double>("min_abs_angular_vel", 0.05);
  this->declare_parameter<double>("ratio_scaling_max_linear_vel", 0.35);
  this->declare_parameter<double>("ratio_scaling_max_angular_vel", 0.25);

  // [Added] Phase 2 (1.0s ~ 2.5s) 속도 제한 파라미터 (기본값 0.3)
  this->declare_parameter<double>("startup_phase2_limit", 0.3);

  this->get_parameter("min_abs_linear_vel", min_abs_linear_vel_);
  this->get_parameter("min_abs_angular_vel", min_abs_angular_vel_);
  this->get_parameter("ratio_scaling_max_linear_vel", ratio_scaling_max_linear_vel_);
  this->get_parameter("ratio_scaling_max_angular_vel", ratio_scaling_max_angular_vel_);
  
  // [Added] 파라미터 읽기
  this->get_parameter("startup_phase2_limit", startup_phase2_limit_);
  
  // [Added] 시간 초기화 (0초로 설정)
  driving_start_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

  RCLCPP_INFO(this->get_logger(), "Startup Phase2 Limit: %.3f m/s", startup_phase2_limit_);

  // (이하 기존 코드와 동일)
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

  recovery_mode_sub_ = this->create_subscription<String>(
    "/robot_status", 10,
    std::bind(&VelocityModifierNode::recoveryModeCallback, this, std::placeholders::_1),
    sub_recovery_opt);

  RCLCPP_INFO(this->get_logger(), "Node has been started successfully.");
}

void VelocityModifierNode::recoveryModeCallback(const String::SharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(data_mutex_);
  
  std::string current_status = msg->data;

  // [Added] 주행 시작 감지 로직 (Edge Detection)
  // 이전 상태는 driving이 아니었는데, 현재 driving이 된 순간을 포착
  if (current_status == "driving" && last_robot_status_ != "driving") {
    driving_start_time_ = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(), "Robot started DRIVING. Engaging startup speed limits.");
  }
  // 상태가 driving이 아니게 되면 타이머 리셋 (0으로 설정하여 로직 비활성화)
  else if (current_status != "driving") {
    driving_start_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  }

  // 상태 업데이트
  last_robot_status_ = current_status;

  // 기존 Recovery Logic 유지
  if (current_status == "recovery_start") {
    if (recovery_mode_ == false) {
      recovery_mode_ = true;
      RCLCPP_INFO(this->get_logger(), "Recovery mode ENABLED.");
    }
  } else if (current_status == "recovery_finish") {
    if (recovery_mode_ == true) {
      recovery_mode_ = false;
      RCLCPP_INFO(this->get_logger(), "Recovery mode DISABLED.");
    }
  } else {
    // driving 등 다른 상태일 때는 recovery_mode_를 false로 두는 것이 안전할 수 있음
    // 하지만 기존 로직 유지를 위해 명시적 finish가 아닐 경우의 처리는 기존 코드 존중
    // (기존 코드의 else { recovery_mode_ = false; } 부분 유지)
    // if (current_status != "recovery_start" && current_status != "driving") { 
      recovery_mode_ = false; 
    // }
  }
}

void VelocityModifierNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(data_mutex_);
  
  auto adjusted_vel = std::make_unique<geometry_msgs::msg::Twist>(*msg);

  // === [Added] 1. 주행 초기 속도 제한 (Startup Speed Limit) ===
  // driving_start_time_이 0이 아니라는 것은 현재 driving 상태라는 의미
  if (driving_start_time_.nanoseconds() > 0) {
    double elapsed_sec = (this->get_clock()->now() - driving_start_time_).seconds();
    double startup_limit = -1.0; // 음수는 제한 없음을 의미

    // Phase 1: 0.0s ~ 1.0s -> 0.1 m/s 제한
    if (elapsed_sec >= 0.0 && elapsed_sec < 1.0) {
      startup_limit = 0.1;
    }
    // Phase 2: 1.0s ~ 2.5s -> 설정된 속도(예: 0.3) 제한
    else if (elapsed_sec >= 1.0 && elapsed_sec < 2.5) {
      startup_limit = startup_phase2_limit_;
    }

    // 제한 값이 설정되었고, 현재 선속도가 그보다 크다면 Clamping 수행
    if (startup_limit > 0.0) {
      double abs_vx = std::abs(adjusted_vel->linear.x);
      if (abs_vx > startup_limit) {
        // 비율 유지 Clamping: scale = limit / current
        double scale = startup_limit / abs_vx;
        
        adjusted_vel->linear.x *= scale;
        adjusted_vel->angular.z *= scale; // 각속도도 동일 비율로 줄임

        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
          "Startup Limit Active (t=%.2fs): Limit=%.2f, OrigVx=%.2f -> NewVx=%.2f", 
          elapsed_sec, startup_limit, abs_vx, adjusted_vel->linear.x);
      }
    }
  }
  // ==========================================================

  // [기존 로직: 2. 모드별 처리]
  // 주행 초기 제한이 걸렸더라도, 사용자가 설정한 Global Limit(STANDARD_LIMIT 등)이
  // 더 작다면 그 값으로 덮어씌워지므로 안전함 (std::clamp 사용 덕분)
  if (current_mode_ == SpeedMode::STANDARD_LIMIT) {
    adjusted_vel->linear.x *= speed_scale_; 
    adjusted_vel->angular.z *= speed_scale_;
    adjusted_vel->linear.x = std::clamp(
      adjusted_vel->linear.x, -speed_limit_linear_, speed_limit_linear_);
    adjusted_vel->angular.z = std::clamp(
      adjusted_vel->angular.z, -speed_limit_angular_, speed_limit_angular_);
  } 
  else if (current_mode_ == SpeedMode::STANDARD_SCALE) {
    adjusted_vel->linear.x *= speed_scale_;
    adjusted_vel->angular.z *= speed_scale_;
    adjusted_vel->linear.x = std::clamp(
      adjusted_vel->linear.x, -speed_limit_linear_, speed_limit_linear_);
    adjusted_vel->angular.z = std::clamp(
      adjusted_vel->angular.z, -speed_limit_angular_, speed_limit_angular_);
  }
  else if (current_mode_ == SpeedMode::RATIO_LIMIT_SCALE) {
    adjusted_vel->linear.x *= speed_scale_;
    adjusted_vel->angular.z *= speed_scale_;

    double abs_vx = std::abs(adjusted_vel->linear.x);
    double abs_wz = std::abs(adjusted_vel->angular.z);
    double scale = 1.0;
    
    double linear_ratio = 1.0;
    if (abs_vx > 1e-6) {
      linear_ratio = ratio_limit_linear_ / abs_vx;
    }
    double angular_ratio = 1.0;
    if (abs_wz > 1e-6) {
      angular_ratio = ratio_limit_angular_ / abs_wz;
    }

    if (abs_vx > ratio_limit_linear_ || abs_wz > ratio_limit_angular_) {
      scale = std::min(linear_ratio, angular_ratio);
    }
    
    scale = std::min(scale, 1.0); 

    adjusted_vel->linear.x *= scale;
    adjusted_vel->angular.z *= scale;
  }

  // [기존 로직: 3. 저속 보정 로직]
  if (recovery_mode_ == true) {
    // ... (기존 저속 보정 코드 내용 그대로 유지) ...
    const double vx = adjusted_vel->linear.x;
    const double wz = adjusted_vel->angular.z;
    const double abs_vx = std::abs(vx);
    const double abs_wz = std::abs(wz);
    const double epsilon = 1e-9;

    bool is_nonzero_and_too_slow = (abs_vx > epsilon || abs_wz > epsilon) &&
                                  ((abs_vx > epsilon && abs_vx < min_abs_linear_vel_) ||
                                    (abs_wz > epsilon && abs_wz < min_abs_angular_vel_));

    if (is_nonzero_and_too_slow) {
      double s_linear = 1.0;
      if (abs_vx > epsilon) {
        s_linear = min_abs_linear_vel_ / abs_vx;
      }

      double s_angular = 1.0;
      if (abs_wz > epsilon) {
        s_angular = min_abs_angular_vel_ / abs_wz;
      }

      double scale = std::max(s_linear, s_angular);
      double new_vx = vx * scale;
      double new_wz = wz * scale;

      adjusted_vel->linear.x = std::clamp(
        new_vx, -ratio_scaling_max_linear_vel_, ratio_scaling_max_linear_vel_);
      adjusted_vel->angular.z = std::clamp(
        new_wz, -ratio_scaling_max_angular_vel_, ratio_scaling_max_angular_vel_);
    }
  } 

  adjusted_cmd_vel_pub_->publish(std::move(adjusted_vel));
}

// (controlCallback은 기존과 동일하므로 생략하거나 그대로 둡니다)
void VelocityModifierNode::controlCallback(const ModifierControl::SharedPtr msg)
{
    const std::lock_guard<std::mutex> lock(data_mutex_);
    // ... (기존 switch case 로직 동일) ...
    // 편의상 생략, 기존 코드 사용
    switch (msg->command_type) {
    case ModifierControl::TYPE_SPEED_LIMIT:
      current_mode_ = SpeedMode::STANDARD_LIMIT;
      speed_limit_linear_ = msg->linear_value;
      speed_limit_angular_ = msg->angular_value;
      speed_scale_ = 1.0;
      break;
    case ModifierControl::TYPE_SPEED_SCALE:
      current_mode_ = SpeedMode::STANDARD_SCALE;
      speed_scale_ = msg->linear_value;
      speed_limit_linear_ = std::numeric_limits<double>::max();
      speed_limit_angular_ = std::numeric_limits<double>::max();
      break;
    case ModifierControl::TYPE_SPEED_LIMIT_SCALE:
      current_mode_ = SpeedMode::RATIO_LIMIT_SCALE;
      ratio_limit_linear_ = msg->linear_value;
      ratio_limit_angular_ = msg->angular_value;
      speed_scale_ = 1.0; 
      speed_limit_linear_ = std::numeric_limits<double>::max();
      speed_limit_angular_ = std::numeric_limits<double>::max();
      break;
    default:
      break;
    }
}

}  // namespace velocity_modifier
