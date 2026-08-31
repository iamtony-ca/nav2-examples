#include "velocity_modifier/velocity_modifier_node.hpp"

#include <algorithm> // For std::clamp

namespace velocity_modifier
{

VelocityModifierNode::VelocityModifierNode(const rclcpp::NodeOptions & options)
: Node("velocity_modifier_node", options)
{
  RCLCPP_INFO(this->get_logger(), "Velocity Modifier Node is initializing...");

  this->declare_parameter<double>("min_abs_linear_vel", 0.03);
  this->declare_parameter<double>("min_abs_angular_vel", 0.03);
  this->declare_parameter<double>("ratio_scaling_max_linear_vel", 0.30);
  this->declare_parameter<double>("ratio_scaling_max_angular_vel", 0.20);

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
  
  // /robot_status 토픽을 구독하는 로직 추가
  auto sub_recovery_opt = rclcpp::SubscriptionOptions();
  sub_recovery_opt.callback_group = cb_group_recovery_;
  // 상태 토픽은 마지막 메시지를 유지하는 것이 좋으므로 transient_local QoS 사용
  // rclcpp::QoS qos_recovery(10);
  // qos_recovery.transient_local(); 
  recovery_mode_sub_ = this->create_subscription<String>(
    // "/robot_status", qos_recovery,
    "/robot_status", 10,
    std::bind(&VelocityModifierNode::recoveryModeCallback, this, std::placeholders::_1),
    sub_recovery_opt);

  RCLCPP_INFO(this->get_logger(), "Node has been started successfully.");
}



void VelocityModifierNode::recoveryModeCallback(const String::SharedPtr msg)
{
  // lock_guard를 통해 공유 변수인 recovery_mode_를 안전하게 수정
  const std::lock_guard<std::mutex> lock(data_mutex_);
  

  std::string current_status = msg->data;

  // [Added] 주행 시작 감지 로직 (Edge Detection)
  // 이전 상태는 driving이 아니었는데, 현재 driving이 된 순간을 포착
  if (current_status == "DRIVING" && last_robot_status_ != "DRIVING") {
    driving_start_time_ = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(), "Robot started DRIVING. Engaging startup speed limits.");
  }
  // 상태가 driving이 아니게 되면 타이머 리셋 (0으로 설정하여 로직 비활성화)
  else if (current_status != "DRIVING") {
    driving_start_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  }

  // 상태 업데이트
  last_robot_status_ = current_status;



  if (msg->data == "RECOVERY_RUNNING") {
    if (recovery_mode_ == false) {
      recovery_mode_ = true;
      RCLCPP_INFO(this->get_logger(), "Recovery mode ENABLED. Low-speed correction is active.");
    }
  } else if (msg->data == "RECOVERY_SUCCESS" || msg->data == "RECOVERY_FAILURE") {
    if (recovery_mode_ == true) {
      recovery_mode_ = false;
      RCLCPP_INFO(this->get_logger(), "Recovery mode DISABLED. Low-speed correction is inactive.");
    }
  } else {
    recovery_mode_ = false;
    RCLCPP_INFO(
      this->get_logger(), "Received unknown command on /robot_status: '%s'", msg->data.c_str());
  }
}


void VelocityModifierNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // lock_guard를 통해 데이터 읽기 전 lock
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
    adjusted_vel->linear.x *= speed_scale_; // (speed_scale_은 1.0일 것)
    adjusted_vel->angular.z *= speed_scale_;
    adjusted_vel->linear.x = std::clamp(
      adjusted_vel->linear.x, -speed_limit_linear_, speed_limit_linear_);
    adjusted_vel->angular.z = std::clamp(
      adjusted_vel->angular.z, -speed_limit_angular_, speed_limit_angular_);
  } 
  else if (current_mode_ == SpeedMode::STANDARD_SCALE) {
    adjusted_vel->linear.x *= speed_scale_;
    adjusted_vel->angular.z *= speed_scale_;
    // (limit은 max일 것)
    adjusted_vel->linear.x = std::clamp(
      adjusted_vel->linear.x, -speed_limit_linear_, speed_limit_linear_);
    adjusted_vel->angular.z = std::clamp(
      adjusted_vel->angular.z, -speed_limit_angular_, speed_limit_angular_);
  }
  else if (current_mode_ == SpeedMode::RATIO_LIMIT_SCALE) {
    // 1.0 스케일 적용 (다른 모드와 일관성)
    adjusted_vel->linear.x *= speed_scale_; // (speed_scale_은 1.0일 것)
    adjusted_vel->angular.z *= speed_scale_;

    double abs_vx = std::abs(adjusted_vel->linear.x);
    double abs_wz = std::abs(adjusted_vel->angular.z);
    double scale = 1.0;
    
    // 비율 계산 (0으로 나누기 방지)
    double linear_ratio = 1.0;
    if (abs_vx > 1e-6) {
      linear_ratio = ratio_limit_linear_ / abs_vx;
    }
    double angular_ratio = 1.0;
    if (abs_wz > 1e-6) {
      angular_ratio = ratio_limit_angular_ / abs_wz;
    }

    // 두 제한을 모두 만족해야 하므로, 더 작은 비율(더 많이 줄여야 하는)을 선택
    if (abs_vx > ratio_limit_linear_ || abs_wz > ratio_limit_angular_) {
      scale = std::min(linear_ratio, angular_ratio);
    }
    
    // 1.0보다 큰 값으로 스케일링되지 않도록 (즉, 속도를 증가시키지 않도록)
    scale = std::min(scale, 1.0); 

    adjusted_vel->linear.x *= scale;
    adjusted_vel->angular.z *= scale;
  }
  // [수정된 로직 끝]

  RCLCPP_INFO(this->get_logger(), "1▶ vx: %.11lf, wz: %.11lf", adjusted_vel->linear.x, adjusted_vel->linear.z);

   // 3. 저속 보정 로직 ( numerically stable version )
  if (recovery_mode_ == true) {
    RCLCPP_INFO(this->get_logger(), "recovery_mode_ == true #################");
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
      RCLCPP_INFO(this->get_logger(), " scale: %.5lf, new_vx: %.5lf, new_wz: %.5lf", scale, new_vx, new_wz);

      // // 이 로직으로 계산된 값에 대해서만 특별 상한선 적용
      adjusted_vel->linear.x = std::clamp(
        new_vx, -ratio_scaling_max_linear_vel_, ratio_scaling_max_linear_vel_);
      adjusted_vel->angular.z = std::clamp(
        new_wz, -ratio_scaling_max_angular_vel_, ratio_scaling_max_angular_vel_);
    }
  } // End of if (recovery_mode_)
  
  RCLCPP_INFO(this->get_logger(), "2▶ vx: %.11lf, wz: %.11lf", adjusted_vel->linear.x, adjusted_vel->linear.z);

  adjusted_cmd_vel_pub_->publish(std::move(adjusted_vel));
}

void VelocityModifierNode::controlCallback(const ModifierControl::SharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(data_mutex_);

  switch (msg->command_type) {
    case ModifierControl::TYPE_SPEED_LIMIT:
      current_mode_ = SpeedMode::STANDARD_LIMIT;
      speed_limit_linear_ = msg->linear_value;
      speed_limit_angular_ = msg->angular_value;
      speed_scale_ = 1.0;
      RCLCPP_INFO(
        this->get_logger(), "Set Mode: STANDARD_LIMIT. Linear: %.2f, Angular: %.2f",
        speed_limit_linear_, speed_limit_angular_);
      break;

    case ModifierControl::TYPE_SPEED_SCALE:
      current_mode_ = SpeedMode::STANDARD_SCALE;
      speed_scale_ = msg->linear_value;
      speed_limit_linear_ = std::numeric_limits<double>::max();
      speed_limit_angular_ = std::numeric_limits<double>::max();
      RCLCPP_INFO(this->get_logger(), "Set Mode: STANDARD_SCALE. Scale: %.2f", speed_scale_);
      break;

    case ModifierControl::TYPE_SPEED_LIMIT_SCALE:
      current_mode_ = SpeedMode::RATIO_LIMIT_SCALE;
      ratio_limit_linear_ = msg->linear_value;
      ratio_limit_angular_ = msg->angular_value;
      // 다른 모드 설정 초기화
      speed_scale_ = 1.0; 
      speed_limit_linear_ = std::numeric_limits<double>::max();
      speed_limit_angular_ = std::numeric_limits<double>::max();

      RCLCPP_INFO(
        this->get_logger(), "Set Mode: RATIO_LIMIT_SCALE. Linear: %.2f, Angular: %.2f",
        ratio_limit_linear_, ratio_limit_angular_);
      break;

    default:
      RCLCPP_WARN(
        this->get_logger(), "Received control command with unknown type: %d", msg->command_type);
      break;
  }
}

}  // namespace velocity_modifier
