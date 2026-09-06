#include "velocity_modifier/velocity_modifier_node.hpp"

#include <algorithm> // For std::clamp

namespace velocity_modifier
{

namespace
{

/// 두 축에 "공통 배율"을 적용해 상한을 지키기 위한 배율을 구한다.
///
/// 축별로 std::clamp 하면 한쪽 축만 잘려서 명령의 곡률(v/w)이 바뀐다.
/// 감속만 해야 할 상황에서 로봇이 계획과 다른 궤적으로 움직이게 되므로,
/// 반드시 두 축에 같은 배율을 곱해야 한다.
double commonLimitScale(double v, double w, double linear_max, double angular_max)
{
  double scale = 1.0;
  const double abs_v = std::abs(v);
  const double abs_w = std::abs(w);
  if (linear_max > 0.0 && abs_v > linear_max) {
    scale = std::min(scale, linear_max / abs_v);
  }
  if (angular_max > 0.0 && abs_w > angular_max) {
    scale = std::min(scale, angular_max / abs_w);
  }
  return scale;
}

/// 차동구동에서 좌/우 바퀴 속도 중 빠른 쪽의 크기.
/// 모터가 실제로 도는지(데드밴드), 그리고 얼마나 빨리 도는지는 몸체 속도가 아니라
/// 이 값으로 정해진다.
double maxWheelSpeed(double v, double w, double wheel_separation)
{
  const double half_b = wheel_separation / 2.0;
  return std::max(std::abs(v - w * half_b), std::abs(v + w * half_b));
}

}  // namespace

VelocityModifierNode::VelocityModifierNode(const rclcpp::NodeOptions & options)
: Node("velocity_modifier_node", options)
{
  RCLCPP_INFO(this->get_logger(), "Velocity Modifier Node is initializing...");

  this->declare_parameter<double>("min_abs_linear_vel", 0.03);
  this->declare_parameter<double>("min_abs_angular_vel", 0.03);
  this->declare_parameter<double>("ratio_scaling_max_linear_vel", 0.30);
  this->declare_parameter<double>("ratio_scaling_max_angular_vel", 0.20);

  // [추가] 회복 구간 저속 보정의 판단 기준 (바퀴 속도 기준).
  // 모터가 안 도는 것은 "바퀴가 너무 느려서"이지 "몸체 v 가 작아서"가 아니다.
  //
  // 실측: 실제 AMR 에서 cmd_vel 이 0.01 m/s 보다 작으면 잘 움직이지 않았다.
  // (직진 주행으로 측정했으므로 몸체 속도 = 바퀴 속도. 바퀴 기준값으로 그대로 쓸 수 있다.
  //  현장 nav2_params.yaml 의 FollowPath.v_linear_min 도 같은 0.01 이다.)
  // 0.01 은 "겨우 움직이는" 경계라 2배 마진을 둬 0.02 로 잡는다.
  // 예전 값 0.03 은 실제 데드밴드의 3배여서, 움직이는 데 아무 문제 없는 지령까지
  // 끌어올려 회전반경을 뒤틀었다.
  this->declare_parameter<double>("min_abs_wheel_vel", 0.02);
  // roboteq.yaml 의 track_width 와 같아야 한다.
  this->declare_parameter<double>("wheel_separation", 0.526);

  // [Added] Phase 2 (1.0s ~ 2.5s) 속도 제한 파라미터 (기본값 0.3)
  this->declare_parameter<double>("startup_phase2_limit", 0.3);

  // [추가] 주행 초기(Phase 1, 2 공통) 각속도 제한.
  // 선속도 제한만으로는 제자리 선회를 못 막는다. 현장 컨트롤러의 v_angular_max 가
  // 1.8 이므로, 이 값은 주행 시작 2.5초 동안 실제로 걸리는 상한이다.
  this->declare_parameter<double>("startup_angular_limit", 0.5);


  this->get_parameter("min_abs_linear_vel", min_abs_linear_vel_);
  this->get_parameter("min_abs_angular_vel", min_abs_angular_vel_);
  this->get_parameter("ratio_scaling_max_linear_vel", ratio_scaling_max_linear_vel_);
  this->get_parameter("ratio_scaling_max_angular_vel", ratio_scaling_max_angular_vel_);
  this->get_parameter("min_abs_wheel_vel", min_abs_wheel_vel_);
  this->get_parameter("wheel_separation", wheel_separation_);

   // [Added] 파라미터 읽기
  this->get_parameter("startup_phase2_limit", startup_phase2_limit_);
  this->get_parameter("startup_angular_limit", startup_angular_limit_);
  
  // [Added] 시간 초기화 (0초로 설정)
  driving_start_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type()); 

  RCLCPP_INFO(this->get_logger(), "Startup Phase2 Limit: %.3f m/s", startup_phase2_limit_);
  RCLCPP_INFO(this->get_logger(), "Startup Angular Limit: %.3f rad/s", startup_angular_limit_);


  // [수정] cmd_vel 은 MutuallyExclusive 여야 한다.
  //
  // Reentrant 는 같은 콜백을 여러 스레드가 동시에 실행하는 것을 허용한다.
  // main.cpp 가 MultiThreadedExecutor 를 쓰므로 실제로 그렇게 동작하는데,
  // data_mutex_ 는 직렬화만 할 뿐 "도착 순서"는 보장하지 않는다.
  // 그래서 뒤에 온 메시지가 먼저 락을 잡으면 이전 지령이 나중에 발행된다.
  //
  //   컨트롤러: #1(v=0.30) -> #2(v=0.00, 정지)
  //   발행:     cmd_vel(0.00) -> cmd_vel(0.30)   <- 정지 지령이 덮여 사라진다
  //
  // roboteq 드라이버는 cmdvel_callback 에서만 시리얼에 쓰고 재전송 경로가 없어서
  // (cmdvel_loop() 는 빈 함수, cmdvel_run() 은 #ifdef _CMDVEL_FORCE_RUN),
  // 이 stale 값이 다음 메시지가 올 때까지 모터에 그대로 유지된다.
  //
  // 이 노드의 콜백은 산술 몇 줄 + publish 라 마이크로초 단위다. 실측상
  // 2000 Hz 로 1000개를 몰아넣어도 유실 0 / 역전 0 / 최대 지연 1.32 ms 였다.
  // 즉 직렬화해도 큐가 쌓이지 않는다(구독 큐도 KEEP_LAST(10) 이라 무한히 밀리지 않는다).
  cb_group_cmd_vel_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  // 아래 둘은 cmd_vel 과 다른 그룹이라 병렬로 처리된다.
  // (속도 제한 변경/상태 갱신이 cmd_vel 처리 뒤에서 대기하지 않는다)
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

  // [추가] 토픽과 같은 기능의 서비스. 기존 토픽 경로는 그대로 둔다.
  //
  // 토픽은 보내고 나면 반영됐는지 알 수 없고, 값이 제한됐는지도 알 수 없다.
  // 응답이 필요한 호출자를 위해 서비스를 함께 제공한다.
  // 적용 로직은 controlCallback 과 applyControl() 하나를 공유하므로 두 경로가
  // 갈라질 수 없다.
  //
  //   ros2 service call /velocity_modifier/set_control
  //       robot_interfaces/srv/SetVelocityModifier
  //       "{control: {command_type: 1, linear_value: 0.2, angular_value: 0.3}}"
  //
  // 콜백 그룹은 토픽과 같은 것을 쓴다. 같은 상태를 만지므로 cmd_vel 처리와는
  // 분리하되, 서로에 대해서는 data_mutex_ 로 직렬화된다.
  control_service_ = this->create_service<SetVelocityModifier>(
    "velocity_modifier/set_control",
    std::bind(&VelocityModifierNode::setControlService, this,
              std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    cb_group_control_);

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
    // DRIVING / IDLE 등 그 밖의 정상 상태. robot_status_manager 가 1초 주기 타이머로
    // 계속 발행하므로, 매번 로그를 남기면 안 된다(예전에는 "unknown command" 로 찍혀
    // 정상 상태가 오류처럼 보였다). 모드가 실제로 바뀔 때만 남긴다.
    if (recovery_mode_ == true) {
      recovery_mode_ = false;
      RCLCPP_INFO(
        this->get_logger(), "Recovery mode DISABLED (status: '%s').", msg->data.c_str());
    }
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

    // 제한 값이 설정되었으면 선속도 상한과 각속도 상한을 함께 적용한다.
    //
    // [수정] 예전에는 선속도만 봤다.
    //   if (abs_vx > startup_limit) { scale = startup_limit / abs_vx; ... }
    // 이러면 제자리 선회 지령(선속도 0, 각속도만 큰 지령)이 조건에 아예 걸리지 않아
    // 주행 시작 직후에도 각속도가 무제한으로 나간다. 실측: 주행 시작 1초 안에
    // (0.000, 1.500) 이 그대로 통과했다. 현장 컨트롤러의 v_angular_max 는 1.8 이라
    // 이 구간에서 로봇이 제자리에서 급하게 돌 수 있다.
    //
    // 두 상한을 "공통 배율"로 적용하므로 곡률(v/w)은 그대로 유지된다.
    // (이 파일의 다른 상한 처리와 같은 방식이다. commonLimitScale 주석 참고)
    if (startup_limit > 0.0) {
      const double orig_vx = adjusted_vel->linear.x;
      const double orig_wz = adjusted_vel->angular.z;

      const double scale = commonLimitScale(
        orig_vx, orig_wz, startup_limit, startup_angular_limit_);

      if (scale < 1.0) {
        adjusted_vel->linear.x = orig_vx * scale;
        adjusted_vel->angular.z = orig_wz * scale;

        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
          "Startup Limit Active (t=%.2fs): lin<=%.2f ang<=%.2f | "
          "(%.3f, %.3f) -> (%.3f, %.3f)",
          elapsed_sec, startup_limit, startup_angular_limit_,
          orig_vx, orig_wz, adjusted_vel->linear.x, adjusted_vel->angular.z);
      }
    }
  }
  // ==========================================================





  // [기존 로직: 2. 모드별 처리]
  // 주행 초기 제한이 걸렸더라도, 사용자가 설정한 Global Limit(STANDARD_LIMIT 등)이
  // 더 작다면 그 값으로 덮어씌워지므로 안전함 (std::clamp 사용 덕분)
  //
  // [수정] 세 모드 모두 상한을 "공통 배율"로 적용한다.
  // 예전에는 STANDARD_LIMIT / STANDARD_SCALE 이 두 축을 각각 std::clamp 했다.
  // 그러면 한쪽 축만 잘려 곡률(v/w)이 바뀐다. 예를 들어 지령 (0.5, 0.3)(반경 1.67 m)에
  // linear 0.2 / angular 0.3 제한이 걸리면 (0.2, 0.3) 즉 반경 0.67 m 가 되어,
  // 감속만 해야 할 상황에서 로봇이 계획과 다른 궤적으로 움직인다.
  // (RATIO_LIMIT_SCALE 은 원래 공통 배율로 맞게 처리하고 있었다. 이제 셋 다 같은
  //  commonLimitScale() 을 쓰므로 이 결함이 다시 생길 수 없다.)
  if (current_mode_ == SpeedMode::STANDARD_LIMIT) {
    adjusted_vel->linear.x *= speed_scale_; // (speed_scale_은 1.0일 것)
    adjusted_vel->angular.z *= speed_scale_;
    const double scale = commonLimitScale(
      adjusted_vel->linear.x, adjusted_vel->angular.z,
      speed_limit_linear_, speed_limit_angular_);
    adjusted_vel->linear.x *= scale;
    adjusted_vel->angular.z *= scale;
  } 
  else if (current_mode_ == SpeedMode::STANDARD_SCALE) {
    adjusted_vel->linear.x *= speed_scale_;
    adjusted_vel->angular.z *= speed_scale_;
    // (limit은 max일 것)
    const double scale = commonLimitScale(
      adjusted_vel->linear.x, adjusted_vel->angular.z,
      speed_limit_linear_, speed_limit_angular_);
    adjusted_vel->linear.x *= scale;
    adjusted_vel->angular.z *= scale;
  }
  else if (current_mode_ == SpeedMode::RATIO_LIMIT_SCALE) {
    // 1.0 스케일 적용 (다른 모드와 일관성)
    adjusted_vel->linear.x *= speed_scale_; // (speed_scale_은 1.0일 것)
    adjusted_vel->angular.z *= speed_scale_;

    // 두 제한을 모두 만족하는 공통 배율 (속도를 증가시키지는 않는다)
    const double scale = commonLimitScale(
      adjusted_vel->linear.x, adjusted_vel->angular.z,
      ratio_limit_linear_, ratio_limit_angular_);

    adjusted_vel->linear.x *= scale;
    adjusted_vel->angular.z *= scale;
  }
  // [수정된 로직 끝]

  RCLCPP_DEBUG(this->get_logger(), "1▶ vx: %.11lf, wz: %.11lf", adjusted_vel->linear.x, adjusted_vel->angular.z);

   // 3. 저속 보정 로직 (모터 데드밴드 대응)
  //
  // [목적] 실제 모터는 지령이 너무 낮으면 아예 돌지 않는다(스톨). 그래서 회복 구간에서
  //        너무 느린 지령이 오면 실제로 움직일 수 있는 최소 속도까지 올려준다.
  //
  // [수정 배경] 예전 로직은 "몸체" 속도(linear.x, angular.z)를 각각 최소값과 비교했다.
  //   그런데 모터가 도는지 마는지는 "바퀴" 속도로 정해진다. 차동구동에서 이 둘은 다르다.
  //     v_left  = v - w * (b/2),  v_right = v + w * (b/2)
  //   제자리 선회 지령은 몸체 v 가 0에 가깝지만 두 바퀴는 반대 방향으로 빠르게 돈다.
  //   현장 회복 구간 실측:
  //     (v=-0.0132, w=-0.2736) -> 바퀴 +0.059 / -0.085 m/s  (충분히 도는 중)
  //   그런데 예전 로직은 |v|=0.013 < 0.03 만 보고 부스트를 걸었고, 그 배율이 각속도까지
  //   밀어올린 뒤 각속도 상한에 잘려서 회전반경이 0.048 m -> 0.15 m 로 부풀었다.
  //   게다가 회전반경 0.15 m 미만인 모든 지령이 똑같이 (0.03, 0.20) 으로 뭉개져,
  //   컨트롤러가 지령을 바꿔도 로봇은 늘 같은 원만 돌았다(제어 루프 단절).
  //   반대로 거의 직진하는 지령(v=0.0116, w=0.00015)에서는 각속도가 최소값보다 작아
  //   배율이 200배까지 폭주해 선속도가 0.30 m/s 로 튀었다(26배).
  //
  // [현재 로직] 좌/우 바퀴 속도 중 큰 쪽이 데드밴드보다 느릴 때만, 두 축에 "공통 배율"을
  //   곱해 그 바퀴가 데드밴드에 닿게 한다. 공통 배율이므로 회전반경(v/w)이 보존된다.
  //   상한도 같은 방식으로 공통 적용한다.
  if (recovery_mode_ == true) {
    const double vx = adjusted_vel->linear.x;
    const double wz = adjusted_vel->angular.z;
    const double epsilon = 1e-9;

    const double max_wheel = maxWheelSpeed(vx, wz, wheel_separation_);

    // 정지 지령(양쪽 바퀴 모두 0)은 그대로 둔다. 억지로 움직이면 안 된다.
    // 빠른 쪽 바퀴가 데드밴드(min_abs_wheel_vel_, 실측 0.01 m/s)보다 느릴 때만 개입한다.
    if (max_wheel > epsilon && max_wheel < min_abs_wheel_vel_) {
      // 느린 쪽 바퀴가 아니라 "빠른 쪽" 바퀴를 데드밴드에 맞춘다.
      // 느린 쪽을 기준으로 하면 배율이 과하게 커진다.
      const double scale = min_abs_wheel_vel_ / max_wheel;

      double new_vx = vx * scale;
      double new_wz = wz * scale;

      // 상한도 두 축 공통 배율로 적용해야 곡률이 유지된다.
      const double clip = commonLimitScale(
        new_vx, new_wz, ratio_scaling_max_linear_vel_, ratio_scaling_max_angular_vel_);

      // 저속 보정은 "올리는" 로직이므로, 상한에 걸려 결과적으로 원래보다 느려지면
      // 적용하지 않는다. 위쪽 RATIO_LIMIT_SCALE 블록의 `min(scale, 1.0)` 과 대칭.
      const double net = scale * clip;
      if (net > 1.0) {
        // 이 보정은 매 사이클 반복될 수 있으므로 throttle 한다.
        // (개입한 경우에만 찍히므로, 이 로그가 보이면 실제로 값을 바꾸고 있다는 뜻)
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          " deadband boost: max_wheel=%.5lf -> scale=%.3lf clip=%.3lf | "
          "(%.5lf, %.5lf) -> (%.5lf, %.5lf)",
          max_wheel, scale, clip, vx, wz, new_vx * clip, new_wz * clip);
        adjusted_vel->linear.x = new_vx * clip;
        adjusted_vel->angular.z = new_wz * clip;
      }
    }
  } // End of if (recovery_mode_)
  
  RCLCPP_DEBUG(this->get_logger(), "2▶ vx: %.11lf, wz: %.11lf", adjusted_vel->linear.x, adjusted_vel->angular.z);

  adjusted_cmd_vel_pub_->publish(std::move(adjusted_vel));
}

bool VelocityModifierNode::applyControl(
  const ModifierControl & req, ModifierControl & applied, std::string & message)
{
  const std::lock_guard<std::mutex> lock(data_mutex_);

  applied = req;
  bool ok = true;

  switch (req.command_type) {
    case ModifierControl::TYPE_SPEED_LIMIT:
      current_mode_ = SpeedMode::STANDARD_LIMIT;
      speed_limit_linear_ = req.linear_value;
      speed_limit_angular_ = req.angular_value;
      speed_scale_ = 1.0;
      RCLCPP_INFO(
        this->get_logger(), "Set Mode: STANDARD_LIMIT. Linear: %.2f, Angular: %.2f",
        speed_limit_linear_, speed_limit_angular_);
      message = "STANDARD_LIMIT applied";
      break;

    case ModifierControl::TYPE_SPEED_SCALE:
    {
      current_mode_ = SpeedMode::STANDARD_SCALE;

      // [수정] 배율을 [0, 1] 로 제한한다.
      //
      // 이 노드는 속도를 "줄이는" 마지막 관문이고, 출력이 곧 /cmd_vel 이라 바로
      // 모터 드라이버로 간다. 그런데 이 모드만은 배율을 검증 없이 그대로 대입하고
      // 상한마저 DBL_MAX 로 열어두기 때문에, 1.0 보다 큰 값이 한 번 들어오면
      // 그대로 증폭되어 나갔다.
      //   실측: scale=5.0 일 때 (0.300, 0.100) -> (1.500, 0.500)
      // 상류 Nav2 파라미터를 아무리 조여도 이 노드 뒤에서 풀려버린다.
      // 단위 착오(퍼센트를 배율로 전송)나 패킷 오해석 한 번이면 5배 가속 지령이 된다.
      //
      // 속도 제한 노드가 속도를 올리는 것은 어떤 경우에도 의도가 아니므로 1.0 에서 자른다.
      // 음수도 막는다(부호가 뒤집히면 로봇이 반대로 간다).
      const bool finite = std::isfinite(req.linear_value);
      const double clamped =
        finite ? std::clamp<double>(req.linear_value, 0.0, 1.0) : 0.0;

      if (!finite || req.linear_value < 0.0 || req.linear_value > 1.0) {
        RCLCPP_WARN(
          this->get_logger(),
          "STANDARD_SCALE: 배율 %.3f 는 허용 범위 [0, 1] 를 벗어난다. %.3f 로 제한한다.",
          req.linear_value, clamped);
        message = "STANDARD_SCALE applied (scale clamped to [0, 1])";
      } else {
        message = "STANDARD_SCALE applied";
      }

      speed_scale_ = clamped;
      applied.linear_value = static_cast<float>(clamped);
      // 이 모드에서 angular_value 는 쓰이지 않는다. 응답에서도 그 사실이 드러나게 0 으로 둔다.
      applied.angular_value = 0.0f;

      speed_limit_linear_ = std::numeric_limits<double>::max();
      speed_limit_angular_ = std::numeric_limits<double>::max();
      RCLCPP_INFO(this->get_logger(), "Set Mode: STANDARD_SCALE. Scale: %.2f", speed_scale_);
      break;
    }

    case ModifierControl::TYPE_SPEED_LIMIT_SCALE:
      current_mode_ = SpeedMode::RATIO_LIMIT_SCALE;
      ratio_limit_linear_ = req.linear_value;
      ratio_limit_angular_ = req.angular_value;
      // 다른 모드 설정 초기화
      speed_scale_ = 1.0;
      speed_limit_linear_ = std::numeric_limits<double>::max();
      speed_limit_angular_ = std::numeric_limits<double>::max();

      RCLCPP_INFO(
        this->get_logger(), "Set Mode: RATIO_LIMIT_SCALE. Linear: %.2f, Angular: %.2f",
        ratio_limit_linear_, ratio_limit_angular_);
      message = "RATIO_LIMIT_SCALE applied";
      break;

    default:
      RCLCPP_WARN(
        this->get_logger(), "Received control command with unknown type: %d", req.command_type);
      message = "unknown command_type: " + std::to_string(req.command_type) +
                " (mode unchanged)";
      ok = false;
      break;
  }

  if (!ok) {
    // 실패했으면 "지금 유지 중인 값" 을 돌려준다. 호출자가 현재 상태를 알 수 있어야 한다.
    switch (current_mode_) {
      case SpeedMode::STANDARD_LIMIT:
        applied.command_type = ModifierControl::TYPE_SPEED_LIMIT;
        applied.linear_value = static_cast<float>(speed_limit_linear_);
        applied.angular_value = static_cast<float>(speed_limit_angular_);
        break;
      case SpeedMode::STANDARD_SCALE:
        applied.command_type = ModifierControl::TYPE_SPEED_SCALE;
        applied.linear_value = static_cast<float>(speed_scale_);
        applied.angular_value = 0.0f;
        break;
      case SpeedMode::RATIO_LIMIT_SCALE:
        applied.command_type = ModifierControl::TYPE_SPEED_LIMIT_SCALE;
        applied.linear_value = static_cast<float>(ratio_limit_linear_);
        applied.angular_value = static_cast<float>(ratio_limit_angular_);
        break;
    }
  }

  return ok;
}

// 기존 토픽 경로. 동작은 그대로 두고 적용 로직만 applyControl 로 옮겼다.
// 토픽과 서비스가 각자 모드를 세팅하면 언젠가 갈라지므로 한 곳에만 둔다.
void VelocityModifierNode::controlCallback(const ModifierControl::SharedPtr msg)
{
  ModifierControl applied;
  std::string message;
  applyControl(*msg, applied, message);
}

// [추가] 서비스 경로. 토픽과 같은 일을 하되 결과를 돌려준다.
//
// 토픽은 보내고 나면 반영됐는지 알 방법이 없고, 값이 제한됐는지도 알 수 없다.
// (예: STANDARD_SCALE 의 배율은 [0, 1] 로 잘린다)
// 서비스는 success / message / 실제 적용된 값을 함께 돌려준다.
void VelocityModifierNode::setControlService(
  const std::shared_ptr<SetVelocityModifier::Request> request,
  std::shared_ptr<SetVelocityModifier::Response> response)
{
  std::string message;
  response->success = applyControl(request->control, response->applied, message);
  response->message = message;

  RCLCPP_INFO(
    this->get_logger(), "[service] set_control: type=%u req=(%.3f, %.3f) -> %s | %s",
    request->control.command_type, request->control.linear_value,
    request->control.angular_value,
    response->success ? "OK" : "REJECTED", response->message.c_str());
}

}  // namespace velocity_modifier