geometry_msgs::msg::TwistStamped GracefulController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity, // <-- velocity는 여기서 동기화용으로만 씁니다!
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> param_lock(param_handler_->getMutex());

  // [1] dt 계산 (루프 주기)
  auto now = clock_->now();
  double dt_control = (now - last_control_time_).seconds();
  last_control_time_ = now;
  if (dt_control <= 0.0 || dt_control > 0.5) { dt_control = 0.05; }

  // =================================================================================
  // [★ 신규 추가 ★] Safety Lidar 외부 개입 감지 및 상태 동기화 (State Reset)
  // =================================================================================
  // 컨트롤러가 명령한 속도와 실제 로봇의 속도 차이를 계산
  double linear_vel_error = last_cmd_vel_.linear.x - velocity.linear.x;
  
  // 허용 오차 (로봇의 가속 지연에 따라 0.1 ~ 0.15 정도로 튜닝)
  // - 정상적인 가속 지연은 이 값을 넘지 않습니다.
  // - 이 값을 넘었다는 것은 Safety Lidar 등이 강제로 브레이크를 잡았다는 뜻입니다.
  double sync_threshold = 0.15; 

  if (std::abs(linear_vel_error) > sync_threshold) {
      // 현실(odom)과 내부 상태(last_cmd)가 너무 크게 벌어지면, 현실에 맞춰 강제 리셋!
      last_cmd_vel_.linear.x = velocity.linear.x;
      last_cmd_vel_.angular.z = velocity.angular.z;
      
      // 디버깅용: 1초에 한 번만 경고 출력 (정상 작동 확인 후 주석 처리하셔도 됩니다)
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, 
        "External override detected (Error: %.2f)! Resetting internal state to match odom.", linear_vel_error);
  }
  // =================================================================================

  // 목표 명령을 임시로 담을 변수
  geometry_msgs::msg::TwistStamped target_cmd;
  
  // ... (이하 기존 코드 동일: Phase 1, 2, 3 로직 및 Post-Processing 로직 유지) ...
