// [1] dt 계산 (루프 주기)
  auto now = clock_->now();
  double dt_control = (now - last_control_time_).seconds();
  last_control_time_ = now;
  if (dt_control <= 0.0 || dt_control > 0.5) { dt_control = 0.05; }

  // =================================================================================
  // ★ [핵심 해결책] 노이즈에 강인한 단방향 가상 목줄 (Magnitude-based One-Sided Leash)
  // =================================================================================
  
  // 1. Odom 벡터의 실제 절대 속력(Speed) 계산 (y축 슬립이나 x축 음수 노이즈 무시)
  double actual_speed = std::hypot(velocity.linear.x, velocity.linear.y);
  double actual_w = std::abs(velocity.angular.z);

  // 2. 가상 목줄의 길이 (정상적인 물리적 가속 지연 허용치)
  double max_lag_x = 0.3; 
  double max_lag_w = 0.4; 

  // 3. 선속도(Linear) 단방향 목줄 체결
  // [논리] 컨트롤러의 목표 속도가 실제 속도보다 'max_lag_x' 이상 터무니없이 클 때만!
  // 즉, Safety Lidar 등이 강제로 로봇을 세웠을 때만 last_cmd_vel_ 을 현실로 끌어내립니다.
  if (std::abs(last_cmd_vel_.linear.x) > actual_speed + max_lag_x) {
      last_cmd_vel_.linear.x = std::copysign(actual_speed + max_lag_x, last_cmd_vel_.linear.x);
  }

  // 4. 각속도(Angular) 단방향 목줄 체결
  if (std::abs(last_cmd_vel_.angular.z) > actual_w + max_lag_w) {
      last_cmd_vel_.angular.z = std::copysign(actual_w + max_lag_w, last_cmd_vel_.angular.z);
  }
  // =================================================================================

  geometry_msgs::msg::TwistStamped target_cmd;
  target_cmd.header = pose.header;
  // ... (이하 기존 로직 동일하게 유지) ...
