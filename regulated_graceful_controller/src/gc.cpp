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







// =================================================================================
  // ★ [Post-Processing] 독립적 가감속(Kinematic Limits) 및 하드 클램핑(안전장치) ★
  // =================================================================================
  geometry_msgs::msg::TwistStamped final_cmd = target_cmd;
  
  // 1. 선속도 Y 원천 차단 (슬립 노이즈 방지)
  final_cmd.twist.linear.y = 0.0; 

  // 2. 컨트롤러가 계산한 순수 목표 속도
  double target_v = target_cmd.twist.linear.x;
  double target_w = target_cmd.twist.angular.z;

  // 3. [버그 픽스] 목표값 자체가 파라미터 최대치를 넘지 않도록 1차 하드 클램핑 (1.0으로 튀는 현상 원천 차단)
  target_v = std::clamp(target_v, -params_->v_linear_max, params_->v_linear_max);
  target_w = std::clamp(target_w, -params_->v_angular_max, params_->v_angular_max);

  // 4. 선속도 / 각속도 독립적 가감속 적용 (last_cmd_vel_ 기반의 부드러운 램프 곡선)
  // *참고: 2.5(가속), -3.2(감속) 값은 로봇 스펙에 맞게 추후 튜닝하세요.
  double limited_v = applyKinematicLimits(
      last_cmd_vel_.linear.x, target_v, 2.5, -3.2, dt_control);
      
  double limited_w = applyKinematicLimits(
      last_cmd_vel_.angular.z, target_w, 2.5, -3.2, dt_control);

  // 5. [안전장치] 가감속 연산 후에도 절대 한계를 넘지 않도록 2차 하드 클램핑
  limited_v = std::clamp(limited_v, -params_->v_linear_max, params_->v_linear_max);
  limited_w = std::clamp(limited_w, -params_->v_angular_max, params_->v_angular_max);

  // 6. 모터 웅웅거림(떨림) 방지를 위해 매우 작은 속도는 완전 정지 처리
  if (std::abs(limited_v) < 0.001) limited_v = 0.0;
  if (std::abs(limited_w) < 0.001) limited_w = 0.0;

  final_cmd.twist.linear.x = limited_v;
  final_cmd.twist.angular.z = limited_w;

  // 최종 명령 업데이트 및 리턴
  last_cmd_vel_ = final_cmd.twist;
  return final_cmd;
}
