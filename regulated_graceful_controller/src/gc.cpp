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








// B. 선속도(X)에만 가감속 한계 적용
      double limited_v = applyKinematicLimits(
          last_cmd_vel_.linear.x, target_v, 2.5, -3.2, dt_control);
      
      // C. 제한된 선속도에 곡률을 곱해 각속도(Z)를 다시 계산
      double limited_w = limited_v * kappa;

      // D. (안전장치) 다시 계산된 각속도가 물리적 한계를 초과하면, 각속도 기준으로 다시 맞춤
      double max_w_limit = applyKinematicLimits(
          last_cmd_vel_.angular.z, target_w, 2.5, -3.2, dt_control);
      
      if (std::abs(limited_w) > std::abs(max_w_limit)) {
          limited_w = max_w_limit;
          
          // ==========================================================
          // [추가/수정된 부분] 적응형 곡률 제어 (Adaptive Curvature Preservation)
          // ==========================================================
          
          // 곡률 반경(Turning Radius) 계산
          double turning_radius = 100.0; // 기본값 (직선)
          if (std::abs(kappa) > 0.001) {
              turning_radius = 1.0 / std::abs(kappa);
          }

          // 회전 반경이 1.2m 이내인 급커브(U턴, 90도 턴)에서는 궤적 이탈 방지를 위해 선속도를 깎음
          if (turning_radius < 1.2) { 
              limited_v = limited_w / kappa; 
          }
          // 회전 반경이 1.2m 이상인 완만한 커브(160도 등)에서는 
          // limited_v 를 깎지 않고そのまま 살려둡니다! (부드러운 고속 통과)
          // ==========================================================
      }

      final_cmd.twist.linear.x = limited_v;
      final_cmd.twist.angular.z = limited_w;
  }

  // 최종 명령 업데이트 및 리턴
  last_cmd_vel_ = final_cmd.twist;
  return final_cmd;
}
