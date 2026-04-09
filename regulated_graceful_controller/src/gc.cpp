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







if (std::abs(limited_w) > std::abs(max_w_limit)) {
          limited_w = max_w_limit;
          
          // [수정 핵심 1] 곡률(kappa)이 1.0보다 큰 '급커브(회전반경 1m 이내)'에서만 비율 강제 유지!
          // 160도 같은 완만한 커브(|kappa| <= 1.0)에서는 속도를 깎지 않고 그대로 살려둡니다.
          if (std::abs(kappa) > 1.0) {
              limited_v = limited_w / kappa; 
          }
      }

// [수정 핵심 2] 파라미터 최대치를 넘을 경우, 궤적(비율)을 유지하며 동기화 축소 (Proportional Scaling)
      double v_scale = 1.0;
      double w_scale = 1.0;

      // 1. 선속도가 최대치를 초과하면 얼마나 줄여야 하는지 비율 계산
      if (std::abs(limited_v) > params_->v_linear_max) {
          v_scale = params_->v_linear_max / std::abs(limited_v);
      }
      
      // 2. 각속도가 최대치를 초과하면 얼마나 줄여야 하는지 비율 계산
      if (std::abs(limited_w) > params_->v_angular_max) {
          w_scale = params_->v_angular_max / std::abs(limited_w);
      }

      // 3. 둘 중 더 빡세게 줄여야 하는(더 작은) 스케일을 선택하여 동시에 곱함
      double final_scale = std::min(v_scale, w_scale);

      limited_v *= final_scale;
      limited_w *= final_scale;

      final_cmd.twist.linear.x = limited_v;
      final_cmd.twist.angular.z = limited_w;
  }

  // 최종 명령 업데이트 및 리턴
  last_cmd_vel_ = final_cmd.twist;
  return final_cmd;
}
