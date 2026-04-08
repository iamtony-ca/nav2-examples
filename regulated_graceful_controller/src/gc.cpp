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
  // ★ [Post-Processing] 동적 윈도우(Dynamic Window) 기반 비율 보존 가감속 로직 ★
  // =================================================================================
  geometry_msgs::msg::TwistStamped final_cmd = target_cmd;
  final_cmd.twist.linear.y = 0.0; // Y속도 원천 차단

  double target_v = target_cmd.twist.linear.x;
  double target_w = target_cmd.twist.angular.z;

  // 1. [버그 픽스] 절대 목표값 하드 클램핑 (1.0으로 튀는 현상 원천 차단)
  target_v = std::clamp(target_v, -params_->v_linear_max, params_->v_linear_max);
  target_w = std::clamp(target_w, -params_->v_angular_max, params_->v_angular_max);

  // 2. 물리적 가감속 한계(Dynamic Window) 윈도우 계산
  // *현재 속도 기준으로 1틱(dt) 동안 최대로 낼 수 있는 최소/최대 속도 범위
  double acc_x = 2.5;  double dec_x = 3.2; // 추후 params_->max_accel_x 등으로 교체 가능
  double acc_w = 2.5;  double dec_w = 3.2;

  auto get_bounds = [](double current, double acc, double dec, double dt) {
      double lower, upper;
      if (current >= 0.0) {
          lower = current - (dec * dt);
          upper = current + (acc * dt);
      } else {
          lower = current - (acc * dt); // 후진 가속
          upper = current + (dec * dt); // 후진 브레이크
      }
      return std::make_pair(lower, upper);
  };

  auto bounds_v = get_bounds(last_cmd_vel_.linear.x, acc_x, dec_x, dt_control);
  auto bounds_w = get_bounds(last_cmd_vel_.angular.z, acc_w, dec_w, dt_control);

  double v_min = bounds_v.first; double v_max = bounds_v.second;
  double w_min = bounds_w.first; double w_max = bounds_w.second;

  // 3. 목표 속도를 일차적으로 물리 한계 내로 클램핑
  double limited_v = std::clamp(target_v, v_min, v_max);
  double limited_w = std::clamp(target_w, w_min, w_max);

  // 4. 고도화된 곡률(Curvature) 비율 보존 로직
  if (std::abs(target_v) > 0.001) {
      double kappa = target_w / target_v;
      
      // 선속도를 기준으로 이상적인 각속도 동기화 계산
      double synchronized_w = limited_v * kappa;
      
      if (synchronized_w < w_min || synchronized_w > w_max) {
          // [Case A] 각속도가 한계를 초과함 (각속도가 병목)
          limited_w = std::clamp(synchronized_w, w_min, w_max);
          
          // 부족한 각속도에 맞춰 선속도도 희생하여 비율 유지
          limited_v = limited_w / kappa;
          
          // [핵심 해결책] 선속도를 희생시켰는데, 그것이 로봇의 최대 감속 능력(급브레이크)을 넘는다면?
          // (이 부분이 기존 160도 커브에서 발생했던 주행 붕괴 현상의 원인입니다)
          if (limited_v < v_min || limited_v > v_max) {
              // 궤적 비율을 살짝 깨더라도 물리법칙을 우선하여 브레이크 한계까지만 감속!
              limited_v = std::clamp(limited_v, v_min, v_max); 
              limited_w = std::clamp(limited_v * kappa, w_min, w_max); // 최종 안전 클램핑
          }
      } else {
          // [Case B] 각속도가 한계 이내라면 완벽하게 비율 유지
          limited_w = synchronized_w;
      }
  } else {
      // 제자리 회전 모드
      limited_v = 0.0;
  }

  // 5. 최종 파라미터 최대치 이중 클램핑 (확실한 안전장치)
  limited_v = std::clamp(limited_v, -params_->v_linear_max, params_->v_linear_max);
  limited_w = std::clamp(limited_w, -params_->v_angular_max, params_->v_angular_max);

  // 6. 미세 떨림(모터 웅웅거림) 방지
  if (std::abs(limited_v) < 0.001) limited_v = 0.0;
  if (std::abs(limited_w) < 0.001) limited_w = 0.0;

  final_cmd.twist.linear.x = limited_v;
  final_cmd.twist.angular.z = limited_w;

  // 최종 명령 업데이트 및 리턴
  last_cmd_vel_ = final_cmd.twist;
  return final_cmd;
}
