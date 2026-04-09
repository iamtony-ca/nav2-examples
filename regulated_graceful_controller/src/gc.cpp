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








if (std::abs(limited_w) > std::abs(max_w_limit)) {
          limited_w = max_w_limit;
          
          // 회전 반경(Turning Radius) 계산 (kappa = 1.0 이면 반경 1m)
          double turning_radius = 100.0;
          if (std::abs(kappa) > 0.001) turning_radius = 1.0 / std::abs(kappa);
          
          // ==========================================================
          // 🎛️ [튜닝 포인트] 곡률 기반 감속 완화 (Curvature Smoothing)
          // ==========================================================
          double min_radius = 1.0;         // 개입 기준 반경 (1.0m 이하의 급커브에서만 감속)
          double smoothing_factor = 0.5;   // 감속 강도 (1.0: 확 깎음 / 0.0: 안 깎고 밀림)

          if (turning_radius < min_radius) {
              // 1. 수학적으로 궤적을 100% 지키기 위해 필요한 (많이 깎인) 선속도
              double strict_v = limited_w / kappa; 
              
              // 2. 확 깎지 않고, 원래 가려던 속도(limited_v)와 혼합하여 부드럽게 타협 (Blending)
              limited_v = (strict_v * smoothing_factor) + (limited_v * (1.0 - smoothing_factor));
          }
          // ==========================================================
      }

      // [수정 핵심 2] 파라미터 최대치 동기화 축소 (비율 유지 클램핑)
      // ... (아래는 지난번 답변의 비율 유지 스케일링 코드 그대로 유지) ...







// -----------------------------------------------------------------------
  // [2] Goal Tolerance 가져오기
  // -----------------------------------------------------------------------
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, velocity_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  } else {
    goal_dist_tolerance_ = pose_tolerance.position.x;
  }

  // -----------------------------------------------------------------------
  // [3] 경로 변환 및 거리 계산 (순서를 위로 끌어올림)
  // -----------------------------------------------------------------------
  auto transformed_plan = path_handler_->transformGlobalPlan(
    pose, params_->max_robot_pose_search_dist);

  // Path Integral Distance (경로상 남은 총 거리 - 목적지까지의 진짜 거리!)
  double dist_to_goal = nav2_util::geometry_utils::calculate_path_length(transformed_plan);

  geometry_msgs::msg::TransformStamped costmap_transform;
  try {
    costmap_transform = tf_buffer_->lookupTransform(
      costmap_ros_->getGlobalFrameID(), costmap_ros_->getBaseFrameID(),
      tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Could not transform %s to %s", costmap_ros_->getBaseFrameID().c_str(), costmap_ros_->getGlobalFrameID().c_str());
    throw ex;
  }

  // Euclidean Distance & Angle (물리적 직선 거리 및 각도)
  double dx = pose.pose.position.x - transformed_plan.poses.back().pose.position.x;
  double dy = pose.pose.position.y - transformed_plan.poses.back().pose.position.y;
  double euclidean_dist = std::hypot(dx, dy);
  double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);

  // =========================================================================
  // ★ [신규 추가] Steering과 Speed Profile의 분리 (Global Slowdown) ★
  // =========================================================================
  double dynamic_v_max = params_->v_linear_max;

  // 목적지까지 남은 거리가 감속 반경(예: 2.0m) 이내로 들어오면 비례 제어 시작
  if (dist_to_goal < params_->slowdown_radius) {
      double ratio = dist_to_goal / params_->slowdown_radius; // 1.0 -> 0.0 으로 수렴
      
      // 거리에 비례하여 부드럽게 감속 (도착 직전 v_linear_min 보장)
      dynamic_v_max = params_->v_linear_min + 
                      (params_->v_linear_max - params_->v_linear_min) * ratio;
  }

  // Control Law 초기화 및 동적 속도 적용
  control_law_->setCurvatureConstants(
    params_->k_phi, params_->k_delta, params_->beta, params_->lambda);
  
  // [핵심] 내부 컨트롤러의 지역적 감속 로직은 이중 개입을 막기 위해 0.1m로 무력화!
  control_law_->setSlowdownRadius(0.1); 
  
  // 방금 계산한 '목적지 거리 기반' 최대 속도를 컨트롤러에 주입
  control_law_->setSpeedLimit(params_->v_linear_min, dynamic_v_max, params_->v_angular_max);
  // =========================================================================

  // =================================================================================
  // [핵심 로직] Checker 상태 및 거리에 따른 3단계 모드 결정
  // =================================================================================
  // ... (이하 기존 코드 동일하게 유지) ...
