geometry_msgs::msg::TwistStamped GracefulController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity, // <-- Odom
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> param_lock(param_handler_->getMutex());

  // [1] dt 계산 (루프 주기)
  auto now = clock_->now();
  double dt_control = (now - last_control_time_).seconds();
  last_control_time_ = now;
  if (dt_control <= 0.0 || dt_control > 0.5) { dt_control = 0.05; }

  // =================================================================================
  // ★ [핵심 해결책] 가상 목줄 (Virtual Leash) 동기화 로직
  // =================================================================================
  // Odom이 비정상적인 노이즈(예: 1.4 초과)로 튈 때는 동기화를 무시하여 내부 상태 보호
  if (std::abs(velocity.linear.x) <= params_->v_linear_max + 0.5) {
      
      // 로봇의 정상적인 물리적 가속 지연(Lag) 허용치 (튜닝 포인트)
      // 이 값보다 명령이 더 앞서나가려고 하면 Odom 근처로 강제 고정됩니다.
      double max_lag_x = 0.3; // 선속도 허용 격차 
      double max_lag_w = 0.4; // 각속도 허용 격차
      
      // last_cmd_vel_ 을 [odom - max_lag, odom + max_lag] 범위 내로 가둠 (Clamping)
      last_cmd_vel_.linear.x = std::clamp(last_cmd_vel_.linear.x, 
                                          velocity.linear.x - max_lag_x, 
                                          velocity.linear.x + max_lag_x);
                                          
      last_cmd_vel_.angular.z = std::clamp(last_cmd_vel_.angular.z, 
                                           velocity.angular.z - max_lag_w, 
                                           velocity.angular.z + max_lag_w);
  }
  // =================================================================================

  geometry_msgs::msg::TwistStamped target_cmd;
  target_cmd.header = pose.header;
  // ... (이하 기존 last_cmd_vel_ 기반 로직 동일하게 유지) ...
