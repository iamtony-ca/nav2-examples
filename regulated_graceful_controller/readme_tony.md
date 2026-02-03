custom goal_checker(stable_stopped_goal_checker) 에 dependency를 갖도록 소스코드 수정.

graceful_controller 자체적으로 goal_tolerance 계산을 안 하고, goal_checker 의 결과만 받아서 로직에 사용.

modified in regulated_graceful_controller/src/graceful_controller.cpp

###########################################  
reverse graceful 
Robotics Software Developer로서 Nav2 Controller의 내부 로직을 수정하려는 시도는 매우 훌륭합니다. 특히 `graceful_controller`는 곡률 기반의 부드러운 제어를 제공하므로, 이를 후진 전용으로 개조하면 주차나 좁은 공간에서의 후진 조작감이 매우 좋아질 것입니다.

제공해준 코드는 이미 `Phase 3 (제자리 회전)`와 `Phase 2 (안정화)` 로직이 커스텀되어 있습니다. **"완벽한 후진 주행(Backward-Only)"**을 위해서는 단순히 마이너스 속도를 주는 것을 넘어, **로봇이 목표 지점을 자신의 '등(Back)'으로 바라보도록 인식체계를 뒤집어야 합니다.**

다음 4가지 핵심 수정 포인트를 제시합니다.

---

### 수정 전략 핵심

1. **Target Selection (목표점 선정):** 로봇은 경로상의 점을 향해 갈 때, 그 점이 내 '뒤'에 있다고 인식해야 하며, 경로의 방향(Orientation)도 뒤집어서 계산해야 Control Law가 올바른 후진 조향각을 내놓습니다.
2. **Initial Rotation (초기 회전):** 출발 전 제자리 회전 시, 로봇의 앞이 아닌 **뒤(Rear)**가 목표점을 향하도록 회전해야 합니다.
3. **Rotate To Target (제자리 회전 함수):** 목표 각도와의 오차(Error)를 계산할 때 180도() 위상 차이를 고려해야 합니다.
4. **Phase 3 (Final Rotation):** 최종 목표 지점에 도달해서 자세를 잡을 때도, 로봇의 뒤가 목표 자세와 일치하도록 해야 합니다.

---

### 상세 코드 수정 가이드

#### 1. `computeVelocityCommands` - 경로 추종 로직 강제 후진화

가장 중요한 부분입니다. `simulateTrajectory`로 진입하기 전, 타겟 포즈의 방향을 강제로 뒤집고 후진 플래그를 고정해야 합니다.

**[수정 위치: `graceful_controller.cpp` 약 245~255라인 부근]**

```cpp
    // ... (기존 코드: if (dist_to_target < params_->min_lookahead) break; 등) ...

    /* [수정 전 원본 코드]
    bool reversing = false;
    if (params_->allow_backward && target_pose.pose.position.x < 0.0) {
      reversing = true;
      target_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
        tf2::getYaw(target_pose.pose.orientation) + M_PI);
    }
    */

    // [수정 후: 무조건 후진 모드 적용] --------------------------------------------------
    // 1. 항상 후진(reversing)을 true로 설정합니다.
    bool reversing = true; 

    // 2. Control Law가 로봇의 후면을 기준으로 조향을 계산하도록, 
    //    Target Pose의 Orientation을 180도(PI) 회전시킵니다.
    //    (경로가 동쪽을 향하면, 후진 주행 시 로봇은 서쪽을 보고 있어야 하므로)
    target_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
        angles::normalize_angle(tf2::getYaw(target_pose.pose.orientation) + M_PI));
    // --------------------------------------------------------------------------------

    // Simulate trajectory (Note: This uses the control_law which might have modified speed limits)
    nav_msgs::msg::Path local_plan;
    // ...

```

#### 2. `rotateToTarget` - 제자리 회전 로직 변경

제자리 회전 시 `0도(Front)`가 아닌 `180도(Back)`가 목표 지점을 향하도록 오차 계산을 변경합니다.

**[수정 위치: `graceful_controller.cpp` 약 355라인 부근]**

```cpp
geometry_msgs::msg::Twist GracefulController::rotateToTarget(double angle_to_target)
{
  geometry_msgs::msg::Twist vel;
  vel.linear.x = 0.0;

  // [수정 전]
  // vel.angular.z = params_->rotation_scaling_factor * angle_to_target * params_->v_angular_max;

  // [수정 후: 후면 기준 회전] ---------------------------------------------------------
  // angle_to_target은 로봇 중심에서 타겟을 바라보는 각도(atan2(y, x))입니다.
  // 로봇의 '뒤'가 타겟을 향해야 하므로, 목표 각도는 (angle_to_target)이고,
  // 현재 로봇의 뒷면 각도는 (PI)라고 생각하면 됩니다. 
  // 즉, 오차 = angle_to_target - PI 입니다.
  double error = angles::shortest_angular_distance(M_PI, angle_to_target);
  
  vel.angular.z = params_->rotation_scaling_factor * error * params_->v_angular_max;
  // --------------------------------------------------------------------------------

  vel.angular.z = std::copysign(1.0, vel.angular.z) * std::max(abs(vel.angular.z),
      params_->v_angular_min_in_place);
  return vel;
}

```

#### 3. `simulateTrajectory` - 초기 회전 판단 로직 수정

경로 생성 시뮬레이션 중 "초기 회전이 필요한가?"를 판단할 때도, 로봇의 뒤가 타겟을 향하고 있는지 확인해야 합니다.

**[수정 위치: `graceful_controller.cpp` 약 315라인 부근]**

```cpp
  // ...
  double angle_to_target =
    std::atan2(motion_target.pose.position.y, motion_target.pose.position.x);

  // [수정 전]
  // if (fabs(angle_to_target) < params_->initial_rotation_tolerance) {

  // [수정 후: 후면 기준 허용 오차 체크] ------------------------------------------------
  // 타겟이 내 뒤(PI 방향)에 있는지 확인
  if (fabs(angles::shortest_angular_distance(M_PI, angle_to_target)) < params_->initial_rotation_tolerance) {
  // --------------------------------------------------------------------------------
    sim_initial_rotation = false;
    do_initial_rotation_ = false;
  }
  
  // ... (중략) ...

  // Generate path
  do{
    if (sim_initial_rotation) {
      // Compute rotation velocity
      double next_pose_yaw = tf2::getYaw(next_pose.pose.orientation);
      
      // [수정 전]
      // auto cmd = rotateToTarget(angle_to_target - next_pose_yaw);
      
      // [수정 후] rotateToTarget 함수 내부를 수정했으므로, 
      // 여기서는 상대 각도를 그대로 넘겨주되, 시뮬레이션 상의 로봇 Yaw를 고려해야 합니다.
      // angle_to_target(Global/Odom frame concept in simulation) - next_pose_yaw
      // 사실 simulateTrajectory는 base_link 기준(0,0)에서 시작하므로 
      // angle_to_target 자체가 상대 각도입니다.
      // 하지만 next_pose가 회전하면서 각도가 변하므로 아래와 같이 처리합니다.
      
      double current_relative_angle = angles::shortest_angular_distance(next_pose_yaw, angle_to_target);
      auto cmd = rotateToTarget(current_relative_angle); 

      // ...

```

#### 4. `computeVelocityCommands` - Phase 3 (Final Rotation) 수정

커스텀된 Phase 3 로직에서 최종 목표 자세에 도달하기 위해 회전할 때, **후진 주차**처럼 로봇의 뒤가 목표 방향과 일치하도록 해야 합니다.

**[수정 위치: `graceful_controller.cpp` 약 145라인 부근 Phase 3 로직]**

```cpp
  // =================================================================================
  // [Phase 3] 제자리 회전 모드 (Rotation Logic)
  // =================================================================================
  if (enter_rotation_mode) {
    goal_reached_ = true;

    // [중요 수정] angle_to_goal 의미 재정의
    // 기존 angle_to_goal은 transformed_plan의 마지막 pose의 Yaw였습니다.
    // 후진 주행 시: (Goal Yaw)와 (Robot Yaw + 180도)가 일치해야 함.
    // 따라서 오차 = Goal Yaw - (Robot Yaw + PI)
    // 로봇 Frame에서 Goal Yaw를 본다면, Goal Yaw 자체가 상대 각도이므로
    // 오차 = Goal Relative Yaw - PI 가 됩니다.
    
    // transformed_plan이 보통 Global Frame에서 변환된 Local Frame(base_link)이라고 가정하면,
    // angle_to_goal은 로봇 기준 목표의 상대 Yaw입니다.
    // 우리는 뒤로 정렬하고 싶으므로 PI를 빼줍니다.
    
    double error_to_goal_orientation = angles::shortest_angular_distance(M_PI, angle_to_goal);

    // [Inner Tolerance Check] 
    if (std::abs(error_to_goal_orientation) < 0.01) { // angle_to_goal -> error_to_goal_orientation
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        return cmd_vel; 
    }

    // [Stability Speed Calculation]
    // angle_to_goal -> error_to_goal_orientation 로 변경
    double target_vel = params_->rotation_scaling_factor * error_to_goal_orientation * params_->v_angular_max;

    // ... (Clamping 로직은 그대로 유지) ...
    
    // ... (Collision Check 로직 수정 필요) ...
    // Collision Check에서도 로봇이 회전할 때 후면 정렬을 목표로 하므로 
    // angle_to_goal 대신 error_to_goal_orientation을 사용하여 step을 나눠야 합니다.
    size_t num_steps = fabs(error_to_goal_orientation) / params_->in_place_collision_resolution;
    
    // ... loop 내부 yaw 계산 ...
    // double yaw = step * angle_to_goal; -> double yaw = step * error_to_goal_orientation;

```

### 요약 및 주의사항

1. **좌표계 해석:** 위 수정 사항은 `simulateTrajectory`와 `rotateToTarget`이 **Robot Base Frame (Local)** 에서 작동한다고 가정했습니다 (Nav2 표준).
2. **Phase 3의 `angle_to_goal`:** 사용자 코드에서 `angle_to_goal`을 `transformed_plan`의 마지막 pose에서 가져오는데, 이것이 `base_link` 기준이라면 위 수정(4번)이 맞습니다. 만약 `odom` 기준이라면 `tf2::getYaw(pose.pose.orientation)` (현재 로봇 자세)와의 차이를 계산해야 합니다. `GracefulController`는 보통 `transformGlobalPlan`을 통해 Local path를 얻으므로 `base_link` 기준일 확률이 높습니다.
3. **Control Law 내부:** `SmoothControlLaw` 내부에서 `backward` 플래그를 받으면 선속도를 음수로 바꾸고 각속도 제어 법칙()을 뒤집습니다. 1번 수정사항에서 `reversing = true`와 `Orientation + PI`를 동시에 적용했으므로, Control Law는 "내 뒤에 있는 타겟을 향해 뒤로 가라"는 명령을 "내 앞에 있는 타겟(가상)을 향해 앞으로 가는 수학적 모델의 역방향"으로 올바르게 해석할 것입니다.

이 수정을 적용하면 로봇은 모든 경로를 후진으로 주행하며, 제자리 회전 시에도 엉덩이(후면)를 목표 방향으로 먼저 돌리게 됩니다.
