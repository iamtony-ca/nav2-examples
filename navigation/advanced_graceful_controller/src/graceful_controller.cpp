// Copyright (c) 2023 Alberto J. Tudela Roldán
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <mutex>

#include "angles/angles.h"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "advanced_graceful_controller/graceful_controller.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_stable_stopped_goal_checker/stable_stopped_goal_checker.hpp"



namespace advanced_graceful_controller
{

void GracefulController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::ControllerException("Unable to lock node!");
  }

// [추가] 시간 측정을 위해 clock 저장
  clock_ = node->get_clock();

  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();

  // Handles storage and dynamic configuration of parameters.
  // Returns pointer to data current param settings.
  param_handler_ = std::make_unique<ParameterHandler>(
    node, plugin_name_, logger_,
    costmap_ros_->getCostmap()->getSizeInMetersX());
  params_ = param_handler_->getParams();

  // Handles global path transformations
  path_handler_ = std::make_unique<PathHandler>(
    tf2::durationFromSec(params_->transform_tolerance), tf_buffer_, costmap_ros_);

  // Handles the control law to generate the velocity commands
  control_law_ = std::make_unique<SmoothControlLaw>(
    params_->k_phi, params_->k_delta, params_->beta, params_->lambda, params_->slowdown_radius,
    params_->v_linear_min, params_->v_linear_max, params_->v_angular_max);

  // Initialize footprint collision checker
  collision_checker_ = std::make_unique<nav2_costmap_2d::
      FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_ros_->getCostmap());

  // Publishers
  transformed_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("transformed_global_plan", 1);
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  motion_target_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("motion_target", 1);
  slowdown_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("slowdown", 1);

  remaining_goals_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    "/remaining_goals", rclcpp::QoS(10),
    std::bind(&GracefulController::remainingGoalsCallback, this, std::placeholders::_1));
  
  RCLCPP_INFO(logger_, "Configured Graceful Motion Controller: %s", plugin_name_.c_str());
}


void GracefulController::remainingGoalsCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  remaining_goals_count_.store(static_cast<int>(msg->poses.size()));  // ⚠️ 접근자 타입 맞춤
}

void GracefulController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type graceful_controller::GracefulController",
    plugin_name_.c_str());
  transformed_plan_pub_.reset();
  local_plan_pub_.reset();
  motion_target_pub_.reset();
  slowdown_pub_.reset();
  collision_checker_.reset();
  path_handler_.reset();
  param_handler_.reset();
  control_law_.reset();
}

void GracefulController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type advanced_graceful_controller::GracefulController",
    plugin_name_.c_str());
  transformed_plan_pub_->on_activate();
  local_plan_pub_->on_activate();
  motion_target_pub_->on_activate();
  slowdown_pub_->on_activate();

// [추가] 활성화 시점의 시간을 저장 및 초기화
  last_control_time_ = clock_->now();  
  last_cmd_vel_ = geometry_msgs::msg::Twist(); // 0으로 초기화
}


void GracefulController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type advanced_graceful_controller::GracefulController",
    plugin_name_.c_str());
  transformed_plan_pub_->on_deactivate();
  local_plan_pub_->on_deactivate();
  motion_target_pub_->on_deactivate();
  slowdown_pub_->on_deactivate();
}


// [추가] 가감속 제한 로직 구현
double GracefulController::applyKinematicLimits(
  double v_current, double v_target, double max_acc, double max_decel, double dt)
{
  double dv = v_target - v_current;
  
  // 가속 중인지 감속 중인지 판단 (방향이 같고, 크기가 커지거나 출발할 때)
  bool is_accelerating = (v_target * v_current > 0.0 && std::abs(v_target) > std::abs(v_current)) || v_current == 0.0;
  
// [수정 핵심] max_decel이 음수로 들어올 수 있으므로 반드시 std::abs() 적용
  double active_limit = is_accelerating ? std::abs(max_acc) : std::abs(max_decel);
  double limit_dv = active_limit * dt;
  
  // 허용 변화량을 초과하면 Clamping
  if (std::abs(dv) > limit_dv) {
    return v_current + std::copysign(limit_dv, dv);
  }
  return v_target;
}



geometry_msgs::msg::TwistStamped GracefulController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity, // <-- 로봇의 실제 현재 속도(odom)
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> param_lock(param_handler_->getMutex());

  // --- [튜닝 파라미터 모음] ---
  // 1. 제자리 회전
  double MAX_IN_PLACE_VEL = 0.35; 
  double ACC_IN_PLACE = 0.2;
  double DEC_IN_PLACE = -2.0;

  // 2. 주행 가감속
  double ACC_LINEAR = 1.0;  
  double DEC_LINEAR = -1.0; 
  double ACC_ANGULAR = 3.0;
  double DEC_ANGULAR = -3.0;

  // 3. 전방 곡률 예측 감속 (Predictive Slowdown)
  double PREDICTIVE_LOOKAHEAD_BASE = 1.0;  // 최소 스캔 거리
  double PREDICTIVE_LOOKAHEAD_SCALE = 1.5; // 속도 비례 추가 스캔 거리
  double BASE_CURVATURE_THRESHOLD = 0.45;  // 감속을 시작할 기본 곡률 임계값
  double PREDICTIVE_LINEAR_GAIN = 0.15;    // 초과 곡률당 감속 강도 (고속 시 최대 적용)

  // 4. 목적지 감속
  double MIN_SLOWDOWN_DIST = 0.5; // 최소 감속 보장 거리

  // 5. 궤적 보존 및 스무딩 (Adaptive Smoothing)
  double MAX_INTERVENTION_RADIUS_ = 0.5;    // 궤적 보존 개입을 시작할 최대 회전 반경
  double MIN_SMOOTHING_FACTOR = 0.15;      // 정지 상태일 때의 기본 스무딩 팩터 (최소 브레이크)
  double SMOOTHING_FACTOR_SCALE = 0.25;    // 최고 속도일 때 추가되는 스무딩 팩터 (고속 브레이크 강화)
  // ----------------------------



  // =========================================================================
  // [1] 루프 제어 주기(dt) 계산 및 안전장치
  // =========================================================================
  auto now = clock_->now();
  double dt_control = (now - last_control_time_).seconds();
  last_control_time_ = now;
  if (dt_control <= 0.0 || dt_control > 0.5) { dt_control = 0.05; }

  // =========================================================================
  // [2] 노이즈 강인형 단방향 가상 목줄 (Magnitude-based One-Sided Leash)
  // Safety Lidar 등 외부 요인에 의해 로봇이 강제 정지했을 때, Overshoot 방지
  // =========================================================================
  double actual_speed = velocity.linear.x;
  double actual_w = velocity.angular.z;


  // testing.... 현재 로직은 odometry 기반이라서 아래 로직은 필요 없음.
  // double max_lag_x = 0.3; 
  // double max_lag_w = 0.4; 
  // if (std::abs(last_cmd_vel_.linear.x) > std::abs(actual_speed) + max_lag_x) {
  //     last_cmd_vel_.linear.x = std::copysign(std::abs(actual_speed) + max_lag_x, last_cmd_vel_.linear.x);
  // }
  // if (std::abs(last_cmd_vel_.angular.z) > std::abs(actual_w) + max_lag_w) {
  //     last_cmd_vel_.angular.z = std::copysign(std::abs(actual_w) + max_lag_w, last_cmd_vel_.angular.z);
  // }


  geometry_msgs::msg::TwistStamped target_cmd;
  target_cmd.header = pose.header;
  target_cmd.twist.linear.x = 0.0;
  target_cmd.twist.linear.y = 0.0;
  target_cmd.twist.angular.z = 0.0;
  bool cmd_found = false; 

  // =========================================================================
  // [3] Goal Tolerance 및 글로벌 경로 계산
  // =========================================================================
  auto * custom_checker = dynamic_cast<nav2_controller::StableStoppedGoalChecker*>(goal_checker);
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, velocity_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  } else {
    goal_dist_tolerance_ = pose_tolerance.position.x;
  }

  auto transformed_plan = path_handler_->transformGlobalPlan(
    pose, params_->max_robot_pose_search_dist);

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

  double dx = pose.pose.position.x - transformed_plan.poses.back().pose.position.x;
  double dy = pose.pose.position.y - transformed_plan.poses.back().pose.position.y;
  double euclidean_dist = std::hypot(dx, dy);
  double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);



// =========================================================================
  // ★ [4] 속도 적응형 전방 매크로 곡률 예측 감속 (Linear Predictive Slowdown) ★
  // =========================================================================
  double dynamic_v_max = params_->v_linear_max;

  // 1. 현재 속도 비율 (0.0 ~ 1.0) 계산
  double speed_ratio = std::clamp(std::abs(actual_speed) / params_->v_linear_max, 0.0, 1.0);

  if (std::abs(actual_speed) > 0.25){ 
      // 2. 속도 기반 동적 스캔 거리 (빠를수록 멀리 봄)
      // double preview_distance = 1.0 + (1.5 * speed_ratio); 
      double preview_distance = PREDICTIVE_LOOKAHEAD_BASE + (PREDICTIVE_LOOKAHEAD_SCALE * speed_ratio);
      double max_macro_curvature = 0.0;
  
      if (transformed_plan.poses.size() > 1) {
          double start_yaw = tf2::getYaw(transformed_plan.poses[0].pose.orientation);
  
          for (size_t i = 1; i < transformed_plan.poses.size(); ++i) {
              double L = nav2_util::geometry_utils::euclidean_distance(
                  transformed_plan.poses[0].pose, transformed_plan.poses[i].pose);
              
              if (L > preview_distance) break;
              if (L < 0.3) continue; // 그리드 노이즈 스킵
  
              double pt_yaw = tf2::getYaw(transformed_plan.poses[i].pose.orientation);
              double yaw_diff = std::abs(angles::shortest_angular_distance(start_yaw, pt_yaw));
  
              double macro_curvature = yaw_diff / L;
              if (macro_curvature > max_macro_curvature) {
                  max_macro_curvature = macro_curvature;
              }
          }
      }

      // 3. 연속적(Linear) 곡률 감속 로직
      // 속도에 따른 동적 임계값 (빠를수록 예민하게 감지)
      double sensitivity_multiplier = 1.5 - speed_ratio; 
      double base_curvature_threshold = BASE_CURVATURE_THRESHOLD * sensitivity_multiplier;

      

      // 계산된 곡률이 임계값을 넘었을 때만 비례 제어(P-Control) 감속 실행
      if (max_macro_curvature > base_curvature_threshold) {
          
          // 임계값을 초과한 '순수 잉여 곡률량' (이 값이 클수록 커브가 심함)
          double excess_curvature = max_macro_curvature - base_curvature_threshold;
          
          //  곡률 1.0당 깎아낼 속도 비율 (기본 감속 민감도)
          // 속도(speed_ratio)를 곱해주어 빠를수록 더 강하게 브레이크를 밟게 함
          // double linear_gain = 0.15 * speed_ratio; 
          double linear_gain = PREDICTIVE_LINEAR_GAIN * speed_ratio;
          
          // 초과 곡률량에 비례하여(Linear) 부드럽게 깎아낼 속도량 계산
          double preview_slowdown = excess_curvature * linear_gain;
          dynamic_v_max = std::max(params_->v_linear_min, dynamic_v_max - preview_slowdown);
      }
  }
  
  // =========================================================================
  // ★ [5] 속도 기반 글로벌 목적지 감속 (Speed-Aware Distance Slowdown) ★
  // =========================================================================
  // 속도에 비례하여 감속 시작 반경을 동적으로 줄이거나 늘림
  double dynamic_slowdown_radius = params_->slowdown_radius * (0.5 + 0.5 * speed_ratio);

  //  아무리 느려도 골인 지점 X 미터 앞에서는 반드시 감속 시작!
  // double min_slowdown_dist = 0.5; // (예: 0.5m)
  double min_slowdown_dist = MIN_SLOWDOWN_DIST; // (예: 0.5m)
  dynamic_slowdown_radius = std::max(dynamic_slowdown_radius, min_slowdown_dist);
  
  // (안전장치) 단, 사용자가 설정한 원본 파라미터 값보다는 크지 않게 제한
  dynamic_slowdown_radius = std::min(dynamic_slowdown_radius, params_->slowdown_radius);

  // 목적지 감속 적용
  if (dist_to_goal < dynamic_slowdown_radius && dynamic_slowdown_radius > 0.0) {
      double ratio = dist_to_goal / dynamic_slowdown_radius; 
      double dist_v_max = params_->v_linear_min + 
                          (params_->v_linear_max - params_->v_linear_min) * ratio;
      dynamic_v_max = std::min(dynamic_v_max, dist_v_max);
  }

  control_law_->setCurvatureConstants(params_->k_phi, params_->k_delta, params_->beta, params_->lambda);
  control_law_->setSlowdownRadius(0.1); // 내부 지역 감속 무력화
  control_law_->setSpeedLimit(params_->v_linear_min, dynamic_v_max, params_->v_angular_max);


  // =========================================================================
  // [6] 3단계 모드 결정 (회전 / XY 안정화 / 경로 추종)
  // =========================================================================
  bool enter_rotation_mode = false;
  bool is_xy_stabilizing_phase = false;

  constexpr double PATH_DIST_MAX = 2.8;                 // goal_checker와 동일값
  const int  rgc = remaining_goals_count_.load();
  const bool is_final_goal = (rgc >= 0 && rgc <= 1);    // -1 차단

  if (custom_checker) {
    const double xy_tol = custom_checker->getXGoalTolerance();
    const bool gate_ok = is_final_goal
                       && (euclidean_dist <= xy_tol)
                       && (dist_to_goal <= PATH_DIST_MAX);
    if (custom_checker->isXYLatched()) {
        enter_rotation_mode = true;          // checker가 이미 XY 통과(=게이트 통과)했으므로 신뢰
    } else if (gate_ok) {
        is_xy_stabilizing_phase = true;
    }
    // gate_ok 거짓 → 어떤 모드도 안 켬 → 경로 추종으로 떨어짐 (루프/교차/중간 = 그냥 주행)
  } else {
    if (dist_to_goal < goal_dist_tolerance_ || goal_reached_) {   // 기존 그대로
        enter_rotation_mode = true;
    }
  }

  // [Phase 3] 제자리 회전 모드
  if (enter_rotation_mode) {
    goal_reached_ = true;
    if (std::abs(angle_to_goal) < 0.01) {
        cmd_found = true; 
    } else {
        double target_vel = params_->rotation_scaling_factor * angle_to_goal * params_->v_angular_max;
        // if (std::abs(target_vel) > 0.35) target_vel = std::copysign(0.35, target_vel);
        if (std::abs(target_vel) > MAX_IN_PLACE_VEL) target_vel = std::copysign(MAX_IN_PLACE_VEL, target_vel);
        if (std::abs(target_vel) < 0.01) target_vel = std::copysign(0.01, target_vel);

        size_t num_steps = fabs(angle_to_goal) / params_->in_place_collision_resolution;
        num_steps = std::max(static_cast<size_t>(1), num_steps);
        bool collision_free = true;
        for (size_t i = 1; i <= num_steps; ++i) {
          double step = static_cast<double>(i) / static_cast<double>(num_steps);
          double yaw = step * angle_to_goal;
          geometry_msgs::msg::PoseStamped next_pose;
          next_pose.header.frame_id = costmap_ros_->getBaseFrameID();
          next_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
          geometry_msgs::msg::PoseStamped costmap_pose;
          tf2::doTransform(next_pose, costmap_pose, costmap_transform);
          if (inCollision(
              costmap_pose.pose.position.x, costmap_pose.pose.position.y,
              tf2::getYaw(costmap_pose.pose.orientation)))
          {
            collision_free = false;
            break;
          }
        }

        if (collision_free) {
            target_cmd.twist.angular.z = target_vel;
            cmd_found = true;
        }
    } 
  } 
  // [Phase 2] XY 안정화 모드
  else if (is_xy_stabilizing_phase) {
    if (euclidean_dist < 0.007) {
        cmd_found = true;
    } else {
        // control_law_->setSpeedLimit(0.01, 0.3, 0.35); 
        control_law_->setSpeedLimit(0.01, 0.3, MAX_IN_PLACE_VEL);    
    }
  }

  // [Phase 1 & 2 Common] 경로 추종 시뮬레이션
  if (!cmd_found) { 
    std::vector<double> target_distances;
    computeDistanceAlongPath(transformed_plan.poses, target_distances);

    for (int i = transformed_plan.poses.size() - 1; i >= 0; --i) {
      geometry_msgs::msg::PoseStamped target_pose = transformed_plan.poses[i];
      double dist_to_target = target_distances[i];
  
      if (dist_to_target > params_->max_lookahead) {continue;}
  
      if (dist_to_goal < params_->max_lookahead) {
        if (params_->prefer_final_rotation) {
          double yaw = std::atan2(target_pose.pose.position.y, target_pose.pose.position.x);
          target_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
        }
      } else if (dist_to_target < params_->min_lookahead) {
        break;
      }
  
      bool reversing = false;
      if (params_->allow_backward && target_pose.pose.position.x < 0.0) {
        reversing = true;
        target_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
          tf2::getYaw(target_pose.pose.orientation) + M_PI);
      }
  
      nav_msgs::msg::Path local_plan;
      if (simulateTrajectory(target_pose, costmap_transform, local_plan, target_cmd, reversing)) {
        motion_target_pub_->publish(target_pose);
        auto slowdown_marker = advanced_graceful_controller::createSlowdownMarker(target_pose, params_->slowdown_radius);
        slowdown_pub_->publish(slowdown_marker);
        local_plan.header = transformed_plan.header;
        local_plan_pub_->publish(local_plan);
        
        cmd_found = true;
        break; 
      }
    }
  } 

  if (!cmd_found) {
    throw nav2_core::NoValidControl("Collision detected in trajectory");
  }



  // =================================================================================
  // ★ [Post-Processing] Kinematic Limits & Curvature Preservation (핵심 해결책) ★
  // =================================================================================
  geometry_msgs::msg::TwistStamped final_cmd = target_cmd;
  
  // 1. 선속도 Y 원천 차단 (linear_y 발생 문제 해결)
  final_cmd.twist.linear.y = 0.0; 

  if (std::abs(target_cmd.twist.linear.x) < 0.001) {
      // [제자리 회전 모드] 선속도가 0이므로 각속도만 단독으로 가감속 적용
      // final_cmd.twist.angular.z = applyKinematicLimits(
      //   last_cmd_vel_.angular.z, target_cmd.twist.angular.z, 0.2, -2.0, dt_control);
      final_cmd.twist.angular.z = applyKinematicLimits(
        last_cmd_vel_.angular.z, target_cmd.twist.angular.z, ACC_IN_PLACE, DEC_IN_PLACE, dt_control);
  // } else {
  //     // [경로 추종 모드] 궤적 이탈(Wobbling) 방지를 위한 곡률(Curvature) 비율 유지 로직
  //     double target_v = target_cmd.twist.linear.x;
  //     double target_w = target_cmd.twist.angular.z;
      
  //     // A. 목표 궤적의 곡률 계산 (k = w / v)
  //     double kappa = target_w / target_v; 

  //     // B. 선속도(X)에만 가감속 한계 적용
  //     // double limited_v = applyKinematicLimits(actual_speed, target_v, 1.0, -1.0, dt_control);
  //     double limited_v = applyKinematicLimits(actual_speed, target_v, ACC_LINEAR, DEC_LINEAR, dt_control);
      
  //     // C. 제한된 선속도에 곡률을 곱해 각속도(Z)를 다시 계산 (조향 비율 완벽 유지!)
  //     double limited_w = limited_v * kappa;

  //     // D. (안전장치) 다시 계산된 각속도가 물리적 한계를 초과하면, 각속도 기준으로 다시 맞춤
  //     // double max_w_limit = applyKinematicLimits(actual_w, target_w, 3.0, -3.0, dt_control);
  //     double max_w_limit = applyKinematicLimits(actual_w, target_w, ACC_ANGULAR, DEC_ANGULAR, dt_control);
      
  //     if (std::abs(limited_w) > std::abs(max_w_limit)) {
  //         limited_w = max_w_limit;

  //         double turning_radius = 100.0;
  //         if (std::abs(kappa) > 0.001) turning_radius = 1.0 / std::abs(kappa);

  //         // ==========================================================
  //         // 속도 및 반경 적응형 연속 스무딩 (Linear Adaptive Smoothing)
  //         // ==========================================================
  //         // double max_intervention_radius = 0.5; // 0.5m 이하의 코너에서만 개입 시작
  //         double max_intervention_radius = MAX_INTERVENTION_RADIUS_; // 0.5m 이하의 코너에서만 개입 시작

  //         if (turning_radius < max_intervention_radius) {
  //             // 1. 반경 기반 가중치 (Radius Severity) : 0.0 ~ 1.0
  //             // 반경이 0.5m면 0.0(개입 안함), 0m에 가까울수록 1.0(최대 개입)으로 선형 증가
  //             double radius_severity = 1.0 - (turning_radius / max_intervention_radius);

  //             // 2. 속도 기반 최대 스무딩 팩터 (Max Smoothing Factor)
  //             // 속도가 빠를수록 원심력 때문에 궤적을 벗어나기 쉬우므로, 브레이크 비중을 더 높여줌
  //             // (예: 정지 시 0.15, 최고 속도 시 0.40까지 선형 증가)
  //             // double max_smoothing_factor = 0.15 + (0.25 * speed_ratio); 
  //             double max_smoothing_factor = MIN_SMOOTHING_FACTOR + (SMOOTHING_FACTOR_SCALE * speed_ratio);

  //             // 3. 최종 동적 스무딩 팩터 산출
  //             double dynamic_smoothing_factor = radius_severity * max_smoothing_factor;

  //             // 수학적 이상적 감속값(strict_v)과 현재 속도(limited_v)를 동적 비율로 혼합
  //             double strict_v = limited_w / kappa;
  //             limited_v = (strict_v * dynamic_smoothing_factor) + (limited_v * (1.0 - dynamic_smoothing_factor));
  //         }
  //     }

  //     // 5. 비율 유지 스케일링 (Proportional Scaling)
  //     double v_scale = 1.0;
  //     double w_scale = 1.0;
    
  //     if (std::abs(limited_v) > dynamic_v_max && dynamic_v_max > 0.0) {
  //         v_scale = dynamic_v_max / std::abs(limited_v);
  //     }
  //     if (std::abs(limited_w) > params_->v_angular_max && params_->v_angular_max > 0.0) {
  //         w_scale = params_->v_angular_max / std::abs(limited_w);
  //     }
    
  //     double final_scale = std::min(v_scale, w_scale);
  //     limited_v *= final_scale;
  //     limited_w *= final_scale;


  //     // testing...
  //     // limited_v = std::clamp(limited_v, params_->v_linear_min, params_->v_linear_max);
  //     limited_v = std::clamp(limited_v, -params_->v_linear_max, params_->v_linear_max);
  //     limited_w = std::clamp(limited_w, -params_->v_angular_max, params_->v_angular_max);

  //     // testing....
  //     // 주행 중일 때 모터 스톨(웅웅거림) 방지 (정지 목표가 아닐 때만 적용)
  //     // [수정된 스톨 방지 로직 - 완전 무결함]
  //     if (std::abs(target_v) > 0.001) {
  //         if (std::abs(limited_v) < params_->v_linear_min) {
              
  //             // [수정 핵심] 부호의 기준을 limited_v가 아닌 target_v로 변경!
  //             double new_v = std::copysign(params_->v_linear_min, target_v); 
              
  //             // v가 0이었든 아니든 상관없이, 무조건 완벽한 조향 비율 유지
  //             limited_w = new_v * kappa;
  //             limited_v = new_v;

  //             limited_w = std::clamp(limited_w, -params_->v_angular_max, params_->v_angular_max);
  //         }
  //     }

  //     final_cmd.twist.linear.x = limited_v;
  //     final_cmd.twist.angular.z = limited_w;
  // }

  } else {
      // [경로 추종 모드] 궤적 보존 + 특이점 가드
      double target_v = target_cmd.twist.linear.x;
      double target_w = target_cmd.twist.angular.z;

      // ── [추가 1] 컨텍스트 각속도 상한 ──
      // 회전/안정화 의도(충돌로 회전이 막혀 여기로 흘러내린 경우 포함)면 0.35로 묶음
      double context_max_w = params_->v_angular_max;
      if (enter_rotation_mode || is_xy_stabilizing_phase) {
          context_max_w = MAX_IN_PLACE_VEL;
      }

      // ── [추가 2] kappa(=curvature) 특이점 가드 ──
      // planner min_turning_radius=0.3 → 합법 곡률 ≲3.3. 반경 0.15m(곡률 6.67) 초과는
      // -1/r 특이점 아티팩트이므로 곡률 보존을 끈다. (1.8 스파이크 원천 차단)
      constexpr double kSingularCurvature = 1.0 / 0.15;  // ≈6.67
      double kappa = (std::abs(target_v) > 1e-6) ? (target_w / target_v) : 0.0;
      const bool curvature_singular =
          (std::abs(kappa) > kSingularCurvature) || (std::abs(target_v) < 0.02);
      const bool use_curvature = !curvature_singular;

      double limited_v = applyKinematicLimits(actual_speed, target_v, ACC_LINEAR, DEC_LINEAR, dt_control);
      double limited_w;

      if (use_curvature) {
          // 곡률 보존: 제한된 선속도 × 곡률
          limited_w = limited_v * kappa;

          double max_w_limit = applyKinematicLimits(actual_w, target_w, ACC_ANGULAR, DEC_ANGULAR, dt_control);
          if (std::abs(limited_w) > std::abs(max_w_limit)) {
              limited_w = max_w_limit;
              double turning_radius = 100.0;
              if (std::abs(kappa) > 0.001) turning_radius = 1.0 / std::abs(kappa);

              double max_intervention_radius = MAX_INTERVENTION_RADIUS_;
              if (turning_radius < max_intervention_radius) {
                  double radius_severity = 1.0 - (turning_radius / max_intervention_radius);
                  double max_smoothing_factor = MIN_SMOOTHING_FACTOR + (SMOOTHING_FACTOR_SCALE * speed_ratio);
                  double dynamic_smoothing_factor = radius_severity * max_smoothing_factor;
                  double strict_v = limited_w / kappa;
                  limited_v = (strict_v * dynamic_smoothing_factor) + (limited_v * (1.0 - dynamic_smoothing_factor));
              }
          }
      } else {
          // [특이점 구간] kappa 곱셈 차단, 각속도 단독 가감속
          limited_w = applyKinematicLimits(actual_w, target_w, ACC_ANGULAR, DEC_ANGULAR, dt_control);
      }

      // 비율 유지 스케일링 (각속도 상한은 context_max_w)
      double v_scale = 1.0;
      double w_scale = 1.0;
      if (std::abs(limited_v) > dynamic_v_max && dynamic_v_max > 0.0) {
          v_scale = dynamic_v_max / std::abs(limited_v);
      }
      if (std::abs(limited_w) > context_max_w && context_max_w > 0.0) {
          w_scale = context_max_w / std::abs(limited_w);
      }
      double final_scale = std::min(v_scale, w_scale);
      limited_v *= final_scale;
      limited_w *= final_scale;

      limited_v = std::clamp(limited_v, -params_->v_linear_max, params_->v_linear_max);
      limited_w = std::clamp(limited_w, -context_max_w, context_max_w);   // [변경] context 상한

      // [버그픽스] 스톨 방지: use_curvature일 때만 곡률 보존
      if (std::abs(target_v) > 0.001 && std::abs(limited_v) < params_->v_linear_min) {
          double new_v = std::copysign(params_->v_linear_min, target_v);
          limited_v = new_v;
          if (use_curvature) {
              limited_w = new_v * kappa;          // 곡률 정상일 때만 재계산
          }
          // use_curvature==false면 위에서 단독 램핑된 limited_w 유지 (kappa=0 오염 방지)
          limited_w = std::clamp(limited_w, -context_max_w, context_max_w);
      }

      final_cmd.twist.linear.x = limited_v;
      final_cmd.twist.angular.z = limited_w;
  }

  // ── [방어] 최종 발행 직전 하드 가드 ──
  // 사실상 병진이 없는데(|v|<=0.02) 각속도가 과도하면(>0.5) 캡.
  // kappa 특이점/스파이크가 위 로직을 빠져나오는 경우의 최후 방어선.
  if (std::abs(final_cmd.twist.linear.x) <= 0.02 &&
      std::abs(final_cmd.twist.angular.z) > 0.5)
  {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
        "[guard] angular capped: v=%.3f w=%.3f -> 0.5",
        final_cmd.twist.linear.x, final_cmd.twist.angular.z);
      final_cmd.twist.angular.z = std::copysign(0.5, final_cmd.twist.angular.z);
  }

  last_cmd_vel_ = final_cmd.twist;
  return final_cmd;
  
}






void GracefulController::setPlan(const nav_msgs::msg::Path & path)
{
  path_handler_->setPlan(path);
  goal_reached_ = false;
  do_initial_rotation_ = true;
}

void GracefulController::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> param_lock(param_handler_->getMutex());

  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    params_->v_linear_max = params_->v_linear_max_initial;
    params_->v_angular_max = params_->v_angular_max_initial;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      params_->v_linear_max = std::max(
        params_->v_linear_max_initial * speed_limit / 100.0, params_->v_linear_min);
      params_->v_angular_max = params_->v_angular_max_initial * speed_limit / 100.0;
    } else {
      // Speed limit is expressed in m/s
      params_->v_linear_max = std::max(speed_limit, params_->v_linear_min);
      // Limit the angular velocity to be proportional to the linear velocity
      params_->v_angular_max = params_->v_angular_max_initial *
        speed_limit / params_->v_linear_max_initial;
    }
  }
}

bool GracefulController::simulateTrajectory(
  const geometry_msgs::msg::PoseStamped & motion_target,
  const geometry_msgs::msg::TransformStamped & costmap_transform,
  nav_msgs::msg::Path & trajectory,
  geometry_msgs::msg::TwistStamped & cmd_vel,
  bool backward)
{
  trajectory.poses.clear();

  // First pose is robot current pose
  geometry_msgs::msg::PoseStamped next_pose;
  next_pose.header.frame_id = costmap_ros_->getBaseFrameID();
  next_pose.pose.orientation.w = 1.0;

  // Should we simulate rotation initially?
  bool sim_initial_rotation = do_initial_rotation_ && params_->initial_rotation;
  double angle_to_target =
    std::atan2(motion_target.pose.position.y, motion_target.pose.position.x);
  if (fabs(angle_to_target) < params_->initial_rotation_tolerance) {
    sim_initial_rotation = false;
    do_initial_rotation_ = false;
  }

  double distance = std::numeric_limits<double>::max();
  double resolution_ = costmap_ros_->getCostmap()->getResolution();
  double dt = (params_->v_linear_max > 0.0) ? resolution_ / params_->v_linear_max : 0.0;

  // Set max iter to avoid infinite loop
  unsigned int max_iter = 3 *
    std::hypot(motion_target.pose.position.x, motion_target.pose.position.y) / resolution_;

  // Generate path
// Generate path
  do{
    if (sim_initial_rotation) {
      // Compute rotation velocity
      double next_pose_yaw = tf2::getYaw(next_pose.pose.orientation);
      auto cmd = rotateToTarget(angle_to_target - next_pose_yaw);

      // If this is first iteration, this is our current target velocity
      if (trajectory.poses.empty()) {
        cmd_vel.twist = cmd; // <--- 원래대로 cmd를 사용하도록 복구!
      }

      // Are we done simulating initial rotation?
      if (fabs(angle_to_target - next_pose_yaw) < params_->initial_rotation_tolerance) {
        sim_initial_rotation = false;
      }

      // Forward simulate rotation command
      next_pose_yaw += cmd_vel.twist.angular.z * dt;
      next_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(next_pose_yaw);
    } else {
      // If this is first iteration, this is our current target velocity
      if (trajectory.poses.empty()) {
        cmd_vel.twist = control_law_->calculateRegularVelocity(
          motion_target.pose, next_pose.pose, backward);
      }

      // Apply velocities to calculate next pose
      next_pose.pose = control_law_->calculateNextPose(
        dt, motion_target.pose, next_pose.pose, backward);
    }

    // Add the pose to the trajectory for visualization
    trajectory.poses.push_back(next_pose);

    // Check for collision
    geometry_msgs::msg::PoseStamped global_pose;
    tf2::doTransform(next_pose, global_pose, costmap_transform);
    if (inCollision(
        global_pose.pose.position.x, global_pose.pose.position.y,
        tf2::getYaw(global_pose.pose.orientation)))
    {
      return false;
    }

    // Check if we reach the goal
    distance = nav2_util::geometry_utils::euclidean_distance(motion_target.pose, next_pose.pose);
  } while(distance > resolution_ && trajectory.poses.size() < max_iter);

  return true;
}

geometry_msgs::msg::Twist GracefulController::rotateToTarget(double angle_to_target)
{
  geometry_msgs::msg::Twist vel;
  vel.linear.x = 0.0;
  vel.angular.z = params_->rotation_scaling_factor * angle_to_target * params_->v_angular_max;
  vel.angular.z = std::copysign(1.0, vel.angular.z) * std::max(abs(vel.angular.z),
      params_->v_angular_min_in_place);
  return vel;
}

bool GracefulController::inCollision(const double & x, const double & y, const double & theta)
{
  unsigned int mx, my;
  if (!costmap_ros_->getCostmap()->worldToMap(x, y, mx, my)) {
    RCLCPP_WARN(
      logger_, "The path is not in the costmap. Cannot check for collisions. "
      "Proceed at your own risk, slow the robot, or increase your costmap size.");
    return false;
  }

  // Calculate the cost of the footprint at the robot's current position depending
  // on the shape of the footprint
  bool is_tracking_unknown =
    costmap_ros_->getLayeredCostmap()->isTrackingUnknown();
  bool consider_footprint = !costmap_ros_->getUseRadius();

  double footprint_cost;
  if (consider_footprint) {
    footprint_cost = collision_checker_->footprintCostAtPose(
      x, y, theta, costmap_ros_->getRobotFootprint());
  } else {
    footprint_cost = collision_checker_->pointCost(mx, my);
  }

  switch (static_cast<unsigned char>(footprint_cost)) {
    case (nav2_costmap_2d::LETHAL_OBSTACLE):
      return true;
    case (nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE):
      return consider_footprint ? false : true;
    case (nav2_costmap_2d::NO_INFORMATION):
      return is_tracking_unknown ? false : true;
  }

  return false;
}

void GracefulController::computeDistanceAlongPath(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses,
  std::vector<double> & distances)
{
  distances.resize(poses.size());
  // Do the first pose from robot
  double d = std::hypot(poses[0].pose.position.x, poses[0].pose.position.y);
  distances[0] = d;
  // Compute remaining poses
  for (size_t i = 1; i < poses.size(); ++i) {
    d += nav2_util::geometry_utils::euclidean_distance(poses[i - 1].pose, poses[i].pose);
    distances[i] = d;
  }
}

void GracefulController::validateOrientations(
  std::vector<geometry_msgs::msg::PoseStamped> & path)
{
  // We never change the orientation of the first & last pose
  // So we need at least three poses to do anything here
  if (path.size() < 3) {return;}

  // Check if we actually need to add orientations
  double initial_yaw = tf2::getYaw(path[1].pose.orientation);
  for (size_t i = 2; i < path.size() - 1; ++i) {
    double this_yaw = tf2::getYaw(path[i].pose.orientation);
    if (angles::shortest_angular_distance(this_yaw, initial_yaw) > 1e-6) {return;}
  }

  // For each pose, point at the next one
  // NOTE: control loop will handle reversing logic
  for (size_t i = 0; i < path.size() - 1; ++i) {
    // Get relative yaw angle
    double dx = path[i + 1].pose.position.x - path[i].pose.position.x;
    double dy = path[i + 1].pose.position.y - path[i].pose.position.y;
    double yaw = std::atan2(dy, dx);
    path[i].pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
  }
}

}  // namespace advanced_graceful_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  advanced_graceful_controller::GracefulController,
  nav2_core::Controller)
