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
#include "regulated_graceful_controller/graceful_controller.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_stable_stopped_goal_checker/stable_stopped_goal_checker.hpp"



namespace regulated_graceful_controller
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

  RCLCPP_INFO(logger_, "Configured Graceful Motion Controller: %s", plugin_name_.c_str());
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
    "Activating controller: %s of type regulated_graceful_controller::GracefulController",
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
    "Deactivating controller: %s of type regulated_graceful_controller::GracefulController",
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
  // ★ [4] 노이즈 강인형 전방 매크로 곡률 예측 감속 (Predictive Slowdown) ★
  // =========================================================================
  double dynamic_v_max = params_->v_linear_max;
  if (actual_speed > 3.0) {
    double preview_distance = 2.0; // 전방 1.5m 스캔
    double max_macro_curvature = 0.0;
  
    if (transformed_plan.poses.size() > 1) {
        double start_yaw = tf2::getYaw(transformed_plan.poses[0].pose.orientation);
  
        for (size_t i = 1; i < transformed_plan.poses.size(); ++i) {
            double L = nav2_util::geometry_utils::euclidean_distance(
                transformed_plan.poses[0].pose, transformed_plan.poses[i].pose);
            
            if (L > preview_distance) break;
            // [핵심] 0.3m 이내의 점들은 그리드 노이즈 방지를 위해 스킵!
            if (L < 0.3) continue; 
  
            double pt_yaw = tf2::getYaw(transformed_plan.poses[i].pose.orientation);
            double yaw_diff = std::abs(angles::shortest_angular_distance(start_yaw, pt_yaw));
  
            double macro_curvature = yaw_diff / L;
            if (macro_curvature > max_macro_curvature) {
                max_macro_curvature = macro_curvature;
            }
        }
    }

    // 예측된 커브가 심하면 목표 최고 속도 상한을 미리 낮춤
    double curvature_threshold1 = 1.35; // 약 1.0m 앞 타겟이 25도 이상 틀어져 있을 때 개입
    double curvature_threshold2 = 0.9; // 약 1.0m 앞 타겟이 25도 이상 틀어져 있을 때 개입
    double curvature_threshold3 = 0.45; // 약 1.0m 앞 타겟이 25도 이상 틀어져 있을 때 개입
  
    if (max_macro_curvature > curvature_threshold1) {
        double preview_slowdown = (max_macro_curvature - curvature_threshold1) * 0.1; // 0.1는 감속 강도 Gain
        dynamic_v_max = std::max(params_->v_linear_min, dynamic_v_max - preview_slowdown);
    }
    else if (max_macro_curvature > curvature_threshold2) {
        double preview_slowdown = (max_macro_curvature - curvature_threshold2) * 0.05; // 0.4는 감속 강도 Gain
        dynamic_v_max = std::max(params_->v_linear_min, dynamic_v_max - preview_slowdown);
    }
    else if (max_macro_curvature > curvature_threshold3) {
        double preview_slowdown = (max_macro_curvature - curvature_threshold3) * 0.02; // 0.4는 감속 강도 Gain
        dynamic_v_max = std::max(params_->v_linear_min, dynamic_v_max - preview_slowdown);
    }
  }
  
  // =========================================================================
  // [5] 글로벌 목적지 감속 (Distance Slowdown) 및 컨트롤러 주입
  // =========================================================================
  if (dist_to_goal < params_->slowdown_radius && params_->slowdown_radius > 0.0) {
      double ratio = dist_to_goal / params_->slowdown_radius; 
      double dist_v_max = params_->v_linear_min + 
                          (params_->v_linear_max - params_->v_linear_min) * ratio;
      // 곡률 예측 감속과 목적지 감속 중, 더 빡센(안전한) 속도 제한을 최종 적용
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

  if (custom_checker) {
    if (custom_checker->isXYLatched()) {
        enter_rotation_mode = true;
    } else if (euclidean_dist <= custom_checker->getXGoalTolerance()) {
        is_xy_stabilizing_phase = true;
    }
  } else {
    if (dist_to_goal < goal_dist_tolerance_ || goal_reached_) {
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
        if (std::abs(target_vel) > 0.35) target_vel = std::copysign(0.35, target_vel);
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
        control_law_->setSpeedLimit(0.01, 0.3, 0.35);     
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
        auto slowdown_marker = regulated_graceful_controller::createSlowdownMarker(target_pose, params_->slowdown_radius);
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
      final_cmd.twist.angular.z = applyKinematicLimits(
          last_cmd_vel_.angular.z, target_cmd.twist.angular.z, 0.2, -2.0, dt_control);
  } else {
      // [경로 추종 모드] 궤적 이탈(Wobbling) 방지를 위한 곡률(Curvature) 비율 유지 로직
      double target_v = target_cmd.twist.linear.x;
      double target_w = target_cmd.twist.angular.z;
      
      // A. 목표 궤적의 곡률 계산 (k = w / v)
      double kappa = target_w / target_v; 

      // B. 선속도(X)에만 가감속 한계 적용
      double limited_v = applyKinematicLimits(
          actual_speed, target_v, 1.0, -1.0, dt_control);
      
      // C. 제한된 선속도에 곡률을 곱해 각속도(Z)를 다시 계산 (조향 비율 완벽 유지!)
      double limited_w = limited_v * kappa;

      // D. (안전장치) 다시 계산된 각속도가 물리적 한계를 초과하면, 각속도 기준으로 다시 맞춤
      double max_w_limit = applyKinematicLimits(
          actual_w, target_w, 3.0, -3.0, dt_control);
      
      if (std::abs(limited_w) > std::abs(max_w_limit)) {
          limited_w = max_w_limit;

          double turning_radius = 100.0;
          if (std::abs(kappa) > 0.001) turning_radius = 1.0 / std::abs(kappa);

          // 🎛️ [튜닝 포인트] 좁은 코너에서만 비율 보존 감속 (0.4m 반경 이하)
          double min_radius1 = 0.2;         
          double smoothing_factor1 = 0.25;   
          double min_radius2 = 0.3;         
          double smoothing_factor2 = 0.1;
          double min_radius3 = 0.4;         
          double smoothing_factor3 = 0.05;
        
          if (turning_radius < min_radius1) {
              double strict_v = limited_w / kappa;
              limited_v = (strict_v * smoothing_factor1) + (limited_v * (1.0 - smoothing_factor1));
          }
          else if (turning_radius < min_radius2) {
              double strict_v = limited_w / kappa;
              limited_v = (strict_v * smoothing_factor2) + (limited_v * (1.0 - smoothing_factor2));
          }              
          else if (turning_radius < min_radius3) {
              double strict_v = limited_w / kappa;
              limited_v = (strict_v * smoothing_factor3) + (limited_v * (1.0 - smoothing_factor3));
          }                   
        
      }

      // 5. 비율 유지 스케일링 (Proportional Scaling)
      double v_scale = 1.0;
      double w_scale = 1.0;
    
      if (std::abs(limited_v) > dynamic_v_max && dynamic_v_max > 0.0) {
          v_scale = dynamic_v_max / std::abs(limited_v);
      }
      if (std::abs(limited_w) > params_->v_angular_max && params_->v_angular_max > 0.0) {
          w_scale = params_->v_angular_max / std::abs(limited_w);
      }
    
      double final_scale = std::min(v_scale, w_scale);
      limited_v *= final_scale;
      limited_w *= final_scale;

      limited_v = std::clamp(limited_v, params_->v_linear_min, params_->v_linear_max);
      limited_w = std::clamp(limited_w, -params_->v_angular_max, params_->v_angular_max);

      final_cmd.twist.linear.x = limited_v;
      final_cmd.twist.angular.z = limited_w;
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

}  // namespace regulated_graceful_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  regulated_graceful_controller::GracefulController,
  nav2_core::Controller)
