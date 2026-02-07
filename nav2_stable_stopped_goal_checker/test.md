제자리에서 회전만 한다는 것은 **컨트롤러는 "목표 위치(X, Y)에 도달했다"고 판단**하여 위치 제어를 멈췄지만, **Goal Checker가 "아직 도착 안 했다(Success False)"고 판정**하고 있기 때문입니다.

이 현상의 가장 유력한 원인은 **"좌표계(TF) 불일치"**입니다.

### 🚨 결정적인 원인: 좌표계(Frame)가 서로 다릅니다!

1. **`query_pose`**: `ControllerServer`가 Goal Checker에게 넘겨주는 로봇의 위치는 **`odom` 프레임** 기준입니다. (Local Costmap 사용 시)
2. **`current_path_`**: `/plan` 토픽을 통해 들어오는 Global Path는 보통 **`map` 프레임** 기준입니다.
3. **문제 발생:**
* 로봇이 `map` 좌표계의 (10, 10)에 있고, `odom` 좌표계로는 (5, 5)라고 가정해 봅시다. (Map-Odom 간의 Drift 발생 시)
* 기존 코드는 좌표 변환 없이 **`(10, 10)`과 `(5, 5)`의 거리를 계산**해버립니다.
* 결과적으로 거리가 매우 멀다고 판단하여, 물리적으로는 도착했지만 **"경로상 멀리 떨어져 있음"**으로 인식해 `xy_ok = false`를 리턴합니다.
* 따라서 로봇은 계속 제자리에서 회전하며 Goal Checker의 승인을 기다리는 것입니다.



---

### 🛠️ 해결책: TF 변환 추가

`query_pose`를 `current_path_`의 좌표계(Frame)로 변환한 뒤에 거리를 계산해야 합니다.

아래는 **TF 변환 로직**과 **Best Match 전략**이 모두 포함된 **완벽한 수정 코드**입니다.
(`tf2_ros` 버퍼를 사용하기 위해 헤더와 `initialize` 함수도 수정이 필요합니다.)

### 1. Header File 수정 (`stable_stopped_goal_checker.hpp`)

`tf2_ros` 관련 헤더와 멤버 변수를 추가해야 합니다.

```cpp
#ifndef NAV2_CONTROLLER__PLUGINS__STABLE_STOPPED_GOAL_CHECKER_HPP_
#define NAV2_CONTROLLER__PLUGINS__STABLE_STOPPED_GOAL_CHECKER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/goal_checker.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "nav_msgs/msg/path.hpp"
// [추가] TF 관련 헤더
#include "tf2_ros/buffer.h" 
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_controller
{

class StableStoppedGoalChecker : public nav2_core::GoalChecker
{
public:
  StableStoppedGoalChecker();
  
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
    
  void reset() override;
  
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;
    
  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;

protected:
  // ... (기존 변수들) ...
  double x_goal_tolerance_, y_goal_tolerance_, yaw_goal_tolerance_;
  double rot_stopped_velocity_, trans_stopped_velocity_;
  double xy_stability_duration_, yaw_stability_duration_;
  bool stateful_;
  double path_tolerance_multiplier_;
  std::string path_topic_;

  bool check_xy_;
  bool in_xy_tolerance_;
  bool in_yaw_tolerance_;
  rclcpp::Time first_xy_tolerance_time_;
  rclcpp::Time first_yaw_tolerance_time_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  nav_msgs::msg::Path::SharedPtr current_path_;
  std::mutex path_mutex_;

  // [추가] TF Buffer 및 Costmap Frame 정보
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string costmap_frame_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("StableStoppedGoalChecker")}; // 로깅용

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};

}  // namespace nav2_controller

#endif 

```

---

### 2. Source File 수정 (`stable_stopped_goal_checker.cpp`)

`initialize`에서 TF 버퍼를 가져오고, `isGoalReached`에서 좌표 변환을 수행하도록 수정했습니다.

```cpp
/*
 * Copyright (c) 2024, Custom Robotics
 */

#include <cmath>
#include <string>
#include <memory>
#include <limits>
#include <vector>
#include <algorithm> // max, min

#include "nav2_stable_stopped_goal_checker/stable_stopped_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "angles/angles.h"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

// [중요] TF Transform 유틸리티
#include "nav2_util/robot_utils.hpp" 

using std::hypot;
using std::fabs;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{

StableStoppedGoalChecker::StableStoppedGoalChecker()
: x_goal_tolerance_(0.25),
  y_goal_tolerance_(0.25),
  yaw_goal_tolerance_(0.25),
  rot_stopped_velocity_(0.25),
  trans_stopped_velocity_(0.25),
  xy_stability_duration_(0.5),
  yaw_stability_duration_(0.5),
  stateful_(true),
  path_tolerance_multiplier_(1.5),
  path_topic_("/plan"),
  check_xy_(true),
  in_xy_tolerance_(false),
  in_yaw_tolerance_(false)
{
}

void StableStoppedGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  // [중요] Costmap으로부터 TF Buffer와 Global Frame 이름 가져오기
  tf_ = costmap_ros->getTfBuffer();
  costmap_frame_ = costmap_ros->getGlobalFrameID(); // 보통 "odom"

  // 1. Declare Standard Parameters
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".x_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".y_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".trans_stopped_velocity", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".rot_stopped_velocity", rclcpp::ParameterValue(0.25));
  
  // 2. Declare Stability Parameters
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".xy_stability_duration", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".yaw_stability_duration", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".stateful", rclcpp::ParameterValue(true));

  // 3. Declare Path Check Parameters
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".path_tolerance_multiplier", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".path_topic", rclcpp::ParameterValue("/plan"));

  // 4. Get Parameters
  node->get_parameter(plugin_name + ".x_goal_tolerance", x_goal_tolerance_);
  node->get_parameter(plugin_name + ".y_goal_tolerance", y_goal_tolerance_);
  node->get_parameter(plugin_name + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node->get_parameter(plugin_name + ".trans_stopped_velocity", trans_stopped_velocity_);
  node->get_parameter(plugin_name + ".rot_stopped_velocity", rot_stopped_velocity_);
  node->get_parameter(plugin_name + ".xy_stability_duration", xy_stability_duration_);
  node->get_parameter(plugin_name + ".yaw_stability_duration", yaw_stability_duration_);
  node->get_parameter(plugin_name + ".stateful", stateful_);
  node->get_parameter(plugin_name + ".path_tolerance_multiplier", path_tolerance_multiplier_);
  node->get_parameter(plugin_name + ".path_topic", path_topic_);

  // 5. Initialize Path Subscription
  rclcpp::QoS qos(1);
  qos.transient_local();
  path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    path_topic_, qos,
    std::bind(&StableStoppedGoalChecker::pathCallback, this, _1));

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&StableStoppedGoalChecker::dynamicParametersCallback, this, _1));
}

void StableStoppedGoalChecker::reset()
{
  check_xy_ = true;
  in_xy_tolerance_ = false;
  in_yaw_tolerance_ = false;
}

void StableStoppedGoalChecker::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  current_path_ = msg;
}

bool StableStoppedGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & velocity)
{
  // 1. Calculate Errors (Standard Euclidean & Yaw Check)
  double dx = fabs(query_pose.position.x - goal_pose.position.x);
  double dy = fabs(query_pose.position.y - goal_pose.position.y);
  double dyaw = fabs(angles::shortest_angular_distance(
    tf2::getYaw(query_pose.orientation),
    tf2::getYaw(goal_pose.orientation)));

  bool xy_ok = (dx <= x_goal_tolerance_) && (dy <= y_goal_tolerance_);
  bool yaw_ok = (dyaw <= yaw_goal_tolerance_);

  // ---------------------------------------------------------
  // [FIXED] Global Path Distance Check Logic with TF
  // ---------------------------------------------------------
  if (xy_ok) {
    std::lock_guard<std::mutex> lock(path_mutex_);
    
    if (current_path_ && !current_path_->poses.empty()) {
      
      // [TF Transformation] query_pose(Odom)를 Path Frame(Map)으로 변환
      geometry_msgs::msg::PoseStamped query_pose_stamped;
      query_pose_stamped.pose = query_pose;
      query_pose_stamped.header.frame_id = costmap_frame_; // e.g. "odom"
      query_pose_stamped.header.stamp = clock_->now(); 

      geometry_msgs::msg::PoseStamped transformed_query_pose;
      bool transform_success = false;

      try {
        // Path의 Frame ID로 변환 (e.g. "map")
        if (nav2_util::transformPoseInTargetFrame(
            query_pose_stamped, transformed_query_pose, *tf_, 
            current_path_->header.frame_id, 1.0)) 
        {
          transform_success = true;
        } else {
          RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, 
            "Failed to transform robot pose to path frame. Skipping path check.");
        }
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, 
          "TF Exception in GoalChecker: %s", ex.what());
      }

      // 변환에 성공했을 때만 거리 체크 수행 (실패하면 기존 xy_ok 유지)
      if (transform_success) {
        double max_tolerance = std::max(x_goal_tolerance_, y_goal_tolerance_);
        double dist_threshold = max_tolerance * path_tolerance_multiplier_;
        
        // 검색 범위: 넉넉하게 잡음 (1.5배)
        double search_limit_sq = pow(max_tolerance * 1.5, 2);

        bool goal_confirmed_by_path = false;

        // [Best Match Strategy]
        // 변환된 좌표(transformed_query_pose)를 사용하여 거리 비교
        for (size_t i = 0; i < current_path_->poses.size(); ++i) {
          
          double p_dx = current_path_->poses[i].pose.position.x - transformed_query_pose.pose.position.x;
          double p_dy = current_path_->poses[i].pose.position.y - transformed_query_pose.pose.position.y;
          double dist_sq = p_dx * p_dx + p_dy * p_dy;

          if (dist_sq <= search_limit_sq) {
            // 후보군 발견. 남은 경로 길이 계산 (Early Exit)
            double accumulated_dist = 0.0;
            bool is_short_path = true;

            for (size_t j = i; j < current_path_->poses.size() - 1; ++j) {
              accumulated_dist += nav2_util::geometry_utils::euclidean_distance(
                current_path_->poses[j], current_path_->poses[j+1]);
              
              if (accumulated_dist > dist_threshold) {
                is_short_path = false;
                break; 
              }
            }

            if (is_short_path) {
              goal_confirmed_by_path = true;
              break; // Success!
            }
          }
        }

        if (!goal_confirmed_by_path) {
          xy_ok = false; // 물리적으론 가깝지만, 경로상으론 아직 멉니다.
        }
      }
    }
  }
  // ---------------------------------------------------------

  // 2. Logic based on 'stateful' parameter
  if (stateful_) {
    // === STATEFUL MODE ===
    if (check_xy_) {
      if (xy_ok) {
        if (!in_xy_tolerance_) {
          first_xy_tolerance_time_ = clock_->now();
          in_xy_tolerance_ = true;
        }
        
        double time_in_xy = (clock_->now() - first_xy_tolerance_time_).seconds();
        
        if (time_in_xy >= xy_stability_duration_) {
          check_xy_ = false;
          in_xy_tolerance_ = false; 
          in_yaw_tolerance_ = false;
        }
      } else {
        in_xy_tolerance_ = false;
      }
      return false; 
    } else {
      // Checking Yaw
      if (yaw_ok) {
        if (!in_yaw_tolerance_) {
          first_yaw_tolerance_time_ = clock_->now();
          in_yaw_tolerance_ = true;
        }

        double time_in_yaw = (clock_->now() - first_yaw_tolerance_time_).seconds();

        if (time_in_yaw >= yaw_stability_duration_) {
          return fabs(velocity.angular.z) <= rot_stopped_velocity_ &&
                 hypot(velocity.linear.x, velocity.linear.y) <= trans_stopped_velocity_;
        }
      } else {
        in_yaw_tolerance_ = false;
      }
      return false;
    }

  } else {
    // === NON-STATEFUL MODE ===
    if (xy_ok && yaw_ok) {
      if (!in_xy_tolerance_) {
        first_xy_tolerance_time_ = clock_->now();
        in_xy_tolerance_ = true;
      }

      double time_in_combined = (clock_->now() - first_xy_tolerance_time_).seconds();

      if (time_in_combined >= xy_stability_duration_) {
        return fabs(velocity.angular.z) <= rot_stopped_velocity_ &&
               hypot(velocity.linear.x, velocity.linear.y) <= trans_stopped_velocity_;
      }
    } else {
      in_xy_tolerance_ = false;
    }
    return false;
  }
}

// ... (나머지 getTolerances 등은 기존과 동일) ...
bool StableStoppedGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  double invalid_field = std::numeric_limits<double>::lowest();

  pose_tolerance.position.x = x_goal_tolerance_;
  pose_tolerance.position.y = y_goal_tolerance_;
  pose_tolerance.position.z = invalid_field;
  pose_tolerance.orientation =
    nav2_util::geometry_utils::orientationAroundZAxis(yaw_goal_tolerance_);

  vel_tolerance.linear.x = trans_stopped_velocity_;
  vel_tolerance.linear.y = trans_stopped_velocity_;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = rot_stopped_velocity_;

  return true;
}

rcl_interfaces::msg::SetParametersResult
StableStoppedGoalChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".x_goal_tolerance") {
        x_goal_tolerance_ = parameter.as_double();
      } else if (name == plugin_name_ + ".y_goal_tolerance") {
        y_goal_tolerance_ = parameter.as_double();
      } else if (name == plugin_name_ + ".yaw_goal_tolerance") {
        yaw_goal_tolerance_ = parameter.as_double();
      } else if (name == plugin_name_ + ".trans_stopped_velocity") {
        trans_stopped_velocity_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rot_stopped_velocity") {
        rot_stopped_velocity_ = parameter.as_double();
      } else if (name == plugin_name_ + ".xy_stability_duration") {
        xy_stability_duration_ = parameter.as_double();
      } else if (name == plugin_name_ + ".yaw_stability_duration") {
        yaw_stability_duration_ = parameter.as_double();
      } else if (name == plugin_name_ + ".path_tolerance_multiplier") {
        path_tolerance_multiplier_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".stateful") {
        stateful_ = parameter.as_bool();
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == plugin_name_ + ".path_topic") {
        path_topic_ = parameter.as_string();
      }
    }
  }
  result.successful = true;
  return result;
}

} 

```