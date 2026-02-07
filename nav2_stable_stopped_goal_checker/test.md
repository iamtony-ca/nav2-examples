요청하신 내용을 반영하여 **최소한의 수정**으로 기능을 구현했습니다.

주요 변경 사항은 다음과 같습니다.

1. **`/plan` 토픽 구독 추가**: Global Path 정보를 받아오기 위해 `nav_msgs/msg/Path`를 구독합니다.
2. **`pathCallback` 추가**: 수신된 경로를 저장합니다 (Thread-safety를 위해 Mutex 사용).
3. **`isGoalReached` 수정**: 기존 로직 수행 **전**에 경로의 길이를 계산하여 `2 * x_goal_tolerance`보다 길다면 바로 `false`를 리턴하도록 했습니다.

아래는 수정된 코드입니다.

### 1. Header File (`stable_stopped_goal_checker.hpp`)

`nav_msgs` 헤더와, 토픽 구독을 위한 변수(`path_sub_`, `current_path_`, `mutex`)가 추가되었습니다.

```cpp
/*
 * ... (License 주석 생략) ...
 */

#ifndef NAV2_CONTROLLER__PLUGINS__STABLE_STOPPED_GOAL_CHECKER_HPP_
#define NAV2_CONTROLLER__PLUGINS__STABLE_STOPPED_GOAL_CHECKER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex> // [추가] Thread safety를 위해 필요

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/goal_checker.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "nav_msgs/msg/path.hpp" // [추가] Path 메시지 타입

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

  // ... (기존 Helper 함수들 생략) ...
  bool isXYLatched() const { return stateful_ && !check_xy_; }
  double getXGoalTolerance() const { return x_goal_tolerance_; }
  double getYGoalTolerance() const { return y_goal_tolerance_; }

protected:
  // Tolerance parameters
  double x_goal_tolerance_;
  double y_goal_tolerance_;
  double yaw_goal_tolerance_;
  
  // Velocity parameters
  double rot_stopped_velocity_;
  double trans_stopped_velocity_;

  // Time stability parameters
  double xy_stability_duration_;
  double yaw_stability_duration_;

  // Logic control parameters
  bool stateful_;

  // State variables
  bool check_xy_;
  
  // Time tracking variables
  bool in_xy_tolerance_;
  bool in_yaw_tolerance_;
  rclcpp::Time first_xy_tolerance_time_;
  rclcpp::Time first_yaw_tolerance_time_;
  rclcpp::Clock::SharedPtr clock_;

  // [추가] Path Subscription 관련 변수
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  nav_msgs::msg::Path::SharedPtr current_path_;
  std::mutex path_mutex_; 
  std::string path_topic_; // 파라미터로 받기 위해 추가

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::string plugin_name_;

  /**
   * @brief Callback executed when a parameter change is detected
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // [추가] Path Callback 함수
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__STABLE_STOPPED_GOAL_CHECKER_HPP_

```

### 2. Source File (`stable_stopped_goal_checker.cpp`)

`initialize`에서 구독 설정, `pathCallback` 구현, 그리고 `isGoalReached` 도입부에 거리 체크 로직이 추가되었습니다.

```cpp
/*
 * ... (License 주석 생략) ...
 */

#include <cmath>
#include <string>
#include <memory>
#include <limits>
#include <vector>

#include "nav2_stable_stopped_goal_checker/stable_stopped_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "angles/angles.h"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

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
  check_xy_(true),
  in_xy_tolerance_(false),
  in_yaw_tolerance_(false),
  path_topic_("/plan") // 기본값
{
}

void StableStoppedGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();
  clock_ = node->get_clock();

  // Declare parameters
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
  
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".xy_stability_duration", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".yaw_stability_duration", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".stateful", rclcpp::ParameterValue(true));
  
  // [추가] path_topic 파라미터 선언
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".path_topic", rclcpp::ParameterValue("/plan"));

  // Get parameters
  node->get_parameter(plugin_name + ".x_goal_tolerance", x_goal_tolerance_);
  node->get_parameter(plugin_name + ".y_goal_tolerance", y_goal_tolerance_);
  node->get_parameter(plugin_name + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node->get_parameter(plugin_name + ".trans_stopped_velocity", trans_stopped_velocity_);
  node->get_parameter(plugin_name + ".rot_stopped_velocity", rot_stopped_velocity_);
  node->get_parameter(plugin_name + ".xy_stability_duration", xy_stability_duration_);
  node->get_parameter(plugin_name + ".yaw_stability_duration", yaw_stability_duration_);
  node->get_parameter(plugin_name + ".stateful", stateful_);
  node->get_parameter(plugin_name + ".path_topic", path_topic_);

  // [추가] Path Subscription 생성 (TransientLocal QoS 사용)
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local();
  path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    path_topic_, qos,
    std::bind(&StableStoppedGoalChecker::pathCallback, this, _1));

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&StableStoppedGoalChecker::dynamicParametersCallback, this, _1));
}

void StableStoppedGoalChecker::reset()
{
  check_xy_ = true;
  in_xy_tolerance_ = false;
  in_yaw_tolerance_ = false;
}

// [추가] Path Callback 구현
void StableStoppedGoalChecker::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  current_path_ = msg;
}

bool StableStoppedGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & velocity)
{
  // [추가] Path Length Check
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (current_path_) {
      // nav2_util을 사용하여 경로 길이 계산
      double total_distance = nav2_util::geometry_utils::calculate_path_length(*current_path_);
      
      // 남은 경로의 길이가 허용 오차의 2배보다 크면, 아직 도착하지 않은 것으로 간주
      if (total_distance > 2.0 * x_goal_tolerance_) {
        // 단, 이미 latch(XY 완료) 상태라면 거리 체크를 무시하고 Yaw 체크로 넘어갈 수 있도록
        // 아래 로직이 필요할 수 있으나, 요구사항에 맞춰 "엄격하게" 리턴합니다.
        // 만약 XY가 이미 맞았더라도 경로가 갑자기 길어지면(Replanning 등) 멈추지 않아야 합니다.
        // 여기서는 요구사항대로 "if distance <= ... 일 때만 체크"를 역으로 적용하여
        // "if distance > ... 이면 False 리턴"으로 구현합니다.
        
        // 주의: XY가 이미 맞아서 Latch된 상태(check_xy_ == false)에서도 
        // 경로가 다시 길어졌다면(새로운 계획) 체크를 재개해야 하는지는 정책에 따릅니다.
        // 여기서는 단순하게 적용합니다.
        return false; 
      }
    }
    // current_path_가 아직 없으면(nullptr), 안전을 위해 기존 로직을 수행하거나 false를 리턴할 수 있습니다.
    // 여기서는 path가 없으면 거리 체크를 패스하고 기존 로직으로 넘어갑니다.
  }

  // --- 기존 로직 시작 ---

  // 1. Calculate Errors
  double dx = fabs(query_pose.position.x - goal_pose.position.x);
  double dy = fabs(query_pose.position.y - goal_pose.position.y);
  double dyaw = fabs(angles::shortest_angular_distance(
    tf2::getYaw(query_pose.orientation),
    tf2::getYaw(goal_pose.orientation)));

  bool xy_ok = (dx <= x_goal_tolerance_) && (dy <= y_goal_tolerance_);
  bool yaw_ok = (dyaw <= yaw_goal_tolerance_);

  // 2. Logic based on 'stateful' parameter
  if (stateful_) {
    // === STATEFUL MODE: Check XY first, then Yaw ===
    
    if (check_xy_) {
      // Phase 1: Checking XY Stability
      if (xy_ok) {
        if (!in_xy_tolerance_) {
          first_xy_tolerance_time_ = clock_->now();
          in_xy_tolerance_ = true;
        }
        
        double time_in_xy = (clock_->now() - first_xy_tolerance_time_).seconds();
        
        // If XY is stable for duration, switch to Yaw phase
        if (time_in_xy >= xy_stability_duration_) {
          check_xy_ = false;
          in_xy_tolerance_ = false; // Reset for cleanliness
          in_yaw_tolerance_ = false; // Reset for next phase
        }
      } else {
        in_xy_tolerance_ = false;
      }
      return false; // Still working on XY or just finished XY
    } else {
      // Phase 2: Checking Yaw Stability (XY is already assumed done)
      if (yaw_ok) {
        if (!in_yaw_tolerance_) {
          first_yaw_tolerance_time_ = clock_->now();
          in_yaw_tolerance_ = true;
        }

        double time_in_yaw = (clock_->now() - first_yaw_tolerance_time_).seconds();

        if (time_in_yaw >= yaw_stability_duration_) {
          // Both phases passed, now check velocity
          return fabs(velocity.angular.z) <= rot_stopped_velocity_ &&
                 hypot(velocity.linear.x, velocity.linear.y) <= trans_stopped_velocity_;
        }
      } else {
        in_yaw_tolerance_ = false;
      }
      return false;
    }

  } else {
    // === NON-STATEFUL MODE: Check XY & Yaw Simultaneously ===
    
    if (xy_ok && yaw_ok) {
      if (!in_xy_tolerance_) {
        // Reuse xy variables for the combined state
        first_xy_tolerance_time_ = clock_->now();
        in_xy_tolerance_ = true;
      }

      double time_in_combined = (clock_->now() - first_xy_tolerance_time_).seconds();

      if (time_in_combined >= xy_stability_duration_) {
        // Tolerances & Duration met, now check velocity
        return fabs(velocity.angular.z) <= rot_stopped_velocity_ &&
               hypot(velocity.linear.x, velocity.linear.y) <= trans_stopped_velocity_;
      }
    } else {
      in_xy_tolerance_ = false;
    }
    return false;
  }
}

// ... (getTolerances 및 dynamicParametersCallback은 기존과 동일하지만 path_topic 처리 추가) ...

bool StableStoppedGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  // (기존 코드와 동일)
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
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".stateful") {
        stateful_ = parameter.as_bool();
      }
    } else if (type == ParameterType::PARAMETER_STRING) { // [추가]
        if (name == plugin_name_ + ".path_topic") {
            path_topic_ = parameter.as_string();
        }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_controller

PLUGINLIB_EXPORT_CLASS(nav2_controller::StableStoppedGoalChecker, nav2_core::GoalChecker)

```