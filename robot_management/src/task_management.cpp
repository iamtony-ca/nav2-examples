// task_management.cpp

#include "robot_management/task_management.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

TaskManagement::TaskManagement(const rclcpp::NodeOptions & options)
: rclcpp::Node("task_management", options),
  start_set_flag_(false),
  moving_set_flag_(false),
  end_set_flag_(false)
{
  nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
    this, "navigate_to_pose");

  trigger_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/nav2_trigger", 10,
    std::bind(&TaskManagement::trigger_callback, this, std::placeholders::_1));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/local_costmap/costmap",
    rclcpp::QoS(10),
    std::bind(&TaskManagement::costmap_callback, this, std::placeholders::_1));

  monitor_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&TaskManagement::monitor_loop, this));

  // this->declare_parameter("use_sim_time", true);
}

void TaskManagement::initialize() {
  param_mgr_ = std::make_unique<Nav2ParamManager>(shared_from_this());
}

void TaskManagement::costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  current_costmap_ = *msg;
  // RCLCPP_INFO(this->get_logger(), "current_costmap_ = *msg;");
}

double TaskManagement::evaluate_cost_near_pose(const geometry_msgs::msg::PoseStamped & pose) {
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  if (current_costmap_.data.empty()) return -1.0;

  double sum = 0.0;
  int count = 0;

  const auto & info = current_costmap_.info;
  int radius = 3;
  int cx = static_cast<int>((pose.pose.position.x - info.origin.position.x) / info.resolution);
  int cy = static_cast<int>((pose.pose.position.y - info.origin.position.y) / info.resolution);

  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      int x = cx + dx;
      int y = cy + dy;
      if (x >= 0 && x < static_cast<int>(info.width) &&
          y >= 0 && y < static_cast<int>(info.height)) {
        int idx = y * info.width + x;
        int8_t cost = current_costmap_.data[idx];
        if (cost >= 0) {
          sum += cost;
          count++;
        }
      }
    }
  }

  return count > 0 ? sum / count : -1.0;
}

geometry_msgs::msg::PoseStamped TaskManagement::get_robot_pose() {
  geometry_msgs::msg::PoseStamped pose_in, pose_out;
  pose_in.header.frame_id = "base_link";
  // pose_in.header.stamp = rclcpp::Time(0);
  pose_in.header.stamp = this->now();

  try {
    tf_buffer_->transform(pose_in, pose_out, "map", tf2::durationFromSec(0.5));
    pose_out.header.stamp = this->now();
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
  }

  return pose_out;
}

// geometry_msgs::msg::PoseStamped TaskManagement::get_robot_pose() {
//   geometry_msgs::msg::PoseStamped pose_in, pose_out;
//   pose_in.header.frame_id = "base_link";
//   //  costmap 기준 시간으로 맞추기
//   {
//     std::lock_guard<std::mutex> lock(costmap_mutex_);
//     pose_in.header.stamp = current_costmap_.header.stamp;
//   }

//   try {
//     tf_buffer_->transform(pose_in, pose_out, current_costmap_.header.frame_id, tf2::durationFromSec(0.5));
//     pose_out.header.stamp = current_costmap_.header.stamp;
//   } catch (const tf2::TransformException & ex) {
//     RCLCPP_WARN(this->get_logger(), "TF transform 실패: %s", ex.what());
//   }

//   return pose_out;
// }


bool TaskManagement::is_within_radius(const geometry_msgs::msg::PoseStamped & a,
                                      const geometry_msgs::msg::PoseStamped & b,
                                      double radius) {
  double dx = a.pose.position.x - b.pose.position.x;
  double dy = a.pose.position.y - b.pose.position.y;
  return std::sqrt(dx * dx + dy * dy) <= radius;
}

void TaskManagement::monitor_loop() {
  // RCLCPP_INFO(this->get_logger(), "000");
  if (!goal_param_set_) return;

  auto current_pose = get_robot_pose();

  // RCLCPP_INFO_THROTTLE(this->get_logger(), "현재 로봇 위치: (%.2f, %.2f)", current_pose.pose.position.x, current_pose.pose.position.y);
  if (evaluate_cost_near_pose(start_pose_) > 25.0 && is_within_radius(current_pose, start_pose_, 3.0)) {
    // RCLCPP_INFO(this->get_logger(), "111");
    if (start_set_flag_) return;
    // RCLCPP_INFO(this->get_logger(), "111");
    param_mgr_->set_param_typed("/local_costmap/local_costmap", "inflation_layer.inflation_radius", 0.3);
    param_mgr_->set_param_typed("/local_costmap/local_costmap", "inflation_layer.cost_scaling_factor", 4.5);
    RCLCPP_INFO(this->get_logger(), "/local_costmap/local_costmap, inflation_layer.inflation_radius, 0.3");
    start_set_flag_ = true;
    end_set_flag_ = false;
  }

  if (!is_within_radius(current_pose, start_pose_, 3.0) && !is_within_radius(current_pose, goal_pose_, 4.0)) {
    RCLCPP_INFO(this->get_logger(), "222");
    if (moving_set_flag_) return;
    param_mgr_->set_param_typed("/local_costmap/local_costmap", "inflation_layer.inflation_radius", 1.0);
    param_mgr_->set_param_typed("/local_costmap/local_costmap", "inflation_layer.cost_scaling_factor", 3.5);
    RCLCPP_INFO(this->get_logger(), "/local_costmap/local_costmap, inflation_layer.inflation_radius, 0.8");
    moving_set_flag_ = true;
    start_set_flag_ = false;
  }


  auto local_goal = transform_goal_pose_to_costmap_frame();
  if (local_goal.header.frame_id.empty()) return;  // TF 변환 실패 시 무시
  // double cost = evaluate_cost_near_pose(local_goal);

  RCLCPP_INFO(this->get_logger(), "%f", evaluate_cost_near_pose(local_goal));
  if (evaluate_cost_near_pose(local_goal) > 15.0 && is_within_radius(current_pose, goal_pose_, 4.0)) {
    RCLCPP_INFO(this->get_logger(), "333");
    if (end_set_flag_) return;
    param_mgr_->set_param_typed("/local_costmap/local_costmap", "inflation_layer.inflation_radius", 0.3);
    param_mgr_->set_param_typed("/local_costmap/local_costmap", "inflation_layer.cost_scaling_factor", 4.5);
    RCLCPP_INFO(this->get_logger(), "/local_costmap/local_costmap, inflation_layer.inflation_radius, 0.3 again ");
    end_set_flag_ = true;
    moving_set_flag_ = false;
    goal_param_set_ = false;
  }
}


geometry_msgs::msg::PoseStamped TaskManagement::transform_goal_pose_to_costmap_frame() {
  geometry_msgs::msg::PoseStamped updated_pose = goal_pose_;
  updated_pose.header.stamp = this->now();  //  현재 시간으로 덮어쓰기

  geometry_msgs::msg::PoseStamped transformed;
  try {
    tf_buffer_->transform(updated_pose, transformed, current_costmap_.header.frame_id, tf2::durationFromSec(0.3));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF transform (goal → costmap) 실패: %s", ex.what());
    return {};
  }

  return transformed;
}


// geometry_msgs::msg::PoseStamped TaskManagement::transform_goal_pose_to_costmap_frame() {
//   geometry_msgs::msg::PoseStamped updated_pose = goal_pose_;

//   {
//     std::lock_guard<std::mutex> lock(costmap_mutex_);
//     updated_pose.header.stamp = current_costmap_.header.stamp;
//   }

//   geometry_msgs::msg::PoseStamped transformed;
//   try {
//     tf_buffer_->transform(updated_pose, transformed, current_costmap_.header.frame_id, tf2::durationFromSec(0.3));
//   } catch (const tf2::TransformException & ex) {
//     RCLCPP_WARN(this->get_logger(), "TF transform (goal → costmap) 실패: %s", ex.what());
//     return {};
//   }

//   return transformed;
// }


void TaskManagement::trigger_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "/nav2_trigger 수신: '%s'", msg->data.c_str());
  if (msg->data == "go") {
    start_pose_ = get_robot_pose();
    start_set_flag_ = false;
    moving_set_flag_ = false;
    end_set_flag_ = false;
    goal_param_set_ = true;
    send_goal();
    
  } else {
    RCLCPP_WARN(this->get_logger(), "알 수 없는 명령: '%s'", msg->data.c_str());
  }
}

void TaskManagement::send_goal()
{
  if (!nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_ERROR(this->get_logger(), "navigate_to_pose 서버 연결 실패");
    return;
  }

  // - Translation: [8.938, 0.107, 0.000]
  // - Rotation: in Quaternion [0.000, 0.000, 1.000, 0.001]


  NavigateToPose::Goal goal_msg;
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = this->now();
  goal_msg.pose.pose.position.x = 8.982;
  goal_msg.pose.pose.position.y = 0.042;
  goal_msg.pose.pose.orientation.w = 1.0;
  // goal_msg.pose.pose.position.x = 1.0;
  // goal_msg.pose.pose.position.y = 2.0;
  // goal_msg.pose.pose.orientation.w = 1.0;
  goal_pose_ = goal_msg.pose;

  auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  options.goal_response_callback = std::bind(&TaskManagement::goal_response_callback, this, std::placeholders::_1);
  options.feedback_callback = std::bind(&TaskManagement::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  options.result_callback = std::bind(&TaskManagement::result_callback, this, std::placeholders::_1);

  nav_to_pose_client_->async_send_goal(goal_msg, options);
  RCLCPP_INFO(this->get_logger(), "NavigateToPose goal 전송 요청");
}

void TaskManagement::goal_response_callback(GoalHandleNav::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "목표가 거부됨");
  } else {
    RCLCPP_INFO(this->get_logger(), "목표 수락됨");
  }
}

void TaskManagement::feedback_callback(
  GoalHandleNav::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "남은 거리: %.2f m", feedback->distance_remaining);
}

void TaskManagement::result_callback(const GoalHandleNav::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "목표 도달 성공");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "중단됨");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "취소됨");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "알 수 없는 결과");
      break;
  }
}
