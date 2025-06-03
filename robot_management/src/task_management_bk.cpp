// task_management.cpp
#include "robot_management/task_management.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

TaskManagement::TaskManagement(const rclcpp::NodeOptions & options)
: rclcpp::Node("task_management", options)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/local_costmap/costmap",
    rclcpp::QoS(10),
    std::bind(&TaskManagement::costmap_callback, this, std::placeholders::_1));

  monitor_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&TaskManagement::monitor_loop, this));

  //  shared_from_this()는 여기서 호출하면 안 됨
}

// 이 함수를 생성 후 반드시 호출
void TaskManagement::initialize() {
  param_mgr_ = std::make_unique<Nav2ParamManager>(shared_from_this());
}


void TaskManagement::costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  current_costmap_ = *msg;
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
  pose_in.header.stamp = rclcpp::Time(0);  // latest available

  try {
    tf_buffer_->transform(pose_in, pose_out, "map", tf2::durationFromSec(0.5));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
  }

  return pose_out;
}


// geometry_msgs::msg::PoseStamped TaskManagement::get_robot_pose() {
//   geometry_msgs::msg::PoseStamped pose_in, pose_out;
//   pose_in.header.frame_id = "base_link";
//   pose_in.header.stamp = this->get_clock()->now();

//   try {
//     tf_buffer_->transform(pose_in, pose_out, "map", tf2::durationFromSec(0.5));
//   } catch (const tf2::TransformException & ex) {
//     RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
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
  auto current_pose = get_robot_pose();

  if (!start_param_set_) {
    start_pose_ = current_pose;
    start_param_set_ = true;
  }

  if (!goal_param_set_) {
    // 예시: goal_pose_은 외부에서 주입되거나 설정된다고 가정
    return;
  }

  if (!left_start_area_ && !is_within_radius(current_pose, start_pose_, 3.0)) {
    double cost = evaluate_cost_near_pose(current_pose);
    if (cost >= 50.0) {
      param_mgr_->set_param_typed("local_costmap", "inflation_radius", 0.2);
      param_mgr_->set_param_typed("local_costmap", "cost_scaling_factor", 5.0);
    }
    left_start_area_ = true;
  }

  if (is_within_radius(current_pose, goal_pose_, 3.0)) {
    double cost = evaluate_cost_near_pose(current_pose);
    if (cost >= 50.0) {
      param_mgr_->set_param_typed("local_costmap", "inflation_radius", 0.3);
      param_mgr_->set_param_typed("local_costmap", "cost_scaling_factor", 4.0);
    }
  }
}
