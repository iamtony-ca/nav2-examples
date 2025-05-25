// replan_monitor_node.cpp

#include "replan_monitor/replan_monitor_node.hpp"
#include <cmath>
#include <unordered_map>

ReplanMonitorNode::ReplanMonitorNode()
: Node("replan_monitor_node") {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/plan", 10,
    std::bind(&ReplanMonitorNode::pathCallback, this, std::placeholders::_1));

  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/robot_pose", 10,
    std::bind(&ReplanMonitorNode::poseCallback, this, std::placeholders::_1));

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/local_costmap/costmap", 10,
    std::bind(&ReplanMonitorNode::costmapCallback, this, std::placeholders::_1));

  replan_pub_ = this->create_publisher<std_msgs::msg::Bool>("/replan_flag", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&ReplanMonitorNode::evaluateReplanCondition, this));

  last_replan_time_ = this->now();
  obstacle_duration_threshold_sec_ = 2.0;
  approach_threshold_dist_ = 0.1;
  max_speed_ = 0.5;
  lookahead_time_sec_ = 2.5;
  goal_ignore_radius_ = 0.5; // goal 근처 예외 처리 거리
}

void ReplanMonitorNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  current_path_ = *msg;
}

void ReplanMonitorNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  current_pose_ = msg->pose;
}

void ReplanMonitorNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_costmap_ = *msg;
}

void ReplanMonitorNode::evaluateReplanCondition() {
  if (current_path_.poses.empty() || current_costmap_.data.empty()) return;

  double resolution = current_costmap_.info.resolution;
  double origin_x = current_costmap_.info.origin.position.x;
  double origin_y = current_costmap_.info.origin.position.y;
  unsigned int width = current_costmap_.info.width;
  unsigned int height = current_costmap_.info.height;

  rclcpp::Time now = this->now();

  double lookahead_distance = max_speed_ * lookahead_time_sec_;

  const geometry_msgs::msg::Pose goal_pose = current_path_.poses.back().pose;

  size_t checked = 0, blocked = 0;

  for (const auto &pose_stamped : current_path_.poses) {
    double dx = pose_stamped.pose.position.x - current_pose_.position.x;
    double dy = pose_stamped.pose.position.y - current_pose_.position.y;
    double dist = std::hypot(dx, dy);

    if (dist < passed_pose_ignore_dist_ || dist > lookahead_distance) continue;

    double goal_dx = pose_stamped.pose.position.x - goal_pose.position.x;
    double goal_dy = pose_stamped.pose.position.y - goal_pose.position.y;
    double goal_dist = std::hypot(goal_dx, goal_dy);
    if (goal_dist < goal_ignore_radius_) continue; // goal 근처 pose는 무시

    int mx = static_cast<int>((pose_stamped.pose.position.x - origin_x) / resolution);
    int my = static_cast<int>((pose_stamped.pose.position.y - origin_y) / resolution);

    if (mx < 0 || my < 0 || mx >= static_cast<int>(width) || my >= static_cast<int>(height)) continue;

    int index = my * width + mx;
    int cost = current_costmap_.data[index];

    if (cost >= 100) {
      auto it = obstacle_seen_time_.find(index);
      if (it == obstacle_seen_time_.end()) {
        obstacle_seen_time_[index] = now;
        obstacle_distance_history_[index] = dist;
        continue;
      } else {
        rclcpp::Duration duration = now - it->second;
        bool is_approaching = false;
        if (obstacle_distance_history_.count(index)) {
          double prev_dist = obstacle_distance_history_[index];
          if (prev_dist - dist > approach_threshold_dist_) {
            is_approaching = true;
          }
          obstacle_distance_history_[index] = dist;
        }

        if (duration.seconds() >= obstacle_duration_threshold_sec_ && is_approaching) {
          blocked++;
        }
      }
    } else {
      obstacle_seen_time_.erase(index);
      obstacle_distance_history_.erase(index);
    }

    checked++;
  }

  double blocked_ratio = checked > 0 ? static_cast<double>(blocked) / checked : 0.0;

  std_msgs::msg::Bool flag_msg;
  flag_msg.data = false;

  if (blocked_ratio > blocked_ratio_threshold_ &&
      (now - last_replan_time_).seconds() > cooldown_sec_) {
    flag_msg.data = true;
    last_replan_time_ = now;
    RCLCPP_WARN(this->get_logger(), "Triggering replan: blocked ratio %.2f", blocked_ratio);
  }

  replan_pub_->publish(flag_msg);
}
