#include "replan_monitor/replan_monitor_node.hpp"
#include <cmath>
#include <limits>

ReplanMonitorNode::ReplanMonitorNode()
: Node("replan_monitor_node") {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/plan", 10,
    std::bind(&ReplanMonitorNode::pathCallback, this, std::placeholders::_1));

  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/amcl_pose", 10,   // tempp
    std::bind(&ReplanMonitorNode::poseCallback, this, std::placeholders::_1));

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/local_costmap/costmap", 10,
    std::bind(&ReplanMonitorNode::costmapCallback, this, std::placeholders::_1));

  replan_pub_ = this->create_publisher<std_msgs::msg::Bool>("/decor_flag", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&ReplanMonitorNode::evaluateReplanCondition, this));

  last_replan_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "ReplanMonitorNode initialized");
}

void ReplanMonitorNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_path_ = *msg;
  last_checked_index_ = 0;  // path 업데이트 시 reset!
}

void ReplanMonitorNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_pose_ = msg->pose;
}

void ReplanMonitorNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_costmap_ = *msg;
}

void ReplanMonitorNode::evaluateReplanCondition() {
  std::lock_guard<std::mutex> lock(data_mutex_);

  if (current_path_.poses.empty() || current_costmap_.data.empty()) return;

  double resolution = current_costmap_.info.resolution;
  double origin_x = current_costmap_.info.origin.position.x;
  double origin_y = current_costmap_.info.origin.position.y;
  unsigned int width = current_costmap_.info.width;
  unsigned int height = current_costmap_.info.height;

  rclcpp::Time now = this->now();
  double lookahead_distance = max_speed_ * lookahead_time_sec_;
  lookahead_distance = 10.0;  // tempp

  const geometry_msgs::msg::Pose goal_pose = current_path_.poses.back().pose;
  if (!std::isfinite(goal_pose.position.x) || !std::isfinite(goal_pose.position.y)) {
    RCLCPP_WARN(this->get_logger(), "Invalid goal pose. Skipping replan check.");
    return;
  }

  // 가장 가까운 경로 index 탐색 (효율성 위해 이전 index부터 탐색)
  double min_dist = std::numeric_limits<double>::max();
  size_t closest_index = last_checked_index_;
  for (size_t i = last_checked_index_; i < current_path_.poses.size(); ++i) {
    double dx = current_path_.poses[i].pose.position.x - current_pose_.position.x;
    double dy = current_path_.poses[i].pose.position.y - current_pose_.position.y;
    double dist = std::hypot(dx, dy);
    if (dist < min_dist) {
      min_dist = dist;
      closest_index = i;
    }
  }
  last_checked_index_ = closest_index;

  size_t checked = 0, blocked = 0;

  for (size_t i = closest_index; i < current_path_.poses.size(); ++i) {
    const auto &pose_stamped = current_path_.poses[i];

    double dx = pose_stamped.pose.position.x - current_pose_.position.x;
    double dy = pose_stamped.pose.position.y - current_pose_.position.y;
    double dist = std::hypot(dx, dy);
    if (dist > lookahead_distance) break;

    double goal_dx = pose_stamped.pose.position.x - goal_pose.position.x;
    double goal_dy = pose_stamped.pose.position.y - goal_pose.position.y;
    double goal_dist = std::hypot(goal_dx, goal_dy);
    if (goal_dist < goal_ignore_radius_) continue;

    int mx = static_cast<int>(std::floor((pose_stamped.pose.position.x - origin_x) / resolution));
    int my = static_cast<int>(std::floor((pose_stamped.pose.position.y - origin_y) / resolution));
    if (mx < 0 || my < 0 || mx >= static_cast<int>(width) || my >= static_cast<int>(height)) continue;

    int index = my * width + mx;
    int cost = current_costmap_.data[index];
    RCLCPP_INFO(this->get_logger(), "[DEBUG] dist=%.2f, goal_dist=%.2f, cost=%d", dist, goal_dist, cost);
    RCLCPP_INFO(this->get_logger(),
    "[GRID] x=%.2f y=%.2f → mx=%d my=%d costmap size: %d x %d",
    pose_stamped.pose.position.x,
    pose_stamped.pose.position.y,
    mx, my, width, height);
    

    // if (cost >= 100) {   //tempp
    if (cost >= cost_threshold_) {

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

        if (duration.seconds() >= obstacle_duration_threshold_sec_) {
        // if (duration.seconds() >= obstacle_duration_threshold_sec_ && is_approaching) {
          blocked++;
        }
        // RCLCPP_INFO(this->get_logger(), "[DEBUG] duration=%.2f, goal_dist=%.2f, cost=f", duration.seconds(), goal_dist, cost);
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

  // RCLCPP_INFO(this->get_logger(), "blocked_ratio: %.2f, checked: %zu, blocked: %zu",
  //             blocked_ratio, checked, blocked);

  
  RCLCPP_INFO(this->get_logger(), "[DEBUG] checked=%ld blocked=%ld ratio=%.2f", checked, blocked, blocked_ratio);
  // RCLCPP_INFO(this->get_logger(), "[DEBUG] dist=%.2f, goal_dist=%.2f, cost=%d", dist, goal_dist, cost);
  if (blocked_ratio > blocked_ratio_threshold_ &&
      (now - last_replan_time_).seconds() > cooldown_sec_) {
    flag_msg.data = true;
    last_replan_time_ = now;
    RCLCPP_WARN(this->get_logger(), "Triggering replan: blocked ratio %.2f", blocked_ratio);
    
  }


  

  if (flag_msg.data) {
    replan_pub_->publish(flag_msg);
    RCLCPP_INFO(this->get_logger(), "Replan triggered True:");
  } 
  

}