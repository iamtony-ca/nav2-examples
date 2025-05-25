#include "replan_monitor/replan_monitor_node.hpp"

ReplanMonitorNode::ReplanMonitorNode()
: Node("replan_monitor_node") {
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::SubscriptionOptions sub_options;
  auto callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  sub_options.callback_group = callback_group;

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/plan", rclcpp::QoS(10), std::bind(&ReplanMonitorNode::pathCallback, this, std::placeholders::_1), sub_options);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/local_costmap/costmap", rclcpp::QoS(10), std::bind(&ReplanMonitorNode::costmapCallback, this, std::placeholders::_1), sub_options);

  replan_pub_ = this->create_publisher<std_msgs::msg::Bool>("/replan_flag", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&ReplanMonitorNode::evaluateReplanCondition, this));


  RCLCPP_INFO(this->get_logger(), "ReplanMonitorNode initialized");  
}

void ReplanMonitorNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_path_ = *msg;
  last_checked_index_ = 0;
}

void ReplanMonitorNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_costmap_ = *msg;
}

bool ReplanMonitorNode::getCurrentPoseFromTF(geometry_msgs::msg::Pose &pose_out) {
  try {
    geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
      source_frame_, target_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));

    pose_out.position.x = tf.transform.translation.x;
    pose_out.position.y = tf.transform.translation.y;
    pose_out.position.z = tf.transform.translation.z;
    pose_out.orientation = tf.transform.rotation;
    return true;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
    return false;
  }
}

void ReplanMonitorNode::evaluateReplanCondition() {
  std::lock_guard<std::mutex> lock(data_mutex_);

  if (current_path_.poses.empty() || current_costmap_.data.empty()) return;

  geometry_msgs::msg::Pose current_pose;
  if (!getCurrentPoseFromTF(current_pose)) return;

  const auto &info = current_costmap_.info;
  double resolution = info.resolution;
  double origin_x = info.origin.position.x;
  double origin_y = info.origin.position.y;
  unsigned int width = info.width;
  unsigned int height = info.height;

  rclcpp::Time now = this->now();
  double lookahead_distance = max_speed_ * lookahead_time_sec_;
  const auto &goal_pose = current_path_.poses.back().pose;
  if (!std::isfinite(goal_pose.position.x) || !std::isfinite(goal_pose.position.y)) return;

  size_t checked = 0, blocked = 0;
  for (size_t i = last_checked_index_; i < current_path_.poses.size(); ++i) {
    const auto &pose = current_path_.poses[i].pose;
    double dx = pose.position.x - current_pose.position.x;
    double dy = pose.position.y - current_pose.position.y;
    double dist = std::hypot(dx, dy);

    if (dist < passed_pose_ignore_dist_ || dist > lookahead_distance) continue;

    double goal_dist = std::hypot(pose.position.x - goal_pose.position.x,
                                  pose.position.y - goal_pose.position.y);
    if (goal_dist < goal_ignore_radius_) continue;

    int mx = static_cast<int>(std::floor((pose.position.x - origin_x) / resolution));
    int my = static_cast<int>(std::floor((pose.position.y - origin_y) / resolution));
    if (mx < 0 || my < 0 || mx >= static_cast<int>(width) || my >= static_cast<int>(height)) continue;

    int index = my * width + mx;
    int cost = current_costmap_.data[index];

    if (cost >= cost_threshold_) {
      auto it = obstacle_seen_time_.find(index);
      if (it == obstacle_seen_time_.end()) {
        obstacle_seen_time_[index] = now;
        obstacle_distance_history_[index] = dist;
        continue;
      }

      rclcpp::Duration duration = now - it->second;
      bool is_approaching = false;
      if (obstacle_distance_history_.count(index)) {
        double prev_dist = obstacle_distance_history_[index];
        if (prev_dist - dist > approach_threshold_dist_) is_approaching = true;
        obstacle_distance_history_[index] = dist;
      }

      if (duration.seconds() >= obstacle_duration_threshold_sec_ && is_approaching) {
        blocked++;
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
