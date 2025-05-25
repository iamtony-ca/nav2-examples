#ifndef REPLAN_MONITOR_NODE_HPP
#define REPLAN_MONITOR_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/bool.hpp"
#include <unordered_map>
#include <mutex>
#include <cmath>
#include <limits>

class ReplanMonitorNode : public rclcpp::Node {
public:
  ReplanMonitorNode();

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void evaluateReplanCondition();

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr replan_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path current_path_;
  geometry_msgs::msg::Pose current_pose_;
  nav_msgs::msg::OccupancyGrid current_costmap_;

  rclcpp::Time last_replan_time_;

  std::unordered_map<int, rclcpp::Time> obstacle_seen_time_;
  std::unordered_map<int, double> obstacle_distance_history_;
  std::mutex data_mutex_;
  size_t last_checked_index_ = 0; // 가장 가까운 pose index 추적용

  // Parameters
  double cooldown_sec_ = 1.0;
  double blocked_ratio_threshold_ = 0.3;
  double passed_pose_ignore_dist_ = 0.3;
  double obstacle_duration_threshold_sec_ = 1.0;  //tempp
  double approach_threshold_dist_ = 0.1;
  double max_speed_ = 0.5;
  double lookahead_time_sec_ = 2.5;
  double goal_ignore_radius_ = 0.005; //tempp
  double cost_threshold_ = 20; // tempp
};

#endif // REPLAN_MONITOR_NODE_HPP
