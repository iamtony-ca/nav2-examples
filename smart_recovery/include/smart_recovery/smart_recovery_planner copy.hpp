// smart_recovery_planner.hpp + .cpp (with SmartRecoveryPath service server)

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <mutex>
#include "robot_interfaces/srv/smart_recovery_path.hpp"  // auto‑generated from .srv

struct RobotConfig {
  float min_turning_radius;
  bool allow_reverse;
};

class SmartRecoveryPlanner : public rclcpp::Node {
public:
  SmartRecoveryPlanner();

private:
  // core API
  nav_msgs::msg::Path generateBestRecoveryPath(const geometry_msgs::msg::PoseStamped &goal);

  // helpers
  void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
  geometry_msgs::msg::PoseStamped getRobotPose();
  nav_msgs::msg::Path createCandidatePath(float angle, float distance);
  float evaluatePath(const nav_msgs::msg::Path &path,
                     const nav_msgs::msg::Path &global_path,
                     const geometry_msgs::msg::Pose &goal);
  float computeAlignmentScore(const nav_msgs::msg::Path &, const geometry_msgs::msg::Pose &);
  float computeLookaheadAlignment(const nav_msgs::msg::Path &, const nav_msgs::msg::Path &, float);
  float computeCurvaturePenalty(const nav_msgs::msg::Path &);
  float computeSmoothnessPenalty(const nav_msgs::msg::Path &);
  float computeDistanceImprovement(const nav_msgs::msg::Path &, const geometry_msgs::msg::Pose &);
  float computeClearanceScore(const nav_msgs::msg::Path &);
  float computeCostScore(const nav_msgs::msg::Path &);

  // service callback
  void handleService(const std::shared_ptr<robot_interfaces::srv::SmartRecoveryPath::Request> req,
                     std::shared_ptr<robot_interfaces::srv::SmartRecoveryPath::Response> res);

  // members
  rclcpp::Service<robot_interfaces::srv::SmartRecoveryPath>::SharedPtr srv_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  nav2_msgs::msg::Costmap::SharedPtr latest_costmap_;
  std::mutex costmap_mutex_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  RobotConfig robot_config_;
};