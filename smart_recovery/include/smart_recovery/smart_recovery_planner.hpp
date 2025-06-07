// smart_recovery_planner.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Core>
#include <mutex>
#include <cstdint>
#include <algorithm>
#include "robot_interfaces/srv/smart_recovery_path.hpp"

struct RobotConfig
{
  std::vector<Eigen::Vector2f> footprint;
  float min_turning_radius{0.40F};
  bool allow_reverse{true};
};

class SmartRecoveryPlanner : public rclcpp::Node
{
public:
  explicit SmartRecoveryPlanner(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions());

private:
  void handleService(
    const std::shared_ptr<robot_interfaces::srv::SmartRecoveryPath::Request> req,
    std::shared_ptr<robot_interfaces::srv::SmartRecoveryPath::Response> res);

  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  geometry_msgs::msg::PoseStamped getRobotPose();
  // bool isPoseTraversable(const geometry_msgs::msg::Pose & pose) const;
  bool isPoseTraversable(const geometry_msgs::msg::Pose & pose);

  nav_msgs::msg::Path createCandidatePath(float angle_rad, float dist_m, const geometry_msgs::msg::PoseStamped & start);
  nav_msgs::msg::Path generateBestRecoveryPath(const geometry_msgs::msg::PoseStamped & goal, const geometry_msgs::msg::PoseStamped & start);

  float evaluatePath(const nav_msgs::msg::Path & p, const nav_msgs::msg::Path & global, const geometry_msgs::msg::Pose & goal);
  float computeCostScore(const nav_msgs::msg::Path & p);
  float computeClearanceScore(const nav_msgs::msg::Path & p);
  float computeAlignmentScore(const nav_msgs::msg::Path & p, const geometry_msgs::msg::Pose & goal);
  float computeLookaheadAlignment(const nav_msgs::msg::Path & c, const nav_msgs::msg::Path & g, float d);
  float computeDistanceImprovement(const nav_msgs::msg::Path & p, const geometry_msgs::msg::Pose & goal);
  float computeCurvaturePenalty(const nav_msgs::msg::Path & p);
  float computeSmoothnessPenalty(const nav_msgs::msg::Path & p);
  float computeMinClearance(const nav_msgs::msg::Path & path);
  float computeLengthPenalty(const nav_msgs::msg::Path & path);

  rclcpp::Service<robot_interfaces::srv::SmartRecoveryPath>::SharedPtr srv_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
  mutable std::mutex costmap_mtx_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  RobotConfig robot_cfg_;
  double search_radius_{1.0};
};