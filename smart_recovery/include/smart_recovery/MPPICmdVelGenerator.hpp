// MPPICmdVelGenerator.hpp
// =============================================================
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <eigen3/Eigen/Dense>
#include <vector>

class MPPICmdVelGenerator : public rclcpp::Node
{
public:
  explicit MPPICmdVelGenerator(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions());

private:
  // callbacks
  void onPathReceived(const nav_msgs::msg::Path::SharedPtr msg);
  void onCostmapReceived(const nav2_msgs::msg::Costmap::SharedPtr msg);
  void controlLoop();

  // helpers
  std::vector<geometry_msgs::msg::Pose> simulateTrajectory(const geometry_msgs::msg::Pose & start,
                                                           double v, double w);
  double evaluateTrajectory(const std::vector<geometry_msgs::msg::Pose> & traj);
  geometry_msgs::msg::Pose getCurrentPose();

  // state
  nav_msgs::msg::Path         latest_path_;
  nav2_msgs::msg::Costmap     latest_costmap_;

  // interfaces
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr       path_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr   costmap_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr          done_pub_;
  rclcpp::TimerBase::SharedPtr                               timer_;

  // TF
  tf2_ros::Buffer                                               tf_buffer_;
  tf2_ros::TransformListener                                    tf_listener_;
};