// task_management.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <mutex>
#include "robot_management/nav2_param_manager.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

class TaskManagement : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit TaskManagement(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  void initialize();  // 생성자 이후 반드시 호출해야 함

private:


  void trigger_callback(const std_msgs::msg::String::SharedPtr msg);
  void send_goal();
  void goal_response_callback(GoalHandleNav::SharedPtr goal_handle);
  void feedback_callback(GoalHandleNav::SharedPtr,
                         const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void result_callback(const GoalHandleNav::WrappedResult & result);

  void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void monitor_loop();
  double evaluate_cost_near_pose(const geometry_msgs::msg::PoseStamped & pose);

  geometry_msgs::msg::PoseStamped get_robot_pose();
  bool is_within_radius(const geometry_msgs::msg::PoseStamped & a,
                        const geometry_msgs::msg::PoseStamped & b,
                        double radius);


  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trigger_sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;
  nav_msgs::msg::OccupancyGrid current_costmap_;
  std::mutex costmap_mutex_;

  std::unique_ptr<Nav2ParamManager> param_mgr_;

  geometry_msgs::msg::PoseStamped start_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  bool start_param_set_ = false;
  bool goal_param_set_ = false;
  bool left_start_area_ = false;


};
