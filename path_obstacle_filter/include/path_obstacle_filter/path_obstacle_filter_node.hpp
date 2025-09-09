#ifndef PATH_OBSTACLE_FILTER__PATH_OBSTACLE_FILTER_NODE_HPP_
#define PATH_OBSTACLE_FILTER__PATH_OBSTACLE_FILTER_NODE_HPP_

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "laser_geometry/laser_geometry.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace path_obstacle_filter
{

class PathObstacleFilterNode : public rclcpp::Node
{
public:
  explicit PathObstacleFilterNode(const rclcpp::NodeOptions & options);

private:
  // Callbacks
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  // CHANGED: Renamed for clarity and added a new callback for the second action
  void navToPoseStatusCallback(const action_msgs::msg::GoalStatusArray::SharedPtr msg);
  void navThroughPosesStatusCallback(const action_msgs::msg::GoalStatusArray::SharedPtr msg);

  // Main logic
  void process();

  // Helper functions
  void getParameters();
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose);
  double pointToPathSegmentDistance(
    const pcl::PointXYZ & point,
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2);
  // ADDED: Helper to update status flags to avoid code duplication
  void updateGoalStatus(const action_msgs::msg::GoalStatusArray::SharedPtr msg, bool & is_active_flag);

  // Subscriptions & Publisher
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  // CHANGED: Two separate subscriptions for each action status
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav_to_pose_status_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav_through_poses_status_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Main timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Data
  std::mutex data_mutex_;
  nav_msgs::msg::Path current_path_;
  pcl::PointCloud<pcl::PointXYZ> latest_cloud_;
  bool path_received_ = false;
  bool cloud_received_ = false;
  // CHANGED: Replaced is_goal_active_ with two separate flags
  bool is_nav_to_pose_active_ = false;
  bool is_nav_through_poses_active_ = false;

  // Parameters
  std::string global_frame_;
  std::string robot_base_frame_;
  double distance_threshold_;
  double lookahead_dist_;
  double goal_tolerance_;
  
  // Pre-inflation & Merging Parameters
  bool enable_pre_inflation_;
  double pre_inflation_radius_;
  int inflation_points_;
  bool enable_data_merging_;

  // For LaserScan to PointCloud2 conversion
  laser_geometry::LaserProjection projector_;
};

}  // namespace path_obstacle_filter

#endif  // PATH_OBSTACLE_FILTER__PATH_OBSTACLE_FILTER_NODE_HPP_