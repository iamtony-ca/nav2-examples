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
  void goalStatusCallback(const action_msgs::msg::GoalStatusArray::SharedPtr msg);

  // Main logic
  void process();

  // Helper functions
  void getParameters();
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose);
  double pointToPathSegmentDistance(
    const pcl::PointXYZ & point,
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2);

  // Subscriptions & Publisher
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr goal_status_sub_;
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
  bool is_goal_active_ = false;

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
  bool enable_data_merging_; // ❗️ 추가: 데이터 통합 기능 활성화 파라미터

  // For LaserScan to PointCloud2 conversion
  laser_geometry::LaserProjection projector_;
};

}  // namespace path_obstacle_filter

#endif  // PATH_OBSTACLE_FILTER__PATH_OBSTACLE_FILTER_NODE_HPP_










#include "path_obstacle_filter/path_obstacle_filter_node.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace path_obstacle_filter
{

PathObstacleFilterNode::PathObstacleFilterNode(const rclcpp::NodeOptions & options)
: Node("path_obstacle_filter", options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing PathObstacleFilterNode...");

  this->getParameters();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/plan", default_qos, std::bind(&PathObstacleFilterNode::pathCallback, this, std::placeholders::_1));

  goal_status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
    "/navigate_to_pose/_action/status", rclcpp::SystemDefaultsQoS(),
    std::bind(&PathObstacleFilterNode::goalStatusCallback, this, std::placeholders::_1));

  std::string sensor_type = this->get_parameter("sensor_topic_type").as_string();
  if (sensor_type == "scan") {
    RCLCPP_INFO(this->get_logger(), "Subscribing to LaserScan topic: /scan");
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(), std::bind(&PathObstacleFilterNode::scanCallback, this, std::placeholders::_1));
  } else if (sensor_type == "pointcloud") {
    RCLCPP_INFO(this->get_logger(), "Subscribing to PointCloud2 topic: /pointcloud");
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/pointcloud", rclcpp::SensorDataQoS(), std::bind(&PathObstacleFilterNode::pointcloudCallback, this, std::placeholders::_1));
  }

  filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/filtered_cloud", default_qos);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&PathObstacleFilterNode::process, this));

  RCLCPP_INFO(this->get_logger(), "PathObstacleFilterNode initialized successfully.");
}

void PathObstacleFilterNode::getParameters()
{
  this->declare_parameter("global_frame", "map");
  this->declare_parameter("robot_base_frame", "base_link");
  this->declare_parameter("sensor_topic_type", "scan");
  this->declare_parameter("distance_threshold", 0.5);
  this->declare_parameter("lookahead_dist", 2.0);
  this->declare_parameter("goal_tolerance", 0.25);
  this->declare_parameter("enable_pre_inflation", true);
  this->declare_parameter("pre_inflation_radius", 0.4);
  this->declare_parameter("inflation_points", 8);
  this->declare_parameter("enable_data_merging", true); // ❗️ 추가

  global_frame_ = this->get_parameter("global_frame").as_string();
  robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();
  distance_threshold_ = this->get_parameter("distance_threshold").as_double();
  lookahead_dist_ = this->get_parameter("lookahead_dist").as_double();
  goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
  enable_pre_inflation_ = this->get_parameter("enable_pre_inflation").as_bool();
  pre_inflation_radius_ = this->get_parameter("pre_inflation_radius").as_double();
  inflation_points_ = this->get_parameter("inflation_points").as_int();
  enable_data_merging_ = this->get_parameter("enable_data_merging").as_bool(); // ❗️ 추가
}

void PathObstacleFilterNode::goalStatusCallback(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
{
  if (msg->status_list.empty()) {
    is_goal_active_ = false;
    return;
  }
  const auto & latest_goal_status = msg->status_list.back();
  int status = latest_goal_status.status;
  if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
      status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
  {
    is_goal_active_ = true;
  } else {
    is_goal_active_ = false;
  }
}

// ... pathCallback, scanCallback, pointcloudCallback, getRobotPose, pointToPathSegmentDistance 함수는 이전과 동일 ...
// ... (이전 답변의 코드들을 그대로 붙여넣기) ...

void PathObstacleFilterNode::process()
{
  if (!is_goal_active_) {
    if (path_received_) {
      RCLCPP_INFO(this->get_logger(), "Goal is not active. Clearing path and publishing empty cloud.");
      std::lock_guard<std::mutex> lock(data_mutex_);
      current_path_.poses.clear();
      path_received_ = false;
      filtered_cloud_pub_->publish(sensor_msgs::msg::PointCloud2());
    }
    return;
  }

  if (!path_received_ || !cloud_received_) { return; }
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!getRobotPose(robot_pose)) { return; }

  nav_msgs::msg::Path path;
  pcl::PointCloud<pcl::PointXYZ> original_cloud;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (current_path_.poses.empty()) { path_received_ = false; return; }
    const auto & goal_pose = current_path_.poses.back().pose;
    double dist_to_goal = std::hypot(
      goal_pose.position.x - robot_pose.pose.position.x,
      goal_pose.position.y - robot_pose.pose.position.y);
    if (dist_to_goal < goal_tolerance_) {
      RCLCPP_INFO(this->get_logger(), "Goal reached. Clearing path.");
      current_path_.poses.clear();
      path_received_ = false;
      filtered_cloud_pub_->publish(sensor_msgs::msg::PointCloud2());
      return;
    }
    path = current_path_;
    original_cloud = latest_cloud_;
  }

  size_t closest_path_idx = 0;
  double min_dist_sq = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    double dist_sq = (path.poses[i].pose.position.x - robot_pose.pose.position.x) * (path.poses[i].pose.position.x - robot_pose.pose.position.x) +
                     (path.poses[i].pose.position.y - robot_pose.pose.position.y) * (path.poses[i].pose.position.y - robot_pose.pose.position.y);
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      closest_path_idx = i;
    }
  }

  pcl::PointCloud<pcl::PointXYZ> filtered_points;
  for (const auto & point : original_cloud.points) {
    double min_dist_to_path = std::numeric_limits<double>::max();
    for (size_t i = closest_path_idx; i < path.poses.size() - 1; ++i) {
      const auto & p1 = path.poses[i].pose.position;
      const auto & p2 = path.poses[i + 1].pose.position;
      double dist_to_segment_start = std::hypot(p1.x - robot_pose.pose.position.x, p1.y - robot_pose.pose.position.y);
      if (dist_to_segment_start > lookahead_dist_) { break; }
      double dist = pointToPathSegmentDistance(point, p1, p2);
      if (dist < min_dist_to_path) { min_dist_to_path = dist; }
    }
    if (min_dist_to_path <= distance_threshold_) {
      filtered_points.points.push_back(point);
    }
  }

  // --- ❗️ 여기가 새롭게 구현된 핵심 로직 ---
  pcl::PointCloud<pcl::PointXYZ> cloud_to_publish;
  
  // 1. 인플레이션이 활성화된 경우, 필터링된 포인트 주변에 가상 포인트를 생성
  pcl::PointCloud<pcl::PointXYZ> inflation_points;
  if (enable_pre_inflation_ && pre_inflation_radius_ > 0.0) {
    if (!filtered_points.points.empty()) {
      inflation_points.points.reserve(filtered_points.points.size() * inflation_points_);
      for (const auto & point : filtered_points.points) {
        for (int i = 0; i < inflation_points_; ++i) {
          double angle = 2.0 * M_PI * i / inflation_points_;
          pcl::PointXYZ inflation_point;
          inflation_point.x = point.x + pre_inflation_radius_ * cos(angle);
          inflation_point.y = point.y + pre_inflation_radius_ * sin(angle);
          inflation_point.z = point.z;
          inflation_points.points.push_back(inflation_point);
        }
      }
    }
  }

  // 2. 데이터 통합 기능 활성화 여부에 따라 최종 발행할 클라우드를 결정
  if (enable_data_merging_) {
    // 통합 기능 ON: 원본 센서 데이터 + 필터링된 포인트 + 인플레이션 포인트
    cloud_to_publish = original_cloud;
    cloud_to_publish += filtered_points;
    cloud_to_publish += inflation_points;
  } else {
    // 통합 기능 OFF: 필터링된 포인트 + 인플레이션 포인트 (기존 방식)
    cloud_to_publish = filtered_points;
    cloud_to_publish += inflation_points;
  }
  
  // 3. 안정적인 방식으로 최종 클라우드 발행
  pcl_conversions::toPCL(this->now(), cloud_to_publish.header.stamp);
  cloud_to_publish.header.frame_id = global_frame_;

  sensor_msgs::msg::PointCloud2 output_msg;
  pcl::toROSMsg(cloud_to_publish, output_msg);
  filtered_cloud_pub_->publish(output_msg);
}

}  // namespace path_obstacle_filter

RCLCPP_COMPONENTS_REGISTER_NODE(path_obstacle_filter::PathObstacleFilterNode)
