#include "path_obstacle_filter/path_obstacle_filter_node.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include "rclcpp_components/register_node_macro.hpp"

namespace path_obstacle_filter
{

PathObstacleFilterNode::PathObstacleFilterNode(const rclcpp::NodeOptions & options)
: Node("path_obstacle_filter", options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing PathObstacleFilterNode");

  getParameters();

  // Initialize TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Setup subscriptions and publisher
  // auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
  // path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
  //   "/plan", default_qos,
  //   std::bind(&PathObstacleFilterNode::pathCallback, this, std::placeholders::_1));



  // scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
  //   "/scan", default_qos,
  //   std::bind(&PathObstacleFilterNode::scanCallback, this, std::placeholders::_1));
  // pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  //   "/pointcloud", default_qos, // Assuming a generic pointcloud topic
  //   std::bind(&PathObstacleFilterNode::pointcloudCallback, this, std::placeholders::_1));

  this->declare_parameter<std::string>("sensor_topic_type", "scan");
  std::string sensor_type = this->get_parameter("sensor_topic_type").as_string();

  auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/plan", default_qos,
      std::bind(&PathObstacleFilterNode::pathCallback, this, std::placeholders::_1));

  // 파라미터 값에 따라 scan 또는 pointcloud 중 하나만 구독
  if (sensor_type == "scan") {
      RCLCPP_INFO(this->get_logger(), "Subscribing to LaserScan topic: /scan");
      scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "/scan", rclcpp::SensorDataQoS(), // SensorDataQoS 사용 권장
          std::bind(&PathObstacleFilterNode::scanCallback, this, std::placeholders::_1));
  } else if (sensor_type == "pointcloud") {
      RCLCPP_INFO(this->get_logger(), "Subscribing to PointCloud2 topic: /pointcloud");
      pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/pointcloud", rclcpp::SensorDataQoS(), // SensorDataQoS 사용 권장
          std::bind(&PathObstacleFilterNode::pointcloudCallback, this, std::placeholders::_1));
  } else {
      RCLCPP_ERROR(this->get_logger(), "Invalid sensor_topic_type: %s", sensor_type.c_str());
  }

    filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/filtered_cloud", default_qos);

  // Main processing timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), // 10 Hz
    std::bind(&PathObstacleFilterNode::process, this));
}

void PathObstacleFilterNode::getParameters()
{
  this->declare_parameter("global_frame", "map");
  this->declare_parameter("robot_base_frame", "base_link");
  this->declare_parameter("distance_threshold", 0.5);
  this->declare_parameter("lookahead_dist", 2.0);
  this->declare_parameter("goal_tolerance", 0.25); // 파라미터 선언 (기본값 0.25m)

  

  global_frame_ = this->get_parameter("global_frame").as_string();
  robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();
  distance_threshold_ = this->get_parameter("distance_threshold").as_double();
  lookahead_dist_ = this->get_parameter("lookahead_dist").as_double();
  goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
}

void PathObstacleFilterNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_path_ = *msg;
  path_received_ = true;
}

void PathObstacleFilterNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  sensor_msgs::msg::PointCloud2 cloud_msg;
  try {
    projector_.transformLaserScanToPointCloud(global_frame_, *msg, cloud_msg, *tf_buffer_);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform scan to %s: %s", global_frame_.c_str(), ex.what());
    return;
  }
  
  std::lock_guard<std::mutex> lock(data_mutex_);
  pcl::fromROSMsg(cloud_msg, latest_cloud_);
  cloud_received_ = true;
}

void PathObstacleFilterNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (msg->header.frame_id != global_frame_) {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(global_frame_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
      sensor_msgs::msg::PointCloud2 transformed_cloud;
      tf2::doTransform(*msg, transformed_cloud, transform);
      std::lock_guard<std::mutex> lock(data_mutex_);
      pcl::fromROSMsg(transformed_cloud, latest_cloud_);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform pointcloud to %s: %s", global_frame_.c_str(), ex.what());
      return;
    }
  } else {
    std::lock_guard<std::mutex> lock(data_mutex_);
    pcl::fromROSMsg(*msg, latest_cloud_);
  }
  cloud_received_ = true;
}


bool PathObstacleFilterNode::getRobotPose(geometry_msgs::msg::PoseStamped & pose)
{
  try {
    geometry_msgs::msg::TransformStamped t;
    t = tf_buffer_->lookupTransform(
      global_frame_, robot_base_frame_, tf2::TimePointZero);
    pose.header = t.header;
    pose.pose.position.x = t.transform.translation.x;
    pose.pose.position.y = t.transform.translation.y;
    pose.pose.position.z = t.transform.translation.z;
    pose.pose.orientation = t.transform.rotation;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get robot pose: %s", ex.what());
    return false;
  }
}

double PathObstacleFilterNode::pointToPathSegmentDistance(
  const pcl::PointXYZ & point,
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2)
{
  double l2 = (p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y);
  if (l2 == 0.0) {
    return std::hypot(point.x - p1.x, point.y - p1.y);
  }
  double t = ((point.x - p1.x) * (p2.x - p1.x) + (point.y - p1.y) * (p2.y - p1.y)) / l2;
  t = std::max(0.0, std::min(1.0, t));
  double proj_x = p1.x + t * (p2.x - p1.x);
  double proj_y = p1.y + t * (p2.y - p1.y);
  return std::hypot(point.x - proj_x, point.y - proj_y);
}

void PathObstacleFilterNode::process()
{
  if (!path_received_ || !cloud_received_) {
    return;
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!getRobotPose(robot_pose)) {
    return;
  }

  nav_msgs::msg::Path path;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // lock_guard를 사용하여 스코프 내에서 데이터 접근을 보호
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 경로가 비어있으면 더 이상 처리할 필요가 없음.
    if (current_path_.poses.empty()) {
      path_received_ = false; // 플래그를 리셋
      return;
    }

    // 로봇과 최종 목표 지점 사이의 거리를 계산
    const auto& goal_pose = current_path_.poses.back().pose;
    double dist_to_goal = std::hypot(
      goal_pose.position.x - robot_pose.pose.position.x,
      goal_pose.position.y - robot_pose.pose.position.y);

    // 거리가 허용 오차보다 가까우면 목표에 도달한 것으로 간주
    if (dist_to_goal < goal_tolerance_) {
      RCLCPP_INFO(this->get_logger(), "Goal reached. Clearing path and stopping filter.");
      current_path_.poses.clear(); // 현재 경로를 비움.
      path_received_ = false;    // 새 경로를 기다리도록 플래그를 리셋

      // 필터링된 장애물이 남아있지 않도록 빈 PointCloud를 발행할 수 있음
      filtered_cloud_pub_->publish(sensor_msgs::msg::PointCloud2());
      return;
    }

    // 목표에 도달하지 않았다면, 필터링을 위해 데이터를 복사합
    path = current_path_;
    cloud = latest_cloud_;
  } // Mutex lock이 여기서 해제.



  // Find the closest path point to the robot
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

  // Filter points
  pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
  filtered_cloud.header = cloud.header;

  for (const auto & point : cloud.points) {
    double min_dist_to_path = std::numeric_limits<double>::max();

    // Check against path segments ahead of the robot
    for (size_t i = closest_path_idx; i < path.poses.size() - 1; ++i) {
      const auto & p1 = path.poses[i].pose.position;
      const auto & p2 = path.poses[i + 1].pose.position;

      // Only check segments within lookahead distance
      double dist_to_segment_start = std::hypot(p1.x - robot_pose.pose.position.x, p1.y - robot_pose.pose.position.y);
      if (dist_to_segment_start > lookahead_dist_) {
        break; // Path goes away from the robot, no need to check further
      }
      
      double dist = pointToPathSegmentDistance(point, p1, p2);
      if (dist < min_dist_to_path) {
        min_dist_to_path = dist;
      }
    }

    if (min_dist_to_path <= distance_threshold_) {
      filtered_cloud.points.push_back(point);
    }
  }
  
  // Publish the filtered cloud
  sensor_msgs::msg::PointCloud2 output_msg;
  pcl::toROSMsg(filtered_cloud, output_msg);
  output_msg.header.stamp = this->now();
  output_msg.header.frame_id = global_frame_;
  filtered_cloud_pub_->publish(output_msg);
}

}  // namespace path_obstacle_filter

// Register the component
RCLCPP_COMPONENTS_REGISTER_NODE(path_obstacle_filter::PathObstacleFilterNode)