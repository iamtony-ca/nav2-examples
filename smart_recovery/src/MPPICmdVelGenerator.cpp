// MPPICmdVelGenerator.cpp
#include "smart_recovery/MPPICmdVelGenerator.hpp"
#include <tf2/utils.h>
#include <cmath>
#include <angles/angles.h>

MPPICmdVelGenerator::MPPICmdVelGenerator(const rclcpp::NodeOptions & opts)
: Node("mppi_cmdvel_generator", opts), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  declare_parameter("linear_velocity", 0.2);
  declare_parameter("angular_velocity_range", 1.0);
  declare_parameter("angular_velocity_step", 0.2);
  declare_parameter("goal_tolerance", 0.1);
  declare_parameter("min_turning_radius", 0.3);

  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    "/recovery_path", rclcpp::SensorDataQoS(),
    std::bind(&MPPICmdVelGenerator::onPathReceived, this, std::placeholders::_1));

  costmap_sub_ = create_subscription<nav2_msgs::msg::Costmap>(
    "/local_costmap/costmap", rclcpp::SensorDataQoS(),
    std::bind(&MPPICmdVelGenerator::onCostmapReceived, this, std::placeholders::_1));

  cmd_pub_  = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  done_pub_ = create_publisher<std_msgs::msg::Bool>("/recovery_done", 10);

  timer_ = create_wall_timer(std::chrono::milliseconds(100),
            std::bind(&MPPICmdVelGenerator::controlLoop, this));
}

// ------------------- Callbacks ------------------------------
void MPPICmdVelGenerator::onPathReceived(const nav_msgs::msg::Path::SharedPtr msg) { latest_path_ = *msg; }
void MPPICmdVelGenerator::onCostmapReceived(const nav2_msgs::msg::Costmap::SharedPtr msg) { latest_costmap_ = *msg; }

// ------------------- Pose via TF ----------------------------
geometry_msgs::msg::Pose MPPICmdVelGenerator::getCurrentPose()
{
  geometry_msgs::msg::Pose pose;
  try {
    auto tf = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
    pose.position.x = tf.transform.translation.x;
    pose.position.y = tf.transform.translation.y;
    pose.position.z = tf.transform.translation.z;
    pose.orientation = tf.transform.rotation;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF lookup failed: %s", ex.what());
  }
  return pose;
}

// ------------------- Control Loop ---------------------------
void MPPICmdVelGenerator::controlLoop()
{
  if (latest_path_.poses.empty()) return;
  auto curr_pose = getCurrentPose();
  auto goal_pt = latest_path_.poses.back().pose.position;
  double dist_goal = std::hypot(curr_pose.position.x - goal_pt.x, curr_pose.position.y - goal_pt.y);
  if (dist_goal < get_parameter("goal_tolerance").as_double()) {
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    
    std_msgs::msg::Bool done; done.data = true; 
    done_pub_->publish(done);
    latest_path_.poses.clear();  // 이걸로 경로 제거
    return;
  }

  double best_cost = 1e9;
  geometry_msgs::msg::Twist best_cmd;
//   double v  = get_parameter("linear_velocity").as_double();
  double v_fwd = get_parameter("linear_velocity").as_double();
  double wR = get_parameter("angular_velocity_range").as_double();
  double wS = get_parameter("angular_velocity_step").as_double();
  double min_rad = get_parameter("min_turning_radius").as_double();

//   for (double w = -wR; w <= wR; w += wS) {
//     if (std::abs(w) > 1e-3 && std::abs(v / w) < min_rad) continue;
//     auto traj = simulateTrajectory(curr_pose, v, w);
  for (double w = -wR; w <= wR; w += wS) {
        /* ---------- ② 경로 방향에 따라 v 부호 결정 ---------- */
        double v = v_fwd;
        if (!latest_path_.poses.empty()) {
          double yaw = tf2::getYaw(curr_pose.orientation);
          const auto & goal = latest_path_.poses.back().pose.position;
          double dx = goal.x - curr_pose.position.x;
          double dy = goal.y - curr_pose.position.y;
          double path_angle = std::atan2(dy, dx);
          double heading_diff = angles::shortest_angular_distance(yaw, path_angle);
          if (std::fabs(heading_diff) > M_PI_2) v = -v_fwd;  // 후방 목표면 후진
        }
    
        if (std::abs(w) > 1e-3 && std::abs(v / w) < min_rad) continue;
        auto traj = simulateTrajectory(curr_pose, v, w);



    double cost = evaluateTrajectory(traj);
    // if (cost < best_cost) { best_cost = cost; best_cmd.linear.x = v; best_cmd.angular.z = w; }
    if (cost < best_cost) {
          best_cost = cost;
          best_cmd.linear.x = v;                 // ④
          best_cmd.angular.z = w;
        }

  }
  cmd_pub_->publish(best_cmd);
  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "cmd_vel published");
}

// ------------------- Helpers --------------------------------
std::vector<geometry_msgs::msg::Pose> MPPICmdVelGenerator::simulateTrajectory(const geometry_msgs::msg::Pose & start,
                                                                             double v, double w)
{
  std::vector<geometry_msgs::msg::Pose> traj;
  geometry_msgs::msg::Pose pose = start;
  const double dt = 0.1;
  for (int i = 0; i < 10; ++i) {
    double yaw = tf2::getYaw(pose.orientation);
    pose.position.x += v * std::cos(yaw) * dt;
    pose.position.y += v * std::sin(yaw) * dt;
    yaw += w * dt;
    tf2::Quaternion q; q.setRPY(0, 0, yaw); pose.orientation = tf2::toMsg(q);
    traj.push_back(pose);
  }
  return traj;
}

double MPPICmdVelGenerator::evaluateTrajectory(const std::vector<geometry_msgs::msg::Pose> & traj)
{
  double cost = 0.0;
  for (const auto & p : traj) {
    double min_d = 1e6;
    for (const auto & ref : latest_path_.poses) {
      double dx = p.position.x - ref.pose.position.x;
      double dy = p.position.y - ref.pose.position.y;
      min_d = std::min(min_d, std::hypot(dx, dy));
    }
    double obs_cost = 0.0;
    if (!latest_costmap_.data.empty()) {
      int i = (p.position.x - latest_costmap_.metadata.origin.position.x) / latest_costmap_.metadata.resolution;
      int j = (p.position.y - latest_costmap_.metadata.origin.position.y) / latest_costmap_.metadata.resolution;
      if (i>=0 && j>=0 && i<(int)latest_costmap_.metadata.size_x && j<(int)latest_costmap_.metadata.size_y) {
        obs_cost = latest_costmap_.data[j * latest_costmap_.metadata.size_x + i] / 100.0;
      }
    }
    cost += min_d + 2.0*obs_cost;
  }
  return cost;
}

// ------------------- main -----------------------------------
#include <rclcpp/executors.hpp>
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MPPICmdVelGenerator>());
  rclcpp::shutdown();
  return 0;
}
