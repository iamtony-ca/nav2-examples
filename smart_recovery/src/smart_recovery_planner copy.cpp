#include "smart_recovery/smart_recovery_planner.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <eigen3/Eigen/Dense>
#include <cmath>

SmartRecoveryPlanner::SmartRecoveryPlanner()
: Node("smart_recovery_planner"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  declare_parameter("planner_search_radius", 1.0);
  costmap_sub_ = create_subscription<nav2_msgs::msg::Costmap>(
    "/local_costmap/costmap", 10,
    std::bind(&SmartRecoveryPlanner::costmapCallback, this, std::placeholders::_1));

  srv_ = create_service<robot_interfaces::srv::SmartRecoveryPath>(
    "generate_recovery_path",
    std::bind(&SmartRecoveryPlanner::handleService, this, std::placeholders::_1, std::placeholders::_2));

  robot_config_.min_turning_radius = 0.4;
  robot_config_.allow_reverse = true;
}

void SmartRecoveryPlanner::handleService(
  const std::shared_ptr<robot_interfaces::srv::SmartRecoveryPath::Request> req,
  std::shared_ptr<robot_interfaces::srv::SmartRecoveryPath::Response> res)
{
  auto path = generateBestRecoveryPath(req->goal);
  res->path = path;
  if (path.poses.empty()) {
    res->success = false;
    res->message = "Failed to generate recovery path";
  } else {
    res->success = true;
    res->message = "Recovery path generated";
  }
}

// ---------- remaining definitions (costmapCallback, getRobotPose, createCandidatePath, evaluatePath, etc.)
// (Assume previously implemented functions are copied here unchanged)

// #include "smart_recovery_planner.hpp"
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <eigen3/Eigen/Dense>
// #include <cmath>


void SmartRecoveryPlanner::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  latest_costmap_ = msg;
}

// geometry_msgs::msg::PoseStamped SmartRecoveryPlanner::getRobotPose()
// {
//   geometry_msgs::msg::PoseStamped base_pose, map_pose;
//   base_pose.header.frame_id = "base_link";
//   base_pose.header.stamp = this->now();
//   base_pose.pose.orientation.w = 1.0;

//   try {
//     tf_buffer_.transform(base_pose, map_pose, "map", tf2::durationFromSec(0.5));
//   } catch (tf2::TransformException &ex) {
//     RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
//   }
//   return map_pose;
// }

geometry_msgs::msg::PoseStamped SmartRecoveryPlanner::getRobotPose()
{
  geometry_msgs::msg::PoseStamped map_pose;
  map_pose.header.frame_id = "map";

  try
  {
    /*  가장 최신 TF를 직접 가져온다 (시간=0) */
    auto tf = tf_buffer_.lookupTransform(
        "map", "base_link",
        tf2::TimePointZero,                    // ← time=0
        tf2::durationFromSec(0.1));            // timeout

    map_pose.header.stamp     = tf.header.stamp;      // TF 시간값 복사
    map_pose.pose.position.x  = tf.transform.translation.x;
    map_pose.pose.position.y  = tf.transform.translation.y;
    map_pose.pose.position.z  = tf.transform.translation.z;
    map_pose.pose.orientation = tf.transform.rotation;
  }
  catch (tf2::TransformException & ex)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "TF lookup failed: %s", ex.what());
  }
  return map_pose;
}



nav_msgs::msg::Path SmartRecoveryPlanner::createCandidatePath(float angle, float distance)
{
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped start_pose = getRobotPose();
  path.header.frame_id = "map";
  path.header.stamp = start_pose.header.stamp;   // TF 시간 그대로
//   path.header.stamp = this->now();

//   geometry_msgs::msg::PoseStamped start_pose = getRobotPose();
  double yaw = tf2::getYaw(start_pose.pose.orientation);

  for (float d = 0.0; d <= distance; d += 0.1) {
    geometry_msgs::msg::PoseStamped pose = start_pose;
    pose.pose.position.x += d * std::cos(yaw + angle);
    pose.pose.position.y += d * std::sin(yaw + angle);
    path.poses.push_back(pose);
  }
  return path;
}

nav_msgs::msg::Path SmartRecoveryPlanner::generateBestRecoveryPath(const geometry_msgs::msg::PoseStamped &goal)
{
  nav_msgs::msg::Path best_path;
  float best_score = -std::numeric_limits<float>::max();
  nav_msgs::msg::Path global_path; // TODO: inject actual global path if needed

  for (float angle = -M_PI_2; angle <= M_PI_2; angle += M_PI / 12.0) {
    nav_msgs::msg::Path candidate = createCandidatePath(angle, 1.0);
    float score = evaluatePath(candidate, global_path, goal.pose);
    if (score > best_score) {
      best_score = score;
      best_path = candidate;
    }
  }
  return best_path;
}


float SmartRecoveryPlanner::evaluatePath(const nav_msgs::msg::Path &path,
                                         const nav_msgs::msg::Path &global_path,
                                         const geometry_msgs::msg::Pose &goal)
{
  float cost_score         = computeCostScore(path);
  float goal_align_score   = computeAlignmentScore(path, goal);
  float lookahead_score    = computeLookaheadAlignment(path, global_path, 1.0);
  float curvature_penalty  = computeCurvaturePenalty(path);
  float smoothness_penalty = computeSmoothnessPenalty(path);
  float distance_gain      = computeDistanceImprovement(path, goal);
  float clearance_score    = computeClearanceScore(path);

//   return 0.25 * cost_score
//        + 0.15 * goal_align_score
//        + 0.20 * lookahead_score
//        + 0.10 * distance_gain
//        + 0.10 * clearance_score
//        - 0.10 * curvature_penalty
//        - 0.10 * smoothness_penalty;
  return 0.8f * cost_score          // 안전 비중 40 %
       + 0.2f * clearance_score     // +20 %
       + 0.0f * goal_align_score
       + 0.0f * lookahead_score
       + 0.0f * distance_gain
       - 0.0f * curvature_penalty
       - 0.0f * smoothness_penalty;
}

float SmartRecoveryPlanner::computeAlignmentScore(const nav_msgs::msg::Path &path,
                                                  const geometry_msgs::msg::Pose &goal)
{
  if (path.poses.size() < 2) return 0.0f;
  const auto &start = path.poses.front().pose.position;
  const auto &end   = path.poses.back().pose.position;

  Eigen::Vector2f path_dir(end.x - start.x, end.y - start.y);
  Eigen::Vector2f goal_dir(goal.position.x - start.x, goal.position.y - start.y);
  path_dir.normalize(); goal_dir.normalize();

  return path_dir.dot(goal_dir);
}

float SmartRecoveryPlanner::computeLookaheadAlignment(const nav_msgs::msg::Path &candidate,
                                                      const nav_msgs::msg::Path &global_path,
                                                      float lookahead_dist)
{
  if (global_path.poses.empty()) return 0.0f;
  const auto &start = candidate.poses.front().pose.position;
  geometry_msgs::msg::Point lookahead_point;

  float dist = 0.0;
  for (size_t i = 1; i < global_path.poses.size(); ++i) {
    const auto &prev = global_path.poses[i-1].pose.position;
    const auto &curr = global_path.poses[i].pose.position;
    dist += std::hypot(curr.x - prev.x, curr.y - prev.y);
    if (dist >= lookahead_dist) {
      lookahead_point = curr;
      break;
    }
  }
  if (dist < lookahead_dist) return 0.0f;

  Eigen::Vector2f path_dir(
    candidate.poses.back().pose.position.x - start.x,
    candidate.poses.back().pose.position.y - start.y);
  Eigen::Vector2f lookahead_dir(
    lookahead_point.x - start.x,
    lookahead_point.y - start.y);
  path_dir.normalize(); lookahead_dir.normalize();

  return path_dir.dot(lookahead_dir);
}

float SmartRecoveryPlanner::computeCurvaturePenalty(const nav_msgs::msg::Path &path)
{
  float min_radius = std::numeric_limits<float>::max();
  for (size_t i = 1; i + 1 < path.poses.size(); ++i) {
    const auto &p1 = path.poses[i-1].pose.position;
    const auto &p2 = path.poses[i].pose.position;
    const auto &p3 = path.poses[i+1].pose.position;

    float a = std::hypot(p1.x - p2.x, p1.y - p2.y);
    float b = std::hypot(p2.x - p3.x, p2.y - p3.y);
    float c = std::hypot(p3.x - p1.x, p3.y - p1.y);
    float s = (a + b + c) / 2.0;
    float area = std::sqrt(s * (s - a) * (s - b) * (s - c));

    float radius = (a * b * c) / (4.0 * area + 1e-5);
    min_radius = std::min(min_radius, radius);
  }
  return (min_radius < robot_config_.min_turning_radius) ? 1.0 : 0.0;
}

float SmartRecoveryPlanner::computeSmoothnessPenalty(const nav_msgs::msg::Path &path)
{
  float total_angle = 0.0;
  for (size_t i = 2; i < path.poses.size(); ++i) {
    auto v1 = Eigen::Vector2f(
      path.poses[i-1].pose.position.x - path.poses[i-2].pose.position.x,
      path.poses[i-1].pose.position.y - path.poses[i-2].pose.position.y).normalized();
    auto v2 = Eigen::Vector2f(
      path.poses[i].pose.position.x - path.poses[i-1].pose.position.x,
      path.poses[i].pose.position.y - path.poses[i-1].pose.position.y).normalized();
    total_angle += std::acos(std::clamp(v1.dot(v2), -1.0f, 1.0f));
  }
  return total_angle / M_PI;  // normalize to 0~1
}

float SmartRecoveryPlanner::computeDistanceImprovement(const nav_msgs::msg::Path &path,
                                                       const geometry_msgs::msg::Pose &goal)
{
  const auto &start = path.poses.front().pose.position;
  const auto &end   = path.poses.back().pose.position;
  float before = std::hypot(goal.position.x - start.x, goal.position.y - start.y);
  float after  = std::hypot(goal.position.x - end.x, goal.position.y - end.y);
  return (before - after) / std::max(before, 1e-3f);
}

float SmartRecoveryPlanner::computeClearanceScore(const nav_msgs::msg::Path &path)
{
  // Simple version: distance from costmap origin boundaries
//   float min_clearance = 1000.0f;
    uint8_t max_cost = 0;
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  if (!latest_costmap_) return 0.0;

  for (const auto &pose_stamped : path.poses) {
    const auto &pt = pose_stamped.pose.position;
    float dx = pt.x - latest_costmap_->metadata.origin.position.x;
    float dy = pt.y - latest_costmap_->metadata.origin.position.y;
    int i = dx / latest_costmap_->metadata.resolution;
    int j = dy / latest_costmap_->metadata.resolution;
    if (i < 0 || j < 0 || i >= (int)latest_costmap_->metadata.size_x || j >= (int)latest_costmap_->metadata.size_y)
      continue;
    int index = j * latest_costmap_->metadata.size_x + i;

    // uint8_t cost = latest_costmap_->data[index];
    // min_clearance = std::min(min_clearance, static_cast<float>(cost));
    max_cost = std::max(max_cost, latest_costmap_->data[index]);
  }
//   return 1.0f - (min_clearance / 100.0f);  // normalize: 1.0 = max clearance
  return (100.0f - max_cost) / 100.0f;   // cost 높으면 점수 ↓
}

float SmartRecoveryPlanner::computeCostScore(const nav_msgs::msg::Path &path)
{
//   float total_cost = 0.0f;
//   int count = 0;
  constexpr uint8_t LETHAL = 15;  //253
  uint8_t worst = 0;                       // 경로에서 가장 높은 cost
  int count = 0;

  std::lock_guard<std::mutex> lock(costmap_mutex_);
//   if (!latest_costmap_) return 1.0f;
  if (!latest_costmap_) return 0.0f;

  for (const auto &pose_stamped : path.poses) {
    const auto &pt = pose_stamped.pose.position;
    float dx = pt.x - latest_costmap_->metadata.origin.position.x;
    float dy = pt.y - latest_costmap_->metadata.origin.position.y;
    int i = dx / latest_costmap_->metadata.resolution;
    int j = dy / latest_costmap_->metadata.resolution;
    if (i < 0 || j < 0 || i >= (int)latest_costmap_->metadata.size_x || j >= (int)latest_costmap_->metadata.size_y)
      continue;
    // int index = j * latest_costmap_->metadata.size_x + i;
    // // total_cost += latest_costmap_->data[index];
    // // count++;
    // uint8_t c = latest_costmap_->data[index];
    // if (c >= LETHAL) return 0.0f;          // lethal → 즉시 0점
    // worst = std::max(worst, c);
    // ++count;
    uint8_t c = latest_costmap_->data[j * latest_costmap_->metadata.size_x + i];  // ← idx → j*size_x+i
    if (c >= LETHAL) return 0.0f;        // lethal 있으면 즉시 탈락
    worst = std::max(worst, c);          // 최악값 추적

  }
  // return count ? (1.0f - (worst / 100.0f)) : 0.0f;   // worst-cell 기반
//   return count > 0 ? (1.0f - (total_cost / (count * 100.0f))) : 0.0f;
  return 1.0f - (worst / 100.0f);        // 최악 셀 기준 0~1
}

// ------------------- main -----------------------------------
#include <rclcpp/executors.hpp>
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SmartRecoveryPlanner>());
  rclcpp::shutdown();
  return 0;
}
