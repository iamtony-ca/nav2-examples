// smart_recovery_planner.cpp
#include "smart_recovery/smart_recovery_planner.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <cmath>
#include <limits>

using namespace std::chrono_literals;

SmartRecoveryPlanner::SmartRecoveryPlanner(const rclcpp::NodeOptions & opts)
: Node("smart_recovery_planner", opts),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  declare_parameter("planner_search_radius", 0.5);
  get_parameter("planner_search_radius", search_radius_);

  double length{0.35}, width{0.35}, padding{0.0};
  get_parameter_or("robot_length", length, length);
  get_parameter_or("robot_width", width, width);
  get_parameter_or("padding", padding, padding);
  get_parameter_or("allow_reverse", robot_cfg_.allow_reverse, robot_cfg_.allow_reverse);
  get_parameter_or("min_turning_radius", robot_cfg_.min_turning_radius, robot_cfg_.min_turning_radius);

  double L = length + 2 * padding, W = width + 2 * padding;
  robot_cfg_.footprint = {
    {-L * 0.5f, -W * 0.5f}, {L * 0.5f, -W * 0.5f},
    {L * 0.5f, W * 0.5f}, {-L * 0.5f, W * 0.5f}
  };

  srv_ = this->create_service<robot_interfaces::srv::SmartRecoveryPath>(
    "generate_recovery_path",
    std::bind(&SmartRecoveryPlanner::handleService, this, std::placeholders::_1, std::placeholders::_2));

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/local_costmap/costmap", rclcpp::QoS(10),
    std::bind(&SmartRecoveryPlanner::costmapCallback, this, std::placeholders::_1));
}

void SmartRecoveryPlanner::handleService(
  const std::shared_ptr<robot_interfaces::srv::SmartRecoveryPath::Request> req,
  std::shared_ptr<robot_interfaces::srv::SmartRecoveryPath::Response> res)
{
  auto start_pose = getRobotPose();
  if (start_pose.header.stamp.sec == 0) {
    res->success = false;
    res->message = "Failed to get robot pose";
    return;
  }

  auto path = generateBestRecoveryPath(req->goal, start_pose);
  res->path = path;
  res->success = !path.poses.empty();
  res->message = res->success ? "Recovery path generated" : "Failed to generate recovery path";
}

void SmartRecoveryPlanner::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::scoped_lock lk(costmap_mtx_);
  latest_costmap_ = msg;
  // RCLCPP_INFO(get_logger(), "Received costmap. Size: %d x %d", msg->info.width, msg->info.height);
}

geometry_msgs::msg::PoseStamped SmartRecoveryPlanner::getRobotPose()
{
  geometry_msgs::msg::PoseStamped out;
  try {
    auto tf = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero, tf2::durationFromSec(0.2));
    out.header = tf.header;
    out.pose.position.x = tf.transform.translation.x;
    out.pose.position.y = tf.transform.translation.y;
    out.pose.position.z = tf.transform.translation.z;
    out.pose.orientation = tf.transform.rotation;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup failed: %s", ex.what());
    return geometry_msgs::msg::PoseStamped{};
  }
  return out;
}

// bool SmartRecoveryPlanner::isPoseTraversable(const geometry_msgs::msg::Pose & pose) const
// {
//   std::scoped_lock lk(costmap_mtx_);
//   if (!latest_costmap_) {
//     RCLCPP_WARN(get_logger(), "Costmap not available");
//     return false;
//   }
//   double yaw = tf2::getYaw(pose.orientation);
//   double cs = std::cos(yaw), sn = std::sin(yaw);
//   const auto & meta = latest_costmap_->info;
//   int size_x = meta.width, size_y = meta.height;
//   constexpr uint8_t LETHAL = 254;

//   auto toWorld = [&](const Eigen::Vector2f& p_local) {
//     double wx = pose.position.x + p_local.x() * cs - p_local.y() * sn;
//     double wy = pose.position.y + p_local.x() * sn + p_local.y() * cs;
//     return std::make_pair(wx, wy);
//   };

//   for (const auto & p_local : robot_cfg_.footprint) {
//     auto [wx, wy] = toWorld(p_local);
//     int mx = (wx - meta.origin.position.x) / meta.resolution;
//     int my = (wy - meta.origin.position.y) / meta.resolution;
//     if (mx < 0 || my < 0 || mx >= size_x || my >= size_y) return false;
//     if (latest_costmap_->data[my * size_x + mx] >= LETHAL) return false;
//   }
//   return true;
// }

// bool SmartRecoveryPlanner::isPoseTraversable(const geometry_msgs::msg::Pose & pose, bool relaxed)
bool SmartRecoveryPlanner::isPoseTraversable(const geometry_msgs::msg::Pose & pose)
{
  std::scoped_lock lk(costmap_mtx_);
  if (!latest_costmap_) return false;

  double yaw = tf2::getYaw(pose.orientation);
  double cs = std::cos(yaw), sn = std::sin(yaw);
  const auto & meta = latest_costmap_->info;
  int lethal_count = 0;
  int total = 0;

  for (const auto & p_local : robot_cfg_.footprint) {
    double wx = pose.position.x + p_local.x() * cs - p_local.y() * sn;
    double wy = pose.position.y + p_local.x() * sn + p_local.y() * cs;
    int mx = (wx - meta.origin.position.x) / meta.resolution;
    int my = (wy - meta.origin.position.y) / meta.resolution;
    if (mx < 0 || my < 0 || mx >= (int)meta.width || my >= (int)meta.height)
      continue;
    uint8_t cost = latest_costmap_->data[my * meta.width + mx];
    if (cost >= 254) ++lethal_count;
    ++total;
  }

  bool relaxed = true;
  if (!relaxed) return lethal_count == 0;
  if (total == 0) return false;
  float ratio = static_cast<float>(lethal_count) / total;
  return ratio < 0.9f;  // "lethal cell 비율이 90% 미만이면 traversable"
}


nav_msgs::msg::Path SmartRecoveryPlanner::createCandidatePath(float angle_rad, float dist_m, const geometry_msgs::msg::PoseStamped & start)
{
  nav_msgs::msg::Path path;
  path.header = start.header;

  if (!isPoseTraversable(start.pose)) {
    RCLCPP_WARN(get_logger(), "Start pose not traversable");
    // return path;
  }

  


  double yaw0 = tf2::getYaw(start.pose.orientation);
  for (double d = 0.0; d <= dist_m + 1e-3; d += 0.05) {
    geometry_msgs::msg::PoseStamped p = start;
    p.pose.position.x += d * std::cos(yaw0 + angle_rad);
    p.pose.position.y += d * std::sin(yaw0 + angle_rad);
    if (!isPoseTraversable(p.pose)) break;
    path.poses.emplace_back(std::move(p));
  }
  if (path.poses.size() < 4) return nav_msgs::msg::Path();  // 최소 20cm 정도 필요
  return path;
}

nav_msgs::msg::Path SmartRecoveryPlanner::generateBestRecoveryPath(const geometry_msgs::msg::PoseStamped & goal, const geometry_msgs::msg::PoseStamped & start)
{
  nav_msgs::msg::Path best;
  float best_score = -std::numeric_limits<float>::infinity();
  const double step = M_PI / 18;
  for (double a = -M_PI; a <= M_PI + 1e-3; a += step) {
    if (std::fabs(a) > M_PI_2 && !robot_cfg_.allow_reverse) continue;
    auto cand = createCandidatePath(a, search_radius_, start);
    if (cand.poses.empty()) continue;
    float s = evaluatePath(cand, nav_msgs::msg::Path(), goal.pose);
    if (s > best_score) {
      best_score = s;
      best = std::move(cand);
    }
  }
  return best;
}

float SmartRecoveryPlanner::computeCostScore(const nav_msgs::msg::Path & p)
{
  constexpr uint8_t LETHAL = 254;
  std::scoped_lock lk(costmap_mtx_);
  if (!latest_costmap_ || p.poses.size() < 3) return 0.0f;  // too short path

  const auto & m = latest_costmap_->info;
  int sx = m.width, sy = m.height;
  float total_cost = 0.0f;

  for (const auto & ps : p.poses) {
    int i = (ps.pose.position.x - m.origin.position.x) / m.resolution;
    int j = (ps.pose.position.y - m.origin.position.y) / m.resolution;
    if (i < 0 || j < 0 || i >= sx || j >= sy) return 0.0f;  // OOB
    uint8_t c = static_cast<uint8_t>(latest_costmap_->data[j * sx + i]);
    if (c >= LETHAL) return 0.0f;  // lethal cell hit
    total_cost += static_cast<float>(c);
  }

  // 마지막 포즈의 인근 상태도 확인해 충돌 직전인지 확인
  const auto & last = p.poses.back().pose.position;
  int li = (last.x - m.origin.position.x) / m.resolution;
  int lj = (last.y - m.origin.position.y) / m.resolution;
  if (li < 0 || lj < 0 || li >= sx || lj >= sy) return 0.0f;
  uint8_t last_cost = static_cast<uint8_t>(latest_costmap_->data[lj * sx + li]);
  if (last_cost >= LETHAL - 1) return 0.0f;

  float avg_cost = total_cost / p.poses.size();
  return 1.f - avg_cost / 100.f;  // normalize: 1.0 = free, 0.0 = worst avg
}


float SmartRecoveryPlanner::computeClearanceScore(const nav_msgs::msg::Path & path)
{
  uint8_t min_cost = 100;
  std::lock_guard<std::mutex> lock(costmap_mtx_);
  if (!latest_costmap_) return 0.0;

  const auto & info = latest_costmap_->info;
  for (const auto & pose_stamped : path.poses) {
    const auto & pt = pose_stamped.pose.position;
    int i = (pt.x - info.origin.position.x) / info.resolution;
    int j = (pt.y - info.origin.position.y) / info.resolution;
    if (i < 0 || j < 0 || i >= static_cast<int>(info.width) || j >= static_cast<int>(info.height)) continue;
    int index = j * info.width + i;
    uint8_t cost = static_cast<uint8_t>(latest_costmap_->data[index]);
    min_cost = std::min<uint8_t>(min_cost, cost);
  }
  return (100.0f - min_cost) / 100.0f;  // 낮은 cost일수록 좋은 점수
}



float SmartRecoveryPlanner::computeLengthPenalty(const nav_msgs::msg::Path & path)
{
  if (path.poses.size() < 2) return 1.0f;  // 극단적 감점
  float length = 0.0f;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & p1 = path.poses[i-1].pose.position;
    const auto & p2 = path.poses[i].pose.position;
    length += std::hypot(p2.x - p1.x, p2.y - p1.y);
  }
  return length < 0.3 ? (1.0f - length / 0.3f) : 0.0f;  // 0.3m 이하면 감점 비율 적용
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
float min_clearance     = computeMinClearance(path);
float length_penalty     = computeLengthPenalty(path);


  // return 0.25 * cost_score
  //      + 0.15 * goal_align_score
  //      + 0.20 * lookahead_score
  //      + 0.10 * distance_gain
  //      + 0.10 * clearance_score
  //      - 0.10 * curvature_penalty
  //      - 0.10 * smoothness_penalty;
  return 0.5f * cost_score          // 안전 비중 40 %
  + 0.15f * clearance_score     // +20 %
  + 0.0f * goal_align_score
  + 0.0f * lookahead_score
  + 0.0f * distance_gain
  - 0.0f * curvature_penalty
  - 0.0f * smoothness_penalty;
  + 0.15f * min_clearance;  // 최소 clearance 점수 추가
  - 0.2f * length_penalty;  // 길이 페널티는 나중에 추가 가능
}

float SmartRecoveryPlanner::computeMinClearance(const nav_msgs::msg::Path & path)
{
  uint8_t min_cost = 255;
  std::scoped_lock lk(costmap_mtx_);
  if (!latest_costmap_) return 0.0f;

  const auto & info = latest_costmap_->info;
  for (const auto & pose_stamped : path.poses) {
    RCLCPP_INFO(get_logger(), "Path point: (%.2f, %.2f)", pose_stamped.pose.position.x, pose_stamped.pose.position.y);

    const auto & pt = pose_stamped.pose.position;
    int i = (pt.x - info.origin.position.x) / info.resolution;
    int j = (pt.y - info.origin.position.y) / info.resolution;
    if (i < 0 || j < 0 || i >= static_cast<int>(info.width) || j >= static_cast<int>(info.height))
      continue;
    int index = j * info.width + i;
    uint8_t cost = static_cast<uint8_t>(latest_costmap_->data[index]);
    min_cost = std::min<uint8_t>(min_cost, cost);
    RCLCPP_INFO(get_logger(), " → Costmap cost: %d", latest_costmap_->data[index]);
  }
  return (100.0f - min_cost) / 100.0f;  // 낮을수록 penalty 부여 가능
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
return (min_radius < robot_cfg_.min_turning_radius) ? 1.0 : 0.0;
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





// ---------------- main --------------------------------------
#include <rclcpp/executors.hpp>
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<SmartRecoveryPlanner>());rclcpp::shutdown();return 0;}
