#include "amr_bt_nodes/compute_validated_path.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace amr_bt_nodes
{

ComputeValidatedPath::ComputeValidatedPath(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(xml_tag_name, conf),
  initialized_(false),
  static_done_(false),
  dynamic_done_(false),
  static_error_code_(0),
  dynamic_error_code_(0)
{
}

void ComputeValidatedPath::initialize()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
    throw std::runtime_error("Failed to get 'node' from blackboard in ComputeValidatedPath");
  }

  logger_ = node_->get_logger();

  std::string static_server, dynamic_server;
  getInput("static_planner_server", static_server);
  getInput("dynamic_planner_server", dynamic_server);

  // [핵심 추가] 메인 스레드와 분리된 전용 콜백 그룹 생성
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  // 콜백 그룹을 할당하여 Action Client 생성
  static_client_ = rclcpp_action::create_client<ActionType>(node_, static_server, callback_group_);
  dynamic_client_ = rclcpp_action::create_client<ActionType>(node_, dynamic_server, callback_group_);

  if (!static_client_->wait_for_action_server(std::chrono::seconds(2)) ||
      !dynamic_client_->wait_for_action_server(std::chrono::seconds(2))) {
    throw std::runtime_error("Planner action servers not available in ComputeValidatedPath");
  }

  initialized_ = true;
  RCLCPP_INFO(logger_, "[ComputeValidatedPath] Initialized successfully. Connected to action servers.");
}

BT::PortsList ComputeValidatedPath::providedPorts()
{
  return {
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("global_goals", "Destinations to plan through"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("current_pose", "Current robot pose"),
    BT::InputPort<std::string>("static_planner_server", "/static_nav/compute_path_through_poses", "Static server action name"),
    BT::InputPort<std::string>("dynamic_planner_server", "compute_path_through_poses", "Dynamic server action name"),
    BT::InputPort<std::string>("static_planner_id", "StaticGridBased", "Mapped name to the static planner plugin"),
    BT::InputPort<std::string>("dynamic_planner_id", "GridBased", "Mapped name to the dynamic planner plugin"),
    BT::InputPort<double>("horizon_distance", 10.0, "Sensor range horizon (m)"),
    BT::InputPort<double>("max_deviation", 2.0, "Max allowed deviation (m)"),
    BT::InputPort<double>("step_distance", 0.3, "Distance interval to check deviation (m)"),
    BT::InputPort<double>("max_check_length", 20.0, "Max accumulated length to check (m)"),
    BT::OutputPort<nav_msgs::msg::Path>("validated_path", "Final validated path for controller"),
    BT::OutputPort<nav_msgs::msg::Path>("static_path", "Reference path from static planner (for debug/viz)"),
    BT::OutputPort<uint16_t>("static_error_code_id", "Error code from static planner"),
    BT::OutputPort<uint16_t>("dynamic_error_code_id", "Error code from dynamic planner"),
    BT::OutputPort<uint16_t>("validation_error_code_id", "Error code for deviation validation (0: Success, 308: Detour)")
  };
}

BT::NodeStatus ComputeValidatedPath::onStart()
{
  if (!initialized_) {
    initialize();
  }

  std::vector<geometry_msgs::msg::PoseStamped> global_goals;
  geometry_msgs::msg::PoseStamped current_pose;
  std::string static_planner_id, dynamic_planner_id;

  if (!getInput("global_goals", global_goals) || global_goals.empty() || 
      !getInput("current_pose", current_pose)) {
    RCLCPP_ERROR(logger_, "[ComputeValidatedPath] Missing required inputs.");
    return BT::NodeStatus::FAILURE;
  }

  getInput("static_planner_id", static_planner_id);
  getInput("dynamic_planner_id", dynamic_planner_id);
  getInput("horizon_distance", horizon_);
  getInput("max_deviation", max_dev_);
  getInput("step_distance", step_dist_);
  getInput("max_check_length", max_check_length_);

  int max_valid_idx = -1;
  const auto & robot_pos = current_pose.pose.position;
  for (int i = static_cast<int>(global_goals.size()) - 1; i >= 0; --i) {
    double dist = std::hypot(global_goals[i].pose.position.x - robot_pos.x,
                             global_goals[i].pose.position.y - robot_pos.y);
    if (dist <= horizon_) {
      max_valid_idx = i;
      break;
    }
  }
  if (max_valid_idx == -1) max_valid_idx = 0;
  
  local_goals_ = std::vector<geometry_msgs::msg::PoseStamped>(
    global_goals.begin(), global_goals.begin() + max_valid_idx + 1);

  static_done_ = false;
  dynamic_done_ = false;
  static_error_code_ = 0;
  dynamic_error_code_ = 0;
  static_goal_handle_.reset();
  dynamic_goal_handle_.reset();
  static_path_ = nav_msgs::msg::Path();
  actual_path_ = nav_msgs::msg::Path();

  auto static_goal_msg = ActionType::Goal();
  static_goal_msg.goals = local_goals_;
  static_goal_msg.planner_id = static_planner_id;
  static_goal_msg.use_start = false;

  auto dynamic_goal_msg = ActionType::Goal();
  dynamic_goal_msg.goals = global_goals;
  dynamic_goal_msg.planner_id = dynamic_planner_id;
  dynamic_goal_msg.use_start = false;

  auto send_opts_static = rclcpp_action::Client<ActionType>::SendGoalOptions();
  send_opts_static.goal_response_callback = [this](auto handle) { static_goal_handle_ = handle; };
  send_opts_static.result_callback = [this](const GoalHandle::WrappedResult & result) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      static_path_ = result.result->path;
      static_error_code_ = 0;
    } else {
      static_error_code_ = result.result ? result.result->error_code : 308;
    }
    static_done_ = true;
  };

  auto send_opts_dynamic = rclcpp_action::Client<ActionType>::SendGoalOptions();
  send_opts_dynamic.goal_response_callback = [this](auto handle) { dynamic_goal_handle_ = handle; };
  send_opts_dynamic.result_callback = [this](const GoalHandle::WrappedResult & result) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      actual_path_ = result.result->path;
      dynamic_error_code_ = 0;
    } else {
      dynamic_error_code_ = result.result ? result.result->error_code : 308; 
    }
    dynamic_done_ = true;
  };

  RCLCPP_DEBUG(logger_, "[ComputeValidatedPath] Sending async goals to both planners.");
  static_client_->async_send_goal(static_goal_msg, send_opts_static);
  dynamic_client_->async_send_goal(dynamic_goal_msg, send_opts_dynamic);

  // [핵심 추가] 비동기 네트워크 요청을 즉시 처리하도록 강제 스핀
  callback_group_executor_.spin_some();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ComputeValidatedPath::onRunning()
{
  // [핵심 추가] 멈춰있던 콜백(서버 응답)들을 처리하기 위해 스핀을 돌림
  callback_group_executor_.spin_some();

  if (!static_done_ || !dynamic_done_) {
    return BT::NodeStatus::RUNNING;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  setOutput("static_error_code_id", static_cast<uint16_t>(static_error_code_.load()));
  setOutput("dynamic_error_code_id", static_cast<uint16_t>(dynamic_error_code_.load()));

  if (!static_path_.poses.empty()) {
    setOutput("static_path", static_path_);
  } else {
    setOutput("static_path", nav_msgs::msg::Path());
  }

  if (static_error_code_ != 0 || dynamic_error_code_ != 0) {
    nav_msgs::msg::Path empty_path;
    setOutput("validated_path", empty_path);
    setOutput("validation_error_code_id", static_cast<uint16_t>(308)); 
    RCLCPP_WARN(logger_, "[ComputeValidatedPath] Planner action failed. Static Err: %d, Dynamic Err: %d", 
                static_error_code_.load(), dynamic_error_code_.load());
    return BT::NodeStatus::FAILURE;
  }

  if (performValidation()) {
    setOutput("validated_path", actual_path_); 
    setOutput("validation_error_code_id", static_cast<uint16_t>(0));
    RCLCPP_DEBUG(logger_, "[ComputeValidatedPath] Path validation passed.");
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_WARN(logger_, "[ComputeValidatedPath] Detour deviation > %.2f detected!", max_dev_);
    nav_msgs::msg::Path empty_path;
    setOutput("validated_path", empty_path);
    setOutput("validation_error_code_id", static_cast<uint16_t>(308)); 
    return BT::NodeStatus::FAILURE;
  }
}

void ComputeValidatedPath::onHalted()
{
  if (!static_done_ && static_goal_handle_) {
    static_client_->async_cancel_goal(static_goal_handle_);
  }
  if (!dynamic_done_ && dynamic_goal_handle_) {
    dynamic_client_->async_cancel_goal(dynamic_goal_handle_);
  }
  RCLCPP_INFO(logger_, "[ComputeValidatedPath] Node halted. Action goals cancelled.");
}

bool ComputeValidatedPath::performValidation()
{
  if (actual_path_.poses.empty() || static_path_.poses.empty()) return true;

  nav_msgs::msg::Path trunc_ref = static_path_;
  nav_msgs::msg::Path trunc_actual = actual_path_;
  auto start_pos = actual_path_.poses.front().pose.position;

  if (local_goals_.size() == 1) {
    truncatePathByEuclidean(trunc_ref, start_pos, horizon_);
    truncatePathByEuclidean(trunc_actual, start_pos, horizon_);
  } else {
    truncatePathToGoal(trunc_actual, local_goals_.back().pose.position);
  }

  if (trunc_actual.poses.empty() || trunc_ref.poses.empty()) return true;

  double accumulated_length = 0.0;          
  double dist_since_last = 0.0;       
  auto prev_pos = trunc_actual.poses[0].pose.position;

  if (!isPoseWithinDeviation(prev_pos, trunc_ref, max_dev_)) return false;

  for (size_t i = 1; i < trunc_actual.poses.size(); ++i) {
    const auto & curr_pos = trunc_actual.poses[i].pose.position;
    double step = std::hypot(curr_pos.x - prev_pos.x, curr_pos.y - prev_pos.y);
    accumulated_length += step;
    dist_since_last += step;

    if (accumulated_length > max_check_length_) break;

    if (dist_since_last >= step_dist_ || i == trunc_actual.poses.size() - 1) {
      if (!isPoseWithinDeviation(curr_pos, trunc_ref, max_dev_)) {
        return false; 
      }
      dist_since_last = 0.0;
    }
    prev_pos = curr_pos; 
  }
  return true;
}

void ComputeValidatedPath::truncatePathByEuclidean(nav_msgs::msg::Path & path, const geometry_msgs::msg::Point & start, double dist_limit) {
  if (path.poses.empty()) return;
  auto it = std::find_if(path.poses.rbegin(), path.poses.rend(),
    [&start, dist_limit](const geometry_msgs::msg::PoseStamped & p) {
      return std::hypot(p.pose.position.x - start.x, p.pose.position.y - start.y) <= dist_limit;
    });
  if (it != path.poses.rend()) path.poses.erase(it.base(), path.poses.end());
}

void ComputeValidatedPath::truncatePathToGoal(nav_msgs::msg::Path & path, const geometry_msgs::msg::Point & target) {
  if (path.poses.empty()) return;
  size_t closest_idx = 0; double min_dist = 1e9;
  for (size_t i = 0; i < path.poses.size(); ++i) {
    double dist = std::hypot(path.poses[i].pose.position.x - target.x, path.poses[i].pose.position.y - target.y);
    if (dist < min_dist) { min_dist = dist; closest_idx = i; }
  }
  path.poses.resize(closest_idx + 1);
}

bool ComputeValidatedPath::isPoseWithinDeviation(const geometry_msgs::msg::Point & p, const nav_msgs::msg::Path & ref_path, double max_dev) {
  if (ref_path.poses.empty()) return false;
  if (ref_path.poses.size() == 1) return std::hypot(p.x - ref_path.poses[0].pose.position.x, p.y - ref_path.poses[0].pose.position.y) <= max_dev;
  double min_dist = 1e9;
  for (size_t j = 1; j < ref_path.poses.size(); ++j) {
    double dist = pointToLineSegmentDistance(p, ref_path.poses[j-1].pose.position, ref_path.poses[j].pose.position);
    min_dist = std::min(min_dist, dist);
  }
  return min_dist <= max_dev;
}

double ComputeValidatedPath::pointToLineSegmentDistance(const geometry_msgs::msg::Point & p, const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b) {
  double ab_x = b.x - a.x; double ab_y = b.y - a.y;
  double ap_x = p.x - a.x; double ap_y = p.y - a.y;
  double ab_len_sq = ab_x * ab_x + ab_y * ab_y;
  if (ab_len_sq == 0.0) return std::hypot(ap_x, ap_y);
  double t = std::max(0.0, std::min(1.0, (ap_x * ab_x + ap_y * ab_y) / ab_len_sq));
  return std::hypot(p.x - (a.x + t * ab_x), p.y - (a.y + t * ab_y));
}

}  // namespace amr_bt_nodes

#include "behaviortree_cpp/bt_factory.h"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<amr_bt_nodes::ComputeValidatedPath>("ComputeValidatedPath");
}
