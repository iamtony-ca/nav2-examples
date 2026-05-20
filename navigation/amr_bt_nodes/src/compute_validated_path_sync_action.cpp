#include "amr_bt_nodes/compute_validated_path_sync_action.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace amr_bt_nodes
{

ComputeValidatedPathSyncAction::ComputeValidatedPathSyncAction(
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

void ComputeValidatedPathSyncAction::initialize()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
    throw std::runtime_error("Failed to get 'node' from blackboard in ComputeValidatedPathSyncAction");
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
    throw std::runtime_error("Planner action servers not available in ComputeValidatedPathSyncAction");
  }

  initialized_ = true;
  RCLCPP_INFO(logger_, "[ComputeValidatedPathSyncAction] Initialized successfully. Connected to action servers.");
}

BT::PortsList ComputeValidatedPathSyncAction::providedPorts()
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

BT::NodeStatus ComputeValidatedPathSyncAction::onStart()
{
  if (!initialized_) {
    if (!initialize()) {
      setOutput("validation_error_code_id", static_cast<uint16_t>(308));
      return BT::NodeStatus::FAILURE; 
    }
  }

  std::vector<geometry_msgs::msg::PoseStamped> global_goals;
  geometry_msgs::msg::PoseStamped current_pose;
  std::string static_planner_id, dynamic_planner_id;

  if (!getInput("global_goals", global_goals) || global_goals.empty() || 
      !getInput("current_pose", current_pose)) {
    RCLCPP_ERROR(logger_, "[ComputeValidatedPathSyncAction] Missing required inputs.");
    return BT::NodeStatus::FAILURE;
  }

  getInput("static_planner_id", static_planner_id);
  getInput("dynamic_planner_id", dynamic_planner_id);
  getInput("horizon_distance", horizon_);
  getInput("max_deviation", max_dev_);
  getInput("step_distance", step_dist_);
  getInput("max_check_length", max_check_length_);

  // Local Goals 추출 (Rolling Horizon)
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

  // 변수 초기화
  static_done_ = false;
  dynamic_done_ = false;
  static_error_code_ = 0;
  dynamic_error_code_ = 0;
  static_goal_handle_.reset();
  dynamic_goal_handle_.reset();
  static_path_ = nav_msgs::msg::Path();
  actual_path_ = nav_msgs::msg::Path();

  // [수정 핵심 1] 상태 머신 초기화
  current_state_ = PlanningState::WAITING_FOR_STATIC;

  // [수정 핵심 2] Static Goal만 먼저 전송
  auto static_goal_msg = ActionType::Goal();
  static_goal_msg.goals = local_goals_;
  static_goal_msg.planner_id = static_planner_id;
  static_goal_msg.use_start = false;

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

  RCLCPP_DEBUG(logger_, "[ComputeValidatedPathSyncAction] Step 1: Sending async goal to STATIC planner.");
  static_client_->async_send_goal(static_goal_msg, send_opts_static);

  // 콜백 처리용 스핀
  callback_group_executor_.spin_some();

  return BT::NodeStatus::RUNNING;
}



BT::NodeStatus ComputeValidatedPathSyncAction::onRunning()
{
  // 멈춰있던 콜백 처리
  callback_group_executor_.spin_some();

  std::lock_guard<std::mutex> lock(mutex_);

  switch (current_state_) {
    
    // ==========================================
    // 상태 1: Static 플래너 응답 대기 중
    // ==========================================
    case PlanningState::WAITING_FOR_STATIC:
    {
      if (!static_done_) {
        return BT::NodeStatus::RUNNING; // 아직 안 왔으면 계속 대기
      }

      setOutput("static_error_code_id", static_cast<uint16_t>(static_error_code_.load()));
      
      if (!static_path_.poses.empty()) {
        setOutput("static_path", static_path_);
      } else {
        setOutput("static_path", nav_msgs::msg::Path());
      }

      // Static이 실패했다면 Dynamic은 돌려볼 필요도 없이 즉시 실패 반환 (옵션 A 정책)
      if (static_error_code_ != 0) {
        RCLCPP_WARN(logger_, "[ComputeValidatedPathSyncAction] Static planner failed (Err: %d). Aborting.", static_error_code_.load());
        nav_msgs::msg::Path empty_path;
        setOutput("validated_path", empty_path);
        setOutput("validation_error_code_id", static_cast<uint16_t>(308));
        return BT::NodeStatus::FAILURE;
      }

      // Static이 성공했다면, 이제 Dynamic Goal 전송!
      std::vector<geometry_msgs::msg::PoseStamped> global_goals;
      std::string dynamic_planner_id;
      getInput("global_goals", global_goals);
      getInput("dynamic_planner_id", dynamic_planner_id);

      auto dynamic_goal_msg = ActionType::Goal();
      dynamic_goal_msg.goals = global_goals;
      dynamic_goal_msg.planner_id = dynamic_planner_id;
      dynamic_goal_msg.use_start = false;

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

      RCLCPP_DEBUG(logger_, "[ComputeValidatedPathSyncAction] Step 2: Static done. Sending async goal to DYNAMIC planner.");
      dynamic_client_->async_send_goal(dynamic_goal_msg, send_opts_dynamic);
      
      // 다음 상태로 전환
      current_state_ = PlanningState::WAITING_FOR_DYNAMIC;
      return BT::NodeStatus::RUNNING;
    }

    // ==========================================
    // 상태 2: Dynamic 플래너 응답 대기 중
    // ==========================================
    case PlanningState::WAITING_FOR_DYNAMIC:
    {
      if (!dynamic_done_) {
        return BT::NodeStatus::RUNNING; // 아직 안 왔으면 계속 대기
      }

      setOutput("dynamic_error_code_id", static_cast<uint16_t>(dynamic_error_code_.load()));

      // Dynamic 실패 처리
      if (dynamic_error_code_ != 0) {
        RCLCPP_WARN(logger_, "[ComputeValidatedPathSyncAction] Dynamic planner failed (Err: %d).", dynamic_error_code_.load());
        nav_msgs::msg::Path empty_path;
        setOutput("validated_path", empty_path);
        setOutput("validation_error_code_id", static_cast<uint16_t>(308)); 
        return BT::NodeStatus::FAILURE;
      }

      // 둘 다 성공했으므로 다음 상태로 전환
      current_state_ = PlanningState::VALIDATING;
      // break 없이 바로 다음 case로 흘러가도록(fall-through) 하거나 명시적 호출
    }

    // ==========================================
    // 상태 3: 검증 (Validation)
    // ==========================================
    case PlanningState::VALIDATING:
    {
      if (performValidation()) {
        setOutput("validated_path", actual_path_); 
        setOutput("validation_error_code_id", static_cast<uint16_t>(0));
        RCLCPP_DEBUG(logger_, "[ComputeValidatedPathSyncAction] Step 3: Path validation passed.");
        current_state_ = PlanningState::IDLE;
        return BT::NodeStatus::SUCCESS;
      } else {
        RCLCPP_WARN(logger_, "[ComputeValidatedPathSyncAction] Step 3: Detour deviation > %.2f detected!", max_dev_);
        nav_msgs::msg::Path empty_path;
        setOutput("validated_path", empty_path);
        setOutput("validation_error_code_id", static_cast<uint16_t>(308)); 
        current_state_ = PlanningState::IDLE;
        return BT::NodeStatus::FAILURE;
      }
    }
    
    default:
      return BT::NodeStatus::FAILURE;
  }
}





void ComputeValidatedPathSyncAction::onHalted()
{
  if (!static_done_ && static_goal_handle_) {
    static_client_->async_cancel_goal(static_goal_handle_);
  }
  if (!dynamic_done_ && dynamic_goal_handle_) {
    dynamic_client_->async_cancel_goal(dynamic_goal_handle_);
  }
  RCLCPP_INFO(logger_, "[ComputeValidatedPathSyncAction] Node halted. Action goals cancelled.");
}

bool ComputeValidatedPathSyncAction::performValidation()
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

void ComputeValidatedPathSyncAction::truncatePathByEuclidean(nav_msgs::msg::Path & path, const geometry_msgs::msg::Point & start, double dist_limit) {
  if (path.poses.empty()) return;
  auto it = std::find_if(path.poses.rbegin(), path.poses.rend(),
    [&start, dist_limit](const geometry_msgs::msg::PoseStamped & p) {
      return std::hypot(p.pose.position.x - start.x, p.pose.position.y - start.y) <= dist_limit;
    });
  if (it != path.poses.rend()) path.poses.erase(it.base(), path.poses.end());
}

void ComputeValidatedPathSyncAction::truncatePathToGoal(nav_msgs::msg::Path & path, const geometry_msgs::msg::Point & target) {
  if (path.poses.empty()) return;
  size_t closest_idx = 0; double min_dist = 1e9;
  for (size_t i = 0; i < path.poses.size(); ++i) {
    double dist = std::hypot(path.poses[i].pose.position.x - target.x, path.poses[i].pose.position.y - target.y);
    if (dist < min_dist) { min_dist = dist; closest_idx = i; }
  }
  path.poses.resize(closest_idx + 1);
}

bool ComputeValidatedPathSyncAction::isPoseWithinDeviation(const geometry_msgs::msg::Point & p, const nav_msgs::msg::Path & ref_path, double max_dev) {
  if (ref_path.poses.empty()) return false;
  if (ref_path.poses.size() == 1) return std::hypot(p.x - ref_path.poses[0].pose.position.x, p.y - ref_path.poses[0].pose.position.y) <= max_dev;
  double min_dist = 1e9;
  for (size_t j = 1; j < ref_path.poses.size(); ++j) {
    double dist = pointToLineSegmentDistance(p, ref_path.poses[j-1].pose.position, ref_path.poses[j].pose.position);
    min_dist = std::min(min_dist, dist);
  }
  return min_dist <= max_dev;
}

double ComputeValidatedPathSyncAction::pointToLineSegmentDistance(const geometry_msgs::msg::Point & p, const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b) {
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
  factory.registerNodeType<amr_bt_nodes::ComputeValidatedPathSyncAction>("ComputeValidatedPathSyncAction");
}
