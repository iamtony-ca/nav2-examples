#include "amr_bt_nodes/compute_validated_path_poses_sync_action.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <vector>
#include <stdexcept>

namespace amr_bt_nodes
{

ComputeValidatedPathPosesSyncAction::ComputeValidatedPathPosesSyncAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(xml_tag_name, conf),
  initialized_(false),
  static_done_(false),
  dynamic_done_(false),
  static_error_code_(0),
  dynamic_error_code_(0),
  max_403_retries_(3)
{
}

bool ComputeValidatedPathPosesSyncAction::initialize()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
    // 초기화 중에는 logger_가 셋업 전일 수 있으므로 직접 로거 호출
    RCLCPP_ERROR(rclcpp::get_logger("ComputeValidatedPathPosesSyncAction"), "Failed to get 'node' from blackboard.");
    return false;
  }

  logger_ = node_->get_logger();

  std::string static_server, dynamic_server;
  getInput("static_planner_server", static_server);
  getInput("dynamic_planner_server", dynamic_server);

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  static_client_ = rclcpp_action::create_client<ActionType>(node_, static_server, callback_group_);
  dynamic_client_ = rclcpp_action::create_client<ActionType>(node_, dynamic_server, callback_group_);

  if (!static_client_->action_server_is_ready() || !dynamic_client_->action_server_is_ready()) {
    RCLCPP_WARN(logger_, "[ComputeValidatedPathPosesSyncAction] Planner action servers are not ready yet.");
    return false;
  }

  std::string ref_viz_topic = "/validation/trunc_ref";
  std::string actual_viz_topic = "/validation/trunc_actual";
  getInput("debug_ref_topic", ref_viz_topic);
  getInput("debug_actual_topic", actual_viz_topic);

  // 기존 PathPublisherAction과 동일하게 depth-1(reliable, volatile).
  // RViz가 늦게 붙어도 마지막 경로를 보고 싶으면 rclcpp::QoS(1).transient_local() 로 교체.
  trunc_ref_pub_    = node_->create_publisher<nav_msgs::msg::Path>(ref_viz_topic, 1);
  trunc_actual_pub_ = node_->create_publisher<nav_msgs::msg::Path>(actual_viz_topic, 1);

  RCLCPP_INFO(logger_, "[ComputeValidatedPathPosesSyncAction] Debug path viz on '%s', '%s'.",
    ref_viz_topic.c_str(), actual_viz_topic.c_str());

  initialized_ = true;
  RCLCPP_INFO(logger_, "[ComputeValidatedPathPosesSyncAction] Initialized successfully. Connected to action servers.");
  return true;
}

BT::PortsList ComputeValidatedPathPosesSyncAction::providedPorts()
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
    BT::InputPort<std::string>("count_key", "consecutive_403_count", "Blackboard key for 403 failure counter"),
    BT::InputPort<int>("max_403_retries", 5, "Number of consecutive 403 errors before triggering alert"),

    BT::InputPort<std::string>("debug_ref_topic", "/validation/trunc_ref",
      "Debug: truncated reference(static) path used in validation"),
    BT::InputPort<std::string>("debug_actual_topic", "/validation/trunc_actual",
      "Debug: truncated actual(dynamic) path used in validation"),

    BT::OutputPort<nav_msgs::msg::Path>("validated_path", "Final validated path for controller"),
    BT::OutputPort<nav_msgs::msg::Path>("static_path", "Reference path from static planner (for debug/viz)"),
    BT::OutputPort<uint16_t>("static_error_code_id", "Error code from static planner"),
    BT::OutputPort<uint16_t>("dynamic_error_code_id", "Error code from dynamic planner"),
    BT::OutputPort<uint16_t>("validation_error_code_id", "Error code for deviation validation (0: Success, 308: Detour)")
  };
}

BT::NodeStatus ComputeValidatedPathPosesSyncAction::onStart()
{
  if (!initialized_) {
    if (!initialize()) {
      setOutput("validation_error_code_id", static_cast<uint16_t>(404)); // Not Initialized
      return BT::NodeStatus::FAILURE;
    }
  }

  std::vector<geometry_msgs::msg::PoseStamped> global_goals;
  geometry_msgs::msg::PoseStamped current_pose;
  std::string static_planner_id, dynamic_planner_id;

  if (!getInput("global_goals", global_goals) || global_goals.empty() ||
      !getInput("current_pose", current_pose)) {
    RCLCPP_ERROR(logger_, "[ComputeValidatedPathPosesSyncAction] Missing required inputs.");
    return BT::NodeStatus::FAILURE;
  }

  getInput("static_planner_id", static_planner_id);
  getInput("dynamic_planner_id", dynamic_planner_id);
  getInput("horizon_distance", horizon_);
  getInput("max_deviation", max_dev_);
  getInput("step_distance", step_dist_);
  getInput("max_check_length", max_check_length_);
  getInput("count_key", count_key_);
  getInput("max_403_retries", max_403_retries_);

  int target_idx = -1;
  const auto & robot_pos = current_pose.pose.position;

  for (int i = static_cast<int>(global_goals.size()) - 1; i >= 0; --i) {
    double dist = std::hypot(global_goals[i].pose.position.x - robot_pos.x,
                             global_goals[i].pose.position.y - robot_pos.y);
    if (dist <= horizon_) {
      target_idx = i;
      break;
    }
  }

  if (target_idx == -1) {
    local_goals_ = std::vector<geometry_msgs::msg::PoseStamped>(global_goals.begin(), global_goals.begin() + 1);
  } else if (target_idx + 1 < static_cast<int>(global_goals.size())) {
    local_goals_ = std::vector<geometry_msgs::msg::PoseStamped>(global_goals.begin(), global_goals.begin() + target_idx + 2);
  } else {
    local_goals_ = global_goals;
  }

  static_done_ = false;
  dynamic_done_ = false;
  static_error_code_ = 0;
  dynamic_error_code_ = 0;
  static_goal_handle_.reset();
  dynamic_goal_handle_.reset();
  static_path_ = nav_msgs::msg::Path();
  actual_path_ = nav_msgs::msg::Path();

  current_state_ = PlanningState::WAITING_FOR_STATIC;

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

  start_time_ = node_->now();

  RCLCPP_DEBUG(logger_, "[ComputeValidatedPathPosesSyncAction] Step 1: Sending async goal to STATIC planner.");
  static_client_->async_send_goal(static_goal_msg, send_opts_static);

  callback_group_executor_.spin_some();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ComputeValidatedPathPosesSyncAction::onRunning()
{
  if ((node_->now() - start_time_).seconds() > 10.0) {
    RCLCPP_ERROR(logger_, "[ComputeValidatedPathPosesSyncAction] Planning Timeout! Exceeded 10s.");
    onHalted();
    setOutput("validation_error_code_id", static_cast<uint16_t>(404)); // Timeout
    return BT::NodeStatus::FAILURE;
  }

  callback_group_executor_.spin_some();

  std::lock_guard<std::mutex> lock(mutex_);

  switch (current_state_) {

    case PlanningState::WAITING_FOR_STATIC:
    {
      if (!static_done_) return BT::NodeStatus::RUNNING;

      setOutput("static_error_code_id", static_cast<uint16_t>(static_error_code_.load()));

      if (!static_path_.poses.empty()) {
        setOutput("static_path", static_path_);
      } else {
        setOutput("static_path", nav_msgs::msg::Path());
      }

      if (static_error_code_ != 0) {
        RCLCPP_WARN(logger_, "[ComputeValidatedPathPosesSyncAction] Static planner failed (Err: %d). Aborting.", static_error_code_.load());
        nav_msgs::msg::Path empty_path;
        setOutput("validated_path", empty_path);
        setOutput("validation_error_code_id", static_cast<uint16_t>(401)); // Static Planner Failure
        return BT::NodeStatus::FAILURE;
      }

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

      RCLCPP_DEBUG(logger_, "[ComputeValidatedPathPosesSyncAction] Step 2: Static done. Sending async goal to DYNAMIC planner.");
      dynamic_client_->async_send_goal(dynamic_goal_msg, send_opts_dynamic);

      current_state_ = PlanningState::WAITING_FOR_DYNAMIC;
      return BT::NodeStatus::RUNNING;
    }

    case PlanningState::WAITING_FOR_DYNAMIC:
    {
      if (!dynamic_done_) return BT::NodeStatus::RUNNING;

      setOutput("dynamic_error_code_id", static_cast<uint16_t>(dynamic_error_code_.load()));

      if (dynamic_error_code_ != 0) {
        RCLCPP_WARN(logger_, "[ComputeValidatedPathPosesSyncAction] Dynamic planner failed (Err: %d).", dynamic_error_code_.load());
        nav_msgs::msg::Path empty_path;
        setOutput("validated_path", empty_path);
        setOutput("validation_error_code_id", static_cast<uint16_t>(402)); // Dynamic Planner Failure
        return BT::NodeStatus::FAILURE;
      }

      current_state_ = PlanningState::VALIDATING;
      // break 없이 바로 다음 상태 검증으로 넘어갑니다. (Fall-through)
    }

    case PlanningState::VALIDATING:
    {
      // 블랙보드에서 현재 누적된 403 에러 횟수 읽어오기 (없으면 0으로 시작)
      int current_failures = 0;
      config().blackboard->get(count_key_, current_failures);

      if (performValidation()) {
        // [성공 시] 어떤 노드가 성공했든 간에, 주행용/수동용 403 카운터를 통째로 전면 리셋!
        config().blackboard->set("drive_403_count", 0);
        config().blackboard->set("manual_403_count", 0);

        setOutput("validated_path", actual_path_);
        setOutput("validation_error_code_id", static_cast<uint16_t>(0));
        RCLCPP_DEBUG(logger_, "[ComputeValidatedPathPosesSyncAction] Step 3: Path validation passed.");
        current_state_ = PlanningState::IDLE;
        return BT::NodeStatus::SUCCESS;

      } else {
        // ==============================================================
        // 플래닝 및 편차 검증 실패 시
        // ==============================================================
        RCLCPP_WARN(logger_, "[ComputeValidatedPathPosesSyncAction] Step 3: Detour deviation > %.2f detected!", max_dev_);

        nav_msgs::msg::Path empty_path;
        setOutput("validated_path", empty_path);
        setOutput("validation_error_code_id", static_cast<uint16_t>(403)); // Validation Failure - Detour

        // [실패 시] 공유 카운터 1 증가
        current_failures++;
        // 임계치 도달 여부 검사
        if (current_failures >= max_403_retries_) {
          RCLCPP_ERROR(logger_, "[ComputeValidatedPathPosesSyncAction] Max 403 retries exceeded! Emitting error 405.");
          setOutput("validation_error_code_id", static_cast<uint16_t>(405));
          config().blackboard->set(count_key_, 0); // 알람 나갔으므로 리셋
        } else {
          setOutput("validation_error_code_id", static_cast<uint16_t>(403));
          config().blackboard->set(count_key_, current_failures); // 증가된 값 블랙보드에 저장
        }

        current_state_ = PlanningState::IDLE;
        return BT::NodeStatus::FAILURE;
      }
    }

    default:
      return BT::NodeStatus::FAILURE;
  }
}

void ComputeValidatedPathPosesSyncAction::onHalted()
{
  if (!static_done_ && static_goal_handle_) {
    static_client_->async_cancel_goal(static_goal_handle_);
  }
  if (!dynamic_done_ && dynamic_goal_handle_) {
    dynamic_client_->async_cancel_goal(dynamic_goal_handle_);
  }
  RCLCPP_INFO(logger_, "[ComputeValidatedPathPosesSyncAction] Node halted. Action goals cancelled.");
}

bool ComputeValidatedPathPosesSyncAction::performValidation()
{
  if (actual_path_.poses.empty() || static_path_.poses.empty()) {
    RCLCPP_WARN(logger_, "[ComputeValidatedPathPosesSyncAction] One of the planners returned an empty path.");
    return false;
  }
  if (local_goals_.empty()) {
    RCLCPP_WARN(logger_, "[ComputeValidatedPathPosesSyncAction] local_goals_ is empty; cannot determine cut waypoint.");
    return false;
  }

  // ref는 10m로 자르지 않고 static 경로 전체(= local_goals 끝까지)를 폴리라인으로 사용한다.
  nav_msgs::msg::Path trunc_ref = static_path_;
  nav_msgs::msg::Path trunc_actual = actual_path_;

  // 기준 끝점 = 마지막 할당 waypoint (static 경로의 종점이기도 함).
  const auto & target_pos = local_goals_.back().pose.position;

  // 경로 점 간격(앞쪽 표본 중앙값)에서 reach_tol 유도. 그리드 플래너 기준 ~ 3 * resolution.
  const double path_res = estimatePathResolution(actual_path_);
  const double reach_tol = (path_res > 1e-6) ? 3.0 * path_res : 0.15;  // 추정 실패 시 안전 floor

  // actual을 '첫 통과'에서 절삭: 같은 지점을 두 번 지나도 첫 pass에서 컷한다.
  truncateAtFirstWaypointPass(trunc_actual, target_pos, reach_tol);

  if (trunc_actual.poses.empty()) {
    RCLCPP_WARN(logger_, "[ComputeValidatedPathPosesSyncAction] Dynamic path is empty after first-pass truncation.");
    return false;
  }

  // 검증에 실제로 쓰인 두 경로를 디버그 발행 (RViz 비교용).
  publishDebugPath(trunc_ref_pub_, trunc_ref);
  publishDebugPath(trunc_actual_pub_, trunc_actual);

  // [제거됨] 끝점 fail-fast: waypoint 기준 컷에서는 두 경로가 같은 지점에서 끝나 무의미.
  //          검출은 아래 점-폴리라인 스캔이 전담한다.

  // 선분 기반 편차 스캔 (actual의 각 점 -> ref 폴리라인 최근접 선분 거리). pass에 무관.
  double accumulated_length = 0.0;
  double dist_since_last = 0.0;
  auto prev_pos = trunc_actual.poses.front().pose.position;

  if (!isPoseWithinDeviation(prev_pos, trunc_ref, max_dev_)) {
    RCLCPP_WARN(logger_, "[ComputeValidatedPathPosesSyncAction] Start pose already deviates beyond %.2f.", max_dev_);
    return false;
  }

  for (size_t i = 1; i < trunc_actual.poses.size(); ++i) {
    const auto & curr_pos = trunc_actual.poses[i].pose.position;
    const double step = std::hypot(curr_pos.x - prev_pos.x, curr_pos.y - prev_pos.y);
    accumulated_length += step;
    dist_since_last += step;

    if (accumulated_length > max_check_length_) break;

    if (dist_since_last >= step_dist_ || i == trunc_actual.poses.size() - 1) {
      if (!isPoseWithinDeviation(curr_pos, trunc_ref, max_dev_)) {
        RCLCPP_WARN(logger_, "[ComputeValidatedPathPosesSyncAction] Detour detected at dist %.2f. Deviation exceeds %.2f", accumulated_length, max_dev_);
        return false;
      }
      dist_since_last = 0.0;
    }
    prev_pos = curr_pos;
  }

  return true;
}

void ComputeValidatedPathPosesSyncAction::publishDebugPath(
  const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & pub,
  const nav_msgs::msg::Path & path,
  const std::string & fallback_frame)
{
  if (!pub) {
    return;
  }
  nav_msgs::msg::Path msg = path;  // header(frame_id) + poses 복사
  if (msg.header.frame_id.empty()) {
    msg.header.frame_id = fallback_frame;
  }
  msg.header.stamp = node_->now();  // RViz 표시 신선도용
  pub->publish(msg);
}

void ComputeValidatedPathPosesSyncAction::truncateAtFirstWaypointPass(
  nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Point & target,
  double reach_tol)
{
  if (path.poses.empty()) return;

  auto dist_to_target = [&target](const geometry_msgs::msg::PoseStamped & ps) {
    return std::hypot(ps.pose.position.x - target.x, ps.pose.position.y - target.y);
  };

  // 1) target에 reach_tol 이내로 '처음' 진입하는 통과(첫 pass)를 찾는다.
  //    진입했다가 다시 벗어나면 그 직전 최근접점에서 멈춘다.
  bool entered = false;
  size_t best_idx = 0;
  double best_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < path.poses.size(); ++i) {
    const double d = dist_to_target(path.poses[i]);
    if (d <= reach_tol) {
      entered = true;
      if (d < best_dist) {
        best_dist = d;
        best_idx = i;
      }
    } else if (entered) {
      break;  // 첫 통과의 최근접을 지나 다시 멀어짐 -> 종료
    }
  }

  if (entered) {
    path.poses.resize(best_idx + 1);
    return;
  }

  // 2) Fallback: target 근처(reach_tol)에 한 번도 못 들어오면 전역 최근접점에서 컷.
  //    plan이 goal을 통과하면 거의 도달하지 않음(희소/부분 경로 대비).
  size_t closest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const double d = dist_to_target(path.poses[i]);
    if (d < min_dist) {
      min_dist = d;
      closest_idx = i;
    }
  }
  path.poses.resize(closest_idx + 1);
}

double ComputeValidatedPathPosesSyncAction::estimatePathResolution(const nav_msgs::msg::Path & path) const
{
  // 그리드 플래너의 점 간격은 거의 일정하므로, 앞쪽 일부 구간만 표본으로 대표 간격을 추정한다.
  // 전체를 훑지 않으므로 경로 길이와 무관하게 O(K).
  constexpr size_t kMaxSamples = 30;

  std::vector<double> steps;
  steps.reserve(kMaxSamples);
  for (size_t i = 1; i < path.poses.size() && steps.size() < kMaxSamples; ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    const double d = std::hypot(b.x - a.x, b.y - a.y);
    if (d > 1e-6) steps.push_back(d);
  }
  if (steps.empty()) return 0.0;

  const size_t mid = steps.size() / 2;
  std::nth_element(steps.begin(), steps.begin() + mid, steps.end());
  return steps[mid];  // 표본 중앙값 (이상치에 강함)
}

bool ComputeValidatedPathPosesSyncAction::isPoseWithinDeviation(const geometry_msgs::msg::Point & p, const nav_msgs::msg::Path & ref_path, double max_dev) {
  if (ref_path.poses.empty()) return false;
  if (ref_path.poses.size() == 1) return std::hypot(p.x - ref_path.poses[0].pose.position.x, p.y - ref_path.poses[0].pose.position.y) <= max_dev;
  double min_dist = 1e9;
  for (size_t j = 1; j < ref_path.poses.size(); ++j) {
    double dist = pointToLineSegmentDistance(p, ref_path.poses[j-1].pose.position, ref_path.poses[j].pose.position);
    min_dist = std::min(min_dist, dist);
  }
  return min_dist <= max_dev;
}

double ComputeValidatedPathPosesSyncAction::pointToLineSegmentDistance(const geometry_msgs::msg::Point & p, const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b) {
  double ab_x = b.x - a.x; double ab_y = b.y - a.y;
  double ap_x = p.x - a.x; double ap_y = p.y - a.y;
  double ab_len_sq = ab_x * ab_x + ab_y * ab_y;
  if (ab_len_sq == 0.0) return std::hypot(ap_x, ap_y);
  double t = std::max(0.0, std::min(1.0, (ap_x * ab_x + ap_y * ab_y) / ab_len_sq));
  return std::hypot(p.x - (a.x + t * ab_x), p.y - (a.y + t * ab_y));
}

}  // namespace amr_bt_nodes

#include "behaviortree_cpp/bt_factory.h"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<amr_bt_nodes::ComputeValidatedPathPosesSyncAction>("ComputeValidatedPathPosesSyncAction");
}
