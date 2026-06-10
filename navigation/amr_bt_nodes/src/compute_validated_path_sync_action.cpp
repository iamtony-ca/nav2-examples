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
  dynamic_error_code_(0),
  max_403_retries_(3)
{
}

// [수정 포인트 1] 모든 스코프를 ComputeValidatedPathSyncAction:: 으로 통일
bool ComputeValidatedPathSyncAction::initialize()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
    // 초기화 중에는 logger_가 셋업 전일 수 있으므로 직접 로거 호출
    RCLCPP_ERROR(rclcpp::get_logger("ComputeValidatedPathSyncAction"), "Failed to get 'node' from blackboard.");
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
    RCLCPP_WARN(logger_, "[ComputeValidatedPathSyncAction] Planner action servers are not ready yet.");
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

  RCLCPP_INFO(logger_, "[ComputeValidatedPathSyncAction] Debug path viz on '%s', '%s'.",
    ref_viz_topic.c_str(), actual_viz_topic.c_str());


  initialized_ = true;
  RCLCPP_INFO(logger_, "[ComputeValidatedPathSyncAction] Initialized successfully. Connected to action servers.");
  return true;
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

BT::NodeStatus ComputeValidatedPathSyncAction::onStart()
{
  if (!initialized_) {
    if (!initialize()) {
      setOutput("validation_error_code_id", static_cast<uint16_t>(404)); // Not Initialized
      RCLCPP_WARN(logger_, "[ComputeValidatedPathSyncAction] Not Initialized. validation_error_code_id : 404");
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


// 새 사이클 시작: 이 시점 이후 도착하는 '이전 사이클' 콜백은 전부 무효화됨
  const uint64_t my_cycle = ++cycle_id_;

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
  send_opts_static.goal_response_callback =
    [this, my_cycle](auto handle) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (my_cycle != cycle_id_.load()) return;          // stale 폐기
      static_goal_handle_ = handle;
    };
  send_opts_static.result_callback =
    [this, my_cycle](const GoalHandle::WrappedResult & result) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (my_cycle != cycle_id_.load()) {                // stale 폐기
        RCLCPP_WARN(logger_,
          "[ComputeValidatedPathSyncAction][static cb] STALE (cb=%lu cur=%lu) ignored.",
          (unsigned long)my_cycle, (unsigned long)cycle_id_.load());
        return;
      }
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        static_path_ = result.result->path;
        static_error_code_ = 0;
      } else {
        static_error_code_ = result.result ? result.result->error_code : 308;
      }
      static_done_ = true;
    };







  start_time_ = node_->now();
  
  RCLCPP_DEBUG(logger_, "[ComputeValidatedPathSyncAction] Step 1: Sending async goal to STATIC planner.");
  static_client_->async_send_goal(static_goal_msg, send_opts_static);

  callback_group_executor_.spin_some();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ComputeValidatedPathSyncAction::onRunning()
{
  if ((node_->now() - start_time_).seconds() > 20.0) {
    RCLCPP_ERROR(logger_, "[ComputeValidatedPathSyncAction] Planning Timeout! Exceeded 20s.");
    onHalted(); 
    setOutput("validation_error_code_id", static_cast<uint16_t>(404)); // Timeout
    RCLCPP_WARN(logger_, "[ComputeValidatedPathSyncAction] Planner Timeout. validation_error_code_id : 404");
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

      // if (static_error_code_ != 0) {
      if (static_error_code_ != 0 || static_path_.poses.empty()) {
        RCLCPP_WARN(logger_, "[ComputeValidatedPathSyncAction] Static planner failed (Err: %d). Aborting.", static_error_code_.load());
        nav_msgs::msg::Path empty_path;
        setOutput("validated_path", empty_path);
        setOutput("validation_error_code_id", static_cast<uint16_t>(401)); // Static Planner Failure
        RCLCPP_WARN(logger_, "[ComputeValidatedPathSyncAction] Static Planner Failure. validation_error_code_id : 401");
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




      const uint64_t my_cycle = cycle_id_.load();   // ← 추가

      auto send_opts_dynamic = rclcpp_action::Client<ActionType>::SendGoalOptions();
      send_opts_dynamic.goal_response_callback =
        [this, my_cycle](auto handle) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (my_cycle != cycle_id_.load()) return;       // stale 폐기
          dynamic_goal_handle_ = handle;
        };
      send_opts_dynamic.result_callback =
        [this, my_cycle](const GoalHandle::WrappedResult & result) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (my_cycle != cycle_id_.load()) {             // ★ stale 폐기 (핵심)
            RCLCPP_WARN(logger_,
              "[ComputeValidatedPathSyncAction][dyn cb] STALE (cb=%lu cur=%lu) ignored. poses=%zu",
              (unsigned long)my_cycle, (unsigned long)cycle_id_.load(),
              result.result ? result.result->path.poses.size() : 0);
            return;
          }
          RCLCPP_WARN(logger_,
            "[ComputeValidatedPathSyncAction][dyn cb] cycle=%lu code=%d err=%u poses=%zu",
            (unsigned long)my_cycle, (int)result.code,
            result.result ? result.result->error_code : 9999,
            result.result ? result.result->path.poses.size() : 0);

          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            actual_path_ = result.result->path;
            dynamic_error_code_ = 0;
          } else {
            dynamic_error_code_ = result.result ? result.result->error_code : 308;
          }
          dynamic_done_ = true;
        };

      RCLCPP_DEBUG(logger_, "[ComputeValidatedPathSyncAction] Step 2: Sending async goal to DYNAMIC planner.");
      dynamic_client_->async_send_goal(dynamic_goal_msg, send_opts_dynamic);

      current_state_ = PlanningState::WAITING_FOR_DYNAMIC;
      return BT::NodeStatus::RUNNING;



    }

    case PlanningState::WAITING_FOR_DYNAMIC:
    {
      if (!dynamic_done_) return BT::NodeStatus::RUNNING;

      setOutput("dynamic_error_code_id", static_cast<uint16_t>(dynamic_error_code_.load()));

      // if (dynamic_error_code_ != 0) {
      if (dynamic_error_code_ != 0 || actual_path_.poses.empty()) {
        RCLCPP_WARN(logger_, "[ComputeValidatedPathSyncAction] Dynamic planner failed (Err: %d).", dynamic_error_code_.load());
        nav_msgs::msg::Path empty_path;
        setOutput("validated_path", empty_path);
        setOutput("validation_error_code_id", static_cast<uint16_t>(402)); // Dynamic Planner Failure
        RCLCPP_WARN(logger_, "[ComputeValidatedPathSyncAction] Dynamic Planner Failure. validation_error_code_id : 402");
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
        // [성공 시] 누가 성공했든 간에 공유 카운터를 0으로 완벽히 리셋!
        // [기존 코드] 자신에게 할당된 방만 초기화함
        // config().blackboard->set(count_key_, 0); 

        // [변경 핵심] 어떤 노드가 성공했든 간에, 주행용/수동용 403 카운터를 통째로 전면 리셋!
        config().blackboard->set("drive_403_count", 0);
        config().blackboard->set("manual_403_count", 0);  
        
        setOutput("validated_path", actual_path_); 
        setOutput("validation_error_code_id", static_cast<uint16_t>(0));
        RCLCPP_DEBUG(logger_, "[ComputeValidatedPathSyncAction] Step 3: Path validation passed.");
        current_state_ = PlanningState::IDLE;
        return BT::NodeStatus::SUCCESS;

      } else {

        if (flag_path_empty == true) {
          nav_msgs::msg::Path empty_path;
          setOutput("validated_path", empty_path);
          setOutput("validation_error_code_id", static_cast<uint16_t>(406));
          RCLCPP_WARN(logger_, "[ComputeValidatedPathSyncAction] empty path. validation_error_code_id : 406");
          return BT::NodeStatus::FAILURE;
        }


        // ==============================================================
        // 플래닝 및 편차 검증 실패 시 (기존 로직 100% 동일 유지)
        // ==============================================================        
        RCLCPP_WARN(logger_, "[ComputeValidatedPathSyncAction] Step 3: Detour deviation > %.2f detected!", max_dev_);
        
        
        nav_msgs::msg::Path empty_path;
        setOutput("validated_path", empty_path);
        setOutput("validation_error_code_id", static_cast<uint16_t>(403)); // Validation Failure - Detour
        
        // [실패 시] 공유 카운터 1 증가
        current_failures++;
        // 임계치 도달 여부 검사
        if (current_failures >= max_403_retries_) {
          RCLCPP_ERROR(logger_, "[ComputeValidatedPathSyncAction] Max 403 retries exceeded! Emitting error 405.");
          setOutput("validation_error_code_id", static_cast<uint16_t>(405));
          config().blackboard->set(count_key_, 0); // 알람 나갔으므로 리셋
          
        } else {
          setOutput("validation_error_code_id", static_cast<uint16_t>(403)); 
          config().blackboard->set(count_key_, current_failures); // 증가된 값 블랙보드에 저장
          RCLCPP_WARN(logger_, "[ComputeValidatedPathSyncAction] Planner Validation Failure(Detour). validation_error_code_id : 403, Count %d", current_failures);
        }



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

  ++cycle_id_;                       // 이 사이클의 모든 콜백을 stale로 만들어 무효화
  static_done_ = false;
  dynamic_done_ = false;
  static_goal_handle_.reset();
  dynamic_goal_handle_.reset();
  current_state_ = PlanningState::IDLE;

  RCLCPP_INFO(logger_,
    "[ComputeValidatedPathSyncAction] Node halted. cycle invalidated -> %lu",
    (unsigned long)cycle_id_.load());
}


bool ComputeValidatedPathSyncAction::performValidation()
{
  if (actual_path_.poses.empty() || static_path_.poses.empty()) {
    RCLCPP_WARN(logger_, "[ComputeValidatedPathSyncAction] One of the planners returned an empty path.");
    RCLCPP_WARN(logger_, "[ComputeValidatedPathSyncAction][Validate] empty path: static=%zu, actual=%zu",
       static_path_.poses.size(), actual_path_.poses.size());
    // setOutput("validation_error_code_id", static_cast<uint16_t>(406));   
    RCLCPP_WARN(logger_, "[ComputeValidatedPathSyncAction] empty path. validation_error_code_id : 406");
    flag_path_empty = true;
    return false;
  }

  flag_path_empty = false;
  nav_msgs::msg::Path trunc_ref = static_path_;
  nav_msgs::msg::Path trunc_actual = actual_path_;
  
  geometry_msgs::msg::PoseStamped current_pose;
  getInput("current_pose", current_pose);
  auto start_pos = current_pose.pose.position; 

  // 1. 정적 경로(trunc_ref)를 로봇 위치 기준 10m 지점에서 절삭
  truncatePathByEuclidean(trunc_ref, start_pos, horizon_);

  if (trunc_ref.poses.empty()) {
    RCLCPP_WARN(logger_, "[ComputeValidatedPathSyncAction] Truncated reference path is empty.");
    return false;
  }

  publishDebugPath(trunc_ref_pub_, trunc_ref); // publishDebugPath(trunc_ref_pub_, trunc_ref, start_pos_frame_or_map(current_pose));


  // 2. 잘려나간 정적 경로의 최종 끝점(Target) 확보
  auto target_pos = trunc_ref.poses.back().pose.position; 

  // [교정 핵심] 기존의 정방향 하드코딩 루프를 지우고, 
  // 이미 완벽하게 구현해 두신 역방향 타겟 추적 함수(truncatePathToGoal)를 호출하여 동기화합니다.
  truncatePathToGoal(trunc_actual, target_pos);

  if (trunc_actual.poses.empty()) {
    RCLCPP_WARN(logger_, "[ComputeValidatedPathSyncAction] Dynamic path sync failed. Path is empty after truncation.");
    return false;
  }

  publishDebugPath(trunc_actual_pub_, trunc_actual);

  // 3. 도달 검증 (Fail-fast): 뒤에서 자른 실제 경로의 끝점이 정적 끝점과 허용 범위 내에 있는지 검사
  auto dynamic_end_pos = trunc_actual.poses.back().pose.position;
  double min_dist = std::hypot(dynamic_end_pos.x - target_pos.x, dynamic_end_pos.y - target_pos.y);

  if (min_dist > max_dev_) {
    RCLCPP_WARN(logger_, "[ComputeValidatedPathSyncAction] Dynamic path failed to reach the static path's end point. Min dist: %.2f > Max Dev: %.2f", min_dist, max_dev_);
    return false; 
  }

  // 4. 본격적인 선분 기반 촘촘한 편차 검사 수행
  double accumulated_length = 0.0;          
  double dist_since_last = 0.0;       
  auto prev_pos = trunc_actual.poses.front().pose.position;

  if (!isPoseWithinDeviation(prev_pos, trunc_ref, max_dev_)) return false;

  for (size_t i = 1; i < trunc_actual.poses.size(); ++i) {
    const auto & curr_pos = trunc_actual.poses[i].pose.position;
    double step = std::hypot(curr_pos.x - prev_pos.x, curr_pos.y - prev_pos.y);
    accumulated_length += step;
    dist_since_last += step;

    if (accumulated_length > max_check_length_) break;

    if (dist_since_last >= step_dist_ || i == trunc_actual.poses.size() - 1) {
      if (!isPoseWithinDeviation(curr_pos, trunc_ref, max_dev_)) {
        RCLCPP_WARN(logger_, "[ComputeValidatedPathSyncAction] Detour detected at dist %.2f. Deviation exceeds %.2f", accumulated_length, max_dev_);
        return false; 
      }
      dist_since_last = 0.0;
    }
    prev_pos = curr_pos; 
  }

  return true; 
}



// bool ComputeValidatedPathSyncAction::performValidation()
// {
//   if (actual_path_.poses.empty() || static_path_.poses.empty()) {
//     RCLCPP_WARN(logger_, "[ComputeValidatedPathSyncAction] One of the planners returned an empty path.");
//     return false;
//   }

//   nav_msgs::msg::Path trunc_ref = static_path_;
//   nav_msgs::msg::Path trunc_actual = actual_path_;
  
//   geometry_msgs::msg::PoseStamped current_pose;
//   getInput("current_pose", current_pose);
//   auto start_pos = current_pose.pose.position; 

//   truncatePathByEuclidean(trunc_ref, start_pos, horizon_);

//   if (trunc_ref.poses.empty()) {
//     RCLCPP_WARN(logger_, "[ComputeValidatedPathSyncAction] Truncated reference path is empty.");
//     return false;
//   }

//   auto target_pos = trunc_ref.poses.back().pose.position; 
//   size_t closest_idx = 0;
//   double min_dist = 1e9;

//   for (size_t i = 0; i < trunc_actual.poses.size(); ++i) {
//     double dist = std::hypot(trunc_actual.poses[i].pose.position.x - target_pos.x,
//                              trunc_actual.poses[i].pose.position.y - target_pos.y);
//     if (dist < min_dist) { 
//       min_dist = dist; 
//       closest_idx = i; 
//     }
//   }

//   if (min_dist > max_dev_) {
//     RCLCPP_WARN(logger_, "[ComputeValidatedPathSyncAction] Dynamic path failed to reach the static path's end point. Min dist: %.2f > Max Dev: %.2f", min_dist, max_dev_);
//     return false; 
//   }

//   trunc_actual.poses.resize(closest_idx + 1);

//   double accumulated_length = 0.0;          
//   double dist_since_last = 0.0;       
//   auto prev_pos = trunc_actual.poses.front().pose.position;

//   if (!isPoseWithinDeviation(prev_pos, trunc_ref, max_dev_)) return false;

//   for (size_t i = 1; i < trunc_actual.poses.size(); ++i) {
//     const auto & curr_pos = trunc_actual.poses[i].pose.position;
//     double step = std::hypot(curr_pos.x - prev_pos.x, curr_pos.y - prev_pos.y);
//     accumulated_length += step;
//     dist_since_last += step;

//     if (accumulated_length > max_check_length_) break;

//     if (dist_since_last >= step_dist_ || i == trunc_actual.poses.size() - 1) {
//       if (!isPoseWithinDeviation(curr_pos, trunc_ref, max_dev_)) {
//         RCLCPP_WARN(logger_, "[ComputeValidatedPathSyncAction] Detour detected at dist %.2f. Deviation exceeds %.2f", accumulated_length, max_dev_);
//         return false; 
//       }
//       dist_since_last = 0.0;
//     }
//     prev_pos = curr_pos; 
//   }

//   return true; 
// }



void ComputeValidatedPathSyncAction::publishDebugPath(
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


// [수정 포인트 3] 10m 안으로 들어오는 최초 지점을 찾아 '그 뒷부분'을 잘라냄
void ComputeValidatedPathSyncAction::truncatePathByEuclidean(nav_msgs::msg::Path & path, const geometry_msgs::msg::Point & start, double dist_limit) {
  if (path.poses.empty()) return;
  
  auto it = std::find_if(path.poses.rbegin(), path.poses.rend(),
    [&start, dist_limit](const geometry_msgs::msg::PoseStamped & p) {
      return std::hypot(p.pose.position.x - start.x, p.pose.position.y - start.y) <= dist_limit;
    });
    
  if (it != path.poses.rend()) {
    // it.base()는 10m 이내로 들어온 점의 "다음 점(더 먼 점)"을 가리킵니다.
    // 즉, 10m 바깥에 있는 꼬리 부분을 모두 지웁니다.
    path.poses.erase(it.base(), path.poses.end());
  }
}

void ComputeValidatedPathSyncAction::truncatePathToGoal(nav_msgs::msg::Path & path, const geometry_msgs::msg::Point & target) 
{
  if (path.poses.empty()) return;

  auto it = std::min_element(path.poses.rbegin(), path.poses.rend(),
    [&target](const geometry_msgs::msg::PoseStamped & a, const geometry_msgs::msg::PoseStamped & b) {
      double dist_a = std::hypot(a.pose.position.x - target.x, a.pose.position.y - target.y);
      double dist_b = std::hypot(b.pose.position.x - target.x, b.pose.position.y - target.y);
      return dist_a < dist_b;
    });

  // 타겟과 가장 가까운 점(it.base()) 이후의 모든 점을 지워 동기화
  path.poses.erase(it.base(), path.poses.end());
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
