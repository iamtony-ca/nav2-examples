#include "replan_monitor/path_validator_node.hpp"
// [NEW] For makeFootprintFromString and makeFootprintFromRadius
#include <nav2_costmap_2d/footprint.hpp>

#include <sstream>

using std::placeholders::_1;

namespace replan_monitor
{


// [NEW] Implementation of static helper (from agent_layer)
std::vector<geometry_msgs::msg::Point32> PathValidatorNode::toPoint32(
    const std::vector<geometry_msgs::msg::Point>& points)
{
    std::vector<geometry_msgs::msg::Point32> points32;
    points32.reserve(points.size());
    for (const auto& p : points) {
        geometry_msgs::msg::Point32 p32;
        p32.x = static_cast<float>(p.x);
        p32.y = static_cast<float>(p.y);
        p32.z = 0.0f;
        points32.push_back(p32);
    }
    return points32;
}

// [NEW] Implementation of helper to get footprint from loaded map
std::vector<geometry_msgs::msg::Point32> 
PathValidatorNode::getFootprintForAgent(const multi_agent_msgs::msg::MultiAgentInfo & a) const
{
    auto it = agent_footprints_.find(a.machine_id);
    if (it == agent_footprints_.end()) {
        RCLCPP_WARN_ONCE(get_logger(), 
          "No footprint data found in YAML for machine_id %u. Cannot check agent collision.",
          a.machine_id);
        return {}; // Return empty vector
    }

    const auto& data = it->second;

    if (data.use_radius) {
        std::vector<geometry_msgs::msg::Point> points = 
            nav2_costmap_2d::makeFootprintFromRadius(data.radius);
        return toPoint32(points);
    } else {
        return data.points; // Return pre-loaded Point32 vector
    }
}






PathValidatorNode::PathValidatorNode()
: Node("path_validator_node")
{
  // ===== 기본 파라미터 =====
  this->declare_parameter<std::string>("global_frame", "map");
  this->declare_parameter<std::string>("base_frame", "base_link");

  this->declare_parameter("self_machine_id", 0);

  this->declare_parameter("cooldown_sec", 1.0);
  this->declare_parameter("consecutive_threshold", 2);  //3
  this->declare_parameter("obstacle_persistence_sec", 1.0);  // 0.5
  this->declare_parameter("max_speed", 0.5);
  this->declare_parameter("lookahead_time_sec", 15.0);
  this->declare_parameter("min_lookahead_m", 0.8);  // 2.0
  this->declare_parameter("cost_threshold", 254.0); //200.0
  this->declare_parameter("ignore_unknown", true);

  this->declare_parameter("db_update_frequency", 5.0);
  this->declare_parameter("obstacle_prune_timeout_sec", 3.0);
  this->declare_parameter("db_stride", 1);  // 2
  this->declare_parameter("cone_angle_deg", 170.0); //100.0
  this->declare_parameter("kernel_half_size", 2); // 1

  this->declare_parameter("path_check_distance_m", 8.0); //6.0

  this->declare_parameter("publish_false_pulse", false); 
  this->declare_parameter("flag_pulse_ms", 120);

  // ===== Footprint / Agent mask / Output =====
  this->declare_parameter("use_footprint_check", true); 
  this->declare_parameter("footprint_step_m", 0.15);

  this->declare_parameter("compare_agent_mask", true);
  this->declare_parameter<std::string>("agent_mask_topic", "/agent_layer/costmap_raw");
  this->declare_parameter("agent_cost_threshold", 254.0);
  this->declare_parameter("agent_mask_manhattan_buffer", 3);  // 1

  this->declare_parameter("publish_agent_collision", true);
  this->declare_parameter<std::string>("agent_collision_topic", "/path_agent_collision_info");

  // MultiAgent 구독
  this->declare_parameter<std::string>("agents_topic", "/multi_agent_infos");
  this->declare_parameter("agents_freshness_timeout_ms", 800);
  this->declare_parameter("agent_match_dilate_m", 0.1); // 0.05

  // Nav2 footprint 스타일
  this->declare_parameter<std::string>("footprint", "[]");
  this->declare_parameter("robot_radius", 0.1);

  // 에이전트 홀드
  this->declare_parameter("agent_block_hold_sec", 2.0);
  this->declare_parameter("agent_block_max_wait_sec", 8.0);

  // === NEW: 에이전트 경로 튜브 매칭 ===
  this->declare_parameter("agent_path_hit_enable", true);
  this->declare_parameter("agent_path_hit_stride_m", 0.35);
  this->declare_parameter("agent_path_hit_dilate_m", 0.05);
  this->declare_parameter("agent_path_hit_max_poses", 1000);

  this->declare_parameter("respect_higher_priority_path", false);

  this->declare_parameter("validation_frequency", 10.0); // 기본 10Hz 주기


  // 생성자(Constructor) 내부에 추가
  this->declare_parameter("goal_doorstep_static_m", 0.1);
  this->declare_parameter("goal_doorstep_agent_m", 0.05); // 에이전트는 지연 오차를 고려해 조금 더 크게 설정 가능

  goal_doorstep_static_m_ = this->get_parameter("goal_doorstep_static_m").as_double();
  goal_doorstep_agent_m_ = this->get_parameter("goal_doorstep_agent_m").as_double();



  // [NEW] remaining_goals 토픽 이름 파라미터 (기본값 설정)
  this->declare_parameter<std::string>("remaining_goals_topic", "/remaining_goals");
  std::string remaining_goals_topic = this->get_parameter("remaining_goals_topic").as_string();

  // ---- load parameters ----
  global_frame_               = this->get_parameter("global_frame").as_string();
  base_frame_                 = this->get_parameter("base_frame").as_string();

  self_machine_id_ = static_cast<uint16_t>(this->get_parameter("self_machine_id").as_int());

  cooldown_sec_               = this->get_parameter("cooldown_sec").as_double();
  consecutive_threshold_      = static_cast<size_t>(this->get_parameter("consecutive_threshold").as_int());
  obstacle_persistence_sec_   = this->get_parameter("obstacle_persistence_sec").as_double();
  max_speed_                  = this->get_parameter("max_speed").as_double();
  lookahead_time_sec_         = this->get_parameter("lookahead_time_sec").as_double();
  min_lookahead_m_            = this->get_parameter("min_lookahead_m").as_double();
  cost_threshold_             = this->get_parameter("cost_threshold").as_double();
  ignore_unknown_             = this->get_parameter("ignore_unknown").as_bool();

  db_update_frequency_        = this->get_parameter("db_update_frequency").as_double();
  obstacle_prune_timeout_sec_ = this->get_parameter("obstacle_prune_timeout_sec").as_double();
  db_stride_                  = std::max<int>(1, static_cast<int>(this->get_parameter("db_stride").as_int()));
  cone_angle_deg_             = this->get_parameter("cone_angle_deg").as_double();
  kernel_half_size_           = std::max<int>(0, static_cast<int>(this->get_parameter("kernel_half_size").as_int()));

  path_check_distance_m_      = this->get_parameter("path_check_distance_m").as_double();

  publish_false_pulse_        = this->get_parameter("publish_false_pulse").as_bool();
  flag_pulse_ms_              = static_cast<int>(this->get_parameter("flag_pulse_ms").as_int());

  use_footprint_check_        = this->get_parameter("use_footprint_check").as_bool();
  footprint_step_m_           = std::max(0.05, this->get_parameter("footprint_step_m").as_double());

  compare_agent_mask_         = this->get_parameter("compare_agent_mask").as_bool();
  agent_mask_topic_           = this->get_parameter("agent_mask_topic").as_string();
  agent_cost_threshold_       = this->get_parameter("agent_cost_threshold").as_double();
  agent_mask_manhattan_buffer_= std::max<int>(0, static_cast<int>(this->get_parameter("agent_mask_manhattan_buffer").as_int()));

  publish_agent_collision_    = this->get_parameter("publish_agent_collision").as_bool();
  agent_collision_topic_      = this->get_parameter("agent_collision_topic").as_string();

  agents_topic_               = this->get_parameter("agents_topic").as_string();
  agents_freshness_timeout_ms_= this->get_parameter("agents_freshness_timeout_ms").as_int();
  agent_match_dilate_m_       = this->get_parameter("agent_match_dilate_m").as_double();

  // Nav2 footprint / radius
  footprint_str_              = this->get_parameter("footprint").as_string();
  robot_radius_m_             = this->get_parameter("robot_radius").as_double();
  use_radius_                 = true;
  footprint_.clear();
  if (!footprint_str_.empty() && footprint_str_ != "[]") {
    if (nav2_costmap_2d::makeFootprintFromString(footprint_str_, footprint_) && footprint_.size() >= 3) {
      use_radius_ = false;
      RCLCPP_INFO(get_logger(), "Using polygon footprint with %zu points.", footprint_.size());
    } else {
      RCLCPP_ERROR(get_logger(),
        "Invalid footprint string: \"%s\". Falling back to robot_radius=%.3f",
        footprint_str_.c_str(), robot_radius_m_);
      use_radius_ = true;
    }
  } else {
    RCLCPP_INFO(get_logger(), "No valid footprint provided. Using robot_radius=%.3f", robot_radius_m_);
  }

  // 홀드
  agent_block_hold_sec_     = this->get_parameter("agent_block_hold_sec").as_double();
  agent_block_max_wait_sec_ = this->get_parameter("agent_block_max_wait_sec").as_double();

  // 경로 튜브 매칭
  agent_path_hit_enable_      = this->get_parameter("agent_path_hit_enable").as_bool();
  agent_path_hit_stride_m_    = this->get_parameter("agent_path_hit_stride_m").as_double();
  agent_path_hit_dilate_m_    = this->get_parameter("agent_path_hit_dilate_m").as_double();
  agent_path_hit_max_poses_   = this->get_parameter("agent_path_hit_max_poses").as_int();

  respect_higher_priority_path_ = this->get_parameter("respect_higher_priority_path").as_bool();

  // [NEW] Add declaration for the robot list
  this->declare_parameter<std::vector<std::string>>("robot_ids", std::vector<std::string>({}));

  // [NEW] Loop 1: Declare all sub-parameters for each robot_id
  // (We get robot_ids first to declare, this is a bit redundant but safe)
  std::vector<std::string> robot_ids_to_declare;
  try {
    robot_ids_to_declare = this->get_parameter("robot_ids").as_string_array();
  } catch (...) {
    RCLCPP_WARN(get_logger(), "No 'robot_ids' list found in YAML, will not load any agent footprints.");
  }
  
  for (const auto & id_str : robot_ids_to_declare) {
    // This is a regular node, no 'name_' prefix.
    this->declare_parameter(id_str + ".machine_id", rclcpp::ParameterValue(0));
    this->declare_parameter(id_str + ".robot_radius", rclcpp::ParameterValue(0.0));
    this->declare_parameter(id_str + ".footprint", rclcpp::ParameterValue(std::string("[]")));
  }

  // [NEW] Loop 2: Get parameters and populate the map
  agent_footprints_.clear();
  std::vector<std::string> robot_ids;
  this->get_parameter("robot_ids", robot_ids); // Get the list again (now that it's declared)
  
  for (const auto & id_str : robot_ids) {
    
    int machine_id_int = 0;
    this->get_parameter(id_str + ".machine_id", machine_id_int);
    if (machine_id_int == 0) continue; 

    uint16_t machine_id = static_cast<uint16_t>(machine_id_int);
    
    AgentFootprintData data;
    std::string footprint_str;
    this->get_parameter(id_str + ".footprint", footprint_str);
    this->get_parameter(id_str + ".robot_radius", data.radius);

    std::vector<geometry_msgs::msg::Point> footprint_points;
    if (nav2_costmap_2d::makeFootprintFromString(footprint_str, footprint_points) &&
        footprint_points.size() >= 3)
    {
      data.points = toPoint32(footprint_points); // Convert Point to Point32
      data.use_radius = false;
    } else {
      data.use_radius = true;
    }

    agent_footprints_[machine_id] = data;
    
    RCLCPP_INFO(get_logger(), 
      "Loaded footprint for machine_id %u: use_radius=%s, points=%zu",
      machine_id, (data.use_radius ? "true" : "false"), data.points.size());
  }

// [NEW] 충돌 예측 상시 검증 타이머
  double val_freq = this->get_parameter("validation_frequency").as_double();
  int val_period_ms = static_cast<int>(1000.0 / std::max(1.0, val_freq));


  // ===== TF =====
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  last_replan_time_       = rclcpp::Time(0,0,this->get_clock()->get_clock_type());
  last_agent_block_time_  = rclcpp::Time(0,0,this->get_clock()->get_clock_type());

  // ===== Callback Groups =====
  subs_callback_group_  = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto subs_options = rclcpp::SubscriptionOptions();
  subs_options.callback_group = subs_callback_group_;

  // ===== Subscriptions =====
  costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
      "/global_costmap/costmap_raw",
      rclcpp::QoS(rclcpp::SystemDefaultsQoS()).transient_local().reliable(),
      std::bind(&PathValidatorNode::costmapCallback, this, _1),
      subs_options);

  if (compare_agent_mask_ && !agent_mask_topic_.empty()) {
    agent_mask_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
        agent_mask_topic_,
        rclcpp::QoS(rclcpp::SystemDefaultsQoS()).transient_local().reliable(),
        std::bind(&PathValidatorNode::agentMaskCallback, this, _1),
        subs_options);
  }

  agents_sub_ = this->create_subscription<multi_agent_msgs::msg::MultiAgentInfoArray>(
      agents_topic_,
      rclcpp::QoS(10).best_effort(),
      std::bind(&PathValidatorNode::agentsCallback, this, _1),
      subs_options);

  robot_status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot_status",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&PathValidatorNode::robotStatusCallback, this, _1),
      subs_options);

  pruned_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/plan_truncated_short",
      rclcpp::QoS(10),
      std::bind(&PathValidatorNode::validatePathCallback, this, _1),
      subs_options);

  remaining_goals_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      remaining_goals_topic,
      rclcpp::QoS(10),
      std::bind(&PathValidatorNode::remainingGoalsCallback, this, _1),
      subs_options);


  // ===== Publishers =====
  {
    static_collision_pub_ = this->create_publisher<multi_agent_msgs::msg::PathStaticCollisionInfo>(
        "/path_static_collision_info", rclcpp::QoS(10).reliable());
  }
  if (publish_agent_collision_) {
    agent_collision_pub_ = this->create_publisher<multi_agent_msgs::msg::PathAgentCollisionInfo>(
        agent_collision_topic_, rclcpp::QoS(10).reliable());
  }

  // ===== Timers =====
  const int period_ms = static_cast<int>(1000.0 / std::max(1.0, db_update_frequency_));
  obstacle_db_update_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&PathValidatorNode::updateObstacleDatabase, this),
      timer_callback_group_);


  
  validation_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(val_period_ms),
      std::bind(&PathValidatorNode::validationTimerCallback, this),
      timer_callback_group_);


  // RCLCPP_INFO(this->get_logger(),
  //   "PathValidatorNode ready. agent_path_hit_enable=%s stride=%.2f dilate=%.2f maxposes=%d",
  //   (agent_path_hit_enable_ ? "true":"false"),
  //   agent_path_hit_stride_m_, agent_path_hit_dilate_m_, agent_path_hit_max_poses_);

// =========================================================
  // [NEW] 모든 파라미터 터미널/로그 출력
  // =========================================================
  std::stringstream ss;
  ss << "\n=======================================================\n"
     << "       PathValidatorNode Parameters Loaded\n"
     << "=======================================================\n"
     << " [General & Frames]\n"
     << "  - global_frame: " << global_frame_ << "\n"
     << "  - base_frame: " << base_frame_ << "\n"
     << "  - self_machine_id: " << self_machine_id_ << "\n"
     << "  - validation_frequency: " << val_freq << " Hz\n"
     << "  - cooldown_sec: " << cooldown_sec_ << "\n"
     << " [Validation Logic]\n"
     << "  - consecutive_threshold: " << consecutive_threshold_ << "\n"
     << "  - obstacle_persistence_sec: " << obstacle_persistence_sec_ << "\n"
     << "  - cost_threshold: " << cost_threshold_ << "\n"
     << "  - max_speed: " << max_speed_ << "\n"
     << "  - lookahead_time_sec: " << lookahead_time_sec_ << "\n"
     << "  - min_lookahead_m: " << min_lookahead_m_ << "\n"
     << "  - path_check_distance_m: " << path_check_distance_m_ << "\n"
     << "  - ignore_unknown: " << (ignore_unknown_ ? "true" : "false") << "\n"
     << " [Obstacle DB (Points Mode)]\n"
     << "  - db_update_frequency: " << db_update_frequency_ << "\n"
     << "  - obstacle_prune_timeout_sec: " << obstacle_prune_timeout_sec_ << "\n"
     << "  - db_stride: " << db_stride_ << "\n"
     << "  - kernel_half_size: " << kernel_half_size_ << "\n"
     << " [Footprint Check]\n"
     << "  - use_footprint_check: " << (use_footprint_check_ ? "true" : "false") << "\n"
     << "  - footprint_step_m: " << footprint_step_m_ << "\n"
     << "  - use_radius: " << (use_radius_ ? "true" : "false") << "\n"
     << "  - robot_radius_m: " << robot_radius_m_ << "\n"
     << " [Agent Mask & Collision]\n"
     << "  - compare_agent_mask: " << (compare_agent_mask_ ? "true" : "false") << "\n"
     << "  - agent_cost_threshold: " << agent_cost_threshold_ << "\n"
     << "  - agent_mask_manhattan_buffer: " << agent_mask_manhattan_buffer_ << "\n"
     << "  - agent_match_dilate_m: " << agent_match_dilate_m_ << "\n"
     << "  - agent_block_hold_sec: " << agent_block_hold_sec_ << "\n"
     << "  - agent_block_max_wait_sec: " << agent_block_max_wait_sec_ << "\n"
     << " [Agent Path Tube Matching]\n"
     << "  - agent_path_hit_enable: " << (agent_path_hit_enable_ ? "true" : "false") << "\n"
     << "  - agent_path_hit_stride_m: " << agent_path_hit_stride_m_ << "\n"
     << "  - agent_path_hit_dilate_m: " << agent_path_hit_dilate_m_ << "\n"
     << "  - agent_path_hit_max_poses: " << agent_path_hit_max_poses_ << "\n"
     << "  - respect_higher_priority_path: " << (respect_higher_priority_path_ ? "true" : "false") << "\n"
     << " [Goal Doorstep]\n"
     << "  - goal_doorstep_static_m: " << goal_doorstep_static_m_ << "\n"
     << "  - goal_doorstep_agent_m: " << goal_doorstep_agent_m_ << "\n"
     << "=======================================================";

  // 생성된 문자열을 INFO 레벨로 출력
  RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

}

// ===================== Costmap handling =====================

void PathValidatorNode::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);

  CostmapSignature sig;
  sig.size_x    = msg->metadata.size_x;
  sig.size_y    = msg->metadata.size_y;
  sig.resolution= msg->metadata.resolution;
  sig.origin_x  = msg->metadata.origin.position.x;
  sig.origin_y  = msg->metadata.origin.position.y;

  if (!costmap_ || !(sig == last_costmap_sig_)) {
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>(
        sig.size_x, sig.size_y, sig.resolution, sig.origin_x, sig.origin_y,
        nav2_costmap_2d::FREE_SPACE);
    last_costmap_sig_ = sig;

    std::lock_guard<std::mutex> db_lock(obstacle_db_mutex_);
    obstacle_db_.clear();
  }

  const size_t expected = static_cast<size_t>(sig.size_x) * static_cast<size_t>(sig.size_y);
  if (msg->data.size() != expected) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Costmap data size mismatch: got %zu, expected %zu",
        msg->data.size(), expected);
    return;
  }
  std::memcpy(costmap_->getCharMap(), msg->data.data(), expected * sizeof(unsigned char));
}

void PathValidatorNode::agentMaskCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(agent_mask_mutex_);

  CostmapSignature sig;
  sig.size_x    = msg->metadata.size_x;
  sig.size_y    = msg->metadata.size_y;
  sig.resolution= msg->metadata.resolution;
  sig.origin_x  = msg->metadata.origin.position.x;
  sig.origin_y  = msg->metadata.origin.position.y;

  if (!agent_mask_ || !(sig == last_agent_sig_)) {
    agent_mask_ = std::make_shared<nav2_costmap_2d::Costmap2D>(
        sig.size_x, sig.size_y, sig.resolution, sig.origin_x, sig.origin_y,
        nav2_costmap_2d::FREE_SPACE);
    last_agent_sig_ = sig;
  }

  const size_t expected = static_cast<size_t>(sig.size_x) * static_cast<size_t>(sig.size_y);
  if (msg->data.size() != expected) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Agent mask data size mismatch: got %zu, expected %zu",
        msg->data.size(), expected);
    return;
  }
  std::memcpy(agent_mask_->getCharMap(), msg->data.data(), expected * sizeof(unsigned char));
}

// ===================== Agents handling =====================

void PathValidatorNode::agentsCallback(const multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(agents_mutex_);
  last_agents_ = msg;
  last_agents_stamp_ = msg->header.stamp;
}

void PathValidatorNode::robotStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string & s = msg->data;
  // PLANNING, DRIVING, PAUSED 인 경우에만 충돌 검사를 활성화
  bool valid_state = (s == "PLANNING" || s == "DRIVING" || s == "PAUSED");
  is_robot_in_driving_state_.store(valid_state);

  // 다른 상태(IDLE, CHARGING 등)인 경우 경로를 비워서 충돌이 없는 것으로 처리
  if (!valid_state) {
    std::lock_guard<std::mutex> lock(path_mutex_);
    latest_global_path_.clear();
  }
}



// ===================== Goals handling =====================

void PathValidatorNode::remainingGoalsCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (!msg) return;

  std::vector<geometry_msgs::msg::PoseStamped> goals_in_global;
  
  // 1. 전달받은 Goals가 비어있지 않다면, global_frame으로 안전하게 TF 변환
  // (기존에 작성해두신 transformPathToGlobal 함수를 재사용하면 완벽합니다!)
  if (!msg->poses.empty()) {
    transformPathToGlobal(*msg, goals_in_global);
  }

  // 2. mutex를 걸고 멤버 변수에 갱신 (Thread-safe)
  {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    current_remaining_goals_ = std::move(goals_in_global);
  }
}


// ===================== TF helpers =====================

bool PathValidatorNode::getCurrentPoseFromTF(geometry_msgs::msg::Pose & pose_out) const
{
  geometry_msgs::msg::PoseStamped base_in, base_in_global;
  base_in.header.stamp = this->now();
  base_in.header.frame_id = base_frame_;
  base_in.pose.orientation.w = 1.0;

  try {
    auto tf = tf_buffer_->lookupTransform(global_frame_, base_frame_, base_in.header.stamp,
                                          rclcpp::Duration::from_seconds(0.2));
    tf2::doTransform(base_in, base_in_global, tf);
    pose_out = base_in_global.pose;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Could not get robot pose %s->%s: %s",
        global_frame_.c_str(), base_frame_.c_str(), ex.what());
    return false;
  }
}

bool PathValidatorNode::transformToGlobal(const geometry_msgs::msg::PoseStamped & in,
                                          geometry_msgs::msg::PoseStamped & out) const
{
  if (in.header.frame_id.empty() || in.header.frame_id == global_frame_) {
    out = in; out.header.frame_id = global_frame_; return true;
  }
  try {
    auto tf = tf_buffer_->lookupTransform(global_frame_, in.header.frame_id, in.header.stamp,
                                          rclcpp::Duration::from_seconds(0.2));
    tf2::doTransform(in, out, tf);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "transformToGlobal failed %s->%s: %s",
        in.header.frame_id.c_str(), global_frame_.c_str(), ex.what());
    return false;
  }
}

void PathValidatorNode::transformPathToGlobal(const nav_msgs::msg::Path & in,
                                              std::vector<geometry_msgs::msg::PoseStamped> & out) const
{
  out.clear(); out.reserve(in.poses.size());
  for (const auto & ps : in.poses) {
    geometry_msgs::msg::PoseStamped g;
    if (transformToGlobal(ps, g)) out.emplace_back(std::move(g));
  }
}




void PathValidatorNode::validatePathOptimized(const std::vector<geometry_msgs::msg::PoseStamped> & gpath, const geometry_msgs::msg::Pose& target_goal, bool is_last_goal){
  // =========================================================
  // [TUNING PARAMETERS] 충돌 민감도 튜닝용 로컬 변수 모음
  // =========================================================
  
  // [Phase 1] 넓게 훑어볼 때 사용할 코스트 임계값 (노이즈 방지를 위해 253 권장)
  const unsigned char phase1_thr = 253; 
  
  // [Phase 1] 경로 중심선 기준 좌우로 몇 미터까지 훑어볼 것인가? (맨해튼 버퍼)
  const double phase1_buffer_m = 0.001; //0.1;   
  
  // [Phase 2] 정밀 검사 시 '충돌(Hit)'로 판정할 코스트 (완전한 물리적 충돌: 254 고정)
  const unsigned char phase2_thr = 254; 

  // [Phase 2] 충돌 예상 지점에서 뒤로 몇 미터 후퇴하여 정밀 검사를 시작할 것인가?
  const double backtrack_m = 0.55;      

  // [Phase 2] '모서리 스침'이 아닌 '경로가 꽉 막힘'으로 간주할 장애물의 물리적 면적 (m^2)
  const double heavy_block_area_m2 = 1.0; //0.0125; 

  // [NEW: Phase 2] 로봇 다각형(Footprint)을 물리적으로 확장할 여유 패딩 (m)
  // 예: 0.02 ~ 0.05를 주면 실제 로봇보다 해당 수치만큼 부풀려진 크기로 충돌을 검사합니다.
  const double phase2_footprint_padding_m = 0.0001; //0.02; 

  // =========================================================

  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (!costmap_) return;
    costmap = costmap_;
  }

  geometry_msgs::msg::Pose cur_pose;
  if (!getCurrentPoseFromTF(cur_pose)) return;

  const double max_dist_by_time = std::max(min_lookahead_m_, max_speed_ * lookahead_time_sec_);
  const double max_check_dist = std::min(max_dist_by_time, path_check_distance_m_);
  const double res = costmap->getResolution();
  
  // =========================================================
  // [Phase 1] Broad-Phase: 빠른 인덱스 스캔
  // =========================================================
  int phase1_hit_index = -1;
  double acc_dist = 0.0;
  
  const int buffer_cells = static_cast<int>(phase1_buffer_m / res); 
  const int sx = static_cast<int>(costmap->getSizeInCellsX());
  const int sy = static_cast<int>(costmap->getSizeInCellsY());

  for (size_t i = 1; i < gpath.size(); ++i) {
    const auto & p0 = gpath[i-1].pose.position;
    const auto & p1 = gpath[i].pose.position;
    acc_dist += std::hypot(p1.x - p0.x, p1.y - p0.y);
    if (acc_dist > max_check_dist) break;

    unsigned int mx, my;
    if (!costmap->worldToMap(p1.x, p1.y, mx, my)) continue;

    bool fast_hit = false;
    const int ix = static_cast<int>(mx);
    const int iy = static_cast<int>(my);

    for(int dx = -buffer_cells; dx <= buffer_cells && !fast_hit; ++dx) {
      for(int dy = -buffer_cells; dy <= buffer_cells && !fast_hit; ++dy) {
        if (std::abs(dx) + std::abs(dy) > buffer_cells) continue;
        int cx = ix + dx, cy = iy + dy;
        if(cx >= 0 && cx < sx && cy >= 0 && cy < sy) {
          const unsigned char c = costmap->getCost(cx, cy);
          if ((!ignore_unknown_ || c != nav2_costmap_2d::NO_INFORMATION) && c >= phase1_thr) {
            fast_hit = true;
          }
        }
      }
    }

    if (fast_hit) {
      phase1_hit_index = static_cast<int>(i);
      break; 
    }
  }

  if (phase1_hit_index == -1) {
    publishSafeStatus(); // [NEW] 안전 상태 퍼블리시
    return;
  }

  // =========================================================
  // [Phase 2] Narrow-Phase 준비: 백트래킹 및 패딩 적용
  // =========================================================
  int narrow_start_idx = phase1_hit_index;
  double back_dist = 0.0;

  for (int i = phase1_hit_index; i > 0; --i) {
    const auto & p0 = gpath[i-1].pose.position;
    const auto & p1 = gpath[i].pose.position;
    back_dist += std::hypot(p1.x - p0.x, p1.y - p0.y);
    narrow_start_idx = i - 1;
    if (back_dist >= backtrack_m) break;
  }

  const int HEAVY_BLOCK_THRESHOLD = std::max(1, static_cast<int>(heavy_block_area_m2 / (res * res)));

  // [NEW] 루프 진입 전, 패딩이 적용된 부풀려진 Footprint 생성 (1회만 연산)
  std::vector<geometry_msgs::msg::Point> padded_footprint;
  padded_footprint.reserve(footprint_.size());
  
  if (phase2_footprint_padding_m > 0.0 && !footprint_.empty()) {
    double cx = 0.0, cy = 0.0;
    for (const auto & p : footprint_) { cx += p.x; cy += p.y; }
    cx /= static_cast<double>(footprint_.size());
    cy /= static_cast<double>(footprint_.size());
    
    for (const auto & p : footprint_) {
      double vx = p.x - cx, vy = p.y - cy;
      double n = std::hypot(vx, vy); 
      if (n < 1e-6) n = 1.0;
      
      geometry_msgs::msg::Point q;
      q.x = p.x + phase2_footprint_padding_m * (vx / n);
      q.y = p.y + phase2_footprint_padding_m * (vy / n);
      q.z = p.z;
      padded_footprint.push_back(q);
    }
  } else {
    padded_footprint = footprint_;
  }

  size_t consecutive = 0;
  double narrow_acc = 0.0;

  // =========================================================
  // [Phase 2] Narrow-Phase 실행
  // =========================================================
  for (size_t i = narrow_start_idx; i < gpath.size(); ++i) {
    if (i > static_cast<size_t>(narrow_start_idx)) {
      const auto & p0 = gpath[i-1].pose.position;
      const auto & p1 = gpath[i].pose.position;
      narrow_acc += std::hypot(p1.x - p0.x, p1.y - p0.y);
      if (narrow_acc < footprint_step_m_) continue;
      narrow_acc = 0.0;
    }

    const auto & ps = gpath[i];
    const double yaw = tf2::getYaw(ps.pose.orientation);
    const double c = std::cos(yaw), s = std::sin(yaw);

    std::vector<geometry_msgs::msg::Point> poly_world;
    poly_world.reserve(padded_footprint.size());
    double minx=1e9, miny=1e9, maxx=-1e9, maxy=-1e9;
    
    // [수정 완료] 패딩이 적용된 footprint를 회전 및 평행이동
    for (const auto & p : padded_footprint) {
      geometry_msgs::msg::Point q;
      q.x = ps.pose.position.x + c * p.x - s * p.y;
      q.y = ps.pose.position.y + s * p.x + c * p.y;
      q.z = 0.0;
      poly_world.push_back(q);
      if(q.x < minx) minx = q.x;
      if(q.y < miny) miny = q.y;
      if(q.x > maxx) maxx = q.x;
      if(q.y > maxy) maxy = q.y;
    }

    int min_i, min_j, max_i, max_j;
    costmap->worldToMapEnforceBounds(minx, miny, min_i, min_j);
    costmap->worldToMapEnforceBounds(maxx, maxy, max_i, max_j);

    int blocked_cell_count = 0;
    unsigned int hit_mx = 0, hit_my = 0;
    double min_hit_dist_sq = 1e9; // [성능 최적화] 제곱합 비교용 변수

    for (int y = min_j; y <= max_j; ++y) {
      for (int x = min_i; x <= max_i; ++x) {
        double wx, wy; costmap->mapToWorld(x, y, wx, wy);
        
        if (!pointInPolygon(poly_world, wx, wy)) continue;
        
        const unsigned int umx = static_cast<unsigned int>(x);
        const unsigned int umy = static_cast<unsigned int>(y);
        
        const unsigned char cost = costmap->getCost(umx, umy);
        
        if ((!ignore_unknown_ || cost != nav2_costmap_2d::NO_INFORMATION) && cost >= phase2_thr) {
          blocked_cell_count++;
          
          // [성능 최적화] std::hypot 대신 유클리디안 제곱합으로 고속 비교
          double dx = wx - cur_pose.position.x;
          double dy = wy - cur_pose.position.y;
          double dist_sq = dx * dx + dy * dy;          
          
          if (dist_sq < min_hit_dist_sq) {
            min_hit_dist_sq = dist_sq;
            hit_mx = umx;
            hit_my = umy;
          }
        }
      }
    }

    // =========================================================
    // [판단 로직] 밀도 확인 및 에이전트 분류
    // =========================================================
    if (blocked_cell_count > 0) {
      if (blocked_cell_count >= HEAVY_BLOCK_THRESHOLD) {
        consecutive += consecutive_threshold_;
      } else {
        consecutive++;
      }

      if (consecutive >= consecutive_threshold_) {
        double hit_wx, hit_wy;
        costmap->mapToWorld(hit_mx, hit_my, hit_wx, hit_wy);

      // -----------------------------------------------------
        // [하이브리드 판단] 이 충돌이 Goal인가, Path인가?
        // -----------------------------------------------------
        bool is_goal_occupied = false;
        
        // isGoalBlocked 인자 정확히 채우기
        if (isGoalBlocked(costmap, target_goal, goal_doorstep_static_m_, 253)) {
          is_goal_occupied = true;
        } 
        else {
          double hit_to_goal = std::hypot(target_goal.position.x - hit_wx, 
                                          target_goal.position.y - hit_wy);
          if (hit_to_goal <= 0.5) is_goal_occupied = true;
        }

        if (is_goal_occupied) {
          goal_occupied_flag_ = true;
          locked_goal_pose_ = target_goal;
        }
      // -----------------------------------------------------

        // [NEW] 실제 Goal 점유일 때만 is_last_goal 값을 쓰고, 아니면 무조건 false 처리
        bool is_last_goal_occupied = is_goal_occupied ? is_last_goal : false;


        if (compare_agent_mask_) {
          bool agent_mark = agentCellBlockedNear(
              hit_mx, hit_my, static_cast<unsigned char>(agent_cost_threshold_), agent_mask_manhattan_buffer_);
              
          if (agent_mark) {
            const double allowed_dist = 0.424 + 0.1; 
            auto hits2 = findNearestAgent(hit_wx, hit_wy, allowed_dist);
            if (!hits2.empty()) {
              publishAgentCollisionList(hits2, is_goal_occupied, is_last_goal_occupied, target_goal); 
              last_agent_block_time_ = this->now();
              return; 
            }
          }
        } else {
          auto hits = whoCoversPoint(hit_wx, hit_wy); 
          if (!hits.empty()) {
            publishAgentCollisionList(hits, is_goal_occupied, is_last_goal_occupied, target_goal); 
            last_agent_block_time_ = this->now();
            return;
          }
        }
        
        // 2. 에이전트가 아닌 일반 장애물 판정 후 리플랜 실행 영역
        triggerReplan("blocked (optimized broad-narrow phase)", is_goal_occupied, is_last_goal_occupied, hit_wx, hit_wy, target_goal); 
        return;
      }
    } else {
      consecutive = 0;
    }
    
    // [수정 완료] 인덱스(칸 수)가 아닌, 물리적 거리(1.0m 이상)를 기준으로 조기 종료 방어 코드 적용
    if (i > static_cast<size_t>(phase1_hit_index)) {
      double dist_passed = std::hypot(
          gpath[i].pose.position.x - gpath[phase1_hit_index].pose.position.x,
          gpath[i].pose.position.y - gpath[phase1_hit_index].pose.position.y
      );
      
      if (dist_passed > 1.0 && consecutive == 0) {
          break;
      }
    }
  }
// 3. 루프를 문제없이 끝까지 다 돌고 안전하게 끝났을 때
  publishSafeStatus();
}




bool PathValidatorNode::isGoalBlocked(
    std::shared_ptr<nav2_costmap_2d::Costmap2D> cm, 
    const geometry_msgs::msg::Pose& goal_pose, 
    double buffer_m, 
    unsigned char threshold) const
{
  if (!cm) return false;

  unsigned int mx, my;
  if (!cm->worldToMap(goal_pose.position.x, goal_pose.position.y, mx, my)) {
    return false;
  }

  // 인자로 받은 실제 물리적 거리(m)를 Costmap 셀 단위로 변환
  int buffer_cells = static_cast<int>(buffer_m / cm->getResolution());
  int K = std::max(0, buffer_cells);
  int sx = static_cast<int>(cm->getSizeInCellsX());
  int sy = static_cast<int>(cm->getSizeInCellsY());

  for (int dx = -K; dx <= K; ++dx) {
    for (int dy = -K; dy <= K; ++dy) {
      int x = static_cast<int>(mx) + dx;
      int y = static_cast<int>(my) + dy;
      if (x < 0 || y < 0 || x >= sx || y >= sy) continue;
      
      // 인자로 받은 threshold 이상인지 검사
      if (cm->getCost(x, y) >= threshold) {
        return true;
      }
    }
  }
  return false;
}



// // ===================== agent mask and footprint checking =====================

// void PathValidatorNode::validateWithFootprint(const std::vector<geometry_msgs::msg::PoseStamped> & gpath)
// {
//   std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
//   {
//     std::lock_guard<std::mutex> lock(costmap_mutex_);
//     if (!costmap_) return;
//     costmap = costmap_;
//   }

//   geometry_msgs::msg::Pose cur_pose;
//   if (!getCurrentPoseFromTF(cur_pose)) return;

//   const double max_dist_by_time = std::max(min_lookahead_m_, max_speed_ * lookahead_time_sec_);
//   const double max_check_dist = std::min(max_dist_by_time, path_check_distance_m_);
//   const double res = costmap->getResolution();

//   // 경로 샘플링
//   std::vector<geometry_msgs::msg::PoseStamped> samples;
//   samples.reserve(gpath.size());
//   double acc = 0.0;
//   samples.push_back(gpath.front());
//   for (size_t i = 1; i < gpath.size(); ++i) {
//     const auto & p0 = gpath[i-1].pose.position;
//     const auto & p1 = gpath[i].pose.position;
//     const double d = std::hypot(p1.x - p0.x, p1.y - p0.y);
//     if (d <= 1e-6) continue;
//     acc += d;
//     if (acc >= footprint_step_m_) {
//       samples.push_back(gpath[i]);
//       acc = 0.0;
//     }
//     const double total = std::hypot(gpath[i].pose.position.x - gpath.front().pose.position.x,
//                                     gpath[i].pose.position.y - gpath.front().pose.position.y);
//     if (total > max_check_dist) break;
//   }

//   const unsigned char master_thr = static_cast<unsigned char>(cost_threshold_);
//   const unsigned char agent_thr  = static_cast<unsigned char>(agent_cost_threshold_);

//   size_t consecutive = 0;

//   for (const auto & ps : samples) {
//     bool blocked_here = false;
//     unsigned int hit_mx = 0, hit_my = 0;

//     if (use_radius_) {
//       unsigned int cx, cy;
//       if (!costmap->worldToMap(ps.pose.position.x, ps.pose.position.y, cx, cy)) {
//         consecutive = 0; continue;
//       }
//       const int r_cells = std::max(1, static_cast<int>(std::ceil(robot_radius_m_ / res)));

//       for (int dx = -r_cells; dx <= r_cells && !blocked_here; ++dx) {
//         for (int dy = -r_cells; dy <= r_cells && !blocked_here; ++dy) {
//           if (dx*dx + dy*dy > r_cells*r_cells) continue;
//           const int mx = static_cast<int>(cx) + dx;
//           const int my = static_cast<int>(cy) + dy;
//           if (mx < 0 || my < 0 ||
//               mx >= static_cast<int>(costmap->getSizeInCellsX()) ||
//               my >= static_cast<int>(costmap->getSizeInCellsY())) continue;

//           const unsigned int umx = static_cast<unsigned int>(mx);
//           const unsigned int umy = static_cast<unsigned int>(my);

//           if (masterCellBlocked(umx, umy, master_thr)) {
//             blocked_here = true; hit_mx = umx; hit_my = umy;
//           }
//         }
//       }
//     } else {
//       // 다각형 footprint → 월드 폴리곤
//       const double yaw = tf2::getYaw(ps.pose.orientation);
//       const double c = std::cos(yaw), s = std::sin(yaw);

//       std::vector<geometry_msgs::msg::Point> poly_world;
//       poly_world.reserve(footprint_.size());
//       double minx=1e9, miny=1e9, maxx=-1e9, maxy=-1e9;

//       for (const auto & p : footprint_) {
//         geometry_msgs::msg::Point q;
//         const double x = p.x, y = p.y;
//         q.x = ps.pose.position.x + c * x - s * y;
//         q.y = ps.pose.position.y + s * x + c * y;
//         q.z = 0.0;
//         poly_world.push_back(q);
//         if (q.x < minx) minx=q.x;
//         if (q.y < miny) miny=q.y;
//         if (q.x > maxx) maxx=q.x; 
//         if (q.y > maxy) maxy=q.y;
//       }

//       int min_i, min_j, max_i, max_j;
//       costmap->worldToMapEnforceBounds(minx, miny, min_i, min_j);
//       costmap->worldToMapEnforceBounds(maxx, maxy, max_i, max_j);

//       for (int j = min_j; j <= max_j && !blocked_here; ++j) {
//         for (int i = min_i; i <= max_i && !blocked_here; ++i) {
//           double wx, wy; costmap->mapToWorld(i, j, wx, wy);
//           if (!pointInPolygon(poly_world, wx, wy)) continue;

//           const unsigned int umx = static_cast<unsigned int>(i);
//           const unsigned int umy = static_cast<unsigned int>(j);

//           if (masterCellBlocked(umx, umy, master_thr)) {
//             blocked_here = true; hit_mx = umx; hit_my = umy;
//           }
//         }
//       }
//     }

//     if (blocked_here) {
//       // --- costmap 포인터를 잠깐 복사 (락 최소화) ---
//       std::shared_ptr<nav2_costmap_2d::Costmap2D> cm;
//       { std::lock_guard<std::mutex> lock(costmap_mutex_); cm = costmap_; }
//       if (!cm) return;

//       // --- 중심 셀(히트셀) 기준 월드좌표 ---
//       double wx, wy;
//       cm->mapToWorld(static_cast<int>(hit_mx), static_cast<int>(hit_my), wx, wy);

//       // [유지] 기존 람다 함수 (compare_agent_mask_ 가 false일 때 플랜 B로 사용)
//       auto agent_hit_around = [&](double cx, double cy,
//                                   unsigned int mx_c, unsigned int my_c) -> std::vector<AgentHit> {
//         auto hits = whoCoversPoint(cx, cy);
//         if (!hits.empty()) return hits;

//         static const int OFFS[8][2] = {
//           { 1, 0},{-1, 0},{ 0, 1},{ 0,-1},
//           { 1, 1},{ 1,-1},{-1, 1},{-1,-1}
//         };
//         for (auto &o : OFFS) {
//           double wxx, wyy;
//           cm->mapToWorld(static_cast<int>(mx_c)+o[0], static_cast<int>(my_c)+o[1], wxx, wyy);
//           auto h2 = whoCoversPoint(wxx, wyy);
//           if (!h2.empty()) return h2;
//         }
//         return {};
//       };

//       // =========================================================
//       // [수정된 판별 로직: 마스크 유무에 따른 스마트 스위칭]
//       // =========================================================
//       if (compare_agent_mask_) {
//         // [Plan A] 마스크를 사용할 때는 복잡한 다각형 검사(agent_hit_around)를 건너뛰고,
//         // 가장 빠르고 확실한 '마스크 버퍼 검사 + 거리 기반 에이전트 찾기'를 수행합니다.
//         const bool agent_mark = agentCellBlockedNear(
//             hit_mx, hit_my,
//             static_cast<unsigned char>(agent_cost_threshold_),
//             agent_mask_manhattan_buffer_);
            
//         if (agent_mark) {
//         //   auto hits2 = findNearestAgent(wx, wy);
//         // [Plan B: 깐깐한 거름망]
//           // 최대 예상 지연(예: 0.5초) * 최고 속도(예: 1.0m/s) = 0.5m의 오차 허용.
//           // 로봇 반경 + 지연 오차 마진 = 약 0.8m ~ 1.0m
//           const double MAX_LATENCY_ERROR_M = 0.1; // [meter] (필요시 파라미터로 도출)
//         //   const double allowed_dist = robot_radius_m_ + MAX_LATENCY_ERROR_M;
//           const double allowed_dist = 0.424 + MAX_LATENCY_ERROR_M; // [튜닝 필요: 로봇 최외각 0.424m + 최대 지연 오차 0.1m = 0.524m]

//           auto hits2 = findNearestAgent(wx, wy, allowed_dist);


//           if (!hits2.empty()) {
//             publishAgentCollisionList(hits2, false, geometry_msgs::msg::Pose());
//             last_agent_block_time_ = this->now();
//             // [NEW] 에이전트 대기로 인해 리플랜은 하지 않으므로 False
//             std_msgs::msg::Bool m; m.data = false;
//             replan_pub_->publish(m);
//             return;
//           }
//         }
//       } else {
//         // [Plan B] 마스크를 사용하지 않을 경우, 기존의 '기하학적 수학 검사' 로직을 호출합니다.
//         auto hits = agent_hit_around(wx, wy, hit_mx, hit_my);
//         if (!hits.empty()) {
//           publishAgentCollisionList(hits, false, false, geometry_msgs::msg::Pose());
//           last_agent_block_time_ = this->now();
//           return;
//         }
//       }

//       // 3) 여기까지 확인했는데도 에이전트가 아니면 일반 장애물로 누적 처리!
//       consecutive++;
//       if (consecutive >= consecutive_threshold_) {
//         triggerReplan("blocked (footprint) streak threshold reached", false, false, wx, wy, geometry_msgs::msg::Pose());
//         return;
//       }
//     } else {
//       consecutive = 0;
//     }
//   }
// }



// ===================== Agent mask checking =====================

std::vector<PathValidatorNode::AgentHit> PathValidatorNode::findNearestAgent(
    double wx, double wy, double max_allowed_dist) const
{
  std::vector<AgentHit> out;
  std::lock_guard<std::mutex> lk(agents_mutex_);
  if (!last_agents_) return out;

  double min_dist_overall = 1e9;
  const multi_agent_msgs::msg::MultiAgentInfo* true_owner = nullptr;

  for (const auto & a : last_agents_->agents) {
    if (a.machine_id == self_machine_id_) continue;

    // 1. 현재 위치 거리
    double d_pose = std::hypot(wx - a.current_pose.pose.position.x, 
                               wy - a.current_pose.pose.position.y);
    if (d_pose < min_dist_overall) {
      min_dist_overall = d_pose; true_owner = &a;
    }

    // 2. 경로 점 거리
    for (const auto & path_pose : a.truncated_path.poses) {
      double d_path = std::hypot(wx - path_pose.pose.position.x, 
                                 wy - path_pose.pose.position.y);
      if (d_path < min_dist_overall) {
        min_dist_overall = d_path; true_owner = &a; 
      }
    }
  }

  // [핵심 해결책] 찾은 에이전트가 "허용된 지연 오차 반경(max_allowed_dist)" 안에 있을 때만 인정!
  if (true_owner != nullptr && min_dist_overall <= max_allowed_dist) {
    AgentHit hit;
    hit.machine_id = true_owner->machine_id;
    hit.type_id = true_owner->type_id;
    hit.x = wx; hit.y = wy;

    const double head = headingTo(true_owner->current_pose.pose, wx, wy);
    const double v_along = speedAlong(true_owner->current_twist, head);
    if (v_along > 0.05) {
      hit.ttc_first = static_cast<float>(min_dist_overall / v_along);
      hit.note = "agent_mask matched (latency compensated)";
    } else {
      hit.ttc_first = -1.0f;
      hit.note = "agent_mask matched (latency compensated)";
    }
    out.push_back(hit);
  }
  
  return out;
}



// Agent mask 기반 근처 셀 검사 (맨해튼 버퍼)
// PathValidatorNode::agentCellBlockedNear() 교체 버전
inline bool PathValidatorNode::agentCellBlockedNear(
    unsigned int mx, unsigned int my,
    unsigned char thr, int manhattan_buf) const
{
  // 1) 로컬 복사 (락 순서 교착 회피)
  std::shared_ptr<nav2_costmap_2d::Costmap2D> master, agent;
  {
    std::lock_guard<std::mutex> lk(costmap_mutex_);
    if (!costmap_) return false;
    master = costmap_;
  }
  {
    std::lock_guard<std::mutex> lk(agent_mask_mutex_);
    if (!agent_mask_) return false;
    agent = agent_mask_;
  }

  // 2) master index -> world
  double wx, wy;
  master->mapToWorld(mx, my, wx, wy);

  // 3) world -> agent_mask index
  unsigned int ax, ay;
  if (!agent->worldToMap(wx, wy, ax, ay)) {
    return false;
  }

  // 4) 맨해튼 버퍼 내에서 검사
  const int sx = static_cast<int>(agent->getSizeInCellsX());
  const int sy = static_cast<int>(agent->getSizeInCellsY());
  const int ix = static_cast<int>(ax);
  const int iy = static_cast<int>(ay);

  for (int dx = -manhattan_buf; dx <= manhattan_buf; ++dx) {
    for (int dy = -manhattan_buf; dy <= manhattan_buf; ++dy) {
      if (std::abs(dx) + std::abs(dy) > manhattan_buf) continue;
      const int x = ix + dx;
      const int y = iy + dy;
      if (x < 0 || y < 0 || x >= sx || y >= sy) continue;

      const unsigned char a = agent->getCost(static_cast<unsigned int>(x),
                                             static_cast<unsigned int>(y));
      if (a >= thr) return true;
    }
  }
  return false;
}




// ===================== Obstacle DB update =====================

void PathValidatorNode::updateObstacleDatabase()
{
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (!costmap_) return;
    costmap = costmap_;
  }

  geometry_msgs::msg::Pose pose;
  if (!getCurrentPoseFromTF(pose)) return;

  const double lookahead = std::max(min_lookahead_m_, max_speed_ * lookahead_time_sec_);

  const double map_min_x = costmap->getOriginX();
  const double map_min_y = costmap->getOriginY();
  const double map_max_x = map_min_x + costmap->getSizeInCellsX() * costmap->getResolution();
  const double map_max_y = map_min_y + costmap->getSizeInCellsY() * costmap->getResolution();

  const double min_x = std::max(pose.position.x - lookahead, map_min_x);
  const double min_y = std::max(pose.position.y - lookahead, map_min_y);
  const double max_x = std::min(pose.position.x + lookahead, map_max_x);
  const double max_y = std::min(pose.position.y + lookahead, map_max_y);

  // unsigned int min_mx, min_my, max_mx, max_my;
  // if (!costmap->worldToMap(min_x, min_y, min_mx, min_my) ||
  //     !costmap->worldToMap(max_x, max_y, max_mx, max_my)) {
  //   RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "ROI outside map bounds");
  //   return;
  // }

// worldToMapEnforceBounds는 int 타입을 사용하므로 캐스팅 변수 준비
  int min_mx_i, min_my_i, max_mx_i, max_my_i;
  
  // 맵 경계를 벗어나더라도 알아서 유효한 인덱스(0 ~ size-1)로 안전하게 잘라줍니다.
  costmap->worldToMapEnforceBounds(min_x, min_y, min_mx_i, min_my_i);
  costmap->worldToMapEnforceBounds(max_x, max_y, max_mx_i, max_my_i);

  // 이후 로직(unsigned int 기반)과 호환되도록 다시 캐스팅
  unsigned int min_mx = static_cast<unsigned int>(min_mx_i);
  unsigned int min_my = static_cast<unsigned int>(min_my_i);
  unsigned int max_mx = static_cast<unsigned int>(max_mx_i);
  unsigned int max_my = static_cast<unsigned int>(max_my_i);
  
  // 이제 경고 로그를 띄우거나 return 할 필요 없이, 잘라진 ROI 내부만 안전하게 검사합니다.
    
  const rclcpp::Time now = this->now();
  std::unordered_set<uint64_t> visible;

  const double yaw = tf2::getYaw(pose.orientation);
  const double ux = std::cos(yaw), uy = std::sin(yaw);
  const double cos_half = std::cos((cone_angle_deg_ * M_PI / 180.0) * 0.5);

  for (unsigned int mx = min_mx; mx <= max_mx; mx += static_cast<unsigned int>(db_stride_)) {
    for (unsigned int my = min_my; my <= max_my; my += static_cast<unsigned int>(db_stride_)) {
      double wx, wy;
      costmap->mapToWorld(mx, my, wx, wy);

      const double dx = wx - pose.position.x;
      const double dy = wy - pose.position.y;
      const double dist = std::hypot(dx, dy);
      if (dist > lookahead) continue;

      const double ndot = (dx * ux + dy * uy) / std::max(1e-6, dist);
      if (ndot < cos_half) continue;

      const unsigned char c = costmap->getCost(mx, my);
      if (ignore_unknown_ && c == nav2_costmap_2d::NO_INFORMATION) continue;
      if (c >= static_cast<unsigned char>(cost_threshold_)) {
        visible.insert(packKey(mx, my));
      }
    }
  }

  {
    std::lock_guard<std::mutex> db_lock(obstacle_db_mutex_);
    for (const auto key : visible) {
      auto it = obstacle_db_.find(key);
      if (it == obstacle_db_.end()) {
        obstacle_db_[key] = ObstacleInfo{now, now};
      } else {
        it->second.last_seen = now;
      }
    }
    for (auto it = obstacle_db_.begin(); it != obstacle_db_.end(); ) {
      if ((now - it->second.last_seen).seconds() > obstacle_prune_timeout_sec_) {
        it = obstacle_db_.erase(it);
      } else {
        ++it;
      }
    }
  }
}

// ===================== Cell checks =====================

bool PathValidatorNode::isBlockedCellKernel(unsigned int mx, unsigned int my) const
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  if (!costmap_) return false;

  const int K = kernel_half_size_;
  const int sx = static_cast<int>(costmap_->getSizeInCellsX());
  const int sy = static_cast<int>(costmap_->getSizeInCellsY());

  for (int dx = -K; dx <= K; ++dx) {
    for (int dy = -K; dy <= K; ++dy) {
      const int x = static_cast<int>(mx) + dx;
      const int y = static_cast<int>(my) + dy;
      if (x < 0 || y < 0 || x >= sx || y >= sy) continue;

      const unsigned char c = costmap_->getCost(static_cast<unsigned int>(x), static_cast<unsigned int>(y));
      if (ignore_unknown_ && c == nav2_costmap_2d::NO_INFORMATION) continue;
      if (c >= static_cast<unsigned char>(cost_threshold_)) return true;
    }
  }
  return false;
}

inline bool PathValidatorNode::masterCellBlocked(unsigned int mx, unsigned int my, unsigned char thr) const
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  if (!costmap_) return false;

  const unsigned char c = costmap_->getCost(mx, my);
  if (ignore_unknown_ && c == nav2_costmap_2d::NO_INFORMATION) return false;
  return (c >= thr);
}

// inline bool PathValidatorNode::agentCellBlockedNear(unsigned int mx, unsigned int my,
//                                                     unsigned char thr, int manhattan_buf) const
// {
//   std::lock_guard<std::mutex> lock(agent_mask_mutex_);
//   if (!agent_mask_) return false;

//   const int sx = static_cast<int>(agent_mask_->getSizeInCellsX());
//   const int sy = static_cast<int>(agent_mask_->getSizeInCellsY());

//   const int ix = static_cast<int>(mx);
//   const int iy = static_cast<int>(my);

//   for (int dx = -manhattan_buf; dx <= manhattan_buf; ++dx) {
//     for (int dy = -manhattan_buf; dy <= manhattan_buf; ++dy) {
//       if (std::abs(dx) + std::abs(dy) > manhattan_buf) continue;
//       const int x = ix + dx;
//       const int y = iy + dy;
//       if (x < 0 || y < 0 || x >= sx || y >= sy) continue;

//       const unsigned char a = agent_mask_->getCost(static_cast<unsigned int>(x),
//                                                    static_cast<unsigned int>(y));
//       if (a >= thr) return true;
//     }
//   }
//   return false;
// }





// ===================== Path validation dispatcher =====================

// [수정 완료] 기존의 dispatcher 역할을 하던 콜백은 이제 데이터 저장만 담당합니다.
void PathValidatorNode::validatePathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (!msg) return;

  std::vector<geometry_msgs::msg::PoseStamped> gpath;
  if (!msg->poses.empty()) {
    transformPathToGlobal(*msg, gpath);
  }

  // 최신 경로만 보관
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    latest_global_path_ = std::move(gpath);
  }
}



void PathValidatorNode::validationTimerCallback()
{
  std::vector<geometry_msgs::msg::PoseStamped> gpath;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    gpath = latest_global_path_;
  }

  if (!is_robot_in_driving_state_.load()) {
    publishSafeStatus();
    return;
  }

  // 1. 현재 Remaining Goals의 0번 타겟(최종 혹은 당장 가야할 Goal)만 가져오기
  geometry_msgs::msg::Pose target_goal;
  bool has_goal = false;
  bool is_last_goal = false; // [NEW] 최종 Goal 여부 저장 변수

  {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    if (!current_remaining_goals_.empty()) {
      target_goal = current_remaining_goals_.front().pose;
      has_goal = true;
      is_last_goal = (current_remaining_goals_.size() == 1);
    }
  }

  if (!has_goal) {
    if (gpath.empty()) publishSafeStatus();
    return; // 남은 Goal이 없으면 검사 스킵
  }

  // =========================================================
  // [Phase 1.5] 타겟 락온(Target Lock-on) 유지 방어!
  // =========================================================
  // ※ 이 부분이 Path가 날아가도 False Negative를 막아주는 핵심입니다.
  if (goal_occupied_flag_.load()) {
    double dist_moved = std::hypot(
        target_goal.position.x - locked_goal_pose_.position.x,
        target_goal.position.y - locked_goal_pose_.position.y);
        
    if (dist_moved > 0.2) {
      goal_occupied_flag_ = false; // Goal이 변경됨 -> 락온 해제
    } else {
      std::shared_ptr<nav2_costmap_2d::Costmap2D> cm;
      { std::lock_guard<std::mutex> lock(costmap_mutex_); cm = costmap_; }
      
      // 유클리디안 거리가 아닌, 락온된 좌표의 Costmap 자체를 검사
      if (isGoalBlocked(cm, locked_goal_pose_, goal_doorstep_static_m_, 253)) {
        // 여전히 막혀있음.
        unsigned int mx, my;
        cm->worldToMap(locked_goal_pose_.position.x, locked_goal_pose_.position.y, mx, my);
        bool is_agent = false;
        std::vector<AgentHit> hits;
        
        if (compare_agent_mask_) {
          if (agentCellBlockedNear(mx, my, static_cast<unsigned char>(agent_cost_threshold_), agent_mask_manhattan_buffer_)) {
            hits = findNearestAgent(locked_goal_pose_.position.x, locked_goal_pose_.position.y, 0.524);
            if (!hits.empty()) is_agent = true;
          }
        } else {
          hits = whoCoversPoint(locked_goal_pose_.position.x, locked_goal_pose_.position.y);
          if (!hits.empty()) is_agent = true;
        }

        if (is_agent) {
            publishAgentCollisionList(hits, true, is_last_goal, locked_goal_pose_);
        } else {
            triggerReplan("Locked Goal still occupied", true, is_last_goal, locked_goal_pose_.position.x, locked_goal_pose_.position.y, locked_goal_pose_);        }
        return; // 🚨 여기서 리턴해야 밑의 gpath.empty() 조기 종료 로직에 빠지지 않음!
      } else {
        goal_occupied_flag_ = false; // 장애물 치워짐
      }
    }
  }

  // =========================================================
  // [Phase 2] Global Path 검사 (최초 탐지 역할)
  // =========================================================
  if (gpath.empty()) {
    publishSafeStatus();
    return;
  }

  // 홀드 타임 체크
  const rclcpp::Time now = this->now();
  const double since_agent = (now - last_agent_block_time_).seconds();
  if (since_agent >= 0.0 && since_agent < agent_block_hold_sec_) {
    return;
  }

  // 검증 로직 실행 (하이브리드 검사 진행)
  if (use_footprint_check_) {
    validatePathOptimized(gpath, target_goal, is_last_goal);
  } else {
    validateWithPoints(gpath);
  }
}

















// ===================== 기존 포인트 검사 =====================

void PathValidatorNode::validateWithPoints(const std::vector<geometry_msgs::msg::PoseStamped> & gpath)
{
  geometry_msgs::msg::Pose cur_pose;
  if (!getCurrentPoseFromTF(cur_pose)) return;

  const double max_dist_by_time = std::max(min_lookahead_m_, max_speed_ * lookahead_time_sec_);
  const double max_check_dist = std::min(max_dist_by_time, path_check_distance_m_);

  size_t best_streak = 0;
  size_t streak = 0;
  rclcpp::Time now = this->now();

  std::unordered_map<uint64_t, ObstacleInfo> db_snapshot;
  {
    std::lock_guard<std::mutex> lock(obstacle_db_mutex_);
    db_snapshot = obstacle_db_;
  }

  double acc = 0.0;
  for (size_t i = 0; i < gpath.size(); ++i) {
    if (i > 0) {
      const auto & p0 = gpath[i-1].pose.position;
      const auto & p1 = gpath[i].pose.position;
      acc += std::hypot(p1.x - p0.x, p1.y - p0.y);
      if (acc > max_check_dist) break;
    }

    unsigned int mx, my;
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      if (!costmap_->worldToMap(gpath[i].pose.position.x, gpath[i].pose.position.y, mx, my)) {
        streak = 0; continue;
      }
    }

    const bool blocked_cell = isBlockedCellKernel(mx, my);

    if (blocked_cell) {
      // --- costmap 포인터 복사 ---
      std::shared_ptr<nav2_costmap_2d::Costmap2D> cm;
      { std::lock_guard<std::mutex> lock(costmap_mutex_); cm = costmap_; }
      if (!cm) return;

      // --- 중심 셀 월드좌표 ---
      double wx, wy;
      cm->mapToWorld(static_cast<int>(mx), static_cast<int>(my), wx, wy);

      // --- 중심+이웃 1셀까지 에이전트 히트 검사 람다 ---
      auto agent_hit_around = [&](double cx, double cy,
                                  unsigned int mx_c, unsigned int my_c) -> std::vector<AgentHit> {
        // 1) 중심
        auto hits = whoCoversPoint(cx, cy);
        if (!hits.empty()) return hits;

        // 2) 8방향 이웃
        static const int OFFS[8][2] = {
          { 1, 0},{-1, 0},{ 0, 1},{ 0,-1},
          { 1, 1},{ 1,-1},{-1, 1},{-1,-1}
        };
        for (auto &o : OFFS) {
          double wxx, wyy;
          cm->mapToWorld(static_cast<int>(mx_c)+o[0], static_cast<int>(my_c)+o[1], wxx, wyy);
          auto h2 = whoCoversPoint(wxx, wyy);
          if (!h2.empty()) return h2;
        }
        return {};
      };

      // 1) 일반 코스트로 막혔을 때: 중심+이웃 검사
      {
        auto hits = agent_hit_around(wx, wy, mx, my);
        if (!hits.empty()) {
          publishAgentCollisionList(hits, false, false, geometry_msgs::msg::Pose());
          last_agent_block_time_ = this->now();
          return; // 에이전트 충돌 확정
        }
      }

      // 2) (보조) agent mask 기준으로도 보강 검사
      if (compare_agent_mask_) {
        const bool agent_mark = agentCellBlockedNear(
            mx, my,
            static_cast<unsigned char>(agent_cost_threshold_),
            agent_mask_manhattan_buffer_);
        if (agent_mark) {
          const double MAX_LATENCY_ERROR_M = 0.1; // [meter] (필요시 파라미터로 도출)
        //   const double allowed_dist = robot_radius_m_ + MAX_LATENCY_ERROR_M;
          const double allowed_dist = 0.424 + MAX_LATENCY_ERROR_M; // [튜닝 필요: 로봇 최외각 0.424m + 최대 지연 오차 0.1m = 0.524m]

          auto hits2 = findNearestAgent(wx, wy, allowed_dist);
          if (!hits2.empty()) {
            publishAgentCollisionList(hits2, false, false, geometry_msgs::msg::Pose());
            last_agent_block_time_ = this->now();
            // to do
            // [NEW] 에이전트 대기로 인해 리플랜은 하지 않으므로 False
            // std_msgs::msg::Bool m; m.data = false;
            // replan_pub_->publish(m);

            return;
          }
        }
      }

      // 3) 에이전트 히트가 아니면 이후의 일반 장애물 로직으로 진행
    }



    bool blocked = blocked_cell;
    // 일반 장애물 성숙도
    const uint64_t key = packKey(mx, my);
    bool persistent_mature = false;
    auto it = db_snapshot.find(key);
    if (it != db_snapshot.end()) {
      if ((now - it->second.first_seen).seconds() >= obstacle_persistence_sec_) {
        persistent_mature = true;
      }
    }

    blocked = blocked && persistent_mature;

    streak = blocked ? (streak + 1) : 0;
    best_streak = std::max(best_streak, streak);

    if (best_streak >= consecutive_threshold_) {
      // [수정 완료] Scope 밖에서 에러가 나지 않도록 여기서 좌표를 다시 계산해서 넘김
      double hit_wx = 0.0, hit_wy = 0.0;
      std::shared_ptr<nav2_costmap_2d::Costmap2D> cm;
      { std::lock_guard<std::mutex> lock(costmap_mutex_); cm = costmap_; }
      if (cm) {
        cm->mapToWorld(static_cast<int>(mx), static_cast<int>(my), hit_wx, hit_wy);
      }
      
      triggerReplan("blocked (points) streak threshold reached", false, false, hit_wx, hit_wy, geometry_msgs::msg::Pose());
      break;
    }
  }

  // [NEW] 루프를 무사히 돌았다면 안전 상태 퍼블리시
  publishSafeStatus();
}








// ===================== Footprint 기반 검사 =====================

bool PathValidatorNode::pointInPolygon(const std::vector<geometry_msgs::msg::Point> & poly,
                                       double x, double y)
{
  bool inside = false;
  const size_t n = poly.size();
  for (size_t i=0, j=n-1; i<n; j=i++) {
    const double xi = poly[i].x, yi = poly[i].y;
    const double xj = poly[j].x, yj = poly[j].y;
    const bool hit = ((yi > y) != (yj > y)) &&
                     (x < (xj - xi) * (y - yi) / std::max(1e-12, (yj - yi)) + xi);
    if (hit) inside = !inside;
  }
  return inside;
}





// ===================== Agent 충돌 식별 =====================

double PathValidatorNode::headingTo(const geometry_msgs::msg::Pose & pose, double wx, double wy)
{
  const double yaw = tf2::getYaw(pose.orientation);
  const double dx = wx - pose.position.x;
  const double dy = wy - pose.position.y;
  const double tgt = std::atan2(dy, dx);
  double d = tgt - yaw;
  while (d > M_PI) d -= 2*M_PI;
  while (d < -M_PI) d += 2*M_PI;
  return d;
}

double PathValidatorNode::speedAlong(const geometry_msgs::msg::Twist & tw, double heading_rad)
{
  const double v = tw.linear.x;
  return v * std::cos(heading_rad);
}

bool PathValidatorNode::pathTubeCoversPoint(const multi_agent_msgs::msg::MultiAgentInfo & a,
                                            double wx, double wy,
                                            double stride_m, double dilate_m,
                                            int max_poses, double /*frame_yaw*/,
                                            const std::string & /*global_frame*/)
{
  const auto & path = a.truncated_path;
  if (path.poses.empty()) return false;

  const int limit = std::min<int>(path.poses.size(), std::max(1, max_poses));
  // 거리 기반 스트라이드
  double acc = 0.0;
  auto prev = path.poses[0].pose.position;

  for (int i = 0; i < limit; ++i) {
    const auto & ps = path.poses[i].pose;

    if (i > 0) {
      const auto & cur = ps.position;
      acc += std::hypot(cur.x - prev.x, cur.y - prev.y);
      if (acc < std::max(0.05, stride_m)) continue;
      acc = 0.0;
      prev = cur;
    }

    // footprint를 얇게 등방성 확장한 로컬 폴리곤
    const auto & fp = a.footprint.polygon.points;
    if (fp.size() < 3) continue;

    // 로컬 dilate
    std::vector<geometry_msgs::msg::Point> poly_local; poly_local.reserve(fp.size());
    double cx=0, cy=0;
    for (auto & p : fp) { cx += p.x; cy += p.y; }
    cx /= static_cast<double>(fp.size());
    cy /= static_cast<double>(fp.size());
    for (auto & p : fp) {
      double vx = p.x - cx, vy = p.y - cy;
      double n = std::hypot(vx, vy); if (n < 1e-6) n = 1.0;
      geometry_msgs::msg::Point q;
      q.x = p.x + dilate_m * (vx / n);
      q.y = p.y + dilate_m * (vy / n);
      q.z = 0.0;
      poly_local.push_back(q);
    }

    // 로컬 → 월드 변환(경로 pose 기준, 스미어 없음)
    const double yaw = tf2::getYaw(ps.orientation);
    const double c = std::cos(yaw), s = std::sin(yaw);
    std::vector<geometry_msgs::msg::Point> poly_world; poly_world.reserve(poly_local.size());
    for (auto & p : poly_local) {
      geometry_msgs::msg::Point q;
      q.x = ps.position.x + c * p.x - s * p.y;
      q.y = ps.position.y + s * p.x + c * p.y;
      q.z = 0.0;
      poly_world.push_back(q);
    }

    if (pointInPolygon(poly_world, wx, wy)) {
      return true;
    }
  }
  return false;
}

std::vector<PathValidatorNode::AgentHit> PathValidatorNode::whoCoversPoint(double wx, double wy) const
{
  std::vector<AgentHit> out;

  std::lock_guard<std::mutex> lk(agents_mutex_);
  if (!last_agents_) return out;

  if ((this->now() - last_agents_stamp_).nanoseconds() >
       static_cast<int64_t>(agents_freshness_timeout_ms_) * 1000000LL) {
    return out;
  }

  if (!last_agents_->header.frame_id.empty() &&
      last_agents_->header.frame_id != global_frame_) {
    return out;
  }

  for (const auto & a : last_agents_->agents) {
    // [add] 대상이 나 자신이면 연산에서 완전히 제외하고 건너뜀
    if (a.machine_id == self_machine_id_) {
      continue;
    }

    const auto fp = getFootprintForAgent(a);
    if (fp.size() < 3) continue;

    // 1) 현재 위치 footprint(소확장) 커버?
    {
      std::vector<geometry_msgs::msg::Point> poly_local; poly_local.reserve(fp.size());
      double cx=0, cy=0;
      for (auto & p : fp) { cx += p.x; cy += p.y; }
      cx /= static_cast<double>(fp.size());
      cy /= static_cast<double>(fp.size());
      for (auto & p : fp) {
        double vx = p.x - cx, vy = p.y - cy;
        double n = std::hypot(vx, vy); if (n < 1e-6) n = 1.0;
        geometry_msgs::msg::Point q;
        q.x = p.x + agent_match_dilate_m_ * (vx / n);
        q.y = p.y + agent_match_dilate_m_ * (vy / n);
        q.z = 0.0;
        poly_local.push_back(q);
      }

      const double yaw = tf2::getYaw(a.current_pose.pose.orientation);
      const double c = std::cos(yaw), s = std::sin(yaw);
      std::vector<geometry_msgs::msg::Point> poly_world; poly_world.reserve(poly_local.size());
      for (auto & p : poly_local) {
        geometry_msgs::msg::Point q;
        q.x = a.current_pose.pose.position.x + c * p.x - s * p.y;
        q.y = a.current_pose.pose.position.y + s * p.x + c * p.y;
        q.z = 0.0;
        poly_world.push_back(q);
      }

      if (pointInPolygon(poly_world, wx, wy)) {
        AgentHit hit;
        hit.machine_id = a.machine_id;
        hit.type_id = a.type_id;
        hit.x = wx; hit.y = wy;

        const double head = headingTo(a.current_pose.pose, wx, wy);
        const double v_along = speedAlong(a.current_twist, head);
        if (v_along > 0.05) {
          const double dx = wx - a.current_pose.pose.position.x;
          const double dy = wy - a.current_pose.pose.position.y;
          const double dist = std::hypot(dx, dy);
          hit.ttc_first = static_cast<float>(dist / v_along);
          hit.note = "agent footprint overlap; TTC estimated";
        } else {
          hit.ttc_first = -1.0f;
          hit.note = "agent footprint overlap; TTC unknown";
        }
        out.emplace_back(std::move(hit));
        continue; // footprint에 걸리면 굳이 path 튜브 검사 불필요
      }
    }

    // 2) (NEW) truncated_path 튜브 커버?
    // if (agent_path_hit_enable_) {
    // if (agent_path_hit_enable_ && (a.machine_id > self_machine_id_)) {
    // [수정] 파라미터에 따라 검사 조건 분기
    bool check_path_tube = agent_path_hit_enable_;
    
    // respect_higher_priority_path_가 false이고 대상이 나보다 ID가 작으면 검사하지 않음
    if (!respect_higher_priority_path_ && (a.machine_id < self_machine_id_)) {
      check_path_tube = false;
    }

    if (check_path_tube) {    
      const bool covered = pathTubeCoversPoint(
          a, wx, wy,
          agent_path_hit_stride_m_,
          agent_path_hit_dilate_m_,
          agent_path_hit_max_poses_,
          /*frame_yaw=*/0.0, global_frame_);
      if (covered) {
        AgentHit hit;
        hit.machine_id = a.machine_id;
        hit.type_id = a.type_id;
        hit.x = wx; hit.y = wy;
        hit.ttc_first = -1.0f; // 경로 포즈에서 TTC는 애매 → -1로
        hit.note = "agent truncated_path overlap";
        out.emplace_back(std::move(hit));
      }
    }
  }

  return out;
}


// ===================== Replan pulse =====================


void PathValidatorNode::publishAgentCollisionList(const std::vector<AgentHit> & hits, bool is_goal_occupied, bool is_last_goal_occupied, const geometry_msgs::msg::Pose& target_goal)
{
  if (!publish_agent_collision_ || !agent_collision_pub_) return;

  multi_agent_msgs::msg::PathAgentCollisionInfo msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = global_frame_;

  msg.is_goal_occupied = is_goal_occupied;
  msg.is_last_goal_occupied = is_last_goal_occupied;
  msg.target_goal = target_goal;

  if (hits.empty()) {
    msg.note.push_back("non_collision");
    agent_collision_pub_->publish(msg);
    return;
  }

  std::string agent_ids = "";
  for (const auto & h : hits) {
    msg.machine_id.push_back(h.machine_id);
    msg.type_id.push_back(h.type_id);
    msg.x.push_back(h.x);
    msg.y.push_back(h.y);
    msg.ttc_first.push_back(h.ttc_first);
    msg.note.push_back(h.note);
    agent_ids += std::to_string(h.machine_id) + "(" + h.type_id + ") ";    
  }

  if (!hits.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Publishing Agent Collision Info: [ %s] (Total: %zu hits)", 
      agent_ids.c_str(), hits.size());
  }

  agent_collision_pub_->publish(msg);
}

void PathValidatorNode::triggerReplan(const std::string & reason, bool is_goal_occupied, bool is_last_goal_occupied, double hit_x, double hit_y, const geometry_msgs::msg::Pose& target_goal)
{
  const rclcpp::Time now = this->now();

  const double since_agent = (now - last_agent_block_time_).seconds();
  if (since_agent >= 0.0 && since_agent < agent_block_hold_sec_) {
    return;
  }

  if ((now - last_replan_time_).seconds() <= cooldown_sec_) return;
  last_replan_time_ = now;

  multi_agent_msgs::msg::PathStaticCollisionInfo m;
  m.header.stamp = this->now();
  m.header.frame_id = global_frame_;
  m.replan_request = true;
  m.is_goal_occupied = is_goal_occupied;
  m.is_last_goal_occupied = is_last_goal_occupied;
  m.hit_x = hit_x;
  m.hit_y = hit_y;
  m.target_goal = target_goal;

  static_collision_pub_->publish(m);
  RCLCPP_WARN(this->get_logger(), "Triggering replan: %s", reason.c_str());

  if (publish_false_pulse_ && flag_pulse_ms_ > 0) {
    flag_reset_timer_.reset();
    auto weak_pub = std::weak_ptr<rclcpp::Publisher<multi_agent_msgs::msg::PathStaticCollisionInfo>>(static_collision_pub_);
    flag_reset_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(flag_pulse_ms_),
        [weak_pub]() {
          if (auto pub = weak_pub.lock()) {
            multi_agent_msgs::msg::PathStaticCollisionInfo off;
            off.replan_request = false;
            off.is_goal_occupied = false;
            off.is_last_goal_occupied = false;
            pub->publish(off);
          }
        });
  }
}

// [NEW] 아무 장애물도, 에이전트도 없을 때 호출
void PathValidatorNode::publishSafeStatus()
{
  multi_agent_msgs::msg::PathStaticCollisionInfo m;
  m.header.stamp = this->now();
  m.header.frame_id = global_frame_;
  m.replan_request = false;
  m.is_goal_occupied = false;
  m.is_last_goal_occupied = false;
  m.hit_x = 0.0;
  m.hit_y = 0.0;
  static_collision_pub_->publish(m);

  publishAgentCollisionList({}, false, false, geometry_msgs::msg::Pose()); // 에이전트 쪽도 클리어
}



}  // namespace replan_monitor
