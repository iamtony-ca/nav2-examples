#include "replan_monitor/path_validator_node.hpp"
#include <nav2_costmap_2d/footprint.hpp>

using std::placeholders::_1;

namespace replan_monitor
{

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

std::vector<geometry_msgs::msg::Point32> 
PathValidatorNode::getFootprintForAgent(const multi_agent_msgs::msg::MultiAgentInfo & a) const
{
    auto it = agent_footprints_.find(a.machine_id);
    if (it == agent_footprints_.end()) {
        RCLCPP_WARN_ONCE(get_logger(), 
          "No footprint data found in YAML for machine_id %u. Cannot check agent collision.",
          a.machine_id);
        return {};
    }
    const auto& data = it->second;
    if (data.use_radius) {
        std::vector<geometry_msgs::msg::Point> points = nav2_costmap_2d::makeFootprintFromRadius(data.radius);
        return toPoint32(points);
    } else {
        return data.points;
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

  // (유지) 사용하지 않더라도 YAML 오류 방지를 위해 선언 유지
  this->declare_parameter("publish_false_pulse", false); //true
  this->declare_parameter("flag_pulse_ms", 120);

  // ===== Footprint / Agent mask / Output =====
  this->declare_parameter("use_footprint_check", true); // false
  this->declare_parameter("footprint_step_m", 0.15);

  this->declare_parameter("compare_agent_mask", true);
  this->declare_parameter<std::string>("agent_mask_topic", "/agent_layer/costmap_raw");
  this->declare_parameter<std::string>("static_map_topic", "/map"); // [NEW] 정적 맵 토픽
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
  this->declare_parameter("lookahead_margin_m", 0.15); // [NEW] 전방 주시 마진

  // 에이전트 홀드
  this->declare_parameter("agent_block_hold_sec", 2.0);
  this->declare_parameter("agent_block_max_wait_sec", 8.0);

  // === NEW: 에이전트 경로 튜브 매칭 ===
  this->declare_parameter("agent_path_hit_enable", true);
  this->declare_parameter("agent_path_hit_stride_m", 0.35);
  this->declare_parameter("agent_path_hit_dilate_m", 0.05);
  this->declare_parameter("agent_path_hit_max_poses", 1000);

  this->declare_parameter("respect_higher_priority_path", false);

  // [NEW] Add declaration for the robot list
  this->declare_parameter<std::vector<std::string>>("robot_ids", std::vector<std::string>({}));

  std::vector<std::string> robot_ids_to_declare;
  try {
    robot_ids_to_declare = this->get_parameter("robot_ids").as_string_array();
  } catch (...) {
    RCLCPP_WARN(get_logger(), "No 'robot_ids' list found in YAML, will not load any agent footprints.");
  }
  
  for (const auto & id_str : robot_ids_to_declare) {
    this->declare_parameter(id_str + ".machine_id", rclcpp::ParameterValue(0));
    this->declare_parameter(id_str + ".robot_radius", rclcpp::ParameterValue(0.0));
    this->declare_parameter(id_str + ".footprint", rclcpp::ParameterValue(std::string("[]")));
  }

  // ---- load parameters ----
  global_frame_               = this->get_parameter("global_frame").as_string();
  base_frame_                 = this->get_parameter("base_frame").as_string();
  self_machine_id_            = static_cast<uint16_t>(this->get_parameter("self_machine_id").as_int());

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

  // 변수 할당 (구버전 파라미터 복구)
  publish_false_pulse_        = this->get_parameter("publish_false_pulse").as_bool();
  flag_pulse_ms_              = static_cast<int>(this->get_parameter("flag_pulse_ms").as_int());

  use_footprint_check_        = this->get_parameter("use_footprint_check").as_bool();
  footprint_step_m_           = std::max(0.05, this->get_parameter("footprint_step_m").as_double());

  compare_agent_mask_         = this->get_parameter("compare_agent_mask").as_bool();
  agent_mask_topic_           = this->get_parameter("agent_mask_topic").as_string();
  static_map_topic_           = this->get_parameter("static_map_topic").as_string();
  agent_cost_threshold_       = this->get_parameter("agent_cost_threshold").as_double();
  agent_mask_manhattan_buffer_= std::max<int>(0, static_cast<int>(this->get_parameter("agent_mask_manhattan_buffer").as_int()));

  publish_agent_collision_    = this->get_parameter("publish_agent_collision").as_bool();
  agent_collision_topic_      = this->get_parameter("agent_collision_topic").as_string();

  agents_topic_               = this->get_parameter("agents_topic").as_string();
  agents_freshness_timeout_ms_= this->get_parameter("agents_freshness_timeout_ms").as_int();
  agent_match_dilate_m_       = this->get_parameter("agent_match_dilate_m").as_double();

  // ---------------------------------------------------------
  // [수정] Nav2 footprint / radius (정사각형/다각형 지원 강화)
  // ---------------------------------------------------------
  footprint_str_              = this->get_parameter("footprint").as_string();
  robot_radius_m_             = this->get_parameter("robot_radius").as_double();
  lookahead_margin_m_         = this->get_parameter("lookahead_margin_m").as_double();
  use_radius_                 = true;
  footprint_.clear();

  if (!footprint_str_.empty() && footprint_str_ != "[]") {
    if (nav2_costmap_2d::makeFootprintFromString(footprint_str_, footprint_) && footprint_.size() >= 3) {
      use_radius_ = false;
      
      // [NEW] 정사각형 등 다각형일 경우, Look-ahead를 위해 외접 반경(최대 거리)을 계산하여 robot_radius_m_ 덮어쓰기
      double max_dist = 0.0;
      for (const auto& p : footprint_) {
          max_dist = std::max(max_dist, std::hypot(p.x, p.y));
      }
      robot_radius_m_ = max_dist; 

      RCLCPP_INFO(get_logger(), "Using polygon footprint with %zu points. Calc circumscribed radius: %.3fm", 
                  footprint_.size(), robot_radius_m_);
    } else {
      RCLCPP_ERROR(get_logger(),
        "Invalid footprint string: \"%s\". Falling back to robot_radius=%.3f",
        footprint_str_.c_str(), robot_radius_m_);
      use_radius_ = true;
    }
  } else {
    RCLCPP_INFO(get_logger(), "No valid footprint provided. Using robot_radius=%.3f", robot_radius_m_);
  }

  // 홀드 파라미터 및 경로 튜브 매칭
  agent_block_hold_sec_     = this->get_parameter("agent_block_hold_sec").as_double();
  agent_block_max_wait_sec_ = this->get_parameter("agent_block_max_wait_sec").as_double();
  agent_path_hit_enable_      = this->get_parameter("agent_path_hit_enable").as_bool();
  agent_path_hit_stride_m_    = this->get_parameter("agent_path_hit_stride_m").as_double();
  agent_path_hit_dilate_m_    = this->get_parameter("agent_path_hit_dilate_m").as_double();
  agent_path_hit_max_poses_   = this->get_parameter("agent_path_hit_max_poses").as_int();
  respect_higher_priority_path_ = this->get_parameter("respect_higher_priority_path").as_bool();

  // 타 로봇 발자국 로드
  agent_footprints_.clear();
  std::vector<std::string> robot_ids;
  this->get_parameter("robot_ids", robot_ids); 
  
  for (const auto & id_str : robot_ids) {
    int machine_id_int = 0;
    this->get_parameter(id_str + ".machine_id", machine_id_int);
    if (machine_id_int == 0) continue; 

    AgentFootprintData data;
    std::string fp_str;
    this->get_parameter(id_str + ".footprint", fp_str);
    this->get_parameter(id_str + ".robot_radius", data.radius);

    std::vector<geometry_msgs::msg::Point> fp_pts;
    if (nav2_costmap_2d::makeFootprintFromString(fp_str, fp_pts) && fp_pts.size() >= 3) {
      data.points = toPoint32(fp_pts);
      data.use_radius = false;
    } else {
      data.use_radius = true;
    }
    agent_footprints_[static_cast<uint16_t>(machine_id_int)] = data;
  }

  // ===== TF 및 내부 변수 초기화 =====
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  last_agent_block_time_  = rclcpp::Time(0,0,this->get_clock()->get_clock_type());

  subs_callback_group_  = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto subs_options = rclcpp::SubscriptionOptions();
  subs_options.callback_group = subs_callback_group_;

  // ===== 구독 (Subscriptions) =====
  costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
      "/global_costmap/costmap_raw", rclcpp::QoS(rclcpp::SystemDefaultsQoS()).transient_local().reliable(),
      std::bind(&PathValidatorNode::costmapCallback, this, _1), subs_options);

  if (compare_agent_mask_ && !agent_mask_topic_.empty()) {
    agent_mask_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
        agent_mask_topic_, rclcpp::QoS(rclcpp::SystemDefaultsQoS()).transient_local().reliable(),
        std::bind(&PathValidatorNode::agentMaskCallback, this, _1), subs_options);
  }

  if (!static_map_topic_.empty()) {
    static_map_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
        static_map_topic_, rclcpp::QoS(rclcpp::SystemDefaultsQoS()).transient_local().reliable(),
        std::bind(&PathValidatorNode::staticMapCallback, this, _1), subs_options);
  }

  agents_sub_ = this->create_subscription<multi_agent_msgs::msg::MultiAgentInfoArray>(
      agents_topic_, rclcpp::QoS(10).best_effort(),
      std::bind(&PathValidatorNode::agentsCallback, this, _1), subs_options);

  robot_status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot_status", rclcpp::SystemDefaultsQoS(),
      std::bind(&PathValidatorNode::robotStatusCallback, this, _1), subs_options);

  pruned_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/plan_pruned", rclcpp::QoS(10),
      std::bind(&PathValidatorNode::pathCallback, this, _1), subs_options);

  // ===== 발행 (Publishers) =====
  block_status_pub_ = this->create_publisher<std_msgs::msg::Int8>("/path_block_status", rclcpp::QoS(10).reliable());
  if (publish_agent_collision_) {
    agent_collision_pub_ = this->create_publisher<multi_agent_msgs::msg::PathAgentCollisionInfo>(
        agent_collision_topic_, rclcpp::QoS(10).reliable());
  }

  // ===== 타이머 (Timers) =====
  obstacle_db_update_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1.0, db_update_frequency_))),
      std::bind(&PathValidatorNode::updateObstacleDatabase, this), timer_callback_group_);

  validation_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PathValidatorNode::validationTimerCallback, this), timer_callback_group_);


// ==========================================================
  // [NEW] Parameter Info Logging (전체 파라미터 반영)
  // ==========================================================
  RCLCPP_INFO(this->get_logger(), "========== PathValidatorNode Parameters ==========");
  
  RCLCPP_INFO(this->get_logger(), " [Frame & ID]");
  RCLCPP_INFO(this->get_logger(), "  - global_frame                 : %s", global_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "  - base_frame                   : %s", base_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "  - self_machine_id              : %u", self_machine_id_);
  
  RCLCPP_INFO(this->get_logger(), " [Core Limits & Timeouts]");
  RCLCPP_INFO(this->get_logger(), "  - cooldown_sec                 : %.2f", cooldown_sec_);
  RCLCPP_INFO(this->get_logger(), "  - consecutive_threshold        : %zu", consecutive_threshold_);
  RCLCPP_INFO(this->get_logger(), "  - obstacle_persistence_sec     : %.2f", obstacle_persistence_sec_);
  RCLCPP_INFO(this->get_logger(), "  - max_speed                    : %.2f", max_speed_);
  RCLCPP_INFO(this->get_logger(), "  - cost_threshold               : %.1f", cost_threshold_);
  RCLCPP_INFO(this->get_logger(), "  - ignore_unknown               : %s", ignore_unknown_ ? "true" : "false");

  RCLCPP_INFO(this->get_logger(), " [Path & Lookahead]");
  RCLCPP_INFO(this->get_logger(), "  - path_check_distance_m        : %.2f", path_check_distance_m_);
  RCLCPP_INFO(this->get_logger(), "  - min_lookahead_m              : %.2f", min_lookahead_m_);
  RCLCPP_INFO(this->get_logger(), "  - lookahead_time_sec           : %.2f", lookahead_time_sec_);
  RCLCPP_INFO(this->get_logger(), "  - lookahead_margin_m           : %.3f", lookahead_margin_m_);

  RCLCPP_INFO(this->get_logger(), " [Obstacle Database (ROI)]");
  RCLCPP_INFO(this->get_logger(), "  - db_update_frequency          : %.2f", db_update_frequency_);
  RCLCPP_INFO(this->get_logger(), "  - obstacle_prune_timeout_sec   : %.2f", obstacle_prune_timeout_sec_);
  RCLCPP_INFO(this->get_logger(), "  - db_stride                    : %d", db_stride_);
  RCLCPP_INFO(this->get_logger(), "  - cone_angle_deg               : %.2f", cone_angle_deg_);
  RCLCPP_INFO(this->get_logger(), "  - kernel_half_size             : %d", kernel_half_size_);

  RCLCPP_INFO(this->get_logger(), " [Footprint & Masks]");
  RCLCPP_INFO(this->get_logger(), "  - use_footprint_check          : %s", use_footprint_check_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  - footprint_step_m             : %.3f", footprint_step_m_);
  RCLCPP_INFO(this->get_logger(), "  - robot_radius_m (calc)        : %.3f (use_radius: %s)", robot_radius_m_, use_radius_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  - static_map_topic             : %s", static_map_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  - compare_agent_mask           : %s", compare_agent_mask_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  - agent_mask_topic             : %s", agent_mask_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  - agent_cost_threshold         : %.1f", agent_cost_threshold_);
  RCLCPP_INFO(this->get_logger(), "  - agent_mask_manhattan_buffer  : %d", agent_mask_manhattan_buffer_);
  
  RCLCPP_INFO(this->get_logger(), " [Agent & Priority]");
  RCLCPP_INFO(this->get_logger(), "  - agents_topic                 : %s", agents_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  - agents_freshness_timeout_ms  : %d", agents_freshness_timeout_ms_);
  RCLCPP_INFO(this->get_logger(), "  - publish_agent_collision      : %s", publish_agent_collision_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  - agent_collision_topic        : %s", agent_collision_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  - agent_match_dilate_m         : %.2f", agent_match_dilate_m_);
  RCLCPP_INFO(this->get_logger(), "  - agent_block_hold_sec         : %.2f", agent_block_hold_sec_);
  RCLCPP_INFO(this->get_logger(), "  - agent_block_max_wait_sec     : %.2f", agent_block_max_wait_sec_);
  RCLCPP_INFO(this->get_logger(), "  - agent_path_hit_enable        : %s", agent_path_hit_enable_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  - agent_path_hit_stride_m      : %.2f", agent_path_hit_stride_m_);
  RCLCPP_INFO(this->get_logger(), "  - agent_path_hit_dilate_m      : %.2f", agent_path_hit_dilate_m_);
  RCLCPP_INFO(this->get_logger(), "  - agent_path_hit_max_poses     : %d", agent_path_hit_max_poses_);
  RCLCPP_INFO(this->get_logger(), "  - respect_higher_priority_path : %s", respect_higher_priority_path_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  - loaded_agent_footprints      : %zu", agent_footprints_.size());

  RCLCPP_INFO(this->get_logger(), " [Legacy / Misc]");
  RCLCPP_INFO(this->get_logger(), "  - publish_false_pulse          : %s", publish_false_pulse_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  - flag_pulse_ms                : %d", flag_pulse_ms_);

  RCLCPP_INFO(this->get_logger(), "====================================================");

  RCLCPP_INFO(this->get_logger(), "PathValidatorNode (10Hz Monitor & 6-Status Mode) Ready.");
}



void PathValidatorNode::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  CostmapSignature sig{msg->metadata.size_x, msg->metadata.size_y, msg->metadata.resolution, msg->metadata.origin.position.x, msg->metadata.origin.position.y};
  if (!costmap_ || !(sig == last_costmap_sig_)) {
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>(sig.size_x, sig.size_y, sig.resolution, sig.origin_x, sig.origin_y, nav2_costmap_2d::FREE_SPACE);
    last_costmap_sig_ = sig;
    std::lock_guard<std::mutex> db_lock(obstacle_db_mutex_);
    obstacle_db_.clear();
  }
  std::memcpy(costmap_->getCharMap(), msg->data.data(), msg->data.size());
}

void PathValidatorNode::agentMaskCallback(const nav2_msgs::msg::Costmap::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(agent_mask_mutex_);
  CostmapSignature sig{msg->metadata.size_x, msg->metadata.size_y, msg->metadata.resolution, msg->metadata.origin.position.x, msg->metadata.origin.position.y};
  if (!agent_mask_ || !(sig == last_agent_sig_)) {
    agent_mask_ = std::make_shared<nav2_costmap_2d::Costmap2D>(sig.size_x, sig.size_y, sig.resolution, sig.origin_x, sig.origin_y, nav2_costmap_2d::FREE_SPACE);
    last_agent_sig_ = sig;
  }
  std::memcpy(agent_mask_->getCharMap(), msg->data.data(), msg->data.size());
}

// [NEW] 정적 맵 처리 콜백
void PathValidatorNode::staticMapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(static_map_mutex_);
  CostmapSignature sig{msg->metadata.size_x, msg->metadata.size_y, msg->metadata.resolution, msg->metadata.origin.position.x, msg->metadata.origin.position.y};
  if (!static_map_ || !(sig == last_static_sig_)) {
    static_map_ = std::make_shared<nav2_costmap_2d::Costmap2D>(sig.size_x, sig.size_y, sig.resolution, sig.origin_x, sig.origin_y, nav2_costmap_2d::FREE_SPACE);
    last_static_sig_ = sig;
  }
  std::memcpy(static_map_->getCharMap(), msg->data.data(), msg->data.size());
}

void PathValidatorNode::agentsCallback(const multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(agents_mutex_);
  last_agents_ = msg;
  last_agents_stamp_ = msg->header.stamp;
}

void PathValidatorNode::robotStatusCallback(const std_msgs::msg::String::SharedPtr msg) {
  const std::string & s = msg->data;
  // [수정] PAUSE, WAIT 상태에서도 타이머가 돌아야 함
  is_robot_in_driving_state_.store(s == "DRIVING" || s == "PLANNING" || s == "PAUSE" || s == "WAIT" || s == "WAITING");
  
  if (s == "IDLE" || s == "MANUAL") {
      std::lock_guard<std::mutex> lock(path_mutex_);
      last_path_.reset(); // 정지 상태면 경로 폐기
  }
}

bool PathValidatorNode::getCurrentPoseFromTF(geometry_msgs::msg::Pose & pose_out) const {
  geometry_msgs::msg::PoseStamped base_in, base_in_global;
  base_in.header.stamp = this->now();
  base_in.header.frame_id = base_frame_;
  base_in.pose.orientation.w = 1.0;
  try {
    auto tf = tf_buffer_->lookupTransform(global_frame_, base_frame_, base_in.header.stamp, rclcpp::Duration::from_seconds(0.2));
    tf2::doTransform(base_in, base_in_global, tf);
    pose_out = base_in_global.pose;
    return true;
  } catch (...) { return false; }
}

bool PathValidatorNode::transformToGlobal(const geometry_msgs::msg::PoseStamped & in, geometry_msgs::msg::PoseStamped & out) const {
  if (in.header.frame_id.empty() || in.header.frame_id == global_frame_) {
    out = in; out.header.frame_id = global_frame_; return true;
  }
  try {
    auto tf = tf_buffer_->lookupTransform(global_frame_, in.header.frame_id, in.header.stamp, rclcpp::Duration::from_seconds(0.2));
    tf2::doTransform(in, out, tf);
    return true;
  } catch (...) { return false; }
}

void PathValidatorNode::transformPathToGlobal(const nav_msgs::msg::Path & in, std::vector<geometry_msgs::msg::PoseStamped> & out) const {
  out.clear(); out.reserve(in.poses.size());
  for (const auto & ps : in.poses) {
    geometry_msgs::msg::PoseStamped g;
    if (transformToGlobal(ps, g)) out.emplace_back(std::move(g));
  }
}

void PathValidatorNode::updateObstacleDatabase() {
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
  { std::lock_guard<std::mutex> lock(costmap_mutex_); if (!costmap_) return; costmap = costmap_; }
  geometry_msgs::msg::Pose pose;
  if (!getCurrentPoseFromTF(pose)) return;

  const double lookahead = std::max(min_lookahead_m_, max_speed_ * lookahead_time_sec_);
  const double map_min_x = costmap->getOriginX(), map_min_y = costmap->getOriginY();
  const double map_max_x = map_min_x + costmap->getSizeInCellsX() * costmap->getResolution();
  const double map_max_y = map_min_y + costmap->getSizeInCellsY() * costmap->getResolution();

  const double min_x = std::max(pose.position.x - lookahead, map_min_x);
  const double min_y = std::max(pose.position.y - lookahead, map_min_y);
  const double max_x = std::min(pose.position.x + lookahead, map_max_x);
  const double max_y = std::min(pose.position.y + lookahead, map_max_y);

  unsigned int min_mx, min_my, max_mx, max_my;
  if (!costmap->worldToMap(min_x, min_y, min_mx, min_my) || !costmap->worldToMap(max_x, max_y, max_mx, max_my)) return;

  const rclcpp::Time now = this->now();
  std::unordered_set<uint64_t> visible;
  const double yaw = tf2::getYaw(pose.orientation), ux = std::cos(yaw), uy = std::sin(yaw);
  const double cos_half = std::cos((cone_angle_deg_ * M_PI / 180.0) * 0.5);

  for (unsigned int mx = min_mx; mx <= max_mx; mx += db_stride_) {
    for (unsigned int my = min_my; my <= max_my; my += db_stride_) {
      double wx, wy; costmap->mapToWorld(mx, my, wx, wy);
      const double dx = wx - pose.position.x, dy = wy - pose.position.y, dist = std::hypot(dx, dy);
      if (dist > lookahead) continue;
      if ((dx * ux + dy * uy) / std::max(1e-6, dist) < cos_half) continue;
      const unsigned char c = costmap->getCost(mx, my);
      if (ignore_unknown_ && c == nav2_costmap_2d::NO_INFORMATION) continue;
      if (c >= cost_threshold_) visible.insert(packKey(mx, my));
    }
  }

  {
    std::lock_guard<std::mutex> db_lock(obstacle_db_mutex_);
    for (const auto key : visible) {
      auto it = obstacle_db_.find(key);
      if (it == obstacle_db_.end()) obstacle_db_[key] = ObstacleInfo{now, now};
      else it->second.last_seen = now;
    }
    for (auto it = obstacle_db_.begin(); it != obstacle_db_.end(); ) {
      if ((now - it->second.last_seen).seconds() > obstacle_prune_timeout_sec_) it = obstacle_db_.erase(it);
      else ++it;
    }
  }
}

bool PathValidatorNode::isBlockedCellKernel(unsigned int mx, unsigned int my) const {
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  if (!costmap_) return false;
  const int sx = static_cast<int>(costmap_->getSizeInCellsX()), sy = static_cast<int>(costmap_->getSizeInCellsY());
  for (int dx = -kernel_half_size_; dx <= kernel_half_size_; ++dx) {
    for (int dy = -kernel_half_size_; dy <= kernel_half_size_; ++dy) {
      const int x = static_cast<int>(mx) + dx, y = static_cast<int>(my) + dy;
      if (x < 0 || y < 0 || x >= sx || y >= sy) continue;
      const unsigned char c = costmap_->getCost(x, y);
      if (ignore_unknown_ && c == nav2_costmap_2d::NO_INFORMATION) continue;
      if (c >= cost_threshold_) return true;
    }
  }
  return false;
}

inline bool PathValidatorNode::masterCellBlocked(unsigned int mx, unsigned int my, unsigned char thr) const {
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  if (!costmap_) return false;
  const unsigned char c = costmap_->getCost(mx, my);
  if (ignore_unknown_ && c == nav2_costmap_2d::NO_INFORMATION) return false;
  return (c >= thr);
}

inline bool PathValidatorNode::agentCellBlockedNear(unsigned int mx, unsigned int my, unsigned char thr, int manhattan_buf) const {
  std::shared_ptr<nav2_costmap_2d::Costmap2D> master, agent;
  { std::lock_guard<std::mutex> lk(costmap_mutex_); if (!costmap_) return false; master = costmap_; }
  { std::lock_guard<std::mutex> lk(agent_mask_mutex_); if (!agent_mask_) return false; agent = agent_mask_; }
  double wx, wy; master->mapToWorld(mx, my, wx, wy);
  unsigned int ax, ay; if (!agent->worldToMap(wx, wy, ax, ay)) return false;

  const int sx = agent->getSizeInCellsX(), sy = agent->getSizeInCellsY();
  const int ix = ax, iy = ay;
  for (int dx = -manhattan_buf; dx <= manhattan_buf; ++dx) {
    for (int dy = -manhattan_buf; dy <= manhattan_buf; ++dy) {
      if (std::abs(dx) + std::abs(dy) > manhattan_buf) continue;
      const int x = ix + dx, y = iy + dy;
      if (x < 0 || y < 0 || x >= sx || y >= sy) continue;
      if (agent->getCost(x, y) >= thr) return true;
    }
  }
  return false;
}

// [NEW] 단순히 수신된 경로를 저장
void PathValidatorNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (!msg || msg->poses.empty()) return;
  std::lock_guard<std::mutex> lock(path_mutex_);
  last_path_ = msg;
}

// [NEW] 10Hz 주기 타이머 콜백
void PathValidatorNode::validationTimerCallback()
{
  if (!is_robot_in_driving_state_.load()) return;

  nav_msgs::msg::Path::SharedPtr current_path;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (!last_path_) return;
    current_path = last_path_;
  }

  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (!costmap_) return;
  }

  std::vector<geometry_msgs::msg::PoseStamped> gpath;
  transformPathToGlobal(*current_path, gpath);
  if (gpath.empty()) return;

  if (use_footprint_check_) {
    validateWithFootprint(gpath);
  } else {
    validateWithPoints(gpath);
  }
}

// [기존 validatePathCallback은 제거됨]

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
  { std::lock_guard<std::mutex> lock(obstacle_db_mutex_); db_snapshot = obstacle_db_; }

  double acc = 0.0;
  bool is_fully_blocked = false; 

  for (size_t i = 0; i < gpath.size(); ++i) {
    if (i > 0) {
      acc += std::hypot(gpath[i].pose.position.x - gpath[i-1].pose.position.x, gpath[i].pose.position.y - gpath[i-1].pose.position.y);
      if (acc > max_check_dist) break;
    }

    unsigned int mx, my;
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      if (!costmap_->worldToMap(gpath[i].pose.position.x, gpath[i].pose.position.y, mx, my)) { streak = 0; continue; }
    }

    const bool blocked_cell = isBlockedCellKernel(mx, my);

    if (blocked_cell) {
      std::shared_ptr<nav2_costmap_2d::Costmap2D> cm;
      { std::lock_guard<std::mutex> lock(costmap_mutex_); cm = costmap_; }
      if (!cm) return;

      double wx, wy;
      cm->mapToWorld(mx, my, wx, wy);

      auto agent_hit_around = [&](double cx, double cy, unsigned int mx_c, unsigned int my_c) -> std::vector<AgentHit> {
        auto hits = whoCoversPoint(cx, cy);
        if (!hits.empty()) return hits;
        static const int OFFS[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
        for (auto &o : OFFS) {
          double wxx, wyy;
          cm->mapToWorld(static_cast<int>(mx_c)+o[0], static_cast<int>(my_c)+o[1], wxx, wyy);
          auto h2 = whoCoversPoint(wxx, wyy);
          if (!h2.empty()) return h2;
        }
        return {};
      };

      {
        auto hits = agent_hit_around(wx, wy, mx, my);
        if (!hits.empty()) {
          publishAgentCollisionList(hits);
          last_agent_block_time_ = this->now();
          publishBlockStatus(AGENT_BLOCK, "Agent overlap"); 
          is_fully_blocked = true;          
          return; 
        }
      }

      if (compare_agent_mask_) {
        if (agentCellBlockedNear(mx, my, agent_cost_threshold_, agent_mask_manhattan_buffer_)) {
          auto hits2 = agent_hit_around(wx, wy, mx, my);
          if (!hits2.empty()) {
            publishAgentCollisionList(hits2);
            last_agent_block_time_ = this->now();
            publishBlockStatus(AGENT_BLOCK, "Agent mask overlap"); 
            is_fully_blocked = true;            
            return;
          }
        }
      }
    }

    bool blocked = blocked_cell;
    const uint64_t key = packKey(mx, my);
    bool persistent_mature = false;
    auto it = db_snapshot.find(key);
    if (it != db_snapshot.end()) {
      if ((now - it->second.first_seen).seconds() >= obstacle_persistence_sec_) persistent_mature = true;
    }

    blocked = blocked && persistent_mature;
    streak = blocked ? (streak + 1) : 0;
    best_streak = std::max(best_streak, streak);

    if (best_streak >= consecutive_threshold_) {
      // --- [NEW] Look-ahead 로직 (Points 용) ---
      std::shared_ptr<nav2_costmap_2d::Costmap2D> cm;
      { std::lock_guard<std::mutex> lock(costmap_mutex_); cm = costmap_; }
      std::shared_ptr<nav2_costmap_2d::Costmap2D> sm;
      { std::lock_guard<std::mutex> lock(static_map_mutex_); sm = static_map_; }
      
      if (cm) {
          unsigned char max_center_cost = 0;
          double dist_checked = 0.0;
          
          for (size_t k = i; k < gpath.size(); ++k) {
              if (k > i) {
                  dist_checked += std::hypot(gpath[k].pose.position.x - gpath[k-1].pose.position.x, 
                                             gpath[k].pose.position.y - gpath[k-1].pose.position.y);
                  if (dist_checked > (robot_radius_m_ + lookahead_margin_m_)) break;
              }
              unsigned int c_mx, c_my;
              if (cm->worldToMap(gpath[k].pose.position.x, gpath[k].pose.position.y, c_mx, c_my)) {
                  unsigned char c_val = cm->getCost(c_mx, c_my);
                  max_center_cost = std::max(max_center_cost, c_val);
                  if (max_center_cost == 254) break;
              }
          }

          if (max_center_cost == 254) {
              publishBlockStatus(DIRECT_OBSTACLE_254, "Direct Obstacle (254)");
          } else if (max_center_cost == 253) {
              publishBlockStatus(DIRECT_OBSTACLE_253, "Direct Obstacle (253)");
          } else {
            bool is_wall = false;
            if (sm) {
                unsigned int sm_x, sm_y;
                if (sm->worldToMap(gpath[i].pose.position.x, gpath[i].pose.position.y, sm_x, sm_y)) {
                    
                    // [수정] 정적 맵에서도 kernel_half_size_ 주변을 훑어서 벽이 있는지 확인!
                    int k = kernel_half_size_;
                    int sx = sm->getSizeInCellsX(), sy = sm->getSizeInCellsY();
                    
                    for (int dx = -k; dx <= k && !is_wall; ++dx) {
                        for (int dy = -k; dy <= k && !is_wall; ++dy) {
                            int x = sm_x + dx, y = sm_y + dy;
                            if (x >= 0 && y >= 0 && x < sx && y < sy) {
                                if (sm->getCost(x, y) >= 253) {
                                    is_wall = true;
                                }
                            }
                        }
                    }
                }
            }
            if (is_wall) publishBlockStatus(WALL_HUGGING, "Wall Hugging (Static Kernel)");
            else publishBlockStatus(SIDE_OBSTACLE, "Side Obstacle (Dynamic Kernel)");
        }
      }
      is_fully_blocked = true;
      break; 
    }
  }

  if (!is_fully_blocked) {
      publishBlockStatus(CLEAR, "Path is clear");
  }
}

bool PathValidatorNode::pointInPolygon(const std::vector<geometry_msgs::msg::Point> & poly, double x, double y)
{
  bool inside = false;
  const size_t n = poly.size();
  for (size_t i=0, j=n-1; i<n; j=i++) {
    const double xi = poly[i].x, yi = poly[i].y, xj = poly[j].x, yj = poly[j].y;
    if (((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / std::max(1e-12, (yj - yi)) + xi)) inside = !inside;
  }
  return inside;
}

void PathValidatorNode::validateWithFootprint(const std::vector<geometry_msgs::msg::PoseStamped> & gpath)
{
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
  { std::lock_guard<std::mutex> lock(costmap_mutex_); if (!costmap_) return; costmap = costmap_; }

  geometry_msgs::msg::Pose cur_pose;
  if (!getCurrentPoseFromTF(cur_pose)) return;

  const double max_dist_by_time = std::max(min_lookahead_m_, max_speed_ * lookahead_time_sec_);
  const double max_check_dist = std::min(max_dist_by_time, path_check_distance_m_);
  const double res = costmap->getResolution();

  std::vector<geometry_msgs::msg::PoseStamped> samples;
  samples.reserve(gpath.size());
  double acc = 0.0;
  samples.push_back(gpath.front());
  for (size_t i = 1; i < gpath.size(); ++i) {
    const double d = std::hypot(gpath[i].pose.position.x - gpath[i-1].pose.position.x, gpath[i].pose.position.y - gpath[i-1].pose.position.y);
    if (d <= 1e-6) continue;
    acc += d;
    if (acc >= footprint_step_m_) { samples.push_back(gpath[i]); acc = 0.0; }
    if (std::hypot(gpath[i].pose.position.x - gpath.front().pose.position.x, gpath[i].pose.position.y - gpath.front().pose.position.y) > max_check_dist) break;
  }

  size_t consecutive = 0;
  bool is_fully_blocked = false;

  for (size_t s_idx = 0; s_idx < samples.size(); ++s_idx) {
    const auto & ps = samples[s_idx];
    bool blocked_here = false;
    unsigned int hit_mx = 0, hit_my = 0;

    if (use_radius_) {
      unsigned int cx, cy;
      if (!costmap->worldToMap(ps.pose.position.x, ps.pose.position.y, cx, cy)) { consecutive = 0; continue; }
      const int r_cells = std::max(1, static_cast<int>(std::ceil(robot_radius_m_ / res)));

      for (int dx = -r_cells; dx <= r_cells && !blocked_here; ++dx) {
        for (int dy = -r_cells; dy <= r_cells && !blocked_here; ++dy) {
          if (dx*dx + dy*dy > r_cells*r_cells) continue;
          const int mx = cx + dx, my = cy + dy;
          if (mx < 0 || my < 0 || mx >= static_cast<int>(costmap->getSizeInCellsX()) || my >= static_cast<int>(costmap->getSizeInCellsY())) continue;
          if (masterCellBlocked(mx, my, cost_threshold_)) { blocked_here = true; hit_mx = mx; hit_my = my; }
        }
      }
    } else {
      const double yaw = tf2::getYaw(ps.pose.orientation), c = std::cos(yaw), s = std::sin(yaw);
      std::vector<geometry_msgs::msg::Point> poly_world; poly_world.reserve(footprint_.size());
      double minx=1e9, miny=1e9, maxx=-1e9, maxy=-1e9;

      for (const auto & p : footprint_) {
        geometry_msgs::msg::Point q;
        q.x = ps.pose.position.x + c * p.x - s * p.y; q.y = ps.pose.position.y + s * p.x + c * p.y; q.z = 0.0;
        poly_world.push_back(q);
        if (q.x < minx) minx=q.x; if (q.y < miny) miny=q.y; if (q.x > maxx) maxx=q.x; if (q.y > maxy) maxy=q.y;
      }
      int min_i, min_j, max_i, max_j;
      costmap->worldToMapEnforceBounds(minx, miny, min_i, min_j); costmap->worldToMapEnforceBounds(maxx, maxy, max_i, max_j);

      for (int j = min_j; j <= max_j && !blocked_here; ++j) {
        for (int i = min_i; i <= max_i && !blocked_here; ++i) {
          double wx, wy; costmap->mapToWorld(i, j, wx, wy);
          if (!pointInPolygon(poly_world, wx, wy)) continue;
          if (masterCellBlocked(i, j, cost_threshold_)) { blocked_here = true; hit_mx = i; hit_my = j; }
        }
      }
    }

    if (blocked_here) {
      std::shared_ptr<nav2_costmap_2d::Costmap2D> cm;
      { std::lock_guard<std::mutex> lock(costmap_mutex_); cm = costmap_; }
      if (!cm) return;

      double wx, wy; cm->mapToWorld(hit_mx, hit_my, wx, wy);

      auto agent_hit_around = [&](double cx, double cy, unsigned int mx_c, unsigned int my_c) -> std::vector<AgentHit> {
        auto hits = whoCoversPoint(cx, cy); if (!hits.empty()) return hits;
        static const int OFFS[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
        for (auto &o : OFFS) {
          double wxx, wyy; cm->mapToWorld(static_cast<int>(mx_c)+o[0], static_cast<int>(my_c)+o[1], wxx, wyy);
          auto h2 = whoCoversPoint(wxx, wyy); if (!h2.empty()) return h2;
        }
        return {};
      };

      {
        auto hits = agent_hit_around(wx, wy, hit_mx, hit_my);
        if (!hits.empty()) {
          publishAgentCollisionList(hits);
          last_agent_block_time_ = this->now();
          publishBlockStatus(AGENT_BLOCK, "Agent overlap");
          is_fully_blocked = true;          
          return;
        }
      }

      if (compare_agent_mask_) {
        if (agentCellBlockedNear(hit_mx, hit_my, agent_cost_threshold_, agent_mask_manhattan_buffer_)) {
          auto hits2 = agent_hit_around(wx, wy, hit_mx, hit_my);
          if (!hits2.empty()) {
            publishAgentCollisionList(hits2);
            last_agent_block_time_ = this->now();
            publishBlockStatus(AGENT_BLOCK, "Agent mask overlap");
            is_fully_blocked = true;            
            return;
          }
        }
      }

      consecutive++;
      if (consecutive >= consecutive_threshold_) {
        
        // --- [NEW] Look-ahead 로직 및 6단계 분류 ---
        std::shared_ptr<nav2_costmap_2d::Costmap2D> sm;
        { std::lock_guard<std::mutex> lock(static_map_mutex_); sm = static_map_; }
        
        unsigned char max_center_cost = 0;
        double dist_checked = 0.0;

        for (size_t k = s_idx; k < samples.size(); ++k) {
            if (k > s_idx) {
                dist_checked += std::hypot(samples[k].pose.position.x - samples[k-1].pose.position.x, 
                                           samples[k].pose.position.y - samples[k-1].pose.position.y);
                if (dist_checked > (robot_radius_m_ + lookahead_margin_m_)) break;
            }
            unsigned int c_mx, c_my;
            if (cm->worldToMap(samples[k].pose.position.x, samples[k].pose.position.y, c_mx, c_my)) {
                unsigned char c_val = cm->getCost(c_mx, c_my);
                max_center_cost = std::max(max_center_cost, c_val);
                if (max_center_cost == 254) break; // 조기 종료 최적화
            }
        }

        if (max_center_cost == 254) {
            publishBlockStatus(DIRECT_OBSTACLE_254, "Direct Obstacle (254)");
        } else if (max_center_cost >= 253) {
            publishBlockStatus(DIRECT_OBSTACLE_253, "Partial Obstacle (253)");
        } else {
            bool is_wall = false;
            if (sm) {
                double h_wx, h_wy;
                cm->mapToWorld(hit_mx, hit_my, h_wx, h_wy);
                unsigned int sm_x, sm_y;
                if (sm->worldToMap(h_wx, h_wy, sm_x, sm_y)) {
                    if (sm->getCost(sm_x, sm_y) >= 253) is_wall = true;
                }
            }
            
            if (is_wall) publishBlockStatus(WALL_HUGGING, "Wall hugging (Static Hit)");
            else publishBlockStatus(SIDE_OBSTACLE, "Side Obstacle (Dynamic Hit)");
        }
        
        is_fully_blocked = true;
        return; 
      }
    } else {
      consecutive = 0;
    }
  }

  if (!is_fully_blocked) {
      publishBlockStatus(CLEAR, "Path is clear");
  }
}

// ===================== Agent 충돌 식별 =====================

double PathValidatorNode::headingTo(const geometry_msgs::msg::Pose & pose, double wx, double wy) {
  const double yaw = tf2::getYaw(pose.orientation), dx = wx - pose.position.x, dy = wy - pose.position.y;
  double d = std::atan2(dy, dx) - yaw;
  while (d > M_PI) d -= 2*M_PI; while (d < -M_PI) d += 2*M_PI;
  return d;
}

double PathValidatorNode::speedAlong(const geometry_msgs::msg::Twist & tw, double heading_rad) {
  return tw.linear.x * std::cos(heading_rad);
}

bool PathValidatorNode::pathTubeCoversPoint(const multi_agent_msgs::msg::MultiAgentInfo & a, double wx, double wy, double stride_m, double dilate_m, int max_poses, double, const std::string &) {
  const auto & path = a.truncated_path;
  if (path.poses.empty()) return false;
  const int limit = std::min<int>(path.poses.size(), std::max(1, max_poses));
  double acc = 0.0; auto prev = path.poses[0].pose.position;

  for (int i = 0; i < limit; ++i) {
    const auto & ps = path.poses[i].pose;
    if (i > 0) {
      const auto & cur = ps.position; acc += std::hypot(cur.x - prev.x, cur.y - prev.y);
      if (acc < std::max(0.05, stride_m)) continue;
      acc = 0.0; prev = cur;
    }
    const auto & fp = a.footprint.polygon.points;
    if (fp.size() < 3) continue;

    std::vector<geometry_msgs::msg::Point> poly_local; poly_local.reserve(fp.size());
    double cx=0, cy=0; for (auto & p : fp) { cx += p.x; cy += p.y; } cx /= fp.size(); cy /= fp.size();
    for (auto & p : fp) {
      double vx = p.x - cx, vy = p.y - cy, n = std::max(1e-6, std::hypot(vx, vy));
      geometry_msgs::msg::Point q; q.x = p.x + dilate_m * (vx / n); q.y = p.y + dilate_m * (vy / n); q.z = 0.0;
      poly_local.push_back(q);
    }
    const double yaw = tf2::getYaw(ps.orientation), c = std::cos(yaw), s = std::sin(yaw);
    std::vector<geometry_msgs::msg::Point> poly_world; poly_world.reserve(poly_local.size());
    for (auto & p : poly_local) {
      geometry_msgs::msg::Point q; q.x = ps.position.x + c * p.x - s * p.y; q.y = ps.position.y + s * p.x + c * p.y; q.z = 0.0;
      poly_world.push_back(q);
    }
    if (pointInPolygon(poly_world, wx, wy)) return true;
  }
  return false;
}

std::vector<PathValidatorNode::AgentHit> PathValidatorNode::whoCoversPoint(double wx, double wy) const {
  std::vector<AgentHit> out;
  std::lock_guard<std::mutex> lk(agents_mutex_);
  if (!last_agents_) return out;
  if ((this->now() - last_agents_stamp_).nanoseconds() > static_cast<int64_t>(agents_freshness_timeout_ms_) * 1000000LL) return out;
  if (!last_agents_->header.frame_id.empty() && last_agents_->header.frame_id != global_frame_) return out;

  for (const auto & a : last_agents_->agents) {
    if (a.machine_id == self_machine_id_) continue;

    const auto fp = getFootprintForAgent(a);
    if (fp.size() < 3) continue;

    {
      std::vector<geometry_msgs::msg::Point> poly_local; poly_local.reserve(fp.size());
      double cx=0, cy=0; for (auto & p : fp) { cx += p.x; cy += p.y; } cx /= fp.size(); cy /= fp.size();
      for (auto & p : fp) {
        double vx = p.x - cx, vy = p.y - cy, n = std::max(1e-6, std::hypot(vx, vy));
        geometry_msgs::msg::Point q; q.x = p.x + agent_match_dilate_m_ * (vx / n); q.y = p.y + agent_match_dilate_m_ * (vy / n); q.z = 0.0;
        poly_local.push_back(q);
      }
      const double yaw = tf2::getYaw(a.current_pose.pose.orientation), c = std::cos(yaw), s = std::sin(yaw);
      std::vector<geometry_msgs::msg::Point> poly_world; poly_world.reserve(poly_local.size());
      for (auto & p : poly_local) {
        geometry_msgs::msg::Point q; q.x = a.current_pose.pose.position.x + c * p.x - s * p.y; q.y = a.current_pose.pose.position.y + s * p.x + c * p.y; q.z = 0.0;
        poly_world.push_back(q);
      }

      if (pointInPolygon(poly_world, wx, wy)) {
        AgentHit hit{a.machine_id, a.type_id, wx, wy, -1.0f, "agent footprint overlap"};
        const double head = headingTo(a.current_pose.pose, wx, wy), v_along = speedAlong(a.current_twist, head);
        if (v_along > 0.05) hit.ttc_first = static_cast<float>(std::hypot(wx - a.current_pose.pose.position.x, wy - a.current_pose.pose.position.y) / v_along);
        out.emplace_back(std::move(hit));
        continue; 
      }
    }

    bool check_path_tube = agent_path_hit_enable_;
    if (!respect_higher_priority_path_ && (a.machine_id < self_machine_id_)) check_path_tube = false;

    if (check_path_tube) {    
      if (pathTubeCoversPoint(a, wx, wy, agent_path_hit_stride_m_, agent_path_hit_dilate_m_, agent_path_hit_max_poses_, 0.0, global_frame_)) {
        out.emplace_back(AgentHit{a.machine_id, a.type_id, wx, wy, -1.0f, "agent truncated_path overlap"});
      }
    }
  }
  return out;
}

void PathValidatorNode::publishAgentCollisionList(const std::vector<AgentHit> & hits) {
  if (!publish_agent_collision_ || !agent_collision_pub_ || hits.empty()) return;
  multi_agent_msgs::msg::PathAgentCollisionInfo msg;
  msg.header.stamp = this->now(); msg.header.frame_id = global_frame_;
  for (const auto & h : hits) {
    msg.machine_id.push_back(h.machine_id); msg.type_id.push_back(h.type_id);
    msg.x.push_back(h.x); msg.y.push_back(h.y);
    msg.ttc_first.push_back(h.ttc_first); msg.note.push_back(h.note);
  }
  agent_collision_pub_->publish(msg);
}

void PathValidatorNode::publishBlockStatus(BlockStatus status, const std::string & reason) {
  const rclcpp::Time now = this->now();

  // 홀드: 에이전트 블록 이후 2초간 일반 장애물(벽, 상자 등) 퍼블리시 무시
  if (status != AGENT_BLOCK) {
      const double since_agent = (now - last_agent_block_time_).seconds();
      if (since_agent >= 0.0 && since_agent < agent_block_hold_sec_) return;
  }

  // 10Hz 상태 송출
  std_msgs::msg::Int8 msg;
  msg.data = status;
  block_status_pub_->publish(msg);
  
  // 로그 스팸 방지 (1초 1회)
  if (status == CLEAR) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Status: 0 (CLEAR)");
  } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                           "Status: %d, Reason: %s", status, reason.c_str());
  }
}

}  // namespace replan_monitor