#include "replan_monitor/path_validator_node.hpp"
// [NEW] For makeFootprintFromString and makeFootprintFromRadius
#include <nav2_costmap_2d/footprint.hpp>

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

  this->declare_parameter("publish_false_pulse", false); //true
  this->declare_parameter("flag_pulse_ms", 120);

  // ===== Footprint / Agent mask / Output =====
  this->declare_parameter("use_footprint_check", true); // false
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
      "/plan_pruned",
      rclcpp::QoS(10),
      std::bind(&PathValidatorNode::validatePathCallback, this, _1),
      subs_options);

  // ===== Publishers =====
  {
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local().reliable();
    replan_pub_ = this->create_publisher<std_msgs::msg::Bool>("/replan_flag", qos);
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

  RCLCPP_INFO(this->get_logger(),
    "PathValidatorNode ready. agent_path_hit_enable=%s stride=%.2f dilate=%.2f maxposes=%d",
    (agent_path_hit_enable_ ? "true":"false"),
    agent_path_hit_stride_m_, agent_path_hit_dilate_m_, agent_path_hit_max_poses_);
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
  is_robot_in_driving_state_.store(s == "DRIVING" || s == "PLANNING");
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







void PathValidatorNode::validatePathOptimized(const std::vector<geometry_msgs::msg::PoseStamped> & gpath)
{
  // =========================================================
  // [TUNING PARAMETERS] 충돌 민감도 튜닝용 로컬 변수 모음
  // =========================================================
  
  // [Phase 1] 넓게 훑어볼 때 사용할 코스트 임계값 (위험 사전 감지용: 노이즈 방지를 위해 253 권장)
  const unsigned char phase1_thr = 253; 
  
  // [Phase 1] 경로 중심선 기준 좌우로 몇 미터까지 훑어볼 것인가? (맨해튼 버퍼)
  const double phase1_buffer_m = 0.1;   
  
  // [Phase 2] 정밀 검사 시 '충돌(Hit)'로 판정할 코스트 (완전한 물리적 충돌: 254 고정)
  const unsigned char phase2_thr = 254; 

  // [Phase 2] 충돌 예상 지점에서 뒤로 몇 미터 후퇴하여 정밀 검사를 시작할 것인가?
  // (0.6m 정사각형 로봇 대각선 절반 0.424m + 여유 마진 0.1m)
  const double backtrack_m = 0.55;      

  // [Phase 2] '모서리 스침'이 아닌 '경로가 꽉 막힘'으로 간주할 장애물의 물리적 면적 (m^2)
  // (값을 줄이면 얇은 막대기에 민감해짐. 값을 늘리면 웬만한 장애물/노이즈는 무시함)
  const double heavy_block_area_m2 = 0.0125; 

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
  // [Phase 1] Broad-Phase: 빠른 인덱스 스캔 (다각형 연산 없음)
  // =========================================================
  int phase1_hit_index = -1;
  double acc_dist = 0.0;
  
  // phase1_buffer_m를 현재 해상도에 맞춰 셀 개수로 환산
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

    // 맨해튼 버퍼 고속 스캔
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

  // 위험 지점이 없으면 안전하므로 검사 조기 종료 (연산량 극적 감소)
  if (phase1_hit_index == -1) {
    return;
  }

  // =========================================================
  // [Phase 2] Narrow-Phase: 후퇴 및 정밀(밀도) 검사
  // =========================================================
  int narrow_start_idx = phase1_hit_index;
  double back_dist = 0.0;

  // 1. 위험 지점에서 backtrack_m 만큼 후퇴한 시작 인덱스 찾기
  for (int i = phase1_hit_index; i > 0; --i) {
    const auto & p0 = gpath[i-1].pose.position;
    const auto & p1 = gpath[i].pose.position;
    back_dist += std::hypot(p1.x - p0.x, p1.y - p0.y);
    narrow_start_idx = i - 1;
    if (back_dist >= backtrack_m) break;
  }

  // 2. 지정한 면적(m^2)을 현재 코스트맵 해상도 기준 셀 개수로 자동 환산
  const int HEAVY_BLOCK_THRESHOLD = std::max(1, static_cast<int>(heavy_block_area_m2 / (res * res)));

  size_t consecutive = 0;
  double narrow_acc = 0.0;

  // 3. 후퇴한 지점부터 footprint_step_m 간격으로 전진하며 정밀 다각형 검사
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
    poly_world.reserve(footprint_.size());
    double minx=1e9, miny=1e9, maxx=-1e9, maxy=-1e9;
    
    // 로봇 다각형 회전 및 평행 이동
    for (const auto & p : footprint_) {
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
    double min_hit_dist = 1e9; // 로봇의 물리적 위치(cur_pose) 기준 가장 가까운 충돌점 거리를 기록

    // 다각형 Bounding Box 내 셀 밀도 카운팅 및 최초 충돌점 추적
    for (int y = min_j; y <= max_j; ++y) {
      for (int x = min_i; x <= max_i; ++x) {
        double wx, wy; costmap->mapToWorld(x, y, wx, wy);
        
        if (!pointInPolygon(poly_world, wx, wy)) continue;
        
        const unsigned int umx = static_cast<unsigned int>(x);
        const unsigned int umy = static_cast<unsigned int>(y);
        
        const unsigned char cost = costmap->getCost(umx, umy);
        
        // 정밀 검사에서는 완전한 물리적 충돌(phase2_thr, 즉 254)만 카운트!
        if ((!ignore_unknown_ || cost != nav2_costmap_2d::NO_INFORMATION) && cost >= phase2_thr) {
          blocked_cell_count++;
          
          // 로봇 실제 위치로부터 가장 가까운 유클리디안 거리의 셀을 기록
          double dist_to_robot = std::hypot(wx - cur_pose.position.x, wy - cur_pose.position.y);
          if (dist_to_robot < min_hit_dist) {
            min_hit_dist = dist_to_robot;
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
        // 면적이 넓으면 얇은 막대기 또는 꽉 막힌 벽으로 간주 -> 단번에 리플랜 임계치 도달
        consecutive += consecutive_threshold_;
      } else {
        // 면적이 좁으면 모서리 스침 노이즈로 간주 -> 카운트 1회만 증가
        consecutive++;
      }

      // 연속성(또는 밀도)이 임계값을 넘었을 때만 최종 판단 수행
      if (consecutive >= consecutive_threshold_) {
        double hit_wx, hit_wy;
        costmap->mapToWorld(hit_mx, hit_my, hit_wx, hit_wy);

        // 에이전트 마스크 검사
        if (compare_agent_mask_) {
          bool agent_mark = agentCellBlockedNear(
              hit_mx, hit_my, static_cast<unsigned char>(agent_cost_threshold_), agent_mask_manhattan_buffer_);
              
          if (agent_mark) {
            const double allowed_dist = 0.424 + 0.1; // 로봇 대각선 + 통신 지연 오차 보정
            auto hits2 = findNearestAgent(hit_wx, hit_wy, allowed_dist);
            if (!hits2.empty()) {
              publishAgentCollisionList(hits2);
              last_agent_block_time_ = this->now();
              return; // 에이전트면 리플랜 대기 (Hold)
            }
          }
        } else {
          auto hits = whoCoversPoint(hit_wx, hit_wy); 
          if (!hits.empty()) {
            publishAgentCollisionList(hits);
            last_agent_block_time_ = this->now();
            return;
          }
        }

        // 에이전트가 아니면 일반 장애물 판정: 즉시 리플랜 실행!
        triggerReplan("blocked (optimized broad-narrow phase)");
        return; 
      }
    } else {
      // 아무것도 닿지 않은 깨끗한 샘플 구역이면 카운트 초기화
      consecutive = 0;
    }
    
    // 위험 구간을 충분히 통과했으면 안전하다고 판단하여 정밀 검사 조기 종료
    // 로봇 대각선(0.42m)의 약 2배 이상인 1.0m를 확실하게 벗어났는지 물리적 거리로 확인합니다.
    double dist_passed = std::hypot(
        gpath[i].pose.position.x - gpath[phase1_hit_index].pose.position.x,
        gpath[i].pose.position.y - gpath[phase1_hit_index].pose.position.y
    );
    
    if (dist_passed > 1.0 && consecutive == 0) {
        break;
    }
    }
  }
}



// ===================== agent mask and footprint checking =====================

void PathValidatorNode::validateWithFootprint(const std::vector<geometry_msgs::msg::PoseStamped> & gpath)
{
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

  // 경로 샘플링
  std::vector<geometry_msgs::msg::PoseStamped> samples;
  samples.reserve(gpath.size());
  double acc = 0.0;
  samples.push_back(gpath.front());
  for (size_t i = 1; i < gpath.size(); ++i) {
    const auto & p0 = gpath[i-1].pose.position;
    const auto & p1 = gpath[i].pose.position;
    const double d = std::hypot(p1.x - p0.x, p1.y - p0.y);
    if (d <= 1e-6) continue;
    acc += d;
    if (acc >= footprint_step_m_) {
      samples.push_back(gpath[i]);
      acc = 0.0;
    }
    const double total = std::hypot(gpath[i].pose.position.x - gpath.front().pose.position.x,
                                    gpath[i].pose.position.y - gpath.front().pose.position.y);
    if (total > max_check_dist) break;
  }

  const unsigned char master_thr = static_cast<unsigned char>(cost_threshold_);
  const unsigned char agent_thr  = static_cast<unsigned char>(agent_cost_threshold_);

  size_t consecutive = 0;

  for (const auto & ps : samples) {
    bool blocked_here = false;
    unsigned int hit_mx = 0, hit_my = 0;

    if (use_radius_) {
      unsigned int cx, cy;
      if (!costmap->worldToMap(ps.pose.position.x, ps.pose.position.y, cx, cy)) {
        consecutive = 0; continue;
      }
      const int r_cells = std::max(1, static_cast<int>(std::ceil(robot_radius_m_ / res)));

      for (int dx = -r_cells; dx <= r_cells && !blocked_here; ++dx) {
        for (int dy = -r_cells; dy <= r_cells && !blocked_here; ++dy) {
          if (dx*dx + dy*dy > r_cells*r_cells) continue;
          const int mx = static_cast<int>(cx) + dx;
          const int my = static_cast<int>(cy) + dy;
          if (mx < 0 || my < 0 ||
              mx >= static_cast<int>(costmap->getSizeInCellsX()) ||
              my >= static_cast<int>(costmap->getSizeInCellsY())) continue;

          const unsigned int umx = static_cast<unsigned int>(mx);
          const unsigned int umy = static_cast<unsigned int>(my);

          if (masterCellBlocked(umx, umy, master_thr)) {
            blocked_here = true; hit_mx = umx; hit_my = umy;
          }
        }
      }
    } else {
      // 다각형 footprint → 월드 폴리곤
      const double yaw = tf2::getYaw(ps.pose.orientation);
      const double c = std::cos(yaw), s = std::sin(yaw);

      std::vector<geometry_msgs::msg::Point> poly_world;
      poly_world.reserve(footprint_.size());
      double minx=1e9, miny=1e9, maxx=-1e9, maxy=-1e9;

      for (const auto & p : footprint_) {
        geometry_msgs::msg::Point q;
        const double x = p.x, y = p.y;
        q.x = ps.pose.position.x + c * x - s * y;
        q.y = ps.pose.position.y + s * x + c * y;
        q.z = 0.0;
        poly_world.push_back(q);
        if (q.x < minx) minx=q.x;
        if (q.y < miny) miny=q.y;
        if (q.x > maxx) maxx=q.x; 
        if (q.y > maxy) maxy=q.y;
      }

      int min_i, min_j, max_i, max_j;
      costmap->worldToMapEnforceBounds(minx, miny, min_i, min_j);
      costmap->worldToMapEnforceBounds(maxx, maxy, max_i, max_j);

      for (int j = min_j; j <= max_j && !blocked_here; ++j) {
        for (int i = min_i; i <= max_i && !blocked_here; ++i) {
          double wx, wy; costmap->mapToWorld(i, j, wx, wy);
          if (!pointInPolygon(poly_world, wx, wy)) continue;

          const unsigned int umx = static_cast<unsigned int>(i);
          const unsigned int umy = static_cast<unsigned int>(j);

          if (masterCellBlocked(umx, umy, master_thr)) {
            blocked_here = true; hit_mx = umx; hit_my = umy;
          }
        }
      }
    }

    if (blocked_here) {
      // --- costmap 포인터를 잠깐 복사 (락 최소화) ---
      std::shared_ptr<nav2_costmap_2d::Costmap2D> cm;
      { std::lock_guard<std::mutex> lock(costmap_mutex_); cm = costmap_; }
      if (!cm) return;

      // --- 중심 셀(히트셀) 기준 월드좌표 ---
      double wx, wy;
      cm->mapToWorld(static_cast<int>(hit_mx), static_cast<int>(hit_my), wx, wy);

      // [유지] 기존 람다 함수 (compare_agent_mask_ 가 false일 때 플랜 B로 사용)
      auto agent_hit_around = [&](double cx, double cy,
                                  unsigned int mx_c, unsigned int my_c) -> std::vector<AgentHit> {
        auto hits = whoCoversPoint(cx, cy);
        if (!hits.empty()) return hits;

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

      // =========================================================
      // [수정된 판별 로직: 마스크 유무에 따른 스마트 스위칭]
      // =========================================================
      if (compare_agent_mask_) {
        // [Plan A] 마스크를 사용할 때는 복잡한 다각형 검사(agent_hit_around)를 건너뛰고,
        // 가장 빠르고 확실한 '마스크 버퍼 검사 + 거리 기반 에이전트 찾기'를 수행합니다.
        const bool agent_mark = agentCellBlockedNear(
            hit_mx, hit_my,
            static_cast<unsigned char>(agent_cost_threshold_),
            agent_mask_manhattan_buffer_);
            
        if (agent_mark) {
        //   auto hits2 = findNearestAgent(wx, wy);
        // [Plan B: 깐깐한 거름망]
          // 최대 예상 지연(예: 0.5초) * 최고 속도(예: 1.0m/s) = 0.5m의 오차 허용.
          // 로봇 반경 + 지연 오차 마진 = 약 0.8m ~ 1.0m
          const double MAX_LATENCY_ERROR_M = 0.1; // [meter] (필요시 파라미터로 도출)
        //   const double allowed_dist = robot_radius_m_ + MAX_LATENCY_ERROR_M;
          const double allowed_dist = 0.424 + MAX_LATENCY_ERROR_M; // [튜닝 필요: 로봇 최외각 0.424m + 최대 지연 오차 0.1m = 0.524m]

          auto hits2 = findNearestAgent(wx, wy, allowed_dist);


          if (!hits2.empty()) {
            publishAgentCollisionList(hits2);
            last_agent_block_time_ = this->now();
            return;
          }
        }
      } else {
        // [Plan B] 마스크를 사용하지 않을 경우, 기존의 '기하학적 수학 검사' 로직을 호출합니다.
        auto hits = agent_hit_around(wx, wy, hit_mx, hit_my);
        if (!hits.empty()) {
          publishAgentCollisionList(hits);
          last_agent_block_time_ = this->now();
          return;
        }
      }

      // 3) 여기까지 확인했는데도 에이전트가 아니면 일반 장애물로 누적 처리!
      consecutive++;
      if (consecutive >= consecutive_threshold_) {
        triggerReplan("blocked (footprint) streak threshold reached");
        return;
      }
    } else {
      consecutive = 0;
    }
  }
}



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

void PathValidatorNode::validatePathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (!is_robot_in_driving_state_.load()) return;
  if (!msg || msg->poses.empty()) return;

  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (!costmap_) return;
  }

  std::vector<geometry_msgs::msg::PoseStamped> gpath;
  transformPathToGlobal(*msg, gpath);
  if (gpath.empty()) return;

  const rclcpp::Time now = this->now();
  const double since_agent = (now - last_agent_block_time_).seconds();
  if (since_agent >= 0.0 && since_agent < agent_block_hold_sec_) {
    RCLCPP_DEBUG(get_logger(), "[hold] agent-block hold active (%.2fs < %.2fs)",
                 since_agent, agent_block_hold_sec_);
    return;
  }

  if (use_footprint_check_) {
    // validateWithFootprint(gpath);
    validatePathOptimized(gpath);
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
          publishAgentCollisionList(hits);
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
            publishAgentCollisionList(hits2);
            last_agent_block_time_ = this->now();
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
      triggerReplan("blocked (points) streak threshold reached");
      break;
    }
  }
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

void PathValidatorNode::publishAgentCollisionList(const std::vector<AgentHit> & hits)
{
  if (!publish_agent_collision_ || !agent_collision_pub_) return;
  if (hits.empty()) return;

  multi_agent_msgs::msg::PathAgentCollisionInfo msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = global_frame_;


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


// [추가] 1초 주기로 현재 충돌 중인 에이전트 목록을 로깅
  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "Publishing Agent Collision Info: [ %s] (Total: %zu hits)", 
    agent_ids.c_str(), hits.size());

  agent_collision_pub_->publish(msg);
}

void PathValidatorNode::triggerReplan(const std::string & reason)
{
  const rclcpp::Time now = this->now();

  // 홀드: 에이전트 알림 이후 일정 시간은 리플랜 방지
  const double since_agent = (now - last_agent_block_time_).seconds();
  if (since_agent >= 0.0 && since_agent < agent_block_hold_sec_) {
    RCLCPP_DEBUG(get_logger(),
      "Replan suppressed by agent-block hold (%.2fs < %.2fs)",
      since_agent, agent_block_hold_sec_);
    return;
  }

  if ((now - last_replan_time_).seconds() <= cooldown_sec_) return;
  last_replan_time_ = now;

// [추가] 리플랜 트리거 사유를 1초 주기로 로깅
//   RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
//     "Publishing Replan Flag [True]. Reason: %s", reason.c_str());

  std_msgs::msg::Bool m; m.data = true;
  replan_pub_->publish(m);
  RCLCPP_WARN(this->get_logger(), "Triggering replan: %s", reason.c_str());

  if (publish_false_pulse_ && flag_pulse_ms_ > 0) {
    flag_reset_timer_.reset();
    auto weak_pub = std::weak_ptr<rclcpp::Publisher<std_msgs::msg::Bool>>(replan_pub_);
    flag_reset_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(flag_pulse_ms_),
        [weak_pub]() {
          if (auto pub = weak_pub.lock()) {
            std_msgs::msg::Bool off; off.data = false;
            pub->publish(off);
          }
        });
  }
}

}  // namespace replan_monitor
