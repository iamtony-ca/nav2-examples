#include "replan_monitor/path_validator_node.hpp"
// [NEW] For makeFootprintFromString and makeFootprintFromRadius
#include <nav2_costmap_2d/footprint.hpp>

#include <optional>   // [10-10 W1-2]
#include <sstream>
#include <iomanip> // std::fixed, std::setprecision 사용을 위해 필요할 수 있습니다.

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
PathValidatorNode::getFootprintForAgent(const robot_interfaces::msg::MultiAgentInfo & a) const
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

  // [10-10 W1] global costmap mirror. 기본값은 예전 동작과 같은 토픽 이름.
  //   always_send_full_costmap: False 이면 Nav2 는 full 을 시작·크기/origin 변경 때만 보내고 나머지는
  //   <topic>_updates (nav2_msgs/CostmapUpdate) 로 보낸다. 그래서 update 를 병합하고, 늦게 붙거나 update 를
  //   놓쳤을 때는 get_costmap service 로 full 을 다시 받는다 (resync).
  // [10-10 W1-4] 기본 source = DiffPublisherLayer (값이 바뀐 셀만 CostmapUpdate 로 보내는 global costmap 맨 끝 layer,
  //   pv_w4_ws/W1_DIFF_PUBLISHER.md). stock Costmap2DPublisher 는 fallback (layer 가 없는 설정용).
  this->declare_parameter<std::string>("global_costmap_topic", "/global_costmap/diff_publisher/costmap_raw");
  this->declare_parameter<std::string>("global_costmap_service", "/global_costmap/diff_publisher/get_costmap");
  // [10-10 W1-4] fallback: global_costmap_topic 에 publisher 가 없고 이 topic 에는 있는 상태가 global_costmap_fallback_sec 동안
  //   이어지면 (시작 뒤 한 번) 이 topic + service 로 바꾼다. 그 뒤 diff publisher 가 생기면 한 번 돌아간다.
  //   topic/service 를 비우거나 sec 를 0.0 으로 두면 fallback 을 끈다. double — 15.0 처럼 소수점.
  this->declare_parameter<std::string>("global_costmap_fallback_topic", "/global_costmap/costmap_raw");
  this->declare_parameter<std::string>("global_costmap_fallback_service", "/global_costmap/get_costmap");
  this->declare_parameter("global_costmap_fallback_sec", 15.0);
  // [10-10 W1-4] global_costmap_topic 의 publisher 가 값이 바뀔 때만 보내는가 (diff layer = true). true 면 watchdog 은
  //   그 source 에서 무갱신이 아니라 publisher 유무로 stale 을 본다. global_costmap_topic 을 stock 으로 바꾸면 false. bool.
  this->declare_parameter("global_costmap_event_driven", true);
  // [10-10 W1-4][리뷰] event_driven source 의 heartbeat 한도 (s). diff layer 는 update 구독자가 있으면 sweep_tick_sec (1 s) 마다
  //   sweep band 하나를 보내므로, 이만큼 아무것도 못 받으면 watchdog 무갱신. 기본 20 s = FULL ack 대기 상한 (full_ack_timeout_sec
  //   10 s, 그동안 publisher thread 가 멈춘다) + 여유. layer 의 sweep 을 끄면 (sweep_period_sec 0) 0.0 으로 끌 것. double.
  this->declare_parameter("global_costmap_heartbeat_sec", 20.0);
  this->declare_parameter("costmap_resync_on_start", true);
  this->declare_parameter("costmap_resync_period_sec", 0.0);       // 0 = 주기 resync 끔
  this->declare_parameter("costmap_resync_min_interval_sec", 5.0); // 요청 최소 간격
  this->declare_parameter("costmap_resync_timeout_sec", 30.0);     // 응답 대기 상한
  this->declare_parameter("costmap_updates_qos_depth", 100);        // [10-10 W1][리뷰] 10 → 100 (reader 쪽 유실은 알 수 없음)
  // [10-10 W1][리뷰] 시작 resync 를 미루는 시간. 그 안에 topic full 이 2 번 이상 오면(True) 시작 resync 를 생략한다.
  this->declare_parameter("costmap_resync_start_grace_sec", 5.0);
  this->declare_parameter("costmap_keep_spare_buffer", true);
  // [10-10 W1-2] agent mask mirror. AgentLayer(full_mask_publish: false)는 full 을 첫 publish·크기/원점 변경 때만 보내고
  //   나머지는 <agent_mask_topic>_updates 로 보낸다. service 는 AgentLayer 의 Costmap2DPublisher 가 만드는
  //   "get_" + "<layer 이름>/raw_costmap" 이다 (global costmap 노드 namespace 기준 → /global_costmap/get_agent_layer/raw_costmap).
  //   min_interval / timeout / start_grace / updates_qos_depth / keep_spare 는 global 값을 같이 쓴다.
  this->declare_parameter<std::string>("agent_mask_service", "/global_costmap/get_agent_layer/raw_costmap");
  this->declare_parameter("agent_mask_resync_on_start", true);
  // mask 는 매 사이클 다시 칠해지므로 update 하나를 잃으면 그 사이클에 지운 셀(유령 agent)이 다음 resync 까지 남을 수 있다.
  // 0 = 끔 (기본). 켜면 마지막 full 이후 이만큼 지나면 get_costmap 으로 다시 받는다.
  this->declare_parameter("agent_mask_resync_period_sec", 0.0);
  // [10-10 W1-2][리뷰] W1-2 AgentLayer 는 "0 이 아닌 셀 ⊆ update 사각형" 을 지키므로 update 하나가 mask 전체다.
  //   true 면 mask mirror 가 update 마다 직전 사각형을 0 으로 만든 뒤 복사한다 → 중간 update 를 잃어도 다음 update 로
  //   맞는다 (DDS 가 알리지 않는 유실 포함). 오래된 latched full 은 geometry 만 쓰고, get_costmap 은 grid 가 아직 없을 때와
  //   geometry 불일치·clock reset 때만 부른다. 예전 AgentLayer(always full)와도 동작한다 (update 가 오지 않을 뿐).
  //   mask topic 을 일반 costmap 으로 바꾸면 false 로 둘 것.
  this->declare_parameter("agent_mask_self_contained_updates", true);
  // [10-10 W1-2][리뷰] replay 보관 개수 (64 → 256). get_costmap 응답 스냅샷 뒤 도착까지 이보다 많은 update 가 오면
  //   응답 적용 뒤 resync 를 다시 건다 (step 1 의 64 는 2 Hz global 에서 32 s, 5 Hz mask 에서 12.8 s 였고, 2e8 cells
  //   지도의 get_costmap 은 16 s 걸렸다). 바이트 상한(64 MB)은 그대로다.
  this->declare_parameter("costmap_replay_max_msgs", 256);
  // [10-10 W1-3] W2PV-1: agent mask 를 global costmap mirror 의 geometry 로 0 grid 로 시작하고 첫 self-contained update 를 바로
  //   적용한다 (현장 순서에서 mask full 은 latch 되지 않는다). mask get_costmap 은 geometry 를 모르거나 맞지 않을 때만.
  //   agent_mask_topic 과 global_costmap_topic 의 첫 이름(namespace) 이 다르면 (예: local mask) 쓰지 않는다.
  this->declare_parameter("agent_mask_seed_from_global", true);
  // [10-10 W1-3] R3: get_costmap timeout·실패 뒤 다음 요청까지 기다릴 시간 (연속 실패마다 2 배, 상한). double — 30.0 처럼 소수점.
  this->declare_parameter("costmap_resync_backoff_base_sec", 30.0);
  this->declare_parameter("costmap_resync_backoff_max_sec", 120.0);
  // [10-10 W1-3] R4: global costmap full/update 를 이만큼 못 받으면 throttled ERROR (0 = 끔). double — 10.0 처럼 소수점.
  this->declare_parameter("costmap_stale_watchdog_sec", 10.0);
  // [10-10 W1-3][리뷰] costmap_resync_timeout_sec 은 이제 "늦음" WARN 기준이다 (요청을 버리지 않는다). 이만큼 응답이 없으면
  //   (또는 timeout 뒤 service 가 사라지면) 포기하고 backoff 뒤 다시 보낸다. double — 300.0 처럼 소수점. timeout 보다 작으면 timeout.
  this->declare_parameter("costmap_resync_abandon_sec", 300.0);
  // [10-10 W1-3][리뷰] READY 에서 global grid 가 아직 없으면 (G=False 의 PV (재)시작 직후 get_costmap 응답 전) 목표 점유를 볼 수
  //   없다. true 면 SAFE 대신 is_status_ready=false 를 내 navigation_manager 의 출발을 보류한다. false = 예전 (SAFE). bool.
  this->declare_parameter("ready_hold_without_global_grid", true);

  this->declare_parameter("publish_agent_collision", true);
  this->declare_parameter<std::string>("agent_collision_topic", "/path_agent_collision_info");

  // MultiAgent 구독
  this->declare_parameter<std::string>("agents_topic", "/multi_agent_infos");
  this->declare_parameter("agents_freshness_timeout_ms", 800);
  // [FIX] 이웃 캐시 보존 시간. 병합만 하고 만료가 없으면 떠난 로봇이 영원히 남는다.
  this->declare_parameter("agent_cache_ttl_sec", 5.0);
  this->declare_parameter("agent_match_dilate_m", 0.1); // 0.05

  // Nav2 footprint 스타일
  this->declare_parameter<std::string>("footprint", "[]");
  this->declare_parameter("robot_radius", 0.1);

  // 에이전트 홀드
  this->declare_parameter("agent_block_hold_sec", 0.5);
  this->declare_parameter("agent_block_max_wait_sec", 8.0);

  // === NEW: 에이전트 경로 튜브 매칭 ===
  this->declare_parameter("agent_path_hit_enable", true);
  this->declare_parameter("agent_path_hit_stride_m", 0.35);
  this->declare_parameter("agent_path_hit_dilate_m", 0.05);
  this->declare_parameter("agent_path_hit_max_poses", 1000);

  this->declare_parameter("respect_higher_priority_path", false);
  // [V2 M-6] 로봇의 현재 발자국(+여유) 안에 있는 정적 lethal 셀은 HIT 로 세지 않는다.
  // 벽에 붙은 채 로컬라이제이션 오차로 자기 발자국이 lethal 에 걸리면 replan 으로는
  // 영영 못 푼다(출발점이 막혀 새 경로도 같은 HIT). Nav2 가 스스로 빠져나오게 둔다.
  this->declare_parameter("ignore_static_hit_in_self_footprint", true);
  this->declare_parameter("self_footprint_margin_m", 0.05);

  this->declare_parameter("validation_frequency", 10.0); // 기본 10Hz 주기


  // 생성자(Constructor) 내부에 추가
  this->declare_parameter("goal_doorstep_static_m", 0.1);
  this->declare_parameter("goal_doorstep_agent_m", 0.05); // 에이전트는 지연 오차를 고려해 조금 더 크게 설정 가능

  goal_doorstep_static_m_ = this->get_parameter("goal_doorstep_static_m").as_double();
  // [V2.30 09-24] READY(출발 전) 목표 점유 검사의 **노선 거리 상한**. 0 이하 = 제한 없음(현장 원본과 동일).
  // 현장 원본은 남은 목표 **전부**를 거리 제한 없이 검사해, 노선 어딘가 하나만 막혀도 출발하지 않는다.
  // 노선 전체를 한 번에 받는 현장 방식에서는 대수가 늘수록 "내 노선 어딘가에 로봇이 서 있을" 확률이
  // 올라간다. sim 실측(K1, 작은 링 5대): 세 대가 40분 내내 0 m — "goals are occupied" 298회.
  // 먼 목표는 가까워졌을 때 주행 중 검사가 다시 본다. 그래서 출발 판단은 가까운 구간만 본다.
  ready_goal_horizon_m_ = this->declare_parameter("ready_goal_horizon_m", 0.0);
  // [V2.31 09-24] READY 교착 **탈출**. 평소에는 위 상한(기본 0 = 현장 원본, 노선 전체)을 그대로 쓰고,
  // READY 에서 ready_escape_after_sec 이상 계속 막혀 있을 때만 ready_escape_horizon_m 로 좁힌다.
  // 예방이 아니라 탈출 장치다. K1 실측: 네 대가 150 s 타임아웃에 동시에 걸려 동시에 재지령을 받고
  // 다시 서로의 "두 번째 목표"에 막히기를 10회 반복했다(26분). 가까운 목표는 계속 검사하고,
  // 출발한 뒤에는 주행 중 검사가 먼 목표를 다시 보므로 안전 판정은 줄지 않는다.
  // navigation_manager 의 MAX_WAIT(150 s) 보다 충분히 짧아야 한다.
  ready_escape_after_sec_ = this->declare_parameter("ready_escape_after_sec", 0.0);
  ready_escape_horizon_m_ = this->declare_parameter("ready_escape_horizon_m", 3.0);
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

  // [10-10 W1] global costmap mirror / resync
  global_costmap_topic_            = this->get_parameter("global_costmap_topic").as_string();
  global_costmap_service_          = this->get_parameter("global_costmap_service").as_string();
  costmap_resync_on_start_         = this->get_parameter("costmap_resync_on_start").as_bool();
  costmap_resync_period_sec_       = std::max(0.0, this->get_parameter("costmap_resync_period_sec").as_double());
  costmap_resync_min_interval_sec_ = std::max(0.0, this->get_parameter("costmap_resync_min_interval_sec").as_double());
  costmap_resync_timeout_sec_      = std::max(1.0, this->get_parameter("costmap_resync_timeout_sec").as_double());
  costmap_updates_qos_depth_       = std::max<int>(1, static_cast<int>(this->get_parameter("costmap_updates_qos_depth").as_int()));
  costmap_keep_spare_buffer_       = this->get_parameter("costmap_keep_spare_buffer").as_bool();
  costmap_resync_start_grace_sec_  = std::max(0.0, this->get_parameter("costmap_resync_start_grace_sec").as_double());  // [10-10 W1][리뷰]
  global_costmap_.setKeepSpareBuffer(costmap_keep_spare_buffer_);
  // [10-10 W1-2] agent mask mirror / resync
  agent_mask_service_              = this->get_parameter("agent_mask_service").as_string();
  agent_mask_resync_on_start_      = this->get_parameter("agent_mask_resync_on_start").as_bool();
  agent_mask_resync_period_sec_    = std::max(0.0, this->get_parameter("agent_mask_resync_period_sec").as_double());
  agent_mask_.setKeepSpareBuffer(costmap_keep_spare_buffer_);
  agent_mask_self_contained_updates_ = this->get_parameter("agent_mask_self_contained_updates").as_bool();   // [10-10 W1-2][리뷰]
  costmap_replay_max_msgs_ = std::max<int>(0, static_cast<int>(this->get_parameter("costmap_replay_max_msgs").as_int()));
  global_costmap_.setReplayMaxMsgs(static_cast<size_t>(costmap_replay_max_msgs_));
  agent_mask_.setReplayMaxMsgs(static_cast<size_t>(costmap_replay_max_msgs_));
  agent_mask_.setSelfContainedUpdates(agent_mask_self_contained_updates_);
  // [10-10 W1-3]
  agent_mask_seed_from_global_     = this->get_parameter("agent_mask_seed_from_global").as_bool();
  costmap_resync_backoff_base_sec_ = std::max(0.0, this->get_parameter("costmap_resync_backoff_base_sec").as_double());
  costmap_resync_backoff_max_sec_  = std::max(costmap_resync_backoff_base_sec_,
                                       this->get_parameter("costmap_resync_backoff_max_sec").as_double());
  costmap_stale_watchdog_sec_      = std::max(0.0, this->get_parameter("costmap_stale_watchdog_sec").as_double());
  costmap_resync_abandon_sec_      = std::max(costmap_resync_timeout_sec_,
                                       this->get_parameter("costmap_resync_abandon_sec").as_double());   // [10-10 W1-3][리뷰]
  ready_hold_without_global_grid_  = this->get_parameter("ready_hold_without_global_grid").as_bool();   // [10-10 W1-3][리뷰]
  // [10-10 W1-4]
  global_costmap_fallback_topic_   = this->get_parameter("global_costmap_fallback_topic").as_string();
  global_costmap_fallback_service_ = this->get_parameter("global_costmap_fallback_service").as_string();
  global_costmap_fallback_sec_     = this->get_parameter("global_costmap_fallback_sec").as_double();
  global_costmap_event_driven_     = this->get_parameter("global_costmap_event_driven").as_bool();
  global_costmap_heartbeat_sec_    = std::max(0.0, this->get_parameter("global_costmap_heartbeat_sec").as_double());
  {
    GlobalSourceSelector::Config gc;
    gc.fallback_sec = global_costmap_fallback_sec_;
    gc.enabled = !global_costmap_fallback_topic_.empty() && !global_costmap_fallback_service_.empty() &&
      global_costmap_fallback_topic_ != global_costmap_topic_ && global_costmap_fallback_sec_ > 0.0;
    global_source_.configure(gc);
  }

  publish_agent_collision_    = this->get_parameter("publish_agent_collision").as_bool();
  agent_collision_topic_      = this->get_parameter("agent_collision_topic").as_string();

  agents_topic_               = this->get_parameter("agents_topic").as_string();
  agents_freshness_timeout_ms_= this->get_parameter("agents_freshness_timeout_ms").as_int();
  agent_cache_ttl_sec_        = this->get_parameter("agent_cache_ttl_sec").as_double();
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
  ignore_self_fp_hit_ = this->get_parameter("ignore_static_hit_in_self_footprint").as_bool();
  self_fp_margin_m_ = this->get_parameter("self_footprint_margin_m").as_double();

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
  mirror_subs_options_ = subs_options;   // [10-10 W1-4] source 전환 때 다시 쓴다

  // ===== Subscriptions =====
  // [10-10 W1] 토픽 이름을 파라미터로 (기본값 = 예전 문자열). QoS 는 예전 그대로.
  // [10-10 W1-2] 구독·client 생성은 setupMirrorSync 로 옮겼다 (global costmap 과 agent mask 가 같은 코드).
  global_sync_.name = "global costmap";
  global_sync_.mirror = &global_costmap_;
  global_sync_.clears_obstacle_db = true;
  global_sync_.topic = global_costmap_topic_;
  global_sync_.service = global_costmap_service_;
  global_sync_.resync_on_start = costmap_resync_on_start_;
  global_sync_.period_sec = costmap_resync_period_sec_;
  global_sync_.min_interval_sec = costmap_resync_min_interval_sec_;
  global_sync_.timeout_sec = costmap_resync_timeout_sec_;
  global_sync_.start_grace_sec = costmap_resync_start_grace_sec_;
  // [10-10 W1-4] primary 가 stock topic 과 같으면 (예전 설정) 무갱신 기준을 쓴다
  global_sync_.event_driven = global_costmap_event_driven_ && global_costmap_topic_ != global_costmap_fallback_topic_;
  setupMirrorSync(global_sync_, subs_options,
    &PathValidatorNode::costmapCallback, &PathValidatorNode::costmapUpdateCallback);

  // [10-10 W1-2] agent mask: 예전에는 full topic 만 구독해 콜백에서 같은 buffer 에 memcpy 했다 (reader 는 lock 밖에서
  //   읽음 → torn read). 이제 global 과 같은 CostmapMirror + update 병합 + get_costmap resync.
  if (compare_agent_mask_ && !agent_mask_topic_.empty()) {
    mask_sync_.name = "agent mask";
    mask_sync_.mirror = &agent_mask_;
    mask_sync_.clears_obstacle_db = false;
    mask_sync_.topic = agent_mask_topic_;
    mask_sync_.service = agent_mask_service_;
    mask_sync_.resync_on_start = agent_mask_resync_on_start_;
    mask_sync_.period_sec = agent_mask_resync_period_sec_;
    mask_sync_.min_interval_sec = costmap_resync_min_interval_sec_;
    mask_sync_.timeout_sec = costmap_resync_timeout_sec_;
    mask_sync_.start_grace_sec = costmap_resync_start_grace_sec_;
    mask_sync_.self_contained = agent_mask_self_contained_updates_;   // [10-10 W1-2][리뷰]
    // [10-10 W1-3] seed 는 mask 가 global costmap 노드의 layer 일 때만 (geometry 가 master 와 같다). "/global_costmap/..." 의
    //   첫 이름을 비교한다. 다르면 (local costmap mask 등) WARN 하고 예전처럼 get_costmap 으로 geometry 를 받는다.
    {
      auto first_ns = [](const std::string & t) {
          const size_t b = (!t.empty() && t[0] == '/') ? 1 : 0;
          const size_t e = t.find('/', b);
          return e == std::string::npos ? std::string() : t.substr(b, e - b);
        };
      const bool same_ns = !first_ns(agent_mask_topic_).empty() &&
        first_ns(agent_mask_topic_) == first_ns(global_costmap_topic_);
      mask_sync_.seed_from_global = agent_mask_seed_from_global_ && mask_sync_.self_contained && same_ns;
      if (agent_mask_seed_from_global_ && mask_sync_.self_contained && !same_ns) {
        RCLCPP_WARN(get_logger(),
          "[10-10 W1-3] agent_mask_topic(%s) 와 global_costmap_topic(%s) 의 namespace 가 다르다 — global geometry seed 를 쓰지 "
          "않는다 (mask geometry 는 get_costmap 으로 받는다)", agent_mask_topic_.c_str(), global_costmap_topic_.c_str());
      }
    }
    setupMirrorSync(mask_sync_, subs_options,
      &PathValidatorNode::agentMaskCallback, &PathValidatorNode::agentMaskUpdateCallback);
  }

  // [10-10 W1] 500 ms timer (subs 그룹). [10-10 W1-2] global 과 mask 를 차례로 tick 한다.
  costmap_resync_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&PathValidatorNode::costmapResyncTimerCallback, this),
      subs_callback_group_);

  agents_sub_ = this->create_subscription<robot_interfaces::msg::MultiAgentInfoArray>(
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
    static_collision_pub_ = this->create_publisher<robot_interfaces::msg::PathStaticCollisionInfo>(
        "/path_static_collision_info", rclcpp::QoS(10).reliable());
  }
  if (publish_agent_collision_) {
    agent_collision_pub_ = this->create_publisher<robot_interfaces::msg::PathAgentCollisionInfo>(
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
     << "  - [V2] ignore_static_hit_in_self_footprint: " << (ignore_self_fp_hit_ ? "true" : "false")
     << " (margin " << self_fp_margin_m_ << ")\n"
     << " [Global Costmap Mirror] [10-10 W1]\n"
     << "  - global_costmap_topic: " << global_costmap_topic_ << " (+ _updates, depth " << costmap_updates_qos_depth_ << ")\n"
     << "  - global_costmap_service: " << global_costmap_service_ << "\n"
     << "  - costmap_resync_on_start: " << (costmap_resync_on_start_ ? "true" : "false") << "\n"
     << "  - costmap_resync_period_sec: " << costmap_resync_period_sec_ << "\n"
     << "  - costmap_resync_min_interval_sec: " << costmap_resync_min_interval_sec_ << "\n"
     << "  - costmap_resync_timeout_sec: " << costmap_resync_timeout_sec_ << "\n"
     << "  - costmap_keep_spare_buffer: " << (costmap_keep_spare_buffer_ ? "true" : "false") << "\n"
     << "  - costmap_resync_start_grace_sec: " << costmap_resync_start_grace_sec_ << "\n"
     << " [Global Source] [10-10 W1-4]\n"
     << "  - global_costmap_fallback_topic: " << global_costmap_fallback_topic_ << "\n"
     << "  - global_costmap_fallback_service: " << global_costmap_fallback_service_ << "\n"
     << "  - global_costmap_fallback_sec: " << global_costmap_fallback_sec_
     << " (fallback " << (global_source_.enabled() ? "켬" : "끔") << ")\n"
     << "  - global_costmap_heartbeat_sec: " << global_costmap_heartbeat_sec_
     << " ([10-10 W1-4][리뷰] event_driven source 무갱신 한도, 0 = 끔)\n"
     << "  - global_costmap_event_driven: " << (global_costmap_event_driven_ ? "true" : "false")
     << " (watchdog 기준: " << (global_sync_.event_driven ? "publisher liveness + heartbeat" : "무갱신") << ")\n"
     << " [Agent Mask Mirror] [10-10 W1-2]\n"
     << "  - agent_mask_topic: " << agent_mask_topic_ << " (+ _updates)\n"
     << "  - agent_mask_service: " << agent_mask_service_ << "\n"
     << "  - agent_mask_resync_on_start: " << (agent_mask_resync_on_start_ ? "true" : "false") << "\n"
     << "  - agent_mask_resync_period_sec: " << agent_mask_resync_period_sec_ << "\n"
     << "  - agent_mask_self_contained_updates: " << (agent_mask_self_contained_updates_ ? "true" : "false") << "\n"
     << "  - costmap_replay_max_msgs: " << costmap_replay_max_msgs_ << "\n"
     << " [Mirror Sync] [10-10 W1-3]\n"
     << "  - agent_mask_seed_from_global: " << (agent_mask_seed_from_global_ ? "true" : "false")
     << " (적용: " << (mask_sync_.seed_from_global ? "yes" : "no") << ")\n"
     << "  - costmap_resync_backoff_base_sec: " << costmap_resync_backoff_base_sec_
     << " (max " << costmap_resync_backoff_max_sec_ << ")\n"
     << "  - costmap_stale_watchdog_sec: " << costmap_stale_watchdog_sec_ << " (global 만, 0 = 끔)\n"
     << "  - costmap_resync_abandon_sec: " << costmap_resync_abandon_sec_
     << " ([리뷰] timeout " << costmap_resync_timeout_sec_ << " 은 늦음 WARN, 늦은 응답도 쓴다)\n"
     << "  - ready_hold_without_global_grid: " << (ready_hold_without_global_grid_ ? "true" : "false") << " ([리뷰])\n"
     << " [Goal Doorstep]\n"
     << "  - goal_doorstep_static_m: " << goal_doorstep_static_m_ << "\n"
     << "  - goal_doorstep_agent_m: " << goal_doorstep_agent_m_ << "\n"
     << "=======================================================";

  // 생성된 문자열을 INFO 레벨로 출력
  RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

}

// ===================== Costmap handling =====================

// [10-10 W1-2] full 구독 + update 구독(+ MESSAGE_LOST) + get_costmap client. step 1 의 global 전용 생성 코드를 옮겼다.
//   QoS 와 callback group 은 step 1 그대로다.
void PathValidatorNode::setupMirrorSync(
  MirrorSync & s, const rclcpp::SubscriptionOptions & subs_options,
  void (PathValidatorNode::*full_cb)(const nav2_msgs::msg::Costmap::SharedPtr),
  void (PathValidatorNode::*update_cb)(const nav2_msgs::msg::CostmapUpdate::ConstSharedPtr))
{
  // [10-10 W1-3] 요청 일정 (간격·timeout·backoff). 두 mirror 가 같은 값을 쓴다.
  ResyncScheduler::Config sc;
  sc.min_interval_sec = s.min_interval_sec;
  sc.timeout_sec = s.timeout_sec;
  sc.backoff_base_sec = costmap_resync_backoff_base_sec_;
  sc.backoff_max_sec = costmap_resync_backoff_max_sec_;
  sc.abandon_sec = costmap_resync_abandon_sec_;   // [10-10 W1-3][리뷰]
  s.sched.configure(sc);

  s.full_cb = full_cb;   // [10-10 W1-4] source 전환 때 다시 만든다
  s.update_cb = update_cb;
  (void)subs_options;    // [10-10 W1-4] mirror_subs_options_ 와 같다 (생성자에서 저장)
  createMirrorIo(s);
  // [10-10 W1][리뷰] 시작 resync 는 바로 걸지 않는다 (resyncTick 의 start_resync_pending 참고).
  s.start_resync_pending = s.resync_on_start;
  s.node_start_steady = std::chrono::steady_clock::now();
  s.source_since = s.node_start_steady;   // [10-10 W1-4]
}

// [10-10 W1-4] 구독·client 생성 (setupMirrorSync 에서 옮김 + source 세대 검사). 처음 만들 때와 global source 전환 때 부른다.
//   subs 그룹 (MutuallyExclusive) 콜백 안 또는 생성자에서만 부른다 → 같은 그룹의 다른 mirror 콜백과 겹치지 않는다.
void PathValidatorNode::createMirrorIo(MirrorSync & s)
{
  // [10-10 W1-3][리뷰] 이 구독의 ROS 시각. 이보다 먼저 찍힌 self-contained update 는 이전 PV 가 받던 latched 것이다
  //   (구독자가 없으면 AgentLayer 는 update 를 만들지 않는다). sim time 이 아직 0 이면 resyncTick 에서 처음 유효할 때 채운다.
  s.subscribe_ros_ns = this->now().nanoseconds();

  const auto & subs_options = mirror_subs_options_;
  // [10-10 W1-4] 콜백은 만들어질 때의 source 세대를 든다. 전환 뒤에 옛 구독의 메시지가 (executor 가 이미 꺼낸 것 등) 들어오면 버린다.
  MirrorSync * sp = &s;   // s 는 node 멤버라 콜백보다 오래 산다
  const uint64_t gen = s.source_gen;
  const auto full_cb = s.full_cb;
  const auto update_cb = s.update_cb;
  s.full_sub = this->create_subscription<nav2_msgs::msg::Costmap>(
      s.topic,
      rclcpp::QoS(rclcpp::SystemDefaultsQoS()).transient_local().reliable(),
      [this, sp, gen, full_cb](const nav2_msgs::msg::Costmap::SharedPtr msg) {
        if (!isCurrentSourceGen(gen, sp->source_gen)) {
          ++sp->stale_source_callbacks;
          return;
        }
        (this->*full_cb)(msg);
      },
      subs_options);

  // [10-10 W1] update 토픽 = full 토픽 + "_updates" (Nav2 Costmap2DPublisher: "costmap" + "_raw_updates").
  //   transient_local + reliable 은 full 과 같고, 깊이만 KeepLast(N) 로 키웠다: SystemDefaults(Cyclone 은 깊이 1)
  //   이면 subs 그룹이 200 MB full 을 처리하는 동안 쌓인 update 가 조용히 덮어써지고, CostmapUpdate 에는
  //   순번이 없어(Cyclone 은 publication_sequence_number 미지원) 유실을 알아챌 방법이 없다.
  // [10-10 W1][리뷰] rmw 가 알려 주는 유실(MESSAGE_LOST, Cyclone 은 SAMPLE_LOST)은 resync 로 메운다. event handler 는
  //   subscription 의 callback group(subs 그룹)에서 돌므로 resync 상태를 그대로 만져도 된다. reader 쪽 KeepLast
  //   덮어쓰기는 DDS 가 유실로 알리지 않으므로 깊이를 넉넉히(기본 100) 둔 것이 유일한 대책이다.
  //   rmw 가 이 event 를 지원하지 않으면 콜백 없이 다시 만든다.
  {
    const auto updates_qos =
      rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(costmap_updates_qos_depth_))).transient_local().reliable();
    auto update_options = subs_options;
    update_options.event_callbacks.message_lost_callback =
      [this, sp, gen](rclcpp::QOSMessageLostInfo & info) {
        if (!isCurrentSourceGen(gen, sp->source_gen)) {   // [10-10 W1-4] 옛 source 의 유실 보고
          ++sp->stale_source_callbacks;
          return;
        }
        // [10-10 W1-3] S2-OPS-10: WARN 을 5 s 로 throttle 하고 보고 횟수·누적 유실을 남긴다.
        ++sp->message_lost_reports;
        sp->message_lost_total += info.total_count_change;
        // [10-10 W1-2][리뷰] self_contained 면 다음 update 하나가 mask 전체를 다시 정하므로 200 MB get_costmap 을 부르지 않는다
        //   (AgentLayer 의 refresh 가 "더 보낼 update 가 없는" 경우도 덮는다).
        if (sp->self_contained) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "[10-10 W1-2][리뷰] %s update 유실 보고 (MESSAGE_LOST, 이번 %zu / rmw 누적 %zu, 보고 %lu 회 / 합 %lu) — "
            "self-contained: 다음 update 로 맞춰진다, resync 안 함",
            sp->name.c_str(), info.total_count_change, info.total_count,
            static_cast<unsigned long>(sp->message_lost_reports), static_cast<unsigned long>(sp->message_lost_total));
          return;
        }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "[10-10 W1] %s update 유실 보고 (MESSAGE_LOST, 이번 %zu / rmw 누적 %zu, 보고 %lu 회 / 합 %lu) — get_costmap resync 요청",
          sp->name.c_str(), info.total_count_change, info.total_count,
          static_cast<unsigned long>(sp->message_lost_reports), static_cast<unsigned long>(sp->message_lost_total));
        // [10-10 W1-3][리뷰] 내용 유실: 요청 진행 중이면 응답 뒤 한 번 더 받는다 (유실이 서버 스냅샷 뒤였을 수 있다)
        requestResync(*sp, "update message lost", ResyncScheduler::Trigger::kContentLoss);
      };
    auto update_lambda = [this, sp, gen, update_cb](const nav2_msgs::msg::CostmapUpdate::ConstSharedPtr msg) {
        if (!isCurrentSourceGen(gen, sp->source_gen)) {   // [10-10 W1-4]
          ++sp->stale_source_callbacks;
          return;
        }
        (this->*update_cb)(msg);
      };
    try {
      s.update_sub = this->create_subscription<nav2_msgs::msg::CostmapUpdate>(
          s.topic + "_updates", updates_qos,
          update_lambda,
          update_options);
    } catch (const rclcpp::UnsupportedEventTypeException & e) {
      RCLCPP_WARN(get_logger(), "[10-10 W1] %s update 구독에 MESSAGE_LOST event 를 못 단다 (%s) — event 없이 구독",
        s.name.c_str(), e.what());
      s.update_sub = this->create_subscription<nav2_msgs::msg::CostmapUpdate>(
          s.topic + "_updates", updates_qos,
          update_lambda,
          subs_options);
    }
  }

  // [10-10 W1] get_costmap client. 응답 콜백도 subs 그룹에서 돈다 → resync 상태·mirror writer 가 한 그룹에서 직렬화된다.
  //   executor thread 에서 응답을 기다리지 않는다 (async_send_request + 콜백, 준비 여부는 timer 가 service_is_ready() 로 본다).
  s.client = this->create_client<nav2_msgs::srv::GetCostmap>(
      s.service, rclcpp::ServicesQoS(), subs_callback_group_);
}

void PathValidatorNode::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  // [10-10 W1] 예전: costmap_mutex_ 아래에서 같은 buffer 에 memcpy → reader 는 shared_ptr 만 복사해 lock 밖에서
  //   읽었으므로 memcpy 도중의 반쯤 바뀐 grid 를 읽을 수 있었다 (torn read). 이제 mirror 가 새 grid 를 lock 밖에서
  //   만들고 exclusive lock 아래에서 swap 하며, reader 는 ReadGuard 를 쥔 동안만 읽는다.
  onTopicFull(global_sync_, *msg);   // [10-10 W1-2]
}

void PathValidatorNode::costmapUpdateCallback(const nav2_msgs::msg::CostmapUpdate::ConstSharedPtr msg)
{
  onUpdate(global_sync_, msg);   // [10-10 W1-2]
}

// [10-10 W1-2] 예전 agentMaskCallback 은 크기가 바뀌면 새 grid 를 만들고 같은 buffer 에 memcpy 했다 (reader 는
//   shared_ptr 만 복사해 lock 밖에서 읽음 → torn read). 크기 불일치 msg 는 예전에는 geometry 가 바뀐 경우 빈 grid 를
//   남겼지만, 이제는 global 과 같이 버리고 기존 grid 를 유지한다.
void PathValidatorNode::agentMaskCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  onTopicFull(mask_sync_, *msg);
}

void PathValidatorNode::agentMaskUpdateCallback(const nav2_msgs::msg::CostmapUpdate::ConstSharedPtr msg)
{
  onUpdate(mask_sync_, msg);
}

// [10-10 W1-2] step 1 의 costmapCallback 본문 (s 로 일반화).
void PathValidatorNode::onTopicFull(MirrorSync & s, const nav2_msgs::msg::Costmap & msg)
{
  s.any_rx = true;                                   // [10-10 W1-3] watchdog
  s.last_rx = std::chrono::steady_clock::now();
  const bool first_since_subscribe = (s.topic_fulls_received++ == 0);   // [10-10 W1-3] W2PV-2

  // 크기 불일치 msg 는 예전에는 (geometry 가 바뀐 경우) 빈 FREE grid 로 바꿨지만, 이제는 버리고 기존 grid 를 유지한다.
  // [10-10 W1-2][리뷰] self_contained (agent mask): start grace 보다 오래된 topic full 은 늦게 붙은 구독자가 받은 latched
  //   activation full 이다 (AgentLayer 는 full 을 첫 publish·크기 변경 때만 보낸다). 그때 칠한 몸체를 살아 있는 agent 로
  //   내보내지 않도록 geometry 만 쓰고 내용은 0 으로 둔다. 빠진 셀은 다음 update 가 채운다.
  // [10-10 W1-3] W2PV-2: 예전에는 self_contained 이면 **모든** topic full 에 이 규칙을 걸어, update 를 보내지 않는 예전
  //   AgentLayer(또는 full_mask_publish:true)의 full 이 늦게 오면 mask 가 0 이 됐다. 이제 (재)구독 뒤 첫 full 이고 update 를
  //   한 번도 받지 않았을 때만 geometry_only 이고, 같은 geometry 의 grid 가 이미 있으면 (seed 한 0 grid 등) 0 으로 다시
  //   만들지 않고 버린다 (decideMaskTopicFull).
  bool geometry_only = false;
  double full_age = 0.0;
  if (s.self_contained) {
    full_age = (this->now() -
      rclcpp::Time(msg.header.stamp, this->get_clock()->get_clock_type())).seconds();
    const auto st0 = s.mirror->stats();
    const bool same_geom = st0.has_grid &&
      CostmapSignature::fromMsg(msg).nearlyEquals(st0.sig, 1e-6, 1e-9);
    const bool any_update = s.fresh_update_seen || st0.updates > 0;
    const auto act = decideMaskTopicFull(true, first_since_subscribe, any_update, full_age,
      s.start_grace_sec, same_geom);
    if (act == MaskTopicFullAction::kDropOld) {
      ++s.old_fulls_dropped;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "[10-10 W1-3] %s topic full 이 %.1f s 전 것 (latched) 이고 같은 geometry grid 가 이미 있다 — 버림 (0 으로 다시 만들지 않음, "
        "누적 %lu)", s.name.c_str(), full_age, static_cast<unsigned long>(s.old_fulls_dropped));
      return;
    }
    geometry_only = (act == MaskTopicFullAction::kApplyGeometryOnly);
  }
  const auto out = s.mirror->applyFull(msg, CostmapMirror::kNoRequestGen, geometry_only);
  if (out.result == CostmapMirror::FullResult::kRejectedSuperseded) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "[10-10 W1-2][리뷰] %s: 이미 적용한 update 보다 오래된 topic full — 버림 (self-contained, resync 불필요)",
      s.name.c_str());
    return;
  }
  if (geometry_only && out.applied()) {
    // [10-10 W1-3] throttle (W2PV-2)
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "[10-10 W1-2][리뷰] %s topic full 이 %.1f s 전 것 (latched) — geometry 만 쓰고 내용은 0 (다음 update 가 채운다)",
      s.name.c_str(), full_age);
  }
  // [10-10 W1-4][리뷰] 빈 topic full (0x0 / resolution 0) — 지금 publisher 들은 보내지 않지만 (diff layer 는 geometry 가 없으면 FULL
  //   token 을 버린다) 받으면 적용하지 않고 topic full 로도 세지 않는다 (시작 resync 판정이 틀어지지 않게).
  if (out.result == CostmapMirror::FullResult::kRejectedEmpty) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
      "[10-10 W1-4][리뷰] %s 빈 topic full (%ux%u res %.3f) — 버림 (grid 그대로)",
      s.name.c_str(), msg.metadata.size_x, msg.metadata.size_y, static_cast<double>(msg.metadata.resolution));
    return;
  }
  if (out.result == CostmapMirror::FullResult::kRejectedBadSize) {
    const size_t expected = static_cast<size_t>(msg.metadata.size_x) * static_cast<size_t>(msg.metadata.size_y);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "%s data size mismatch: got %zu, expected %zu",
      s.name.c_str(), msg.data.size(), expected);
    return;
  }
  // [10-10 W1][리뷰] 현재 grid 보다 오래된 topic full 은 버린다 (False 에서 늦게 붙을 때 get_costmap 응답 뒤에 도착한
  //   latched activation full). clock reset 일 수도 있으므로 resync 를 걸어 둔다: 그 응답은 요청 뒤 스냅샷이라
  //   mirror 가 clock reset 을 판정할 수 있다. latched full 은 한 번만 오므로 비용은 GetCostmap 한 번이다.
  if (out.result == CostmapMirror::FullResult::kRejectedStale) {
    const auto st = s.mirror->stats();
    RCLCPP_INFO(get_logger(),
      "[10-10 W1] %s: 현재 grid(stamp %.3f) 보다 오래된 topic full(stamp %.3f, latched 로 보임) — 버리고 get_costmap resync 요청",
      s.name.c_str(), st.last_full_stamp_ns * 1e-9, rclcpp::Time(msg.header.stamp).nanoseconds() * 1e-9);
    requestResync(s, "older topic full");
    return;
  }
  if (++s.topic_fulls_applied == 1) {
    s.first_topic_full_at = std::chrono::steady_clock::now();   // [10-10 W1][리뷰] 시작 resync grace 기준
  }
  // [10-10 W1][리뷰] 시작 resync 대기 중인데 받은 full 이 오래된 것이면(False 의 latched activation full) 바로 resync.
  //   node clock 과 Nav2 clock 이 다르면(sim/wall 섞임) 나이가 크게 나와 바로 resync 하는 쪽으로만 틀린다.
  if (s.start_resync_pending && s.self_contained) {
    // [10-10 W1-2][리뷰] self_contained: grid 가 생겼으니 시작 resync 는 필요 없다 (update 하나로 맞는다).
    s.start_resync_pending = false;
  }
  if (s.start_resync_pending) {
    const double age = (this->now() -
      rclcpp::Time(msg.header.stamp, this->get_clock()->get_clock_type())).seconds();
    if (age > s.start_grace_sec) {
      s.start_resync_pending = false;
      RCLCPP_INFO(get_logger(),
        "[10-10 W1] %s 첫 topic full 이 %.1f s 전 것 (latched) — 시작 get_costmap resync 요청", s.name.c_str(), age);
      requestResync(s, "start (old latched full)");
    }
  }
  onFullApplied(s, out, "topic");
}

// [10-10 W1] full 적용 뒤 처리 (topic / get_costmap 공통, subs 그룹 전용).
//   geometry 가 바뀌었으면 obstacle_db_ 를 비운다 (예전 costmapCallback 과 같음, global 만). grid lock 은 이미 놓은 상태라
//   lock 순서(grid → obstacle_db_)를 거스르지 않는다. 바뀌기 전 grid 로 계산한 key 는 updateObstacleDatabase 가
//   ReadGuard 를 쥔 채 넣으므로 swap 전에 들어가 있고, 여기서 함께 지워진다.
// [10-10 W1-2] step 1 의 onGlobalFullApplied 를 s 로 일반화.
void PathValidatorNode::onFullApplied(MirrorSync & s, const CostmapMirror::FullOutcome & out, const char * source)
{
  if (!out.applied()) {
    return;
  }
  const auto st = s.mirror->stats();
  // [10-10 W1][리뷰] clock reset 이거나, 이 full 보다 새 update 중 replay 보관을 못 한 것이 있으면 다시 받는다.
  if (out.clock_reset) {
    RCLCPP_WARN(get_logger(),
      "[10-10 W1] %s: get_costmap 응답 stamp 가 현재 grid 보다 오래됐는데 요청 뒤 스냅샷이다 — Nav2 clock reset 으로 보고 "
      "적용 (replay 버림), 한 번 더 resync", s.name.c_str());
  }
  if (out.needs_resync) {
    if (!out.clock_reset) {
      // [10-10 W1-2][리뷰] 예전에는 unreplayable_updates(정상 동작에서도 계속 느는 밀려난 수)를 찍어 뜻이 없었다.
      RCLCPP_WARN(get_logger(),
        "[10-10 W1] %s: %s full 보다 새 update 중 replay 보관 못 한 것이 있다 (보관 한도 %d 개, 이 때문에 다시 받은 횟수 %lu) "
        "— get_costmap resync 다시 요청",
        s.name.c_str(), source, costmap_replay_max_msgs_, static_cast<unsigned long>(st.replay_loss_resyncs));
    }
    // [10-10 W1-3][리뷰] replay 를 못 한 update 는 내용 유실이다 (topic full 적용 중 get_costmap 이 진행 중이어도 기억한다)
    requestResync(s, out.clock_reset ? "clock reset" : "unreplayable update",
      out.clock_reset ? ResyncScheduler::Trigger::kStructural : ResyncScheduler::Trigger::kContentLoss);
  }
  if (out.result == CostmapMirror::FullResult::kAppliedNewSignature) {
    if (s.clears_obstacle_db) {
      std::lock_guard<std::mutex> db_lock(obstacle_db_mutex_);
      obstacle_db_.clear();
    }
    if (s.first_full_logged) {
      RCLCPP_INFO(get_logger(),
        "[10-10 W1] %s geometry 변경 (%s): %ux%u res %.3f origin (%.2f, %.2f)%s",
        s.name.c_str(), source, st.sig.size_x, st.sig.size_y, st.sig.resolution, st.sig.origin_x, st.sig.origin_y,
        s.clears_obstacle_db ? " — obstacle_db 초기화" : "");
    }
  }
  if (!s.first_full_logged) {
    s.first_full_logged = true;
    RCLCPP_INFO(get_logger(),
      "[10-10 W1] %s 첫 full 수신 (%s): %ux%u res %.3f origin (%.2f, %.2f), %.1f MB, 먼저 온 update %lu 개 replay",
      s.name.c_str(), source, st.sig.size_x, st.sig.size_y, st.sig.resolution, st.sig.origin_x, st.sig.origin_y,
      static_cast<double>(st.sig.size_x) * st.sig.size_y / (1024.0 * 1024.0),
      static_cast<unsigned long>(st.replayed_updates));
  }
  // [10-10 W1-3] global grid 가 생겼으면 mask 를 그 geometry 로 seed 한다 (mask 가 아직 없을 때만).
  if (&s == &global_sync_) {
    trySeedMaskFromGlobal("global full 적용");
  }
}

// [10-10 W1-3] W2PV-1: 현장 순서 (Nav2 activation 뒤 PV 시작·respawn) 에서는 mask full 이 latch 되지 않는다 (AgentLayer 의
//   첫 full 때 구독자가 없었다). 예전에는 그때 mask 를 200 MB get_costmap 으로만 받아, 응답(bigmap 약 16~35 s)이 올 때까지
//   mask 가 null 이었다 → 이웃 차단이 STATIC, M-6 이 이웃 몸체를 건너뜀. AgentLayer 의 mask grid 는 master 와 같은 geometry 다
//   (agent_mask_canvas beginCycle). 그래서 global mirror 의 geometry 로 0 grid 를 만들고, 다음 self-contained update 를 바로
//   적용한다 (update 하나 = mask 전체). 이웃이 있는데 update 가 아직 없으면 agentMaskMode() 가 "모름" 으로 본다.
bool PathValidatorNode::trySeedMaskFromGlobal(const char * why)
{
  MirrorSync & m = mask_sync_;
  if (!m.mirror || !m.self_contained || !m.seed_from_global) {
    return false;
  }
  if (agent_mask_.hasGrid() || !global_costmap_.hasGrid()) {
    return false;
  }
  const auto gst = global_costmap_.stats();
  const auto r = agent_mask_.seedZero(gst.sig);
  if (!r.seeded) {
    return false;
  }
  m.start_resync_pending = false;
  m.sched.cancel();   // geometry 를 얻었으니 "grid 없음" 때문에 걸린 get_costmap 은 필요 없다 (진행 중이면 그대로 둔다)
  if (r.replayed > 0) {
    m.fresh_update_seen = true;
  }
  if (!m.first_full_logged) {
    m.first_full_logged = true;
  }
  RCLCPP_INFO(get_logger(),
    "[10-10 W1-3] agent mask 를 global geometry 로 0 grid 로 시작 (%s): %ux%u res %.3f origin (%.2f, %.2f), "
    "먼저 온 update %s — mask get_costmap 생략",
    why, gst.sig.size_x, gst.sig.size_y, gst.sig.resolution, gst.sig.origin_x, gst.sig.origin_y,
    r.replayed > 0 ? "1 개 적용" : "없음 (이웃이 있으면 첫 update 까지 '모름')");
  return true;
}

// [10-10 W1] CostmapUpdate 병합. update 마다 로그를 남기지 않는다 (WARN/INFO 는 throttle).
// [10-10 W1-2] step 1 의 costmapUpdateCallback 본문 (s 로 일반화).
void PathValidatorNode::onUpdate(MirrorSync & s, const nav2_msgs::msg::CostmapUpdate::ConstSharedPtr & msg)
{
  const auto now_steady = std::chrono::steady_clock::now();
  s.any_rx = true;          // [10-10 W1-3] watchdog
  s.last_rx = now_steady;

  // [10-10 W1-3] W2PV-1: (재)구독 직후 받은 오래된 latched update 는 버린다 (dropLatchedMaskUpdate 주석).
  if (s.self_contained && msg) {
    const double age = (this->now() -
      rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type())).seconds();
    const double since_sub = std::chrono::duration<double>(now_steady - s.node_start_steady).count();
    // [10-10 W1-3][리뷰] 출처로 먼저 거른다: 이 구독보다 0.2 s 넘게 먼저 찍힌 update 는 이전 PV instance 가 받던 latched 것이다
    //   (respawn 이 빨라 나이가 start_grace 안쪽이어도). 구독 시각을 모르면 (sim time 0) 나이 규칙만 쓴다.
    const int64_t stamp_ns =
      rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type()).nanoseconds();
    const bool before_sub = s.subscribe_ros_ns > 0 && stamp_ns < s.subscribe_ros_ns - 200000000LL;
    if (dropLatchedMaskUpdate(true, s.fresh_update_seen, since_sub, 4.0 * s.start_grace_sec, age, s.start_grace_sec,
      before_sub))
    {
      ++s.stale_latched_dropped;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "[10-10 W1-3] %s: 구독 직후 받은 update 가 %.1f s 전 것 (latched, PV 가 없던 동안의 마지막 update 로 보임%s) — 버림 "
        "(누적 %lu). 이웃이 있으면 곧 새 update 가 온다", s.name.c_str(), age,
        before_sub ? ", [리뷰] 구독보다 먼저 찍힘" : "",
        static_cast<unsigned long>(s.stale_latched_dropped));
      return;
    }
    s.fresh_update_seen = true;
  }

  auto r = s.mirror->applyUpdate(msg);
  // [10-10 W1-3] mask 에 grid 가 없으면 global geometry 로 seed 한다 (방금 update 는 replay 에 보관돼 있어 seed 가 적용한다).
  //   seed 가 이 update 를 이미 적용했어도 다시 적용하는 것은 같은 결과다 (self-contained). 맞지 않으면 범위 밖으로 처리된다.
  if (r == CostmapMirror::UpdateResult::kNoGrid && &s == &mask_sync_ && trySeedMaskFromGlobal("update 수신")) {
    r = s.mirror->applyUpdate(msg);
  }
  // [10-10 W1][리뷰] update 가 온다 = always_send_full_costmap: False. latched full 은 옛것일 수 있으니 시작 resync 를 바로 건다.
  if (s.start_resync_pending) {
    s.start_resync_pending = false;
    // [10-10 W1-2][리뷰] self_contained 는 grid 만 있으면 이 update 로 맞는다. grid 가 없으면 아래 kNoGrid 가 resync 를 건다.
    if (!s.self_contained) {
      requestResync(s, "start (update 수신)");
    }
  }
  switch (r) {
    case CostmapMirror::UpdateResult::kApplied:
    case CostmapMirror::UpdateResult::kStale:
      return;
    case CostmapMirror::UpdateResult::kNoGrid:
      s.no_grid_update_seen = true;
      // [10-10 W1-3] seed 할 수 있는 mask 는 global grid 를 기다린다 (seed 가 geometry 를 준다). get_costmap 은 resyncTick 의
      //   fallback (시작 뒤 grace×4 에도 global grid 가 없을 때) 만 부른다.
      if (s.self_contained && s.seed_from_global) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
          "[10-10 W1-3] %s: grid 보다 update 가 먼저 왔다 — 보관해 두고 global grid 를 기다린다 (seed)", s.name.c_str());
        return;
      }
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "[10-10 W1] %s: full 보다 update 가 먼저 왔다 — 보관해 두고 get_costmap resync 요청", s.name.c_str());
      requestResync(s, "update before full");
      return;
    case CostmapMirror::UpdateResult::kRejectedBadSize:
    case CostmapMirror::UpdateResult::kRejectedOutOfBounds:
      {
        const auto st = s.mirror->stats();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "[10-10 W1] %s update 버림 (%s): rect (%u,%u) %ux%u data %zu, grid %ux%u — 누적 dropped %lu, "
          "get_costmap resync 요청",
          s.name.c_str(),
          r == CostmapMirror::UpdateResult::kRejectedBadSize ? "data 크기 불일치" : "grid 범위 밖",
          msg->x, msg->y, msg->size_x, msg->size_y, msg->data.size(), st.sig.size_x, st.sig.size_y,
          static_cast<unsigned long>(st.dropped_updates));
        // [10-10 W1-3][리뷰] data 크기 불일치는 replay 에도 보관되지 않는다 (내용 유실). 범위 밖은 보관돼 새 geometry 의 응답·full
        //   뒤에 다시 적용되므로 구조 트리거다. self-contained 는 다음 update 가 mask 전체라 유실로 보지 않는다.
        requestResync(s, "update dropped",
          (r == CostmapMirror::UpdateResult::kRejectedBadSize && !s.self_contained) ?
          ResyncScheduler::Trigger::kContentLoss : ResyncScheduler::Trigger::kStructural);
        return;
      }
  }
}

// [10-10 W1] resync 가 필요하다고 표시하고 곧바로 보낼 수 있으면 보낸다 (간격 제한은 timer 콜백이 지킨다).
// [10-10 W1-3] 요청이 진행 중이면 트리거를 올리지 않는다 (W2PV-3: 예전에는 kNoGrid update 5 Hz 마다 올라가 응답 뒤 같은
//   200 MB 요청이 한 번 더 나갔다). 진행 중인 요청의 응답이 그 필요를 채우고, 채우지 못하면 응답 처리가 다시 트리거한다.
// [10-10 W1-3][리뷰] 단 내용 유실 (kind = kContentLoss) 은 진행 중이어도 기억해 응답 뒤 한 번 더 받는다.
void PathValidatorNode::requestResync(MirrorSync & s, const char * reason, ResyncScheduler::Trigger kind)
{
  const bool was_needed = s.sched.needed();
  if (!s.sched.trigger(kind)) {
    if (kind == ResyncScheduler::Trigger::kContentLoss) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "[10-10 W1-3][리뷰] %s get_costmap 진행 중에 내용 유실 (%s) — 응답 뒤 한 번 더 받는다 (진행 중 유실 누적 %lu)",
        s.name.c_str(), reason, static_cast<unsigned long>(s.sched.counters().loss_in_flight));
      return;
    }
    RCLCPP_DEBUG(get_logger(), "[10-10 W1-3] %s get_costmap 진행 중 — 트리거 무시: %s", s.name.c_str(), reason);
    return;
  }
  if (!was_needed) {
    RCLCPP_DEBUG(get_logger(), "[10-10 W1] %s get_costmap resync 필요: %s", s.name.c_str(), reason);
  }
  resyncTick(s);
}

// [10-10 W1] 500 ms timer (subs 그룹). [10-10 W1-2] global 다음 mask 를 tick 한다.
// [10-10 W1-3] + global watchdog (R4).
void PathValidatorNode::costmapResyncTimerCallback()
{
  globalSourceTick();   // [10-10 W1-4] publisher 수 (watchdog liveness) + diff ↔ stock 전환. resyncTick 전에.
  resyncTick(global_sync_);
  if (mask_sync_.mirror) {
    resyncTick(mask_sync_);
  }
  globalWatchdogTick();
}

// [10-10 W1] get_costmap 요청 관리 (subs 그룹). 절대 기다리지 않는다.
// [10-10 W1-2] step 1 의 costmapResyncTimerCallback 본문 (s 로 일반화).
// [10-10 W1-3] 요청 일정은 ResyncScheduler (진행 중 트리거 무시, timeout 뒤 backoff, global → mask 순서).
void PathValidatorNode::resyncTick(MirrorSync & s)
{
  using Clock = std::chrono::steady_clock;
  const auto now = Clock::now();
  auto secs = [](Clock::duration d) {return std::chrono::duration<double>(d).count();};

  if (!s.mirror || !s.client) {
    return;
  }

  // [10-10 W1][리뷰] full 이 끊긴 뒤 남은 spare grid 해제.
  s.mirror->releaseIdleSpare();

  // [10-10 W1-3][리뷰] 구독 때 sim time 이 0 이었으면 처음 유효한 시각을 구독 시각으로 쓴다 (늦게 잡혀 0.5 s 안쪽 오차 —
  //   그동안 막 온 update 를 버려도 0 grid 쪽으로만 틀리고, 이웃이 있으면 다음 update 가 곧 온다).
  if (s.subscribe_ros_ns <= 0) {
    const int64_t n = this->now().nanoseconds();
    if (n > 0) {
      s.subscribe_ros_ns = n;
    }
  }

  // [10-10 W1-3] mask 는 global geometry 로 seed 할 수 있으면 get_costmap 대신 seed 한다.
  if (&s == &mask_sync_ && !s.mirror->hasGrid()) {
    trySeedMaskFromGlobal("timer");
  }

  // [10-10 W1][리뷰] 시작 resync (update 수신 / 오래된 latched full 이면 각 콜백에서 이미 걸었다):
  //   - 첫 topic full 뒤 grace 안에 topic full 이 또 오면 True 로 보고 생략한다.
  //   - 첫 topic full 뒤 grace 동안 두 번째가 없으면 (False) 건다.
  //   - topic full 이 하나도 없으면 (False 에서 activation 때 구독자가 없어 full 이 안 나간 경우 등) 시작 뒤
  //     grace × 4 가 지나면 건다. Nav2 가 늦게 뜨는 경우 True 의 첫 full 을 기다리려고 길게 잡았다.
  if (s.self_contained) {
    // [10-10 W1-2][리뷰] self_contained: grid(geometry) 가 없을 때만 grace × 4 뒤에 건다.
    // [10-10 W1-3] seed 를 쓰면 이것은 "global grid 도 없을 때" 의 fallback 이다 (geometry 를 받을 곳이 get_costmap 뿐).
    //   update 가 grid 보다 먼저 온 경우(no_grid_update_seen)도 같은 시점에 건다 (예전: 그 즉시).
    if (s.mirror->hasGrid()) {
      s.start_resync_pending = false;
    } else if ((s.start_resync_pending || s.no_grid_update_seen) &&
      secs(now - s.node_start_steady) >= 4.0 * s.start_grace_sec)
    {
      s.start_resync_pending = false;
      s.no_grid_update_seen = false;
      if (s.sched.trigger()) {
        RCLCPP_INFO(get_logger(),
          "[10-10 W1-3] %s: 시작 뒤 %.0f s 동안 grid 가 없다 (global grid 로 seed 못 함) — get_costmap fallback",
          s.name.c_str(), 4.0 * s.start_grace_sec);
      }
    }
  } else if (s.start_resync_pending) {
    bool fire = false;
    if (s.topic_fulls_applied >= 2) {
      s.start_resync_pending = false;
      RCLCPP_INFO(get_logger(),
        "[10-10 W1] %s topic full 이 연달아 온다 (always_send_full 로 보임) — 시작 get_costmap resync 생략", s.name.c_str());
    } else if (s.topic_fulls_applied == 1) {
      fire = secs(now - s.first_topic_full_at) >= s.start_grace_sec;
    } else {
      fire = secs(now - s.node_start_steady) >= 4.0 * s.start_grace_sec;
    }
    if (fire) {
      s.start_resync_pending = false;
      s.sched.trigger();
    }
  }

  if (!s.sched.needed() && !s.sched.inFlight() && s.period_sec > 0.0) {
    const auto st = s.mirror->stats();
    if (st.has_grid && secs(now - st.last_full_time) >= s.period_sec) {
      s.sched.trigger();
    }
  }

  // [10-10 W1-3] W2PV-3: global 이 먼저다 — global 요청이 진행 중이거나 (service 가 있는데) 보낼 차례면 mask 는 기다린다.
  //   두 service 는 Nav2 global costmap 노드의 같은 executor 에서 차례로 처리되므로 동시에 보내도 빨라지지 않고, 200 MB 응답
  //   두 개가 겹쳐 timeout 만 늘린다.
  bool blocked_by_other = false;
  if (&s == &mask_sync_ && global_sync_.client) {
    blocked_by_other = global_sync_.sched.inFlight() ||
      (global_sync_.sched.needed() && global_sync_.client->service_is_ready());
  }
  const auto tick = s.sched.tick(now, s.client->service_is_ready(), blocked_by_other);
  if (tick == ResyncScheduler::Tick::kSlow) {
    // [10-10 W1-3][리뷰] R3: timeout 은 경고만 한다. 요청을 버리지 않는다 — 서버는 버려진 요청도 끝까지 200 MB 를 만들고,
    //   응답 시간이 늘 timeout 을 넘으면 버리고 다시 보내는 방식으로는 영원히 못 받는다. 늦은 응답도 적용한다
    //   (mirror 의 stale·superseded·clock reset 규칙이 옛 스냅샷을 거른다). 진행 중에는 새 요청을 보내지 않는다 (하나만).
    const auto & c = s.sched.counters();
    RCLCPP_WARN(get_logger(),
      "[10-10 W1-3][리뷰] %s get_costmap 응답이 %.0f s 넘게 없다 — 요청은 그대로 두고 기다린다 (포기 %.0f s, 요청 %lu / "
      "늦음 %lu / 포기 %lu / 실패 %lu)",
      s.name.c_str(), s.timeout_sec, s.sched.config().abandon_sec,
      static_cast<unsigned long>(c.requests), static_cast<unsigned long>(c.timeouts),
      static_cast<unsigned long>(c.abandoned), static_cast<unsigned long>(c.failures));
    return;
  }
  if (tick == ResyncScheduler::Tick::kTimedOut) {
    s.client->remove_pending_request(s.resync_in_flight_id);
    const auto & c = s.sched.counters();
    // [10-10 W1-3] R3: 예전에는 같은 tick 에 바로 다시 요청했다 (서버는 버려진 요청도 끝까지 200 MB 를 만든다 → 혼잡을 키움).
    // [10-10 W1-3][리뷰] 이제 포기는 abandon_sec 뒤 또는 service 가 사라졌을 때만.
    RCLCPP_WARN(get_logger(),
      "[10-10 W1-3][리뷰] %s get_costmap 응답이 %.0f s 동안 없다%s — 요청을 버리고 %.0f s 뒤에 다시 (연속 실패 %d, 요청 %lu / "
      "늦음 %lu / 포기 %lu / 실패 %lu)",
      s.name.c_str(), std::chrono::duration<double>(now - s.sched.sentAt()).count(),
      s.client->service_is_ready() ? "" : " (service 사라짐)", s.sched.currentBackoffSec(),
      s.sched.consecutiveFailures(),
      static_cast<unsigned long>(c.requests), static_cast<unsigned long>(c.timeouts),
      static_cast<unsigned long>(c.abandoned), static_cast<unsigned long>(c.failures));
    return;
  }
  if (tick != ResyncScheduler::Tick::kSend) {
    return;
  }

  auto req = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
  s.resync_sent_full_gen = s.mirror->fullGeneration();   // [10-10 W1][리뷰] clock reset 판정용
  s.sched.onSent(now);
  MirrorSync * sp = &s;   // s 는 node 멤버라 응답 콜백보다 오래 산다
  const uint64_t req_gen = s.source_gen;   // [10-10 W1-4] 이 요청을 보낸 source 세대
  auto fr = s.client->async_send_request(
    req,
    [this, sp, req_gen](rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedFuture future) {
      MirrorSync & sync = *sp;
      // [10-10 W1-4] source 전환 전에 옛 service 로 보낸 요청의 응답이다 (전환 때 remove_pending_request 와 client 교체를 했지만
      //   이미 꺼내진 응답이 늦게 실행될 수 있다). scheduler 는 이미 새 source 로 초기화됐으므로 건드리지 않고 버린다.
      if (!isCurrentSourceGen(req_gen, sync.source_gen)) {
        ++sync.stale_source_callbacks;
        RCLCPP_WARN(get_logger(),
          "[10-10 W1-4] %s: source 전환 전 요청의 get_costmap 응답 — 버림 (세대 %lu → 지금 %lu, 버린 옛 콜백 누적 %lu)",
          sync.name.c_str(), static_cast<unsigned long>(req_gen), static_cast<unsigned long>(sync.source_gen),
          static_cast<unsigned long>(sync.stale_source_callbacks));
        return;
      }
      sync.sched.onResponseArrived();
      const auto rnow = std::chrono::steady_clock::now();
      std::shared_ptr<nav2_msgs::srv::GetCostmap::Response> resp;
      try {
        resp = future.get();
      } catch (const std::exception & e) {
        sync.sched.onResponseFailed(rnow);
        RCLCPP_WARN(get_logger(), "[10-10 W1] %s get_costmap 응답 실패: %s — %.0f s 뒤에 다시 (연속 실패 %d)",
          sync.name.c_str(), e.what(), sync.sched.currentBackoffSec(), sync.sched.consecutiveFailures());
        return;
      }
      if (!resp) {
        sync.sched.onResponseFailed(rnow);
        return;
      }
      const double took = std::chrono::duration<double>(rnow - sync.sched.sentAt()).count();
      // [10-10 W1][리뷰] 오래된 응답은 mirror 가 버린다. 요청 뒤 다른 full 이 없었는데 오래됐으면 clock reset 으로 적용.
      const auto out = sync.mirror->applyFull(resp->map, sync.resync_sent_full_gen);
      const auto res = out.result;
      // [10-10 W1-4][리뷰] 빈 map (0x0 / resolution 0) = 서버에 아직 데이터가 없다 (diff layer 첫 cycle 전 — Nav2 configure~activate
      //   사이, 복귀 직후·시작 resync 가 여기 걸린다 / snapshot 실패 / layer enabled:false). 예전에는 0x0 grid 로 적용돼 resync 완료로
      //   보였다. mirror 는 아무것도 바꾸지 않았다 (grid·baseline_pending 그대로). 실패로 세지 않고 (30 s backoff 없음) needed 를
      //   남겨 min_interval 뒤에 다시 묻는다. 그 사이 topic FULL (activation·새 구독자) 이 오면 그것이 grid 가 된다.
      if (res == CostmapMirror::FullResult::kRejectedEmpty) {
        sync.sched.onResponseNotReady();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
          "[10-10 W1-4][리뷰] %s get_costmap 응답이 빈 map (%ux%u res %.3f, %.2f s) — 서버에 아직 데이터가 없다 (diff layer 첫 cycle 전 / "
          "snapshot 실패 / enabled:false). 적용하지 않고 grid(%s) 그대로, %.0f s 뒤 다시 묻는다 (빈 응답 누적 %lu)",
          sync.name.c_str(), resp->map.metadata.size_x, resp->map.metadata.size_y,
          static_cast<double>(resp->map.metadata.resolution), took, sync.mirror->hasGrid() ? "있음" : "없음",
          sync.sched.config().min_interval_sec, static_cast<unsigned long>(sync.sched.counters().not_ready));
        return;
      }
      if (res == CostmapMirror::FullResult::kRejectedBadSize) {
        sync.sched.onResponseFailed(rnow);
        RCLCPP_WARN(get_logger(),
          "[10-10 W1] %s get_costmap 응답 크기 불일치: data %zu, %ux%u — %.0f s 뒤에 다시 요청",
          sync.name.c_str(), resp->map.data.size(), resp->map.metadata.size_x, resp->map.metadata.size_y,
          sync.sched.currentBackoffSec());
        return;
      }
      // 요청 뒤 새 트리거가 없었으면 needed 를 내린다 (진행 중 트리거는 받지 않는다).
      // [10-10 W1-3][리뷰] 진행 중에 내용 유실 트리거가 있었으면 needed 를 남겨 한 번 더 받는다.
      const bool loss_retrigger = sync.sched.dirtyDuringFlight();
      sync.sched.onResponseDefinitive();
      if (loss_retrigger) {
        RCLCPP_WARN(get_logger(),
          "[10-10 W1-3][리뷰] %s get_costmap 응답(%.2f s) 진행 중에 update 유실이 있었다 — 응답 스냅샷 뒤 것일 수 있어 한 번 더 받는다 "
          "(누적 %lu)", sync.name.c_str(), took, static_cast<unsigned long>(sync.sched.counters().loss_retriggers));
      }
      if (res == CostmapMirror::FullResult::kRejectedStale) {
        RCLCPP_INFO(get_logger(),
          "[10-10 W1] %s get_costmap 응답이 이미 받은 topic full 보다 오래돼 쓰지 않았다 (%.2f s)", sync.name.c_str(), took);
        return;
      }
      if (res == CostmapMirror::FullResult::kRejectedSuperseded) {   // [10-10 W1-2][리뷰]
        RCLCPP_INFO(get_logger(),
          "[10-10 W1-2][리뷰] %s get_costmap 응답(%.2f s)보다 새 update 를 이미 적용했다 — 응답은 쓰지 않음 (self-contained)",
          sync.name.c_str(), took);
        return;
      }
      onFullApplied(sync, out, "get_costmap");
      const auto st = sync.mirror->stats();
      const auto & c = sync.sched.counters();
      RCLCPP_INFO(get_logger(),
        "[10-10 W1] %s get_costmap resync 완료: %ux%u res %.3f, %.1f MB, %.2f s (full %lu, update %lu, "
        "dropped %lu, replay 누적 %lu) [10-10 W1-3] 요청 %lu / timeout %lu / 실패 %lu / 진행 중 무시한 트리거 %lu "
        "[리뷰] 늦은 응답 %lu%s / 포기 %lu / 진행 중 유실 %lu",
        sync.name.c_str(), st.sig.size_x, st.sig.size_y, st.sig.resolution,
        static_cast<double>(resp->map.data.size()) / (1024.0 * 1024.0), took,
        static_cast<unsigned long>(st.fulls), static_cast<unsigned long>(st.updates),
        static_cast<unsigned long>(st.dropped_updates), static_cast<unsigned long>(st.replayed_updates),
        static_cast<unsigned long>(c.requests), static_cast<unsigned long>(c.timeouts),
        static_cast<unsigned long>(c.failures), static_cast<unsigned long>(c.ignored_in_flight),
        static_cast<unsigned long>(c.late_responses), sync.sched.lastResponseLate() ? " (이번 것)" : "",
        static_cast<unsigned long>(c.abandoned), static_cast<unsigned long>(c.loss_in_flight));
    });
  s.resync_in_flight_id = fr.request_id;
}

// [10-10 W1-4] graph 의 publisher 수. 이름이 잘못돼 throw 하면 0 으로 본다 (WARN 한 번).
//   주의: count_publishers 는 launch 의 remap 을 적용하지 않는다 (path_validator.launch.py 의 '/global_costmap/costmap_raw' remap
//   기본값은 같은 이름이라 영향 없음. 그 인자를 바꿔 쓰면 fallback topic param 도 같이 바꿀 것).
int PathValidatorNode::countPublishersSafe(const std::string & topic)
{
  try {
    return static_cast<int>(this->count_publishers(topic));
  } catch (const std::exception & e) {
    RCLCPP_WARN_ONCE(get_logger(), "[10-10 W1-4] count_publishers(%s) 실패: %s — 0 으로 본다", topic.c_str(), e.what());
    return 0;
  }
}

// [10-10 W1-4] global source: (1) 지금 full topic 의 publisher 수 → watchdog liveness, (2) diff layer 가 없으면 stock 으로 fallback
//   (한 번), fallback 중 diff publisher 가 생기면 복귀 (한 번). 판정은 GlobalSourceSelector (mirror_sync_policy.hpp).
//   subs 그룹 timer 에서만 부른다 → 같은 그룹의 full/update/응답 콜백과 겹치지 않는다.
void PathValidatorNode::globalSourceTick()
{
  MirrorSync & s = global_sync_;
  if (!s.mirror) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  const bool on_primary = global_source_.active() == GlobalSourceSelector::Source::kPrimary;
  const int active_pubs = countPublishersSafe(s.topic);
  s.last_pub_count = active_pubs;
  if (active_pubs > 0) {
    s.pub_ever_seen = true;
    s.last_pub_seen = now;
  }
  if (!global_source_.enabled()) {
    return;
  }
  const int primary_pubs = on_primary ? active_pubs : countPublishersSafe(global_costmap_topic_);
  const int fallback_pubs = on_primary ? countPublishersSafe(global_costmap_fallback_topic_) : active_pubs;
  const double cond_sec = global_source_.conditionSec(now);
  const auto act = global_source_.tick(now, primary_pubs, fallback_pubs);
  if (act == GlobalSourceSelector::Action::kSwitchToFallback) {
    switchGlobalSource(true, std::max(cond_sec, global_costmap_fallback_sec_));
  } else if (act == GlobalSourceSelector::Action::kSwitchToPrimary) {
    switchGlobalSource(false, 0.0);
  }
}

// [10-10 W1-4] global mirror 의 source 를 바꾼다. 순서 (모두 subs 그룹 안, mirror writer·응답 콜백과 직렬):
//   1. 진행 중 get_costmap 은 옛 client 에서 지운다 (remove_pending_request). source_gen 을 올려 옛 구독·client 에서 이미 꺼내진
//      콜백·응답이 와도 버리게 한다. 그 뒤 옛 구독·client 를 놓는다.
//   2. mirror.beginNewSource(): grid 는 reader 용으로 남기고 stamp 기준·replay 만 비운다 (write_mtx_ 아래 — reader 는 안 막음).
//   3. MirrorSync 시작 상태 초기화 + scheduler.resetForNewSource() (진행 중 표시·backoff·간격 지움, needed=true).
//   4. 새 topic/service 로 구독·client 를 만든다 → 다음 resyncTick 이 service 가 준비되면 get_costmap 을 보낸다.
//   mask mirror 는 건드리지 않는다 (geometry 는 같은 global costmap 이다).
void PathValidatorNode::switchGlobalSource(bool to_fallback, double cond_sec)
{
  MirrorSync & s = global_sync_;
  const std::string old_topic = s.topic;
  const std::string old_service = s.service;
  const bool had_in_flight = s.sched.inFlight();
  if (had_in_flight && s.client && s.resync_in_flight_id >= 0) {
    s.client->remove_pending_request(s.resync_in_flight_id);
  }
  ++s.source_gen;
  s.full_sub.reset();
  s.update_sub.reset();
  s.client.reset();

  s.topic = to_fallback ? global_costmap_fallback_topic_ : global_costmap_topic_;
  s.service = to_fallback ? global_costmap_fallback_service_ : global_costmap_service_;
  s.event_driven = !to_fallback && global_costmap_event_driven_ &&
    global_costmap_topic_ != global_costmap_fallback_topic_;

  global_costmap_.beginNewSource();

  const auto now = std::chrono::steady_clock::now();
  s.resync_in_flight_id = -1;
  s.resync_sent_full_gen = CostmapMirror::kNoRequestGen;
  s.first_full_logged = false;          // 새 source 의 첫 full 을 다시 INFO 로 남긴다
  s.topic_fulls_received = 0;
  s.topic_fulls_applied = 0;
  s.first_topic_full_at = {};
  s.no_grid_update_seen = false;
  s.any_rx = false;
  s.last_rx = {};
  s.last_pub_count = -1;
  s.pub_ever_seen = false;
  s.last_pub_seen = {};
  s.node_start_steady = now;
  s.source_since = now;
  s.start_resync_pending = false;       // 시작 resync 대신 바로 resync (아래 needed)
  s.sched.resetForNewSource();

  createMirrorIo(s);

  const auto st = global_costmap_.stats();
  if (to_fallback) {
    RCLCPP_WARN(get_logger(),
      "[10-10 W1-4] global costmap source 전환 → stock fallback: '%s' 에 publisher 가 없고 '%s' 에는 있는 상태가 %.1f s "
      "이어졌다 (diff_publisher layer 가 없는 설정으로 보임). 구독·client 를 '%s' / '%s' 로 다시 만들고, mirror 는 grid(%s) 를 둔 채 "
      "stamp 기준을 비우고, get_costmap resync 를 요청한다%s. diff publisher 가 생기면 한 번 되돌아간다. watchdog 기준: 무갱신",
      old_topic.c_str(), s.topic.c_str(), cond_sec, s.topic.c_str(), s.service.c_str(),
      st.has_grid ? "있음" : "없음", had_in_flight ? " (진행 중이던 옛 요청은 버림)" : "");
  } else {
    RCLCPP_WARN(get_logger(),
      "[10-10 W1-4] global costmap source 복귀 → diff layer: '%s' 에 publisher 가 생겼다. 구독·client 를 '%s' / '%s' 로 다시 만들고, "
      "mirror 는 grid(%s) 를 둔 채 stamp 기준을 비우고, get_costmap resync 를 요청한다%s. 이 뒤로는 source 를 다시 바꾸지 않는다. "
      "watchdog 기준: %s",
      s.topic.c_str(), s.topic.c_str(), s.service.c_str(), st.has_grid ? "있음" : "없음",
      had_in_flight ? " (진행 중이던 stock 요청은 버림)" : "", s.event_driven ? "publisher liveness + heartbeat" : "무갱신");
  }
  resyncTick(s);
}

// [10-10 W1-3] R4: global costmap 무갱신 / grid 없음 / resync 연속 실패 감시. 예전에는 2 Hz full 이 0.5 s 안에 고쳐 주던
//   것이 이제는 update 를 잃거나 publisher 가 멈추면 조용히 틀린 채 남는다. global 은 footprint_clearing(현장 global
//   obstacle layer 하나) 때문에 매 publish 주기 update 가 오므로 침묵 자체가 이상 신호다. agent mask 에는 쓰지 않는다.
//   알림은 throttled ERROR 와 global_watchdog_flags_ 뿐이다 (새 topic 없음).
// [10-10 W1-4] diff layer source 는 값이 바뀐 셀만 보낸다 (footprint_clearing 의 0 덮어쓰기도 값이 같으면 보내지 않는다).
//   그래서 그 source 는 "full topic publisher 가 사라짐" (본 적 있으면 사라진 뒤 stale_sec, 본 적 없으면 source 시작 뒤
//   no_grid_sec) 과 get_costmap 연속 실패로도 본다.
// [10-10 W1-4][리뷰] 예전 주석은 "diff source 는 침묵이 정상" 이라 무갱신을 아예 보지 않았다. 틀렸다: layer 의 publisher thread 는
//   update 구독자 (이 PV) 가 있으면 sweep_tick_sec (1 s) 마다 sweep band 하나를 보낸다 (DiffEngine::pop → scheduleRepairs, costmap
//   cycle 과 무관). 그래서 global_costmap_heartbeat_sec (20 s) 동안 아무것도 못 받으면 무갱신으로 본다 — publisher 는 graph 에
//   있는데 publisher thread 가 아무것도 못 보내는 경우 (FULL 버퍼 할당 실패로 FULL token 이 큐 맨 앞에 남는 F6 등).
//   못 잡는 것: map update thread 가 멈춘 경우. sweep band 는 publisher thread 가 (멈춘) shadow 에서 node clock stamp 로 계속
//   만들므로 PV 쪽에서는 정상과 구별되지 않는다 (W1-3 stock 은 publish 가 map thread 에서 나가 그 경우도 무갱신으로 잡았다).
void PathValidatorNode::globalWatchdogTick()
{
  const auto now = std::chrono::steady_clock::now();
  auto secs = [](std::chrono::steady_clock::duration d) {return std::chrono::duration<double>(d).count();};
  GlobalWatchdogInput in;
  in.has_grid = global_costmap_.hasGrid();
  in.since_start_sec = secs(now - node_start_steady_);
  in.any_rx = global_sync_.any_rx;
  in.since_last_rx_sec = global_sync_.any_rx ? secs(now - global_sync_.last_rx) : 0.0;
  in.resync_consecutive_failures = global_sync_.sched.consecutiveFailures();
  in.stale_sec = costmap_stale_watchdog_sec_;
  // grid 없음: 늦게 붙은 False 는 get_costmap(bigmap 약 16 s) 으로 받으므로 시작 fallback(grace×4) + timeout 까지는 정상
  in.no_grid_sec = 4.0 * costmap_resync_start_grace_sec_ + costmap_resync_timeout_sec_;
  in.fail_threshold = 2;
  // [10-10 W1-4] diff layer source 는 값이 바뀔 때만 보내므로 침묵이 정상이다 → 무갱신 대신 full topic publisher 유무로 본다.
  //   stock source (fallback 이거나 primary 를 stock 으로 설정) 는 W1-3 그대로 무갱신으로 본다.
  in.liveness_mode = global_sync_.event_driven;
  in.publisher_present = global_sync_.last_pub_count > 0;
  in.publisher_ever_seen = global_sync_.pub_ever_seen;
  in.since_publisher_seen_sec = global_sync_.pub_ever_seen ? secs(now - global_sync_.last_pub_seen) : 0.0;
  in.since_source_sec = secs(now - global_sync_.source_since);
  in.heartbeat_sec = global_costmap_heartbeat_sec_;   // [10-10 W1-4][리뷰] liveness_mode 의 무갱신 한도
  const unsigned f = evaluateGlobalWatchdog(in);
  const unsigned prev = global_watchdog_flags_.exchange(f);
  if (f != kWatchdogOk) {
    const auto & c = global_sync_.sched.counters();
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000,
      "[10-10 W1-3] global costmap watchdog:%s%s%s%s (source %s, 기준 %s, 마지막 수신 %.1f s 전, publisher %d 개%s, 기준 %.1f s / "
      "무갱신 %.1f s, "
      "resync 연속 실패 %d, 요청 %lu / timeout %lu) — path_validator 의 static 판정이 옛 grid 로 돌고 있을 수 있다",
      (f & kWatchdogNoGrid) ? " grid 없음" : "", (f & kWatchdogSilent) ? " 무갱신" : "",
      (f & kWatchdogNoPublisher) ? " [10-10 W1-4] publisher 없음" : "",
      (f & kWatchdogResyncFailing) ? " resync 연속 실패" : "",
      global_sync_.topic.c_str(), in.liveness_mode ? "publisher liveness + heartbeat" : "무갱신",
      in.any_rx ? in.since_last_rx_sec : in.since_source_sec, global_sync_.last_pub_count,
      in.publisher_ever_seen ? "" : " (본 적 없음)", costmap_stale_watchdog_sec_,
      in.liveness_mode ? global_costmap_heartbeat_sec_ : costmap_stale_watchdog_sec_,
      in.resync_consecutive_failures, static_cast<unsigned long>(c.requests),
      static_cast<unsigned long>(c.timeouts));
  } else if (prev != kWatchdogOk) {
    RCLCPP_INFO(get_logger(), "[10-10 W1-3] global costmap watchdog 정상으로 돌아옴 (이전 flag 0x%x, source %s)", prev,
      global_sync_.topic.c_str());
  }
}

// [10-10 W1-3] W2PV-1 (2): agent mask 를 아는가. 모르면 판정은 whoCoversPoint 경로(compare_agent_mask=false 와 같은)로 하고
//   M-6 은 이웃이 덮는 셀을 건너뛰지 않는다. 판정 규칙은 decideAgentMaskMode (mirror_sync_policy.hpp).
//   lock: agents_mutex_ 만 잠깐 쥔다 (global/mask guard 를 쥔 채 불러도 lock 순서에 맞다).
AgentMaskMode PathValidatorNode::agentMaskMode() const
{
  bool neighbours = false;
  {
    std::lock_guard<std::mutex> lk(agents_mutex_);
    for (const auto & kv : agent_cache_) {
      if (kv.first != self_machine_id_) {
        neighbours = true;
        break;
      }
    }
  }
  // compare_agent_mask=true 인데 agent_mask_topic 이 비어 mirror 가 없으면 grid 가 영원히 없다 → "모름" (whoCoversPoint).
  const bool use_mask = compare_agent_mask_;
  const auto mode = decideAgentMaskMode(use_mask, agent_mask_.hasGrid(), agent_mask_.contentKnown(), neighbours);
  const int prev = last_mask_mode_.exchange(static_cast<int>(mode));
  if (prev != static_cast<int>(mode) && prev != -1 && use_mask) {
    RCLCPP_INFO(get_logger(), "[10-10 W1-3] agent mask 상태: %s → %s (grid %s, 내용 근거 %s, 이웃 %s, 누적 fallback %lu)",
      prev == static_cast<int>(AgentMaskMode::kKnown) ? "앎" : "모름",
      mode == AgentMaskMode::kKnown ? "앎" : "모름",
      agent_mask_.hasGrid() ? "있음" : "없음", agent_mask_.contentKnown() ? "있음" : "없음",
      neighbours ? "있음" : "없음", static_cast<unsigned long>(mask_unknown_fallbacks_.load()));
  }
  return mode;
}

void PathValidatorNode::noteMaskUnknownFallback(const char * where)
{
  const uint64_t n = ++mask_unknown_fallbacks_;
  const auto st = agent_mask_.stats();
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
    "[10-10 W1-3] agent mask not ready (%s) — 이웃 귀속을 whoCoversPoint 로, M-6 은 이웃 셀을 건너뛰지 않음 "
    "(누적 %lu, grid %s, seed %lu, update %lu)",
    where, static_cast<unsigned long>(n), st.has_grid ? "있음" : "없음",
    static_cast<unsigned long>(st.seeds), static_cast<unsigned long>(st.updates));
}

// ===================== Agents handling =====================

void PathValidatorNode::agentsCallback(const robot_interfaces::msg::MultiAgentInfoArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(agents_mutex_);
  last_agents_ = msg;
  last_agents_stamp_ = msg->header.stamp;

  // [FIX] 통째로 들고 있지 않고 machine_id 별로 누적한다. (agent_cache_ 주석 참고)
  if (!msg->header.frame_id.empty() && msg->header.frame_id != global_frame_) {
    return;   // 좌표계가 다르면 받지 않는다 (예전에는 조회 시점에 걸렀다)
  }

  const rclcpp::Time now = this->now();
  for (const auto & a : msg->agents) {
    if (!a.header.frame_id.empty() && a.header.frame_id != global_frame_) continue;
    agent_cache_[a.machine_id] = a;
    agent_seen_at_[a.machine_id] = now;
  }

  // 소식이 끊긴 이웃은 버린다. 병합만 하면 떠난 로봇의 몇 분 전 자세/경로로
  // 계속 판단하게 되므로, 만료가 병합의 필수 짝이다.
  for (auto it = agent_seen_at_.begin(); it != agent_seen_at_.end(); ) {
    if ((now - it->second).seconds() > agent_cache_ttl_sec_) {
      RCLCPP_WARN(this->get_logger(),
        "[agents] agent %u 소식 끊김 (%.1fs). 캐시에서 제거한다.",
        static_cast<unsigned>(it->first), agent_cache_ttl_sec_);
      agent_cache_.erase(it->first);
      it = agent_seen_at_.erase(it);
    } else {
      ++it;
    }
  }
}

void PathValidatorNode::robotStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string & s = msg->data;
  // PLANNING, DRIVING, PAUSED 인 경우에만 충돌 검사를 활성화
  bool valid_state = (s == "PLANNING" || s == "DRIVING" || s == "PAUSED" || s == "READY" || s == "RECEIVED_GOAL" || s == "RECOVERY_FAILURE" || s == "RECOVERY_RUNNING" || s == "RECOVERY_SUCCESS");
  is_robot_in_driving_state_.store(valid_state);
  
  is_robot_in_ready_state_.store(s == "READY");
  if (s != "READY") {                     // [V2.31] READY 를 벗어나면 막힘 기록을 지운다
    ready_blocked_since_ns_.store(0);
    ready_escape_logged_.store(false);
  }

  // [09-29 D1-ⓒ] RECOVERY_* 로 들어가는 순간 저장해 둔 경로를 버린다.
  //   BT 가 recovery 에 들어가면 주행 루프는 /plan_truncated_short 를 더 내지 않는데, 예전에는 RECOVERY_* 를 활성 상태로 보아
  //   pause 직전의 옛 경로를 계속 검사했다. 그 HIT 가 fleet static pause 를 붙들고, fleet pause 는 BT recovery maneuver
  //   (새 짧은 경로를 같은 토픽으로 낸다)를 "Recovery held by fleet" 로 붙잡는 순환이 생겼다
  //   (sim L5m_grid r3 09-29 06:26~06:29: 교차로 안쪽 벽 모서리 HIT 179회 → 10/10 관제 보고).
  //   버린 뒤에는 recovery maneuver 가 내는 새 경로를 다시 검사한다 (막히면 그때 다시 HIT).
  {
    auto is_recovery = [](const std::string & x) { return x.rfind("RECOVERY_", 0) == 0; };
    if (is_recovery(s) && !is_recovery(last_robot_status_)) {
      bool had_path = false;
      {
        std::lock_guard<std::mutex> lock(path_mutex_);
        had_path = !latest_global_path_.empty();
        latest_global_path_.clear();
      }
      if (had_path) {
        RCLCPP_INFO(this->get_logger(),
          "[Validator] %s -> %s: recovery 진입, 저장된 옛 경로를 버린다 (recovery 경로가 오면 다시 검사)",
          last_robot_status_.c_str(), s.c_str());
      }
    }
    last_robot_status_ = s;
  }

// 다른 상태(IDLE, CHARGING 등 미션 완전히 종료/대기)인 경우 데이터 초기화
  if (!valid_state) {
    {
      std::lock_guard<std::mutex> lock(path_mutex_);
      latest_global_path_.clear();
    }
    // 다음 미션을 위해 Goal 정보와 Lock-on 깃발도 깔끔하게 비워줍니다.
    {
      std::lock_guard<std::mutex> lock(goals_mutex_);
      current_remaining_goals_.clear();
      goal_occupied_flag_ = false; 
    }
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
  const double phase1_buffer_m = 0.001;  //0.1;   
  
  // [Phase 2] 정밀 검사 시 '충돌(Hit)'로 판정할 코스트 (완전한 물리적 충돌: 254 고정)
  const unsigned char phase2_thr = 254; 

  // [Phase 2] 충돌 예상 지점에서 뒤로 몇 미터 후퇴하여 정밀 검사를 시작할 것인가?
  const double backtrack_m = 0.55;      

  // [Phase 2] '모서리 스침'이 아닌 '경로가 꽉 막힘'으로 간주할 장애물의 물리적 면적 (m^2)
  const double heavy_block_area_m2 = 1.0; //0.0125; 

  // [NEW: Phase 2] 로봇 다각형(Footprint)을 물리적으로 확장할 여유 패딩 (m)
  // 예: 0.02 ~ 0.05를 주면 실제 로봇보다 해당 수치만큼 부풀려진 크기로 충돌을 검사합니다.
  const double phase2_footprint_padding_m = 0.01; // 0.0001; //0.02; 

  // =========================================================

  // [10-10 W1] TF 조회(최대 0.2 s 대기)를 먼저 하고, global grid 는 ReadGuard 로 이 함수 끝까지 쥔다
  //   (예전: shared_ptr 만 복사해 lock 밖에서 읽음 → costmapCallback 의 memcpy 와 겹치는 torn read).
  if (!global_costmap_.hasGrid()) return;

  geometry_msgs::msg::Pose cur_pose;
  if (!getCurrentPoseFromTF(cur_pose)) return;

  const auto costmap_guard = global_costmap_.read();
  if (!costmap_guard) return;
  const nav2_costmap_2d::Costmap2D * costmap = costmap_guard.get();

  // [10-10 W1-2] agent mask 는 처음 필요할 때 ReadGuard 를 잡아 이 함수 끝까지 쥔다 (lock 순서 global → mask,
  //   한 판정 안에서는 같은 mask 스냅샷을 본다). Phase 1 에서 막힌 곳이 없으면 mask lock 은 잡지 않는다.
  //   예전: agentCellBlockedNear 가 부를 때마다 agent_mask_mutex_ 아래에서 shared_ptr 만 복사해 lock 밖에서 읽었다
  //   (agentMaskCallback 의 memcpy 와 겹치는 torn read).
  // [10-10 W1-3] W2PV-1 (2): mask 를 모르면 (grid 없음, 또는 이웃이 있는데 내용 근거 없는 0 grid) mask 를 보지 않고
  //   whoCoversPoint 경로(compare_agent_mask=false 와 같은)로 귀속하고, M-6 은 이웃이 덮는 셀을 건너뛰지 않는다.
  //   mask 를 알면 amhs 와 같은 mask 경로다.
  const AgentMaskMode mask_mode = agentMaskMode();
  bool mask_fallback_used = false;
  std::optional<CostmapMirror::ReadGuard> mask_guard;
  auto mask_cm = [&]() -> const nav2_costmap_2d::Costmap2D * {
      if (mask_mode != AgentMaskMode::kKnown) {return nullptr;}   // [10-10 W1-3] (kDisabled = compare_agent_mask false)
      if (!mask_guard) {mask_guard.emplace(agent_mask_.read());}
      return mask_guard->get();
    };

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

  // [V2 M-6] 현재 자세의 자기 발자국(+여유) 다각형. 이 안의 lethal 셀은 무시한다.
  std::vector<geometry_msgs::msg::Point> self_poly;
  if (ignore_self_fp_hit_ && !footprint_.empty()) {
    double scx = 0.0, scy = 0.0;
    for (const auto & p : footprint_) { scx += p.x; scy += p.y; }
    scx /= static_cast<double>(footprint_.size());
    scy /= static_cast<double>(footprint_.size());
    const double cyaw = tf2::getYaw(cur_pose.orientation);
    const double cc = std::cos(cyaw), cs = std::sin(cyaw);
    for (const auto & p : footprint_) {
      double vx = p.x - scx, vy = p.y - scy;
      double n = std::hypot(vx, vy); if (n < 1e-6) n = 1.0;
      const double lx = p.x + self_fp_margin_m_ * (vx / n);
      const double ly = p.y + self_fp_margin_m_ * (vy / n);
      geometry_msgs::msg::Point q;
      q.x = cur_pose.position.x + cc * lx - cs * ly;
      q.y = cur_pose.position.y + cs * lx + cc * ly;
      q.z = 0.0;
      self_poly.push_back(q);
    }
  }
  size_t self_fp_skipped = 0;

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
          if (!self_poly.empty() && pointInPolygon(self_poly, wx, wy)) {
            // [V2 M-6] 내 발자국 안의 **정적** lethal 만 무시한다. agent 마스크가 칠한 셀(상대 몸체가 내
            // 발자국과 겹칠 만큼 가까움) 은 그대로 HIT 로 둔다 — S8 3차에서 대면 중 agent HIT 가 사라져
            // fleet_decision 이 "비었다" 고 재개 → 컨트롤러 충돌 → recovery 111회.
            // [10-10 W1-3] mask 를 모르면 whoCoversPoint 가 이웃이 덮는다고 하는 셀은 건너뛰지 않는다 (skipSelfFootprintCell).
            bool mask_mark = false;
            bool neighbour_cover = false;
            if (mask_mode == AgentMaskMode::kKnown) {
              mask_mark = agentCellBlockedNear(*costmap, mask_cm(), umx, umy,   // [10-10 W1] [10-10 W1-2]
                  static_cast<unsigned char>(agent_cost_threshold_), 1);
            } else if (mask_mode == AgentMaskMode::kUnknown) {
              neighbour_cover = !whoCoversPoint(wx, wy).empty();
              mask_fallback_used = true;
            }
            if (skipSelfFootprintCell(mask_mode, true, mask_mark, neighbour_cover)) {
              ++self_fp_skipped;
              continue;
            }
          }
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
        
        // 1. 일반 장애물(Static) 기준 Goal 점유 확인
        bool is_static_goal_occ = isGoalBlocked(costmap, target_goal, goal_doorstep_static_m_, 253);
        
        // 2. [NEW] 에이전트(Agent) 기준 Goal 점유 확인
        bool is_agent_goal_occ = false;
        if (useMaskForGoalCheck(mask_mode)) {   // [10-10 W1-3] mask 를 모르면 보지 않는다 (정적 검사가 master 의 이웃 셀도 본다)
          const nav2_costmap_2d::Costmap2D * agent_cm = mask_cm();   // [10-10 W1-2] mask ReadGuard (global 다음)
          if (agent_cm) {
            is_agent_goal_occ = isGoalBlocked(agent_cm, target_goal, goal_doorstep_agent_m_, static_cast<unsigned char>(agent_cost_threshold_));   // [10-10 W1] .get()
          }
        }

        // 3. 둘 중 하나라도 걸렸거나, 거리 상으로 입구가 막혔다면 Goal Occupied!
        if (is_static_goal_occ || is_agent_goal_occ) {
          is_goal_occupied = true;
        } else {
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


        if (attributeWithMask(mask_mode)) {   // [10-10 W1-3] mask 를 알 때만 mask 경로 (amhs 와 같음)
          bool agent_mark = agentCellBlockedNear(*costmap, mask_cm(),   // [10-10 W1] [10-10 W1-2]
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
          // compare_agent_mask=false 경로. [10-10 W1-3] mask 를 모를 때도 이 경로 (W2PV-1: 예전에는 mask 가 없으면 STATIC 이었다).
          if (mask_mode == AgentMaskMode::kUnknown) {
            noteMaskUnknownFallback("Phase 2 HIT 귀속");
            mask_fallback_used = false;   // 이미 셌다
          }
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
  if (mask_fallback_used) {
    noteMaskUnknownFallback("M-6 자기 발자국 셀");   // [10-10 W1-3]
  }
  if (self_fp_skipped > 0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[V2 M-6] 자기 발자국 안의 lethal 셀 %zu 개를 무시했다 (벽 붙음/로컬라이제이션 오차). 경로는 안전으로 본다.",
      self_fp_skipped);
  }
  publishSafeStatus();
}




bool PathValidatorNode::isGoalBlocked(
    const nav2_costmap_2d::Costmap2D * cm,   // [10-10 W1] global grid 면 호출자가 ReadGuard 를 쥐고 있어야 한다
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





// ===================== Agent mask checking =====================

std::vector<PathValidatorNode::AgentHit> PathValidatorNode::findNearestAgent(
    double wx, double wy, double max_allowed_dist) const
{
  std::vector<AgentHit> out;
  std::lock_guard<std::mutex> lk(agents_mutex_);
  if (agent_cache_.empty()) return out;   // [FIX] 누적 캐시를 본다

  double min_dist_overall = 1e9;
  const robot_interfaces::msg::MultiAgentInfo* true_owner = nullptr;

  // [FIX] 맵을 직접 순회한다. 임시 벡터로 복사하면 true_owner 포인터가 매달린다.
  for (const auto & kv : agent_cache_) {
    const auto & a = kv.second;
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
    const nav2_costmap_2d::Costmap2D & master,   // [10-10 W1]
    const nav2_costmap_2d::Costmap2D * agent,    // [10-10 W1-2]
    unsigned int mx, unsigned int my,
    unsigned char thr, int manhattan_buf) const
{
  // 1) [10-10 W1] master(global grid)는 호출자가 쥔 ReadGuard 의 grid 를 받는다 (여기서 다시 lock 하지 않는다).
  //    [10-10 W1-2] agent mask 도 호출자가 쥔 mask ReadGuard 의 grid 를 받는다 (예전: 부를 때마다 shared_ptr 복사).
  if (!agent) return false;

  // 2) master index -> world
  double wx, wy;
  master.mapToWorld(mx, my, wx, wy);   // [10-10 W1]

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
  // [10-10 W1] TF 먼저, 그다음 ReadGuard 를 함수 끝까지 쥔다. obstacle_db_ 갱신도 guard 를 쥔 채 한다
  //   (lock 순서 grid → obstacle_db_): geometry 가 바뀌면 swap 뒤 onGlobalFullApplied 가 db 를 비우므로
  //   옛 grid 로 계산한 key 가 swap 뒤에 남지 않는다.
  if (!global_costmap_.hasGrid()) return;

  geometry_msgs::msg::Pose pose;
  if (!getCurrentPoseFromTF(pose)) return;

  const auto costmap_guard = global_costmap_.read();
  if (!costmap_guard) return;
  const nav2_costmap_2d::Costmap2D * costmap = costmap_guard.get();

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

bool PathValidatorNode::isBlockedCellKernel(   // [10-10 W1] master 인자 추가
    const nav2_costmap_2d::Costmap2D & master, unsigned int mx, unsigned int my) const
{
  // [10-10 W1] 호출자가 쥔 ReadGuard 의 grid 를 받는다.
  const int K = kernel_half_size_;
  const int sx = static_cast<int>(master.getSizeInCellsX());
  const int sy = static_cast<int>(master.getSizeInCellsY());

  for (int dx = -K; dx <= K; ++dx) {
    for (int dy = -K; dy <= K; ++dy) {
      const int x = static_cast<int>(mx) + dx;
      const int y = static_cast<int>(my) + dy;
      if (x < 0 || y < 0 || x >= sx || y >= sy) continue;

      const unsigned char c = master.getCost(static_cast<unsigned int>(x), static_cast<unsigned int>(y));   // [10-10 W1]
      if (ignore_unknown_ && c == nav2_costmap_2d::NO_INFORMATION) continue;
      if (c >= static_cast<unsigned char>(cost_threshold_)) return true;
    }
  }
  return false;
}

inline bool PathValidatorNode::masterCellBlocked(   // [10-10 W1] master 인자 추가
    const nav2_costmap_2d::Costmap2D & master, unsigned int mx, unsigned int my, unsigned char thr) const
{
  // [10-10 W1] 호출자가 쥔 ReadGuard 의 grid 를 받는다. (현재 호출처 없음)
  const unsigned char c = master.getCost(mx, my);
  if (ignore_unknown_ && c == nav2_costmap_2d::NO_INFORMATION) return false;
  return (c >= thr);
}




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
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
      "[Validator] Robot not in driving state. Publishing Safe Status.");
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
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
      "[Validator] No remaining goals. gpath empty? %s", gpath.empty() ? "true" : "false");
    if (gpath.empty()) publishSafeStatus();
    return; // 남은 Goal이 없으면 검사 스킵
  }




  // =========================================================
  // [NEW] READY 상태 전용 로직: 경로(gpath) 없이 '모든' 타겟 Goal 점유 여부 검사
  // =========================================================
  if (is_robot_in_ready_state_.load()) {
    // [10-10 W1-3][리뷰] global grid 가 없으면 목표 점유를 볼 수 없다 → SAFE 대신 출발 보류 (is_status_ready=false) 를 낸다.
    //   조용히 있지 않고 다시 낸다: navigation_manager 는 마지막 값을 쓰고, 그것이 이전 instance 의 SAFE 일 수 있다.
    if (readyHoldWithoutGlobalGrid(ready_hold_without_global_grid_, global_costmap_.hasGrid())) {
      publishReadyHoldNoGrid();
      return;
    }
    if (ready_hold_active_.exchange(false)) {
      RCLCPP_INFO(this->get_logger(),
        "[10-10 W1-3][리뷰] global grid 가 생겼다 — READY 출발 보류 해제, 목표 점유 검사 시작 (보류 낸 횟수 %lu)",
        static_cast<unsigned long>(ready_hold_no_grid_count_.load()));
    }
    geometry_msgs::msg::Pose cur_pose;
    if (!getCurrentPoseFromTF(cur_pose)) return;

    bool is_any_goal_occupied = false;
    bool is_agent = false;
    bool is_occupied_goal_last = false;
    geometry_msgs::msg::Pose occupied_goal_pose;
    std::vector<AgentHit> hits;

    // [10-10 W1] global grid 는 아래 목표 순회 동안만 ReadGuard 로 쥔다 (예전: shared_ptr 복사 후 lock 밖에서 읽음).

    // [10-10 W1-2] agent mask 는 아래에서 global guard 다음에 ReadGuard 로 쥔다 (예전: 여기서 shared_ptr 복사).

    // [핵심] 모든 Remaining Goals를 순회하며 장애물이 있는지 검사
    std::vector<geometry_msgs::msg::PoseStamped> goals_snapshot;
    {
      std::lock_guard<std::mutex> lock(goals_mutex_);
      goals_snapshot = current_remaining_goals_;
    }

    // [V2.31] 막힌 지 ready_escape_after_sec 가 지났으면 탈출 상한으로 좁힌다
    double horizon_m = ready_goal_horizon_m_;
    const int64_t blocked_since = ready_blocked_since_ns_.load();
    if (ready_escape_after_sec_ > 0.0 && blocked_since != 0) {
      const double blocked_sec = (this->now().nanoseconds() - blocked_since) / 1e9;
      if (blocked_sec >= ready_escape_after_sec_ &&
          (horizon_m <= 0.0 || ready_escape_horizon_m_ < horizon_m)) {
        horizon_m = ready_escape_horizon_m_;
        if (!ready_escape_logged_.exchange(true)) {
          RCLCPP_WARN(this->get_logger(),
            "[V2.31 READY escape] %.0fs 동안 출발 못 함 — 노선 %.1fm 안의 목표만 보고 출발 판단",
            blocked_sec, horizon_m);
        }
      }
    }

    // [10-10 W1-3] mask 를 모르면 mask 로 이웃 점유를 보지 않고, 정적 점유를 whoCoversPoint/findNearestAgent 로 다시 본다.
    const AgentMaskMode mask_mode = agentMaskMode();
    auto cm_guard = global_costmap_.read();         // [10-10 W1] grid 없으면 nullptr → static 검사는 false (예전과 같음)
    const nav2_costmap_2d::Costmap2D * cm = cm_guard.get();
    // [10-10 W1-2] lock 순서 global → mask. mask 가 없으면 nullptr → agent 검사는 false (예전과 같음)
    auto mask_guard = agent_mask_.read();
    const nav2_costmap_2d::Costmap2D * agent_cm = useMaskForGoalCheck(mask_mode) ? mask_guard.get() : nullptr;
    double route_acc = 0.0;                         // [V2.30] 로봇 → g0 → g1 … 누적 노선 거리
    double prev_x = cur_pose.position.x, prev_y = cur_pose.position.y;
    for (size_t i = 0; i < goals_snapshot.size(); ++i) {
      const auto& current_check_goal = goals_snapshot[i].pose;
      route_acc += std::hypot(current_check_goal.position.x - prev_x,
                              current_check_goal.position.y - prev_y);
      prev_x = current_check_goal.position.x; prev_y = current_check_goal.position.y;
      if (horizon_m > 0.0 && route_acc > horizon_m) {
        break;   // [V2.30] 노선을 따라 이만큼 먼 목표는 출발 판단에서 뺀다 (가까워지면 주행 중 검사가 본다)
      }
      
      // 로봇과 현재 검사 중인 Goal 사이의 거리
      double dist_robot_to_goal = std::hypot(
          current_check_goal.position.x - cur_pose.position.x,
          current_check_goal.position.y - cur_pose.position.y);

      // 0.25m 이내면 내 발밑이므로 안전(스킵)
      if (dist_robot_to_goal <= 0.25) continue;

      // Static & Agent 마스크 독립 검사
      bool is_static_blocked = isGoalBlocked(cm, current_check_goal, goal_doorstep_static_m_, 253);
      bool is_agent_blocked  = agent_cm ? isGoalBlocked(agent_cm, current_check_goal, goal_doorstep_agent_m_, static_cast<unsigned char>(agent_cost_threshold_)) : false;   // [10-10 W1-2] mask guard

      if (is_static_blocked || is_agent_blocked) {
        is_any_goal_occupied = true;
        occupied_goal_pose = current_check_goal;
        is_occupied_goal_last = (i == goals_snapshot.size() - 1); // 지금 막힌 Goal이 마지막 Goal인가?

        if (is_agent_blocked) {
          hits = findNearestAgent(current_check_goal.position.x, current_check_goal.position.y, 0.524);
          if (!hits.empty()) is_agent = true;
        } else if (fallbackGoalAttribution(mask_mode, is_static_blocked)) {
          // [10-10 W1-3] W2PV-1 (b): mask 를 몰라 "정적 점유" 로만 보이는 목표를 이웃 쪽으로 다시 본다.
          hits = whoCoversPoint(current_check_goal.position.x, current_check_goal.position.y);
          if (hits.empty()) {
            hits = findNearestAgent(current_check_goal.position.x, current_check_goal.position.y, 0.524);
          }
          if (!hits.empty()) is_agent = true;
          noteMaskUnknownFallback("READY 목표 점유");
        }
        
        // 막힌 곳을 하나라도 발견하면 즉시 순회 중단
        break; 
      }
    }
    mask_guard.reset();   // [10-10 W1-2]
    agent_cm = nullptr;
    cm_guard.reset();   // [10-10 W1] 발행 전에 grid lock 을 놓는다
    cm = nullptr;

    // [V2.31] 처음 막힌 시각을 남긴다. 풀려도 READY 를 벗어날 때까지 지우지 않는다
    // (지우면 다음 주기에 전체 검사로 돌아가 출발 조건이 깜빡인다).
    if (is_any_goal_occupied && ready_blocked_since_ns_.load() == 0) {
      ready_blocked_since_ns_.store(this->now().nanoseconds());
    }

    // 3. 검사 결과 퍼블리시
    if (is_any_goal_occupied) {
      if (is_agent) {
        publishAgentCollisionList(hits, true, is_occupied_goal_last, occupied_goal_pose);
      } else {
        triggerReplan("READY state: One of the target goals is occupied", true, is_occupied_goal_last, 
                      occupied_goal_pose.position.x, occupied_goal_pose.position.y, occupied_goal_pose);
      }
    } else {
      publishSafeStatus();
    }
    
    return; // READY 상태이므로 여기서 조기 종료!
  }






  // =========================================================
  // [Phase 1.5] 타겟 락온(Target Lock-on) 유지 방어!
  // =========================================================
  if (goal_occupied_flag_.load()) {
    double dist_moved = std::hypot(
        target_goal.position.x - locked_goal_pose_.position.x,
        target_goal.position.y - locked_goal_pose_.position.y);
        
    // if (dist_moved > 0.2) {
    if (dist_moved > 0.15) {
      RCLCPP_INFO(this->get_logger(), 
        "[Validator][Phase 1.5] Target goal changed (Moved %.2fm). Releasing Lock-on.", dist_moved);
      goal_occupied_flag_ = false; // Goal이 변경됨 -> 락온 해제
    } else if (!global_costmap_.hasGrid()) {
      // [10-10 W1-3][리뷰] global grid 가 없으면 정적 점유를 볼 수 없다. "CLEAR" 로 lock 을 풀면 아래 Phase 2 가 (경로가 비었을 때)
      //   SAFE 를 낸다. lock 을 유지하고 아무것도 내지 않는다 (마지막으로 낸 것은 이 instance 의 "점유" 다).
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "[10-10 W1-3][리뷰][Phase 1.5] global grid 없음 — locked goal 판정 보류 (lock 유지)");
      return;
    } else {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
        "[Validator][Phase 1.5] Lock-on active. Checking locked goal pose...");

      // [10-10 W1-2] agent mask 는 아래에서 static 검사 뒤에 따로 쥔다 (두 guard 를 겹쳐 쥐지 않음).


      // [NEW] Static과 Agent 마스크를 각각 검사
      // [10-10 W1] global grid 는 static 검사 동안만 ReadGuard 로 쥔다.
      bool is_static_blocked = false;
      {
        const auto cm_guard = global_costmap_.read();
        is_static_blocked = isGoalBlocked(cm_guard.get(), locked_goal_pose_, goal_doorstep_static_m_, 253);
      }
      bool is_agent_blocked = false;
      const AgentMaskMode mask_mode = agentMaskMode();   // [10-10 W1-3]
      if (useMaskForGoalCheck(mask_mode)) {   // [10-10 W1-2] mask ReadGuard 는 이 검사 동안만
        const auto mask_guard = agent_mask_.read();
        is_agent_blocked = isGoalBlocked(mask_guard.get(), locked_goal_pose_, goal_doorstep_agent_m_, static_cast<unsigned char>(agent_cost_threshold_));
      }
      
      // 둘 중 하나라도 막혀있다면?
      if (is_static_blocked || is_agent_blocked) {
        
        // [10-10 W1] 예전 `cm->worldToMap(..., mx, my)` 는 결과를 아래 주석 처리된 fallback 에서만 썼고,
        //   global costmap 없이 agent mask 만 있을 때 null 역참조였다. 지웠다.
        bool is_agent = false;
        std::vector<AgentHit> hits;
        
        // Agent Mask가 확실히 켜져있다면 최우선으로 Agent 정보 추출
        if (is_agent_blocked) {
           hits = findNearestAgent(locked_goal_pose_.position.x, locked_goal_pose_.position.y, 0.524);
           if (!hits.empty()) is_agent = true;
        } else if (fallbackGoalAttribution(mask_mode, is_static_blocked)) {
           // [10-10 W1-3] mask 를 몰라 정적 점유로만 보이는 목표를 이웃 쪽으로 다시 본다 (READY 와 같음)
           hits = whoCoversPoint(locked_goal_pose_.position.x, locked_goal_pose_.position.y);
           if (hits.empty()) {
             hits = findNearestAgent(locked_goal_pose_.position.x, locked_goal_pose_.position.y, 0.524);
           }
           if (!hits.empty()) is_agent = true;
           noteMaskUnknownFallback("Phase 1.5 목표 점유");
        }
        
        // // 혹시 Agent Mask는 안 걸렸는데, 일반 Costmap 상에 에이전트가 겹친 경우 (Fallback)
        // if (!is_agent && is_static_blocked) {
        //   if (compare_agent_mask_) {
        //     if (agentCellBlockedNear(mx, my, static_cast<unsigned char>(agent_cost_threshold_), agent_mask_manhattan_buffer_)) {
        //       hits = findNearestAgent(locked_goal_pose_.position.x, locked_goal_pose_.position.y, 0.524);
        //       if (!hits.empty()) is_agent = true;
        //     }
        //   } else {
        //     hits = whoCoversPoint(locked_goal_pose_.position.x, locked_goal_pose_.position.y);
        //     if (!hits.empty()) is_agent = true;
        //   }
        // }



        if (is_agent) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
              "[Validator][Phase 1.5] Locked Goal is still occupied by an AGENT.");
            publishAgentCollisionList(hits, true, is_last_goal, locked_goal_pose_);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
              "[Validator][Phase 1.5] Locked Goal is still occupied by STATIC obstacle.");
            triggerReplan("Locked Goal still occupied", true, is_last_goal, locked_goal_pose_.position.x, locked_goal_pose_.position.y, locked_goal_pose_);        
        }
        return; // 여기서 리턴해야 밑의 gpath.empty() 조기 종료 로직에 빠지지 않음!
      } else {
        RCLCPP_INFO(this->get_logger(), 
          "[Validator][Phase 1.5] Locked Goal is now CLEAR! Releasing Lock-on.");
        goal_occupied_flag_ = false; // 장애물 치워짐
      }
    }
  }

  // =========================================================
  // [Phase 2] Global Path 검사 (최초 탐지 역할)
  // =========================================================
  if (gpath.empty()) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
      "[Validator][Phase 2] Global path is empty and no Lock-on. Publishing Safe Status.");
    publishSafeStatus();
    return;
  }

  // 홀드 타임 체크
  const rclcpp::Time now = this->now();
  const double since_agent = (now - last_agent_block_time_).seconds();
  if (since_agent >= 0.0 && since_agent < agent_block_hold_sec_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
      "[Validator][Phase 2] Suppressed by Agent Hold Time (%.2f / %.2f)", since_agent, agent_block_hold_sec_);
    return;
  }

  // 검증 로직 실행 (하이브리드 검사 진행)
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
    "[Validator][Phase 2] Running path validation (Footprint check: %s, Is Last Goal: %s)", 
    use_footprint_check_ ? "true" : "false", is_last_goal ? "true" : "false");

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

  // [10-10 W1] obstacle_db_mutex_ 를 놓은 뒤 ReadGuard 를 잡아 이 함수 끝까지 쥔다.
  //   (예전: 루프 안에서 costmap_mutex_ 를 매번 잡았고 costmap_ 가 없으면 null 역참조였다.)
  const auto costmap_guard = global_costmap_.read();
  if (!costmap_guard) return;
  const nav2_costmap_2d::Costmap2D * master = costmap_guard.get();

  double acc = 0.0;
  for (size_t i = 0; i < gpath.size(); ++i) {
    if (i > 0) {
      const auto & p0 = gpath[i-1].pose.position;
      const auto & p1 = gpath[i].pose.position;
      acc += std::hypot(p1.x - p0.x, p1.y - p0.y);
      if (acc > max_check_dist) break;
    }

    unsigned int mx, my;
    if (!master->worldToMap(gpath[i].pose.position.x, gpath[i].pose.position.y, mx, my)) {   // [10-10 W1]
      streak = 0; continue;
    }

    const bool blocked_cell = isBlockedCellKernel(*master, mx, my);   // [10-10 W1]

    if (blocked_cell) {
      // --- [10-10 W1] ReadGuard 의 grid 를 그대로 쓴다 ---
      const nav2_costmap_2d::Costmap2D * cm = master;

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
      //    [10-10 W1-3] mask 를 알 때만 (모르면 위 whoCoversPoint 결과만 쓴다)
      if (attributeWithMask(agentMaskMode())) {
        bool agent_mark = false;
        {
          // [10-10 W1-2] global guard 를 쥔 채 mask guard 를 이 검사 동안만 잡는다 (순서 global → mask).
          const auto mask_guard = agent_mask_.read();
          agent_mark = agentCellBlockedNear(*master, mask_guard.get(),   // [10-10 W1] [10-10 W1-2]
              mx, my,
              static_cast<unsigned char>(agent_cost_threshold_),
              agent_mask_manhattan_buffer_);
        }
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
      master->mapToWorld(static_cast<int>(mx), static_cast<int>(my), hit_wx, hit_wy);   // [10-10 W1]
      
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
    // [10-08 W2 PV-PIP] 버그 수정: 예전 `std::max(1e-12, (yj - yi))` 는 내려가는 변 (yj < yi) 의 분모를 1e-12 로 바꿔
    //   교차점을 폭발시켰다 — yaw 가 축에서 조금만 (1e-10 rad 이상) 틀어져도 발자국 다각형이 앞쪽 띠가 되어 앞 장애물 셀을
    //   '내 발자국' 으로 버렸다 (M-6, 늦은 HIT·놓침). 앞 조건 ((yi > y) != (yj > y)) 이 yi != yj 를 보장하므로 그대로 나눈다.
    //   검증: verify/pv_pip_unit_1008/PIP_UNIT_RESULT.md, sim A/B op_w2_0[245]_pv[AB]·op_pvB_13 (CHANGE_STATUS 10-08).
    const bool hit = ((yi > y) != (yj > y)) &&
                     (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
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

bool PathValidatorNode::pathTubeCoversPoint(const robot_interfaces::msg::MultiAgentInfo & a,
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
  if (agent_cache_.empty()) return out;   // [FIX] 누적 캐시를 본다

  // 관제 링크 자체가 끊겼는지 확인 (마지막 수신 패킷 기준)
  if ((this->now() - last_agents_stamp_).nanoseconds() >
       static_cast<int64_t>(agents_freshness_timeout_ms_) * 1000000LL) {
    return out;
  }
  // 좌표계 검사는 agentsCallback 의 병합 시점으로 옮겼다.

  for (const auto & kv : agent_cache_) {
    const auto & a = kv.second;
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

  robot_interfaces::msg::PathAgentCollisionInfo msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = global_frame_;

  msg.is_goal_occupied = is_goal_occupied;
  msg.is_last_goal_occupied = is_last_goal_occupied;
  msg.target_goal = target_goal;

  msg.is_status_ready = is_robot_in_ready_state_.load();


  if (hits.empty()) {
    msg.note.push_back("non_collision");
    agent_collision_pub_->publish(msg);

    // [LOG 추가] 에이전트가 없어져서 Clear 상태를 쏠 때를 알려주는 INFO 로그 (2초에 한 번만 출력)
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[AgentCollision] Clear - No agents detected. Published 'non_collision'.");
    return;
  }




  // std::string agent_ids = "";


  std::ostringstream oss; // [코드 개선] std::string += 보다 성능과 포맷팅에 유리한 ostringstream 사용
  for (const auto & h : hits) {
    msg.machine_id.push_back(h.machine_id);
    msg.type_id.push_back(h.type_id);
    msg.x.push_back(h.x);
    msg.y.push_back(h.y);
    msg.ttc_first.push_back(h.ttc_first);
    msg.note.push_back(h.note);
    // agent_ids += std::to_string(h.machine_id) + "(" + h.type_id + ") ";    

    // [LOG 보강] ID뿐만 아니라 예상 충돌 시간(TTC)도 함께 기록하여 위험도 파악
    oss << h.machine_id << "(" << h.type_id << ", TTC:" << std::fixed << std::setprecision(1) << h.ttc_first << "s) ";
  }

  std::string agent_ids = oss.str();

  if (!hits.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "[AgentCollision] HIT! Agents: [%s] (Total: %zu) | GoalOcc: %s | LastGoalOcc: %s", 
      agent_ids.c_str(), 
      hits.size(),
      is_goal_occupied ? "TRUE" : "FALSE",
      is_last_goal_occupied ? "TRUE" : "FALSE");


    // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    //   "Publishing Agent Collision Info: [ %s] (Total: %zu hits)", 
    //   agent_ids.c_str(), hits.size());
  }

  agent_collision_pub_->publish(msg);


    // 배타적 퍼블리시: 에이전트가 확실하므로 일반(Static) 장애물 토픽은 False로 pub.
  if (static_collision_pub_) {
    robot_interfaces::msg::PathStaticCollisionInfo static_msg;
    static_msg.header.stamp = this->now();
    static_msg.header.frame_id = global_frame_;
    static_msg.replan_request = false;
    static_msg.is_goal_occupied = false;
    static_msg.is_last_goal_occupied = false;
    static_msg.hit_x = 0.0;
    static_msg.hit_y = 0.0;
    static_msg.target_goal = target_goal;
    static_collision_pub_->publish(static_msg);
  }
    
}

void PathValidatorNode::triggerReplan(const std::string & reason, bool is_goal_occupied, bool is_last_goal_occupied, double hit_x, double hit_y, const geometry_msgs::msg::Pose& target_goal)
{
  const rclcpp::Time now = this->now();

  const double since_agent = (now - last_agent_block_time_).seconds();
  if (since_agent >= 0.0 && since_agent < agent_block_hold_sec_) {
    // [LOG 추가] 에이전트가 방금 지나가서 생긴 잔상(Ghost)을 무시할 때 출력 (스팸 방지를 위해 DEBUG 사용)
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "[StaticCollision] Suppressed by Agent Hold Time (%.2fs < %.2fs)", since_agent, agent_block_hold_sec_);

    return;
  }

  // if ((now - last_replan_time_).seconds() <= cooldown_sec_) return;
  last_replan_time_ = now;

  robot_interfaces::msg::PathStaticCollisionInfo m;
  m.header.stamp = this->now();
  m.header.frame_id = global_frame_;
  m.replan_request = true;
  m.is_goal_occupied = is_goal_occupied;
  m.is_last_goal_occupied = is_last_goal_occupied;
  m.hit_x = hit_x;
  m.hit_y = hit_y;
  m.target_goal = target_goal;

  static_collision_pub_->publish(m);
  // RCLCPP_WARN(this->get_logger(), "Triggering replan: %s", reason.c_str());

// [LOG 보강] 단순 WARN을 THROTTLE로 변경해 터미널 도배를 막고, 충돌 좌표와 Goal 점유 여부를 상세히 출력
  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
    "[StaticCollision] HIT! Reason: %s | Pos: (%.2f, %.2f) | GoalOcc: %s | LastGoalOcc: %s", 
    reason.c_str(), hit_x, hit_y, 
    is_goal_occupied ? "TRUE" : "FALSE", 
    is_last_goal_occupied ? "TRUE" : "FALSE");


// 배타적 퍼블리시: 정적 장애물이 확실하므로 에이전트 토픽은 빈 배열(False)로 pub.
  // publishAgentCollisionList 내부에 hits가 empty()일 때 non_collision을 쏘는 로직 활용.
  publishAgentCollisionList({}, false, false, target_goal);

// [LOG 추가] 배타적 퍼블리시가 정상적으로 호출되었음을 확인하기 위한 로깅
  // RCLCPP_INFO(this->get_logger(), 
  //   "[StaticCollision] Mutually exclusive trigger: Sent agent_collision=CLEAR");

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "[StaticCollision] Mutually exclusive trigger: Sent agent_collision=CLEAR");    


  // if (publish_false_pulse_ && flag_pulse_ms_ > 0) {
  //   flag_reset_timer_.reset();
  //   auto weak_pub = std::weak_ptr<rclcpp::Publisher<robot_interfaces::msg::PathStaticCollisionInfo>>(static_collision_pub_);
  //   flag_reset_timer_ = this->create_wall_timer(
  //       std::chrono::milliseconds(flag_pulse_ms_),
  //       [weak_pub]() {
  //         if (auto pub = weak_pub.lock()) {
  //           robot_interfaces::msg::PathStaticCollisionInfo off;
  //           off.replan_request = false;
  //           off.is_goal_occupied = false;
  //           off.is_last_goal_occupied = false;
  //           pub->publish(off);
  //         }
  //       });
  // }
}

// [10-10 W1-3][리뷰] READY 인데 global grid 가 없다. static 쪽에 is_status_ready=false (나머지는 SAFE 와 같음) 를 내
//   navigation_manager 의 cond_static 을 false 로 만든다 (is_status_ready 를 읽는 곳은 navigation_manager 뿐). agent 쪽은
//   publishSafeStatus 와 같은 non_collision 이다 (fleet_decision 이 받는 내용은 예전과 같다).
void PathValidatorNode::publishReadyHoldNoGrid()
{
  const uint64_t n = ready_hold_no_grid_count_.fetch_add(1) + 1;
  ready_hold_active_.store(true);
  robot_interfaces::msg::PathStaticCollisionInfo m;
  m.header.stamp = this->now();
  m.header.frame_id = global_frame_;
  m.replan_request = false;
  m.is_goal_occupied = false;
  m.is_last_goal_occupied = false;
  m.hit_x = 0.0;
  m.hit_y = 0.0;
  m.is_status_ready = false;
  geometry_msgs::msg::Pose tgt_goal;
  {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    if (!current_remaining_goals_.empty()) {
      tgt_goal = current_remaining_goals_.front().pose;
    }
  }
  m.target_goal = tgt_goal;
  static_collision_pub_->publish(m);
  publishAgentCollisionList({}, false, false, tgt_goal);
  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
    "[10-10 W1-3][리뷰] global grid 없음 — READY 출발 보류 (is_status_ready=false, 누적 %lu). global get_costmap 응답을 기다린다 "
    "(요청·늦음·포기 수는 resync WARN/INFO 참고 — scheduler 는 subs 그룹 전용이라 여기서 읽지 않는다)",
    static_cast<unsigned long>(n));
}

// [NEW] 아무 장애물도, 에이전트도 없을 때 호출
void PathValidatorNode::publishSafeStatus()
{
robot_interfaces::msg::PathStaticCollisionInfo m;
  m.header.stamp = this->now();
  m.header.frame_id = global_frame_;
  m.replan_request = false;
  m.is_goal_occupied = false;
  m.is_last_goal_occupied = false;
  m.hit_x = 0.0;
  m.hit_y = 0.0;
  m.is_status_ready = is_robot_in_ready_state_.load();
    

  // false를 보낼 때도 항상 최신 Goal을 가져와서 채워넣음
  geometry_msgs::msg::Pose tgt_goal;
  {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    if (!current_remaining_goals_.empty()) {
      tgt_goal = current_remaining_goals_.front().pose;
    }
  }
  m.target_goal = tgt_goal;

  static_collision_pub_->publish(m);

  // 에이전트 쪽 클리어 할 때도 target_goal 전달
  publishAgentCollisionList({}, false, false, tgt_goal);
}



}  // namespace replan_monitor
