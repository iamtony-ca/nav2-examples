#ifndef PATH_VALIDATOR_NODE_HPP
#define PATH_VALIDATOR_NODE_HPP

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <chrono>
#include <algorithm>

// [NEW] Added for std::map
#include <map>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/msg/costmap_update.hpp"   // [10-10 W1]
#include "nav2_msgs/srv/get_costmap.hpp"      // [10-10 W1]
#include "replan_monitor/costmap_mirror.hpp"  // [10-10 W1]
#include "replan_monitor/mirror_sync_policy.hpp"  // [10-10 W1-3]

#include "robot_interfaces/msg/multi_agent_info_array.hpp"
#include "robot_interfaces/msg/multi_agent_info.hpp"
#include "robot_interfaces/msg/agent_status.hpp"
#include "robot_interfaces/msg/path_agent_collision_info.hpp"
#include "robot_interfaces/msg/path_static_collision_info.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace replan_monitor
{

struct ObstacleInfo
{
  rclcpp::Time first_seen;
  rclcpp::Time last_seen;
};

// [10-10 W1] CostmapSignature 는 replan_monitor/costmap_mirror.hpp 로 옮겼다.

class PathValidatorNode : public rclcpp::Node
{
public:
  PathValidatorNode();

private:
  // [10-10 W1-2] mirror 하나(full topic + <topic>_updates + get_costmap resync)의 구독·resync 상태.
  //   step 1 에서 global costmap 전용 멤버(resync_*, start_resync_pending_ 등)였던 것을 묶었다. global costmap 과
  //   agent mask 가 하나씩 쓰고, 동작은 step 1 의 global 과 같다. 모두 subs_callback_group_ (MutuallyExclusive)
  //   콜백(full / update / timer / get_costmap 응답 / MESSAGE_LOST event)에서만 만지므로 mutex 가 없다.
  struct MirrorSync
  {
    std::string name;                         // 로그용 ("global costmap" / "agent mask")
    CostmapMirror * mirror{nullptr};
    bool clears_obstacle_db{false};           // geometry 가 바뀌면 obstacle_db_ 를 비운다 (global 만)
    std::string topic;                        // full topic. update topic = topic + "_updates"
    std::string service;                      // nav2_msgs/srv/GetCostmap
    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr full_sub;
    rclcpp::Subscription<nav2_msgs::msg::CostmapUpdate>::SharedPtr update_sub;
    rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr client;
    // 파라미터
    bool resync_on_start{true};
    double period_sec{0.0};
    double min_interval_sec{5.0};
    double timeout_sec{30.0};
    double start_grace_sec{5.0};
    // [10-10 W1-2][리뷰] mirror 가 self_contained_updates 모드 (agent mask). 그러면 update 하나가 mask 전체라
    //   시작 resync(grid 가 아직 없을 때만 빼고)·MESSAGE_LOST resync 가 필요 없고, 오래된 latched full 은 geometry 만 쓴다.
    bool self_contained{false};
    // [10-10 W1-3] 요청 일정 (needed / in-flight / 간격 / timeout / backoff / 횟수). 예전 resync_needed·resync_trigger_gen·
    //   resync_sent_gen·resync_in_flight·resync_sent_at·resync_last_request·resync_ever_requested 를 대신한다.
    //   요청이 진행 중이면 트리거를 올리지 않고 (W2PV-3), timeout 뒤에는 backoff 를 둔다 (R3).
    ResyncScheduler sched;
    int64_t resync_in_flight_id{-1};
    bool first_full_logged{false};
    uint64_t resync_sent_full_gen{CostmapMirror::kNoRequestGen};   // 요청 때의 fullGeneration() (clock reset 판정)
    bool start_resync_pending{false};
    std::chrono::steady_clock::time_point node_start_steady{};
    std::chrono::steady_clock::time_point first_topic_full_at{};
    uint64_t topic_fulls_applied{0};
    // [10-10 W1-3]
    uint64_t topic_fulls_received{0};         // 받은 topic full 수 (적용 여부 무관) — W2PV-2 "구독 뒤 첫 full" 판정
    bool fresh_update_seen{false};            // 받아들인 update 가 있다 (latched 오래된 update 거르기 종료)
    uint64_t stale_latched_dropped{0};        // 버린 오래된 latched update 수 (W2PV-1)
    uint64_t old_fulls_dropped{0};            // 같은 geometry grid 가 있어 버린 오래된 topic full 수 (W2PV-2)
    bool no_grid_update_seen{false};          // grid 없이 update 가 왔다 (geometry fallback 판정)
    bool seed_from_global{false};             // mask 전용: global mirror geometry 로 0 grid seed (W2PV-1)
    bool any_rx{false};                       // full/update 를 하나라도 받았다 (watchdog)
    std::chrono::steady_clock::time_point last_rx{};
    uint64_t message_lost_reports{0};         // MESSAGE_LOST 보고 횟수 (S2-OPS-10)
    uint64_t message_lost_total{0};           // 그 보고들의 유실 누적
    // [10-10 W1-3][리뷰] 구독 시각 (node clock, ns). 0 = 아직 모름 (sim time 이 0). latched update 를 출처로 거른다.
    int64_t subscribe_ros_ns{0};
    // [10-10 W1-4] source 전환 (global 만). 구독·client 를 다시 만들 때마다 source_gen 을 올리고, 콜백·응답은 만들어질 때의
    //   세대를 들고 있다가 다르면 버린다 (전환 직전에 꺼내진 옛 source 메시지·늦은 get_costmap 응답).
    uint64_t source_gen{0};
    uint64_t stale_source_callbacks{0};       // 그렇게 버린 콜백 수
    void (PathValidatorNode::*full_cb)(const nav2_msgs::msg::Costmap::SharedPtr){nullptr};
    void (PathValidatorNode::*update_cb)(const nav2_msgs::msg::CostmapUpdate::ConstSharedPtr){nullptr};
    // [10-10 W1-4] watchdog: 지금 source 가 값이 바뀔 때만 보내는 diff layer 다 → 침묵 대신 publisher 유무로 본다.
    bool event_driven{false};
    std::chrono::steady_clock::time_point source_since{};   // 지금 source 로 구독을 만든 시각
    int last_pub_count{-1};                                  // 지금 full topic 의 publisher 수 (500 ms 마다, -1 = 아직 안 셈)
    bool pub_ever_seen{false};
    std::chrono::steady_clock::time_point last_pub_seen{};
  };

  // ========= Callbacks =========
  void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
  // [10-10 W1] global costmap update 병합 + get_costmap resync
  void costmapUpdateCallback(const nav2_msgs::msg::CostmapUpdate::ConstSharedPtr msg);
  void costmapResyncTimerCallback();
  // [10-10 W1-2] 아래 다섯은 step 1 의 global 전용 함수를 MirrorSync 를 받게 일반화한 것.
  void setupMirrorSync(MirrorSync & s, const rclcpp::SubscriptionOptions & subs_options,
                       void (PathValidatorNode::*full_cb)(const nav2_msgs::msg::Costmap::SharedPtr),
                       void (PathValidatorNode::*update_cb)(const nav2_msgs::msg::CostmapUpdate::ConstSharedPtr));
  void onTopicFull(MirrorSync & s, const nav2_msgs::msg::Costmap & msg);
  void onUpdate(MirrorSync & s, const nav2_msgs::msg::CostmapUpdate::ConstSharedPtr & msg);
  void requestResync(MirrorSync & s, const char * reason,
                     ResyncScheduler::Trigger kind = ResyncScheduler::Trigger::kStructural);   // [10-10 W1-3][리뷰] kind
  void resyncTick(MirrorSync & s);
  // [10-10 W1-4] 구독 (full·update + MESSAGE_LOST) 과 get_costmap client 를 s.topic / s.service 로 만든다 (setupMirrorSync 와
  //   source 전환이 같이 쓴다). 콜백은 s.source_gen 을 검사한다.
  void createMirrorIo(MirrorSync & s);
  // [10-10 W1-4] global source 선택 (diff layer → stock fallback / 복귀). 500 ms timer 에서 resyncTick(global) 전에 부른다.
  void globalSourceTick();
  void switchGlobalSource(bool to_fallback, double cond_sec);
  int countPublishersSafe(const std::string & topic);
  void onFullApplied(MirrorSync & s, const CostmapMirror::FullOutcome & out, const char * source);
  // [10-10 W1-3] mask 를 global mirror geometry 로 0 grid seed (W2PV-1). seed 했으면 true.
  bool trySeedMaskFromGlobal(const char * why);
  // [10-10 W1-3] global costmap watchdog (R4). 500 ms timer 에서 부른다.
  void globalWatchdogTick();
  // [10-10 W1-3] agent mask 를 아는가 (mirror 상태 + 이웃 유무). 판정 함수는 mirror_sync_policy.hpp.
  AgentMaskMode agentMaskMode() const;
  // [10-10 W1-3] mask 를 몰라서 whoCoversPoint 경로로 판정했다 — 횟수를 세고 throttled WARN.
  void noteMaskUnknownFallback(const char * where);
  // [10-10 W1-3][리뷰] READY 인데 global grid 가 없다 — SAFE 대신 is_status_ready=false 로 출발 보류를 낸다.
  void publishReadyHoldNoGrid();
  void agentMaskCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
  void agentMaskUpdateCallback(const nav2_msgs::msg::CostmapUpdate::ConstSharedPtr msg);   // [10-10 W1-2]
  void robotStatusCallback(const std_msgs::msg::String::SharedPtr msg);
  void validatePathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void updateObstacleDatabase();
  void agentsCallback(const robot_interfaces::msg::MultiAgentInfoArray::SharedPtr msg);

  void remainingGoalsCallback(const nav_msgs::msg::Path::SharedPtr msg);

  // ========= Helpers / Utils =========
  bool getCurrentPoseFromTF(geometry_msgs::msg::Pose & pose_out) const;
  bool transformToGlobal(const geometry_msgs::msg::PoseStamped & in,
                         geometry_msgs::msg::PoseStamped & out) const;
  void transformPathToGlobal(const nav_msgs::msg::Path & in,
                             std::vector<geometry_msgs::msg::PoseStamped> & out) const;

  inline uint64_t packKey(unsigned int mx, unsigned int my) const
  {
    return (static_cast<uint64_t>(mx) << 32) | static_cast<uint64_t>(my);
  }

  // [10-10 W1] global grid 를 읽는 helper 는 호출자가 쥔 ReadGuard 의 grid 를 인자로 받는다 (재귀 lock 금지).
  bool isBlockedCellKernel(const nav2_costmap_2d::Costmap2D & master, unsigned int mx, unsigned int my) const;

  // void validateWithFootprint(const std::vector<geometry_msgs::msg::PoseStamped> & gpath);
  void validateWithPoints(const std::vector<geometry_msgs::msg::PoseStamped> & gpath);
  void validatePathOptimized(const std::vector<geometry_msgs::msg::PoseStamped> & gpath, const geometry_msgs::msg::Pose& target_goal, bool is_last_goal); // [NEW] bool 인자 추가

  inline bool masterCellBlocked(const nav2_costmap_2d::Costmap2D & master,
                                unsigned int mx, unsigned int my, unsigned char thr) const;   // [10-10 W1]
  // [10-10 W1-2] agent mask 도 호출자가 쥔 mask ReadGuard 의 grid 를 받는다 (nullptr = mask 없음 → false).
  inline bool agentCellBlockedNear(const nav2_costmap_2d::Costmap2D & master,                 // [10-10 W1]
                                   const nav2_costmap_2d::Costmap2D * agent,                  // [10-10 W1-2]
                                   unsigned int mx, unsigned int my,
                                   unsigned char thr, int manhattan_buf) const;

  static bool pointInPolygon(const std::vector<geometry_msgs::msg::Point> & poly,
                             double x, double y);

// ========= (NEW) Timer & Path Storage =========
  void validationTimerCallback();
  
  rclcpp::TimerBase::SharedPtr validation_timer_;
  std::vector<geometry_msgs::msg::PoseStamped> latest_global_path_;
  mutable std::mutex path_mutex_;


  // === Agent 충돌 식별 ===
  struct AgentHit {
    uint16_t machine_id{0};
    std::string type_id;
    double x{0.0}, y{0.0};
    float ttc_first{-1.0f};
    std::string note;
  };

  // [NEW] Map to store footprint data from YAML
  struct AgentFootprintData
  {
    // existing code uses Point32, so we store Point32
    std::vector<geometry_msgs::msg::Point32> points;
    double radius{0.0};
    bool use_radius{true};
  };
  // Map from machine_id to its footprint/radius data
  std::map<uint16_t, AgentFootprintData> agent_footprints_;

  // [NEW] Helper to get footprint for a given agent
  std::vector<geometry_msgs::msg::Point32> 
  getFootprintForAgent(const robot_interfaces::msg::MultiAgentInfo & a) const;

  // [NEW] Helper to convert nav2_costmap_2d::makeFootprint... results
  static std::vector<geometry_msgs::msg::Point32> toPoint32(
      const std::vector<geometry_msgs::msg::Point>& points);



  // wx, wy를 커버하는 agent의 footprint 또는 truncated_path 튜브를 찾아 리턴
  std::vector<AgentHit> whoCoversPoint(double wx, double wy) const;

  // 내부: 경로 튜브(footprint를 얇게 확장)에서 포함 여부 검사
  static bool pathTubeCoversPoint(const robot_interfaces::msg::MultiAgentInfo & a,
                                  double wx, double wy,
                                  double stride_m, double dilate_m,
                                  int max_poses, double frame_yaw,
                                  const std::string & global_frame);

  static double headingTo(const geometry_msgs::msg::Pose & pose, double wx, double wy);
  static double speedAlong(const geometry_msgs::msg::Twist & tw, double heading_rad);

  void triggerReplan(const std::string & reason, bool is_goal_occupied, bool is_last_goal_occupied, double hit_x, double hit_y, const geometry_msgs::msg::Pose& target_goal); // [NEW] bool 인자 추가

  void publishAgentCollisionList(const std::vector<AgentHit> & hits, bool is_goal_occupied, bool is_last_goal_occupied, const geometry_msgs::msg::Pose& target_goal); // [NEW] bool 인자 추가
  void publishSafeStatus();


  std::vector<AgentHit> findNearestAgent(double wx, double wy, double max_allowed_dist) const;



  // [함수 시그니처 변경] 범용 검사기로 리팩토링
  // [10-10 W1] shared_ptr → raw const 포인터. global grid 는 호출자가 ReadGuard 를 쥔 채로 넘긴다.
  bool isGoalBlocked(
      const nav2_costmap_2d::Costmap2D * cm, 
      const geometry_msgs::msg::Pose& goal_pose, 
      double buffer_m, 
      unsigned char threshold) const;



  // ========= Callback Groups =========
  rclcpp::CallbackGroup::SharedPtr subs_callback_group_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

  // ========= ROS I/O =========
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pruned_path_sub_;
  // [10-10 W1-2] global costmap / agent mask 의 full·update 구독과 get_costmap client 는 MirrorSync 안에 있다.
  rclcpp::TimerBase::SharedPtr costmap_resync_timer_;                                   // [10-10 W1] (둘 다 tick)
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_status_sub_;
  rclcpp::Subscription<robot_interfaces::msg::MultiAgentInfoArray>::SharedPtr agents_sub_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr remaining_goals_sub_;

  rclcpp::Publisher<robot_interfaces::msg::PathStaticCollisionInfo>::SharedPtr static_collision_pub_;
  rclcpp::Publisher<robot_interfaces::msg::PathAgentCollisionInfo>::SharedPtr agent_collision_pub_;

  rclcpp::TimerBase::SharedPtr obstacle_db_update_timer_;
  rclcpp::TimerBase::SharedPtr flag_reset_timer_;

  // ========= State =========
  // [10-10 W1] global costmap = mirror (full + update 병합). 예전 costmap_ / costmap_mutex_ / last_costmap_sig_ 대체.
  // [10-10 W1-2] agent mask 도 같은 CostmapMirror (예전 agent_mask_ shared_ptr / agent_mask_mutex_ / last_agent_sig_ 대체).
  //   lock 순서 (한 방향만):
  //     global_costmap_ ReadGuard → agent_mask_ ReadGuard → {obstacle_db_mutex_, agents_mutex_, goals_mutex_, path_mutex_}
  //   - mask guard 를 쥔 채 global guard 를 잡지 않는다. 오른쪽 mutex 를 쥔 채 guard 를 잡지 않는다.
  //   - 한 thread 가 같은 mirror 의 guard 를 겹쳐 잡지 않는다 (helper 는 호출자의 grid 포인터를 인자로 받는다).
  //   - 두 guard 를 함께 쥐는 곳은 global grid 와 mask 를 한 루프에서 같이 보는 곳뿐이다
  //     (validatePathOptimized, READY 목표 순회, validateWithPoints 의 mask 보강 검사). Phase 1.5 는 따로 쥔다.
  //   - writer(full/update 콜백, get_costmap 응답)는 subs 그룹에서 한 번에 하나씩, 각자 자기 mirror 안에서만
  //     write → gate → grid(exclusive) 를 잡는다. 두 mirror 의 lock 을 함께 쥐는 writer 는 없다.
  //   obstacle_db_ 정리는 applyFull 이 끝난 뒤(grid lock 을 놓은 뒤) 한다.
  CostmapMirror global_costmap_;
  CostmapMirror agent_mask_;   // [10-10 W1-2]

  // [10-10 W1-2] resync 상태 (step 1 의 resync_* 멤버를 MirrorSync 로 묶음)
  MirrorSync global_sync_;
  MirrorSync mask_sync_;
  rclcpp::SubscriptionOptions mirror_subs_options_;   // [10-10 W1-4] 전환 때 구독을 다시 만들 때 쓴다 (subs 그룹)
  GlobalSourceSelector global_source_;                // [10-10 W1-4]

  // [10-10 W1-3] global costmap watchdog (R4) — GlobalWatchdogFlag 비트. 0 = 정상. 로그와 이 flag 로만 알린다.
  std::atomic<unsigned> global_watchdog_flags_{0u};
  std::chrono::steady_clock::time_point node_start_steady_{std::chrono::steady_clock::now()};
  // [10-10 W1-3] agent mask "모름" 으로 whoCoversPoint 경로를 탄 판정 수 (W2PV-1 (3)), 직전 mode (전환 로그용)
  std::atomic<uint64_t> mask_unknown_fallbacks_{0};
  mutable std::atomic<int> last_mask_mode_{-1};

  // 최신 MultiAgentInfoArray
  robot_interfaces::msg::MultiAgentInfoArray::SharedPtr last_agents_;
  rclcpp::Time last_agents_stamp_;
  mutable std::mutex agents_mutex_;

  // [FIX] machine_id 별 이웃 캐시.
  // winros_bridge 는 이웃 패킷 1개당 agent 1개짜리 배열을 발행하므로
  // (winros_bridge.cpp:1466 makeAndPublishMultiAgentMsg, :1614 push_back)
  // last_agents_ 만 들고 있으면 한 tick 에 이웃 한 대만 검사 대상이 된다.
  // 그러면 다른 로봇이 경로를 막아도 agent 로 귀속되지 않고 정적 장애물로
  // 떨어져(triggerReplan), ID 우선순위 양보 프로토콜이 개입하지 못한다.
  std::unordered_map<uint16_t, robot_interfaces::msg::MultiAgentInfo> agent_cache_;
  std::unordered_map<uint16_t, rclcpp::Time> agent_seen_at_;
  double agent_cache_ttl_sec_{5.0};

  std::atomic<bool> is_robot_in_driving_state_{false};
  std::atomic<bool> is_robot_in_ready_state_{false};
  // [09-29 D1-ⓒ] 직전 로봇 상태 — RECOVERY_* 로 들어가는 순간을 알아 저장해 둔 옛 경로를 버린다 (robotStatusCallback 전용)
  std::string last_robot_status_;
  rclcpp::Time last_replan_time_;        // replan 쿨다운 기준
  rclcpp::Time last_agent_block_time_;   // 에이전트 홀드 기준

  std::unordered_map<uint64_t, ObstacleInfo> obstacle_db_;
  mutable std::mutex obstacle_db_mutex_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ========= Parameters =========
  std::string global_frame_;
  std::string base_frame_;

// [add] 본인 식별용 ID 변수
  uint16_t self_machine_id_{0};

  double cooldown_sec_;
  size_t consecutive_threshold_;
  double obstacle_persistence_sec_;
  double max_speed_;
  double lookahead_time_sec_;
  double min_lookahead_m_;
  double cost_threshold_;
  bool ignore_unknown_;

  // DB/ROI
  double db_update_frequency_;
  double obstacle_prune_timeout_sec_;
  int db_stride_;
  double cone_angle_deg_;
  int kernel_half_size_;

  // Path checking
  double path_check_distance_m_;

  // Replan flag pulse
  bool publish_false_pulse_;
  int flag_pulse_ms_;

  // Footprint / Agent mask / Output
  bool use_footprint_check_;
  double footprint_step_m_;
  bool compare_agent_mask_;
  std::string agent_mask_topic_;

  // [10-10 W1-2] agent mask mirror / resync 파라미터 (min_interval / timeout / grace / qos depth / spare 는 global 것을 같이 쓴다)
  std::string agent_mask_service_;
  bool agent_mask_resync_on_start_{true};
  double agent_mask_resync_period_sec_{0.0};
  bool agent_mask_self_contained_updates_{true};   // [10-10 W1-2][리뷰]
  bool agent_mask_seed_from_global_{true};          // [10-10 W1-3] W2PV-1
  int costmap_replay_max_msgs_{256};               // [10-10 W1-2][리뷰] 두 mirror 공통 replay 보관 개수

  // [10-10 W1] global costmap mirror / resync 파라미터
  std::string global_costmap_topic_;
  std::string global_costmap_service_;
  bool costmap_resync_on_start_{true};
  double costmap_resync_period_sec_{0.0};
  double costmap_resync_min_interval_sec_{5.0};
  double costmap_resync_timeout_sec_{30.0};
  int costmap_updates_qos_depth_{100};          // [10-10 W1][리뷰] 기본 10 → 100
  double costmap_resync_start_grace_sec_{5.0};  // [10-10 W1][리뷰]
  bool costmap_keep_spare_buffer_{true};
  double costmap_resync_backoff_base_sec_{30.0};  // [10-10 W1-3] R3
  double costmap_resync_backoff_max_sec_{120.0};  // [10-10 W1-3] R3
  double costmap_stale_watchdog_sec_{10.0};       // [10-10 W1-3] R4 (0 = 끔)
  double costmap_resync_abandon_sec_{300.0};      // [10-10 W1-3][리뷰] R3: 이만큼 응답이 없으면 포기 (timeout 은 늦음 WARN)
  bool ready_hold_without_global_grid_{true};     // [10-10 W1-3][리뷰] READY + global grid 없음 → 출발 보류
  // [10-10 W1-4] global source: 기본 = diff layer. publisher 가 없으면 stock 으로 fallback (한 번), diff 가 생기면 복귀 (한 번).
  std::string global_costmap_fallback_topic_;
  std::string global_costmap_fallback_service_;
  double global_costmap_fallback_sec_{15.0};
  bool global_costmap_event_driven_{true};        // primary (diff layer) 는 값이 바뀔 때만 보낸다 → watchdog 은 publisher liveness
  double global_costmap_heartbeat_sec_{20.0};     // [10-10 W1-4][리뷰] event_driven source 의 무갱신 한도 (sweep heartbeat, 0 = 끔)
  std::atomic<uint64_t> ready_hold_no_grid_count_{0};   // [10-10 W1-3][리뷰] 보류를 낸 횟수
  std::atomic<bool> ready_hold_active_{false};          // [10-10 W1-3][리뷰] 보류 중 (해제 INFO 용)
  double agent_cost_threshold_;
  int agent_mask_manhattan_buffer_;

  // 충돌 메시지
  bool publish_agent_collision_;
  std::string agent_collision_topic_;

  // MultiAgent 구독
  std::string agents_topic_;
  int agents_freshness_timeout_ms_;
  double agent_match_dilate_m_;

  // Nav2 footprint
  std::string footprint_str_;
  std::vector<geometry_msgs::msg::Point> footprint_;
  bool use_radius_{true};
  double robot_radius_m_{0.1};

  // 에이전트 홀드 파라미터
  double agent_block_hold_sec_{2.0};
  double agent_block_max_wait_sec_{8.0};

  // === NEW: 에이전트 경로 튜브 매칭 파라미터 ===
  bool   agent_path_hit_enable_{true};
  double agent_path_hit_stride_m_{0.35};
  double agent_path_hit_dilate_m_{0.02};
  int    agent_path_hit_max_poses_{200};

  // [NEW] 우선순위에 따른 경로 검사 옵션
  bool ignore_self_fp_hit_{true};   // [V2 M-6]
  double self_fp_margin_m_{0.05};   // [V2 M-6]
  bool respect_higher_priority_path_{false};  



  std::atomic<bool> goal_occupied_flag_{false};
  geometry_msgs::msg::Pose locked_goal_pose_;
  std::vector<geometry_msgs::msg::PoseStamped> current_remaining_goals_;
  std::mutex goals_mutex_; // goals 데이터 보호용


  // [파라미터 추가]
  double goal_doorstep_static_m_;
  double ready_goal_horizon_m_{0.0};   // [V2.30] READY 목표 점유 검사의 노선 거리 상한 (0=무제한)
  double ready_escape_after_sec_{0.0};   // [V2.31] READY 에서 이만큼 막혀 있으면 탈출 상한으로 좁힌다 (0=끔)
  double ready_escape_horizon_m_{3.0};   // [V2.31] 탈출 때 쓰는 노선 거리 상한
  std::atomic<int64_t> ready_blocked_since_ns_{0};  // [V2.31] READY 에서 처음 막힌 시각 (0=기록 없음)
  std::atomic<bool> ready_escape_logged_{false};
  double goal_doorstep_agent_m_;

};

}  // namespace replan_monitor

#endif  // PATH_VALIDATOR_NODE_HPP
