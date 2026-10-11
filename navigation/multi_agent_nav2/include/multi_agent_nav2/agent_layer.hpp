#pragma once

#include <atomic>   // [10-10 W1-2][리뷰]
#include <chrono>   // [10-10 W1-3]
#include <mutex>
#include <vector>
#include <string>
#include <utility>
#include <algorithm>
#include <cmath>
#include <map>
#include <memory> // unique_ptr

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_publisher.hpp> 
#include "multi_agent_nav2/agent_mask_canvas.hpp"   // [10-10 W1-2]

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <robot_interfaces/msg/multi_agent_info_array.hpp>
#include <robot_interfaces/msg/agent_layer_meta_array.hpp>
#include <robot_interfaces/msg/agent_status.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// [NEW] Dynamic parameter callback 지원
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace multi_agent_nav2
{

class AgentLayer : public nav2_costmap_2d::Layer
{
public:
  AgentLayer();

  // lifecycle
  void onInitialize() override;
  void activate() override;
  void deactivate() override;

  void reset() override { 
    current_ = true; 
    last_touched_ = false; 
    touched_ = false; 
  }

  // costmap callbacks
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;

  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                   int min_i, int min_j, int max_i, int max_j) override;

  // [10-10 W1-2][리뷰] true → false. ClearCostmapExceptRegion / ClearCostmapAroundRobot 은 isClearable() 이 true 인
  //   layer 를 std::static_pointer_cast<CostmapLayer> 해 clearArea 를 부르는데 (fork clear_costmap_service.cpp:116-118),
  //   AgentLayer 는 CostmapLayer 가 아니라 Layer 라서 엉뚱한 메모리(viz_costmap_·mask_canvas_ 등)를 덮어쓰는 UB 였다.
  //   agent layer 는 매 사이클 다시 칠하므로 지울 것도 없다. ClearEntireCostmap(resetLayers → reset()) 은 영향 없다.
  bool isClearable() override { return false; }

private:
  // ====== [NEW] Dynamic parameter 지원 ======
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
      std::vector<rclcpp::Parameter> parameters);

  // 파라미터 변경과 updateBounds/updateCosts 간 race 방지용
  std::mutex param_mtx_;

  // Callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr 
      dyn_params_handler_;
  // ==========================================

  // node handle
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_shared_;

  // I/O
  rclcpp::Subscription<robot_interfaces::msg::MultiAgentInfoArray>::SharedPtr sub_;
  rclcpp::Publisher<robot_interfaces::msg::AgentLayerMetaArray>::SharedPtr meta_pub_;

  // 시각화 전용
  nav2_costmap_2d::Costmap2D viz_costmap_;
  // [10-10 W1-2] viz_costmap_ 의 칠한 사각형 관리 (직전 사각형만 지우고, 바뀐 사각형만 publish).
  //   viz_costmap_ 뒤에 선언해야 한다 (참조로 받는다).
  AgentMaskCanvas mask_canvas_{viz_costmap_};
  std::unique_ptr<nav2_costmap_2d::Costmap2DPublisher> costmap_pub_;
  // [10-10 W1-2] true = 예전 동작 (매 사이클 viz 전체 memset + always_send_full publisher + bounds 전체). A/B 용.
  //   publisher 의 always_send_full 은 생성자 인자라 onInitialize 에서 한 번만 읽는다 (dynamic 변경은 무시).
  // [10-10 W1-3] S2-OPS-1/W2-1: 기본값 false → true (예전 소비자와 호환). update 방식은 params 에서 false 로 명시해 켠다.
  bool full_mask_publish_{true};
  // [10-10 W1-3] 배포 guard timer (activate 뒤, callback_group_ 에서 돈다): 예전 full-only 소비자 감지 WARN.
  rclcpp::TimerBase::SharedPtr legacy_listener_timer_;
  std::chrono::steady_clock::time_point activated_at_{};
  bool legacy_listener_warned_{false};
  void checkLegacyMaskListener();
  // [10-10 W1-2] update 사각형에 넣을 직전 사이클 수 (1 = 직전 사각형 ∪ 이번 사각형). N 이면 연속 N-1 개 유실을 다음
  //   update 가 메운다. onInitialize 에서 한 번만 읽는다.
  int mask_update_history_{1};
  // [10-10 W1-2][리뷰] refresh 주기 (사이클 수, 0 = 끔). 이 주기마다 최근에 보낸 사각형 전체를 한 번 더 보내,
  //   DDS 가 알리지 않은 update 유실(특히 떠난 이웃의 마지막 지우기)로 구독자에 남은 유령 셀을 덮는다. onInitialize 에서만 읽는다.
  int mask_refresh_cycles_{10};
  // [10-10 W1-2][리뷰] activate() 가 세우고 다음 updateCosts 가 내린다 (lifecycle thread → map update thread).
  //   그 사이클은 grid 전체 사각형을 update 로 보낸다: deactivate 중 inactive publisher 에 버려진 update 와
  //   fork Costmap2DPublisher 의 초기화 안 된 geometry 멤버(첫 full 이 빠질 수 있음)를 덮는다.
  std::atomic<bool> force_full_rect_{false};

  // [FIX AL-1] 이웃 캐시 — machine_id 별 누적.
  //
  // winros_bridge 는 관제의 이웃 패킷 1개마다 agent 1개짜리 배열을 발행한다
  // (winros_bridge.cpp makeAndPublishMultiAgentMsg: 구조체 하나를 push_back 하고
  // 바로 publish). 예전처럼 마지막 배열 하나(last_infos_)만 들고 있으면
  // 코스트맵에는 항상 "마지막 패킷의 1대" 만 그려지고, 어느 1대인지는 위치가
  // 아니라 관제의 송신 순서로 정해졌다. compare_agent_mask 를 켠 path_validator 가
  // 이 마스크를 충돌 판정 입력으로 쓰므로 나머지 이웃은 플래너에도 충돌 판정에도
  // 안 보였다. fleet_decision(B-2a)/path_validator(PV-1) 과 같은 형태로 고친다.
  //
  // 배열 길이가 1 이든 N 이든 항목 단위로 병합하므로, 나중에 winros_bridge 가
  // 이웃 전체를 한 메시지에 담아 보내도 이 코드는 그대로 동작한다.
  struct CachedAgent {
    robot_interfaces::msg::MultiAgentInfo info;   // header.frame_id 는 항상 채워 둔다
    rclcpp::Time seen_at;                          // 메시지 stamp (freshness 판정용)
    // [V2] 자세 이력으로 추정한 속도 (관제 twist 는 0 으로 온다). 수신 시각 기준.
    double hist_x{0.0}, hist_y{0.0};
    rclcpp::Time hist_t{0, 0, RCL_ROS_TIME};
    double speed_mps{-1.0};                        // -1 = 아직 모름
  };
  std::mutex data_mtx_;
  std::map<uint16_t, CachedAgent> agent_cache_;

  // parameters
  bool        enabled_{true};
  std::string topic_{"/multi_agent_infos"};
  uint16_t    self_machine_id_{0};
  std::string self_type_id_{};
  bool        use_path_header_frame_{true};
  double      roi_range_m_{12.0};
  double      time_decay_sec_{1.0};
  unsigned char lethal_cost_{254};
  unsigned char moving_cost_{180};
  unsigned char waiting_cost_{200};
  int         manual_cost_bias_{30};

  double      dilation_m_{0.05};          
  double      forward_smear_m_{0.25};     
  double      sigma_k_{2.0};              

  // ========================================================
  // 경로(Path) 전용 파라미터
  // ========================================================
  double      path_dilation_m_{-0.1}; // 본체와 분리된 경로 전용 팽창값 (음수 허용)
  int         path_base_cost_{200};   // 로봇 바로 앞 경로 코스트
  int         path_end_cost_{50};     // 멀어질수록 도달하는 끝점 코스트
  // ========================================================

  bool        publish_meta_{true};
  int         meta_stride_{3};
  int         freshness_timeout_ms_{800};
  int         max_poses_{40};
  bool        qos_reliable_{true};

  // bounds cache for this cycle
  double touch_min_x_{0.0}, touch_min_y_{0.0}, touch_max_x_{0.0}, touch_max_y_{0.0};
  bool   touched_{false};

  // 이전 사이클의 Bounds (잔상 지우기용)
  double last_min_x_{1e9}, last_min_y_{1e9}, last_max_x_{-1e9}, last_max_y_{-1e9};
  bool   last_touched_{false};

  // Cached robot pose
  double cached_robot_x_{0.0};
  double cached_robot_y_{0.0};

  bool ignore_higher_machine_id_path_{true};
  // [V2] 시간 인지형 경로 튜브: 상대 속도 × tube_horizon_sec 만큼만 칠한다 (min/max 로 자름).
  // 정지한 상대(속도 < agent_moving_mps) 는 paint_stopped_agent_path 가 아니면 몸체만 칠한다.
  double tube_horizon_sec_{4.0};
  double tube_min_m_{0.5};
  double tube_max_m_{4.0};
  double agent_moving_mps_{0.05};
  bool   paint_stopped_agent_path_{false};
  std::map<uint16_t, double> agent_speed_;      // updateBounds 에서 캐시에서 복사 (updateCosts 용)

  double load_rear_smear_m_{0.4};   // [NEW] LOADING/UNLOADING 시 rear 확장량


  // TF 시간차 캐시
  std::vector<robot_interfaces::msg::MultiAgentInfo> transformed_agents_;

  struct AgentFootprintData
  {
    std::vector<geometry_msgs::msg::Point32> points;
    double radius{0.0};
    bool use_radius{true};
  };
  std::map<uint16_t, AgentFootprintData> agent_footprints_;

  // Helpers
  geometry_msgs::msg::PolygonStamped 
  getFootprintForAgent(const robot_interfaces::msg::MultiAgentInfo & a);

  static std::vector<geometry_msgs::msg::Point32> toPoint32(
      const std::vector<geometry_msgs::msg::Point>& points);

  bool transformAgentInfo(
      const robot_interfaces::msg::MultiAgentInfo & agent_in_map,
      robot_interfaces::msg::MultiAgentInfo & agent_in_costmap_frame,
      const std::string & costmap_frame) const;

  void infosCallback(const robot_interfaces::msg::MultiAgentInfoArray::SharedPtr msg);
  bool isSelf(const robot_interfaces::msg::MultiAgentInfo & a) const;
  bool stale(const rclcpp::Time & stamp) const;

  // [10-10 W1-2] grid 대신 canvas 를 받는다 (쓰기는 canvas.paintMax 로만 — 칠한 사각형 추적).
  void rasterizeAgentPath(const robot_interfaces::msg::MultiAgentInfo & a,
                          AgentMaskCanvas & canvas,
                          std::vector<std::pair<unsigned int,unsigned int>> & meta_hits);

  void fillFootprintAt(const geometry_msgs::msg::PolygonStamped & fp,
                       const geometry_msgs::msg::Pose & pose,
                       double extra_dilation_m,
                       double forward_len_m,
                       double rear_len_m,                  // [NEW]
                       AgentMaskCanvas & canvas,           // [10-10 W1-2] grid → canvas
                       unsigned char cost,
                       std::vector<std::pair<unsigned int,unsigned int>> * meta_hits = nullptr);

  static bool pointInPolygon(const std::vector<geometry_msgs::msg::Point> & poly,
                             double x, double y);

  double computeDilation(const robot_interfaces::msg::MultiAgentInfo & a) const;

  double computeRearSmear(const robot_interfaces::msg::MultiAgentInfo & a) const;


  unsigned char computeCost(const robot_interfaces::msg::MultiAgentInfo & a) const;

  static inline bool isMovingPhase(uint8_t phase)
  {
    using S = robot_interfaces::msg::AgentStatus;
    return phase == S::STATUS_MOVING || phase == S::STATUS_PATH_SEARCHING;
  }

  inline bool inRoi(double wx, double wy) const
  {
    const double dx = wx - cached_robot_x_;
    const double dy = wy - cached_robot_y_;
    return std::hypot(dx, dy) <= roi_range_m_;
  }


};

} // namespace multi_agent_nav2
