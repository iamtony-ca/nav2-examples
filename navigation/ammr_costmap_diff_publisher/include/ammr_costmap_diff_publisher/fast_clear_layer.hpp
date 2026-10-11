// [10-10 W1-5] FastClearLayer — 큰 global costmap (2e8 셀) 용 "빠른 전체 clear".
//
// 왜: 표준 ClearEntireCostmap (ClearCostmapService::clearEntirely → Costmap2DROS::resetLayers) 은 master 를 통째로
//   default 로 지우고 모든 layer 의 reset() 을 부른다. StaticLayer::reset() 이 has_updated_data_ 를 세우므로 다음
//   updateMap 의 bounds 는 지도 전체가 되고, InflationLayer 의 last bounds 때문에 그 다음 cycle 도 지도 전체다.
//   2e8 셀에서 한 cycle 이 ~10 s → planner "Costmap timed out waiting for update" → BT recovery → 또 clear.
//
// 하는 일 (서비스 <name>/clear_entirely, nav2_msgs/srv/ClearEntireCostmap — 표준과 같은 type):
//   1. master mutex 를 표준 clearEntirely 와 똑같이 잡는다.
//   2. plugins (+filters) 중 isClearable() 이 true 인 layer 마다 (= ClearCostmapService::clearRegion 이 지우는 layer 와 같은 기준)
//      그 layer 의 char map 에서 default 값과 다른 셀의 bounding box 를 구하고, 그 layer 의 reset() 을 부른다
//      (표준과 같은 per-layer 효과: ObstacleLayer 는 resetMaps + buffer last-updated + current_=false/was_reset_).
//   3. 모든 box 의 합집합을 world 좌표로 보관 → 다음 updateMap 의 이 layer updateBounds 가 그 사각형으로 bounds 를 넓힌다.
//      inflation layer 가 이 layer 뒤에 있으면 그 updateBounds 가 inflation_radius 만큼 더 넓힌다 (fork inflation_layer.cpp:619-622).
//      뒤에 없으면 (목록 순서 실수) 이 layer 가 가장 큰 inflation_radius 만큼 스스로 넓힌다.
//   - master 는 건드리지 않는다. static / keepout(lane_filter) / denoise / agent / inflation / diff_publisher 는 reset 하지 않는다
//     (표준 reset 이 그들에게 하는 일은 "다시 계산하라" 뿐 — 결과 값이 같다. W1_STEP5.md §3).
//   - isClearable() = false, updateCosts 는 아무것도 안 한다, bounds 는 clear 직후 한 번만 넓힌다.
//   - CostmapLayer 가 아닌 clearable layer (또는 PluginContainerLayer·크기 불일치) 는 extent 를 못 구하므로 reset 하고 지도 전체를 bounds 로
//     (표준과 같은 비용, WARN).
//   - enabled: false 면 표준 clearEntirely 와 똑같이 (master 전체 resetMap + 모든 layer/filter reset) — 이름만 바꾼 BT/관리자를
//     그대로 두고 표준 동작으로 되돌리는 A/B 스위치.
//
// 서비스는 전용 callback group + 전용 executor thread 에서 돈다 (DiffPublisherLayer get_costmap 과 같은 방식).
//
// [10-10 W1-5][리뷰] 반영 3 가지:
//   - 창 여유: InflationLayer::updateBounds 는 inflation_radius (m, 0.61 = 20.33 셀) 만 넓히는데 updateCosts 는
//     cell_inflation_radius = ceil(20.33) = 21 셀까지 칠한다 → 여유를 "cellDistance(반경) 셀" 기준으로 잡는다 (뒤 inflation 이 넓히는 몫은 뺀다).
//   - keepout mask 가 늦게 / 다시 도착: KeepoutFilter::maskCallback 은 bounds 를 넓히지 않는다. 표준 clear 는 명령마다 지도 전체를 다시 칠해
//     이것을 고쳤는데 빠른 clear 는 아니다 → keepout 이 inactive → active 로 바뀌는 것을 보면 그 cycle 한 번 지도 전체 bounds 를 요구한다.
//   - lifecycle: deactivate() 이후 clear 요청은 아무것도 안 한다 (진행 중인 clear 가 끝날 때까지 deactivate 가 기다린다).
//     Costmap2DROS::on_cleanup 의 ~LayeredCostmap 이 lock 없이 plugins_ 를 pop_back 하기 전에 on_deactivate → stop() 이 deactivate() 를 부른다.
#ifndef AMMR_COSTMAP_DIFF_PUBLISHER__FAST_CLEAR_LAYER_HPP_
#define AMMR_COSTMAP_DIFF_PUBLISHER__FAST_CLEAR_LAYER_HPP_

#include <atomic>
#include <chrono>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ammr_costmap_diff_publisher
{

// 반열린 셀 사각형 [x0,x1) x [y0,y1)
struct CellBox
{
  int x0{0}, y0{0}, x1{0}, y1{0};
  bool valid() const {return x1 > x0 && y1 > y0;}
  size_t area() const {return valid() ? size_t(x1 - x0) * size_t(y1 - y0) : 0;}
  void merge(const CellBox & o);
};

// data[size_x*size_y] (행 우선) 에서 값이 default_value 와 다른 셀의 bounding box. 없으면 invalid.
// 8 byte 단위 비교로 행을 건너뛴다 (2e8 셀 ~ 수십 ms).
CellBox scanNonDefaultExtent(
  const unsigned char * data, unsigned int size_x, unsigned int size_y, unsigned char default_value);

struct FastClearLayerReport
{
  std::string name;
  CellBox box;          // 값이 있던 셀 (invalid = 비어 있었음)
  bool full{false};     // extent 를 못 구해 지도 전체로 잡음
  std::string why;      // full 인 이유
};

struct FastClearReport
{
  bool standard{false};           // enabled=false → 표준 clearEntirely 를 했다
  bool skipped{false};            // [10-10 W1-5][리뷰] deactivate 상태라 아무것도 안 했다
  std::vector<FastClearLayerReport> layers;
  CellBox extent;                 // 합집합 (셀)
  // 이 layer 가 스스로 더한 여유 (m). [10-10 W1-5][리뷰] = extra_margin_m + max(0, cellDistance(반경)*res − 뒤 inflation 이 더하는 반경)
  //   → 최종 창이 extent ± cell_inflation_radius 셀 (InflationLayer 가 실제로 칠하는 거리) 을 덮는다. 0.61 m @0.03 → 0.02 (앞) / 0.63 (뒤 없음)
  double margin_m{0.0};
  bool inflation_after{false};    // inflation layer 가 이 layer 뒤에 있다 (그 updateBounds 가 반경만큼 넓힌다)
  double inflation_radius_m{0.0}; // 찾은 inflation layer 의 가장 큰 반경
  double inflation_after_radius_m{0.0};  // 뒤에 있는 inflation layer 의 가장 큰 반경 (그 updateBounds 가 m 로 더하는 몫)
  unsigned int cell_inflation_radius{0};  // cellDistance(inflation_radius_m) — InflationLayer 와 같은 올림
  unsigned int size_x{0}, size_y{0};
  double lock_wait_ms{0.0}, scan_ms{0.0}, reset_ms{0.0}, total_ms{0.0};
};

class FastClearLayer : public nav2_costmap_2d::Layer
{
public:
  FastClearLayer() = default;
  ~FastClearLayer() override;

  void onInitialize() override;
  // 표준 ClearEntireCostmap 이 불리면 지도 전체가 다시 계산되므로 보관한 사각형은 의미가 없다 — 그대로 둬도 해는 없다 (넓히기만 함).
  void reset() override {}
  bool isClearable() override {return false;}
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  // master 는 바꾸지 않는다 (clear 뒤 2 cycle 동안 다시 계산된 창 크기만 로그로 남긴다)
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) override;
  void matchSize() override {}
  // [10-10 W1-5][리뷰] Costmap2DROS::start()/stop() (on_activate / on_deactivate) 이 부른다
  void activate() override;
  void deactivate() override;

  // 서비스 콜백과 같은 일 (master lock 포함). 시험·서비스 둘 다 이것을 부른다. deactivate 뒤에는 skipped 만 세우고 돌아온다.
  FastClearReport clearNow();

  std::string serviceName() const {return service_name_;}
  // 마지막 clear 뒤 cycle 들에서 updateCosts 로 받은 창의 합집합 (= 실제로 다시 계산된 범위, 시험·진단용)
  CellBox postClearWindow();
  bool hasPending();
  uint64_t calls() const {return calls_.load();}
  uint64_t skippedCalls() const {return skipped_calls_.load();}
  // keepout mask 도착을 보고 지도 전체 bounds 를 요구한 횟수
  uint64_t maskRepaints() const {return mask_repaints_.load();}

private:
  void serviceCallback(
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Request> req,
    std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Response> res);
  void logReport(const FastClearReport & r);

  // keepout (CostmapFilter) mask 도착 감시 — updateMap thread 에서만 쓴다
  void watchFilterMasks(double * min_x, double * min_y, double * max_x, double * max_y);

  double extra_margin_m_{0.0};
  std::string service_name_;

  // 다음 updateBounds 에서 넓힐 world 사각형 (pending_mtx_)
  std::mutex pending_mtx_;
  bool pending_{false};
  double p_min_x_{0.0}, p_min_y_{0.0}, p_max_x_{0.0}, p_max_y_{0.0};

  rclcpp::Service<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr service_;
  rclcpp::CallbackGroup::SharedPtr srv_cb_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr srv_executor_;
  std::thread srv_thread_;
  std::atomic<bool> srv_running_{true};
  // [10-10 W1-5][리뷰] deactivate 뒤 false. clear_mtx_ 는 clearNow 전체를 감싼다 (deactivate 가 진행 중인 clear 를 기다린다).
  //   순서: clear_mtx_ → master mutex → layer mutex. deactivate 는 clear_mtx_ 만 잡는다.
  std::mutex clear_mtx_;
  std::atomic<bool> active_{true};
  std::atomic<uint64_t> calls_{0};
  std::atomic<uint64_t> skipped_calls_{0};
  std::chrono::steady_clock::time_point skip_log_last_{};   // 서비스 thread 에서만 쓴다
  std::atomic<uint64_t> mask_repaints_{0};
  bool filters_scanned_{false};
  struct WatchedMask
  {
    std::weak_ptr<nav2_costmap_2d::Layer> layer;
    std::string name;
    bool active{false};
  };
  std::vector<WatchedMask> watched_masks_;
  std::atomic<int> watch_cycles_{0};   // clear 뒤 창 크기를 로그로 남길 남은 cycle 수
  std::atomic<bool> last_extent_whole_{false};
  std::mutex window_mtx_;
  CellBox post_clear_window_;   // 마지막 clear 의 범위가 이미 지도 전체였다 (그때는 WARN 안 함)
};

}  // namespace ammr_costmap_diff_publisher

#endif  // AMMR_COSTMAP_DIFF_PUBLISHER__FAST_CLEAR_LAYER_HPP_
