// [10-10 W1-D] DiffEngine — master costmap 과 "구독자가 가진 것"(shadow) 을 비교해
// 값이 바뀐 셀만 nav2_msgs/CostmapUpdate 로 만드는 엔진. ROS 노드와 무관하게 gtest 로 돌릴 수 있다.
//
// 스레드 모델
//   - processCycle()  : costmap map-update thread (master lock 을 쥔 LayeredCostmap::updateMap 안, layer updateCosts)
//   - pop()/onPublished()/onPublishFailed() : publisher thread (costmap lock 밖)
//   - snapshot()       : get_costmap 서비스 전용 executor thread
//   - requestFull()    : 아무 thread
// lock 순서는 항상 shadow_mtx_ → queue_mtx_ → catchup_mtx_ → stats_mtx_ (안쪽으로만 잡는다).
//
// 불변식
//   (I1) shadow_ = 지금까지 큐에 넣은(enqueue) 모든 메시지를 순서대로 적용한 결과. 즉 "구독자가 큐를 다 받으면 가질 grid".
//   (I2) 큐에 넣는/보내는 메시지의 stamp 는 보내는 순서대로 엄격히 증가한다 (shadow_mtx_ 아래에서 부여).
//   (I3) FULL 은 큐에 token 으로만 들어가고, publisher thread 가 꺼낼 때 shadow_ 에서 만든다.
//        만들기 시작할 때 큐에 남아 있는 update 는 모두 그 FULL 에 포함되므로 버린다.
//        [10-10 W1-D][리뷰] FULL·snapshot 사본은 copy_chunk_bytes 행 묶음으로 나눠 복사하고 묶음 사이에 shadow_mtx_ 를 놓는다
//        (map thread 가 지도 크기 사본 동안 막히지 않게). 복사 도중 map thread 가 넣은 update 는 큐에 남아 FULL 뒤에
//        (더 큰 stamp 로) 나가므로, 구독자는 FULL + 그 update 들을 적용하면 shadow 와 같아진다 (CostmapUpdate 는 절대값).
//   (I4) [10-10 W1-D][리뷰] repair (catch-up / refresh / sweep / 재시도) update 는 큐가 비었을 때만 shadow 에서 그 자리에서
//        만든다 (shadow·queue lock 아래에서 stamp) → (I2) 유지, shadow 는 바꾸지 않는다.
#ifndef AMMR_COSTMAP_DIFF_PUBLISHER__DIFF_ENGINE_HPP_
#define AMMR_COSTMAP_DIFF_PUBLISHER__DIFF_ENGINE_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/msg/costmap_update.hpp"

namespace ammr_costmap_diff_publisher
{

// 셀 사각형, 반열림 [x0,x1) × [y0,y1)
struct Rect
{
  uint32_t x0{0}, y0{0}, x1{0}, y1{0};
  bool empty() const {return x1 <= x0 || y1 <= y0;}
  uint64_t area() const {return empty() ? 0 : uint64_t(x1 - x0) * uint64_t(y1 - y0);}
};

struct Geometry
{
  uint32_t size_x{0}, size_y{0};
  double resolution{0.0};
  double origin_x{0.0}, origin_y{0.0};        // Costmap2D::getOriginX/Y (변경 판정용)
  double msg_origin_x{0.0}, msg_origin_y{0.0};  // 메시지용 (Costmap2DPublisher::prepareCostmap 와 같은 식)
  bool valid() const {return size_x > 0 && size_y > 0;}
  bool sameAs(const Geometry & o) const
  {
    return size_x == o.size_x && size_y == o.size_y && resolution == o.resolution &&
           origin_x == o.origin_x && origin_y == o.origin_y;
  }
  static Geometry of(const nav2_costmap_2d::Costmap2D & c);
};

enum class FullReason : uint8_t
{
  kInitial, kGeometry, kNewSubscriber, kBacklog, kActivate, kLargeDiff, kManual,
  kRetry   // [10-10 W1-D][리뷰] FULL publish 실패 뒤 다시 요청
};
const char * toString(FullReason r);

// [10-10 W1-D][리뷰] 큐 밖에서 shadow 로 만드는 재전송 update 의 출처
enum class RepairSource : uint8_t { kNone, kCatchup, kRefresh, kSweep, kRetry };
const char * toString(RepairSource r);

struct OutMsg
{
  enum class Kind : uint8_t { kNone, kFull, kUpdate };
  Kind kind{Kind::kNone};
  FullReason reason{FullReason::kManual};       // kFull 일 때만
  bool repair{false};                           // [10-10 W1-D][리뷰] 재전송 update (catch-up/refresh/sweep/retry) 인지
  RepairSource src{RepairSource::kNone};
  std::unique_ptr<nav2_msgs::msg::Costmap> full;
  std::unique_ptr<nav2_msgs::msg::CostmapUpdate> update;
  size_t bytes{0};
};

// 주기 통계 (takeStats() 가 꺼내고 0 으로 되돌린다)
struct EngineStats
{
  uint64_t cycles{0};
  uint64_t compared_cells{0};
  uint64_t changed_cells{0};
  uint64_t rects{0};
  uint64_t update_msgs_enqueued{0};
  uint64_t update_bytes_enqueued{0};
  uint64_t update_msgs_sent{0};
  uint64_t update_bytes_sent{0};
  uint64_t fulls_sent{0};
  uint64_t full_bytes_sent{0};
  uint64_t dropped_updates{0};       // FULL 에 포함되어 버린 update 수 (backlog 포함)
  uint64_t backlog_trips{0};         // max_queue_bytes 초과로 FULL 로 바꾼 횟수
  uint64_t large_diff_fulls{0};      // full_fallback_ratio 초과로 FULL 로 바꾼 횟수
  uint64_t catchups{0};              // catch-up 단계 실행 수 (FULL 하나당 catchup_delays_sec 개)
  uint64_t catchup_msgs{0};
  uint64_t catchup_bytes{0};
  // [10-10 W1-D][리뷰] 손실을 모르는 구독자(CostmapSubscriber KeepLast(10)) 를 위한 자가 복구
  uint64_t refreshes{0};
  uint64_t refresh_msgs{0};
  uint64_t refresh_bytes{0};
  uint64_t sweep_msgs{0};
  uint64_t sweep_bytes{0};
  uint64_t sweeps_completed{0};      // 지도 전체를 한 바퀴 다 보낸 횟수
  uint64_t retry_msgs{0};
  uint64_t repair_overflow_merges{0};
  // [10-10 W1-D][리뷰] 실패·중단
  uint64_t full_aborts{0};           // 묶음 복사 중 geometry 변경 / 새 FULL 요청으로 버린 FULL
  uint64_t full_publish_failures{0};
  uint64_t update_publish_failures{0};
  uint64_t full_ack_timeouts{0};
  double max_full_ack_ms{0.0};       // FULL publish → 모든 reader ack 까지
  double max_shadow_hold_ms{0.0};    // publisher/service thread 가 shadow_mtx_ 를 한 번에 쥔 최대 시간
  uint64_t full_compare_cycles{0};   // reset 뒤 / audit 로 지도 전체를 비교한 cycle 수
  uint64_t outside_bounds_cells{0};  // full compare 에서 bounds 밖에서 발견된 변경 셀 (불변식 위반 감시)
  double max_compare_ms{0.0};
  double max_cycle_ms{0.0};
};

struct CycleResult
{
  bool full_requested{false};
  bool full_compare{false};
  uint64_t compared_cells{0};
  uint64_t changed_cells{0};
  uint64_t outside_bounds_cells{0};
  size_t rects{0};
  size_t bytes{0};
  double compare_ms{0.0};
  double cycle_ms{0.0};
};

// ---- 사각형 추출 (테스트에서 직접 부른다) ----
// Morton 순서로 이웃한 쌍 중 합쳤을 때 늘어나는 면적이 가장 작은 것부터 합쳐 max_rects 개 이하로 만든다.
std::vector<Rect> mergeRectsGreedy(std::vector<Rect> rects, uint32_t max_rects);
// mask: w × h 개 tile (row-major, 0/1). tile (0,0) 의 왼쪽 아래 셀 = (base_x, base_y), tile 한 변 = tile 셀.
// 같은 tile row 의 연속 run 을 사각형으로, 위아래 row 의 run 이 정확히 같으면 세로로 이어 붙인다. clip 으로 자른다.
// boxes 가 있으면 (mask 와 같은 크기, mask=1 인 tile 만 유효) 각 tile 의 tight box 합집합으로 사각형을 줄인다.
std::vector<Rect> rectsFromMask(
  const std::vector<uint8_t> & mask, uint32_t w, uint32_t h,
  uint32_t base_x, uint32_t base_y, uint32_t tile, const Rect & clip,
  const std::vector<Rect> * boxes = nullptr);
// 위 결과가 max_rects 를 넘으면 mergeRectsGreedy 로 줄인다 (개수가 상한의 16 배를 넘으면 먼저 2×2 OR 로 줄임).
// 항상 max_rects 이하, dirty tile 은 모두 덮는다.
std::vector<Rect> rectsFromMaskCapped(
  std::vector<uint8_t> mask, uint32_t w, uint32_t h,
  uint32_t base_x, uint32_t base_y, uint32_t tile, const Rect & clip, uint32_t max_rects,
  const std::vector<Rect> * boxes = nullptr);

class DiffEngine
{
public:
  struct Config
  {
    uint32_t tile_size{64};
    uint32_t max_rects{256};
    size_t max_queue_bytes{64ull * 1024ull * 1024ull};
    size_t history_depth{10};            // = updates topic 의 transient_local depth
    // [10-10 W1-D][리뷰] FULL 이 모든 reader 에 ack 된 뒤(layer 가 wait_for_all_acked 후 onPublished) 각 시각에
    //   catch-up 을 보낸다. 마지막 단계까지 FULL 뒤 넣은 update 사각형을 계속 모은다. 비우면 끔.
    std::vector<double> catchup_delays_sec{2.0, 10.0};
    // [10-10 W1-D][리뷰] refresh: refresh_period_sec 마다 직전 refresh_rounds 주기 동안 보낸 (repair 아닌) update 사각형을
    //   shadow 현재 값으로 다시 보낸다 (refresh_max_rects 개 이하로 병합). <=0 이면 끔.
    double refresh_period_sec{5.0};
    uint32_t refresh_rounds{2};
    uint32_t refresh_max_rects{32};
    // [10-10 W1-D][리뷰] sweep: sweep_period_sec 에 지도 전체를 한 바퀴 (sweep_tick_sec 마다 행 band 하나). <=0 이면 끔.
    //   update 를 잃어도 늦어도 한 바퀴 뒤에는 모든 구독자가 shadow 와 같아진다.
    double sweep_period_sec{300.0};
    double sweep_tick_sec{1.0};
    // [10-10 W1-D][리뷰] update 메시지 하나의 data 상한 (넘는 사각형은 행 band 로 나눔) → durable history 크기 상한
    size_t max_msg_bytes{2ull * 1024ull * 1024ull};
    // [10-10 W1-D][리뷰] 한 cycle 사각형 면적(byte) 이 이것을 넘으면 update 대신 FULL (0 이면 끔)
    size_t max_update_cycle_bytes{16ull * 1024ull * 1024ull};
    // [10-10 W1-D][리뷰] FULL·snapshot 을 이 크기 행 묶음으로 복사하고 묶음 사이에 shadow_mtx_ 를 놓는다 (0 = 한 번에)
    size_t copy_chunk_bytes{2ull * 1024ull * 1024ull};
    uint32_t audit_period_cycles{0};     // >0 이면 N cycle 마다 지도 전체 비교 (bounds 밖 변경 감시용, 비쌈)
    // 한 cycle 의 사각형 면적이 지도의 이 비율을 넘으면 update 대신 FULL (master lock 밖, publisher thread 에서 만든다).
    // 흩어진 변경이 사각형 상한에 걸려 지도 대부분을 덮을 때. 0 이하면 끔.
    double full_fallback_ratio{0.5};
    std::string frame_id{"map"};
    // 시험용 hook (기본 비어 있음): FULL 버퍼 할당 직전 / FULL 묶음 복사 사이 (lock 없음)
    std::function<void(size_t)> test_before_full_alloc;
    std::function<void()> test_between_full_chunks;
  };
  using StampFn = std::function<int64_t()>;   // ns (node clock)
  using SteadyClock = std::chrono::steady_clock;

  DiffEngine(const Config & cfg, StampFn stamp_fn);

  const Config & config() const {return cfg_;}

  // map thread. master lock 은 호출자가 쥐고 있다 (LayeredCostmap::updateMap).
  // [min_i,max_i) × [min_j,max_j) 는 LayeredCostmap 이 updateCosts 에 넘기는 bounds (반열림, 이미 지도 안으로 clamp).
  CycleResult processCycle(
    const nav2_costmap_2d::Costmap2D & master,
    int min_i, int min_j, int max_i, int max_j);

  // Layer::reset() (ClearEntireCostmap → Costmap2DROS::resetLayers) 에서 부른다.
  // resetLayers 는 updateMap 밖에서 master 전체를 default 로 지우므로, 다음 cycle 은 bounds 와 상관없이 지도 전체를 비교한다.
  void markResetPending() {reset_pending_.store(true);}

  void requestFull(FullReason reason);

  // publisher thread. 꺼낼 것이 없으면 false.
  bool pop(OutMsg & out, SteadyClock::time_point now);
  bool waitPop(OutMsg & out, std::chrono::milliseconds timeout);
  // FULL 이면 now = 모든 reader ack 를 기다린 뒤의 시각 (catch-up 시각의 기준)
  void onPublished(const OutMsg & m, SteadyClock::time_point now);
  // [10-10 W1-D][리뷰] publish 가 throw 했을 때: FULL 이면 다시 요청, update 면 그 사각형을 repair 로 다시 보낸다.
  void onPublishFailed(const OutMsg & m);
  // [10-10 W1-D][리뷰] FULL ack 대기 결과 (통계)
  void noteFullAck(double ms, bool acked);
  // [10-10 W1-D][리뷰] update 구독자가 없으면 refresh/sweep 을 만들지 않는다
  void setHaveSubscribers(bool v) {have_subscribers_.store(v);}
  size_t repairPending() const {return repair_n_.load();}
  // publisher thread 를 멈출 때 true (waitPop 이 바로 false 를 돌려준다), 다시 시작할 때 false.
  void setStopping(bool stopping);

  // get_costmap 서비스: shadow 사본 + 사본 시작 때 마지막으로 큐에 넣은 메시지의 stamp. geometry 가 없으면 false.
  // [10-10 W1-D][리뷰] 묶음 복사 — 사본에 stamp 보다 새 값이 섞일 수 있지만 그 값의 update 는 모두 stamp 보다 큰 stamp 로
  //   나가므로, 응답 stamp 보다 큰 update 를 다시 적용하는 구독자(path_validator CostmapMirror) 는 shadow 와 같아진다.
  bool snapshot(nav2_msgs::msg::Costmap & out) const;

  EngineStats takeStats();
  size_t queueLength() const;
  size_t queuedUpdateBytes() const;
  bool fullPending() const;
  Geometry geometry() const;
  int64_t lastStamp() const;

  static void stampToMsg(int64_t ns, builtin_interfaces::msg::Time & t);

private:
  struct Item
  {
    OutMsg::Kind kind{OutMsg::Kind::kUpdate};
    FullReason reason{FullReason::kManual};
    std::unique_ptr<nav2_msgs::msg::CostmapUpdate> update;
    size_t bytes{0};
  };
  struct RepairRect
  {
    Rect r;
    RepairSource src{RepairSource::kNone};
  };

  int64_t nextStampLocked();   // shadow_mtx_
  void enqueueUpdatesLocked(std::vector<Item> && items);   // shadow_mtx_ 를 쥔 채 호출
  void requestFullQueueLocked(FullReason reason);          // queue_mtx_ 를 쥔 채 호출
  void fillMetadata(nav2_msgs::msg::Costmap & out, int64_t stamp) const;          // shadow_mtx_
  void scheduleRepairs(int64_t now_ns);                    // catchup_mtx_ 를 쥐고 부른다
  void pushRepairsLocked(const std::vector<Rect> & rects, RepairSource src);   // catchup_mtx_
  void clearRepairStateLocked();                           // catchup_mtx_
  bool popFull(OutMsg & out);
  bool popRepair(OutMsg & out);
  void copyRowsLocked(uint8_t * dst, uint32_t y0, uint32_t y1) const;   // shadow_mtx_
  uint32_t chunkRows(uint32_t width) const;
  void noteHold(double ms) const;
  // [10-10 W1-D][리뷰] std::mutex 는 공정하지 않다 — 묶음 사이에 놓았다가 곧바로 다시 잡으면 기다리던 map thread 가
  //   계속 밀린다 (시험에서 1.7 s). map thread 는 shadow lock 을 잡기 전에 map_waiting_ 을 올리고, 묶음 복사 쪽은
  //   다시 잡기 전에 그것이 0 이 될 때까지 (최대 20 ms) 양보한다.
  std::unique_lock<std::mutex> lockShadowForMap();
  void yieldToMap() const;
  void addStats(const std::function<void(EngineStats &)> & f) const;

  Config cfg_;
  StampFn stamp_fn_;

  // ---- shadow (writer = map thread 만; reader 는 shadow_mtx_ 아래) ----
  mutable std::mutex shadow_mtx_;
  std::vector<uint8_t> shadow_;
  Geometry geom_;
  uint64_t geom_gen_{0};                     // geometry 가 바뀔 때마다 +1 (묶음 복사 중단 판정)
  std::atomic<uint32_t> geom_w_{0}, geom_h_{0};   // lock 없이 읽는 크기 (할당·sweep 계획용)
  std::atomic<int> map_waiting_{0};          // shadow lock 을 기다리는 map thread (0/1)
  int64_t last_stamp_{0};
  std::vector<uint8_t> mask_;                // map thread 전용 scratch
  std::vector<Rect> tile_box_;               // map thread 전용 scratch (dirty tile 의 tight box)
  std::atomic<bool> reset_pending_{false};
  uint64_t cycle_count_{0};                  // map thread 전용

  // ---- queue ----
  std::atomic<bool> stopping_{false};
  mutable std::mutex queue_mtx_;
  std::condition_variable queue_cv_;
  std::deque<Item> queue_;
  size_t queued_update_bytes_{0};

  // ---- catch-up / history / repair (catchup_mtx_) ----
  mutable std::mutex catchup_mtx_;
  std::deque<Rect> history_;                 // 마지막 history_depth 개 publish 된 update 의 사각형
  bool catchup_tracking_{false};             // FULL 을 만든 뒤 catch-up 대상 수집 중 (마지막 단계까지)
  std::vector<Rect> catchup_rects_;
  size_t catchup_stage_{0};                  // 다음에 보낼 catch-up 단계
  int64_t catchup_base_ns_{0};               // FULL ack 시각 (steady ns), 0 = 아직 publish 안 됨
  std::deque<RepairRect> repair_;            // 보낼 repair 사각형 (앞에서부터, 큰 것은 band 로 쪼개 보냄)
  std::atomic<size_t> repair_n_{0};
  std::vector<Rect> refresh_cur_;            // 이번 refresh 주기에 보낸 update 사각형
  std::deque<std::vector<Rect>> refresh_hist_;   // 직전 주기들
  int64_t refresh_next_ns_{0};
  uint32_t sweep_row_{0};
  int64_t sweep_next_ns_{0};
  std::atomic<bool> have_subscribers_{true};

  // ---- stats ----
  mutable std::mutex stats_mtx_;
  mutable EngineStats stats_;
};

}  // namespace ammr_costmap_diff_publisher

#endif  // AMMR_COSTMAP_DIFF_PUBLISHER__DIFF_ENGINE_HPP_
