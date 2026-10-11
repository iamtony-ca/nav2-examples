// [10-10 W1] Nav2 costmap 의 로컬 mirror.
//
//   Nav2 Costmap2DPublisher 는 always_send_full_costmap: False 이면 full grid(nav2_msgs/Costmap)를
//   시작·크기/해상도/origin 변경 때만 보내고, 그 밖에는 바뀐 사각형만 nav2_msgs/CostmapUpdate 로
//   <topic>_updates 에 보낸다. 이 클래스는 둘을 받아 grid 하나를 유지한다.
//
//   - applyFull()  : 새 grid 를 lock 밖에서 만들고(memcpy + replay) exclusive lock 아래에서 swap 만 한다.
//   - applyUpdate(): grid 안에 들어오는 사각형만 exclusive lock 아래에서 행 단위로 복사한다.
//                    grid 가 없거나 범위·크기가 맞지 않으면 버리고 센다 (replay 용으로는 보관).
//   - read()       : ReadGuard 를 돌려준다. guard 가 살아 있는 동안 shared lock 을 쥐고 있으므로
//                    reader 는 guard 수명 안에서만 grid 메모리를 읽어야 한다 (포인터를 밖으로 빼지 말 것).
//
//   lock 순서 (mirror 내부): write_mtx_ → gate_mtx_ → grid_mtx_ → stats_mtx_.
//   reader 는 gate_mtx_ 를 잠깐 지나 grid_mtx_(shared) 를 쥔다. writer 는 gate_mtx_ 를 쥔 채 exclusive 를
//   기다리므로 그동안 새 reader 가 끼어들지 못한다 (writer 우선). glibc 의 std::shared_mutex 는 reader 우선이라
//   reader 가 끊임없이 겹치면 writer 가 굶는다 — 단위 테스트에서 reader 4 개로 1 s 동안 update 2 건만 들어간 것을 확인.
//   같은 thread 에서 read() 를 겹쳐 부르지 말 것 (writer 가 gate 를 쥐고 기다리면 교착, 재귀 shared lock 도 UB).
//
//   ROS 의존은 nav2_msgs / nav2_costmap_2d 뿐이라 agent mask 등 다른 grid 에도 그대로 쓸 수 있다.
//
//   [10-10 W1-2][리뷰] self_contained_updates (agent mask 전용):
//     W1-2 AgentLayer 는 "0 이 아닌 셀 ⊆ 이번 update 사각형" 을 지킨다 (multi_agent_nav2/agent_mask_canvas.hpp).
//     그러면 update 하나가 그 시점의 mask 전체다: 사각형 밖은 모두 0 이다. 이 모드에서 applyUpdate 는 직전에 적용한
//     update 의 사각형(full 뒤 첫 update 면 grid 의 0 아닌 셀 bbox)을 0 으로 만든 뒤 새 사각형을 복사한다.
//     - 중간 update 를 몇 개 잃어도 (DDS 가 알리든 말든) 다음 update 하나로 mirror == 실제 mask 가 된다.
//     - 그래서 replay 는 가장 새 update 하나만 두고, 밀려난 update 를 "잃은 것" 으로 세지 않는다 (needs_resync 없음).
//     - 같은 geometry 에서 마지막으로 적용한 update 보다 오래된 full 은 kRejectedSuperseded 로 버린다 (지금 grid 가 더 새
//       상태다. resync 를 다시 걸 이유가 아니다).
//     - applyFull(..., geometry_only=true) 는 내용을 0 으로 둔다 (오래된 latched full: 그때 칠한 몸체를 살아 있는
//       agent 로 내보내지 않는다. 빠진 셀은 다음 update 가 채운다 — 있는 agent 는 매 update 사각형 안에 있다).
//     일반 costmap(global)은 이 성질이 없으므로 이 모드를 쓰면 안 된다.

#ifndef REPLAN_MONITOR__COSTMAP_MIRROR_HPP_
#define REPLAN_MONITOR__COSTMAP_MIRROR_HPP_

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <shared_mutex>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/msg/costmap_update.hpp"

namespace replan_monitor
{

// [10-10 W1] path_validator_node.hpp 에서 옮겨 왔다 (agent mask 쪽도 계속 이 타입을 쓴다).
struct CostmapSignature
{
  unsigned int size_x{0};
  unsigned int size_y{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};

  bool operator==(const CostmapSignature & other) const
  {
    return size_x == other.size_x && size_y == other.size_y &&
           resolution == other.resolution &&
           origin_x == other.origin_x && origin_y == other.origin_y;
  }

  // [10-10 W1] topic full(origin = mapToWorld(0,0) - res/2)과 get_costmap 응답(origin = getOriginX())은
  //   같은 grid 라도 origin 이 1 ulp 다를 수 있다. 그 차이로 obstacle_db_ 를 지우지 않도록 허용 오차로 비교한다.
  bool nearlyEquals(const CostmapSignature & other, double origin_tol, double res_tol) const;

  static CostmapSignature fromMsg(const nav2_msgs::msg::Costmap & msg);
};

class CostmapMirror
{
public:
  struct Options
  {
    // full 을 받은 뒤 다시 적용할 update 보관 한도 (full 보다 늦게 찍힌 update 만 다시 적용한다).
    size_t replay_max_msgs{64};
    size_t replay_max_bytes{64u * 1024u * 1024u};
    // 같은 geometry 의 full 이 또 오면 직전 grid 메모리를 재사용한다 (할당·memset 생략, 대신 grid 2 장 상주).
    // [10-10 W1][리뷰] full 이 spare_keep_window_sec 간격 안으로 3 번 이상 연달아 올 때만 (always_send_full_costmap:
    //   True 처럼) 남긴다. 시작 때의 topic full + get_costmap 응답 2 번으로는 남기지 않는다. 남긴 spare 도
    //   window 동안 full 이 없으면 applyUpdate()/releaseIdleSpare() 에서 해제한다 (False 에서 grid 한 장 유지).
    bool keep_spare_buffer{true};
    double spare_keep_window_sec{10.0};
    double origin_tol_m{1e-6};
    double resolution_tol{1e-9};
    // [10-10 W1-2][리뷰] update 하나가 grid 전체 상태를 정한다 (머리 주석). agent mask 전용.
    bool self_contained_updates{false};
  };

  enum class FullResult
  {
    kApplied,              // 적용, geometry 같음
    kAppliedNewSignature,  // 적용, 첫 grid 이거나 geometry(size/res/origin) 바뀜
    kRejectedBadSize,      // data.size() != size_x*size_y — 버림, 기존 grid 유지
    kRejectedStale,        // [10-10 W1][리뷰] 현재 grid 의 full 보다 오래됨 — 출처(topic/service) 무관하게 버림
    // [10-10 W1-2][리뷰] self_contained 전용: 같은 geometry 에서 이미 적용한 update 보다 오래됨 — 버림 (resync 불필요)
    kRejectedSuperseded,
    // [10-10 W1-4][리뷰] size_x==0 || size_y==0 || resolution<=0 — "아직 데이터 없음" 응답 (DiffPublisherLayer 는 첫 cycle 전·
    //   snapshot 실패·enabled:false 때 get_costmap 에 기본값 Costmap() 을 돌려준다). 0x0 은 data.size()==0 으로 크기 검사를
    //   통과하므로 따로 거른다. grid·baseline_pending·full generation·replay 는 그대로 둔다.
    kRejectedEmpty,
  };

  // [10-10 W1][리뷰] applyFull 결과. result 외에 node 가 resync 를 다시 걸어야 하는지 알려준다.
  struct FullOutcome
  {
    FullResult result{FullResult::kRejectedBadSize};
    // stamp 는 현재 full 보다 오래됐지만 "현재 grid 를 받은 뒤 보낸 get_costmap 요청의 응답" 이라 Nav2 clock 이
    // 거꾸로 간 것(sim 재시작 등)으로 보고 적용했다. 옛 epoch 의 replay 는 버렸고, 그 사이 stale 로 버려진
    // 새 epoch update 가 있을 수 있으므로 needs_resync 도 함께 선다.
    bool clock_reset{false};
    // 이 full 보다 늦게 찍힌 update 중 replay 보관을 못 한 것(크기 초과·밀려남)이 있다 → 그 내용이 빠졌으니
    // get_costmap 으로 다시 받아야 한다.
    bool needs_resync{false};
    uint64_t replayed{0};

    bool applied() const
    {
      return result == FullResult::kApplied || result == FullResult::kAppliedNewSignature;
    }
  };

  // [10-10 W1][리뷰] applyFull 의 requested_at_gen 기본값: "요청 시점 정보 없음" (topic full).
  static constexpr uint64_t kNoRequestGen = std::numeric_limits<uint64_t>::max();

  enum class UpdateResult
  {
    kApplied,
    kNoGrid,               // 아직 full 이 없다 — replay 용으로 보관만
    kStale,                // 현재 full 보다 오래된 stamp — 이미 full 에 반영된 내용이라 버림
    kRejectedBadSize,      // data.size() != size_x*size_y — 버림 (dropped_updates)
    kRejectedOutOfBounds,  // 사각형이 grid 밖 — 버림 (dropped_updates), replay 용으로는 보관
  };

  struct Stats
  {
    bool has_grid{false};
    CostmapSignature sig;
    uint64_t fulls{0};
    uint64_t rejected_fulls{0};
    uint64_t stale_fulls{0};
    uint64_t updates{0};
    uint64_t dropped_updates{0};
    uint64_t stale_updates{0};
    uint64_t early_updates{0};
    uint64_t replayed_updates{0};
    uint64_t unreplayable_updates{0};   // [10-10 W1][리뷰] replay 보관 못 한 update (크기 초과 + 밀려남)
    uint64_t clock_resets{0};           // [10-10 W1][리뷰]
    uint64_t spare_releases{0};         // [10-10 W1][리뷰] idle spare 해제 횟수
    // [10-10 W1-2][리뷰] unreplayable_updates 는 replay 한도에서 밀려난 update 수라 정상 동작에서도 계속 는다.
    //   실제로 문제가 된 횟수(= 그 때문에 full 적용 뒤 needs_resync 를 세운 횟수)는 이것이다.
    uint64_t replay_loss_resyncs{0};
    uint64_t superseded_fulls{0};       // [10-10 W1-2][리뷰] kRejectedSuperseded 횟수
    uint64_t empty_fulls{0};            // [10-10 W1-4][리뷰] kRejectedEmpty 횟수
    uint64_t geometry_only_fulls{0};    // [10-10 W1-2][리뷰] geometry_only 로 적용한 full 수
    uint64_t sc_bbox_scans{0};          // [10-10 W1-2][리뷰] full 뒤 첫 update 에서 grid bbox 를 훑은 횟수
    uint64_t seeds{0};                  // [10-10 W1-3] seedZero 로 만든 grid 수
    bool content_known{false};          // [10-10 W1-3] contentKnown() 과 같다 (시험·로그용 사본)
    uint64_t source_resets{0};          // [10-10 W1-4] beginNewSource 횟수
    bool baseline_pending{false};       // [10-10 W1-4] source 전환 뒤 새 source 의 full 을 아직 못 받았다
    int64_t last_full_stamp_ns{0};
    int64_t last_update_stamp_ns{0};
    std::chrono::steady_clock::time_point last_full_time{};
    std::chrono::steady_clock::time_point last_update_time{};
  };

  // shared lock 을 쥔 채 grid 를 읽게 해 주는 guard. move 만 된다.
  class ReadGuard
  {
public:
    // move 된 쪽은 lock 도 포인터도 비운다 (빈 guard 로 grid 를 읽는 일이 없도록).
    ReadGuard(ReadGuard && o) noexcept
    : lock_(std::move(o.lock_)), cm_(o.cm_) {o.cm_ = nullptr;}
    ReadGuard & operator=(ReadGuard && o) noexcept
    {
      if (this != &o) {
        lock_ = std::move(o.lock_);
        cm_ = o.cm_;
        o.cm_ = nullptr;
      }
      return *this;
    }
    ReadGuard(const ReadGuard &) = delete;
    ReadGuard & operator=(const ReadGuard &) = delete;

    explicit operator bool() const noexcept {return cm_ != nullptr;}
    // 읽기가 끝났으면 수명 전에 lock 을 놓는다 (이후 get() 은 nullptr).
    void reset() noexcept
    {
      if (lock_.owns_lock()) {lock_.unlock();}
      cm_ = nullptr;
    }
    const nav2_costmap_2d::Costmap2D * get() const noexcept {return cm_;}
    const nav2_costmap_2d::Costmap2D * operator->() const noexcept {return cm_;}
    const nav2_costmap_2d::Costmap2D & operator*() const noexcept {return *cm_;}

private:
    friend class CostmapMirror;
    ReadGuard(std::shared_lock<std::shared_mutex> && lk, const nav2_costmap_2d::Costmap2D * cm)
    : lock_(std::move(lk)), cm_(cm) {}
    std::shared_lock<std::shared_mutex> lock_;
    const nav2_costmap_2d::Costmap2D * cm_{nullptr};
  };

  CostmapMirror();
  explicit CostmapMirror(const Options & opts);

  CostmapMirror(const CostmapMirror &) = delete;
  CostmapMirror & operator=(const CostmapMirror &) = delete;

  // writer. 내부 write_mtx_ 로 서로 직렬화된다 (여러 thread 에서 불러도 안전).
  // [10-10 W1][리뷰] 현재 grid 의 full 보다 오래된 full 은 출처와 무관하게 kRejectedStale 로 버린다 (grid·replay 그대로).
  //   always_send_full_costmap: False 에서 늦게 붙으면 latched topic full 은 activation 때 것이라, get_costmap 응답보다
  //   늦게 도착해도 덮어쓰면 안 된다. 예외는 clock reset 하나: requested_at_gen 에 get_costmap 요청을 보낼 때의
  //   fullGeneration() 을 넘기면, 그 뒤 다른 full 이 적용되지 않은 상태에서 온 오래된 응답은 (요청 뒤에 만든
  //   스냅샷이므로) clock 이 거꾸로 간 것으로 보고 적용한다 (FullOutcome::clock_reset).
  // [10-10 W1-2][리뷰] geometry_only=true 면 msg 의 geometry 만 쓰고 내용은 0 으로 둔다 (self_contained 에서만 뜻이 있다).
  FullOutcome applyFull(
    const nav2_msgs::msg::Costmap & msg, uint64_t requested_at_gen = kNoRequestGen,
    bool geometry_only = false);
  UpdateResult applyUpdate(const nav2_msgs::msg::CostmapUpdate::ConstSharedPtr & msg);

  // [10-10 W1-3] (self_contained 전용) grid 가 없으면 주어진 geometry 로 0 grid 를 만든다 (W2PV-1). global costmap mirror 의
  //   geometry 를 넘긴다: AgentLayer 의 mask grid 는 master 와 같은 크기·해상도·원점이다 (agent_mask_canvas beginCycle).
  //   먼저 와서 보관된 update 가 있으면 가장 새 것 하나를 적용한다 (self_contained 라 그것이 mask 전체). grid 가 이미 있거나
  //   self_contained 가 아니면 아무것도 하지 않는다 (seeded=false). seed 한 grid 는 stamp 기준이 없으므로 이후 모든
  //   update 와 full 을 받아들인다 (full 은 이미 적용한 update 보다 오래되면 kRejectedSuperseded 그대로).
  struct SeedOutcome
  {
    bool seeded{false};
    uint64_t replayed{0};
  };
  SeedOutcome seedZero(const CostmapSignature & sig);
  // [10-10 W1-3] 지금 grid 내용에 근거가 있는가: update 를 하나라도 적용했거나, 내용이 있는 full 을 적용했다.
  //   seedZero / geometry_only full 로 만든 0 grid 는 (replay 한 update 가 없으면) false. 일반 mirror 는 full 뒤 true.
  bool contentKnown() const;

  // [10-10 W1-4] global source 전환 (diff layer ↔ stock). 두 publisher 의 stamp 는 같은 Nav2 clock 이지만 서로의 순서는
  //   보장되지 않는다 (diff layer 는 직전 + 1 ns 로 밀기도 하고, latched FULL 은 오래된 것일 수 있다). 그래서 옛 source 의
  //   stamp 기준·replay·잃은 update 기록을 버리고 "새 source 의 첫 full 을 기다리는" 상태로 둔다:
  //     - grid 는 **그대로 둔다** (reader 는 전환 중에도 마지막 grid 로 static 판정을 계속한다 — grid 를 비우면
  //       validatePath 가 static 검사를 건너뛰고 READY 는 보류된다). 같은 global costmap 이라 geometry 도 같다.
  //     - 그동안 온 update 는 grid 에 쓰지 않고 replay 에 보관만 한다 (UpdateResult::kNoGrid — 새 source 의 full 이 기준).
  //     - 새 source 의 첫 full (topic 이든 get_costmap 응답이든) 은 stamp 와 상관없이 적용하고 보관한 update 중 그보다 새것을
  //       다시 적용한다. 그 뒤로는 예전 규칙 그대로 (오래된 full·update 는 버림).
  //   write_mtx_ 아래에서 바꾸므로 다른 writer 와 겹치지 않고, grid lock 은 잡지 않는다 (reader 를 막지 않는다).
  //   full generation 을 올린다 (전환 전에 보낸 get_costmap 응답이 혹시 오더라도 "현재 grid 뒤 요청" 으로 보지 않게).
  void beginNewSource();
  bool baselinePending() const;

  // [10-10 W1][리뷰] 적용된 full 수 (grid 내용의 기준이 바뀐 횟수). get_costmap 요청 때 기록해 applyFull 에 넘긴다.
  uint64_t fullGeneration() const;
  // [10-10 W1][리뷰] spare_keep_window_sec 동안 full 이 없었으면 spare 를 해제한다 (node timer 에서 주기 호출).
  void releaseIdleSpare();
  bool hasSpare() const;   // [10-10 W1][리뷰] 시험·진단용

  // reader. grid 가 없으면 lock 을 쥐지 않은 빈 guard 를 돌려준다.
  ReadGuard read() const;

  bool hasGrid() const;
  Stats stats() const;
  void setKeepSpareBuffer(bool keep);
  // [10-10 W1-2][리뷰] node 가 param 을 읽은 뒤 (첫 메시지 전에) 설정한다.
  void setSelfContainedUpdates(bool on);
  bool selfContained() const;
  void setReplayMaxMsgs(size_t n);

private:
  static int64_t stampNs(const builtin_interfaces::msg::Time & t);
  static bool updateFits(const nav2_costmap_2d::Costmap2D & cm, const nav2_msgs::msg::CostmapUpdate & u);
  static void copyRect(nav2_costmap_2d::Costmap2D & cm, const nav2_msgs::msg::CostmapUpdate & u);
  void bufferForReplay(const nav2_msgs::msg::CostmapUpdate::ConstSharedPtr & msg);  // write_mtx_ 필요
  void pruneReplay(int64_t older_than_ns);                                          // write_mtx_ 필요
  void releaseIdleSpareLocked(std::chrono::steady_clock::time_point now);           // write_mtx_ 필요

  Options opts_;

  mutable std::mutex write_mtx_;         // writer 직렬화. 아래 grid_/spare_/replay_ 의 쓰기 권한
  mutable std::mutex gate_mtx_;          // writer 우선용 관문 (writer 는 exclusive 구간 내내, reader 는 잠깐)
  mutable std::shared_mutex grid_mtx_;   // grid_ 포인터와 grid 메모리 보호
  std::unique_ptr<nav2_costmap_2d::Costmap2D> grid_;
  std::unique_ptr<nav2_costmap_2d::Costmap2D> spare_;  // writer 전용 (reader 는 못 본다)
  CostmapSignature sig_;                                // write_mtx_ 아래에서만 바뀐다
  int64_t last_full_stamp_ns_{0};                       // write_mtx_ 전용
  std::chrono::steady_clock::time_point last_full_wall_{};  // write_mtx_ 전용 (spare 유지 판정)
  int full_streak_{0};                                  // [10-10 W1][리뷰] window 간격 안으로 연달아 온 full 수 - 1
  std::atomic<uint64_t> full_gen_{0};                   // [10-10 W1][리뷰] 적용된 full 수 (쓰기는 write_mtx_ 아래)
  // [10-10 W1][리뷰] replay 보관을 못 한 update 중 가장 늦은 stamp. 이보다 오래되지 않은 full 을 적용하면
  //   그 update 내용이 빠지므로 needs_resync 를 세운다. write_mtx_ 전용.
  int64_t lost_replay_max_stamp_ns_{std::numeric_limits<int64_t>::min()};
  std::atomic<bool> has_grid_{false};
  std::atomic<bool> content_known_{false};              // [10-10 W1-3] (쓰기는 write_mtx_ 아래)
  bool baseline_pending_{false};                        // [10-10 W1-4] write_mtx_ 전용 (beginNewSource 주석)

  std::deque<nav2_msgs::msg::CostmapUpdate::ConstSharedPtr> replay_;  // write_mtx_ 전용
  size_t replay_bytes_{0};

  // [10-10 W1-2][리뷰] self_contained 상태 (write_mtx_ 전용)
  struct Rect
  {
    uint32_t x{0}, y{0}, w{0}, h{0};
    bool empty() const {return w == 0 || h == 0;}
  };
  static Rect nonzeroBBox(const nav2_costmap_2d::Costmap2D & cm);
  static void zeroRect(nav2_costmap_2d::Costmap2D & cm, const Rect & r);
  bool sc_prev_known_{false};   // false = full 뒤 아직 update 없음 (0 아닌 셀 범위를 모름)
  Rect sc_prev_;                // 직전에 적용한 update 사각형 (= 지금 0 아닌 셀을 모두 담는 사각형)
  int64_t sc_last_update_stamp_ns_{std::numeric_limits<int64_t>::min()};

  mutable std::mutex stats_mtx_;
  Stats stats_;
};

}  // namespace replan_monitor

#endif  // REPLAN_MONITOR__COSTMAP_MIRROR_HPP_
