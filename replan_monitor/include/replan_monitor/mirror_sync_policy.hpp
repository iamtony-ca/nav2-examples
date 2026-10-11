// [10-10 W1-3] path_validator 의 mirror 동기화·agent mask 판정 규칙을 ROS node 밖으로 뺀 것 (단위 시험용).
//
//   path_validator_node.cpp 의 MirrorSync 상태 기계와 판정 분기는 이 파일의 함수·클래스를 부른다. 여기에는 ROS 호출이
//   없고, 시각은 호출자가 넘긴다 (std::chrono::steady_clock). test/test_mirror_sync_policy.cpp 가 직접 시험한다.
//
//   들어 있는 것 (감사 id 는 verify/bigmap_1009/W1_SIDE_EFFECTS_{A,B}_1010.md):
//     - ResyncScheduler          : get_costmap 요청 일정. 진행 중에는 트리거를 올리지 않고 (W2PV-3, 내용 유실 트리거만
//                                  기억해 응답 뒤 한 번 더 — [리뷰]), timeout 은 경고만 하고 늦은 응답도 쓴다 (포기는
//                                  abandon_sec 뒤, 그 뒤 지수 backoff — R3), 요청·timeout·포기·실패 수를 센다.
//     - decideAgentMaskMode      : agent mask 를 "안다 / 모른다" 로 나눈다 (W2PV-1 (2)). 모르면 "비어 있음" 으로 보지 않는다.
//     - skipSelfFootprintCell    : [V2 M-6] 자기 발자국 안 lethal 셀을 건너뛸지 (mask 를 모르면 이웃 셀은 건너뛰지 않는다).
//     - decideMaskTopicFull      : 오래된 latched mask topic full 을 어떻게 쓸지 (W2PV-2).
//     - dropLatchedMaskUpdate    : 재구독 직후 받은 오래된 latched mask update 를 버릴지 (W2PV-1 (1)).
//     - evaluateGlobalWatchdog   : global costmap 무갱신·grid 없음·resync 연속 실패 (R4). mask 에는 쓰지 않는다.
//                                  [10-10 W1-4] diff layer 처럼 값이 바뀔 때만 보내는 source 는 "publisher 없음" (liveness)
//                                  으로 본다. stock source 는 예전처럼 침묵으로 본다.
//                                  [10-10 W1-4][리뷰] diff source 도 sweep heartbeat (1 s) 가 있으므로 heartbeat_sec 침묵은 본다.
//     - GlobalSourceSelector     : [10-10 W1-4] global mirror 의 source 선택 (diff layer → stock fallback 한 번, 복귀 한 번).

#ifndef REPLAN_MONITOR__MIRROR_SYNC_POLICY_HPP_
#define REPLAN_MONITOR__MIRROR_SYNC_POLICY_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>

namespace replan_monitor
{

// ============================================================================================
// get_costmap 요청 일정
// ============================================================================================
class ResyncScheduler
{
public:
  using Clock = std::chrono::steady_clock;

  struct Config
  {
    double min_interval_sec{5.0};   // 요청 사이 최소 간격
    // [10-10 W1-3][리뷰] timeout_sec 은 "늦음" 경고 기준이다. 넘어도 요청은 버리지 않고 응답을 기다린다 (R3: 서버는 버려진
    //   요청도 끝까지 200 MB 를 만들고, 응답 시간이 늘 timeout 을 넘으면 버리고 다시 보내는 방식으로는 영원히 못 받는다).
    double timeout_sec{30.0};
    double abandon_sec{300.0};      // [10-10 W1-3][리뷰] 이만큼 응답이 없으면 (또는 timeout 뒤 service 가 사라지면) 포기
    double backoff_base_sec{30.0};  // 연속 실패 1 회 뒤 기다릴 시간. 2 회 = 2 배, ... (상한 backoff_max_sec)
    double backoff_max_sec{120.0};
  };

  // [10-10 W1-3][리뷰] 트리거 이유. 진행 중 요청이 있을 때 무시해도 되는가가 다르다.
  enum class Trigger
  {
    kStructural,    // grid 없음·시작·update 먼저·범위 밖 (replay 에 보관) 등 — 진행 중 응답이 채운다 (W2PV-3: 무시)
    kContentLoss,   // MESSAGE_LOST·되살릴 수 없는 update — 진행 중 응답의 스냅샷보다 뒤 내용일 수 있다 → 응답 뒤 한 번 더
  };

  enum class Tick
  {
    kIdle,           // 할 일 없음
    kWaitInFlight,   // 응답 기다리는 중
    kSlow,           // [10-10 W1-3][리뷰] 방금 timeout_sec 을 넘었다 (호출자는 WARN 만, 요청은 그대로 기다린다)
    kTimedOut,       // 방금 포기했다 (abandon_sec 또는 service 사라짐. 호출자는 pending request 를 지우고 WARN). 다음 요청은 backoff 뒤
    kWaitInterval,   // min_interval 대기
    kWaitBackoff,    // 연속 실패 backoff 대기
    kWaitService,    // service 가 아직 없다
    kWaitOther,      // 다른 mirror 의 resync 가 먼저다 (global → mask 순서)
    kSend,           // 지금 보내라 (호출자는 보낸 뒤 onSent)
  };

  struct Counters
  {
    uint64_t triggers{0};            // 받아들인 트리거
    uint64_t ignored_in_flight{0};   // 요청 진행 중이라 무시한 트리거 (W2PV-3: 예전에는 응답 뒤 같은 요청을 한 번 더 보냈다)
    uint64_t loss_in_flight{0};      // [리뷰] 그중 내용 유실 트리거 (응답 뒤 한 번 더 받게 표시)
    uint64_t loss_retriggers{0};     // [리뷰] 그 표시 때문에 응답 뒤 다시 요청하게 된 횟수
    uint64_t requests{0};            // 보낸 요청
    uint64_t responses{0};           // 받은 응답 (쓸 수 있든 없든)
    uint64_t timeouts{0};            // timeout_sec 을 넘긴 요청 (늦음 — [리뷰] 버리지 않는다)
    uint64_t late_responses{0};      // [리뷰] timeout_sec 뒤에 도착한 응답 (예전이면 버려졌다)
    uint64_t abandoned{0};           // [리뷰] 포기한 요청 (abandon_sec / service 사라짐)
    uint64_t failures{0};            // 응답은 왔지만 쓸 수 없었던 것 (예외·크기 불일치)
    uint64_t source_resets{0};       // [10-10 W1-4] source 전환으로 상태를 비운 횟수
    uint64_t dropped_in_flight_on_reset{0};   // [10-10 W1-4] 그때 버린 진행 중 요청 수 (응답이 와도 쓰지 않는다)
    uint64_t not_ready{0};           // [10-10 W1-4][리뷰] "아직 데이터 없음" (빈 map) 응답 수 — 실패로 세지 않는다
  };

  ResyncScheduler() = default;
  explicit ResyncScheduler(const Config & c)
  : cfg_(c) {}
  void configure(const Config & c) {cfg_ = c;}
  const Config & config() const {return cfg_;}

  // 연속 실패 n 회 뒤의 대기 시간. n<=0 → 0. base·2^(n-1), 상한 cap.
  static double backoffSec(int consecutive_failures, double base, double cap)
  {
    if (consecutive_failures <= 0 || base <= 0.0) {
      return 0.0;
    }
    const int e = std::min(consecutive_failures - 1, 30);
    return std::min(std::max(cap, base), base * std::ldexp(1.0, e));
  }

  // resync 가 필요하다고 표시한다. 요청이 진행 중이면 무시한다 (false). 진행 중인 요청의 응답이 그 필요를 채우거나
  // (같은 grid 를 다시 받는 것뿐이다), 채우지 못하면 응답 처리(onFullApplied 의 needs_resync 등)가 다시 트리거한다.
  // [10-10 W1-3][리뷰] 단 kContentLoss 는 기억한다: 유실된 update 가 서버 스냅샷 뒤의 것이면 응답에도 replay 에도 없다.
  //   응답이 결론 나면 (onResponseDefinitive) 한 번 더 받는다 (min_interval·backoff 는 그대로).
  bool trigger(Trigger kind = Trigger::kStructural)
  {
    if (in_flight_) {
      ++cnt_.ignored_in_flight;
      if (kind == Trigger::kContentLoss) {
        dirty_during_flight_ = true;
        ++cnt_.loss_in_flight;
      }
      return false;
    }
    needed_ = true;
    ++trigger_gen_;
    ++cnt_.triggers;
    return true;
  }

  // 보내기 전에 필요가 사라졌을 때 (예: mask 를 global geometry 로 seed 했다). 진행 중인 요청은 그대로 둔다.
  void cancel()
  {
    if (!in_flight_) {
      needed_ = false;
    }
  }

  Tick tick(Clock::time_point now, bool service_ready, bool blocked_by_other)
  {
    if (in_flight_) {
      const double waited = secs(now - sent_at_);
      // [10-10 W1-3][리뷰] 포기: abandon_sec, 또는 timeout 이 지났는데 service 가 사라졌다 (서버가 죽으면 응답이 오지 않는다)
      if (waited >= std::max(cfg_.abandon_sec, cfg_.timeout_sec) ||
        (waited >= cfg_.timeout_sec && !service_ready))
      {
        in_flight_ = false;
        slow_marked_ = false;
        dirty_during_flight_ = false;   // needed 는 그대로라 backoff 뒤 다시 보낸다
        ++cnt_.abandoned;
        ++consecutive_failures_;
        next_allowed_ = now + toDur(backoffSec(consecutive_failures_, cfg_.backoff_base_sec, cfg_.backoff_max_sec));
        return Tick::kTimedOut;
      }
      if (waited >= cfg_.timeout_sec && !slow_marked_) {
        slow_marked_ = true;
        ++cnt_.timeouts;
        return Tick::kSlow;
      }
      return Tick::kWaitInFlight;
    }
    if (!needed_) {
      return Tick::kIdle;
    }
    if (consecutive_failures_ > 0 && now < next_allowed_) {
      return Tick::kWaitBackoff;
    }
    if (ever_sent_ && secs(now - last_sent_) < cfg_.min_interval_sec) {
      return Tick::kWaitInterval;
    }
    if (!service_ready) {
      return Tick::kWaitService;
    }
    if (blocked_by_other) {
      return Tick::kWaitOther;
    }
    return Tick::kSend;
  }

  void onSent(Clock::time_point now)
  {
    in_flight_ = true;
    ever_sent_ = true;
    slow_marked_ = false;
    dirty_during_flight_ = false;
    sent_at_ = now;
    last_sent_ = now;
    sent_gen_ = trigger_gen_;
    ++cnt_.requests;
  }

  // 응답이 도착했다 (내용 판정 전). 포기한 요청의 응답은 client 가 버리므로 여기 오지 않는다.
  // [10-10 W1-3][리뷰] timeout_sec 을 넘긴 (늦은) 응답도 여기 온다 — 버리지 않고 쓴다.
  void onResponseArrived()
  {
    in_flight_ = false;
    ++cnt_.responses;
    last_response_late_ = slow_marked_;
    if (slow_marked_) {
      ++cnt_.late_responses;
    }
    slow_marked_ = false;
  }
  // 응답으로 결론이 났다 (적용·더 오래돼 버림·superseded). 연속 실패를 지우고, 요청 뒤 새 트리거가 없었으면 필요를 내린다.
  // [10-10 W1-3][리뷰] 진행 중에 내용 유실 트리거가 있었으면 필요를 남긴다 (한 번 더 받는다).
  void onResponseDefinitive()
  {
    consecutive_failures_ = 0;
    if (dirty_during_flight_) {
      dirty_during_flight_ = false;
      needed_ = true;
      ++trigger_gen_;
      ++cnt_.loss_retriggers;
      return;
    }
    if (trigger_gen_ == sent_gen_) {
      needed_ = false;
    }
  }
  // 응답을 쓸 수 없었다 (예외·크기 불일치). 필요는 그대로, 다음 요청은 backoff 뒤.
  void onResponseFailed(Clock::time_point now)
  {
    dirty_during_flight_ = false;   // needed 가 남으므로 다시 받는다
    ++cnt_.failures;
    ++consecutive_failures_;
    next_allowed_ = now + toDur(backoffSec(consecutive_failures_, cfg_.backoff_base_sec, cfg_.backoff_max_sec));
  }

  // [10-10 W1-4][리뷰] 응답은 왔지만 서버에 아직 데이터가 없다 (빈 map — DiffPublisherLayer 의 첫 cycle 전·snapshot 실패·
  //   enabled:false). 실패가 아니라 "service 준비 안 됨" 이다: 필요는 그대로 두고, 연속 실패·backoff (30 s) 를 올리지 않는다.
  //   다음 요청은 min_interval 뒤 (onSent 의 last_sent_ 기준). 진행 중 유실 표시는 지운다 (needed 가 남으므로 다시 받는다).
  void onResponseNotReady()
  {
    dirty_during_flight_ = false;
    needed_ = true;
    ++cnt_.not_ready;
  }

  // [10-10 W1-4] global source 전환 (구독·client 를 새 topic/service 로 다시 만든다). 진행 중 요청은 호출자가 옛 client 에서
  //   지우고 (remove_pending_request) client 를 바꾼다 — 그 응답이 늦게 와도 node 의 source 세대 검사가 버리므로 여기서는
  //   "진행 중" 표시만 내린다 (onResponseArrived 가 다시 불리지 않는다). 옛 source 의 실패·backoff·간격·진행 중 유실 표시도
  //   새 source 와 무관하므로 지우고, needed 를 세워 새 source 로 바로 resync 한다. 누적 counter 는 그대로 둔다.
  //   돌려주는 값: 진행 중 요청이 있었다 (버렸다).
  bool resetForNewSource()
  {
    const bool had_in_flight = in_flight_;
    if (had_in_flight) {
      ++cnt_.dropped_in_flight_on_reset;
    }
    in_flight_ = false;
    slow_marked_ = false;
    dirty_during_flight_ = false;
    last_response_late_ = false;
    consecutive_failures_ = 0;
    next_allowed_ = Clock::time_point{};
    ever_sent_ = false;   // min_interval 을 기다리지 않는다 (옛 source 에 보낸 시각이다)
    needed_ = true;
    ++trigger_gen_;
    ++cnt_.triggers;
    ++cnt_.source_resets;
    return had_in_flight;
  }

  bool needed() const {return needed_;}
  bool inFlight() const {return in_flight_;}
  bool everSent() const {return ever_sent_;}
  bool dirtyDuringFlight() const {return dirty_during_flight_;}
  bool lastResponseLate() const {return last_response_late_;}
  int consecutiveFailures() const {return consecutive_failures_;}
  double backoffRemainingSec(Clock::time_point now) const
  {
    return (consecutive_failures_ > 0 && now < next_allowed_) ? secs(next_allowed_ - now) : 0.0;
  }
  // 직전 실패 뒤 다음 backoff (로그용)
  double currentBackoffSec() const
  {
    return backoffSec(consecutive_failures_, cfg_.backoff_base_sec, cfg_.backoff_max_sec);
  }
  Clock::time_point sentAt() const {return sent_at_;}
  const Counters & counters() const {return cnt_;}

private:
  static double secs(Clock::duration d) {return std::chrono::duration<double>(d).count();}
  static Clock::duration toDur(double s)
  {
    return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(s));
  }

  Config cfg_;
  bool needed_{false};
  bool in_flight_{false};
  bool ever_sent_{false};
  bool slow_marked_{false};          // [리뷰] 진행 중 요청이 timeout_sec 을 넘겼다 (WARN 한 번)
  bool dirty_during_flight_{false};  // [리뷰] 진행 중에 내용 유실 트리거가 있었다
  bool last_response_late_{false};
  uint64_t trigger_gen_{0};
  uint64_t sent_gen_{0};
  int consecutive_failures_{0};
  Clock::time_point sent_at_{};
  Clock::time_point last_sent_{};
  Clock::time_point next_allowed_{};
  Counters cnt_;
};

// ============================================================================================
// agent mask: 안다 / 모른다
// ============================================================================================
enum class AgentMaskMode
{
  kDisabled,   // compare_agent_mask=false (amhs 와 같은 whoCoversPoint 경로, M-6 은 무조건 건너뜀)
  kKnown,      // mask 를 믿는다 (amhs 와 같은 mask 경로)
  kUnknown,    // mask 를 모른다 → whoCoversPoint 로 귀속, 이웃이 덮는 셀은 M-6 으로 건너뛰지 않는다
};

// has_grid       : mirror 에 grid 가 있다 (geometry 를 안다)
// content_known  : 내용 근거가 있다 — update 를 하나라도 적용했거나, 내용이 있는 full (geometry_only 가 아닌) 을 적용했다
// neighbours     : /multi_agent_infos 캐시에 (자기 말고) 이웃이 있다
// grid 는 있는데 근거가 없는 0 grid (global geometry 로 seed, 또는 geometry_only full) 는 이웃이 없을 때만 믿는다
// (이웃이 없으면 0 이 맞다). 이웃이 있는데 update 가 아직 없으면 "모름" 이다 — "비어 있음" 으로 보면 이웃 차단이 STATIC
// 으로 분류되고 M-6 이 이웃 몸체를 건너뛴다 (W2PV-1).
inline AgentMaskMode decideAgentMaskMode(
  bool compare_enabled, bool has_grid, bool content_known, bool neighbours)
{
  if (!compare_enabled) {
    return AgentMaskMode::kDisabled;
  }
  if (!has_grid) {
    return AgentMaskMode::kUnknown;
  }
  if (content_known || !neighbours) {
    return AgentMaskMode::kKnown;
  }
  return AgentMaskMode::kUnknown;
}

// [V2 M-6] 자기 발자국(+여유) 안의 lethal 셀을 HIT 에서 뺄지.
//   kDisabled : amhs 와 같다 (compare_agent_mask=false 이면 무조건 뺀다).
//   kKnown    : amhs 와 같다 (mask 가 이웃이라고 하지 않을 때만 뺀다).
//   kUnknown  : whoCoversPoint 가 이웃이 덮는다고 하면 빼지 않는다 (S8: 대면 중 이웃 HIT 가 사라지면 resume → 충돌).
inline bool skipSelfFootprintCell(
  AgentMaskMode mode, bool in_self_poly, bool mask_marks_agent, bool covered_by_neighbour)
{
  if (!in_self_poly) {
    return false;
  }
  switch (mode) {
    case AgentMaskMode::kDisabled:
      return true;
    case AgentMaskMode::kKnown:
      return !mask_marks_agent;
    case AgentMaskMode::kUnknown:
    default:
      return !covered_by_neighbour;
  }
}

// HIT 를 누구에게 귀속시킬지: mask 를 알 때만 mask 경로, 아니면 whoCoversPoint (compare_agent_mask=false 경로).
inline bool attributeWithMask(AgentMaskMode mode) {return mode == AgentMaskMode::kKnown;}

// 목표 점유 검사(READY·Phase 1.5)에서 mask 로 이웃 점유를 볼지 / mask 를 몰라서 정적 점유를 이웃 쪽으로 다시 볼지.
inline bool useMaskForGoalCheck(AgentMaskMode mode) {return mode == AgentMaskMode::kKnown;}
inline bool fallbackGoalAttribution(AgentMaskMode mode, bool static_blocked)
{
  return mode == AgentMaskMode::kUnknown && static_blocked;
}

// [10-10 W1-3][리뷰] READY 출발 판단: global grid 가 없으면 목표 정적 점유를 볼 수 없고 (isGoalBlocked(nullptr) = false),
//   mask 도 seed 할 수 없어 "모름" 이다. 그때 SAFE (is_status_ready=true) 를 내면 navigation_manager 가 점유된 목표로 출발한다
//   (G=False 에서 PV (재)시작 뒤 global get_costmap 응답까지 16~35 s). hold_enabled 면 보류한다.
inline bool readyHoldWithoutGlobalGrid(bool hold_enabled, bool has_global_grid)
{
  return hold_enabled && !has_global_grid;
}

// ============================================================================================
// mask topic full / latched update 처리 (self-contained mask 전용)
// ============================================================================================
enum class MaskTopicFullAction
{
  kApplyContent,        // 내용째 적용 (mirror 의 stale/superseded 규칙은 그대로)
  kApplyGeometryOnly,   // geometry 만 쓰고 내용은 0
  kDropOld,             // 버린다 (이미 같은 geometry 의 grid 가 있다 — 0 으로 만들지 않는다)
};

// W2PV-2: geometry_only 는 (재)구독 뒤 첫 full 이고, update 를 한 번도 받지 않았고, start_grace 보다 오래됐을 때만.
//   - full 이 이어서 오면 (예전 AgentLayer / full_mask_publish:true) 두 번째부터는 내용째 쓴다 (update 가 없는 publisher 다).
//   - update 를 이미 받았으면 내용째 넘긴다: 그 update 보다 오래된 full 은 mirror 가 kRejectedSuperseded 로 버린다.
//   - 같은 geometry 의 grid 가 이미 있으면 (seed 한 0 grid 등) 0 으로 다시 만들지 않고 버린다.
inline MaskTopicFullAction decideMaskTopicFull(
  bool self_contained, bool first_full_since_subscribe, bool any_update_seen,
  double full_age_sec, double start_grace_sec, bool has_grid_same_geometry)
{
  if (!self_contained || full_age_sec <= start_grace_sec || !first_full_since_subscribe ||
    any_update_seen)
  {
    return MaskTopicFullAction::kApplyContent;
  }
  return has_grid_same_geometry ? MaskTopicFullAction::kDropOld : MaskTopicFullAction::kApplyGeometryOnly;
}

// W2PV-1 (1): 재구독(PV 시작·respawn) 직후 받은 latched update 가 이 구독보다 먼저 찍혔거나 start_grace 보다 오래됐으면 버린다.
//   AgentLayer 는 구독자가 없으면 update 를 만들지 않으므로, PV 가 죽어 있던 동안의 마지막 update (옛 이웃 몸체) 가 latch 돼
//   있을 수 있다. 그 뒤로 칠할 것이 없으면 다음 update 가 오지 않아 유령 셀이 남는다. 0 grid 가 맞다 (이웃이 있으면 곧 새
//   update 가 온다). 처음으로 받아들인 update 이후, 또는 구독 뒤 window_sec 이 지나면 거르지 않는다 (node·Nav2 clock 이 달라
//   모든 update 가 오래돼 보이는 경우에 mask 가 영원히 멈추지 않게).
// [10-10 W1-3][리뷰] 나이만 보면 respawn_delay(5 s)·기동이 빠를 때 latched update 의 나이가 5 s 안쪽이 될 수 있다 (유령 영구).
//   그래서 출처로 먼저 본다: stamped_before_subscribe = update stamp < (이 구독 시각 − 여유). 나이 규칙은 clock 종류가 달라
//   구독 시각을 쓸 수 없을 때를 위한 보조로 남긴다.
inline bool dropLatchedMaskUpdate(
  bool self_contained, bool fresh_update_seen, double since_subscribe_sec, double window_sec,
  double update_age_sec, double start_grace_sec, bool stamped_before_subscribe = false)
{
  return self_contained && !fresh_update_seen && since_subscribe_sec < window_sec &&
         (stamped_before_subscribe || update_age_sec > start_grace_sec);
}

// ============================================================================================
// global costmap watchdog (R4). agent mask 에는 쓰지 않는다 (이웃이 없으면 mask 는 조용한 것이 정상).
// ============================================================================================
enum GlobalWatchdogFlag : unsigned
{
  kWatchdogOk = 0u,
  kWatchdogNoGrid = 1u,          // 시작 뒤 no_grid_sec 이 지나도 grid 가 없다
  kWatchdogSilent = 2u,          // grid 는 있는데 full/update 를 stale_sec 동안 받지 못했다
  kWatchdogResyncFailing = 4u,   // get_costmap 이 연속 fail_threshold 번 실패 (timeout 포함)
  kWatchdogNoPublisher = 8u,     // [10-10 W1-4] (liveness 기준) 지금 source 의 full topic 에 publisher 가 없다
};

struct GlobalWatchdogInput
{
  bool has_grid{false};
  double since_start_sec{0.0};
  bool any_rx{false};              // full/update 를 하나라도 받았다
  double since_last_rx_sec{0.0};   // 마지막 full/update 수신 뒤 시간 (any_rx 일 때만 뜻이 있다)
  int resync_consecutive_failures{0};
  double stale_sec{10.0};          // 0 이하 = watchdog 끔
  double no_grid_sec{50.0};
  int fail_threshold{2};
  // [10-10 W1-4] source 별 기준. liveness_mode = 지금 source 가 값이 바뀔 때만 보내는 diff layer 다.
  //   그러면 무갱신 (kWatchdogSilent) 은 stale_sec 이 아니라 heartbeat_sec 으로 보고 ([리뷰], 아래), full topic 의 publisher 유무도 본다:
  //     - publisher 를 본 적이 있으면: 사라진 뒤 stale_sec 이 지나면 kWatchdogNoPublisher
  //     - 본 적이 없으면: source 시작 뒤 no_grid_sec 이 지나면 (Nav2 가 늦게 뜨는 시작·fallback 판정 시간을 넘긴 뒤)
  //   liveness_mode=false (stock Costmap2DPublisher) 는 W1-3 그대로 무갱신으로 본다 (footprint_clearing 때문에 주기 update 가 온다).
  bool liveness_mode{false};
  bool publisher_present{false};          // 지금 full topic publisher 수 > 0
  bool publisher_ever_seen{false};        // 이 source 에서 publisher 를 본 적이 있다
  double since_publisher_seen_sec{0.0};   // 마지막으로 publisher 를 본 뒤 시간 (ever_seen 일 때만)
  double since_source_sec{-1.0};          // 지금 source 로 바꾼 뒤 시간. 음수 = since_start_sec 를 쓴다 (전환 없음)
  // [10-10 W1-4][리뷰] liveness_mode 의 heartbeat (0 이하 = 끔). diff layer 는 update 구독자가 있으면 publisher thread 가
  //   sweep_tick_sec (1 s) 마다 sweep band 하나를 보낸다 (DiffEngine::pop → scheduleRepairs, costmap cycle 과 무관). 그래서
  //   "값이 바뀔 때만" 이라도 PV 는 1 s 에 한 번은 받는다. publisher 는 살아 있는데 아무것도 안 보내는 경우 (FULL 버퍼 할당이
  //   계속 실패해 FULL token 이 큐 맨 앞에 남음 — F6, publisher thread 정지) 를 이것으로 잡는다 → kWatchdogSilent.
  //   layer 의 sweep 을 끈 설정 (sweep_period_sec 0) 이면 0 으로 끌 것.
  double heartbeat_sec{0.0};
};

inline unsigned evaluateGlobalWatchdog(const GlobalWatchdogInput & in)
{
  if (in.stale_sec <= 0.0) {
    return kWatchdogOk;
  }
  const double since_src = in.since_source_sec >= 0.0 ? in.since_source_sec : in.since_start_sec;
  unsigned f = kWatchdogOk;
  if (!in.has_grid) {
    if (in.since_start_sec >= in.no_grid_sec) {
      f |= kWatchdogNoGrid;
    }
  } else {
    // grid 는 있는데 이 source 에서 한 번도 받은 적이 없다 = get_costmap 으로만 받았다 → source 시작 시각부터 센다
    const double silent = in.any_rx ? in.since_last_rx_sec : since_src;
    // [10-10 W1-4][리뷰] liveness_mode 도 침묵을 본다 (기준 = heartbeat_sec, 0 이하면 예전처럼 보지 않음).
    const double limit = in.liveness_mode ? in.heartbeat_sec : in.stale_sec;
    if (limit > 0.0 && silent >= limit) {
      f |= kWatchdogSilent;
    }
  }
  if (in.liveness_mode && !in.publisher_present) {
    const bool gone = in.publisher_ever_seen ?
      (in.since_publisher_seen_sec >= in.stale_sec) : (since_src >= in.no_grid_sec);
    if (gone) {
      f |= kWatchdogNoPublisher;
    }
  }
  if (in.fail_threshold > 0 && in.resync_consecutive_failures >= in.fail_threshold) {
    f |= kWatchdogResyncFailing;
  }
  return f;
}

// ============================================================================================
// [10-10 W1-4] global costmap mirror 의 source 선택
// ============================================================================================
//   primary  = DiffPublisherLayer (/global_costmap/diff_publisher/costmap_raw + _updates, .../get_costmap). 값이 바뀔 때만 보낸다.
//   fallback = stock Costmap2DPublisher (/global_costmap/costmap_raw + _updates, /global_costmap/get_costmap).
//   layer 가 없는 설정 (현장 params 에 plugin 미등록, overlay 없는 sim) 이면 primary topic 에 publisher 가 영원히 없다.
//
//   fallback: "primary publisher 0 개 && fallback publisher ≥ 1 개" 가 fallback_sec 동안 **이어지면** 한 번 fallback 으로.
//     - 두 조건을 같이 보는 이유: planner_server 재기동 동안에는 둘 다 사라졌다가 거의 같이 (같은 노드) 돌아온다. primary 없음만
//       세면 재기동 동안 쌓인 시간 때문에 stock 이 먼저 보인 순간 바로 fallback 해 버린다.
//     - 시작 직후 (Nav2 가 아직 안 떴다) 에는 둘 다 0 이라 세지 않는다 → "시작 뒤 fallback_sec" 보다 늦게만 바뀐다.
//   복귀: fallback 중에 primary publisher 가 switch_back_confirm_sec 동안 이어서 보이면 **한 번** primary 로 돌아간다.
//     그 뒤에는 다시 바꾸지 않는다 (flapping 방지 — primary 가 다시 사라지면 watchdog 의 "publisher 없음" 이 신호다).
//   fallback 도 한 번뿐이다 (복귀 뒤 다시 fallback 하지 않는다).
class GlobalSourceSelector
{
public:
  using Clock = std::chrono::steady_clock;
  enum class Source {kPrimary, kFallback};
  enum class Action {kNone, kSwitchToFallback, kSwitchToPrimary};

  struct Config
  {
    bool enabled{true};                    // fallback topic/service 가 비었거나 primary 와 같으면 false
    double fallback_sec{15.0};             // 0 이하 = 끔
    double switch_back_confirm_sec{1.0};   // primary 가 이만큼 이어서 보이면 복귀
  };

  GlobalSourceSelector() = default;
  explicit GlobalSourceSelector(const Config & c)
  : cfg_(c) {}
  void configure(const Config & c) {cfg_ = c;}
  const Config & config() const {return cfg_;}
  bool enabled() const {return cfg_.enabled && cfg_.fallback_sec > 0.0;}

  // 500 ms 마다. primary_pubs / fallback_pubs = 각 full topic 의 publisher 수 (graph).
  Action tick(Clock::time_point now, int primary_pubs, int fallback_pubs)
  {
    if (!enabled()) {
      return Action::kNone;
    }
    if (active_ == Source::kPrimary) {
      if (fallen_back_) {
        return Action::kNone;   // 이미 한 번 fallback 했다가 돌아왔다 → 더 바꾸지 않는다
      }
      const bool cond = primary_pubs <= 0 && fallback_pubs > 0;
      if (!cond) {
        cond_active_ = false;
        return Action::kNone;
      }
      if (!cond_active_) {
        cond_active_ = true;
        cond_since_ = now;
      }
      if (secs(now - cond_since_) >= cfg_.fallback_sec) {
        active_ = Source::kFallback;
        fallen_back_ = true;
        cond_active_ = false;
        switched_at_ = now;
        return Action::kSwitchToFallback;
      }
      return Action::kNone;
    }
    // fallback 중
    if (switched_back_) {
      return Action::kNone;
    }
    if (primary_pubs <= 0) {
      back_active_ = false;
      return Action::kNone;
    }
    if (!back_active_) {
      back_active_ = true;
      back_since_ = now;
    }
    if (secs(now - back_since_) >= cfg_.switch_back_confirm_sec) {
      active_ = Source::kPrimary;
      switched_back_ = true;
      back_active_ = false;
      switched_at_ = now;
      return Action::kSwitchToPrimary;
    }
    return Action::kNone;
  }

  Source active() const {return active_;}
  bool fallenBack() const {return fallen_back_;}
  bool switchedBack() const {return switched_back_;}
  // fallback 조건이 이어진 시간 (로그용, 조건이 없으면 0)
  double conditionSec(Clock::time_point now) const {return cond_active_ ? secs(now - cond_since_) : 0.0;}
  Clock::time_point switchedAt() const {return switched_at_;}

private:
  static double secs(Clock::duration d) {return std::chrono::duration<double>(d).count();}
  Config cfg_;
  Source active_{Source::kPrimary};
  bool fallen_back_{false};
  bool switched_back_{false};
  bool cond_active_{false};
  Clock::time_point cond_since_{};
  bool back_active_{false};
  Clock::time_point back_since_{};
  Clock::time_point switched_at_{};
};

// [10-10 W1-4] 콜백이 만들어질 때의 source 세대와 지금 세대가 다르면 옛 source (전환 전 구독·client) 의 늦은 콜백이다 → 버린다.
inline bool isCurrentSourceGen(uint64_t callback_gen, uint64_t current_gen) {return callback_gen == current_gen;}

}  // namespace replan_monitor

#endif  // REPLAN_MONITOR__MIRROR_SYNC_POLICY_HPP_
