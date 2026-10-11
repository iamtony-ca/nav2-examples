// [10-10 W1] Nav2 costmap 로컬 mirror 구현. 설명·lock 순서는 costmap_mirror.hpp 참고.

#include "replan_monitor/costmap_mirror.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"

namespace replan_monitor
{

bool CostmapSignature::nearlyEquals(
  const CostmapSignature & other, double origin_tol, double res_tol) const
{
  return size_x == other.size_x && size_y == other.size_y &&
         std::fabs(resolution - other.resolution) <= res_tol &&
         std::fabs(origin_x - other.origin_x) <= origin_tol &&
         std::fabs(origin_y - other.origin_y) <= origin_tol;
}

CostmapSignature CostmapSignature::fromMsg(const nav2_msgs::msg::Costmap & msg)
{
  CostmapSignature sig;
  sig.size_x = msg.metadata.size_x;
  sig.size_y = msg.metadata.size_y;
  sig.resolution = msg.metadata.resolution;
  sig.origin_x = msg.metadata.origin.position.x;
  sig.origin_y = msg.metadata.origin.position.y;
  return sig;
}

CostmapMirror::CostmapMirror()
: CostmapMirror(Options{}) {}

CostmapMirror::CostmapMirror(const Options & opts)
: opts_(opts) {}

int64_t CostmapMirror::stampNs(const builtin_interfaces::msg::Time & t)
{
  return static_cast<int64_t>(t.sec) * 1000000000LL + static_cast<int64_t>(t.nanosec);
}

bool CostmapMirror::updateFits(
  const nav2_costmap_2d::Costmap2D & cm, const nav2_msgs::msg::CostmapUpdate & u)
{
  const uint64_t sx = cm.getSizeInCellsX();
  const uint64_t sy = cm.getSizeInCellsY();
  // uint32 덧셈 overflow 를 피하려고 64 bit 로 계산한다.
  return static_cast<uint64_t>(u.x) + u.size_x <= sx &&
         static_cast<uint64_t>(u.y) + u.size_y <= sy &&
         u.data.size() == static_cast<size_t>(u.size_x) * static_cast<size_t>(u.size_y);
}

void CostmapMirror::copyRect(nav2_costmap_2d::Costmap2D & cm, const nav2_msgs::msg::CostmapUpdate & u)
{
  unsigned char * dst = cm.getCharMap();
  const size_t sx = cm.getSizeInCellsX();
  for (size_t row = 0; row < u.size_y; ++row) {
    std::memcpy(
      dst + (static_cast<size_t>(u.y) + row) * sx + u.x,
      u.data.data() + row * u.size_x,
      u.size_x);
  }
}

// [10-10 W1-2][리뷰] grid 의 0 이 아닌 셀 bbox. full 뒤 첫 self-contained update 에서 한 번 훑는다 (O(map)).
//   행마다 4 KB 씩 0 블록과 memcmp 해서 0 인 행을 빨리 넘기고, 0 이 아닌 블록 안에서만 바이트 단위로 찾는다.
CostmapMirror::Rect CostmapMirror::nonzeroBBox(const nav2_costmap_2d::Costmap2D & cm)
{
  static const unsigned char kZero[4096] = {0};
  const size_t sx = cm.getSizeInCellsX();
  const size_t sy = cm.getSizeInCellsY();
  const unsigned char * d = cm.getCharMap();
  size_t x0 = sx, xn = 0, y0 = sy, yn = 0;
  for (size_t j = 0; j < sy; ++j) {
    const unsigned char * row = d + j * sx;
    size_t first = sx, last = 0;
    for (size_t off = 0; off < sx; off += sizeof(kZero)) {
      const size_t n = std::min(sizeof(kZero), sx - off);
      if (std::memcmp(row + off, kZero, n) == 0) {
        continue;
      }
      for (size_t k = off; k < off + n; ++k) {
        if (row[k] != 0) {
          first = std::min(first, k);
          last = std::max(last, k + 1);
        }
      }
    }
    if (first < last) {
      x0 = std::min(x0, first); xn = std::max(xn, last);
      y0 = std::min(y0, j); yn = std::max(yn, j + 1);
    }
  }
  Rect r;
  if (x0 < xn && y0 < yn) {
    r.x = static_cast<uint32_t>(x0); r.w = static_cast<uint32_t>(xn - x0);
    r.y = static_cast<uint32_t>(y0); r.h = static_cast<uint32_t>(yn - y0);
  }
  return r;
}

void CostmapMirror::zeroRect(nav2_costmap_2d::Costmap2D & cm, const Rect & r)
{
  if (r.empty()) {
    return;
  }
  unsigned char * d = cm.getCharMap();
  const size_t sx = cm.getSizeInCellsX();
  for (size_t row = 0; row < r.h; ++row) {
    std::memset(d + (static_cast<size_t>(r.y) + row) * sx + r.x, 0, r.w);
  }
}

void CostmapMirror::bufferForReplay(const nav2_msgs::msg::CostmapUpdate::ConstSharedPtr & msg)
{
  // [10-10 W1-2][리뷰] self_contained: 가장 새 update 하나가 그 시점의 mask 전체이므로 그것만 둔다. 밀려난 것은
  //   잃은 것이 아니다 (더 새 update 가 대신한다). 한도보다 크면 (예: activate 뒤 grid 전체 update) 두지 않는다 —
  //   그보다 오래된 full 은 applyFull 이 kRejectedSuperseded 로 버리므로 되돌아가지 않는다.
  if (opts_.self_contained_updates) {
    replay_.clear();
    replay_bytes_ = 0;
    if (msg->data.size() <= opts_.replay_max_bytes && opts_.replay_max_msgs > 0) {
      replay_.push_back(msg);
      replay_bytes_ = msg->data.size();
    }
    return;
  }
  // [10-10 W1][리뷰] 보관하지 못한 update(크기 초과 / 한도로 밀려남)는 stamp 를 기억한다. 이보다 오래된 full
  //   (예: 그 update 전에 뜬 get_costmap 스냅샷)을 나중에 적용하면 그 내용이 빠지므로 applyFull 이 resync 를 요구한다.
  //   ClearEntireCostmap 뒤에는 StaticLayer::reset 으로 지도 전체 크기 update(2e8 cells = 200 MB)가 나온다.
  uint64_t lost = 0;
  auto mark_lost = [&](const nav2_msgs::msg::CostmapUpdate & u) {
      lost_replay_max_stamp_ns_ = std::max(lost_replay_max_stamp_ns_, stampNs(u.header.stamp));
      ++lost;
    };
  if (opts_.replay_max_msgs == 0 || msg->data.size() > opts_.replay_max_bytes) {
    mark_lost(*msg);
  } else {
    replay_.push_back(msg);
    replay_bytes_ += msg->data.size();
    while (!replay_.empty() &&
      (replay_.size() > opts_.replay_max_msgs || replay_bytes_ > opts_.replay_max_bytes))
    {
      // 현재 full 보다 오래된 것은 이미 grid 에 들어 있고 앞으로 올 (더 새) full 에도 들어 있으니 잃은 것이 아니다.
      // [10-10 W1-4] source 전환 뒤 새 full 전 (baseline_pending_) 에 보관한 것은 아직 grid 에 없다 → 밀려나면 잃은 것
      if (stampNs(replay_.front()->header.stamp) >= last_full_stamp_ns_ || !grid_ || baseline_pending_) {
        mark_lost(*replay_.front());
      }
      replay_bytes_ -= replay_.front()->data.size();
      replay_.pop_front();
    }
  }
  if (lost > 0) {
    std::lock_guard<std::mutex> sl(stats_mtx_);
    stats_.unreplayable_updates += lost;
  }
}

void CostmapMirror::releaseIdleSpareLocked(std::chrono::steady_clock::time_point now)
{
  if (spare_ &&
    std::chrono::duration<double>(now - last_full_wall_).count() > opts_.spare_keep_window_sec)
  {
    spare_.reset();
    full_streak_ = 0;
    std::lock_guard<std::mutex> sl(stats_mtx_);
    ++stats_.spare_releases;
  }
}

void CostmapMirror::releaseIdleSpare()
{
  std::lock_guard<std::mutex> wlock(write_mtx_);
  releaseIdleSpareLocked(std::chrono::steady_clock::now());
}

bool CostmapMirror::hasSpare() const
{
  std::lock_guard<std::mutex> wlock(write_mtx_);
  return static_cast<bool>(spare_);
}

uint64_t CostmapMirror::fullGeneration() const
{
  return full_gen_.load();
}

void CostmapMirror::pruneReplay(int64_t older_than_ns)
{
  std::deque<nav2_msgs::msg::CostmapUpdate::ConstSharedPtr> keep;
  size_t bytes = 0;
  for (auto & u : replay_) {
    if (stampNs(u->header.stamp) >= older_than_ns) {
      bytes += u->data.size();
      keep.push_back(std::move(u));
    }
  }
  replay_.swap(keep);
  replay_bytes_ = bytes;
}

CostmapMirror::FullOutcome CostmapMirror::applyFull(
  const nav2_msgs::msg::Costmap & msg, uint64_t requested_at_gen, bool geometry_only)
{
  std::lock_guard<std::mutex> wlock(write_mtx_);
  FullOutcome out;

  const CostmapSignature msg_sig = CostmapSignature::fromMsg(msg);
  // [10-10 W1-4][리뷰] 빈 full (0x0 또는 resolution<=0) 은 "아직 데이터 없음" 이다. 예전에는 0 == 0 으로 크기 검사를 통과해
  //   0x0 Costmap2D 로 바꿔 넣었고 (hasGrid()=true → resync 완료로 보고 다시 묻지 않음), worldToMapEnforceBounds 가
  //   size_x_-1 = UINT_MAX 를 돌려 0 길이 버퍼를 읽었다. stamp·baseline 판정보다 먼저 거르고 아무 상태도 바꾸지 않는다.
  if (msg_sig.size_x == 0 || msg_sig.size_y == 0 || !(msg_sig.resolution > 0.0)) {
    std::lock_guard<std::mutex> sl(stats_mtx_);
    ++stats_.empty_fulls;
    out.result = FullResult::kRejectedEmpty;
    return out;
  }
  const size_t expected = static_cast<size_t>(msg_sig.size_x) * static_cast<size_t>(msg_sig.size_y);
  if (msg.data.size() != expected) {
    std::lock_guard<std::mutex> sl(stats_mtx_);
    ++stats_.rejected_fulls;
    out.result = FullResult::kRejectedBadSize;
    return out;
  }

  const int64_t stamp = stampNs(msg.header.stamp);
  const bool had_grid = static_cast<bool>(grid_);   // grid_ 포인터는 write_mtx_ 아래에서만 바뀐다
  // [10-10 W1][리뷰] 예전에는 topic full 이면 오래돼도 (clock reset 대비) 적용하고 replay 를 비웠다. 그러면 False 에서
  //   늦게 붙을 때 get_costmap 응답(T_s) 뒤에 도착한 latched activation full(T0 < T_s)이 grid 를 T0 로 되돌리고,
  //   resync 는 이미 끝나 다시 고쳐지지 않았다. 이제 오래된 full 은 출처와 무관하게 버린다 (grid·replay·stamp 그대로).
  //   clock reset 은 메시지 stamp 만으로는 latched 옛 full 과 구별되지 않으므로, "현재 grid 를 받은 뒤 보낸 요청의
  //   응답" 이라는 순서 정보로만 판정한다: Nav2 는 요청을 받은 시점에 스냅샷을 뜨고 stamp 를 찍으므로, clock 이
  //   단조롭다면 그 응답은 현재 grid 의 full 보다 오래될 수 없다.
  // [10-10 W1-4] source 전환 뒤 첫 full 은 stamp 와 상관없이 적용한다 (옛 source 의 stamp 와 비교하지 않는다).
  const bool baseline_pending = baseline_pending_;
  if (had_grid && !baseline_pending && stamp < last_full_stamp_ns_) {
    const bool requested_after_current = requested_at_gen != kNoRequestGen &&
      requested_at_gen == full_gen_.load();
    if (!requested_after_current) {
      std::lock_guard<std::mutex> sl(stats_mtx_);
      ++stats_.stale_fulls;
      out.result = FullResult::kRejectedStale;
      return out;
    }
    // clock reset: 옛 epoch 의 update 는 쓸모없다. 잃은 update 기록도 옛 epoch 것이다.
    out.clock_reset = true;
    replay_.clear();
    replay_bytes_ = 0;
    lost_replay_max_stamp_ns_ = std::numeric_limits<int64_t>::min();
    sc_last_update_stamp_ns_ = std::numeric_limits<int64_t>::min();   // [10-10 W1-2][리뷰]
  }

  const bool sig_changed = !had_grid ||
    !msg_sig.nearlyEquals(sig_, opts_.origin_tol_m, opts_.resolution_tol);
  // geometry 가 같으면 지금 grid 의 geometry 를 그대로 쓴다 (1 ulp 차이 origin 으로 바꾸지 않는다).
  const CostmapSignature use_sig = sig_changed ? msg_sig : sig_;

  // [10-10 W1-2][리뷰] self_contained: 같은 geometry 에서 이미 적용한 update 가 이 full 보다 새것이면 지금 grid 가 더 새
  //   상태다 (update 하나가 mask 전체). 버리고, resync 를 다시 걸 이유도 아니다. geometry 가 바뀌었으면 (원점만 바뀐 사이
  //   update 가 옛 grid 에 맞아 들어갔을 수 있다) 오래돼도 적용한다.
  const bool sc = opts_.self_contained_updates;
  if (sc && had_grid && !baseline_pending && !out.clock_reset && !sig_changed && stamp < sc_last_update_stamp_ns_) {
    std::lock_guard<std::mutex> sl(stats_mtx_);
    ++stats_.superseded_fulls;
    out.result = FullResult::kRejectedSuperseded;
    return out;
  }

  // ---- lock 밖에서 새 grid 를 만든다 ----
  std::unique_ptr<nav2_costmap_2d::Costmap2D> next;
  if (!sig_changed && spare_) {
    next = std::move(spare_);
  } else {
    spare_.reset();
    next = std::make_unique<nav2_costmap_2d::Costmap2D>(
      use_sig.size_x, use_sig.size_y, use_sig.resolution, use_sig.origin_x, use_sig.origin_y,
      nav2_costmap_2d::FREE_SPACE);
  }
  uint64_t replayed = 0;
  bool replay_loss = false;
  if (sc) {
    // [10-10 W1-2][리뷰] self_contained: full 보다 새 update 가 있으면 그 하나가 정답이다 → grid 를 0 으로 두고 그것만
    //   복사한다. 없으면 full 내용 (geometry_only 면 0). replay 에는 가장 새 update 하나만 있다.
    nav2_msgs::msg::CostmapUpdate::ConstSharedPtr newest;
    for (const auto & u : replay_) {
      if (stampNs(u->header.stamp) >= stamp && updateFits(*next, *u)) {
        newest = u;
      }
    }
    if (newest || geometry_only) {
      if (expected > 0) {
        std::memset(next->getCharMap(), 0, expected);
      }
    } else if (expected > 0) {
      std::memcpy(next->getCharMap(), msg.data.data(), expected);
    }
    if (newest) {
      copyRect(*next, *newest);
      ++replayed;
      sc_prev_ = Rect{newest->x, newest->y, newest->size_x, newest->size_y};
      sc_prev_known_ = true;
      sc_last_update_stamp_ns_ = stampNs(newest->header.stamp);
    } else {
      sc_prev_ = Rect{};
      sc_prev_known_ = geometry_only;   // 0 으로 채웠으면 0 아닌 셀이 없다. 아니면 다음 update 때 bbox 를 훑는다
      sc_last_update_stamp_ns_ = std::numeric_limits<int64_t>::min();
    }
    out.needs_resync = out.clock_reset;
  } else {
    if (expected > 0) {
      std::memcpy(next->getCharMap(), msg.data.data(), expected);
    }

    // [10-10 W1][리뷰] 이 full 이후에 찍혔는데 보관하지 못한 update 가 있으면 그 내용은 replay 로 되살릴 수 없다.
    //   그래도 적용은 한다 (dropped update 로 resync 하는 경우처럼 현재 grid 가 틀렸을 수 있으므로). 대신 다시 받게 한다.
    replay_loss = lost_replay_max_stamp_ns_ >= stamp;
    out.needs_resync = out.clock_reset || replay_loss;

    // full 보다 늦게 찍힌 update 는 full 에 빠져 있을 수 있다 (다른 topic·service 라 도착 순서가 보장되지 않음).
    for (const auto & u : replay_) {
      if (stampNs(u->header.stamp) >= stamp && updateFits(*next, *u)) {
        copyRect(*next, *u);
        ++replayed;
      }
    }
  }
  pruneReplay(stamp);

  // ---- exclusive lock 아래에서는 swap 만 ----
  {
    std::lock_guard<std::mutex> gate(gate_mtx_);       // 새 reader 를 막고
    std::unique_lock<std::shared_mutex> xl(grid_mtx_);  // 지금 읽는 reader 가 끝나길 기다린다
    grid_.swap(next);
  }
  sig_ = use_sig;
  last_full_stamp_ns_ = stamp;
  baseline_pending_ = false;   // [10-10 W1-4]
  full_gen_.fetch_add(1);
  has_grid_.store(true);
  // [10-10 W1-3] 0 으로 채우고 replay 할 update 도 없었으면 내용 근거가 없다 (self_contained geometry_only)
  const bool content = !(sc && geometry_only) || replayed > 0;
  content_known_.store(content);

  // 옛 grid (next) — 같은 geometry 면 다음 full 에 재사용, 아니면 여기서(lock 밖) 해제.
  // [10-10 W1][리뷰] 예전에는 2 번째 full 이 window 안이면 spare 를 남겼다. False 의 정상 시작(topic full + get_costmap
  //   응답)이 정확히 그 경우라 이후 full 이 없어도 200 MB 가 끝까지 남았다. 이제 window 간격 안으로 3 번째 full 부터만
  //   남기고, 남긴 것도 window 동안 full 이 없으면 applyUpdate()/releaseIdleSpare() 에서 해제한다.
  const auto wall_now = std::chrono::steady_clock::now();
  const bool within_window = had_grid &&
    std::chrono::duration<double>(wall_now - last_full_wall_).count() <= opts_.spare_keep_window_sec;
  full_streak_ = within_window ? full_streak_ + 1 : 0;
  last_full_wall_ = wall_now;
  if (opts_.keep_spare_buffer && !sig_changed && full_streak_ >= 2) {
    spare_ = std::move(next);
  } else {
    next.reset();
  }

  {
    std::lock_guard<std::mutex> sl(stats_mtx_);
    stats_.has_grid = true;
    stats_.sig = sig_;
    ++stats_.fulls;
    stats_.replayed_updates += replayed;
    if (out.clock_reset) {++stats_.clock_resets;}
    if (replay_loss && !out.clock_reset) {++stats_.replay_loss_resyncs;}   // [10-10 W1-2][리뷰]
    if (sc && geometry_only) {++stats_.geometry_only_fulls;}              // [10-10 W1-2][리뷰]
    stats_.content_known = content;                                       // [10-10 W1-3]
    stats_.baseline_pending = false;                                       // [10-10 W1-4]
    stats_.last_full_stamp_ns = stamp;
    stats_.last_full_time = wall_now;
  }
  out.result = sig_changed ? FullResult::kAppliedNewSignature : FullResult::kApplied;
  out.replayed = replayed;
  return out;
}

CostmapMirror::UpdateResult CostmapMirror::applyUpdate(
  const nav2_msgs::msg::CostmapUpdate::ConstSharedPtr & msg)
{
  if (!msg) {
    return UpdateResult::kRejectedBadSize;
  }
  std::lock_guard<std::mutex> wlock(write_mtx_);

  if (msg->data.size() != static_cast<size_t>(msg->size_x) * static_cast<size_t>(msg->size_y)) {
    std::lock_guard<std::mutex> sl(stats_mtx_);
    ++stats_.dropped_updates;
    return UpdateResult::kRejectedBadSize;
  }

  const int64_t stamp = stampNs(msg->header.stamp);
  // [10-10 W1][리뷰] full 이 끊기면(False) spare 를 오래 쥐지 않는다.
  releaseIdleSpareLocked(std::chrono::steady_clock::now());
  // [10-10 W1-4] source 전환 뒤 새 source 의 full 전이면 grid (옛 source 기준) 에 쓰지 않고 보관만 한다.
  if (!grid_ || baseline_pending_) {
    bufferForReplay(msg);
    std::lock_guard<std::mutex> sl(stats_mtx_);
    ++stats_.early_updates;
    return UpdateResult::kNoGrid;
  }
  if (stamp < last_full_stamp_ns_) {
    std::lock_guard<std::mutex> sl(stats_mtx_);
    ++stats_.stale_updates;
    return UpdateResult::kStale;
  }

  // 범위 밖이어도 보관한다 — 아직 안 온 새 geometry 의 full 뒤에 다시 적용될 수 있다.
  bufferForReplay(msg);
  if (!updateFits(*grid_, *msg)) {
    std::lock_guard<std::mutex> sl(stats_mtx_);
    ++stats_.dropped_updates;
    return UpdateResult::kRejectedOutOfBounds;
  }

  if (opts_.self_contained_updates) {
    // [10-10 W1-2][리뷰] 직전 update 사각형(= 지금 0 아닌 셀을 모두 담는 사각형)을 0 으로 만들고 새 사각형을 복사한다.
    //   full 뒤 첫 update 면 그 범위를 모르므로 grid 를 한 번 훑는다. 훑기는 exclusive lock 밖에서 한다: grid 를 바꾸는
    //   것은 writer 뿐이고 writer 는 write_mtx_ 로 직렬화되며, reader 와 함께 읽기만 하는 것은 안전하다.
    Rect zr = sc_prev_;
    bool scanned = false;
    if (!sc_prev_known_) {
      zr = nonzeroBBox(*grid_);
      scanned = true;
    }
    {
      std::lock_guard<std::mutex> gate(gate_mtx_);
      std::unique_lock<std::shared_mutex> xl(grid_mtx_);
      zeroRect(*grid_, zr);
      copyRect(*grid_, *msg);
    }
    sc_prev_ = Rect{msg->x, msg->y, msg->size_x, msg->size_y};
    sc_prev_known_ = true;
    sc_last_update_stamp_ns_ = stamp;
    if (scanned) {
      std::lock_guard<std::mutex> sl(stats_mtx_);
      ++stats_.sc_bbox_scans;
    }
  } else {
    std::lock_guard<std::mutex> gate(gate_mtx_);
    std::unique_lock<std::shared_mutex> xl(grid_mtx_);
    copyRect(*grid_, *msg);
  }

  content_known_.store(true);   // [10-10 W1-3]
  std::lock_guard<std::mutex> sl(stats_mtx_);
  ++stats_.updates;
  stats_.content_known = true;
  stats_.last_update_stamp_ns = stamp;
  stats_.last_update_time = std::chrono::steady_clock::now();
  return UpdateResult::kApplied;
}

// [10-10 W1-3] 머리 주석은 hpp. applyFull 의 self_contained 분기와 같은 규칙으로 0 grid 를 만들고 가장 새 update 를 적용한다.
CostmapMirror::SeedOutcome CostmapMirror::seedZero(const CostmapSignature & sig)
{
  std::lock_guard<std::mutex> wlock(write_mtx_);
  SeedOutcome out;
  if (grid_ || !opts_.self_contained_updates || sig.size_x == 0 || sig.size_y == 0 ||
    !(sig.resolution > 0.0))
  {
    return out;
  }
  // ---- lock 밖에서 새 grid ----  (Costmap2D 생성자가 default_value(0)로 채운다)
  auto next = std::make_unique<nav2_costmap_2d::Costmap2D>(
    sig.size_x, sig.size_y, sig.resolution, sig.origin_x, sig.origin_y, nav2_costmap_2d::FREE_SPACE);
  nav2_msgs::msg::CostmapUpdate::ConstSharedPtr newest;
  for (const auto & u : replay_) {
    if (updateFits(*next, *u) &&
      (!newest || stampNs(u->header.stamp) >= stampNs(newest->header.stamp)))
    {
      newest = u;
    }
  }
  if (newest) {
    copyRect(*next, *newest);
    out.replayed = 1;
    sc_prev_ = Rect{newest->x, newest->y, newest->size_x, newest->size_y};
    sc_last_update_stamp_ns_ = stampNs(newest->header.stamp);
  } else {
    sc_prev_ = Rect{};
    sc_last_update_stamp_ns_ = std::numeric_limits<int64_t>::min();
  }
  sc_prev_known_ = true;   // 0 grid (+ 사각형 하나) — 0 아닌 셀은 sc_prev_ 안에만 있다
  replay_.clear();
  replay_bytes_ = 0;
  {
    std::lock_guard<std::mutex> gate(gate_mtx_);
    std::unique_lock<std::shared_mutex> xl(grid_mtx_);
    grid_.swap(next);
  }
  sig_ = sig;
  // stamp 기준이 없다: 이후 update·full 은 stale 로 버리지 않는다 (full 은 superseded 규칙만 남는다).
  last_full_stamp_ns_ = std::numeric_limits<int64_t>::min();
  lost_replay_max_stamp_ns_ = std::numeric_limits<int64_t>::min();
  full_gen_.fetch_add(1);
  has_grid_.store(true);
  content_known_.store(out.replayed > 0);
  const auto wall_now = std::chrono::steady_clock::now();
  last_full_wall_ = wall_now;
  full_streak_ = 0;
  out.seeded = true;
  {
    std::lock_guard<std::mutex> sl(stats_mtx_);
    stats_.has_grid = true;
    stats_.sig = sig_;
    ++stats_.seeds;
    stats_.replayed_updates += out.replayed;
    stats_.content_known = out.replayed > 0;
    if (out.replayed > 0) {
      stats_.last_update_stamp_ns = sc_last_update_stamp_ns_;
      stats_.last_update_time = wall_now;
    }
    stats_.last_full_time = wall_now;
  }
  return out;
}

bool CostmapMirror::contentKnown() const
{
  return content_known_.load();
}

// [10-10 W1-4] 머리 주석은 hpp.
void CostmapMirror::beginNewSource()
{
  std::lock_guard<std::mutex> wlock(write_mtx_);
  replay_.clear();
  replay_bytes_ = 0;
  lost_replay_max_stamp_ns_ = std::numeric_limits<int64_t>::min();
  last_full_stamp_ns_ = std::numeric_limits<int64_t>::min();
  // self_contained 상태도 옛 source 것이다 (global 에는 쓰지 않지만 일관되게)
  sc_prev_known_ = false;
  sc_prev_ = Rect{};
  sc_last_update_stamp_ns_ = std::numeric_limits<int64_t>::min();
  spare_.reset();
  full_streak_ = 0;
  baseline_pending_ = true;
  full_gen_.fetch_add(1);
  std::lock_guard<std::mutex> sl(stats_mtx_);
  ++stats_.source_resets;
  stats_.baseline_pending = true;
}

bool CostmapMirror::baselinePending() const
{
  std::lock_guard<std::mutex> wlock(write_mtx_);
  return baseline_pending_;
}

CostmapMirror::ReadGuard CostmapMirror::read() const
{
  std::shared_lock<std::shared_mutex> lk;
  {
    std::lock_guard<std::mutex> gate(gate_mtx_);   // writer 가 기다리는 중이면 여기서 줄 선다
    lk = std::shared_lock<std::shared_mutex>(grid_mtx_);
  }
  if (!grid_) {
    return ReadGuard(std::shared_lock<std::shared_mutex>(), nullptr);
  }
  const nav2_costmap_2d::Costmap2D * cm = grid_.get();
  return ReadGuard(std::move(lk), cm);
}

bool CostmapMirror::hasGrid() const
{
  return has_grid_.load();
}

CostmapMirror::Stats CostmapMirror::stats() const
{
  std::lock_guard<std::mutex> sl(stats_mtx_);
  return stats_;
}

void CostmapMirror::setKeepSpareBuffer(bool keep)
{
  std::lock_guard<std::mutex> wlock(write_mtx_);
  opts_.keep_spare_buffer = keep;
  if (!keep) {
    spare_.reset();
  }
}

void CostmapMirror::setSelfContainedUpdates(bool on)
{
  std::lock_guard<std::mutex> wlock(write_mtx_);
  opts_.self_contained_updates = on;
  sc_prev_known_ = false;
  sc_prev_ = Rect{};
  sc_last_update_stamp_ns_ = std::numeric_limits<int64_t>::min();
}

bool CostmapMirror::selfContained() const
{
  std::lock_guard<std::mutex> wlock(write_mtx_);
  return opts_.self_contained_updates;
}

void CostmapMirror::setReplayMaxMsgs(size_t n)
{
  std::lock_guard<std::mutex> wlock(write_mtx_);
  opts_.replay_max_msgs = n;
}

}  // namespace replan_monitor
