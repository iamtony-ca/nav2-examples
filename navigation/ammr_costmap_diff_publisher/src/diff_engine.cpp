// [10-10 W1-D] DiffEngine 구현. 설계와 불변식은 diff_engine.hpp 머리말 참고.
#include "ammr_costmap_diff_publisher/diff_engine.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <queue>
#include <thread>
#include <tuple>
#include <utility>

namespace ammr_costmap_diff_publisher
{

namespace
{
constexpr size_t kUpdateHeaderBytes = 64;   // CostmapUpdate 의 data 외 대략 크기 (통계·backlog 계산용)

double msSince(DiffEngine::SteadyClock::time_point t0)
{
  return std::chrono::duration<double, std::milli>(DiffEngine::SteadyClock::now() - t0).count();
}

Rect clampRect(const Rect & r, uint32_t sx, uint32_t sy)
{
  Rect o;
  o.x0 = std::min(r.x0, sx);
  o.y0 = std::min(r.y0, sy);
  o.x1 = std::min(r.x1, sx);
  o.y1 = std::min(r.y1, sy);
  return o;
}
}  // namespace

const char * toString(FullReason r)
{
  switch (r) {
    case FullReason::kInitial: return "initial";
    case FullReason::kGeometry: return "geometry";
    case FullReason::kNewSubscriber: return "new_subscriber";
    case FullReason::kBacklog: return "backlog";
    case FullReason::kActivate: return "activate";
    case FullReason::kLargeDiff: return "large_diff";
    case FullReason::kManual: return "manual";
    case FullReason::kRetry: return "retry";
  }
  return "?";
}

const char * toString(RepairSource r)
{
  switch (r) {
    case RepairSource::kNone: return "none";
    case RepairSource::kCatchup: return "catchup";
    case RepairSource::kRefresh: return "refresh";
    case RepairSource::kSweep: return "sweep";
    case RepairSource::kRetry: return "retry";
  }
  return "?";
}

Geometry Geometry::of(const nav2_costmap_2d::Costmap2D & c)
{
  Geometry g;
  g.size_x = c.getSizeInCellsX();
  g.size_y = c.getSizeInCellsY();
  g.resolution = c.getResolution();
  g.origin_x = c.getOriginX();
  g.origin_y = c.getOriginY();
  // Costmap2DPublisher::prepareCostmap 과 같은 식 (mapToWorld(0,0) - res/2)
  double wx = 0.0, wy = 0.0;
  c.mapToWorld(0, 0, wx, wy);
  g.msg_origin_x = wx - g.resolution / 2;
  g.msg_origin_y = wy - g.resolution / 2;
  return g;
}

// ---------------------------------------------------------------------------
// 사각형 추출
// ---------------------------------------------------------------------------
std::vector<Rect> rectsFromMask(
  const std::vector<uint8_t> & mask, uint32_t w, uint32_t h,
  uint32_t base_x, uint32_t base_y, uint32_t tile, const Rect & clip,
  const std::vector<Rect> * boxes)
{
  struct Open
  {
    uint32_t tx0, tx1, ty0;
    Rect ub;   // boxes 가 있을 때: 소속 tile 들의 tight box 합집합
  };
  std::vector<Rect> out;
  std::vector<Open> prev, cur;
  auto emit = [&](const Open & o, uint32_t ty_end) {
      Rect r;
      if (boxes) {
        r.x0 = std::max(clip.x0, o.ub.x0);
        r.x1 = std::min(clip.x1, o.ub.x1);
        r.y0 = std::max(clip.y0, o.ub.y0);
        r.y1 = std::min(clip.y1, o.ub.y1);
      } else {
        r.x0 = std::max(clip.x0, base_x + o.tx0 * tile);
        r.x1 = std::min(clip.x1, base_x + o.tx1 * tile);
        r.y0 = std::max(clip.y0, base_y + o.ty0 * tile);
        r.y1 = std::min(clip.y1, base_y + ty_end * tile);
      }
      if (!r.empty()) {
        out.push_back(r);
      }
    };
  auto unite = [](Rect & a, const Rect & b) {
      a.x0 = std::min(a.x0, b.x0);
      a.y0 = std::min(a.y0, b.y0);
      a.x1 = std::max(a.x1, b.x1);
      a.y1 = std::max(a.y1, b.y1);
    };
  for (uint32_t ty = 0; ty < h; ++ty) {
    cur.clear();
    const uint8_t * row = mask.data() + size_t(ty) * w;
    size_t p = 0;
    uint32_t tx = 0;
    while (tx < w) {
      if (!row[tx]) {
        ++tx;
        continue;
      }
      const uint32_t s = tx;
      Rect ub{UINT32_MAX, UINT32_MAX, 0, 0};
      while (tx < w && row[tx]) {
        if (boxes) {
          unite(ub, (*boxes)[size_t(ty) * w + tx]);
        }
        ++tx;
      }
      // run [s, tx)
      while (p < prev.size() && prev[p].tx0 < s) {
        emit(prev[p], ty);
        ++p;
      }
      if (p < prev.size() && prev[p].tx0 == s && prev[p].tx1 == tx) {
        Open o = prev[p];                // 위 row 와 같은 run → 세로로 이어 붙임
        if (boxes) {
          unite(o.ub, ub);
        }
        cur.push_back(o);
        ++p;
      } else {
        cur.push_back({s, tx, ty, ub});
      }
    }
    for (; p < prev.size(); ++p) {
      emit(prev[p], ty);
    }
    prev.swap(cur);
  }
  for (const auto & o : prev) {
    emit(o, h);
  }
  return out;
}

namespace
{
uint64_t spreadBits(uint32_t v)
{
  uint64_t x = v;
  x = (x | (x << 16)) & 0x0000FFFF0000FFFFull;
  x = (x | (x << 8)) & 0x00FF00FF00FF00FFull;
  x = (x | (x << 4)) & 0x0F0F0F0F0F0F0F0Full;
  x = (x | (x << 2)) & 0x3333333333333333ull;
  x = (x | (x << 1)) & 0x5555555555555555ull;
  return x;
}
uint64_t mortonOf(const Rect & r)
{
  return spreadBits((r.x0 + r.x1) / 2) | (spreadBits((r.y0 + r.y1) / 2) << 1);
}
Rect bbox(const Rect & a, const Rect & b)
{
  return Rect{std::min(a.x0, b.x0), std::min(a.y0, b.y0), std::max(a.x1, b.x1), std::max(a.y1, b.y1)};
}
bool contains(const Rect & a, const Rect & b)
{
  return a.x0 <= b.x0 && a.y0 <= b.y0 && a.x1 >= b.x1 && a.y1 >= b.y1;
}
}  // namespace

std::vector<Rect> mergeRectsGreedy(std::vector<Rect> rects, uint32_t max_rects)
{
  max_rects = std::max<uint32_t>(1, max_rects);
  if (rects.size() <= max_rects) {
    return rects;
  }
  // Morton(Z) 순서로 늘어놓고, 이웃한 두 사각형 중 "합쳤을 때 늘어나는 면적" 이 가장 작은 쌍부터 합친다.
  // (2×2 coarsening 은 멀리 떨어진 덩어리를 거대한 사각형으로 묶어 버려서 쓰지 않는다 — 시험 (d') 참고)
  std::sort(rects.begin(), rects.end(), [](const Rect & a, const Rect & b) {return mortonOf(a) < mortonOf(b);});
  const size_t n = rects.size();
  constexpr size_t kNone = static_cast<size_t>(-1);
  std::vector<size_t> prv(n), nxt(n);
  std::vector<uint32_t> ver(n, 0);
  std::vector<uint8_t> alive(n, 1);
  for (size_t i = 0; i < n; ++i) {
    prv[i] = i == 0 ? kNone : i - 1;
    nxt[i] = i + 1 < n ? i + 1 : kNone;
  }
  struct Cand
  {
    int64_t cost;
    size_t i, j;
    uint32_t vi, vj;
    bool operator>(const Cand & o) const {return cost > o.cost;}
  };
  auto cost = [&](size_t i, size_t j) {
      return static_cast<int64_t>(bbox(rects[i], rects[j]).area()) -
             static_cast<int64_t>(rects[i].area()) - static_cast<int64_t>(rects[j].area());
    };
  std::priority_queue<Cand, std::vector<Cand>, std::greater<Cand>> pq;
  for (size_t i = 0; i + 1 < n; ++i) {
    pq.push({cost(i, i + 1), i, i + 1, 0, 0});
  }
  size_t count = n;
  while (count > max_rects && !pq.empty()) {
    const Cand c = pq.top();
    pq.pop();
    if (!alive[c.i] || !alive[c.j] || ver[c.i] != c.vi || ver[c.j] != c.vj || nxt[c.i] != c.j) {
      continue;
    }
    rects[c.i] = bbox(rects[c.i], rects[c.j]);
    alive[c.j] = 0;
    nxt[c.i] = nxt[c.j];
    if (nxt[c.j] != kNone) {
      prv[nxt[c.j]] = c.i;
    }
    ++ver[c.i];
    --count;
    if (prv[c.i] != kNone) {
      pq.push({cost(prv[c.i], c.i), prv[c.i], c.i, ver[prv[c.i]], ver[c.i]});
    }
    if (nxt[c.i] != kNone) {
      pq.push({cost(c.i, nxt[c.i]), c.i, nxt[c.i], ver[c.i], ver[nxt[c.i]]});
    }
  }
  std::vector<Rect> out;
  out.reserve(count);
  for (size_t i = 0; i < n; ++i) {
    if (alive[i]) {
      out.push_back(rects[i]);
    }
  }
  // 합친 사각형 안에 통째로 들어간 것은 뺀다 (같은 셀을 두 번 보내지 않게, k <= max_rects 라 O(k^2) 로 충분)
  std::vector<uint8_t> drop(out.size(), 0);
  for (size_t a = 0; a < out.size(); ++a) {
    for (size_t b = 0; b < out.size(); ++b) {
      if (a != b && !drop[a] && !drop[b] && contains(out[a], out[b])) {
        drop[b] = 1;
      }
    }
  }
  std::vector<Rect> res;
  res.reserve(out.size());
  for (size_t a = 0; a < out.size(); ++a) {
    if (!drop[a]) {
      res.push_back(out[a]);
    }
  }
  return res;
}

std::vector<Rect> rectsFromMaskCapped(
  std::vector<uint8_t> mask, uint32_t w, uint32_t h,
  uint32_t base_x, uint32_t base_y, uint32_t tile, const Rect & clip, uint32_t max_rects,
  const std::vector<Rect> * boxes_in)
{
  max_rects = std::max<uint32_t>(1, max_rects);
  std::vector<Rect> boxes;
  if (boxes_in) {
    boxes = *boxes_in;
  }
  const std::vector<Rect> * bp = boxes_in ? &boxes : nullptr;
  auto rects = rectsFromMask(mask, w, h, base_x, base_y, tile, clip, bp);
  // 사각형이 아주 많으면 (상한의 16 배 초과) 먼저 2×2 OR 로 가까운 tile 끼리만 묶어 개수를 줄인다 (병합 비용 상한).
  while (rects.size() > size_t(max_rects) * 16 && (w > 1 || h > 1)) {
    const uint32_t w2 = (w + 1) / 2, h2 = (h + 1) / 2;
    std::vector<uint8_t> m2(size_t(w2) * h2, 0);
    std::vector<Rect> b2(bp ? size_t(w2) * h2 : 0, Rect{UINT32_MAX, UINT32_MAX, 0, 0});
    for (uint32_t y = 0; y < h; ++y) {
      const uint8_t * row = mask.data() + size_t(y) * w;
      uint8_t * row2 = m2.data() + size_t(y / 2) * w2;
      for (uint32_t x = 0; x < w; ++x) {
        if (!row[x]) {
          continue;
        }
        row2[x / 2] = 1;
        if (bp) {
          Rect & d = b2[size_t(y / 2) * w2 + x / 2];
          const Rect & sb = boxes[size_t(y) * w + x];
          d.x0 = std::min(d.x0, sb.x0);
          d.y0 = std::min(d.y0, sb.y0);
          d.x1 = std::max(d.x1, sb.x1);
          d.y1 = std::max(d.y1, sb.y1);
        }
      }
    }
    mask.swap(m2);
    if (bp) {
      boxes.swap(b2);
    }
    w = w2;
    h = h2;
    tile *= 2;
    rects = rectsFromMask(mask, w, h, base_x, base_y, tile, clip, bp);
  }
  return mergeRectsGreedy(std::move(rects), max_rects);
}


namespace
{
// [10-10 W1-D][리뷰] repair 사각형 정리: 같은 것·다른 것에 통째로 들어간 것을 빼고 cap 개 이하로 병합.
std::vector<Rect> compactRects(std::vector<Rect> rects, uint32_t cap)
{
  rects.erase(
    std::remove_if(rects.begin(), rects.end(), [](const Rect & r) {return r.empty();}), rects.end());
  std::sort(
    rects.begin(), rects.end(), [](const Rect & a, const Rect & b) {
      return std::tie(a.x0, a.y0, a.x1, a.y1) < std::tie(b.x0, b.y0, b.x1, b.y1);
    });
  rects.erase(
    std::unique(
      rects.begin(), rects.end(), [](const Rect & a, const Rect & b) {
        return a.x0 == b.x0 && a.y0 == b.y0 && a.x1 == b.x1 && a.y1 == b.y1;
      }), rects.end());
  rects = mergeRectsGreedy(std::move(rects), cap);
  if (rects.size() <= 1024) {
    std::vector<uint8_t> drop(rects.size(), 0);
    for (size_t a = 0; a < rects.size(); ++a) {
      for (size_t b = 0; b < rects.size(); ++b) {
        if (a != b && !drop[a] && !drop[b] && contains(rects[a], rects[b])) {
          drop[b] = 1;
        }
      }
    }
    std::vector<Rect> res;
    res.reserve(rects.size());
    for (size_t a = 0; a < rects.size(); ++a) {
      if (!drop[a]) {
        res.push_back(rects[a]);
      }
    }
    return res;
  }
  return rects;
}

int64_t steadyNs(DiffEngine::SteadyClock::time_point t)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
}

int64_t secToNs(double s)
{
  return static_cast<int64_t>(s * 1e9);
}
}  // namespace

// ---------------------------------------------------------------------------
// DiffEngine
// ---------------------------------------------------------------------------
DiffEngine::DiffEngine(const Config & cfg, StampFn stamp_fn)
: cfg_(cfg), stamp_fn_(std::move(stamp_fn))
{
  cfg_.tile_size = std::max<uint32_t>(1, cfg_.tile_size);
  cfg_.max_rects = std::max<uint32_t>(1, cfg_.max_rects);
  cfg_.refresh_max_rects = std::max<uint32_t>(1, cfg_.refresh_max_rects);
  cfg_.refresh_rounds = std::max<uint32_t>(1, cfg_.refresh_rounds);
  cfg_.max_msg_bytes = std::max<size_t>(1, cfg_.max_msg_bytes);
  if (cfg_.sweep_tick_sec <= 0.0) {
    cfg_.sweep_tick_sec = 1.0;
  }
  // 0 이하 단계는 빼고 오름차순
  std::vector<double> d;
  for (double v : cfg_.catchup_delays_sec) {
    if (v > 0.0) {
      d.push_back(v);
    }
  }
  std::sort(d.begin(), d.end());
  cfg_.catchup_delays_sec = d;
}

void DiffEngine::stampToMsg(int64_t ns, builtin_interfaces::msg::Time & t)
{
  if (ns < 0) {
    ns = 0;
  }
  t.sec = static_cast<int32_t>(ns / 1000000000LL);
  t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
}

int64_t DiffEngine::nextStampLocked()
{
  int64_t now = stamp_fn_ ? stamp_fn_() : 0;
  // (I2) 같은 시각이거나 시계가 뒤로 가면 1 ns 씩 올린다
  if (now <= last_stamp_) {
    now = last_stamp_ + 1;
  }
  last_stamp_ = now;
  return now;
}

void DiffEngine::addStats(const std::function<void(EngineStats &)> & f) const
{
  std::lock_guard<std::mutex> l(stats_mtx_);
  f(stats_);
}

std::unique_lock<std::mutex> DiffEngine::lockShadowForMap()
{
  map_waiting_.fetch_add(1);
  std::unique_lock<std::mutex> l(shadow_mtx_);
  map_waiting_.fetch_sub(1);
  return l;
}

void DiffEngine::yieldToMap() const
{
  if (map_waiting_.load() == 0) {
    return;
  }
  const auto t0 = SteadyClock::now();
  while (map_waiting_.load() > 0 && SteadyClock::now() - t0 < std::chrono::milliseconds(20)) {
    std::this_thread::yield();
  }
}

void DiffEngine::noteHold(double ms) const
{
  addStats([&](EngineStats & s) {s.max_shadow_hold_ms = std::max(s.max_shadow_hold_ms, ms);});
}

EngineStats DiffEngine::takeStats()
{
  std::lock_guard<std::mutex> l(stats_mtx_);
  EngineStats s = stats_;
  stats_ = EngineStats{};
  return s;
}

size_t DiffEngine::queueLength() const
{
  std::lock_guard<std::mutex> l(queue_mtx_);
  return queue_.size();
}

size_t DiffEngine::queuedUpdateBytes() const
{
  std::lock_guard<std::mutex> l(queue_mtx_);
  return queued_update_bytes_;
}

bool DiffEngine::fullPending() const
{
  std::lock_guard<std::mutex> l(queue_mtx_);
  return !queue_.empty() && queue_.front().kind == OutMsg::Kind::kFull;
}

Geometry DiffEngine::geometry() const
{
  std::lock_guard<std::mutex> l(shadow_mtx_);
  return geom_;
}

int64_t DiffEngine::lastStamp() const
{
  std::lock_guard<std::mutex> l(shadow_mtx_);
  return last_stamp_;
}

void DiffEngine::requestFullQueueLocked(FullReason reason)
{
  // FULL token 은 언제나 큐 맨 앞 하나뿐이다. 앞에 있던 update 는 FULL 에 포함되므로 버린다.
  uint64_t dropped = 0;
  for (const auto & it : queue_) {
    if (it.kind == OutMsg::Kind::kUpdate) {
      ++dropped;
    }
  }
  queue_.clear();
  queued_update_bytes_ = 0;
  Item tok;
  tok.kind = OutMsg::Kind::kFull;
  tok.reason = reason;
  queue_.push_back(std::move(tok));
  if (dropped) {
    addStats([&](EngineStats & s) {s.dropped_updates += dropped;});
  }
}

void DiffEngine::requestFull(FullReason reason)
{
  {
    std::lock_guard<std::mutex> ql(queue_mtx_);
    requestFullQueueLocked(reason);
  }
  queue_cv_.notify_all();
}

void DiffEngine::clearRepairStateLocked()
{
  catchup_tracking_ = false;
  catchup_rects_.clear();
  catchup_stage_ = 0;
  catchup_base_ns_ = 0;
  repair_.clear();
  repair_n_.store(0);
  refresh_cur_.clear();
  refresh_hist_.clear();
}

void DiffEngine::enqueueUpdatesLocked(std::vector<Item> && items)
{
  if (items.empty()) {
    return;
  }
  for (auto & it : items) {
    stampToMsg(nextStampLocked(), it.update->header.stamp);
  }
  {
    std::lock_guard<std::mutex> cl(catchup_mtx_);
    if (catchup_tracking_) {
      for (const auto & it : items) {
        const auto & u = *it.update;
        catchup_rects_.push_back(Rect{u.x, u.y, u.x + u.size_x, u.y + u.size_y});
      }
      // [10-10 W1-D][리뷰] 무한히 늘지 않게 (FULL publish 가 계속 실패하는 경우 등)
      if (catchup_rects_.size() > 8u * cfg_.max_rects) {
        catchup_rects_ = compactRects(std::move(catchup_rects_), cfg_.max_rects);
      }
    }
  }
  uint64_t enq_msgs = 0, enq_bytes = 0, dropped = 0;
  bool tripped = false;
  {
    std::lock_guard<std::mutex> ql(queue_mtx_);
    const bool full_pending = !queue_.empty() && queue_.front().kind == OutMsg::Kind::kFull;
    if (full_pending) {
      // (I3) 대기 중인 FULL 이 shadow 에서 만들어지므로 이 update 들은 거기에 포함된다.
      dropped += items.size();
    } else {
      const bool had_backlog = !queue_.empty();
      for (auto & it : items) {
        enq_msgs++;
        enq_bytes += it.bytes;
        queued_update_bytes_ += it.bytes;
        queue_.push_back(std::move(it));
      }
      // backlog guard: 이미 못 보낸 것이 있는데 한도를 넘으면 전부 버리고 FULL 하나로 바꾼다.
      // (빈 큐에 한 cycle 분량이 한꺼번에 들어오는 것은 허용 — 큐 상한 = max_queue_bytes + 한 cycle 분량)
      if (had_backlog && queued_update_bytes_ > cfg_.max_queue_bytes) {
        tripped = true;
        requestFullQueueLocked(FullReason::kBacklog);
      }
    }
  }
  queue_cv_.notify_all();
  addStats([&](EngineStats & s) {
      s.update_msgs_enqueued += enq_msgs;
      s.update_bytes_enqueued += enq_bytes;
      s.dropped_updates += dropped;
      if (tripped) {
        s.backlog_trips++;
      }
    });
}

uint32_t DiffEngine::chunkRows(uint32_t width) const
{
  if (cfg_.copy_chunk_bytes == 0 || width == 0) {
    return UINT32_MAX;
  }
  return static_cast<uint32_t>(std::max<size_t>(1, cfg_.copy_chunk_bytes / width));
}

void DiffEngine::copyRowsLocked(uint8_t * dst, uint32_t y0, uint32_t y1) const
{
  const size_t W = geom_.size_x;
  std::memcpy(dst + size_t(y0) * W, shadow_.data() + size_t(y0) * W, size_t(y1 - y0) * W);
}

CycleResult DiffEngine::processCycle(
  const nav2_costmap_2d::Costmap2D & master,
  int min_i, int min_j, int max_i, int max_j)
{
  const auto t_start = SteadyClock::now();
  CycleResult res;
  ++cycle_count_;
  const Geometry g = Geometry::of(master);
  if (!g.valid()) {
    return res;
  }
  const uint8_t * m = master.getCharMap();
  const size_t total = size_t(g.size_x) * g.size_y;

  // ---- 첫 cycle / geometry 변경: shadow = master 전체 사본, FULL 요청 ----
  if (!geom_.valid() || !geom_.sameAs(g)) {
    const FullReason reason = geom_.valid() ? FullReason::kGeometry : FullReason::kInitial;
    {
      auto sl = lockShadowForMap();   // [10-10 W1-D][리뷰] 묶음 복사보다 먼저
      shadow_.assign(m, m + total);
      geom_ = g;
      ++geom_gen_;   // [10-10 W1-D][리뷰] 진행 중인 묶음 복사(FULL/snapshot) 를 중단시킨다
      geom_w_.store(g.size_x);
      geom_h_.store(g.size_y);
      reset_pending_.store(false);
      std::lock_guard<std::mutex> ql(queue_mtx_);
      {
        std::lock_guard<std::mutex> cl(catchup_mtx_);
        history_.clear();            // 옛 geometry 의 사각형은 의미 없음
        clearRepairStateLocked();
        sweep_row_ = 0;
      }
      requestFullQueueLocked(reason);
    }
    queue_cv_.notify_all();
    res.full_requested = true;
    res.cycle_ms = msSince(t_start);
    addStats([&](EngineStats & s) {
        s.cycles++;
        s.max_cycle_ms = std::max(s.max_cycle_ms, res.cycle_ms);
      });
    return res;
  }

  const uint32_t W = g.size_x, H = g.size_y;
  Rect bounds;
  bounds.x0 = static_cast<uint32_t>(std::clamp(min_i, 0, static_cast<int>(W)));
  bounds.x1 = static_cast<uint32_t>(std::clamp(max_i, 0, static_cast<int>(W)));
  bounds.y0 = static_cast<uint32_t>(std::clamp(min_j, 0, static_cast<int>(H)));
  bounds.y1 = static_cast<uint32_t>(std::clamp(max_j, 0, static_cast<int>(H)));

  // reset (ClearEntireCostmap) 뒤 첫 cycle 은 지도 전체를 비교한다. audit 도 지도 전체.
  const bool after_reset = reset_pending_.exchange(false);
  const bool audit = !after_reset && cfg_.audit_period_cycles > 0 &&
    (cycle_count_ % cfg_.audit_period_cycles) == 0;
  res.full_compare = after_reset || audit;
  const Rect win = res.full_compare ? Rect{0, 0, W, H} : bounds;
  if (win.empty()) {
    res.cycle_ms = msSince(t_start);
    addStats([&](EngineStats & s) {s.cycles++;});
    return res;
  }

  // ---- 비교: 행 단위 memcmp → 다른 행만 tile 단위 memcmp ----
  const auto t_cmp = SteadyClock::now();
  const uint32_t T = cfg_.tile_size;
  const uint32_t tx0 = win.x0 / T, tx1 = (win.x1 + T - 1) / T;
  const uint32_t ty0 = win.y0 / T, ty1 = (win.y1 + T - 1) / T;
  const uint32_t mw = tx1 - tx0, mh = ty1 - ty0;
  mask_.assign(size_t(mw) * mh, 0);
  if (tile_box_.size() < mask_.size()) {
    tile_box_.resize(mask_.size());   // 값은 mask 가 0→1 이 될 때 쓴다
  }
  const uint8_t * s = shadow_.data();   // writer 는 이 thread 뿐 → lock 없이 읽는다
  uint64_t changed = 0, outside = 0;
  const uint32_t ww = win.x1 - win.x0;
  for (uint32_t y = win.y0; y < win.y1; ++y) {
    const uint8_t * mr = m + size_t(y) * W;
    const uint8_t * sr = s + size_t(y) * W;
    if (std::memcmp(mr + win.x0, sr + win.x0, ww) == 0) {
      continue;
    }
    uint8_t * mrow = mask_.data() + size_t(y / T - ty0) * mw;
    const bool row_out = (y < bounds.y0 || y >= bounds.y1);
    for (uint32_t tx = tx0; tx < tx1; ++tx) {
      const uint32_t xa = std::max(win.x0, tx * T);
      const uint32_t xb = std::min(win.x1, (tx + 1) * T);
      if (std::memcmp(mr + xa, sr + xa, xb - xa) == 0) {
        continue;
      }
      uint32_t fx = UINT32_MAX, lx = 0;
      for (uint32_t x = xa; x < xb; ++x) {
        if (mr[x] != sr[x]) {
          ++changed;
          fx = std::min(fx, x);
          lx = x;
          if (audit && (row_out || x < bounds.x0 || x >= bounds.x1)) {
            ++outside;
          }
        }
      }
      // tile 안에서 실제로 바뀐 셀의 tight box (tile 전체 대신 이것만 보낸다)
      const size_t ti = size_t(y / T - ty0) * mw + (tx - tx0);
      Rect & b = tile_box_[ti];
      if (!mrow[tx - tx0]) {
        mrow[tx - tx0] = 1;
        b = Rect{fx, y, lx + 1, y + 1};
      } else {
        b.x0 = std::min(b.x0, fx);
        b.x1 = std::max(b.x1, lx + 1);
        b.y1 = y + 1;
      }
    }
  }
  res.compare_ms = msSince(t_cmp);
  res.compared_cells = uint64_t(ww) * (win.y1 - win.y0);
  res.changed_cells = changed;
  res.outside_bounds_cells = outside;

  size_t bytes = 0;
  if (changed > 0) {
    tile_box_.resize(mask_.size());
    const auto rects = rectsFromMaskCapped(
      mask_, mw, mh, tx0 * T, ty0 * T, T, win, cfg_.max_rects, &tile_box_);
    uint64_t rect_area = 0;
    for (const auto & r : rects) {
      rect_area += r.area();
    }
    const bool over_ratio = cfg_.full_fallback_ratio > 0.0 &&
      static_cast<double>(rect_area) > cfg_.full_fallback_ratio * static_cast<double>(total);
    // [10-10 W1-D][리뷰] 한 cycle 분량이 max_update_cycle_bytes 를 넘어도 FULL — update durable history 가 크게 남지 않게
    const bool over_bytes = cfg_.max_update_cycle_bytes > 0 && rect_area > cfg_.max_update_cycle_bytes;
    if (over_ratio || over_bytes) {
      // 사각형이 지도 대부분을 덮는다 → 메시지를 master lock 아래에서 만들지 않고, window 를 shadow 에 복사한 뒤 FULL 요청.
      // (FULL 은 publisher thread 가 master lock 밖에서 shadow 로 만든다)
      {
        auto sl = lockShadowForMap();   // [10-10 W1-D][리뷰] 묶음 복사보다 먼저
        for (uint32_t y = win.y0; y < win.y1; ++y) {
          std::memcpy(shadow_.data() + size_t(y) * W + win.x0, m + size_t(y) * W + win.x0, ww);
        }
        std::lock_guard<std::mutex> ql(queue_mtx_);
        requestFullQueueLocked(FullReason::kLargeDiff);
      }
      queue_cv_.notify_all();
      res.full_requested = true;
      res.rects = rects.size();
      res.cycle_ms = msSince(t_start);
      addStats([&](EngineStats & st) {
          st.cycles++;
          st.compared_cells += res.compared_cells;
          st.changed_cells += res.changed_cells;
          st.large_diff_fulls++;
          if (res.full_compare) {
            st.full_compare_cycles++;
          }
          st.outside_bounds_cells += res.outside_bounds_cells;
          st.max_compare_ms = std::max(st.max_compare_ms, res.compare_ms);
          st.max_cycle_ms = std::max(st.max_cycle_ms, res.cycle_ms);
        });
      return res;
    }
    // master 에서 메시지 data 복사 (shadow lock 밖, master lock 은 호출자가 쥐고 있음)
    // [10-10 W1-D][리뷰] 메시지 하나가 max_msg_bytes 를 넘으면 행 band 로 나눈다
    std::vector<Item> items;
    items.reserve(rects.size());
    for (const auto & r : rects) {
      const uint32_t rw = r.x1 - r.x0;
      const uint32_t band = static_cast<uint32_t>(std::max<size_t>(1, cfg_.max_msg_bytes / rw));
      for (uint32_t by = r.y0; by < r.y1; by += band) {
        const uint32_t ey = std::min(r.y1, by + band);
        Item it;
        it.kind = OutMsg::Kind::kUpdate;
        it.update = std::make_unique<nav2_msgs::msg::CostmapUpdate>();
        auto & u = *it.update;
        u.header.frame_id = cfg_.frame_id;
        u.x = r.x0;
        u.y = by;
        u.size_x = rw;
        u.size_y = ey - by;
        u.data.resize(size_t(u.size_x) * u.size_y);
        for (uint32_t y = by; y < ey; ++y) {
          std::memcpy(u.data.data() + size_t(y - by) * u.size_x, m + size_t(y) * W + r.x0, u.size_x);
        }
        it.bytes = u.data.size() + kUpdateHeaderBytes;
        bytes += it.bytes;
        items.push_back(std::move(it));
      }
    }
    {
      auto sl = lockShadowForMap();   // [10-10 W1-D][리뷰] 묶음 복사보다 먼저
      // (I1) shadow 갱신 = 이번에 보내는 사각형만
      for (const auto & r : rects) {
        for (uint32_t y = r.y0; y < r.y1; ++y) {
          std::memcpy(shadow_.data() + size_t(y) * W + r.x0, m + size_t(y) * W + r.x0, r.x1 - r.x0);
        }
      }
      enqueueUpdatesLocked(std::move(items));
    }
    res.rects = rects.size();
    res.bytes = bytes;
  }

  res.cycle_ms = msSince(t_start);
  addStats([&](EngineStats & st) {
      st.cycles++;
      st.compared_cells += res.compared_cells;
      st.changed_cells += res.changed_cells;
      st.rects += res.rects;
      if (res.full_compare) {
        st.full_compare_cycles++;
      }
      st.outside_bounds_cells += res.outside_bounds_cells;
      st.max_compare_ms = std::max(st.max_compare_ms, res.compare_ms);
      st.max_cycle_ms = std::max(st.max_cycle_ms, res.cycle_ms);
    });
  return res;
}

void DiffEngine::fillMetadata(nav2_msgs::msg::Costmap & out, int64_t stamp) const
{
  out.header.frame_id = cfg_.frame_id;
  stampToMsg(stamp, out.header.stamp);
  out.metadata.layer = "master";
  out.metadata.resolution = static_cast<float>(geom_.resolution);
  out.metadata.size_x = geom_.size_x;
  out.metadata.size_y = geom_.size_y;
  out.metadata.origin.position.x = geom_.msg_origin_x;
  out.metadata.origin.position.y = geom_.msg_origin_y;
  out.metadata.origin.position.z = 0.0;
  out.metadata.origin.orientation.w = 1.0;
  out.metadata.map_load_time = out.header.stamp;
  out.metadata.update_time = out.header.stamp;
}

bool DiffEngine::snapshot(nav2_msgs::msg::Costmap & out) const
{
  // [10-10 W1-D][리뷰] 지도 크기 사본을 shadow_mtx_ 한 번에 쥐고 만들지 않는다 (map thread 가 그동안 막힌다).
  //   stamp·metadata 는 시작 때, data 는 copy_chunk_bytes 묶음으로. geometry 가 도중에 바뀌면 다시 (최대 3 번).
  for (int attempt = 0; attempt < 3; ++attempt) {
    uint64_t gen = 0;
    uint32_t W = 0, H = 0;
    {
      yieldToMap();
      std::lock_guard<std::mutex> sl(shadow_mtx_);
      if (!geom_.valid()) {
        return false;
      }
      fillMetadata(out, last_stamp_);
      gen = geom_gen_;
      W = geom_.size_x;
      H = geom_.size_y;
    }
    out.data.resize(size_t(W) * H);   // lock 밖에서 할당 (page fault 포함)
    const uint32_t rows = chunkRows(W);
    bool ok = true;
    for (uint32_t y = 0; y < H; ) {
      const uint32_t ey = rows >= H - y ? H : y + rows;
      yieldToMap();
      std::lock_guard<std::mutex> sl(shadow_mtx_);
      if (geom_gen_ != gen) {
        ok = false;
        break;
      }
      const auto t0 = SteadyClock::now();
      copyRowsLocked(out.data.data(), y, ey);
      noteHold(msSince(t0));
      y = ey;
    }
    if (ok) {
      return true;
    }
  }
  return false;
}

void DiffEngine::pushRepairsLocked(const std::vector<Rect> & rects, RepairSource src)
{
  for (const auto & r : rects) {
    if (!r.empty()) {
      repair_.push_back(RepairRect{r, src});
    }
  }
  // 큐가 계속 차 있어 repair 를 못 보내는 동안 무한히 쌓이지 않게: 넘으면 한데 병합 (출처는 refresh 로)
  const size_t cap = 8u * std::max<uint32_t>(cfg_.max_rects, cfg_.refresh_max_rects) + 64u;
  if (repair_.size() > cap) {
    std::vector<Rect> all;
    all.reserve(repair_.size());
    for (const auto & rr : repair_) {
      all.push_back(rr.r);
    }
    repair_.clear();
    for (const auto & r : compactRects(std::move(all), cfg_.max_rects)) {
      repair_.push_back(RepairRect{r, RepairSource::kRefresh});
    }
    addStats([](EngineStats & s) {s.repair_overflow_merges++;});
  }
  repair_n_.store(repair_.size());
}

void DiffEngine::scheduleRepairs(int64_t now_ns)
{
  const uint32_t W = geom_w_.load(), H = geom_h_.load();
  if (W == 0 || H == 0) {
    return;
  }
  // (1) catch-up 단계 (FULL ack 시각 기준)
  if (catchup_tracking_ && catchup_base_ns_ != 0 && catchup_stage_ < cfg_.catchup_delays_sec.size() &&
    now_ns >= catchup_base_ns_ + secToNs(cfg_.catchup_delays_sec[catchup_stage_]))
  {
    ++catchup_stage_;
    const bool last = catchup_stage_ >= cfg_.catchup_delays_sec.size();
    std::vector<Rect> rects;
    if (last) {
      rects.swap(catchup_rects_);
      catchup_tracking_ = false;
      catchup_base_ns_ = 0;
    } else {
      rects = catchup_rects_;   // 마지막 단계까지 계속 모은다
    }
    // [10-10 W1-D][리뷰] 크기와 상관없이 건너뛰지 않는다 — 보낼 때 max_msg_bytes band 로 나눠 하나씩 shadow 에서 만든다.
    pushRepairsLocked(compactRects(std::move(rects), cfg_.max_rects), RepairSource::kCatchup);
    addStats([](EngineStats & s) {s.catchups++;});
  }
  const bool subs = have_subscribers_.load();
  // (2) refresh: 최근 refresh_rounds 주기 동안 보낸 update 사각형을 한 번 더
  if (cfg_.refresh_period_sec > 0.0) {
    if (refresh_next_ns_ == 0) {
      refresh_next_ns_ = now_ns + secToNs(cfg_.refresh_period_sec);
    } else if (now_ns >= refresh_next_ns_) {
      refresh_next_ns_ = now_ns + secToNs(cfg_.refresh_period_sec);
      refresh_hist_.push_back(std::move(refresh_cur_));
      refresh_cur_.clear();
      while (refresh_hist_.size() > cfg_.refresh_rounds) {
        refresh_hist_.pop_front();
      }
      std::vector<Rect> all;
      for (const auto & v : refresh_hist_) {
        all.insert(all.end(), v.begin(), v.end());
      }
      if (subs && !all.empty()) {
        pushRepairsLocked(compactRects(std::move(all), cfg_.refresh_max_rects), RepairSource::kRefresh);
        addStats([](EngineStats & s) {s.refreshes++;});
      }
    }
  }
  // (3) sweep: 지도 전체를 sweep_period_sec 에 한 바퀴
  if (cfg_.sweep_period_sec > 0.0) {
    if (sweep_next_ns_ == 0) {
      sweep_next_ns_ = now_ns + secToNs(cfg_.sweep_tick_sec);
    } else if (now_ns >= sweep_next_ns_) {
      sweep_next_ns_ = now_ns + secToNs(cfg_.sweep_tick_sec);
      if (subs) {
        const double ticks = std::max(1.0, cfg_.sweep_period_sec / cfg_.sweep_tick_sec);
        const uint32_t rows = static_cast<uint32_t>(
          std::clamp(std::ceil(double(H) / ticks), 1.0, double(H)));
        if (sweep_row_ >= H) {
          sweep_row_ = 0;
        }
        const Rect band{0, sweep_row_, W, std::min(H, sweep_row_ + rows)};
        sweep_row_ = band.y1;
        if (sweep_row_ >= H) {
          sweep_row_ = 0;
          addStats([](EngineStats & s) {s.sweeps_completed++;});
        }
        pushRepairsLocked({band}, RepairSource::kSweep);
      }
    }
  }
}

bool DiffEngine::popRepair(OutMsg & out)
{
  yieldToMap();
  std::lock_guard<std::mutex> sl(shadow_mtx_);
  std::unique_lock<std::mutex> ql(queue_mtx_);
  if (!queue_.empty()) {
    return false;   // 큐가 먼저 (다음 pop 에서)
  }
  std::lock_guard<std::mutex> cl(catchup_mtx_);
  if (!geom_.valid()) {
    repair_.clear();
    repair_n_.store(0);
    return false;
  }
  const uint32_t W = geom_.size_x, H = geom_.size_y;
  while (!repair_.empty()) {
    RepairRect rr = repair_.front();
    repair_.pop_front();
    Rect r = clampRect(rr.r, W, H);
    if (r.empty()) {
      continue;
    }
    // 큰 사각형은 위에서부터 max_msg_bytes 행 band 로 잘라 하나씩 (나머지는 다시 맨 앞)
    const uint32_t rw = r.x1 - r.x0;
    const uint32_t band = static_cast<uint32_t>(std::max<size_t>(1, cfg_.max_msg_bytes / rw));
    if (r.y1 - r.y0 > band) {
      repair_.push_front(RepairRect{Rect{r.x0, r.y0 + band, r.x1, r.y1}, rr.src});
      r.y1 = r.y0 + band;
    }
    repair_n_.store(repair_.size());
    const auto t0 = SteadyClock::now();
    auto u = std::make_unique<nav2_msgs::msg::CostmapUpdate>();
    u->header.frame_id = cfg_.frame_id;
    u->x = r.x0;
    u->y = r.y0;
    u->size_x = rw;
    u->size_y = r.y1 - r.y0;
    u->data.resize(size_t(u->size_x) * u->size_y);
    for (uint32_t y = r.y0; y < r.y1; ++y) {
      std::memcpy(u->data.data() + size_t(y - r.y0) * rw, shadow_.data() + size_t(y) * W + r.x0, rw);
    }
    // (I4) 큐가 비어 있고 shadow lock 을 쥐고 있으므로 이 stamp 는 지금까지 보낸 것보다 크고 이후 것보다 작다
    stampToMsg(nextStampLocked(), u->header.stamp);
    if (catchup_tracking_) {
      catchup_rects_.push_back(r);
    }
    noteHold(msSince(t0));
    out.kind = OutMsg::Kind::kUpdate;
    out.repair = true;
    out.src = rr.src;
    out.bytes = u->data.size() + kUpdateHeaderBytes;
    out.update = std::move(u);
    return true;
  }
  repair_n_.store(0);
  return false;
}

bool DiffEngine::popFull(OutMsg & out)
{
  // [10-10 W1-D][리뷰] (F4) 지도 크기 버퍼는 lock 밖에서 할당하고, 복사는 copy_chunk_bytes 행 묶음마다 shadow_mtx_ 를 놓는다.
  //   (F6) 할당이 throw 하면 FULL token 은 큐에 그대로 남는다 → publisher thread 가 잡아서 나중에 다시 시도.
  for (int attempt = 0; attempt < 3; ++attempt) {
    const uint32_t W0 = geom_w_.load(), H0 = geom_h_.load();
    const size_t total = size_t(W0) * H0;
    if (total == 0) {
      std::lock_guard<std::mutex> ql(queue_mtx_);
      if (!queue_.empty() && queue_.front().kind == OutMsg::Kind::kFull) {
        queue_.pop_front();   // 아직 첫 cycle 전 — 첫 cycle 이 다시 요청한다
      }
      return false;
    }
    if (cfg_.test_before_full_alloc) {
      cfg_.test_before_full_alloc(total);
    }
    auto f = std::make_unique<nav2_msgs::msg::Costmap>();
    f->data.resize(total);
    uint64_t gen = 0;
    uint32_t y = 0;
    const uint32_t rows = chunkRows(W0);
    FullReason reason = FullReason::kManual;
    {
      yieldToMap();
      std::lock_guard<std::mutex> sl(shadow_mtx_);
      std::unique_lock<std::mutex> ql(queue_mtx_);
      if (queue_.empty()) {
        return false;
      }
      if (queue_.front().kind == OutMsg::Kind::kUpdate) {
        Item it = std::move(queue_.front());
        queue_.pop_front();
        queued_update_bytes_ -= std::min(queued_update_bytes_, it.bytes);
        out.kind = OutMsg::Kind::kUpdate;
        out.bytes = it.bytes;
        out.update = std::move(it.update);
        return true;
      }
      if (geom_.size_x != W0 || geom_.size_y != H0) {
        continue;   // 그 사이 geometry 가 바뀜 — 새 크기로 다시
      }
      const auto t0 = SteadyClock::now();
      reason = queue_.front().reason;
      uint64_t dropped = 0;
      for (const auto & it : queue_) {
        if (it.kind == OutMsg::Kind::kUpdate) {
          ++dropped;
        }
      }
      // (I3) 지금 shadow 는 큐에 넣은 모든 것을 포함한다 → 남은 update 는 전부 FULL 에 포함
      queue_.clear();
      queued_update_bytes_ = 0;
      if (dropped) {
        addStats([&](EngineStats & s) {s.dropped_updates += dropped;});
      }
      fillMetadata(*f, nextStampLocked());
      gen = geom_gen_;
      {
        std::lock_guard<std::mutex> cl(catchup_mtx_);
        // FULL 이 그 전 손실을 모두 덮으므로 대기 중인 repair·refresh 는 버린다. catch-up 수집 시작.
        clearRepairStateLocked();
        if (!cfg_.catchup_delays_sec.empty()) {
          // 늦게 붙은 구독자가 (a) durable history 의 옛 update 를 FULL 뒤에 적용하거나
          // (b) FULL(큰 메시지) 보다 먼저 도착한 새 update 를 FULL 로 덮어쓸 수 있다 → 그 사각형을 나중에 다시 보낸다.
          catchup_tracking_ = true;
          catchup_rects_.assign(history_.begin(), history_.end());
        }
      }
      const uint32_t ey = rows >= H0 ? H0 : rows;
      copyRowsLocked(f->data.data(), 0, ey);
      y = ey;
      noteHold(msSince(t0));
    }
    bool aborted = false;
    while (y < H0) {
      if (cfg_.test_between_full_chunks) {
        cfg_.test_between_full_chunks();
      }
      yieldToMap();
      std::lock_guard<std::mutex> sl(shadow_mtx_);
      if (geom_gen_ != gen) {
        aborted = true;   // geometry 변경: map thread 가 새 FULL 을 요청해 두었다
        break;
      }
      {
        std::lock_guard<std::mutex> ql(queue_mtx_);
        if (!queue_.empty() && queue_.front().kind == OutMsg::Kind::kFull) {
          aborted = true;   // backlog / large diff 등으로 새 FULL 요청 — 그것이 이 FULL 을 대신한다
          break;
        }
      }
      const auto t0 = SteadyClock::now();
      const uint32_t ey = rows >= H0 - y ? H0 : y + rows;
      copyRowsLocked(f->data.data(), y, ey);
      y = ey;
      noteHold(msSince(t0));
    }
    if (aborted) {
      addStats([](EngineStats & s) {s.full_aborts++;});
      return false;
    }
    out.kind = OutMsg::Kind::kFull;
    out.reason = reason;
    out.full = std::move(f);
    out.bytes = out.full->data.size();
    return true;
  }
  return false;
}

bool DiffEngine::pop(OutMsg & out, SteadyClock::time_point now)
{
  out = OutMsg{};
  {
    std::lock_guard<std::mutex> cl(catchup_mtx_);
    scheduleRepairs(steadyNs(now));
  }
  {
    std::unique_lock<std::mutex> ql(queue_mtx_);
    if (!queue_.empty()) {
      if (queue_.front().kind == OutMsg::Kind::kUpdate) {
        Item it = std::move(queue_.front());
        queue_.pop_front();
        queued_update_bytes_ -= std::min(queued_update_bytes_, it.bytes);
        out.kind = OutMsg::Kind::kUpdate;
        out.bytes = it.bytes;
        out.update = std::move(it.update);
        return true;
      }
      ql.unlock();
      return popFull(out);
    }
  }
  if (repair_n_.load() == 0) {
    return false;
  }
  return popRepair(out);
}

bool DiffEngine::waitPop(OutMsg & out, std::chrono::milliseconds timeout)
{
  {
    std::unique_lock<std::mutex> ql(queue_mtx_);
    queue_cv_.wait_for(
      ql, timeout, [this] {return !queue_.empty() || repair_n_.load() > 0 || stopping_.load();});
  }
  if (stopping_.load()) {
    return false;
  }
  return pop(out, SteadyClock::now());
}

void DiffEngine::setStopping(bool stopping)
{
  {
    std::lock_guard<std::mutex> ql(queue_mtx_);
    stopping_.store(stopping);
  }
  queue_cv_.notify_all();
}

void DiffEngine::onPublished(const OutMsg & m, SteadyClock::time_point now)
{
  if (m.kind == OutMsg::Kind::kFull) {
    {
      std::lock_guard<std::mutex> cl(catchup_mtx_);
      if (catchup_tracking_ && !cfg_.catchup_delays_sec.empty()) {
        // [10-10 W1-D][리뷰] now = layer 가 wait_for_all_acked 를 기다린 뒤 → 모든 reader 가 FULL 을 받은 뒤부터 잰다
        catchup_base_ns_ = steadyNs(now);
        catchup_stage_ = 0;
      }
    }
    addStats([&](EngineStats & s) {
        s.fulls_sent++;
        s.full_bytes_sent += m.bytes;
      });
  } else if (m.kind == OutMsg::Kind::kUpdate && m.update) {
    const auto & u = *m.update;
    const Rect r{u.x, u.y, u.x + u.size_x, u.y + u.size_y};
    {
      std::lock_guard<std::mutex> cl(catchup_mtx_);
      history_.push_back(r);   // durable history 에 남는 것 (repair 포함)
      while (history_.size() > cfg_.history_depth) {
        history_.pop_front();
      }
      if (!m.repair && cfg_.refresh_period_sec > 0.0) {
        refresh_cur_.push_back(r);
        if (refresh_cur_.size() > 8u * cfg_.refresh_max_rects + 64u) {
          refresh_cur_ = compactRects(std::move(refresh_cur_), cfg_.refresh_max_rects);
        }
      }
    }
    addStats([&](EngineStats & s) {
        s.update_msgs_sent++;
        s.update_bytes_sent += m.bytes;
        switch (m.src) {
          case RepairSource::kCatchup: s.catchup_msgs++; s.catchup_bytes += m.bytes; break;
          case RepairSource::kRefresh: s.refresh_msgs++; s.refresh_bytes += m.bytes; break;
          case RepairSource::kSweep: s.sweep_msgs++; s.sweep_bytes += m.bytes; break;
          case RepairSource::kRetry: s.retry_msgs++; break;
          case RepairSource::kNone: break;
        }
      });
  }
}

void DiffEngine::onPublishFailed(const OutMsg & m)
{
  if (m.kind == OutMsg::Kind::kFull) {
    {
      std::lock_guard<std::mutex> cl(catchup_mtx_);
      clearRepairStateLocked();   // catch-up 수집을 멈춘다 (다음 FULL 이 다시 시작)
    }
    addStats([](EngineStats & s) {s.full_publish_failures++;});
    requestFull(FullReason::kRetry);
  } else if (m.kind == OutMsg::Kind::kUpdate && m.update) {
    const auto & u = *m.update;
    {
      std::lock_guard<std::mutex> cl(catchup_mtx_);
      pushRepairsLocked({Rect{u.x, u.y, u.x + u.size_x, u.y + u.size_y}}, RepairSource::kRetry);
    }
    addStats([](EngineStats & s) {s.update_publish_failures++;});
    queue_cv_.notify_all();
  }
}

void DiffEngine::noteFullAck(double ms, bool acked)
{
  addStats([&](EngineStats & s) {
      s.max_full_ack_ms = std::max(s.max_full_ack_ms, ms);
      if (!acked) {
        s.full_ack_timeouts++;
      }
    });
}

}  // namespace ammr_costmap_diff_publisher
