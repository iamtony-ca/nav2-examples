// [10-10 W1-D] DiffEngine 단위 테스트 (ROS 통신 없음).
//   (1) bounds 안 무작위 편집 다수 cycle → 구독자 mirror([full + updates] 순서 적용) == master
//   (2) clear: 먼 곳 ghost(254 + inflation) → reset + 재계산 → update 가 ghost 를 덮고 mirror == master, 보낸 byte ∝ ghost 면적
//   (3) bounds 안 변경 없음 / 같은 값 덮어쓰기 → update 없음
//   (4) geometry 변경 → FULL
//   (5) backlog guard → FULL 하나
//   (6) stamp 엄격 증가 (시계가 멈추거나 뒤로 가도)
//   (7) 성능 20000×10000
//   (+) catch-up: FULL 과 update 순서가 뒤바뀐 구독자 / 늦게 붙은 구독자가 catch-up 뒤 master 와 같아짐
//   [10-10 W1-D][리뷰] (R1) catch-up 2 단계 (1 단계 뒤 FULL 이 덮어도 2 단계가 메움), (R2) catch-up 은 크기와 상관없이
//     건너뛰지 않고 max_msg_bytes band 로 나눠 보냄, (R3) refresh 가 잃은 update 를 메움 (2 round), (R4) sweep 이 refresh
//     없이도 한 바퀴 안에 메움, (R5) publish 실패 → FULL 재요청 / update repair, (R6) FULL 버퍼 할당 실패 → token 유지,
//     (R7) 묶음 복사 중 update·geometry 변경·새 FULL, (R8) 메시지 크기·cycle 크기 상한, (R9) thread 3 개 동시 (FULL·
//     snapshot·map) 뒤 mirror == master, (R10) 성능: 2e8 지도에서 FULL·snapshot 이 도는 동안 map thread 최대 cycle 시간
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <cstring>
#include <atomic>
#include <mutex>
#include <new>
#include <random>
#include <thread>
#include <vector>

#include "ammr_costmap_diff_publisher/diff_engine.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

using ammr_costmap_diff_publisher::DiffEngine;
using ammr_costmap_diff_publisher::FullReason;
using ammr_costmap_diff_publisher::OutMsg;
using ammr_costmap_diff_publisher::Rect;
using nav2_costmap_2d::Costmap2D;
using SteadyClock = std::chrono::steady_clock;

namespace
{

int64_t stampNs(const builtin_interfaces::msg::Time & t)
{
  return int64_t(t.sec) * 1000000000LL + t.nanosec;
}

// 구독자 쪽 mirror: 받은 순서대로 full / update 를 적용한다 (CostmapSubscriber 와 같은 방식).
struct Mirror
{
  uint32_t sx{0}, sy{0};
  std::vector<uint8_t> data;
  int64_t last_stamp{-1};
  bool stamps_ok{true};
  size_t fulls{0}, updates{0}, update_bytes{0};

  void apply(const OutMsg & m)
  {
    if (m.kind == OutMsg::Kind::kFull) {
      check(stampNs(m.full->header.stamp));
      sx = m.full->metadata.size_x;
      sy = m.full->metadata.size_y;
      data = m.full->data;
      ++fulls;
    } else if (m.kind == OutMsg::Kind::kUpdate) {
      check(stampNs(m.update->header.stamp));
      applyUpdate(*m.update);
    }
  }
  void applyUpdate(const nav2_msgs::msg::CostmapUpdate & u)
  {
    ASSERT_LE(u.x + u.size_x, sx);
    ASSERT_LE(u.y + u.size_y, sy);
    for (uint32_t y = 0; y < u.size_y; ++y) {
      std::memcpy(data.data() + size_t(u.y + y) * sx + u.x, u.data.data() + size_t(y) * u.size_x, u.size_x);
    }
    ++updates;
    update_bytes += u.data.size();
  }
  void check(int64_t s)
  {
    if (s <= last_stamp) {
      stamps_ok = false;
    }
    last_stamp = s;
  }
  bool equals(const Costmap2D & c) const
  {
    return sx == c.getSizeInCellsX() && sy == c.getSizeInCellsY() &&
           std::memcmp(data.data(), c.getCharMap(), data.size()) == 0;
  }
};

struct FakeClock
{
  int64_t t{1000000000LL};
  bool frozen{false};
  int64_t operator()()
  {
    if (!frozen) {
      t += 1000;
    }
    return t;
  }
};

// 큐를 전부 꺼내 mirror 에 적용 (= publish 완료). 꺼낸 메시지 수를 돌려준다.
size_t drain(DiffEngine & e, Mirror & mir, SteadyClock::time_point now = SteadyClock::now())
{
  size_t n = 0;
  OutMsg m;
  while (e.pop(m, now)) {
    mir.apply(m);
    e.onPublished(m, now);
    ++n;
  }
  return n;
}

DiffEngine::Config cfg(uint32_t tile = 64, uint32_t max_rects = 256)
{
  DiffEngine::Config c;
  c.tile_size = tile;
  c.max_rects = max_rects;
  c.catchup_delays_sec.clear();   // 기본 테스트는 catch-up·refresh·sweep 끔 (따로 시험)
  c.refresh_period_sec = 0.0;
  c.sweep_period_sec = 0.0;
  c.frame_id = "map";
  return c;
}

void cycleAll(DiffEngine & e, const Costmap2D & m)
{
  e.processCycle(m, 0, 0, m.getSizeInCellsX(), m.getSizeInCellsY());
}

}  // namespace

// ---------------------------------------------------------------------------
// (rects) 사각형 추출 기본 성질
// ---------------------------------------------------------------------------
TEST(RectExtraction, CoversMaskAndRespectsCap)
{
  std::mt19937 rng(7);
  for (int iter = 0; iter < 200; ++iter) {
    const uint32_t w = 1 + rng() % 40, h = 1 + rng() % 40, T = 1 + rng() % 9;
    std::vector<uint8_t> mask(size_t(w) * h);
    const int density = rng() % 100;
    for (auto & v : mask) {
      v = (int(rng() % 100) < density) ? 1 : 0;
    }
    const Rect clip{0, 0, w * T, h * T};
    for (uint32_t cap : {1u, 3u, 16u, 100000u}) {
      auto rects = ammr_costmap_diff_publisher::rectsFromMaskCapped(mask, w, h, 0, 0, T, clip, cap);
      ASSERT_LE(rects.size(), cap);
      // 모든 dirty tile 의 모든 셀이 어떤 사각형 안에 있어야 한다
      std::vector<uint8_t> cov(size_t(w * T) * h * T, 0);
      for (const auto & r : rects) {
        for (uint32_t y = r.y0; y < r.y1; ++y) {
          for (uint32_t x = r.x0; x < r.x1; ++x) {
            cov[size_t(y) * w * T + x] = 1;
          }
        }
      }
      for (uint32_t ty = 0; ty < h; ++ty) {
        for (uint32_t tx = 0; tx < w; ++tx) {
          if (mask[ty * w + tx]) {
            for (uint32_t y = ty * T; y < (ty + 1) * T; ++y) {
              for (uint32_t x = tx * T; x < (tx + 1) * T; ++x) {
                ASSERT_TRUE(cov[size_t(y) * w * T + x]);
              }
            }
          }
        }
      }
      if (cap == 100000u) {
        // 상한이 없으면 정확히 dirty tile 만 덮는다 (겹침 없음)
        uint64_t area = 0, dirty = 0;
        for (const auto & r : rects) {
          area += r.area();
        }
        for (auto v : mask) {
          dirty += v;
        }
        EXPECT_EQ(area, dirty * T * T);
      }
    }
  }
}

// ---------------------------------------------------------------------------
// (1) 무작위 편집
// ---------------------------------------------------------------------------
TEST(DiffEngine, RandomEditsMirrorEqualsMasterEveryCycle)
{
  for (uint32_t tile : {1u, 7u, 16u, 64u}) {
    for (uint32_t max_rects : {1u, 4u, 256u}) {
      std::mt19937 rng(tile * 1000 + max_rects);
      Costmap2D master(517, 389, 0.05, -3.0, 2.0);
      unsigned char * d = master.getCharMap();
      for (size_t i = 0; i < size_t(517) * 389; ++i) {
        d[i] = (rng() % 10 == 0) ? 254 : 0;
      }
      FakeClock clk;
      DiffEngine e(cfg(tile, max_rects), std::ref(clk));
      Mirror mir;
      cycleAll(e, master);
      drain(e, mir);
      ASSERT_EQ(mir.fulls, 1u);
      ASSERT_TRUE(mir.equals(master));
      for (int c = 0; c < 150; ++c) {
        const uint32_t x0 = rng() % 517, y0 = rng() % 389;
        const uint32_t x1 = std::min<uint32_t>(517, x0 + 1 + rng() % 200);
        const uint32_t y1 = std::min<uint32_t>(389, y0 + 1 + rng() % 200);
        const int edits = rng() % 300;
        for (int k = 0; k < edits; ++k) {
          const uint32_t x = x0 + rng() % (x1 - x0), y = y0 + rng() % (y1 - y0);
          master.setCost(x, y, static_cast<unsigned char>(rng() % 256));
        }
        e.processCycle(master, x0, y0, x1, y1);
        drain(e, mir);
        ASSERT_TRUE(mir.equals(master)) << "tile " << tile << " max_rects " << max_rects << " cycle " << c;
      }
      EXPECT_EQ(mir.fulls, 1u);
      EXPECT_TRUE(mir.stamps_ok);
      auto st = e.takeStats();
      EXPECT_EQ(st.backlog_trips, 0u);
    }
  }
}

// ---------------------------------------------------------------------------
// (2) clear 시나리오
// ---------------------------------------------------------------------------
namespace
{
void paintGhost(Costmap2D & m, int cx, int cy, int r)
{
  for (int y = cy - r; y <= cy + r; ++y) {
    for (int x = cx - r; x <= cx + r; ++x) {
      const double dd = std::hypot(x - cx, y - cy);
      if (dd > r) {
        continue;
      }
      unsigned char v = dd < 2.0 ? 254 : static_cast<unsigned char>(253 - (dd - 2.0) / r * 200);
      m.setCost(x, y, std::max(m.getCost(x, y), v));
    }
  }
}
}  // namespace

TEST(DiffEngine, ClearRemovesFarGhostsWithBytesProportionalToGhostArea)
{
  const uint32_t W = 4000, H = 2000;
  Costmap2D base(W, H, 0.03, 0.0, 0.0);
  // 정적 벽
  for (uint32_t x = 0; x < W; ++x) {
    base.setCost(x, 0, 254);
    base.setCost(x, H - 1, 254);
    base.setCost(x, H / 2, (x % 400 < 300) ? 254 : 0);
  }
  Costmap2D master(base);
  FakeClock clk;
  DiffEngine e(cfg(64, 256), std::ref(clk));
  Mirror mir;
  cycleAll(e, master);
  drain(e, mir);
  ASSERT_TRUE(mir.equals(master));

  // 먼 곳 ghost 3 개 (반경 12 셀 inflation) — 그때의 bounds 는 ghost 주변만
  const int ghosts[3][2] = {{3500, 1700}, {300, 1800}, {3800, 200}};
  const int R = 12;
  for (auto & g : ghosts) {
    paintGhost(master, g[0], g[1], R);
    e.processCycle(master, g[0] - R, g[1] - R, g[0] + R + 1, g[1] + R + 1);
  }
  drain(e, mir);
  ASSERT_TRUE(mir.equals(master));

  // 로봇 근처 작은 변경 몇 cycle
  for (int c = 0; c < 5; ++c) {
    master.setCost(100 + c, 100, 254);
    e.processCycle(master, 90, 90, 130, 110);
  }
  drain(e, mir);
  Costmap2D base2(master);   // ghost 가 빠진 "다시 계산된" 결과 = base + 로봇 근처 변경
  for (auto & g : ghosts) {
    for (int y = g[1] - R; y <= g[1] + R; ++y) {
      for (int x = g[0] - R; x <= g[0] + R; ++x) {
        base2.setCost(x, y, base.getCost(x, y));
      }
    }
  }
  e.takeStats();
  const size_t upd_bytes_before = mir.update_bytes;

  // ClearEntireCostmap: resetLayers 가 master 전체를 default 로 → layer reset() → 다음 updateMap 이 다시 채움.
  // (a) static layer 처럼 다음 bounds 가 지도 전체인 경우
  master.resetMap(0, 0, W, H);
  e.markResetPending();
  std::memcpy(master.getCharMap(), base2.getCharMap(), size_t(W) * H);
  const auto r = e.processCycle(master, 0, 0, W, H);
  drain(e, mir);
  EXPECT_TRUE(mir.equals(master));
  const size_t sent = mir.update_bytes - upd_bytes_before;
  const size_t ghost_area = 3 * (2 * R + 1) * (2 * R + 1);
  std::printf(
    "[clear] map %ux%u = %.1f MB, ghost cells %lu, changed %lu, rects %zu, sent %zu B (%.4f%% of map), compare %.2f ms\n",
    W, H, W * double(H) / 1048576.0, (unsigned long)ghost_area, (unsigned long)r.changed_cells, r.rects,
    sent, 100.0 * sent / (double(W) * H), r.compare_ms);
  EXPECT_GT(r.changed_cells, 0u);
  EXPECT_LE(r.changed_cells, ghost_area);
  // 각 ghost 는 최대 2×2 tile(64) 에 걸친다
  EXPECT_LE(sent, 3u * 4u * 64u * 64u);
  EXPECT_LT(sent, size_t(W) * H / 100);

  // (b) static layer 없이 다음 bounds 가 작은 경우에도 reset 뒤 첫 cycle 은 지도 전체 비교로 덮는다
  for (auto & g : ghosts) {
    paintGhost(master, g[0], g[1], R);
    e.processCycle(master, g[0] - R, g[1] - R, g[0] + R + 1, g[1] + R + 1);
  }
  drain(e, mir);
  ASSERT_TRUE(mir.equals(master));
  master.resetMap(0, 0, W, H);
  e.markResetPending();
  std::memcpy(master.getCharMap(), base2.getCharMap(), size_t(W) * H);
  auto r2 = e.processCycle(master, 90, 90, 130, 110);   // 작은 bounds
  drain(e, mir);
  EXPECT_TRUE(r2.full_compare);
  EXPECT_TRUE(mir.equals(master));
  EXPECT_TRUE(mir.stamps_ok);
  EXPECT_EQ(mir.fulls, 1u);
}

// ---------------------------------------------------------------------------
// (3) 변경 없음
// ---------------------------------------------------------------------------
TEST(DiffEngine, UnchangedCellsProduceNoUpdate)
{
  Costmap2D master(300, 200, 0.05, 0.0, 0.0);
  for (uint32_t i = 0; i < 300; ++i) {
    master.setCost(i, 50, 254);
  }
  FakeClock clk;
  DiffEngine e(cfg(), std::ref(clk));
  Mirror mir;
  cycleAll(e, master);
  ASSERT_EQ(drain(e, mir), 1u);
  for (int c = 0; c < 20; ++c) {
    // 같은 값 덮어쓰기
    for (uint32_t i = 0; i < 300; ++i) {
      master.setCost(i, 50, 254);
    }
    auto r = e.processCycle(master, 0, 0, 300, 200);
    EXPECT_EQ(r.changed_cells, 0u);
    EXPECT_EQ(r.rects, 0u);
    EXPECT_EQ(e.queueLength(), 0u);
  }
  EXPECT_EQ(drain(e, mir), 0u);
}

// ---------------------------------------------------------------------------
// (4) geometry 변경
// ---------------------------------------------------------------------------
TEST(DiffEngine, GeometryChangeSendsFull)
{
  Costmap2D master(200, 100, 0.05, 0.0, 0.0);
  FakeClock clk;
  DiffEngine e(cfg(), std::ref(clk));
  Mirror mir;
  cycleAll(e, master);
  drain(e, mir);
  master.setCost(10, 10, 254);
  e.processCycle(master, 0, 0, 20, 20);
  drain(e, mir);
  ASSERT_EQ(mir.fulls, 1u);

  master.resizeMap(250, 120, 0.05, 0.0, 0.0);   // 크기
  master.setCost(240, 110, 254);
  auto r = e.processCycle(master, 0, 0, 5, 5);
  EXPECT_TRUE(r.full_requested);
  EXPECT_TRUE(e.fullPending());
  drain(e, mir);
  EXPECT_EQ(mir.fulls, 2u);
  EXPECT_TRUE(mir.equals(master));

  master.updateOrigin(1.5, 0.0);                 // origin 만
  r = e.processCycle(master, 0, 0, 5, 5);
  EXPECT_TRUE(r.full_requested);
  drain(e, mir);
  EXPECT_EQ(mir.fulls, 3u);
  EXPECT_TRUE(mir.equals(master));

  master.resizeMap(250, 120, 0.10, 1.5, 0.0);    // 해상도
  r = e.processCycle(master, 0, 0, 5, 5);
  EXPECT_TRUE(r.full_requested);
  OutMsg m;
  ASSERT_TRUE(e.pop(m, SteadyClock::now()));
  ASSERT_EQ(m.kind, OutMsg::Kind::kFull);
  EXPECT_EQ(m.reason, FullReason::kGeometry);
  EXPECT_FLOAT_EQ(m.full->metadata.resolution, 0.10f);
  // Costmap2DPublisher::prepareCostmap 과 같은 origin 식
  double wx, wy;
  master.mapToWorld(0, 0, wx, wy);
  EXPECT_DOUBLE_EQ(m.full->metadata.origin.position.x, wx - 0.10 / 2);
  EXPECT_EQ(m.full->header.frame_id, "map");
}

// ---------------------------------------------------------------------------
// (5) backlog guard
// ---------------------------------------------------------------------------
TEST(DiffEngine, BacklogGuardCollapsesToSingleFull)
{
  Costmap2D master(400, 400, 0.05, 0.0, 0.0);
  FakeClock clk;
  auto c = cfg(16, 256);
  c.max_queue_bytes = 20 * 1024;
  DiffEngine e(c, std::ref(clk));
  Mirror mir;
  cycleAll(e, master);
  drain(e, mir);
  std::mt19937 rng(3);
  // publisher 가 멈춘 상황: 꺼내지 않고 계속 쌓는다
  for (int cyc = 0; cyc < 40; ++cyc) {
    for (int k = 0; k < 50; ++k) {
      master.setCost(rng() % 400, rng() % 400, static_cast<unsigned char>(1 + rng() % 254));
    }
    e.processCycle(master, 0, 0, 400, 400);
    EXPECT_LE(e.queuedUpdateBytes(), c.max_queue_bytes + 400u * 400u + 256u * 64u);
  }
  auto st = e.takeStats();
  EXPECT_GE(st.backlog_trips, 1u);
  EXPECT_GT(st.dropped_updates, 0u);
  EXPECT_TRUE(e.fullPending());
  EXPECT_EQ(e.queueLength(), 1u);   // FULL token 하나만 (그 뒤 update 는 FULL 에 포함되어 버림)
  OutMsg m;
  ASSERT_TRUE(e.pop(m, SteadyClock::now()));
  EXPECT_EQ(m.kind, OutMsg::Kind::kFull);
  EXPECT_EQ(m.reason, FullReason::kBacklog);
  mir.apply(m);
  e.onPublished(m, SteadyClock::now());
  EXPECT_TRUE(mir.equals(master));
  EXPECT_EQ(e.queueLength(), 0u);
  // 그 뒤로는 다시 update
  master.setCost(1, 1, 77);
  e.processCycle(master, 0, 0, 4, 4);
  drain(e, mir);
  EXPECT_TRUE(mir.equals(master));
  EXPECT_TRUE(mir.stamps_ok);
}

// ---------------------------------------------------------------------------
// (6) stamp
// ---------------------------------------------------------------------------
TEST(DiffEngine, StampsStrictlyIncreasingEvenWithFrozenOrBackwardClock)
{
  Costmap2D master(100, 100, 0.05, 0.0, 0.0);
  FakeClock clk;
  clk.frozen = true;
  DiffEngine e(cfg(8, 256), std::ref(clk));
  Mirror mir;
  cycleAll(e, master);
  drain(e, mir);
  for (int c = 0; c < 30; ++c) {
    for (int k = 0; k < 10; ++k) {
      master.setCost((c * 7 + k * 13) % 100, (c * 11 + k * 3) % 100, static_cast<unsigned char>(c + k + 1));
    }
    if (c == 10) {
      clk.t -= 5000000000LL;   // 시계가 뒤로 (sim time reset 흉내)
    }
    if (c == 20) {
      e.requestFull(FullReason::kManual);
    }
    e.processCycle(master, 0, 0, 100, 100);
    drain(e, mir);
  }
  EXPECT_TRUE(mir.stamps_ok);
  EXPECT_TRUE(mir.equals(master));
  nav2_msgs::msg::Costmap snap;
  ASSERT_TRUE(e.snapshot(snap));
  EXPECT_EQ(stampNs(snap.header.stamp), mir.last_stamp);   // 서비스 stamp = 마지막으로 큐에 넣은 메시지
  EXPECT_EQ(0, std::memcmp(snap.data.data(), master.getCharMap(), snap.data.size()));
}

// ---------------------------------------------------------------------------
// (+) catch-up
// ---------------------------------------------------------------------------
TEST(DiffEngine, CatchupHealsReorderedAndLateJoiningSubscribers)
{
  Costmap2D master(300, 300, 0.05, 0.0, 0.0);
  FakeClock clk;
  auto c = cfg(16, 256);
  c.catchup_delays_sec = {1.0};
  c.history_depth = 50;
  DiffEngine e(c, std::ref(clk));
  Mirror on_time;   // 정상 순서로 다 받은 구독자
  auto t0 = SteadyClock::now();
  cycleAll(e, master);
  drain(e, on_time, t0);

  // durable history 에 남을 옛 update 들
  std::vector<nav2_msgs::msg::CostmapUpdate> history;
  for (int k = 0; k < 10; ++k) {
    master.setCost(10 + k * 20, 10, 254);
    e.processCycle(master, 0, 0, 300, 20);
    OutMsg m;
    while (e.pop(m, t0)) {
      on_time.apply(m);
      e.onPublished(m, t0);
      history.push_back(*m.update);
    }
  }
  // 이 셀들을 다시 바꿈 → 옛 history 값은 이제 stale
  for (int k = 0; k < 10; ++k) {
    master.setCost(10 + k * 20, 10, 0);
  }
  e.processCycle(master, 0, 0, 300, 20);
  drain(e, on_time, t0);

  // 새 구독자 → FULL
  e.requestFull(FullReason::kNewSubscriber);
  OutMsg full;
  ASSERT_TRUE(e.pop(full, t0));
  ASSERT_EQ(full.kind, OutMsg::Kind::kFull);
  on_time.apply(full);
  e.onPublished(full, t0);

  // FULL 뒤 새 update (late 구독자에게는 FULL 보다 먼저 도착한다고 가정)
  master.setCost(150, 150, 254);
  e.processCycle(master, 140, 140, 160, 160);
  std::vector<OutMsg> after;
  {
    OutMsg m;
    while (e.pop(m, t0)) {
      on_time.apply(m);
      e.onPublished(m, t0);
      after.push_back(std::move(m));
    }
  }
  ASSERT_FALSE(after.empty());
  ASSERT_TRUE(on_time.equals(master));

  // late 구독자: 새 update 먼저 → FULL → durable 옛 history (stale) 순으로 받은 최악의 경우
  Mirror late;
  late.sx = 300;
  late.sy = 300;
  late.data.assign(300 * 300, 0);
  for (auto & m : after) {
    late.applyUpdate(*m.update);
  }
  late.data = full.full->data;
  for (auto & u : history) {
    late.applyUpdate(u);
  }
  EXPECT_FALSE(late.equals(master));   // 아직 틀림

  // catch-up 시각 전에는 아무 것도 없음
  OutMsg m;
  EXPECT_FALSE(e.pop(m, t0 + std::chrono::milliseconds(500)));
  // 지나면 catch-up update 가 나온다
  size_t n = 0;
  const auto t1 = t0 + std::chrono::milliseconds(1100);
  while (e.pop(m, t1)) {
    EXPECT_TRUE(m.repair);
    EXPECT_EQ(m.src, ammr_costmap_diff_publisher::RepairSource::kCatchup);
    late.applyUpdate(*m.update);
    on_time.apply(m);
    e.onPublished(m, t1);
    ++n;
  }
  EXPECT_GT(n, 0u);
  EXPECT_TRUE(late.equals(master));
  EXPECT_TRUE(on_time.equals(master));
  EXPECT_TRUE(on_time.stamps_ok);
  auto st = e.takeStats();
  EXPECT_EQ(st.catchups, 1u);
  std::printf("[catchup] %zu msgs, %lu bytes\n", n, (unsigned long)st.catchup_bytes);
}

// ---------------------------------------------------------------------------
// (fallback) 흩어진 변경이 사각형 상한에 걸려 지도 대부분을 덮으면 FULL
// ---------------------------------------------------------------------------
TEST(DiffEngine, LargeScatteredDiffFallsBackToFull)
{
  Costmap2D master(512, 512, 0.05, 0.0, 0.0);
  FakeClock clk;
  auto c = cfg(16, 1);   // 사각형 1 개 상한
  c.full_fallback_ratio = 0.5;
  DiffEngine e(c, std::ref(clk));
  Mirror mir;
  cycleAll(e, master);
  drain(e, mir);
  // 지도 곳곳에 흩어진 9 점 → 사각형 1 개로는 지도의 절반 넘게 덮어야 한다
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      master.setCost(5 + i * 250, 5 + j * 250, 254);
    }
  }
  auto r = e.processCycle(master, 0, 0, 512, 512);
  EXPECT_TRUE(r.full_requested);
  EXPECT_EQ(e.queueLength(), 1u);
  OutMsg m;
  ASSERT_TRUE(e.pop(m, SteadyClock::now()));
  EXPECT_EQ(m.kind, OutMsg::Kind::kFull);
  EXPECT_EQ(m.reason, FullReason::kLargeDiff);
  mir.apply(m);
  e.onPublished(m, SteadyClock::now());
  EXPECT_TRUE(mir.equals(master));
  // 가까운 변경 몇 개는 그대로 update
  master.setCost(100, 100, 7);
  master.setCost(101, 100, 7);
  r = e.processCycle(master, 90, 90, 110, 110);
  EXPECT_FALSE(r.full_requested);
  drain(e, mir);
  EXPECT_TRUE(mir.equals(master));
}

// ---------------------------------------------------------------------------
// (audit) bounds 밖 변경 감시
// ---------------------------------------------------------------------------
TEST(DiffEngine, AuditDetectsOutsideBoundsWrites)
{
  Costmap2D master(200, 200, 0.05, 0.0, 0.0);
  FakeClock clk;
  auto c = cfg(16, 256);
  c.audit_period_cycles = 2;
  DiffEngine e(c, std::ref(clk));
  Mirror mir;
  cycleAll(e, master);   // cycle 1
  drain(e, mir);
  master.setCost(190, 190, 254);         // bounds 밖에 몰래 씀
  auto r = e.processCycle(master, 0, 0, 10, 10);   // cycle 2 = audit
  EXPECT_TRUE(r.full_compare);
  EXPECT_EQ(r.outside_bounds_cells, 1u);
  drain(e, mir);
  EXPECT_TRUE(mir.equals(master));
}

// ===========================================================================
// [10-10 W1-D][리뷰] 리뷰 반영 시험
// ===========================================================================
namespace
{
using ammr_costmap_diff_publisher::RepairSource;
using ms_t = std::chrono::milliseconds;

// 큐·repair 를 now 시각으로 전부 꺼낸다. apply=false 면 구독자가 "잃은" 것으로 친다 (publish 는 된 것).
size_t drainAt(DiffEngine & e, Mirror * mir, SteadyClock::time_point now, std::vector<OutMsg> * keep = nullptr)
{
  size_t n = 0;
  OutMsg m;
  while (e.pop(m, now)) {
    if (mir) {
      mir->apply(m);
    }
    e.onPublished(m, now);
    if (keep) {
      keep->push_back(std::move(m));
    }
    ++n;
  }
  return n;
}

void paintBlock(Costmap2D & m, uint32_t x0, uint32_t y0, uint32_t w, uint32_t h, unsigned char v)
{
  for (uint32_t y = y0; y < y0 + h; ++y) {
    for (uint32_t x = x0; x < x0 + w; ++x) {
      m.setCost(x, y, v);
    }
  }
}
}  // namespace

// (R1) catch-up 1 단계가 FULL 보다 먼저 적용되고 FULL 이 그것을 덮어써도 2 단계가 메운다 (리뷰 F1 시나리오)
TEST(DiffEngineReview, CatchupSecondStageHealsFullArrivingAfterFirstStage)
{
  Costmap2D master(300, 300, 0.05, 0.0, 0.0);
  FakeClock clk;
  auto c = cfg(16, 256);
  c.catchup_delays_sec = {1.0, 5.0};
  DiffEngine e(c, std::ref(clk));
  Mirror on_time;
  const auto t0 = SteadyClock::now();
  cycleAll(e, master);
  drainAt(e, &on_time, t0);
  const std::vector<uint8_t> old_grid = on_time.data;   // smoother 가 가진 옛 grid

  e.requestFull(FullReason::kLargeDiff);
  OutMsg full;
  ASSERT_TRUE(e.pop(full, t0));
  ASSERT_EQ(full.kind, OutMsg::Kind::kFull);
  on_time.apply(full);
  e.onPublished(full, t0);   // (layer 는 ack 뒤 시각을 준다)

  // FULL 을 만든 뒤 바뀐 셀 (smoother 에게는 FULL 보다 먼저 도착)
  paintBlock(master, 100, 100, 10, 10, 254);
  e.processCycle(master, 100, 100, 110, 110);
  std::vector<OutMsg> after;
  drainAt(e, &on_time, t0, &after);
  ASSERT_FALSE(after.empty());

  // smoother: 옛 grid + FULL 뒤 update + 1 단계 catch-up 까지 받은 다음, 늦게 처리된 FULL 이 통째로 덮어씀
  Mirror late;
  late.sx = 300;
  late.sy = 300;
  late.data = old_grid;
  for (auto & m : after) {
    late.applyUpdate(*m.update);
  }
  std::vector<OutMsg> stage1;
  EXPECT_EQ(drainAt(e, &on_time, t0 + ms_t(500)), 0u);
  ASSERT_GT(drainAt(e, &on_time, t0 + ms_t(1100), &stage1), 0u);
  for (auto & m : stage1) {
    EXPECT_EQ(m.src, RepairSource::kCatchup);
    late.applyUpdate(*m.update);
  }
  late.data = full.full->data;   // FULL 이 뒤늦게 적용됨 → 그 뒤 값을 잃는다
  EXPECT_FALSE(late.equals(master));
  EXPECT_EQ(drainAt(e, &on_time, t0 + ms_t(3000)), 0u);
  std::vector<OutMsg> stage2;
  ASSERT_GT(drainAt(e, &on_time, t0 + ms_t(5100), &stage2), 0u);
  for (auto & m : stage2) {
    late.applyUpdate(*m.update);
  }
  EXPECT_TRUE(late.equals(master));
  EXPECT_TRUE(on_time.equals(master));
  EXPECT_TRUE(on_time.stamps_ok);
  // 마지막 단계 뒤에는 더 없음
  EXPECT_EQ(drainAt(e, &on_time, t0 + ms_t(20000)), 0u);
  const auto st = e.takeStats();
  EXPECT_EQ(st.catchups, 2u);
}

// (R2) catch-up 면적이 max_queue_bytes 보다 커도 건너뛰지 않고, max_msg_bytes 이하 band 로 나눠 보낸다
TEST(DiffEngineReview, CatchupIsNeverSkippedAndIsChunked)
{
  Costmap2D master(512, 512, 0.05, 0.0, 0.0);
  FakeClock clk;
  auto c = cfg(16, 256);
  c.catchup_delays_sec = {1.0};
  c.max_queue_bytes = 1024;          // 예전에는 이것을 넘는 catch-up 을 건너뛰었다
  c.max_msg_bytes = 4096;
  c.max_update_cycle_bytes = 50000;
  c.history_depth = 10;
  DiffEngine e(c, std::ref(clk));
  Mirror on_time;
  const auto t0 = SteadyClock::now();
  cycleAll(e, master);
  drainAt(e, &on_time, t0);
  // update 200×100 (20000 B → 4096 B band 5 개) → durable history (depth 10) 에 남는다
  paintBlock(master, 50, 50, 200, 100, 200);
  e.processCycle(master, 50, 50, 250, 150);
  std::vector<OutMsg> hist;
  drainAt(e, &on_time, t0, &hist);
  ASSERT_GE(hist.size(), 5u);
  ASSERT_LE(hist.size(), c.history_depth);
  for (auto & m : hist) {
    EXPECT_LE(m.update->data.size(), c.max_msg_bytes);
  }
  // 같은 곳을 포함한 300×300 변경 = 90000 B > max_update_cycle_bytes → update 없이 FULL (large diff).
  //   → durable history 의 위 update 들은 이 FULL 보다 옛 값 (늦게 붙은 구독자가 FULL 뒤에 받으면 stale)
  paintBlock(master, 40, 40, 300, 300, 7);
  auto r = e.processCycle(master, 40, 40, 340, 340);
  ASSERT_TRUE(r.full_requested);
  OutMsg full;
  ASSERT_TRUE(e.pop(full, t0));
  ASSERT_EQ(full.kind, OutMsg::Kind::kFull);
  EXPECT_EQ(full.reason, FullReason::kLargeDiff);
  on_time.apply(full);
  e.onPublished(full, t0);
  Mirror late;   // FULL → durable 옛 history (stale) 순
  late.sx = 512;
  late.sy = 512;
  late.data = full.full->data;
  for (auto & m : hist) {   // durable replay
    late.applyUpdate(*m.update);
  }
  EXPECT_FALSE(late.equals(master));
  std::vector<OutMsg> cu;
  drainAt(e, &on_time, t0 + ms_t(1100), &cu);
  size_t bytes = 0;
  for (auto & m : cu) {
    EXPECT_LE(m.update->data.size(), c.max_msg_bytes);
    late.applyUpdate(*m.update);
    bytes += m.update->data.size();
  }
  EXPECT_GT(bytes, c.max_queue_bytes);
  EXPECT_TRUE(late.equals(master));
  EXPECT_TRUE(on_time.equals(master));
  EXPECT_TRUE(on_time.stamps_ok);
  std::printf("[R2] catch-up %zu msgs, %zu B (max_queue_bytes %zu, max_msg_bytes %zu)\n",
    cu.size(), bytes, c.max_queue_bytes, c.max_msg_bytes);
}

// (R3) refresh: 구독자가 잃은 update (CostmapSubscriber KeepLast(10) 덮임) 를 다음 refresh 가 메운다.
//      refresh 자체도 잃으면 다음 round 가 한 번 더 보낸다.
TEST(DiffEngineReview, RefreshHealsLostUpdatesWithinRounds)
{
  Costmap2D master(400, 300, 0.05, 0.0, 0.0);
  FakeClock clk;
  auto c = cfg(16, 256);
  c.refresh_period_sec = 1.0;
  c.refresh_rounds = 2;
  c.refresh_max_rects = 8;
  DiffEngine e(c, std::ref(clk));
  Mirror sub;
  const auto t0 = SteadyClock::now();
  cycleAll(e, master);
  drainAt(e, &sub, t0);   // 첫 pop 이 refresh 시각을 t0+1s 로 잡는다
  // 흩어진 30 개 변경: 앞 20 개는 구독자가 잃는다
  std::vector<OutMsg> burst;
  for (int k = 0; k < 30; ++k) {
    const uint32_t x = 10 + (k * 37) % 370, y = 10 + (k * 53) % 270;
    paintBlock(master, x, y, 3, 3, static_cast<unsigned char>(100 + k));
    e.processCycle(master, x, y, x + 3, y + 3);
  }
  drainAt(e, nullptr, t0, &burst);
  ASSERT_GE(burst.size(), 30u);
  for (size_t i = burst.size() - 10; i < burst.size(); ++i) {   // reader KeepLast(10): 마지막 10 개만 남는다
    sub.apply(burst[i]);
  }
  EXPECT_FALSE(sub.equals(master));
  // round 1: 잃었다고 치고 적용 안 함
  std::vector<OutMsg> r1;
  EXPECT_EQ(drainAt(e, nullptr, t0 + ms_t(500)), 0u);
  ASSERT_GT(drainAt(e, nullptr, t0 + ms_t(1100), &r1), 0u);
  EXPECT_LE(r1.size(), c.refresh_max_rects);
  for (auto & m : r1) {
    EXPECT_EQ(m.src, RepairSource::kRefresh);
  }
  EXPECT_FALSE(sub.equals(master));
  // round 2: 같은 사각형을 다시 → 적용하면 같아진다
  std::vector<OutMsg> r2;
  ASSERT_GT(drainAt(e, &sub, t0 + ms_t(2200), &r2), 0u);
  EXPECT_TRUE(sub.equals(master));
  // round 3: refresh 가 보낸 것은 다시 refresh 하지 않는다 → 더 없음
  EXPECT_EQ(drainAt(e, &sub, t0 + ms_t(3300)), 0u);
  const auto st = e.takeStats();
  std::printf("[R3] refresh rounds %lu, msgs %lu, bytes %lu (subscriber kept only the last 10 of the burst)\n",
    (unsigned long)st.refreshes, (unsigned long)st.refresh_msgs, (unsigned long)st.refresh_bytes);
}

// (R4) sweep: refresh 를 꺼도 sweep_period 한 바퀴 안에 지도 전체가 다시 간다
TEST(DiffEngineReview, SweepHealsAnyStaleCellWithinOneLap)
{
  Costmap2D master(200, 100, 0.05, 0.0, 0.0);
  FakeClock clk;
  auto c = cfg(16, 256);
  c.sweep_period_sec = 4.0;
  c.sweep_tick_sec = 1.0;
  c.max_msg_bytes = 1000;   // band(25 행 × 200) 을 다시 5 행씩
  DiffEngine e(c, std::ref(clk));
  Mirror sub;
  const auto t0 = SteadyClock::now();
  cycleAll(e, master);
  drainAt(e, &sub, t0);
  paintBlock(master, 5, 5, 4, 4, 254);
  paintBlock(master, 190, 95, 4, 4, 254);
  e.processCycle(master, 0, 0, 200, 100);
  drainAt(e, nullptr, t0);   // 구독자가 둘 다 잃음
  EXPECT_FALSE(sub.equals(master));
  size_t msgs = 0;
  for (int k = 1; k <= 5; ++k) {
    std::vector<OutMsg> out;
    msgs += drainAt(e, &sub, t0 + ms_t(1000 * k + 10), &out);
    for (auto & m : out) {
      EXPECT_EQ(m.src, RepairSource::kSweep);
      EXPECT_LE(m.update->data.size(), c.max_msg_bytes);
    }
  }
  EXPECT_TRUE(sub.equals(master));
  EXPECT_TRUE(sub.stamps_ok);
  const auto st = e.takeStats();
  EXPECT_GE(st.sweeps_completed, 1u);
  std::printf("[R4] sweep %zu msgs, laps %lu\n", msgs, (unsigned long)st.sweeps_completed);
  // 구독자가 없으면 sweep 을 만들지 않는다
  e.setHaveSubscribers(false);
  EXPECT_EQ(drainAt(e, &sub, t0 + ms_t(9000)), 0u);
  EXPECT_EQ(drainAt(e, &sub, t0 + ms_t(10100)), 0u);
}

// (R5) publish 실패: FULL 은 다시 요청 (catch-up 수집도 멈춤), update 는 그 사각형을 repair 로 다시
TEST(DiffEngineReview, PublishFailureIsRetried)
{
  Costmap2D master(300, 200, 0.05, 0.0, 0.0);
  FakeClock clk;
  auto c = cfg(16, 256);
  c.catchup_delays_sec = {1.0};
  DiffEngine e(c, std::ref(clk));
  Mirror sub;
  const auto t0 = SteadyClock::now();
  cycleAll(e, master);
  OutMsg m;
  ASSERT_TRUE(e.pop(m, t0));
  ASSERT_EQ(m.kind, OutMsg::Kind::kFull);
  e.onPublishFailed(m);           // FULL publish 가 throw 했다
  EXPECT_TRUE(e.fullPending());
  // FULL 이 대기 중인 동안 많은 update → catch-up 사각형이 무한히 늘지 않는다 (FULL 에 포함되어 버려짐)
  for (int k = 0; k < 2000; ++k) {
    master.setCost(k % 300, (k / 300) % 200, static_cast<unsigned char>(k % 250 + 1));
    e.processCycle(master, k % 300, (k / 300) % 200, k % 300 + 1, (k / 300) % 200 + 1);
  }
  ASSERT_TRUE(e.pop(m, t0));
  ASSERT_EQ(m.kind, OutMsg::Kind::kFull);
  EXPECT_EQ(m.reason, FullReason::kRetry);
  sub.apply(m);
  e.onPublished(m, t0);
  EXPECT_TRUE(sub.equals(master));
  // update publish 실패 → repair(retry)
  paintBlock(master, 20, 20, 5, 5, 254);
  e.processCycle(master, 20, 20, 25, 25);
  ASSERT_TRUE(e.pop(m, t0));
  ASSERT_EQ(m.kind, OutMsg::Kind::kUpdate);
  e.onPublishFailed(m);
  EXPECT_EQ(e.repairPending(), 1u);
  ASSERT_TRUE(e.pop(m, t0));
  EXPECT_EQ(m.src, RepairSource::kRetry);
  sub.apply(m);
  e.onPublished(m, t0);
  EXPECT_TRUE(sub.equals(master));
  EXPECT_TRUE(sub.stamps_ok);
  const auto st = e.takeStats();
  EXPECT_EQ(st.full_publish_failures, 1u);
  EXPECT_EQ(st.update_publish_failures, 1u);
}

// (R6) FULL 버퍼(지도 크기) 할당이 throw 해도 FULL token 은 남고 다음 pop 에서 만든다
TEST(DiffEngineReview, FullAllocationFailureKeepsToken)
{
  Costmap2D master(300, 200, 0.05, 0.0, 0.0);
  FakeClock clk;
  auto c = cfg(16, 256);
  int fail = 1;
  c.test_before_full_alloc = [&fail](size_t) {
      if (fail-- > 0) {
        throw std::bad_alloc();
      }
    };
  DiffEngine e(c, std::ref(clk));
  cycleAll(e, master);
  OutMsg m;
  EXPECT_THROW(e.pop(m, SteadyClock::now()), std::bad_alloc);
  EXPECT_TRUE(e.fullPending());
  Mirror sub;
  ASSERT_EQ(drainAt(e, &sub, SteadyClock::now()), 1u);
  EXPECT_EQ(sub.fulls, 1u);
  EXPECT_TRUE(sub.equals(master));
}

// (R7) 묶음 복사 도중: (a) map thread update → FULL + 뒤 update 로 같아짐, (b) geometry 변경 → 중단 후 새 FULL,
//      (c) 새 FULL 요청 → 중단 후 새 FULL
TEST(DiffEngineReview, ChunkedFullCopyWithConcurrentChanges)
{
  const uint32_t W = 256, H = 256;
  Costmap2D master(W, H, 0.05, 0.0, 0.0);
  FakeClock clk;
  auto c = cfg(16, 256);
  c.copy_chunk_bytes = W * 8;   // 8 행씩 → 32 묶음
  int hook_calls = 0;
  std::function<void()> action;
  c.test_between_full_chunks = [&]() {
      ++hook_calls;
      if (action) {
        action();
      }
    };
  DiffEngine e(c, std::ref(clk));
  cycleAll(e, master);
  Mirror sub;
  drainAt(e, &sub, SteadyClock::now());
  ASSERT_TRUE(sub.equals(master));
  EXPECT_EQ(hook_calls, 31);

  // (a) 복사 도중 위·아래 양쪽을 바꾼다 (이미 복사한 행과 아직 안 한 행)
  std::mt19937 rng(9);
  action = [&]() {
      const uint32_t x = rng() % (W - 4), y = rng() % (H - 4);
      paintBlock(master, x, y, 4, 4, static_cast<unsigned char>(1 + rng() % 254));
      e.processCycle(master, x, y, x + 4, y + 4);
    };
  e.requestFull(FullReason::kManual);
  OutMsg full;
  ASSERT_TRUE(e.pop(full, SteadyClock::now()));
  ASSERT_EQ(full.kind, OutMsg::Kind::kFull);
  action = nullptr;
  EXPECT_GT(e.queueLength(), 20u);   // 복사 중 들어온 update 는 FULL 뒤에 나간다
  sub.apply(full);
  e.onPublished(full, SteadyClock::now());
  drainAt(e, &sub, SteadyClock::now());
  EXPECT_TRUE(sub.equals(master));
  EXPECT_TRUE(sub.stamps_ok);

  // (b) geometry 변경 → 중단
  bool once = true;
  action = [&]() {
      if (once) {
        once = false;
        master.resizeMap(300, 200, 0.05, 0.0, 0.0);
        paintBlock(master, 290, 190, 5, 5, 254);
        e.processCycle(master, 0, 0, 1, 1);
      }
    };
  e.requestFull(FullReason::kManual);
  OutMsg m;
  EXPECT_FALSE(e.pop(m, SteadyClock::now()));
  action = nullptr;
  EXPECT_TRUE(e.fullPending());
  ASSERT_TRUE(e.pop(m, SteadyClock::now()));
  ASSERT_EQ(m.kind, OutMsg::Kind::kFull);
  EXPECT_EQ(m.reason, FullReason::kGeometry);
  sub.apply(m);
  e.onPublished(m, SteadyClock::now());
  EXPECT_TRUE(sub.equals(master));

  // (c) 새 FULL 요청 → 중단
  once = true;
  action = [&]() {
      if (once) {
        once = false;
        e.requestFull(FullReason::kBacklog);
      }
    };
  e.requestFull(FullReason::kManual);
  EXPECT_FALSE(e.pop(m, SteadyClock::now()));
  action = nullptr;
  ASSERT_TRUE(e.pop(m, SteadyClock::now()));
  EXPECT_EQ(m.reason, FullReason::kBacklog);
  sub.apply(m);
  e.onPublished(m, SteadyClock::now());
  EXPECT_TRUE(sub.equals(master));
  EXPECT_TRUE(sub.stamps_ok);
  const auto st = e.takeStats();
  EXPECT_EQ(st.full_aborts, 2u);
}

// (R8) update 메시지 하나 ≤ max_msg_bytes (행 band), cycle 분량 > max_update_cycle_bytes 이면 FULL
TEST(DiffEngineReview, MessageAndCycleSizeCaps)
{
  Costmap2D master(400, 400, 0.05, 0.0, 0.0);
  FakeClock clk;
  auto c = cfg(16, 256);
  c.max_msg_bytes = 1000;
  c.max_update_cycle_bytes = 12000;
  DiffEngine e(c, std::ref(clk));
  Mirror sub;
  cycleAll(e, master);
  drainAt(e, &sub, SteadyClock::now());
  paintBlock(master, 10, 10, 100, 100, 254);   // 10000 B → update, band 로
  auto r = e.processCycle(master, 10, 10, 110, 110);
  EXPECT_FALSE(r.full_requested);
  std::vector<OutMsg> out;
  drainAt(e, &sub, SteadyClock::now(), &out);
  EXPECT_GE(out.size(), 10u);
  for (auto & m : out) {
    EXPECT_LE(m.update->data.size(), c.max_msg_bytes);
  }
  EXPECT_TRUE(sub.equals(master));
  paintBlock(master, 200, 200, 120, 120, 254);  // 14400 B > 12000 → FULL
  r = e.processCycle(master, 200, 200, 320, 320);
  EXPECT_TRUE(r.full_requested);
  out.clear();
  drainAt(e, &sub, SteadyClock::now(), &out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].reason, FullReason::kLargeDiff);
  EXPECT_TRUE(sub.equals(master));
}

// (R9) map thread · publisher thread(FULL·catch-up·refresh·sweep) · service thread(snapshot) 동시. 끝에 mirror == master,
//      snapshot 은 (stamp 이후 update 를 적용하면) master 와 같다. TSAN 빌드에서도 돌릴 수 있게 작게.
TEST(DiffEngineReview, ThreeThreadsConverge)
{
  const uint32_t W = 600, H = 400;
  Costmap2D master(W, H, 0.05, 0.0, 0.0);
  std::atomic<int64_t> clk_ns{1000000000LL};
  auto c = cfg(16, 64);
  c.catchup_delays_sec = {0.02, 0.05};
  c.refresh_period_sec = 0.03;
  c.sweep_period_sec = 0.2;
  c.sweep_tick_sec = 0.01;
  c.copy_chunk_bytes = W * 16;
  c.max_queue_bytes = 64 * 1024;
  DiffEngine e(c, [&clk_ns]() {return clk_ns.fetch_add(1000);});
  Mirror sub;
  std::mutex sub_mtx;
  std::atomic<bool> run{true};
  cycleAll(e, master);
  std::thread pub([&]() {
      while (run.load()) {
        OutMsg m;
        if (e.waitPop(m, ms_t(5))) {
          std::lock_guard<std::mutex> l(sub_mtx);
          sub.apply(m);
          e.onPublished(m, SteadyClock::now());
        }
      }
    });
  std::atomic<int> snaps{0};
  std::thread srv([&]() {
      while (run.load()) {
        nav2_msgs::msg::Costmap s;
        if (e.snapshot(s)) {
          snaps++;
        }
        std::this_thread::sleep_for(ms_t(2));
      }
    });
  std::mt19937 rng(21);
  for (int cyc = 0; cyc < 1500; ++cyc) {
    if (cyc % 300 == 299) {
      e.requestFull(FullReason::kManual);
    }
    if (cyc % 500 == 250) {
      master.resetMap(0, 0, W, H);
      e.markResetPending();
    }
    const uint32_t x = rng() % (W - 30), y = rng() % (H - 30);
    paintBlock(master, x, y, 1 + rng() % 30, 1 + rng() % 30, static_cast<unsigned char>(rng() % 255));
    e.processCycle(master, x, y, x + 30, y + 30);
    if (cyc % 7 == 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(300));
    }
  }
  // 마지막 변경이 모두 나갈 때까지
  const auto end = SteadyClock::now() + std::chrono::seconds(5);
  bool eq = false;
  while (SteadyClock::now() < end && !eq) {
    std::this_thread::sleep_for(ms_t(20));
    std::lock_guard<std::mutex> l(sub_mtx);
    eq = e.queueLength() == 0 && sub.equals(master);
  }
  run = false;
  pub.join();
  srv.join();
  EXPECT_TRUE(eq);
  EXPECT_TRUE(sub.stamps_ok);
  nav2_msgs::msg::Costmap s;
  ASSERT_TRUE(e.snapshot(s));
  EXPECT_EQ(0, std::memcmp(s.data.data(), master.getCharMap(), s.data.size()));
  const auto st = e.takeStats();
  std::printf(
    "[R9] snapshots %d, fulls %lu, aborts %lu, catchup stages %lu, refresh msgs %lu, sweep msgs %lu, max hold %.3f ms\n",
    snaps.load(), (unsigned long)st.fulls_sent, (unsigned long)st.full_aborts, (unsigned long)st.catchups,
    (unsigned long)st.refresh_msgs, (unsigned long)st.sweep_msgs, st.max_shadow_hold_ms);
}

// (R10) 성능: 2e8 지도에서 publisher thread 가 FULL 을, service thread 가 snapshot 을 계속 만드는 동안
//       map thread (작은 bounds + 변경, 매 cycle shadow lock 필요) 의 최대 cycle 시간. copy_chunk 0 (= 예전처럼 한 번에) 과 비교.
TEST(DiffEnginePerf, ConcurrentFullAndSnapshotDoNotStallMapThread)
{
  const uint32_t W = 20000, H = 10000;
  Costmap2D master(W, H, 0.03, 0.0, 0.0);
  unsigned char * d = master.getCharMap();
  for (uint32_t y = 0; y < H; y += 50) {
    std::memset(d + size_t(y) * W, 254, W);
  }
  double worst_by_chunk[2] = {0, 0};
  for (int variant = 0; variant < 2; ++variant) {
    auto c = cfg(64, 256);
    c.copy_chunk_bytes = variant == 0 ? 0 : 2u * 1024u * 1024u;
    std::atomic<int64_t> t{1000000000LL};
    DiffEngine e(c, [&t]() {return t.fetch_add(1000);});
    cycleAll(e, master);
    {
      OutMsg m;
      ASSERT_TRUE(e.pop(m, SteadyClock::now()));
    }
    std::atomic<bool> run{true};
    std::atomic<int> fulls{0}, snaps{0};
    std::thread pub([&]() {
        while (run.load()) {
          e.requestFull(FullReason::kManual);
          OutMsg m;
          while (run.load() && e.pop(m, SteadyClock::now())) {
            if (m.kind == OutMsg::Kind::kFull) {
              fulls++;
            }
            e.onPublished(m, SteadyClock::now());
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      });
    std::thread srv([&]() {
        nav2_msgs::msg::Costmap s;
        while (run.load()) {
          if (e.snapshot(s)) {
            snaps++;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      });
    std::mt19937 rng(3);
    std::vector<double> cyc;
    const auto t_end = SteadyClock::now() + std::chrono::milliseconds(3000);
    while (SteadyClock::now() < t_end) {
      const uint32_t cx = 10000 + rng() % 300, cy = 5000 + rng() % 300;
      std::unique_lock<Costmap2D::mutex_t> lk(*master.getMutex());
      for (int k = 0; k < 20; ++k) {
        master.setCost(cx + rng() % 100, cy + rng() % 100, static_cast<unsigned char>(rng() % 255));
      }
      auto r = e.processCycle(master, cx - 116, cy - 116, cx + 217, cy + 217);
      lk.unlock();
      cyc.push_back(r.cycle_ms);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    run = false;
    pub.join();
    srv.join();
    std::sort(cyc.begin(), cyc.end());
    const double p50 = cyc[cyc.size() / 2], p99 = cyc[cyc.size() * 99 / 100], worst = cyc.back();
    const auto st = e.takeStats();
    std::printf(
      "[perf] (f) 2e8 map, concurrent FULL + snapshot loop, copy_chunk %s: map-thread cycles %zu, "
      "p50 %.3f ms, p99 %.2f ms, max %.2f ms (FULL %d, snapshot %d, max shadow hold %.2f ms)\n",
      variant == 0 ? "whole map (no chunks, buffer pre-allocated outside lock)" : "2 MB", cyc.size(), p50, p99, worst, fulls.load(), snaps.load(),
      st.max_shadow_hold_ms);
    worst_by_chunk[variant] = worst;
    EXPECT_GT(fulls.load(), 2);
    EXPECT_GT(snaps.load(), 2);
  }
  // 묶음 복사면 map thread 가 지도 크기 사본을 기다리지 않는다 (2 MB 사본 ~1 ms + 스케줄링)
  EXPECT_LT(worst_by_chunk[1], 15.0);
  EXPECT_LT(worst_by_chunk[1], worst_by_chunk[0]);
}

// ---------------------------------------------------------------------------
// (7) 성능 20000×10000 (2e8 cells, 현장 600×300 m @0.03)
// ---------------------------------------------------------------------------
TEST(DiffEnginePerf, HugeMap20000x10000)
{
  const uint32_t W = 20000, H = 10000;
  using ms = std::chrono::duration<double, std::milli>;
  auto t = SteadyClock::now();
  Costmap2D master(W, H, 0.03, 0.0, 0.0);
  unsigned char * d = master.getCharMap();
  for (uint32_t y = 0; y < H; y += 50) {
    std::memset(d + size_t(y) * W, 254, W);   // 벽 줄
  }
  std::printf("[perf] alloc+fill master %.1f ms\n", ms(SteadyClock::now() - t).count());

  FakeClock clk;
  DiffEngine e(cfg(64, 256), std::ref(clk));
  Mirror mir;
  t = SteadyClock::now();
  auto r0 = e.processCycle(master, 0, 0, W, H);
  const double first_cycle = ms(SteadyClock::now() - t).count();
  t = SteadyClock::now();
  OutMsg full;
  ASSERT_TRUE(e.pop(full, SteadyClock::now()));
  const double full_build = ms(SteadyClock::now() - t).count();
  mir.apply(full);
  e.onPublished(full, SteadyClock::now());
  std::printf(
    "[perf] first cycle (shadow copy + FULL token) %.1f ms, FULL materialize (publisher thread) %.1f ms, %.1f MB\n",
    first_cycle, full_build, full.bytes / 1048576.0);
  (void)r0;

  // 기준선: 지도 크기 메시지 하나 만드는 비용 (예전 always_send_full / update(전체 bounds) 의 복사만)
  t = SteadyClock::now();
  {
    std::vector<uint8_t> copy(d, d + size_t(W) * H);
    ASSERT_EQ(copy[0], d[0]);
  }
  std::printf("[perf] baseline: map-sized copy %.1f ms\n", ms(SteadyClock::now() - t).count());

  // (a) bounds = 지도 전체, 변경 없음
  double worst = 0;
  for (int i = 0; i < 3; ++i) {
    auto r = e.processCycle(master, 0, 0, W, H);
    worst = std::max(worst, r.cycle_ms);
    EXPECT_EQ(r.changed_cells, 0u);
  }
  std::printf("[perf] (a) full-bounds compare, 0 changes: worst cycle %.1f ms (3 runs)\n", worst);

  // (b) bounds = 지도 전체, 변경 몇 개 (흩어진 100 셀)
  std::mt19937 rng(11);
  for (int k = 0; k < 100; ++k) {
    master.setCost(rng() % W, rng() % H, 253);
  }
  auto rb = e.processCycle(master, 0, 0, W, H);
  t = SteadyClock::now();
  const size_t nb = drain(e, mir);
  std::printf(
    "[perf] (b) full-bounds, 100 scattered changes: compare %.1f ms, cycle %.1f ms, rects %zu, bytes %zu, msgs %zu\n",
    rb.compare_ms, rb.cycle_ms, rb.rects, rb.bytes, nb);
  EXPECT_TRUE(mir.equals(master));

  // (b2) bounds = 지도 전체, 흩어진 5000 셀 → 사각형 상한(256) 병합 비용과 면적
  for (int k = 0; k < 5000; ++k) {
    master.setCost(rng() % W, rng() % H, 252);
  }
  auto rb2 = e.processCycle(master, 0, 0, W, H);
  t = SteadyClock::now();
  drain(e, mir);
  const double drain_b2 = ms(SteadyClock::now() - t).count();
  std::printf(
    "[perf] (b2) full-bounds, 5000 scattered changes: compare %.1f ms, cycle %.1f ms, rects %zu, "
    "full_fallback %d (FULL materialize in publisher thread %.1f ms)\n",
    rb2.compare_ms, rb2.cycle_ms, rb2.rects, rb2.full_requested, drain_b2);
  EXPECT_TRUE(mir.equals(master));
  EXPECT_TRUE(rb2.full_requested);   // 사각형 256 개가 지도의 50 % 넘게 덮음 → FULL 로

  // (c) 작은 bounds (로봇 주변 10 m × 10 m = 333×333) 변경 있음, 50 cycle
  double worst_c = 0, sum_c = 0;
  for (int c = 0; c < 50; ++c) {
    const uint32_t cx = 10000 + c * 3, cy = 5000;
    for (int k = 0; k < 40; ++k) {
      master.setCost(cx + rng() % 100, cy + rng() % 100, static_cast<unsigned char>(rng() % 255));
    }
    auto r = e.processCycle(master, cx - 116, cy - 116, cx + 217, cy + 217);
    worst_c = std::max(worst_c, r.cycle_ms);
    sum_c += r.cycle_ms;
    drain(e, mir);
  }
  std::printf("[perf] (c) small bounds 333x333 with changes: avg %.3f ms, worst %.3f ms (50 cycles)\n",
    sum_c / 50, worst_c);
  EXPECT_TRUE(mir.equals(master));

  // (d) clear: 먼 곳 ghost 20 개 (반경 20 셀) → reset → 지도 전체 재계산
  Costmap2D before(master);
  const uint32_t TW = (W + 63) / 64, TH = (H + 63) / 64;
  std::vector<uint8_t> ghost_mask(size_t(TW) * TH, 0);
  for (int g = 0; g < 20; ++g) {
    const int gx = 100 + rng() % (W - 200), gy = 100 + rng() % (H - 200);
    paintGhost(master, gx, gy, 20);
    e.processCycle(master, gx - 20, gy - 20, gx + 21, gy + 21);
    for (int ty = (gy - 20) / 64; ty <= (gy + 20) / 64; ++ty) {
      for (int tx = (gx - 20) / 64; tx <= (gx + 20) / 64; ++tx) {
        ghost_mask[size_t(ty) * TW + tx] = 1;
      }
    }
  }
  // max_rects 에 따른 전송 면적 (같은 ghost 20 개, 지도 전체 window, tile 64) — 상한이 작을수록 거칠게 합쳐진다
  for (uint32_t cap : {4u, 8u, 16u, 32u, 64u, 256u}) {
    auto rr = ammr_costmap_diff_publisher::rectsFromMaskCapped(
      ghost_mask, TW, TH, 0, 0, 64, Rect{0, 0, W, H}, cap);
    uint64_t area = 0;
    for (const auto & q : rr) {
      area += q.area();
    }
    std::printf("[perf] (d') max_rects %3u -> rects %3zu, area %10lu cells (%.3f%% of map)\n",
      cap, rr.size(), (unsigned long)area, 100.0 * area / (double(W) * H));
  }
  drain(e, mir);
  ASSERT_TRUE(mir.equals(master));
  const size_t ub0 = mir.update_bytes;
  t = SteadyClock::now();
  master.resetMap(0, 0, W, H);
  e.markResetPending();
  std::memcpy(master.getCharMap(), before.getCharMap(), size_t(W) * H);
  const double reset_ms = ms(SteadyClock::now() - t).count();
  auto rd = e.processCycle(master, 0, 0, W, H);
  t = SteadyClock::now();
  drain(e, mir);
  const double drain_ms = ms(SteadyClock::now() - t).count();
  const size_t sent = mir.update_bytes - ub0;
  std::printf(
    "[perf] (d) clear with 20 ghosts r=20: changed %lu cells, rects %zu, sent %zu B (%.4f%% of map), "
    "compare %.1f ms, cycle %.1f ms (test's reset+recompute memcpy %.1f ms, drain %.2f ms)\n",
    (unsigned long)rd.changed_cells, rd.rects, sent, 100.0 * sent / (double(W) * H),
    rd.compare_ms, rd.cycle_ms, reset_ms, drain_ms);
  EXPECT_TRUE(mir.equals(master));
  EXPECT_LT(sent, size_t(20) * 4 * 64 * 64 + 1);

  // (e) snapshot (get_costmap 서비스 사본) 비용
  t = SteadyClock::now();
  nav2_msgs::msg::Costmap snap;
  ASSERT_TRUE(e.snapshot(snap));
  std::printf("[perf] (e) get_costmap snapshot copy %.1f ms\n", ms(SteadyClock::now() - t).count());
  EXPECT_EQ(0, std::memcmp(snap.data.data(), master.getCharMap(), snap.data.size()));
}
