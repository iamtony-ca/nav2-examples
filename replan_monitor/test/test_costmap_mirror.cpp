// [10-10 W1] CostmapMirror 단위 테스트.
//   1) 무작위 full + 무작위 범위 안 update 다수 == 기준 배열
//   2) 범위 밖 / 크기 불일치 / full 전 update 는 버리고 센다
//   3) 크기·origin 이 바뀐 full 은 grid 를 바꾸고 signature 변경을 알린다 (1 ulp origin 차이는 같은 것으로 본다)
//   4) stamp 기반 replay·stale 처리 (get_costmap 응답과 topic 의 순서 역전)
//   5) reader/writer 동시 실행 (~1 s): reader 는 반쯤 적용된 update 사각형을 보지 않는다
//   [10-10 W1][리뷰]
//   6) get_costmap 응답 뒤에 도착한 오래된 latched topic full 은 버린다 (grid·stamp 그대로)
//   7) clock reset 은 "현재 grid 를 받은 뒤 보낸 요청의 응답" 일 때만 인정한다
//   8) replay 보관을 못 한 (크기 초과·밀려난) 새 update 가 있으면 full 적용 뒤 needs_resync
//   9) spare grid 는 full 이 3 번 이상 연달아 올 때만 남고, full 이 끊기면 해제된다
//   [10-10 W1-2]
//  10) mirror 두 개(global costmap + agent mask)를 node 와 같은 순서(global guard → mask guard)로 함께 읽는
//      reader 와 각 mirror 의 writer 를 동시에 돌려도 교착이 없고, 어느 쪽도 반쯤 적용된 update 를 보지 않는다

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <set>
#include <thread>
#include <vector>

#include "replan_monitor/costmap_mirror.hpp"

using replan_monitor::CostmapMirror;
using FullResult = CostmapMirror::FullResult;
using UpdateResult = CostmapMirror::UpdateResult;

namespace
{

builtin_interfaces::msg::Time stamp(int64_t ns)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(ns / 1000000000LL);
  t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return t;
}

nav2_msgs::msg::Costmap makeFull(
  unsigned sx, unsigned sy, double res, double ox, double oy, int64_t ns,
  const std::vector<uint8_t> & data)
{
  nav2_msgs::msg::Costmap m;
  m.header.stamp = stamp(ns);
  m.header.frame_id = "map";
  m.metadata.size_x = sx;
  m.metadata.size_y = sy;
  m.metadata.resolution = static_cast<float>(res);
  m.metadata.origin.position.x = ox;
  m.metadata.origin.position.y = oy;
  m.data = data;
  return m;
}

nav2_msgs::msg::CostmapUpdate::ConstSharedPtr makeUpdate(
  unsigned x, unsigned y, unsigned w, unsigned h, int64_t ns, const std::vector<uint8_t> & data)
{
  auto u = std::make_shared<nav2_msgs::msg::CostmapUpdate>();
  u->header.stamp = stamp(ns);
  u->x = x;
  u->y = y;
  u->size_x = w;
  u->size_y = h;
  u->data = data;
  return u;
}

// [10-10 W1][리뷰] applyFull 이 FullOutcome 을 돌려주게 바뀌어 result 만 보는 helper.
FullResult full_result(
  CostmapMirror & m, const nav2_msgs::msg::Costmap & msg,
  uint64_t requested_at_gen = CostmapMirror::kNoRequestGen)
{
  return m.applyFull(msg, requested_at_gen).result;
}

std::vector<uint8_t> grabGrid(const CostmapMirror & m)
{
  auto g = m.read();
  if (!g) {
    return {};
  }
  const size_t n = static_cast<size_t>(g->getSizeInCellsX()) * g->getSizeInCellsY();
  return std::vector<uint8_t>(g->getCharMap(), g->getCharMap() + n);
}

}  // namespace

TEST(CostmapMirror, RandomFullAndUpdatesMatchReference)
{
  std::mt19937 rng(20261010);
  const unsigned sx = 317, sy = 211;
  std::vector<uint8_t> ref(static_cast<size_t>(sx) * sy);
  for (auto & v : ref) {v = static_cast<uint8_t>(rng());}

  CostmapMirror m;
  EXPECT_FALSE(m.hasGrid());
  EXPECT_FALSE(static_cast<bool>(m.read()));
  EXPECT_EQ(full_result(m, makeFull(sx, sy, 0.05, -3.0, 2.0, 1000, ref)), FullResult::kAppliedNewSignature);
  EXPECT_TRUE(m.hasGrid());

  const int N = 5000;
  int64_t ns = 2000;
  for (int i = 0; i < N; ++i) {
    std::uniform_int_distribution<unsigned> dx(0, sx - 1), dy(0, sy - 1);
    const unsigned x = dx(rng), y = dy(rng);
    std::uniform_int_distribution<unsigned> dw(0, sx - x), dh(0, sy - y);   // 0 크기 사각형도 섞는다
    const unsigned w = dw(rng), h = dh(rng);
    std::vector<uint8_t> d(static_cast<size_t>(w) * h);
    for (auto & v : d) {v = static_cast<uint8_t>(rng());}
    for (unsigned r = 0; r < h; ++r) {
      std::memcpy(&ref[(static_cast<size_t>(y) + r) * sx + x], &d[static_cast<size_t>(r) * w], w);
    }
    ASSERT_EQ(m.applyUpdate(makeUpdate(x, y, w, h, ns++, d)), UpdateResult::kApplied);
  }
  EXPECT_EQ(grabGrid(m), ref);

  const auto st = m.stats();
  EXPECT_EQ(st.fulls, 1u);
  EXPECT_EQ(st.updates, static_cast<uint64_t>(N));
  EXPECT_EQ(st.dropped_updates, 0u);
  EXPECT_EQ(st.last_update_stamp_ns, ns - 1);

  {
    auto g = m.read();
    ASSERT_TRUE(static_cast<bool>(g));
    EXPECT_EQ(g->getSizeInCellsX(), sx);
    EXPECT_EQ(g->getSizeInCellsY(), sy);
    EXPECT_NEAR(g->getResolution(), 0.05, 1e-6);
    EXPECT_DOUBLE_EQ(g->getOriginX(), -3.0);
    EXPECT_DOUBLE_EQ(g->getOriginY(), 2.0);
    // move / reset 은 lock 과 포인터를 함께 옮기고 비운다
    auto g2 = std::move(g);
    EXPECT_FALSE(static_cast<bool>(g));
    EXPECT_TRUE(static_cast<bool>(g2));
    g2.reset();
    EXPECT_FALSE(static_cast<bool>(g2));
  }
  // guard 를 놓았으니 writer 가 막히지 않는다
  EXPECT_EQ(m.applyUpdate(makeUpdate(0, 0, 1, 1, ns++, {7})), UpdateResult::kApplied);
}

TEST(CostmapMirror, BadUpdatesAreDroppedAndCounted)
{
  CostmapMirror m;
  // full 전 update: 보관만 하고 early 로 센다
  EXPECT_EQ(m.applyUpdate(makeUpdate(0, 0, 2, 2, 50, {1, 1, 1, 1})), UpdateResult::kNoGrid);

  const unsigned sx = 40, sy = 30;
  std::vector<uint8_t> base(sx * sy, 9);
  ASSERT_EQ(full_result(m, makeFull(sx, sy, 0.1, 0, 0, 10, base)), FullResult::kAppliedNewSignature);
  // 위 early update(stamp 50 > full 10)는 full 뒤에 replay 된다
  base[0] = base[1] = base[sx] = base[sx + 1] = 1;
  EXPECT_EQ(grabGrid(m), base);

  // x 방향 범위 밖, y 방향 범위 밖, 데이터 크기 불일치, uint32 overflow
  EXPECT_EQ(m.applyUpdate(makeUpdate(39, 0, 2, 1, 100, {5, 5})), UpdateResult::kRejectedOutOfBounds);
  EXPECT_EQ(m.applyUpdate(makeUpdate(0, 29, 1, 2, 101, {5, 5})), UpdateResult::kRejectedOutOfBounds);
  EXPECT_EQ(m.applyUpdate(makeUpdate(0, 0, 3, 3, 102, {5, 5, 5})), UpdateResult::kRejectedBadSize);
  EXPECT_EQ(
    m.applyUpdate(makeUpdate(std::numeric_limits<uint32_t>::max(), 0, 2, 1, 103, {5, 5})),
    UpdateResult::kRejectedOutOfBounds);
  EXPECT_EQ(grabGrid(m), base);   // grid 는 그대로

  const auto st = m.stats();
  EXPECT_EQ(st.dropped_updates, 4u);
  EXPECT_EQ(st.early_updates, 1u);
  EXPECT_EQ(st.replayed_updates, 1u);
  EXPECT_EQ(st.updates, 0u);

  // 정상 update 는 계속 적용된다
  EXPECT_EQ(m.applyUpdate(makeUpdate(38, 28, 2, 2, 104, {3, 3, 3, 3})), UpdateResult::kApplied);
  EXPECT_EQ(m.stats().updates, 1u);
}

TEST(CostmapMirror, FullWithNewGeometryReplacesGrid)
{
  CostmapMirror m;
  std::vector<uint8_t> a(100 * 100, 1);
  EXPECT_EQ(full_result(m, makeFull(100, 100, 0.05, 1.0, 2.0, 1, a)), FullResult::kAppliedNewSignature);
  // 같은 geometry → 변경 아님 (spare 재사용 경로)
  std::vector<uint8_t> a2(100 * 100, 2);
  EXPECT_EQ(full_result(m, makeFull(100, 100, 0.05, 1.0, 2.0, 2, a2)), FullResult::kApplied);
  EXPECT_EQ(grabGrid(m), a2);
  std::vector<uint8_t> a3(100 * 100, 3);
  EXPECT_EQ(full_result(m, makeFull(100, 100, 0.05, 1.0, 2.0, 3, a3)), FullResult::kApplied);
  EXPECT_EQ(grabGrid(m), a3);
  // origin 1 ulp 차이 (topic mapToWorld-res/2 vs service getOriginX) → 같은 것으로 본다
  EXPECT_EQ(
    full_result(m, makeFull(100, 100, 0.05, std::nextafter(1.0, 2.0), 2.0, 4, a)),
    FullResult::kApplied);

  // 크기 변경
  std::vector<uint8_t> b(150 * 80, 7);
  EXPECT_EQ(full_result(m, makeFull(150, 80, 0.05, 1.0, 2.0, 5, b)), FullResult::kAppliedNewSignature);
  {
    auto g = m.read();
    EXPECT_EQ(g->getSizeInCellsX(), 150u);
    EXPECT_EQ(g->getSizeInCellsY(), 80u);
  }
  EXPECT_EQ(grabGrid(m), b);
  // origin 변경
  EXPECT_EQ(full_result(m, makeFull(150, 80, 0.05, 1.5, 2.0, 6, b)), FullResult::kAppliedNewSignature);
  EXPECT_DOUBLE_EQ(m.read()->getOriginX(), 1.5);
  // 해상도 변경
  EXPECT_EQ(full_result(m, makeFull(150, 80, 0.10, 1.5, 2.0, 7, b)), FullResult::kAppliedNewSignature);

  // 크기 불일치 full 은 버리고 기존 grid 유지
  std::vector<uint8_t> bad(10, 0);
  EXPECT_EQ(full_result(m, makeFull(150, 80, 0.10, 1.5, 2.0, 8, bad)), FullResult::kRejectedBadSize);
  EXPECT_EQ(grabGrid(m), b);
  EXPECT_EQ(m.stats().rejected_fulls, 1u);

  // 옛 크기 update 는 새 grid 밖이면 버린다
  EXPECT_EQ(m.applyUpdate(makeUpdate(149, 79, 2, 2, 9, {0, 0, 0, 0})), UpdateResult::kRejectedOutOfBounds);

  // spare 를 끈 경우에도 결과는 같다
  CostmapMirror::Options o;
  o.keep_spare_buffer = false;
  CostmapMirror m2(o);
  EXPECT_EQ(full_result(m2, makeFull(100, 100, 0.05, 0, 0, 1, a)), FullResult::kAppliedNewSignature);
  EXPECT_EQ(full_result(m2, makeFull(100, 100, 0.05, 0, 0, 2, a2)), FullResult::kApplied);
  EXPECT_EQ(grabGrid(m2), a2);
}

TEST(CostmapMirror, StampOrderingReplayAndStale)
{
  CostmapMirror m;
  const unsigned sx = 20, sy = 20;
  std::vector<uint8_t> f1(sx * sy, 0);
  ASSERT_EQ(full_result(m, makeFull(sx, sy, 0.1, 0, 0, 100, f1)), FullResult::kAppliedNewSignature);

  // update 두 개 (stamp 200, 300)
  ASSERT_EQ(m.applyUpdate(makeUpdate(0, 0, 1, 1, 200, {20})), UpdateResult::kApplied);
  ASSERT_EQ(m.applyUpdate(makeUpdate(5, 5, 1, 1, 300, {30})), UpdateResult::kApplied);

  // get_costmap 응답: stamp 250 스냅샷 (200 은 반영, 300 은 아직) → 300 만 replay 되어야 한다
  std::vector<uint8_t> snap(sx * sy, 0);
  snap[0] = 20;
  const uint64_t gen_at_send = m.fullGeneration();
  EXPECT_EQ(full_result(m, makeFull(sx, sy, 0.1, 0, 0, 250, snap), gen_at_send), FullResult::kApplied);
  auto g = grabGrid(m);
  EXPECT_EQ(g[0], 20);
  EXPECT_EQ(g[5 * sx + 5], 30);
  EXPECT_EQ(m.stats().replayed_updates, 1u);

  // full(250) 보다 오래된 update 는 stale 로 버린다
  EXPECT_EQ(m.applyUpdate(makeUpdate(1, 1, 1, 1, 240, {99})), UpdateResult::kStale);
  EXPECT_EQ(grabGrid(m)[1 * sx + 1], 0);
  EXPECT_EQ(m.stats().stale_updates, 1u);

  // 현재 full(250) 보다 오래된 service 응답(현재 grid 이전에 보낸 요청)은 버린다
  EXPECT_EQ(full_result(m, makeFull(sx, sy, 0.1, 0, 0, 240, f1), gen_at_send), FullResult::kRejectedStale);
  EXPECT_EQ(m.stats().stale_fulls, 1u);
  // [10-10 W1][리뷰] 오래된 topic full 도 이제 버린다 (예전에는 clock reset 대비로 적용했다)
  std::vector<uint8_t> f_old(sx * sy, 4);
  EXPECT_EQ(full_result(m, makeFull(sx, sy, 0.1, 0, 0, 5, f_old)), FullResult::kRejectedStale);
  EXPECT_EQ(grabGrid(m)[5 * sx + 5], 30);
  EXPECT_EQ(m.stats().stale_fulls, 2u);
}

// [10-10 W1][리뷰] 리뷰 F1/F3/F7 재현: 늦게 붙은 path_validator 에서 get_costmap 응답이 latched activation full 보다
//   먼저 적용된 경우. 예전 코드에서는 cell(5,5) 가 0 으로 돌아갔고 t=50 update 도 kApplied 였다.
TEST(CostmapMirror, OlderLatchedTopicFullAfterServiceIsRejected)
{
  CostmapMirror m;
  const unsigned sx = 10, sy = 10;
  // latched update (t=90) 가 먼저 → grid 없음
  EXPECT_EQ(m.applyUpdate(makeUpdate(0, 0, 1, 1, 90, {3})), UpdateResult::kNoGrid);
  // get_costmap 요청(gen 0) 의 응답 t=100 (모두 7)
  const uint64_t gen0 = m.fullGeneration();
  EXPECT_EQ(gen0, 0u);
  auto o1 = m.applyFull(makeFull(sx, sy, 0.1, 0, 0, 100, std::vector<uint8_t>(sx * sy, 7)), gen0);
  EXPECT_EQ(o1.result, FullResult::kAppliedNewSignature);
  EXPECT_FALSE(o1.clock_reset);
  EXPECT_FALSE(o1.needs_resync);
  EXPECT_EQ(m.fullGeneration(), 1u);
  // 그 뒤 latched activation full t=10 (모두 0) 도착 → 버림
  auto o2 = m.applyFull(makeFull(sx, sy, 0.1, 0, 0, 10, std::vector<uint8_t>(sx * sy, 0)));
  EXPECT_EQ(o2.result, FullResult::kRejectedStale);
  EXPECT_EQ(m.read()->getCost(5, 5), 7);
  EXPECT_EQ(m.fullGeneration(), 1u);
  EXPECT_EQ(m.stats().last_full_stamp_ns, 100);
  // t=50 update 는 stale
  EXPECT_EQ(m.applyUpdate(makeUpdate(5, 5, 1, 1, 50, {0})), UpdateResult::kStale);
  EXPECT_EQ(m.read()->getCost(5, 5), 7);

  // 리뷰 F7 의 'service t=250 then topic t=100' 도 같은 결과
  CostmapMirror m2;
  EXPECT_EQ(full_result(m2, makeFull(sx, sy, 0.1, 0, 0, 250, std::vector<uint8_t>(sx * sy, 254)), 0),
    FullResult::kAppliedNewSignature);
  EXPECT_EQ(full_result(m2, makeFull(sx, sy, 0.1, 0, 0, 100, std::vector<uint8_t>(sx * sy, 0))),
    FullResult::kRejectedStale);
  EXPECT_EQ(m2.read()->getCost(2, 2), 254);
  EXPECT_EQ(m2.stats().fulls, 1u);
}

// [10-10 W1][리뷰] clock reset: 현재 grid 를 받은 뒤 보낸 요청의 응답만 오래된 stamp 로도 적용한다.
TEST(CostmapMirror, ClockResetOnlyViaResponseRequestedAfterCurrentGrid)
{
  CostmapMirror m;
  const unsigned sx = 10, sy = 10;
  ASSERT_EQ(full_result(m, makeFull(sx, sy, 0.1, 0, 0, 1000, std::vector<uint8_t>(sx * sy, 1))),
    FullResult::kAppliedNewSignature);
  ASSERT_EQ(m.applyUpdate(makeUpdate(0, 0, 1, 1, 1100, {2})), UpdateResult::kApplied);
  // sim 재시작: 새 epoch 의 topic full t=5 → 버림 (node 는 여기서 resync 를 건다)
  EXPECT_EQ(full_result(m, makeFull(sx, sy, 0.1, 0, 0, 5, std::vector<uint8_t>(sx * sy, 9))),
    FullResult::kRejectedStale);
  // 새 epoch update t=6 도 stale (아직 epoch 를 모름)
  EXPECT_EQ(m.applyUpdate(makeUpdate(1, 1, 1, 1, 6, {9})), UpdateResult::kStale);

  // 현재 grid 이전(gen 0)에 보낸 요청의 오래된 응답 → 버림 (clock reset 으로 보지 않는다)
  EXPECT_EQ(full_result(m, makeFull(sx, sy, 0.1, 0, 0, 7, std::vector<uint8_t>(sx * sy, 9)), 0),
    FullResult::kRejectedStale);

  // 현재 grid(gen 1) 를 받은 뒤 보낸 요청의 응답 t=8 → clock reset 으로 적용, 옛 epoch replay(1100) 는 버린다
  const uint64_t gen = m.fullGeneration();
  ASSERT_EQ(gen, 1u);
  auto o = m.applyFull(makeFull(sx, sy, 0.1, 0, 0, 8, std::vector<uint8_t>(sx * sy, 9)), gen);
  EXPECT_EQ(o.result, FullResult::kApplied);
  EXPECT_TRUE(o.clock_reset);
  EXPECT_TRUE(o.needs_resync);   // 그 사이 stale 로 버린 새 epoch update 를 메우려고
  EXPECT_EQ(o.replayed, 0u);
  EXPECT_EQ(grabGrid(m), std::vector<uint8_t>(sx * sy, 9));
  EXPECT_EQ(m.stats().clock_resets, 1u);
  // 새 epoch 의 update 는 이제 적용된다
  EXPECT_EQ(m.applyUpdate(makeUpdate(3, 3, 1, 1, 9, {4})), UpdateResult::kApplied);
  EXPECT_EQ(m.read()->getCost(3, 3), 4);
  // 다음 resync 응답(t=20) 은 보통대로 적용, needs_resync 없음
  auto o2 = m.applyFull(makeFull(sx, sy, 0.1, 0, 0, 20, std::vector<uint8_t>(sx * sy, 9)), m.fullGeneration());
  EXPECT_EQ(o2.result, FullResult::kApplied);
  EXPECT_FALSE(o2.clock_reset);
  EXPECT_FALSE(o2.needs_resync);

  // 요청 뒤에 다른 full(t=30) 이 적용됐으면 그 요청의 오래된 응답(t=25)은 clock reset 이 아니라 stale
  const uint64_t gen_send = m.fullGeneration();
  ASSERT_EQ(full_result(m, makeFull(sx, sy, 0.1, 0, 0, 30, std::vector<uint8_t>(sx * sy, 5))),
    FullResult::kApplied);
  EXPECT_EQ(full_result(m, makeFull(sx, sy, 0.1, 0, 0, 25, std::vector<uint8_t>(sx * sy, 9)), gen_send),
    FullResult::kRejectedStale);
  EXPECT_EQ(grabGrid(m), std::vector<uint8_t>(sx * sy, 5));
}

// [10-10 W1][리뷰] 리뷰 F2/F4/F9 재현: replay 에 보관 못 한 새 update 가 있으면 그보다 오래된 스냅샷 적용 뒤 needs_resync.
TEST(CostmapMirror, UnreplayableNewerUpdateRequestsResync)
{
  const unsigned sx = 10, sy = 10;
  {
    // 크기 초과 (replay_max_bytes=50 < 100 cells): ClearEntireCostmap 뒤의 지도 전체 update 모사
    CostmapMirror::Options o;
    o.replay_max_bytes = 50;
    CostmapMirror m(o);
    ASSERT_EQ(full_result(m, makeFull(sx, sy, 0.1, 0, 0, 100, std::vector<uint8_t>(sx * sy, 254))),
      FullResult::kAppliedNewSignature);
    const uint64_t gen = m.fullGeneration();   // resync 요청을 보낸 시점
    // 응답 전에 지도 전체 clear update(t=210) 도착 → 적용되지만 보관은 못 함
    EXPECT_EQ(m.applyUpdate(makeUpdate(0, 0, sx, sy, 210, std::vector<uint8_t>(sx * sy, 0))),
      UpdateResult::kApplied);
    EXPECT_EQ(m.stats().unreplayable_updates, 1u);
    // clear 전에 뜬 스냅샷(t=200) 응답 → 적용은 되지만 clear 가 빠졌으니 needs_resync
    auto r = m.applyFull(makeFull(sx, sy, 0.1, 0, 0, 200, std::vector<uint8_t>(sx * sy, 254)), gen);
    EXPECT_EQ(r.result, FullResult::kApplied);
    EXPECT_TRUE(r.needs_resync);
    EXPECT_FALSE(r.clock_reset);
    EXPECT_EQ(m.stats().dropped_updates, 0u);
    // clear 뒤에 뜬 다음 응답(t=300) 은 needs_resync 없이 끝난다
    auto r2 = m.applyFull(makeFull(sx, sy, 0.1, 0, 0, 300, std::vector<uint8_t>(sx * sy, 0)), m.fullGeneration());
    EXPECT_EQ(r2.result, FullResult::kApplied);
    EXPECT_FALSE(r2.needs_resync);
    EXPECT_EQ(m.read()->getCost(3, 3), 0);
  }
  {
    // 개수 한도로 밀려남 (replay_max_msgs=2): 101 이 밀려나고 102, 103 만 남는다
    CostmapMirror::Options o;
    o.replay_max_msgs = 2;
    CostmapMirror m(o);
    ASSERT_EQ(full_result(m, makeFull(sx, sy, 0.1, 0, 0, 50, std::vector<uint8_t>(sx * sy, 0))),
      FullResult::kAppliedNewSignature);
    const uint64_t gen = m.fullGeneration();
    EXPECT_EQ(m.applyUpdate(makeUpdate(1, 1, 1, 1, 101, {11})), UpdateResult::kApplied);
    EXPECT_EQ(m.applyUpdate(makeUpdate(2, 2, 1, 1, 102, {12})), UpdateResult::kApplied);
    EXPECT_EQ(m.applyUpdate(makeUpdate(3, 3, 1, 1, 103, {13})), UpdateResult::kApplied);
    EXPECT_EQ(m.stats().unreplayable_updates, 1u);
    auto r = m.applyFull(makeFull(sx, sy, 0.1, 0, 0, 100, std::vector<uint8_t>(sx * sy, 0)), gen);
    EXPECT_EQ(r.result, FullResult::kApplied);
    EXPECT_TRUE(r.needs_resync);
    EXPECT_EQ(r.replayed, 2u);
    EXPECT_EQ(m.read()->getCost(2, 2), 12);
    EXPECT_EQ(m.read()->getCost(3, 3), 13);
    EXPECT_EQ(m.read()->getCost(1, 1), 0);   // 101 은 잃었다 → node 가 resync
    // 101 이후에 뜬 스냅샷이면 잃은 것이 없다
    auto r2 = m.applyFull(makeFull(sx, sy, 0.1, 0, 0, 150, std::vector<uint8_t>(sx * sy, 0)), m.fullGeneration());
    EXPECT_FALSE(r2.needs_resync);
  }
}

// [10-10 W1][리뷰] 리뷰 F5/F8: 시작 때 topic full + get_costmap 응답(2 번)으로는 spare 를 남기지 않는다.
TEST(CostmapMirror, SpareKeptOnlyWhileFullsStream)
{
  const unsigned sx = 10, sy = 10;
  CostmapMirror::Options o;
  o.spare_keep_window_sec = 0.2;
  CostmapMirror m(o);
  std::vector<uint8_t> a(sx * sy, 1);
  ASSERT_EQ(full_result(m, makeFull(sx, sy, 0.1, 0, 0, 1, a)), FullResult::kAppliedNewSignature);
  ASSERT_EQ(full_result(m, makeFull(sx, sy, 0.1, 0, 0, 2, a), m.fullGeneration()), FullResult::kApplied);
  EXPECT_FALSE(m.hasSpare());   // False 의 정상 시작 (예전에는 여기서 spare 가 남아 끝까지 상주)
  // True 처럼 연달아 오면 3 번째부터 남긴다
  ASSERT_EQ(full_result(m, makeFull(sx, sy, 0.1, 0, 0, 3, a)), FullResult::kApplied);
  EXPECT_TRUE(m.hasSpare());
  ASSERT_EQ(full_result(m, makeFull(sx, sy, 0.1, 0, 0, 4, a)), FullResult::kApplied);
  EXPECT_TRUE(m.hasSpare());
  // full 이 끊기면 update 경로에서 해제
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_EQ(m.applyUpdate(makeUpdate(0, 0, 1, 1, 5, {2})), UpdateResult::kApplied);
  EXPECT_FALSE(m.hasSpare());
  EXPECT_EQ(m.stats().spare_releases, 1u);
  // timer 경로(releaseIdleSpare)도 해제
  ASSERT_EQ(full_result(m, makeFull(sx, sy, 0.1, 0, 0, 6, a)), FullResult::kApplied);   // streak 0
  ASSERT_EQ(full_result(m, makeFull(sx, sy, 0.1, 0, 0, 7, a)), FullResult::kApplied);   // 1
  ASSERT_EQ(full_result(m, makeFull(sx, sy, 0.1, 0, 0, 8, a)), FullResult::kApplied);   // 2 → 남김
  EXPECT_TRUE(m.hasSpare());
  m.releaseIdleSpare();
  EXPECT_TRUE(m.hasSpare());     // 아직 window 안
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  m.releaseIdleSpare();
  EXPECT_FALSE(m.hasSpare());
  EXPECT_EQ(m.stats().spare_releases, 2u);
}

TEST(CostmapMirror, ConcurrentReadersNeverSeeTornUpdates)
{
  // grid 를 T×T 타일로 나누고, update 는 항상 타일 하나를 한 값으로 채운다. full 은 전체를 한 값으로 채운다.
  // 어느 순간의 일관된 grid 에서도 각 타일은 균일해야 한다 → 균일하지 않으면 반쯤 적용된 update 를 본 것.
  const unsigned tile = 64, tiles_x = 16, tiles_y = 12;
  const unsigned sx = tile * tiles_x, sy = tile * tiles_y;
  CostmapMirror m;
  ASSERT_EQ(
    full_result(m, makeFull(sx, sy, 0.05, 0, 0, 1, std::vector<uint8_t>(static_cast<size_t>(sx) * sy, 0))),
    FullResult::kAppliedNewSignature);

  std::atomic<bool> stop{false};
  std::atomic<uint64_t> torn{0}, reads{0}, tiles_checked{0}, writes{0}, fulls{0};
  std::atomic<int64_t> clock_ns{10};

  auto writer = [&](unsigned seed) {
      std::mt19937 rng(seed);
      while (!stop.load()) {
        if (rng() % 200 == 0) {
          const uint8_t v = static_cast<uint8_t>(rng());
          // 가끔 origin 도 바꿔 swap(새 geometry) 경로를 섞는다
          const double ox = (rng() % 2) ? 0.0 : 0.5;
          full_result(m, makeFull(sx, sy, 0.05, ox, 0, clock_ns.fetch_add(1),
              std::vector<uint8_t>(static_cast<size_t>(sx) * sy, v)));
          fulls.fetch_add(1);
        } else {
          const unsigned tx = rng() % tiles_x, ty = rng() % tiles_y;
          const uint8_t v = static_cast<uint8_t>(rng());
          const auto r = m.applyUpdate(makeUpdate(tx * tile, ty * tile, tile, tile, clock_ns.fetch_add(1),
              std::vector<uint8_t>(static_cast<size_t>(tile) * tile, v)));
          if (r == UpdateResult::kApplied) {writes.fetch_add(1);}
        }
      }
    };

  auto reader = [&](unsigned seed) {
      std::mt19937 rng(seed);
      std::set<int> seen_values;
      while (!stop.load()) {
        auto g = m.read();
        if (!g) {continue;}
        const unsigned char * d = g->getCharMap();
        const size_t w = g->getSizeInCellsX();
        for (int k = 0; k < 8; ++k) {
          const unsigned tx = rng() % tiles_x, ty = rng() % tiles_y;
          const unsigned char v0 = d[static_cast<size_t>(ty * tile) * w + tx * tile];
          bool uniform = true;
          for (unsigned r = 0; r < tile && uniform; ++r) {
            const unsigned char * row = d + static_cast<size_t>(ty * tile + r) * w + tx * tile;
            for (unsigned c = 0; c < tile; ++c) {
              if (row[c] != v0) {uniform = false; break;}
            }
          }
          if (!uniform) {torn.fetch_add(1);}
          tiles_checked.fetch_add(1);
        }
        reads.fetch_add(1);
      }
    };

  std::vector<std::thread> th;
  th.emplace_back(writer, 1u);
  th.emplace_back(writer, 2u);
  for (unsigned i = 0; i < 4; ++i) {th.emplace_back(reader, 100u + i);}
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  stop.store(true);
  for (auto & t : th) {t.join();}

  const auto st = m.stats();
  std::printf(
    "[ConcurrentReaders] reads=%lu tiles_checked=%lu torn=%lu update_writes=%lu full_writes=%lu "
    "mirror(fulls=%lu updates=%lu dropped=%lu stale=%lu)\n",
    static_cast<unsigned long>(reads.load()), static_cast<unsigned long>(tiles_checked.load()),
    static_cast<unsigned long>(torn.load()), static_cast<unsigned long>(writes.load()),
    static_cast<unsigned long>(fulls.load()),
    static_cast<unsigned long>(st.fulls), static_cast<unsigned long>(st.updates),
    static_cast<unsigned long>(st.dropped_updates), static_cast<unsigned long>(st.stale_updates));
  EXPECT_EQ(torn.load(), 0u);
  EXPECT_GT(reads.load(), 100u);
  EXPECT_GT(writes.load(), 100u);
  EXPECT_GT(fulls.load(), 0u);
  EXPECT_EQ(st.dropped_updates, 0u);
}


// [10-10 W1-2] mirror 두 개를 함께 쓰는 node 와 같은 lock 패턴.
//   - reader 는 timer 그룹처럼 여러 개 (node 는 MutuallyExclusive 1 개지만 더 가혹하게 4 개).
//     세 가지 패턴을 섞는다: (a) global guard → (중간에 lazy 로) mask guard, 함수 끝까지 둘 다 쥠 (validatePathOptimized,
//     READY), (b) global guard 를 쥔 채 mask guard 를 잠깐 잡았다 놓음 (validateWithPoints), (c) mask guard 만 (Phase 1.5).
//     mask guard 를 쥔 채 global guard 를 잡는 경우는 없다 (lock 순서).
//   - writer 는 mirror 마다 2 개 (update 와 가끔 full/새 geometry). node 에서는 subs 그룹 하나라 한 번에 하나지만
//     여기서는 두 mirror 의 writer 가 동시에 돈다.
//   교착이면 정해진 시간 안에 thread 가 끝나지 않는다 → 실패 처리 후 프로세스를 끝낸다 (join 에서 멈추지 않도록).
TEST(CostmapMirror, TwoMirrorsGlobalThenMaskNoDeadlockNoTorn)
{
  const unsigned tile = 32, tiles_x = 12, tiles_y = 10;
  const unsigned sx = tile * tiles_x, sy = tile * tiles_y;
  CostmapMirror g_m, a_m;   // global costmap, agent mask
  for (CostmapMirror * m : {&g_m, &a_m}) {
    ASSERT_EQ(
      full_result(*m, makeFull(sx, sy, 0.05, 0, 0, 1, std::vector<uint8_t>(static_cast<size_t>(sx) * sy, 0))),
      FullResult::kAppliedNewSignature);
  }

  std::atomic<bool> stop{false};
  std::atomic<int> finished{0};
  std::atomic<uint64_t> torn{0}, reads_both{0}, reads_nested{0}, reads_mask_only{0};
  std::atomic<uint64_t> g_writes{0}, a_writes{0}, g_fulls{0}, a_fulls{0};
  std::atomic<int64_t> clock_g{10}, clock_a{10};

  auto tileUniform = [&](const nav2_costmap_2d::Costmap2D & cm, std::mt19937 & rng) {
      const unsigned char * d = cm.getCharMap();
      const size_t w = cm.getSizeInCellsX();
      const unsigned tx = rng() % tiles_x, ty = rng() % tiles_y;
      const unsigned char v0 = d[static_cast<size_t>(ty * tile) * w + tx * tile];
      for (unsigned r = 0; r < tile; ++r) {
        const unsigned char * row = d + static_cast<size_t>(ty * tile + r) * w + tx * tile;
        for (unsigned c = 0; c < tile; ++c) {
          if (row[c] != v0) {return false;}
        }
      }
      return true;
    };

  auto writer = [&](CostmapMirror & m, std::atomic<int64_t> & clk, std::atomic<uint64_t> & nw,
      std::atomic<uint64_t> & nf, unsigned seed) {
      std::mt19937 rng(seed);
      while (!stop.load()) {
        if (rng() % 150 == 0) {
          const uint8_t v = static_cast<uint8_t>(rng());
          const double ox = (rng() % 2) ? 0.0 : 0.5;   // 가끔 새 geometry (swap)
          full_result(m, makeFull(sx, sy, 0.05, ox, 0, clk.fetch_add(1),
              std::vector<uint8_t>(static_cast<size_t>(sx) * sy, v)));
          nf.fetch_add(1);
        } else {
          const unsigned tx = rng() % tiles_x, ty = rng() % tiles_y;
          const uint8_t v = static_cast<uint8_t>(rng());
          if (m.applyUpdate(makeUpdate(tx * tile, ty * tile, tile, tile, clk.fetch_add(1),
              std::vector<uint8_t>(static_cast<size_t>(tile) * tile, v))) == UpdateResult::kApplied)
          {
            nw.fetch_add(1);
          }
        }
      }
      finished.fetch_add(1);
    };

  auto reader = [&](unsigned seed) {
      std::mt19937 rng(seed);
      while (!stop.load()) {
        const unsigned pattern = rng() % 3;
        if (pattern == 0) {
          // (a) global → (lazy) mask, 둘 다 끝까지
          auto gg = g_m.read();
          if (!gg) {continue;}
          if (!tileUniform(*gg, rng)) {torn.fetch_add(1);}
          std::optional<CostmapMirror::ReadGuard> ag;
          for (int k = 0; k < 4; ++k) {
            if (!tileUniform(*gg, rng)) {torn.fetch_add(1);}
            if (k == 1) {ag.emplace(a_m.read());}
            if (ag && *ag && !tileUniform(**ag, rng)) {torn.fetch_add(1);}
          }
          reads_both.fetch_add(1);
        } else if (pattern == 1) {
          // (b) global 을 쥔 채 mask 를 잠깐
          auto gg = g_m.read();
          if (!gg) {continue;}
          for (int k = 0; k < 3; ++k) {
            if (!tileUniform(*gg, rng)) {torn.fetch_add(1);}
            {
              const auto ag = a_m.read();
              if (ag && !tileUniform(*ag, rng)) {torn.fetch_add(1);}
            }
          }
          reads_nested.fetch_add(1);
        } else {
          // (c) mask 만
          const auto ag = a_m.read();
          if (ag && !tileUniform(*ag, rng)) {torn.fetch_add(1);}
          reads_mask_only.fetch_add(1);
        }
      }
      finished.fetch_add(1);
    };

  std::vector<std::thread> th;
  th.emplace_back(writer, std::ref(g_m), std::ref(clock_g), std::ref(g_writes), std::ref(g_fulls), 1u);
  th.emplace_back(writer, std::ref(g_m), std::ref(clock_g), std::ref(g_writes), std::ref(g_fulls), 2u);
  th.emplace_back(writer, std::ref(a_m), std::ref(clock_a), std::ref(a_writes), std::ref(a_fulls), 3u);
  th.emplace_back(writer, std::ref(a_m), std::ref(clock_a), std::ref(a_writes), std::ref(a_fulls), 4u);
  for (unsigned i = 0; i < 4; ++i) {th.emplace_back(reader, 200u + i);}
  const int nthreads = static_cast<int>(th.size());

  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  stop.store(true);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (finished.load() < nthreads && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (finished.load() < nthreads) {
    std::fprintf(stderr, "[TwoMirrors] 교착: %d/%d thread 만 끝났다\n", finished.load(), nthreads);
    std::fflush(stderr);
    ADD_FAILURE() << "deadlock: only " << finished.load() << "/" << nthreads << " threads finished";
    std::_Exit(1);   // 멈춘 thread 를 join 하지 않고 끝낸다
  }
  for (auto & t : th) {t.join();}

  std::printf(
    "[TwoMirrors] reads(both=%lu nested=%lu mask_only=%lu) torn=%lu global(updates=%lu fulls=%lu) "
    "mask(updates=%lu fulls=%lu)\n",
    static_cast<unsigned long>(reads_both.load()), static_cast<unsigned long>(reads_nested.load()),
    static_cast<unsigned long>(reads_mask_only.load()), static_cast<unsigned long>(torn.load()),
    static_cast<unsigned long>(g_writes.load()), static_cast<unsigned long>(g_fulls.load()),
    static_cast<unsigned long>(a_writes.load()), static_cast<unsigned long>(a_fulls.load()));
  EXPECT_EQ(torn.load(), 0u);
  EXPECT_GT(reads_both.load(), 50u);
  EXPECT_GT(reads_nested.load(), 50u);
  EXPECT_GT(reads_mask_only.load(), 50u);
  EXPECT_GT(g_writes.load(), 100u);   // writer 우선 gate 덕에 reader 가 많아도 두 writer 모두 진행한다
  EXPECT_GT(a_writes.load(), 100u);
  EXPECT_GT(g_fulls.load(), 0u);
  EXPECT_GT(a_fulls.load(), 0u);
}

// ============================================================================================
// [10-10 W1-2][리뷰] self_contained_updates (agent mask) 와 replay 한도
// ============================================================================================
namespace
{

// W1-2 AgentLayer 와 같은 성질의 mask 생성기: 사이클마다 0 이 아닌 셀은 painted 사각형 안에만 있고,
// update 사각형 = painted(이번) ∪ painted(직전). 0 이 아닌 셀 ⊆ update 사각형.
struct MaskGen
{
  unsigned sx, sy;
  std::vector<uint8_t> grid;
  unsigned px0{0}, py0{0}, pw{0}, ph{0};   // 직전 painted
  std::mt19937 rng;

  MaskGen(unsigned sx_, unsigned sy_, unsigned seed)
  : sx(sx_), sy(sy_), grid(static_cast<size_t>(sx_) * sy_, 0), rng(seed) {}

  // 한 사이클: 직전 painted 를 지우고 새로 칠한다. 칠할 것이 없으면 painted 는 빈 사각형. 돌려주는 update 가 비면 nullptr.
  nav2_msgs::msg::CostmapUpdate::ConstSharedPtr cycle(int64_t ns, bool paint)
  {
    for (unsigned r = 0; r < ph; ++r) {
      std::memset(&grid[(static_cast<size_t>(py0) + r) * sx + px0], 0, pw);
    }
    unsigned nx0 = 0, ny0 = 0, nw = 0, nh = 0;
    if (paint) {
      nw = std::uniform_int_distribution<unsigned>(1, sx / 4)(rng);
      nh = std::uniform_int_distribution<unsigned>(1, sy / 4)(rng);
      nx0 = std::uniform_int_distribution<unsigned>(0, sx - nw)(rng);
      ny0 = std::uniform_int_distribution<unsigned>(0, sy - nh)(rng);
      for (unsigned r = 0; r < nh; ++r) {
        for (unsigned c = 0; c < nw; ++c) {
          if (rng() % 3 == 0) {grid[(static_cast<size_t>(ny0) + r) * sx + nx0 + c] = static_cast<uint8_t>(1 + rng() % 254);}
        }
      }
    }
    // update 사각형 = 직전 ∪ 이번
    unsigned ux0 = sx, uy0 = sy, uxn = 0, uyn = 0;
    auto unite = [&](unsigned x0, unsigned y0, unsigned w, unsigned h) {
        if (w == 0 || h == 0) {return;}
        ux0 = std::min(ux0, x0); uy0 = std::min(uy0, y0);
        uxn = std::max(uxn, x0 + w); uyn = std::max(uyn, y0 + h);
      };
    unite(px0, py0, pw, ph);
    unite(nx0, ny0, nw, nh);
    px0 = nx0; py0 = ny0; pw = nw; ph = nh;
    if (ux0 >= uxn || uy0 >= uyn) {return nullptr;}
    const unsigned w = uxn - ux0, h = uyn - uy0;
    std::vector<uint8_t> d(static_cast<size_t>(w) * h);
    for (unsigned r = 0; r < h; ++r) {
      std::memcpy(&d[static_cast<size_t>(r) * w], &grid[(static_cast<size_t>(uy0) + r) * sx + ux0], w);
    }
    return makeUpdate(ux0, uy0, w, h, ns, d);
  }
};

CostmapMirror::Options scOptions()
{
  CostmapMirror::Options o;
  o.self_contained_updates = true;
  return o;
}

}  // namespace

// 유실(무작위, 연속 포함)이 있어도 받은 update 하나로 mirror == 실제 mask. 같은 유실을 일반 모드(복사만)로 겪으면 유령 셀이 남는다.
TEST(CostmapMirrorSelfContained, AnyDeliveredUpdateRestoresExactMask)
{
  const unsigned sx = 211, sy = 157;
  for (unsigned seed : {3u, 11u, 2026u}) {
    SCOPED_TRACE("seed " + std::to_string(seed));
    MaskGen gen(sx, sy, seed);
    CostmapMirror sc(scOptions()), plain;
    const std::vector<uint8_t> zeros(static_cast<size_t>(sx) * sy, 0);
    ASSERT_TRUE(sc.applyFull(makeFull(sx, sy, 0.03, 0.0, 0.0, 1000, zeros)).applied());
    ASSERT_TRUE(plain.applyFull(makeFull(sx, sy, 0.03, 0.0, 0.0, 1000, zeros)).applied());
    std::mt19937 rng(seed * 7 + 1);
    int delivered = 0, dropped = 0, plain_ghost_cycles = 0;
    for (int c = 0; c < 3000; ++c) {
      const bool paint = (c % 50) < 40;   // 가끔 칠할 것이 없는 구간 (마지막 지우기 update 다음엔 보낼 것이 없다)
      auto u = gen.cycle(2000 + c, paint);
      if (!u) {continue;}
      if (rng() % 4 == 0) {++dropped; continue;}   // DDS 가 알리지 않는 유실 (연속 유실 포함)
      ++delivered;
      ASSERT_EQ(sc.applyUpdate(u), UpdateResult::kApplied);
      ASSERT_EQ(plain.applyUpdate(u), UpdateResult::kApplied);
      ASSERT_EQ(grabGrid(sc), gen.grid) << "cycle " << c;
      if (grabGrid(plain) != gen.grid) {++plain_ghost_cycles;}
    }
    const auto st = sc.stats();
    EXPECT_EQ(st.sc_bbox_scans, 1u);   // full 뒤 첫 update 에서 한 번만 훑었다
    EXPECT_EQ(st.unreplayable_updates, 0u);
    EXPECT_GT(dropped, 100);
    EXPECT_GT(plain_ghost_cycles, 0);   // 음성 대조: 일반 모드는 유실 뒤 유령 셀이 남는다
    std::printf("[SC seed %u] delivered=%d dropped=%d plain_ghost_cycles=%d sc_mismatch=0\n",
      seed, delivered, dropped, plain_ghost_cycles);
  }
}

// full 내용(0 아닌 셀이 흩어져 있음) 뒤 첫 update: grid 를 훑어 bbox 를 0 으로 만든 뒤 복사 → update 만 남는다.
TEST(CostmapMirrorSelfContained, FirstUpdateAfterFullClearsFullContent)
{
  const unsigned sx = 4500, sy = 37;   // 4 KB 블록 경계를 넘는 행
  std::vector<uint8_t> full(static_cast<size_t>(sx) * sy, 0);
  full[5 * sx + 4096] = 9;     // 블록 경계 바로 뒤
  full[30 * sx + 4095] = 7;    // 블록 경계 바로 앞
  full[0] = 3;
  full[static_cast<size_t>(sx) * sy - 1] = 4;
  CostmapMirror m(scOptions());
  ASSERT_TRUE(m.applyFull(makeFull(sx, sy, 0.03, 0.0, 0.0, 1000, full)).applied());
  EXPECT_EQ(grabGrid(m), full);   // update 전에는 full 내용 그대로
  std::vector<uint8_t> d(4 * 2, 0); d[1] = 200;
  ASSERT_EQ(m.applyUpdate(makeUpdate(100, 10, 4, 2, 2000, d)), UpdateResult::kApplied);
  std::vector<uint8_t> expect(full.size(), 0);
  expect[10 * sx + 101] = 200;
  EXPECT_EQ(grabGrid(m), expect);
  EXPECT_EQ(m.stats().sc_bbox_scans, 1u);
  // 두 번째부터는 직전 사각형만 지운다
  std::vector<uint8_t> d2(1, 5);
  ASSERT_EQ(m.applyUpdate(makeUpdate(4499, 36, 1, 1, 3000, d2)), UpdateResult::kApplied);
  std::vector<uint8_t> expect2(full.size(), 0);
  expect2[static_cast<size_t>(sx) * sy - 1] = 5;
  EXPECT_EQ(grabGrid(m), expect2);
  EXPECT_EQ(m.stats().sc_bbox_scans, 1u);
}

// geometry_only full: 내용은 0 (오래된 latched full 의 몸체를 살아 있는 agent 로 내보내지 않는다), 다음 update 로 정확.
TEST(CostmapMirrorSelfContained, GeometryOnlyFullStartsEmpty)
{
  const unsigned sx = 50, sy = 40;
  std::vector<uint8_t> stale(static_cast<size_t>(sx) * sy, 0);
  for (unsigned i = 0; i < 30; ++i) {stale[12 * sx + i] = 254;}   // activation 때 칠한 이웃 몸체
  CostmapMirror m(scOptions());
  const auto out = m.applyFull(makeFull(sx, sy, 0.03, 1.0, 2.0, 1000, stale), CostmapMirror::kNoRequestGen, true);
  ASSERT_EQ(out.result, FullResult::kAppliedNewSignature);
  EXPECT_FALSE(out.needs_resync);
  EXPECT_EQ(grabGrid(m), std::vector<uint8_t>(stale.size(), 0));
  EXPECT_EQ(m.stats().geometry_only_fulls, 1u);
  {
    auto g = m.read();
    ASSERT_TRUE(static_cast<bool>(g));
    EXPECT_EQ(g->getSizeInCellsX(), sx);
    EXPECT_DOUBLE_EQ(g->getOriginX(), 1.0);
  }
  std::vector<uint8_t> d(3, 100);
  ASSERT_EQ(m.applyUpdate(makeUpdate(7, 8, 3, 1, 2000, d)), UpdateResult::kApplied);
  std::vector<uint8_t> expect(stale.size(), 0);
  for (unsigned i = 7; i < 10; ++i) {expect[8 * sx + i] = 100;}
  EXPECT_EQ(grabGrid(m), expect);
  EXPECT_EQ(m.stats().sc_bbox_scans, 0u);   // geometry_only 뒤에는 훑을 필요가 없다
}

// 같은 geometry 에서 이미 적용한 update 보다 오래된 full 은 kRejectedSuperseded (resync 아님). geometry 가 바뀐 full 은 적용.
TEST(CostmapMirrorSelfContained, OlderFullThanAppliedUpdateIsSuperseded)
{
  const unsigned sx = 30, sy = 20;
  CostmapMirror m(scOptions());
  ASSERT_TRUE(m.applyFull(makeFull(sx, sy, 0.03, 0.0, 0.0, 1000, std::vector<uint8_t>(sx * sy, 0))).applied());
  std::vector<uint8_t> d(4, 77);
  ASSERT_EQ(m.applyUpdate(makeUpdate(2, 3, 2, 2, 3000, d)), UpdateResult::kApplied);
  const auto before = grabGrid(m);
  // get_costmap 응답(스냅샷 2000)이 늦게 도착: 지금 grid(3000 의 update)가 더 새 상태
  std::vector<uint8_t> snap(sx * sy, 0); snap[0] = 254;   // 2000 시점에는 다른 곳에 agent
  const auto out = m.applyFull(makeFull(sx, sy, 0.03, 0.0, 0.0, 2000, snap), 1);
  EXPECT_EQ(out.result, FullResult::kRejectedSuperseded);
  EXPECT_FALSE(out.applied());
  EXPECT_FALSE(out.needs_resync);
  EXPECT_EQ(grabGrid(m), before);
  EXPECT_EQ(m.stats().superseded_fulls, 1u);
  // 크기가 바뀐 full 은 오래돼도 (last full 1000 보다는 새것) 적용한다
  const auto out2 = m.applyFull(makeFull(sx + 5, sy, 0.03, 0.0, 0.0, 2500, std::vector<uint8_t>((sx + 5) * sy, 0)));
  EXPECT_EQ(out2.result, FullResult::kAppliedNewSignature);
}

// grid 전에 온 update 여럿 (한도 초과): 가장 새 것 하나만 replay 해 full 내용 대신 쓴다. needs_resync 없음.
TEST(CostmapMirrorSelfContained, ReplayUsesNewestUpdateOnly)
{
  const unsigned sx = 40, sy = 30;
  CostmapMirror::Options o = scOptions();
  o.replay_max_msgs = 4;
  CostmapMirror m(o);
  MaskGen gen(sx, sy, 99);
  nav2_msgs::msg::CostmapUpdate::ConstSharedPtr last;
  int n = 0;
  for (int c = 0; c < 40; ++c) {
    auto u = gen.cycle(5000 + c, true);
    if (!u) {continue;}
    EXPECT_EQ(m.applyUpdate(u), UpdateResult::kNoGrid);
    last = u; ++n;
  }
  ASSERT_GT(n, 10);
  // get_costmap 스냅샷은 그보다 오래됨 (요청 뒤 update 가 계속 옴). 내용은 엉뚱한 값.
  std::vector<uint8_t> snap(sx * sy, 0); snap[3] = 254; snap[sx * sy - 2] = 254;
  const auto out = m.applyFull(makeFull(sx, sy, 0.03, 0.0, 0.0, 5001, snap));
  ASSERT_TRUE(out.applied());
  EXPECT_FALSE(out.needs_resync);
  EXPECT_EQ(out.replayed, 1u);
  EXPECT_EQ(grabGrid(m), gen.grid);
  EXPECT_EQ(m.stats().unreplayable_updates, 0u);
}

// 리뷰 지적 재현: 일반 모드 replay 한도 64 에서 스냅샷 뒤 70 개가 오면 needs_resync (5 Hz mask 면 12.8 s 초과).
//   한도 256 이면 아니다. self_contained 면 몇 개든 아니다. replay_loss_resyncs 는 실제로 resync 를 세운 횟수만 센다.
TEST(CostmapMirrorSelfContained, ReplayCapVersusSnapshotLatency)
{
  const unsigned sx = 64, sy = 48;
  const std::vector<uint8_t> zeros(static_cast<size_t>(sx) * sy, 0);
  for (const size_t cap : {size_t{64}, size_t{256}}) {
    for (const bool sc : {false, true}) {
      for (const int between : {60, 70, 300}) {
        CostmapMirror::Options o; o.replay_max_msgs = cap; o.self_contained_updates = sc;
        CostmapMirror m(o);
        MaskGen gen(sx, sy, 5);
        ASSERT_TRUE(m.applyFull(makeFull(sx, sy, 0.03, 0.0, 0.0, 1000, zeros)).applied());
        int64_t ns = 2000;
        for (int i = 0; i < 200; ++i) {   // 정상 동작: 계속 update (한도에서 밀려남)
          auto u = gen.cycle(ns++, true); if (u) {m.applyUpdate(u);}
        }
        const int64_t snap_ns = ns++;
        const auto snap_grid = gen.grid;   // 스냅샷 = 그 시점 mask
        for (int i = 0; i < between; ++i) {
          auto u = gen.cycle(ns++, true); if (u) {m.applyUpdate(u);}
        }
        const auto out = m.applyFull(makeFull(sx, sy, 0.03, 0.0, 0.0, snap_ns, snap_grid), 1);
        const bool expect_resync = !sc && static_cast<size_t>(between) > cap;
        SCOPED_TRACE("cap " + std::to_string(cap) + " sc " + std::to_string(sc) + " between " + std::to_string(between));
        if (sc) {
          EXPECT_EQ(out.result, FullResult::kRejectedSuperseded);   // 이미 더 새 update 를 적용함
          EXPECT_EQ(grabGrid(m), gen.grid);
        } else {
          ASSERT_TRUE(out.applied());
          EXPECT_EQ(out.needs_resync, expect_resync);
          EXPECT_EQ(m.stats().replay_loss_resyncs, expect_resync ? 1u : 0u);
          if (!expect_resync) {EXPECT_EQ(grabGrid(m), gen.grid);}
        }
        std::printf("[ReplayCap] cap=%zu sc=%d between=%d -> needs_resync=%d unreplayable(누적 밀려남)=%lu "
                    "replay_loss_resyncs=%lu\n", cap, sc ? 1 : 0, between, out.needs_resync ? 1 : 0,
          static_cast<unsigned long>(m.stats().unreplayable_updates),
          static_cast<unsigned long>(m.stats().replay_loss_resyncs));
      }
    }
  }
}

// self_contained 에서 한도보다 큰 update (activate 뒤 grid 전체 update) 는 replay 에 두지 않지만, 그보다 오래된 full 은
// superseded 로 버려 되돌아가지 않는다.
TEST(CostmapMirrorSelfContained, OversizeUpdateNotBufferedButOlderFullSuperseded)
{
  const unsigned sx = 64, sy = 64;
  CostmapMirror::Options o = scOptions();
  o.replay_max_bytes = 1024;
  CostmapMirror m(o);
  ASSERT_TRUE(m.applyFull(makeFull(sx, sy, 0.03, 0.0, 0.0, 1000, std::vector<uint8_t>(sx * sy, 0))).applied());
  std::vector<uint8_t> all(sx * sy, 0); all[100] = 50;
  ASSERT_EQ(m.applyUpdate(makeUpdate(0, 0, sx, sy, 3000, all)), UpdateResult::kApplied);
  EXPECT_EQ(m.applyFull(makeFull(sx, sy, 0.03, 0.0, 0.0, 2000, std::vector<uint8_t>(sx * sy, 9))).result,
    FullResult::kRejectedSuperseded);
  EXPECT_EQ(grabGrid(m), all);
}

// self_contained 동시성: writer 가 update(직전 사각형 0 + 새 사각형 복사)와 가끔 full(뒤 첫 update 는 grid 를 lock 밖에서 훑음)을
//   넣는 동안 reader 4 개가 읽는다. 사이클 c 의 mask 는 "사각형 하나를 값 v_c 로 꽉 채운 것" 이라, 반쯤 적용된 상태
//   (직전 사각형만 지워졌거나 두 값이 섞임)를 보면 바로 드러난다. ThreadSanitizer 빌드에서도 돈다 (훑기는 읽기만 한다).
TEST(CostmapMirrorSelfContained, ConcurrentReadersNeverSeeHalfAppliedSelfContainedUpdate)
{
  const unsigned sx = 300, sy = 200;
  CostmapMirror m(scOptions());
  std::vector<uint8_t> mask(static_cast<size_t>(sx) * sy, 0);
  ASSERT_TRUE(m.applyFull(makeFull(sx, sy, 0.03, 0.0, 0.0, 1000, mask)).applied());
  std::atomic<bool> stop{false};
  std::atomic<uint64_t> reads{0}, torn{0};
  std::vector<std::thread> readers;
  for (int t = 0; t < 4; ++t) {
    readers.emplace_back([&]() {
        while (!stop.load()) {
          auto g = m.read();
          if (!g) {continue;}
          const unsigned char * d = g->getCharMap();
          const size_t n = static_cast<size_t>(g->getSizeInCellsX()) * g->getSizeInCellsY();
          unsigned char v = 0;
          size_t cnt = 0;
          unsigned x0 = sx, y0 = sy, xn = 0, yn = 0;
          bool mixed = false;
          for (size_t k = 0; k < n; ++k) {
            if (d[k] == 0) {continue;}
            if (v == 0) {v = d[k];} else if (d[k] != v) {mixed = true; break;}
            ++cnt;
            const unsigned x = static_cast<unsigned>(k % sx), y = static_cast<unsigned>(k / sx);
            x0 = std::min(x0, x); xn = std::max(xn, x + 1); y0 = std::min(y0, y); yn = std::max(yn, y + 1);
          }
          // 0 아닌 셀은 한 값이고 사각형을 꽉 채운다
          if (mixed || (cnt > 0 && cnt != static_cast<size_t>(xn - x0) * (yn - y0))) {torn.fetch_add(1);}
          reads.fetch_add(1);
        }
      });
  }
  std::mt19937 rng(17);
  unsigned px0 = 0, py0 = 0, pw = 0, ph = 0;
  int64_t ns = 2000;
  uint64_t fulls = 0, updates = 0;
  const auto t_end = std::chrono::steady_clock::now() + std::chrono::milliseconds(1200);
  for (int c = 0; std::chrono::steady_clock::now() < t_end; ++c) {
    for (unsigned r = 0; r < ph; ++r) {std::memset(&mask[(static_cast<size_t>(py0) + r) * sx + px0], 0, pw);}
    const unsigned w = 1 + rng() % 60, h = 1 + rng() % 40;
    const unsigned x0 = rng() % (sx - w + 1), y0 = rng() % (sy - h + 1);
    const uint8_t v = static_cast<uint8_t>(1 + c % 250);
    for (unsigned r = 0; r < h; ++r) {std::memset(&mask[(static_cast<size_t>(y0) + r) * sx + x0], v, w);}
    if (c % 40 == 39) {
      ASSERT_TRUE(m.applyFull(makeFull(sx, sy, 0.03, 0.0, 0.0, ns++, mask)).applied());
      ++fulls;
    } else {
      const unsigned ux0 = ph ? std::min(px0, x0) : x0, uy0 = ph ? std::min(py0, y0) : y0;
      const unsigned uxn = ph ? std::max(px0 + pw, x0 + w) : x0 + w, uyn = ph ? std::max(py0 + ph, y0 + h) : y0 + h;
      std::vector<uint8_t> d(static_cast<size_t>(uxn - ux0) * (uyn - uy0));
      for (unsigned r = 0; r < uyn - uy0; ++r) {
        std::memcpy(&d[static_cast<size_t>(r) * (uxn - ux0)], &mask[(static_cast<size_t>(uy0) + r) * sx + ux0], uxn - ux0);
      }
      ASSERT_EQ(m.applyUpdate(makeUpdate(ux0, uy0, uxn - ux0, uyn - uy0, ns++, d)), UpdateResult::kApplied);
      ++updates;
    }
    px0 = x0; py0 = y0; pw = w; ph = h;
  }
  stop.store(true);
  for (auto & th : readers) {th.join();}
  EXPECT_EQ(grabGrid(m), mask);
  EXPECT_EQ(torn.load(), 0u);
  EXPECT_GT(reads.load(), 100u);
  EXPECT_GE(m.stats().sc_bbox_scans, fulls);
  std::printf("[SC concurrent] reads=%lu torn=%lu updates=%lu fulls=%lu bbox_scans=%lu\n",
    static_cast<unsigned long>(reads.load()), static_cast<unsigned long>(torn.load()),
    static_cast<unsigned long>(updates), static_cast<unsigned long>(fulls),
    static_cast<unsigned long>(m.stats().sc_bbox_scans));
}

// [10-10 W1-4] global source 전환 (diff layer ↔ stock). beginNewSource 는 grid 를 reader 용으로 남기고, 옛 source 의 stamp 기준·
//   replay 를 버린다. 새 source 의 첫 full 은 stamp 와 상관없이 적용하고, 그 전에 온 update 는 grid 에 쓰지 않고 보관했다가
//   그 full 보다 새것만 다시 적용한다. 음성 대조 (beginNewSource 없음 = W1-3): 옛 source 보다 오래된 stamp 의 새 source full 은
//   kRejectedStale, 그 사이 새 source update 는 옛 source grid 위에 섞여 들어간다.
TEST(CostmapMirror, BeginNewSourceKeepsGridAndRebasesOnFirstFull)
{
  const unsigned sx = 20, sy = 10;
  for (bool rule : {true, false}) {
    CostmapMirror m;
    // 옛 source (stock): full t=1000 (모두 5) + update t=1100
    ASSERT_EQ(full_result(m, makeFull(sx, sy, 0.1, 0, 0, 1000, std::vector<uint8_t>(sx * sy, 5))),
      FullResult::kAppliedNewSignature);
    ASSERT_EQ(m.applyUpdate(makeUpdate(0, 0, 1, 1, 1100, {9})), UpdateResult::kApplied);
    const auto gen_before = m.fullGeneration();
    if (rule) {
      m.beginNewSource();
      EXPECT_TRUE(m.baselinePending());
      EXPECT_EQ(m.stats().source_resets, 1u);
      EXPECT_GT(m.fullGeneration(), gen_before);
    }
    // 전환 중에도 reader 는 옛 grid 를 읽는다
    ASSERT_TRUE(m.hasGrid());
    EXPECT_EQ(m.read()->getCost(3, 3), 5);
    // 새 source (diff): durable update t=700, t=1200 이 full 보다 먼저 온다
    const auto r700 = m.applyUpdate(makeUpdate(1, 1, 1, 1, 700, {11}));
    const auto r1200 = m.applyUpdate(makeUpdate(2, 2, 1, 1, 1200, {12}));
    if (rule) {
      EXPECT_EQ(r700, UpdateResult::kNoGrid);     // 보관만
      EXPECT_EQ(r1200, UpdateResult::kNoGrid);
      EXPECT_EQ(m.read()->getCost(2, 2), 5);       // 옛 grid 에 섞이지 않는다
    } else {
      EXPECT_EQ(r700, UpdateResult::kStale);       // 옛 기준 (1000) 보다 오래됐다고 버림
      EXPECT_EQ(r1200, UpdateResult::kApplied);    // 옛 grid 위에 섞임
    }
    // 새 source 의 latched FULL t=800 (옛 기준 1000 보다 오래됨, 내용 모두 3, (1,1)=11 은 t=700 에 이미 반영)
    std::vector<uint8_t> d(sx * sy, 3);
    d[1 * sx + 1] = 11;
    const auto o = m.applyFull(makeFull(sx, sy, 0.1, 0, 0, 800, d));
    std::printf("[BeginNewSource] rule=%d latched_full=%d cell22=%d cell00=%d replayed=%lu\n", rule ? 1 : 0,
      static_cast<int>(o.result), m.read()->getCost(2, 2), m.read()->getCost(0, 0),
      static_cast<unsigned long>(o.replayed));
    if (rule) {
      EXPECT_EQ(o.result, FullResult::kApplied);
      EXPECT_FALSE(o.clock_reset);
      EXPECT_EQ(o.replayed, 1u);                    // t=1200 만 (t=700 은 full 보다 오래됨)
      EXPECT_FALSE(m.baselinePending());
      auto g = grabGrid(m);
      std::vector<uint8_t> want(d);
      want[2 * sx + 2] = 12;
      EXPECT_EQ(g, want);                           // 새 source 상태 그대로 (옛 (0,0)=9 는 사라짐)
      // 이후는 예전 규칙: full 보다 오래된 update 는 stale
      EXPECT_EQ(m.applyUpdate(makeUpdate(4, 4, 1, 1, 790, {1})), UpdateResult::kStale);
      EXPECT_EQ(m.applyUpdate(makeUpdate(4, 4, 1, 1, 1300, {13})), UpdateResult::kApplied);
      EXPECT_EQ(full_result(m, makeFull(sx, sy, 0.1, 0, 0, 750, d)), FullResult::kRejectedStale);
    } else {
      EXPECT_EQ(o.result, FullResult::kRejectedStale);   // 음성 대조: 새 source 의 full 을 못 쓴다
      EXPECT_EQ(m.read()->getCost(0, 0), 9);              // 옛 source 내용이 남는다
    }
  }
}

// [10-10 W1-4] 전환 직후 보관한 update 가 replay 한도에서 밀려나면 (새 full 전이라 grid 에 없다) 잃은 것으로 보고
//   새 full 적용 뒤 needs_resync 를 세운다 (옛 기준이었다면 "현재 full 보다 오래됨" 으로 잃은 것이 아니라고 볼 수 있었다).
TEST(CostmapMirror, BeginNewSourceReplayOverflowRequestsResync)
{
  const unsigned sx = 10, sy = 10;
  CostmapMirror::Options o;
  o.replay_max_msgs = 2;
  CostmapMirror m(o);
  ASSERT_TRUE(m.applyFull(makeFull(sx, sy, 0.1, 0, 0, 5000, std::vector<uint8_t>(sx * sy, 0))).applied());
  m.beginNewSource();
  for (int k = 0; k < 4; ++k) {
    EXPECT_EQ(m.applyUpdate(makeUpdate(k, 0, 1, 1, 100 + k, {static_cast<uint8_t>(50 + k)})), UpdateResult::kNoGrid);
  }
  const auto out = m.applyFull(makeFull(sx, sy, 0.1, 0, 0, 50, std::vector<uint8_t>(sx * sy, 0)));
  EXPECT_TRUE(out.applied());
  EXPECT_TRUE(out.needs_resync);   // t=100, 101 은 밀려났다 (full t=50 보다 새것)
  EXPECT_EQ(out.replayed, 2u);
}

// [10-10 W1-4][리뷰] 빈 full (DiffPublisherLayer 의 get_costmap 이 첫 cycle 전·snapshot 실패·enabled:false 때 돌려주는 기본값
//   nav2_msgs::msg::Costmap(): 0x0, resolution 0, stamp 0) 은 어느 상태에서도 grid 가 되지 않는다. 예전에는 data.size()==0 ==
//   size_x*size_y 로 크기 검사를 통과해 0x0 grid 가 kAppliedNewSignature 로 들어갔다 (리뷰가 빌드된 .o 로 세 상태 모두 재현,
//   steady state 는 clock_reset=1 로 replay 까지 비움). 세 상태: (a) grid 없음, (b) beginNewSource 뒤 baseline_pending,
//   (c) 현재 grid 뒤 보낸 요청의 응답 (clock reset 경로). 각 상태에서 grid·baseline_pending·full generation·replay 가 그대로인지.
TEST(CostmapMirror, EmptyFullRejectedInEveryState)
{
  const unsigned sx = 20, sy = 10;
  const nav2_msgs::msg::Costmap empty;   // layer 의 빈 응답 그대로
  // (a) grid 없음 — requested_at_gen 이 있든 없든
  {
    CostmapMirror m;
    ASSERT_EQ(m.applyUpdate(makeUpdate(1, 1, 1, 1, 500, {7})), UpdateResult::kNoGrid);   // 보관
    const auto g0 = m.fullGeneration();
    const auto o1 = m.applyFull(empty);
    const auto o2 = m.applyFull(empty, g0);
    EXPECT_EQ(o1.result, FullResult::kRejectedEmpty);
    EXPECT_EQ(o2.result, FullResult::kRejectedEmpty);
    EXPECT_FALSE(o1.applied());
    EXPECT_FALSE(m.hasGrid());
    EXPECT_EQ(m.fullGeneration(), g0);
    EXPECT_EQ(m.stats().empty_fulls, 2u);
    EXPECT_EQ(m.stats().fulls, 0u);
    // 진짜 full 이 오면 보관한 update 가 replay 된다 (replay 를 비우지 않았다)
    const auto o = m.applyFull(makeFull(sx, sy, 0.1, 0, 0, 400, std::vector<uint8_t>(sx * sy, 0)));
    EXPECT_EQ(o.result, FullResult::kAppliedNewSignature);
    EXPECT_EQ(o.replayed, 1u);
    EXPECT_EQ(m.read()->getCost(1, 1), 7);
  }
  // (b) source 전환 뒤 baseline_pending (switch-back 직후 configure~activate 사이 응답)
  {
    CostmapMirror m;
    ASSERT_TRUE(m.applyFull(makeFull(sx, sy, 0.1, 0, 0, 1000, std::vector<uint8_t>(sx * sy, 5))).applied());
    m.beginNewSource();
    ASSERT_TRUE(m.baselinePending());
    ASSERT_EQ(m.applyUpdate(makeUpdate(2, 2, 1, 1, 1200, {12})), UpdateResult::kNoGrid);
    const auto g0 = m.fullGeneration();
    const auto o = m.applyFull(empty, g0);
    EXPECT_EQ(o.result, FullResult::kRejectedEmpty);
    EXPECT_FALSE(o.clock_reset);
    EXPECT_TRUE(m.baselinePending());                 // 새 source 의 첫 full 을 아직 기다린다
    EXPECT_EQ(m.fullGeneration(), g0);
    EXPECT_EQ(m.read()->getSizeInCellsX(), sx);       // 옛 (stock) grid 그대로
    EXPECT_EQ(m.read()->getCost(3, 3), 5);
    const auto o2 = m.applyFull(makeFull(sx, sy, 0.1, 0, 0, 800, std::vector<uint8_t>(sx * sy, 3)));
    EXPECT_EQ(o2.result, FullResult::kApplied);       // 옛 기준보다 오래돼도 새 source 기준으로 적용
    EXPECT_EQ(o2.replayed, 1u);
    EXPECT_EQ(m.read()->getCost(2, 2), 12);
    EXPECT_FALSE(m.baselinePending());
  }
  // (c) steady state, 현재 grid 뒤 보낸 요청의 응답 (예전: stamp 0 < 현재 → clock reset 으로 적용, replay 비움)
  {
    CostmapMirror m;
    ASSERT_TRUE(m.applyFull(makeFull(sx, sy, 0.1, 0, 0, 1000, std::vector<uint8_t>(sx * sy, 5))).applied());
    const auto req_gen = m.fullGeneration();
    ASSERT_EQ(m.applyUpdate(makeUpdate(4, 4, 1, 1, 1100, {44})), UpdateResult::kApplied);
    const auto o = m.applyFull(empty, req_gen);
    EXPECT_EQ(o.result, FullResult::kRejectedEmpty);
    EXPECT_FALSE(o.clock_reset);
    EXPECT_FALSE(o.needs_resync);
    EXPECT_EQ(m.fullGeneration(), req_gen);
    EXPECT_EQ(m.stats().clock_resets, 0u);
    EXPECT_EQ(m.read()->getSizeInCellsX(), sx);
    EXPECT_EQ(m.read()->getCost(4, 4), 44);
    // replay 가 남아 있다: t=1050 full 뒤에 t=1100 update 가 다시 덮인다
    const auto o2 = m.applyFull(makeFull(sx, sy, 0.1, 0, 0, 1050, std::vector<uint8_t>(sx * sy, 5)), req_gen);
    EXPECT_TRUE(o2.applied());
    EXPECT_EQ(o2.replayed, 1u);
    EXPECT_EQ(m.read()->getCost(4, 4), 44);
  }
  // 경계: 크기는 있는데 resolution 0 / 크기 0 인데 resolution 있음 — 둘 다 빈 것으로 본다
  {
    CostmapMirror m;
    EXPECT_EQ(m.applyFull(makeFull(sx, sy, 0.0, 0, 0, 10, std::vector<uint8_t>(sx * sy, 1))).result,
      FullResult::kRejectedEmpty);
    EXPECT_EQ(m.applyFull(makeFull(0, sy, 0.1, 0, 0, 10, {})).result, FullResult::kRejectedEmpty);
    EXPECT_EQ(m.applyFull(makeFull(sx, 0, 0.1, 0, 0, 10, {})).result, FullResult::kRejectedEmpty);
    EXPECT_FALSE(m.hasGrid());
  }
  std::printf("[EmptyFull] (a)(b)(c) 모두 kRejectedEmpty=%d, grid·baseline·generation·replay 그대로\n",
    static_cast<int>(FullResult::kRejectedEmpty));
}
