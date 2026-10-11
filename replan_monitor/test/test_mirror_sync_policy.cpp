// [10-10 W1-3] mirror_sync_policy.hpp + CostmapMirror::seedZero 시험. ROS node 를 만들지 않는다 (DDS 트래픽 없음).
//
//   A1 seed      : mask 를 global geometry 의 0 grid 로 시작, 먼저 온 update 중 가장 새 것 적용, 이후 update·full 규칙
//   A2 모름 판정 : decideAgentMaskMode / skipSelfFootprintCell / 귀속 경로
//   A3 W2PV-2    : decideMaskTopicFull (구독 뒤 첫 full + update 없음 + 오래됨 일 때만 geometry_only, 같은 geometry 면 버림)
//   A1 latched   : dropLatchedMaskUpdate
//   A4 W2PV-3/R3 : ResyncScheduler — 진행 중 트리거 무시 (중복 요청 없음), global → mask 순서
//                  [10-10 W1-3][리뷰] timeout 은 늦음 경고만 (늦은 응답도 적용), 포기는 abandon_sec 뒤 30/60/120 s backoff,
//                  진행 중 내용 유실 트리거는 응답 뒤 한 번 더
//   [리뷰] READY : readyHoldWithoutGlobalGrid — global grid 가 없으면 출발 보류
//   A5 R4        : evaluateGlobalWatchdog
//   통합 흉내    : node 의 MirrorSync 가 부르는 순서대로 두 scheduler + 두 mirror 를 돌려 현장 시작 순서·respawn 을 본다.
//   [10-10 W1-4] GlobalSourceSelector (diff → stock fallback 한 번, 복귀 한 번), watchdog liveness 기준 (diff source 는 침묵이 정상),
//                ResyncScheduler::resetForNewSource, source 전환 흐름 (옛 요청 응답은 세대 검사로 버림 — 중복 요청 없음).

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <random>
#include <vector>

#include "replan_monitor/costmap_mirror.hpp"
#include "replan_monitor/mirror_sync_policy.hpp"

using replan_monitor::AgentMaskMode;
using replan_monitor::CostmapMirror;
using replan_monitor::CostmapSignature;
using replan_monitor::MaskTopicFullAction;
using replan_monitor::ResyncScheduler;
using Tick = ResyncScheduler::Tick;
using Clock = std::chrono::steady_clock;

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
  unsigned sx, unsigned sy, double res, double ox, double oy, int64_t ns, const std::vector<uint8_t> & data)
{
  nav2_msgs::msg::Costmap m;
  m.header.stamp = stamp(ns);
  m.metadata.size_x = sx;
  m.metadata.size_y = sy;
  m.metadata.resolution = static_cast<float>(res);
  m.metadata.origin.position.x = ox;
  m.metadata.origin.position.y = oy;
  m.data = data;
  return m;
}

nav2_msgs::msg::CostmapUpdate::ConstSharedPtr makeUpdate(
  unsigned x, unsigned y, unsigned w, unsigned h, int64_t ns, uint8_t v)
{
  auto u = std::make_shared<nav2_msgs::msg::CostmapUpdate>();
  u->header.stamp = stamp(ns);
  u->x = x; u->y = y; u->size_x = w; u->size_y = h;
  u->data.assign(static_cast<size_t>(w) * h, v);
  return u;
}

std::vector<uint8_t> grab(const CostmapMirror & m)
{
  auto g = m.read();
  if (!g) {return {};}
  const size_t n = static_cast<size_t>(g->getSizeInCellsX()) * g->getSizeInCellsY();
  return std::vector<uint8_t>(g->getCharMap(), g->getCharMap() + n);
}

// 기준 mask: 0 grid 위에 update 사각형 하나 (self-contained: update 하나 = mask 전체)
std::vector<uint8_t> expectFrom(unsigned sx, unsigned sy, const nav2_msgs::msg::CostmapUpdate & u)
{
  std::vector<uint8_t> g(static_cast<size_t>(sx) * sy, 0);
  for (unsigned r = 0; r < u.size_y; ++r) {
    for (unsigned c = 0; c < u.size_x; ++c) {
      g[(static_cast<size_t>(u.y) + r) * sx + u.x + c] = u.data[static_cast<size_t>(r) * u.size_x + c];
    }
  }
  return g;
}

CostmapSignature sig(unsigned sx, unsigned sy, double res, double ox, double oy)
{
  CostmapSignature s;
  s.size_x = sx; s.size_y = sy; s.resolution = res; s.origin_x = ox; s.origin_y = oy;
  return s;
}

CostmapMirror::Options scOpts()
{
  CostmapMirror::Options o;
  o.self_contained_updates = true;
  return o;
}

Clock::time_point T(double s)
{
  return Clock::time_point{} + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(s + 1000.0));
}

}  // namespace

// ============================== A1 seed ==============================

TEST(MaskSeed, SeedZeroAppliesNewestBufferedUpdateOnly)
{
  CostmapMirror m(scOpts());
  // grid 없이 update 두 개 (kNoGrid → 보관). self-contained 는 가장 새 것 하나만 둔다.
  EXPECT_EQ(m.applyUpdate(makeUpdate(2, 2, 3, 3, 1000, 200)), CostmapMirror::UpdateResult::kNoGrid);
  auto u2 = makeUpdate(10, 5, 4, 2, 2000, 254);
  EXPECT_EQ(m.applyUpdate(u2), CostmapMirror::UpdateResult::kNoGrid);
  EXPECT_FALSE(m.hasGrid());
  EXPECT_FALSE(m.contentKnown());

  const auto r = m.seedZero(sig(40, 30, 0.05, -1.0, 2.0));
  EXPECT_TRUE(r.seeded);
  EXPECT_EQ(r.replayed, 1u);
  EXPECT_TRUE(m.hasGrid());
  EXPECT_TRUE(m.contentKnown());
  EXPECT_EQ(grab(m), expectFrom(40, 30, *u2));
  const auto st = m.stats();
  EXPECT_EQ(st.seeds, 1u);
  EXPECT_EQ(st.sig.size_x, 40u);
  // 두 번째 seed 는 아무것도 하지 않는다
  EXPECT_FALSE(m.seedZero(sig(40, 30, 0.05, -1.0, 2.0)).seeded);
}

TEST(MaskSeed, SeedWithoutUpdateIsEmptyButContentUnknownThenFirstUpdateExact)
{
  CostmapMirror m(scOpts());
  const auto r = m.seedZero(sig(50, 40, 0.03, 0.0, 0.0));
  ASSERT_TRUE(r.seeded);
  EXPECT_EQ(r.replayed, 0u);
  EXPECT_FALSE(m.contentKnown());   // 0 grid 는 근거가 없다 (이웃이 있으면 "모름")
  EXPECT_EQ(grab(m), std::vector<uint8_t>(50 * 40, 0));
  // stamp 기준이 없으므로 아주 작은 stamp 의 update 도 받는다 (Nav2 clock 이 0 근처인 sim 포함)
  auto u = makeUpdate(0, 0, 5, 5, 1, 254);
  EXPECT_EQ(m.applyUpdate(u), CostmapMirror::UpdateResult::kApplied);
  EXPECT_TRUE(m.contentKnown());
  EXPECT_EQ(grab(m), expectFrom(50, 40, *u));
  // 다음 update 는 직전 사각형을 지우고 자기 사각형만 남긴다
  auto u2 = makeUpdate(30, 20, 6, 6, 2, 180);
  EXPECT_EQ(m.applyUpdate(u2), CostmapMirror::UpdateResult::kApplied);
  EXPECT_EQ(grab(m), expectFrom(50, 40, *u2));
}

TEST(MaskSeed, NoSeedForNonSelfContainedOrBadGeometry)
{
  CostmapMirror g;   // global 같은 일반 mirror 에는 0 grid 를 만들지 않는다 (내용이 틀린다)
  EXPECT_FALSE(g.seedZero(sig(10, 10, 0.05, 0, 0)).seeded);
  EXPECT_FALSE(g.hasGrid());
  CostmapMirror m(scOpts());
  EXPECT_FALSE(m.seedZero(sig(0, 10, 0.05, 0, 0)).seeded);
  EXPECT_FALSE(m.seedZero(sig(10, 10, 0.0, 0, 0)).seeded);
  // grid 가 이미 있으면 (topic full 을 받았다) seed 하지 않는다
  ASSERT_TRUE(m.applyFull(makeFull(10, 10, 0.05, 0, 0, 100, std::vector<uint8_t>(100, 7))).applied());
  EXPECT_FALSE(m.seedZero(sig(10, 10, 0.05, 0, 0)).seeded);
  EXPECT_EQ(grab(m), std::vector<uint8_t>(100, 7));
}

TEST(MaskSeed, AfterSeedFullsFollowSupersededAndGeometryRules)
{
  CostmapMirror m(scOpts());
  ASSERT_TRUE(m.seedZero(sig(20, 20, 0.05, 0.0, 0.0)).seeded);
  auto u = makeUpdate(3, 3, 2, 2, 5000, 254);
  ASSERT_EQ(m.applyUpdate(u), CostmapMirror::UpdateResult::kApplied);
  // 같은 geometry 의 오래된 full (latched activation full, get_costmap 늦은 응답) → superseded, grid 그대로
  auto old_full = makeFull(20, 20, 0.05, 0.0, 0.0, 4000, std::vector<uint8_t>(400, 99));
  EXPECT_EQ(m.applyFull(old_full).result, CostmapMirror::FullResult::kRejectedSuperseded);
  EXPECT_EQ(grab(m), expectFrom(20, 20, *u));
  // 더 새 full (예전 AgentLayer 가 보내는 full) 은 내용째 적용 (1 ulp 다른 origin 도 같은 geometry)
  std::vector<uint8_t> d(400, 0); d[0] = 254;
  auto new_full = makeFull(20, 20, 0.05, 1e-12, 0.0, 6000, d);
  const auto o = m.applyFull(new_full);
  EXPECT_EQ(o.result, CostmapMirror::FullResult::kApplied);
  EXPECT_EQ(grab(m), d);
  EXPECT_TRUE(m.contentKnown());
  // geometry 가 다른 full 은 grid 를 바꾼다
  auto other = makeFull(10, 10, 0.05, 1.0, 1.0, 7000, std::vector<uint8_t>(100, 1));
  EXPECT_EQ(m.applyFull(other).result, CostmapMirror::FullResult::kAppliedNewSignature);
  // seed 와 맞지 않는 update 는 범위 밖 → node 가 get_costmap fallback (mismatch)
  CostmapMirror m2(scOpts());
  ASSERT_TRUE(m2.seedZero(sig(20, 20, 0.05, 0.0, 0.0)).seeded);
  EXPECT_EQ(m2.applyUpdate(makeUpdate(18, 18, 5, 5, 10, 254)), CostmapMirror::UpdateResult::kRejectedOutOfBounds);
}

TEST(MaskSeed, GeometryOnlyFullIsContentUnknownUntilUpdate)
{
  CostmapMirror m(scOpts());
  auto f = makeFull(8, 8, 0.05, 0, 0, 100, std::vector<uint8_t>(64, 254));
  ASSERT_TRUE(m.applyFull(f, CostmapMirror::kNoRequestGen, true).applied());
  EXPECT_FALSE(m.contentKnown());
  EXPECT_EQ(grab(m), std::vector<uint8_t>(64, 0));
  ASSERT_EQ(m.applyUpdate(makeUpdate(0, 0, 1, 1, 200, 254)), CostmapMirror::UpdateResult::kApplied);
  EXPECT_TRUE(m.contentKnown());
  // 일반 mirror 는 full 이 곧 근거다
  CostmapMirror g;
  ASSERT_TRUE(g.applyFull(f).applied());
  EXPECT_TRUE(g.contentKnown());
}

// ============================== A2 모름 판정 ==============================

TEST(AgentMaskMode, DecisionTable)
{
  using replan_monitor::decideAgentMaskMode;
  // compare_agent_mask=false → 예전 경로
  EXPECT_EQ(decideAgentMaskMode(false, true, true, true), AgentMaskMode::kDisabled);
  // grid 없음 → 모름 (이웃 유무 무관)
  EXPECT_EQ(decideAgentMaskMode(true, false, false, false), AgentMaskMode::kUnknown);
  EXPECT_EQ(decideAgentMaskMode(true, false, false, true), AgentMaskMode::kUnknown);
  // 내용 근거 있음 → 앎
  EXPECT_EQ(decideAgentMaskMode(true, true, true, true), AgentMaskMode::kKnown);
  EXPECT_EQ(decideAgentMaskMode(true, true, true, false), AgentMaskMode::kKnown);
  // seed 한 0 grid, update 아직 없음: 이웃이 없으면 0 이 맞다, 있으면 모름
  EXPECT_EQ(decideAgentMaskMode(true, true, false, false), AgentMaskMode::kKnown);
  EXPECT_EQ(decideAgentMaskMode(true, true, false, true), AgentMaskMode::kUnknown);
}

TEST(AgentMaskMode, SelfFootprintSkipAndAttribution)
{
  using replan_monitor::skipSelfFootprintCell;
  using replan_monitor::attributeWithMask;
  using replan_monitor::useMaskForGoalCheck;
  using replan_monitor::fallbackGoalAttribution;
  // 발자국 밖은 언제나 HIT
  for (auto m : {AgentMaskMode::kDisabled, AgentMaskMode::kKnown, AgentMaskMode::kUnknown}) {
    EXPECT_FALSE(skipSelfFootprintCell(m, false, false, false));
  }
  // amhs 와 같음: disabled 는 무조건 건너뜀, known 은 mask 가 이웃이라 하면 HIT
  EXPECT_TRUE(skipSelfFootprintCell(AgentMaskMode::kDisabled, true, true, true));
  EXPECT_TRUE(skipSelfFootprintCell(AgentMaskMode::kKnown, true, false, true));
  EXPECT_FALSE(skipSelfFootprintCell(AgentMaskMode::kKnown, true, true, false));
  // 모름: mask 는 보지 않고 whoCoversPoint 결과로 (이웃이 덮으면 HIT 로 남긴다 — S8 실패 모드 방지)
  EXPECT_FALSE(skipSelfFootprintCell(AgentMaskMode::kUnknown, true, false, true));
  EXPECT_TRUE(skipSelfFootprintCell(AgentMaskMode::kUnknown, true, true, false));
  // 귀속: known 만 mask 경로
  EXPECT_TRUE(attributeWithMask(AgentMaskMode::kKnown));
  EXPECT_FALSE(attributeWithMask(AgentMaskMode::kUnknown));
  EXPECT_FALSE(attributeWithMask(AgentMaskMode::kDisabled));
  EXPECT_TRUE(useMaskForGoalCheck(AgentMaskMode::kKnown));
  EXPECT_FALSE(useMaskForGoalCheck(AgentMaskMode::kUnknown));
  EXPECT_TRUE(fallbackGoalAttribution(AgentMaskMode::kUnknown, true));
  EXPECT_FALSE(fallbackGoalAttribution(AgentMaskMode::kUnknown, false));
  EXPECT_FALSE(fallbackGoalAttribution(AgentMaskMode::kKnown, true));
  EXPECT_FALSE(fallbackGoalAttribution(AgentMaskMode::kDisabled, true));   // amhs 와 같음 (정적)
}

// ============================== A3 / A1 latched ==============================

TEST(MaskTopicFull, GeometryOnlyOnlyForFirstOldFullWithoutUpdates)
{
  using replan_monitor::decideMaskTopicFull;
  const double g = 5.0;
  // 일반 costmap (self_contained 아님) 은 언제나 내용째
  EXPECT_EQ(decideMaskTopicFull(false, true, false, 100.0, g, false), MaskTopicFullAction::kApplyContent);
  // 제때 온 full 은 내용째
  EXPECT_EQ(decideMaskTopicFull(true, true, false, 1.0, g, false), MaskTopicFullAction::kApplyContent);
  // 첫 full, update 없음, 오래됨, grid 없음 → geometry 만
  EXPECT_EQ(decideMaskTopicFull(true, true, false, 30.0, g, false), MaskTopicFullAction::kApplyGeometryOnly);
  // 같은 geometry 의 grid 가 이미 있음 (seed) → 0 으로 다시 만들지 않고 버림
  EXPECT_EQ(decideMaskTopicFull(true, true, false, 30.0, g, true), MaskTopicFullAction::kDropOld);
  // 두 번째 full 부터 (예전 AgentLayer 의 full stream, 전송이 느려 늦게 와도) 내용째 — W2PV-2 의 0 만들기 방지
  EXPECT_EQ(decideMaskTopicFull(true, false, false, 30.0, g, false), MaskTopicFullAction::kApplyContent);
  EXPECT_EQ(decideMaskTopicFull(true, false, false, 30.0, g, true), MaskTopicFullAction::kApplyContent);
  // update 를 이미 받았으면 내용째 넘긴다 (mirror 가 superseded 로 거른다)
  EXPECT_EQ(decideMaskTopicFull(true, true, true, 30.0, g, true), MaskTopicFullAction::kApplyContent);
}

TEST(MaskTopicFull, LegacyFullStreamNeverZeroesMask)
{
  // W2PV-2 재현: 예전 publisher (update 없음, 5 Hz full) 의 full 이 늦게(> grace) 도착. 예전 규칙(모든 full 에 age 검사)은
  //   매번 0 으로 만들었다. 새 규칙은 첫 full 만 geometry_only 이고 그 뒤로는 내용째다.
  using replan_monitor::decideMaskTopicFull;
  CostmapMirror m(scOpts());
  uint64_t received = 0;
  int zeroed_old_rule = 0, zeroed_new_rule = 0;
  for (int k = 0; k < 20; ++k) {
    std::vector<uint8_t> d(100, 0); d[static_cast<size_t>(k) % 100] = 254;
    auto f = makeFull(10, 10, 0.05, 0, 0, 1000 + k, d);
    const double age = 8.0;   // 200 MB 전송 지연으로 늘 늦다
    if (age > 5.0) {++zeroed_old_rule;}
    const bool first = (received++ == 0);
    const auto st = m.stats();
    const bool same = st.has_grid && CostmapSignature::fromMsg(f).nearlyEquals(st.sig, 1e-6, 1e-9);
    const auto act = decideMaskTopicFull(true, first, st.updates > 0, age, 5.0, same);
    ASSERT_NE(act, MaskTopicFullAction::kDropOld);
    const bool go = act == MaskTopicFullAction::kApplyGeometryOnly;
    if (go) {++zeroed_new_rule;}
    ASSERT_TRUE(m.applyFull(f, CostmapMirror::kNoRequestGen, go).applied());
    if (k > 0) {
      EXPECT_EQ(grab(m), d) << k;
      EXPECT_TRUE(m.contentKnown());
    }
  }
  EXPECT_EQ(zeroed_old_rule, 20);
  EXPECT_EQ(zeroed_new_rule, 1);
}

TEST(MaskLatchedUpdate, DropOnlyOldLatchedBeforeFirstFreshWithinWindow)
{
  using replan_monitor::dropLatchedMaskUpdate;
  const double g = 5.0, win = 20.0;
  EXPECT_TRUE(dropLatchedMaskUpdate(true, false, 0.5, win, 7.0, g));     // respawn 직후 7 s 전 update → 버림
  EXPECT_FALSE(dropLatchedMaskUpdate(true, false, 0.5, win, 0.2, g));    // 제때 온 것
  EXPECT_FALSE(dropLatchedMaskUpdate(true, true, 0.5, win, 7.0, g));     // 이미 받아들인 update 뒤
  EXPECT_FALSE(dropLatchedMaskUpdate(true, false, 25.0, win, 7.0, g));   // window 뒤 (clock 이 달라도 결국 받는다)
  EXPECT_FALSE(dropLatchedMaskUpdate(false, false, 0.5, win, 7.0, g));   // global 에는 쓰지 않는다
  // [10-10 W1-3][리뷰] 출처 규칙: 구독보다 먼저 찍힌 update 는 나이가 grace 안쪽 (빠른 respawn) 이어도 버린다
  EXPECT_TRUE(dropLatchedMaskUpdate(true, false, 0.3, win, 4.5, g, true));
  EXPECT_TRUE(dropLatchedMaskUpdate(true, false, 0.3, win, 0.3, g, true));
  EXPECT_FALSE(dropLatchedMaskUpdate(true, false, 0.3, win, 4.5, g, false));   // 구독 뒤에 찍힌 것 (나이 규칙만)
  EXPECT_FALSE(dropLatchedMaskUpdate(true, true, 0.3, win, 4.5, g, true));     // 이미 받아들인 update 뒤
  EXPECT_FALSE(dropLatchedMaskUpdate(true, false, 25.0, win, 4.5, g, true));   // window 뒤
  EXPECT_FALSE(dropLatchedMaskUpdate(false, false, 0.3, win, 4.5, g, true));   // global
}

// ============================== A4 scheduler ==============================

TEST(ResyncScheduler, BackoffSequence)
{
  EXPECT_DOUBLE_EQ(ResyncScheduler::backoffSec(0, 30, 120), 0.0);
  EXPECT_DOUBLE_EQ(ResyncScheduler::backoffSec(1, 30, 120), 30.0);
  EXPECT_DOUBLE_EQ(ResyncScheduler::backoffSec(2, 30, 120), 60.0);
  EXPECT_DOUBLE_EQ(ResyncScheduler::backoffSec(3, 30, 120), 120.0);
  EXPECT_DOUBLE_EQ(ResyncScheduler::backoffSec(4, 30, 120), 120.0);
  EXPECT_DOUBLE_EQ(ResyncScheduler::backoffSec(50, 30, 120), 120.0);
  EXPECT_DOUBLE_EQ(ResyncScheduler::backoffSec(3, 30, 10), 30.0);   // cap < base 면 base
}

// W2PV-3 재현: 요청 진행 중 kNoGrid update 가 5 Hz 로 트리거를 건다. 예전에는 응답 때 trigger_gen != sent_gen 이라
//   needed 가 남아 같은 요청을 한 번 더 보냈다. 이제 진행 중 트리거는 무시하므로 요청은 하나다.
TEST(ResyncScheduler, NoSecondRequestFromTriggersWhileInFlight)
{
  ResyncScheduler s;
  ASSERT_TRUE(s.trigger());
  ASSERT_EQ(s.tick(T(0), true, false), Tick::kSend);
  s.onSent(T(0));
  // 16 s 응답 지연 동안 5 Hz 트리거 + 500 ms tick
  for (int k = 0; k < 80; ++k) {
    EXPECT_FALSE(s.trigger());
    if (k % 2 == 0) {EXPECT_EQ(s.tick(T(0.2 * k), true, false), Tick::kWaitInFlight);}
  }
  s.onResponseArrived();
  s.onResponseDefinitive();
  EXPECT_FALSE(s.needed());
  for (int k = 0; k < 100; ++k) {
    EXPECT_NE(s.tick(T(16.5 + 0.5 * k), true, false), Tick::kSend);
  }
  EXPECT_EQ(s.counters().requests, 1u);
  EXPECT_EQ(s.counters().ignored_in_flight, 80u);
  // 응답 뒤 새 트리거는 min_interval 뒤에 보낸다
  ASSERT_TRUE(s.trigger());
  EXPECT_EQ(s.tick(T(17.0), true, false), Tick::kSend);
}

// [10-10 W1-3][리뷰] R3: 서버 응답이 늘 timeout(30 s) 보다 늦다 (45 s). 예전 (timeout = 포기) 은 응답을 영원히 못 받는다.
//   이제 timeout 은 늦음 경고 한 번이고 요청은 하나만 둔 채 기다려 45 s 응답을 쓴다. 음성 대조: abandon_sec = timeout_sec.
TEST(ResyncScheduler, SlowResponseIsKeptAndApplied)
{
  for (bool new_rule : {true, false}) {
    ResyncScheduler::Config c;
    c.min_interval_sec = 5.0; c.timeout_sec = 30.0; c.backoff_base_sec = 30.0; c.backoff_max_sec = 120.0;
    c.abandon_sec = new_rule ? 300.0 : 30.0;
    ResyncScheduler s(c);
    s.trigger();
    const double latency = 45.0;
    double sent_at = -1.0;
    int applied = 0, slow_ticks = 0, abandoned_ticks = 0;
    double first_applied_at = -1.0;
    for (double t = 0.0; t < 600.0; t += 0.5) {
      const auto k = s.tick(T(t), true, false);
      if (k == Tick::kSlow) {++slow_ticks;}
      if (k == Tick::kTimedOut) {++abandoned_ticks; sent_at = -1.0;}   // client 가 pending 을 지운다 → 응답은 안 온다
      if (k == Tick::kSend) {s.onSent(T(t)); sent_at = t;}
      if (sent_at >= 0.0 && s.inFlight() && t - sent_at >= latency) {
        s.onResponseArrived();
        s.onResponseDefinitive();
        sent_at = -1.0;
        if (applied++ == 0) {first_applied_at = t;}
      }
    }
    std::printf("[SlowService] rule=%d requests=%lu applied=%d first_applied_at=%.1f slow=%lu late=%lu abandoned=%lu\n",
      new_rule ? 1 : 0, static_cast<unsigned long>(s.counters().requests), applied, first_applied_at,
      static_cast<unsigned long>(s.counters().timeouts), static_cast<unsigned long>(s.counters().late_responses),
      static_cast<unsigned long>(s.counters().abandoned));
    if (new_rule) {
      EXPECT_EQ(applied, 1);
      EXPECT_NEAR(first_applied_at, latency, 0.6);
      EXPECT_EQ(s.counters().requests, 1u);
      EXPECT_EQ(slow_ticks, 1);
      EXPECT_EQ(s.counters().late_responses, 1u);
      EXPECT_EQ(s.counters().abandoned, 0u);
      EXPECT_EQ(s.consecutiveFailures(), 0);
      EXPECT_FALSE(s.needed());
      EXPECT_TRUE(s.lastResponseLate());
    } else {
      EXPECT_EQ(applied, 0) << "음성 대조: timeout 에 버리면 45 s 응답은 한 번도 못 쓴다";
      EXPECT_GE(abandoned_ticks, 3);
    }
  }
}

// 응답이 아예 없다 (서버가 받은 요청을 잃음): abandon_sec 뒤 포기, 그 뒤 30 → 60 → 120 s backoff. 포기 직후 같은 tick 재요청 없음.
TEST(ResyncScheduler, AbandonBackoffThenSuccessResets)
{
  ResyncScheduler::Config c;
  c.min_interval_sec = 5.0; c.timeout_sec = 30.0; c.abandon_sec = 300.0;
  c.backoff_base_sec = 30.0; c.backoff_max_sec = 120.0;
  ResyncScheduler s(c);
  s.trigger();
  double t = 0.0;
  std::vector<double> sends;
  for (; t < 1500.0; t += 0.5) {
    const auto k = s.tick(T(t), true, false);
    if (k == Tick::kSend) {s.onSent(T(t)); sends.push_back(t);}
    if (k == Tick::kTimedOut) {
      EXPECT_NE(s.tick(T(t), true, false), Tick::kSend) << "포기 직후 같은 tick 에 재요청";
    }
  }
  ASSERT_GE(sends.size(), 4u);
  EXPECT_NEAR(sends[1] - sends[0], 300.0 + 30.0, 0.6);
  EXPECT_NEAR(sends[2] - sends[1], 300.0 + 60.0, 0.6);
  EXPECT_NEAR(sends[3] - sends[2], 300.0 + 120.0, 0.6);
  EXPECT_EQ(s.counters().abandoned + (s.inFlight() ? 1u : 0u), s.counters().requests);
  EXPECT_EQ(s.counters().timeouts, s.counters().requests);   // 요청마다 늦음 WARN 한 번
  // 성공 응답 → 연속 실패 초기화
  if (!s.inFlight()) {
    while (s.tick(T(t), true, false) != Tick::kSend) {t += 0.5;}
    s.onSent(T(t));
  }
  s.onResponseArrived();
  s.onResponseDefinitive();
  EXPECT_EQ(s.consecutiveFailures(), 0);
  EXPECT_FALSE(s.needed());
  EXPECT_DOUBLE_EQ(s.backoffRemainingSec(T(t)), 0.0);
}

// timeout 뒤 service 가 사라지면 (Nav2 재기동) 그 요청의 응답은 오지 않는다 → 바로 포기. timeout 전에는 기다린다.
TEST(ResyncScheduler, AbandonWhenServiceGoneAfterTimeout)
{
  ResyncScheduler s;   // timeout 30, abandon 300
  s.trigger();
  ASSERT_EQ(s.tick(T(0), true, false), Tick::kSend);
  s.onSent(T(0));
  EXPECT_EQ(s.tick(T(10), false, false), Tick::kWaitInFlight);   // timeout 전 service 깜빡임은 기다린다
  EXPECT_EQ(s.tick(T(31), true, false), Tick::kSlow);
  EXPECT_EQ(s.tick(T(32), true, false), Tick::kWaitInFlight);
  EXPECT_EQ(s.tick(T(40), false, false), Tick::kTimedOut);
  EXPECT_TRUE(s.needed());
  EXPECT_EQ(s.tick(T(41), true, false), Tick::kWaitBackoff);
  EXPECT_EQ(s.tick(T(70.5), true, false), Tick::kSend);
  EXPECT_EQ(s.counters().abandoned, 1u);
}

// [10-10 W1-3][리뷰] 진행 중 트리거: 구조 트리거 (kNoGrid 5 Hz 등) 는 그대로 무시, 내용 유실 (MESSAGE_LOST) 은 응답 뒤 한 번 더.
TEST(ResyncScheduler, ContentLossDuringFlightRequestsOnceMore)
{
  using Trig = ResyncScheduler::Trigger;
  ResyncScheduler s;
  s.trigger();
  ASSERT_EQ(s.tick(T(0), true, false), Tick::kSend);
  s.onSent(T(0));
  for (int k = 0; k < 20; ++k) {EXPECT_FALSE(s.trigger(Trig::kStructural));}
  EXPECT_FALSE(s.trigger(Trig::kContentLoss));
  EXPECT_FALSE(s.trigger(Trig::kContentLoss));
  EXPECT_TRUE(s.dirtyDuringFlight());
  s.onResponseArrived();
  s.onResponseDefinitive();
  EXPECT_TRUE(s.needed()) << "유실이 스냅샷 뒤였을 수 있다 → 한 번 더";
  EXPECT_FALSE(s.dirtyDuringFlight());
  EXPECT_EQ(s.tick(T(16), true, false), Tick::kSend);   // min_interval 5 s 는 지났다
  s.onSent(T(16));
  s.onResponseArrived();
  s.onResponseDefinitive();
  EXPECT_FALSE(s.needed());
  for (double t = 32; t < 100; t += 0.5) {EXPECT_NE(s.tick(T(t), true, false), Tick::kSend);}
  EXPECT_EQ(s.counters().requests, 2u);
  EXPECT_EQ(s.counters().loss_in_flight, 2u);
  EXPECT_EQ(s.counters().loss_retriggers, 1u);
  EXPECT_EQ(s.counters().ignored_in_flight, 22u);
  // 실패 응답이면 needed 가 그대로라 표시는 지운다 (backoff 뒤 한 번만)
  s.trigger();
  ASSERT_EQ(s.tick(T(100), true, false), Tick::kSend);
  s.onSent(T(100));
  s.trigger(Trig::kContentLoss);
  s.onResponseArrived();
  s.onResponseFailed(T(110));
  EXPECT_FALSE(s.dirtyDuringFlight());
  EXPECT_TRUE(s.needed());
}

TEST(ResyncScheduler, FailedResponseBacksOffAndKeepsNeeded)
{
  ResyncScheduler s;
  s.trigger();
  ASSERT_EQ(s.tick(T(0), true, false), Tick::kSend);
  s.onSent(T(0));
  s.onResponseArrived();
  s.onResponseFailed(T(1));   // 크기 불일치 / 예외
  EXPECT_TRUE(s.needed());
  EXPECT_EQ(s.tick(T(10), true, false), Tick::kWaitBackoff);
  EXPECT_EQ(s.tick(T(31.5), true, false), Tick::kSend);
  EXPECT_EQ(s.counters().failures, 1u);
}

TEST(ResyncScheduler, IntervalServiceOtherCancel)
{
  ResyncScheduler s;
  EXPECT_EQ(s.tick(T(0), true, false), Tick::kIdle);
  s.trigger();
  EXPECT_EQ(s.tick(T(0), false, false), Tick::kWaitService);
  EXPECT_EQ(s.tick(T(0), true, true), Tick::kWaitOther);   // global 이 먼저
  EXPECT_EQ(s.tick(T(0), true, false), Tick::kSend);
  s.onSent(T(0));
  s.onResponseArrived();
  s.onResponseDefinitive();
  s.trigger();
  EXPECT_EQ(s.tick(T(2), true, false), Tick::kWaitInterval);
  // seed 로 필요가 사라짐
  s.cancel();
  EXPECT_EQ(s.tick(T(10), true, false), Tick::kIdle);
  // 진행 중에는 cancel 이 needed 를 내리지 않는다 (응답이 내린다)
  s.trigger();
  ASSERT_EQ(s.tick(T(10), true, false), Tick::kSend);
  s.onSent(T(10));
  s.cancel();
  EXPECT_TRUE(s.needed());
}

// 응답 처리 중(onResponseArrived 뒤) 새 트리거 (clock reset → 한 번 더) 는 받아들이고 needed 를 남긴다.
TEST(ResyncScheduler, TriggerAfterResponseArrivedIsKept)
{
  ResyncScheduler s;
  s.trigger();
  ASSERT_EQ(s.tick(T(0), true, false), Tick::kSend);
  s.onSent(T(0));
  s.onResponseArrived();
  s.onResponseDefinitive();
  EXPECT_TRUE(s.trigger());   // onFullApplied 의 needs_resync
  EXPECT_TRUE(s.needed());
  EXPECT_EQ(s.tick(T(6), true, false), Tick::kSend);
}

// ============================== A5 watchdog ==============================

TEST(GlobalWatchdog, Flags)
{
  using namespace replan_monitor;
  GlobalWatchdogInput in;
  in.stale_sec = 10.0; in.no_grid_sec = 50.0; in.fail_threshold = 2;
  // 시작 직후 grid 없음 → 아직 정상
  in.has_grid = false; in.since_start_sec = 20.0;
  EXPECT_EQ(evaluateGlobalWatchdog(in), kWatchdogOk);
  in.since_start_sec = 51.0;
  EXPECT_EQ(evaluateGlobalWatchdog(in), kWatchdogNoGrid);
  // grid 있음, 0.5 s 전 update → 정상
  in.has_grid = true; in.any_rx = true; in.since_last_rx_sec = 0.5;
  EXPECT_EQ(evaluateGlobalWatchdog(in), kWatchdogOk);
  // 12 s 무갱신 → silent
  in.since_last_rx_sec = 12.0;
  EXPECT_EQ(evaluateGlobalWatchdog(in), kWatchdogSilent);
  // get_costmap 으로만 받고 topic 은 한 번도 → 시작부터 센다
  in.any_rx = false; in.since_start_sec = 60.0;
  EXPECT_EQ(evaluateGlobalWatchdog(in), kWatchdogSilent);
  // resync 연속 실패
  in.any_rx = true; in.since_last_rx_sec = 1.0; in.resync_consecutive_failures = 2;
  EXPECT_EQ(evaluateGlobalWatchdog(in), kWatchdogResyncFailing);
  in.since_last_rx_sec = 30.0;
  EXPECT_EQ(evaluateGlobalWatchdog(in), kWatchdogSilent | kWatchdogResyncFailing);
  // 끔
  in.stale_sec = 0.0;
  EXPECT_EQ(evaluateGlobalWatchdog(in), kWatchdogOk);
}

// ============================== 통합 흉내 ==============================
//   node 의 MirrorSync 가 하는 순서를 그대로 흉내 낸다 (setupMirrorSync → onUpdate/onTopicFull → resyncTick 500 ms →
//   응답 콜백). 현장 순서: Nav2 activation 뒤 PV 가 늦게 뜬다 → mask full 은 latch 돼 있지 않다 (구독자가 없었음).
//   global 은 G=False 라 update 가 먼저 오고 get_costmap 응답(16 s) 으로 grid 를 얻는다. 그동안 mask update 가 5 Hz 로 온다.
//   기대: mask get_costmap 요청 0 번, global 요청 1 번, global grid 가 생기는 순간 mask 가 seed 되고 그 뒤 mask 는 받은
//   update 마다 정확. mask 가 "모름" 인 시간 = global 응답까지 (예전: mask 응답까지 + 중복 요청).
TEST(MirrorSyncFlow, FieldOrderLateStartSeedsMaskWithoutMaskGetCostmap)
{
  const unsigned SX = 300, SY = 200;
  const double RES = 0.05;
  CostmapMirror global;
  CostmapMirror mask(scOpts());
  ResyncScheduler gsched, msched;
  bool mask_start_pending = true, mask_no_grid_update = false;
  int64_t now_ns = 100LL * 1000000000LL;   // Nav2 clock (activation 은 오래전)
  double t = 0.0;
  bool g_in_flight = false; double g_sent_at = 0.0;
  const double g_latency = 16.0;
  std::vector<uint8_t> gdata(static_cast<size_t>(SX) * SY, 0);
  for (size_t k = 0; k < gdata.size(); k += 101) {gdata[k] = 254;}
  std::mt19937 rng(9);
  nav2_msgs::msg::CostmapUpdate::ConstSharedPtr last_mask_update;
  double unknown_time = 0.0;
  int mask_bad_after_seed = 0;
  bool seeded = false;
  double seeded_at = -1.0;
  auto try_seed = [&]() {
      if (!mask.hasGrid() && global.hasGrid()) {
        const auto r = mask.seedZero(global.stats().sig);
        if (r.seeded) {seeded = true; seeded_at = t; mask_start_pending = false; msched.cancel();}
      }
    };
  // global: update 가 먼저 온다 → 시작 resync 트리거 (G=False)
  gsched.trigger();
  for (int step = 0; step < 400; ++step, t += 0.1, now_ns += 100000000LL) {
    // mask update 5 Hz (이웃 있음)
    if (step % 2 == 0) {
      const unsigned x = static_cast<unsigned>(rng() % (SX - 10)), y = static_cast<unsigned>(rng() % (SY - 10));
      auto u = makeUpdate(x, y, 8, 8, now_ns, 254);
      auto r = mask.applyUpdate(u);
      if (r == CostmapMirror::UpdateResult::kNoGrid) {
        try_seed();
        if (mask.hasGrid()) {r = mask.applyUpdate(u);} else {mask_no_grid_update = true;}
      }
      last_mask_update = u;
      if (mask.hasGrid() && grab(mask) != expectFrom(SX, SY, *u)) {++mask_bad_after_seed;}
    }
    // global update 2 Hz (grid 없으면 보관)
    if (step % 5 == 0) {
      global.applyUpdate(makeUpdate(0, 0, 4, 4, now_ns, 0));
    }
    // 500 ms timer
    if (step % 5 == 0) {
      try_seed();
      // mask fallback (grid 없음 + grace×4)
      if (!mask.hasGrid() && (mask_start_pending || mask_no_grid_update) && t >= 20.0) {
        mask_start_pending = false; mask_no_grid_update = false; msched.trigger();
      }
      const auto gt = gsched.tick(T(t), true, false);
      if (gt == Tick::kSend) {gsched.onSent(T(t)); g_in_flight = true; g_sent_at = t;}
      const bool blocked = gsched.inFlight() || gsched.needed();
      const auto mt = msched.tick(T(t), true, blocked);
      EXPECT_NE(mt, Tick::kSend) << "mask get_costmap 이 나가면 안 된다 (t=" << t << ")";
    }
    // global 응답
    if (g_in_flight && t - g_sent_at >= g_latency) {
      g_in_flight = false;
      gsched.onResponseArrived();
      auto f = makeFull(SX, SY, RES, -5.0, -3.0, now_ns - 15LL * 1000000000LL, gdata);   // 요청 직후 스냅샷
      const auto o = global.applyFull(f, 0);
      ASSERT_TRUE(o.applied());
      gsched.onResponseDefinitive();
      try_seed();   // onFullApplied → trySeedMaskFromGlobal
    }
    // mask 를 모르는 시간 (이웃 있음)
    const auto mode = replan_monitor::decideAgentMaskMode(true, mask.hasGrid(), mask.contentKnown(), true);
    if (mode == AgentMaskMode::kUnknown) {unknown_time += 0.1;}
  }
  std::printf("[FieldOrder] global requests=%lu mask requests=%lu seeded_at=%.1f s unknown=%.1f s "
              "mask_ignored_in_flight=%lu mask_bad_after_seed=%d\n",
    static_cast<unsigned long>(gsched.counters().requests), static_cast<unsigned long>(msched.counters().requests),
    seeded_at, unknown_time, static_cast<unsigned long>(msched.counters().ignored_in_flight), mask_bad_after_seed);
  EXPECT_TRUE(seeded);
  EXPECT_EQ(gsched.counters().requests, 1u);
  EXPECT_EQ(msched.counters().requests, 0u);
  EXPECT_EQ(mask_bad_after_seed, 0);
  EXPECT_NEAR(seeded_at, g_latency, 0.6);
  EXPECT_LE(unknown_time, g_latency + 0.6);
  EXPECT_TRUE(mask.contentKnown());
}

// PV respawn: 이전 instance 가 받던 마지막 update 가 latch 돼 있다 (그 뒤 PV 가 없던 5 s 동안 AgentLayer 는 update 를
//   만들지 않았다). 그사이 이웃이 떠나 더 보낼 update 가 없으면, 옛 update 를 적용한 mask 에는 유령 몸체가 영원히 남는다.
//   새 규칙(구독 직후 grace 보다 오래된 latched update 는 버림)이면 seed 한 0 grid 가 맞다.
TEST(MirrorSyncFlow, RespawnDropsOldLatchedUpdate)
{
  const unsigned SX = 100, SY = 80;
  auto latched = makeUpdate(40, 30, 10, 10, 50LL * 1000000000LL, 254);   // 7 s 전 것
  const double age = 7.0, grace = 5.0;
  for (bool use_rule : {true, false}) {
    CostmapMirror global, mask(scOpts());
    ASSERT_TRUE(global.applyFull(makeFull(SX, SY, 0.05, 0, 0, 1, std::vector<uint8_t>(SX * SY, 0))).applied());
    bool fresh_seen = false;
    const bool drop = use_rule && replan_monitor::dropLatchedMaskUpdate(true, fresh_seen, 0.3, 20.0, age, grace);
    if (!drop) {
      if (mask.applyUpdate(latched) == CostmapMirror::UpdateResult::kNoGrid) {
        ASSERT_TRUE(mask.seedZero(global.stats().sig).seeded);
      }
    } else {
      ASSERT_TRUE(mask.seedZero(global.stats().sig).seeded);
    }
    // 이웃은 떠났다: 더 오는 update 없음. 실제 mask = 0
    const auto g = grab(mask);
    const bool ghost = g != std::vector<uint8_t>(SX * SY, 0);
    std::printf("[Respawn] rule=%d ghost=%d content_known=%d\n", use_rule ? 1 : 0, ghost ? 1 : 0,
      mask.contentKnown() ? 1 : 0);
    EXPECT_EQ(ghost, !use_rule);   // 규칙 없음 = 음성 대조 (유령 몸체)
  }
}

// ============================== [10-10 W1-3][리뷰] 흐름 ==============================

// 리뷰 #2/#4: G=False global mirror. get_costmap 진행 중 (서버 스냅샷 Ts 뒤) 에 lethal 을 담은 update 가 MESSAGE_LOST 로
//   사라진다. 응답은 Ts 스냅샷이라 그 셀이 없고, replay 에도 없다. 새 규칙 (내용 유실은 기억 → 응답 뒤 한 번 더) 이면 두 번째
//   응답이 그 셀을 담는다. 음성 대조: 유실도 구조 트리거로 무시 (리뷰 전 W1-3).
TEST(MirrorSyncFlow, MessageLostDuringFlightHealsWithSecondRequest)
{
  using Trig = ResyncScheduler::Trigger;
  const unsigned SX = 120, SY = 100;
  const unsigned PX = 50, PY = 50;   // 팔레트 (5x5)
  for (bool new_rule : {true, false}) {
    CostmapMirror global;
    ResyncScheduler sched;
    std::vector<uint8_t> server(static_cast<size_t>(SX) * SY, 0);
    int64_t ns0 = 1000LL * 1000000000LL;
    auto ns = [&](double t) {return ns0 + static_cast<int64_t>(t * 1e9);};
    ASSERT_TRUE(global.applyFull(makeFull(SX, SY, 0.05, 0, 0, ns(0), server)).applied());
    bool in_flight = false; double snap_t = 0, resp_t = 0; std::vector<uint8_t> snap; uint64_t req_gen = 0;
    sched.trigger();   // 시작 resync (update 먼저 옴)
    for (double t = 0.1; t < 120.0; t += 0.1) {
      // 서버: t=12 에 팔레트를 놓는다 → update 유실
      if (std::abs(t - 12.0) < 0.05) {
        for (unsigned y = PY; y < PY + 5; ++y) {for (unsigned x = PX; x < PX + 5; ++x) {server[y * SX + x] = 254;}}
        sched.trigger(new_rule ? Trig::kContentLoss : Trig::kStructural);   // MESSAGE_LOST
      }
      // 2 Hz footprint clearing update (전달됨)
      if (static_cast<int>(std::lround(t * 10)) % 5 == 0) {
        global.applyUpdate(makeUpdate(0, 0, 4, 4, ns(t), 0));
      }
      if (static_cast<int>(std::lround(t * 10)) % 5 == 0) {
        const auto k = sched.tick(T(t), true, false);
        if (k == Tick::kSend) {
          sched.onSent(T(t)); in_flight = true; req_gen = global.fullGeneration();
          snap_t = t + 0.1; snap.clear(); resp_t = t + 16.0;
        }
      }
      if (in_flight && snap.empty() && t >= snap_t - 1e-9) {snap = server;}
      if (in_flight && t >= resp_t - 1e-9) {
        in_flight = false;
        sched.onResponseArrived();
        const auto o = global.applyFull(makeFull(SX, SY, 0.05, 0, 0, ns(snap_t), snap), req_gen);
        ASSERT_TRUE(o.applied());
        sched.onResponseDefinitive();
        if (o.needs_resync) {sched.trigger(Trig::kContentLoss);}
      }
    }
    const auto g = grab(global);
    const bool pallet_seen = g[(PY + 2) * SX + PX + 2] == 254;
    std::printf("[LostInFlight] rule=%d requests=%lu loss_retriggers=%lu pallet_in_mirror=%d\n", new_rule ? 1 : 0,
      static_cast<unsigned long>(sched.counters().requests),
      static_cast<unsigned long>(sched.counters().loss_retriggers), pallet_seen ? 1 : 0);
    if (new_rule) {
      EXPECT_TRUE(pallet_seen);
      EXPECT_EQ(sched.counters().requests, 2u);
      EXPECT_EQ(g, server);
    } else {
      EXPECT_FALSE(pallet_seen) << "음성 대조: 유실을 무시하면 팔레트가 mirror 에 없다";
      EXPECT_EQ(sched.counters().requests, 1u);
    }
  }
}

// 리뷰 #1: G=False, READY 에서 기다리는 중 PV respawn. 이웃이 다음 목표 위에 서 있다 (global 에 lethal). global grid 는
//   get_costmap 응답 (16 s) 뒤에야 생긴다. navigation_manager 흉내: static·agent 의 마지막 값이 1.0 s 이어서
//   (점유 아님 && is_status_ready) 이면 출발. 이전 instance 의 마지막 값은 SAFE (그때는 목표가 비어 있었다).
//   새 규칙: grid 가 없는 동안 보류 → 이웃이 떠난 (30 s) 뒤에 출발. 음성 대조 (보류 없음): 약 1 s 에 점유된 목표로 출발.
TEST(MirrorSyncFlow, ReadyHoldsUntilGlobalGrid)
{
  const unsigned SX = 100, SY = 80;
  const unsigned GX = 60, GY = 40;   // 목표 셀
  for (bool rule : {true, false}) {
    CostmapMirror global;
    std::vector<uint8_t> server(static_cast<size_t>(SX) * SY, 0);
    server[GY * SX + GX] = 254;   // 이웃이 목표 위에 서 있다
    struct Msg {bool occupied; bool ready;};
    Msg nav_static{false, true};   // 이전 instance 의 SAFE
    double cond_since = -1.0, departed_at = -1.0;
    int holds = 0;
    for (double t = 0.0; t < 60.0 && departed_at < 0.0; t += 0.1) {
      if (std::abs(t - 16.0) < 0.05) {   // global get_costmap 응답 (스냅샷 = t 0.5)
        std::vector<uint8_t> snap(server);
        ASSERT_TRUE(global.applyFull(makeFull(SX, SY, 0.05, 0, 0, 500000000LL, snap), 0).applied());
      }
      if (std::abs(t - 30.0) < 0.05) {   // 이웃이 떠난다 → update
        server[GY * SX + GX] = 0;
        global.applyUpdate(makeUpdate(GX, GY, 1, 1, 30LL * 1000000000LL, 0));
      }
      // PV (validation timer) — READY 분기
      if (replan_monitor::readyHoldWithoutGlobalGrid(rule, global.hasGrid())) {
        nav_static = {false, false};
        ++holds;
      } else {
        const auto g = global.read();
        const bool blocked = g && g->getCost(GX, GY) >= 253;   // isGoalBlocked (grid 없으면 false)
        nav_static = blocked ? Msg{true, false} : Msg{false, true};   // triggerReplan 은 is_status_ready 를 채우지 않는다
      }
      // navigation_manager READY 루프 (agent 쪽은 non_collision + ready 라 생략)
      const bool cond = !nav_static.occupied && nav_static.ready;
      if (cond) {
        if (cond_since < 0.0) {cond_since = t;} else if (t - cond_since >= 1.0) {departed_at = t;}
      } else {
        cond_since = -1.0;
      }
    }
    std::printf("[ReadyNoGrid] rule=%d departed_at=%.1f s holds=%d\n", rule ? 1 : 0, departed_at, holds);
    if (rule) {
      EXPECT_GE(departed_at, 31.0 - 0.05) << "이웃이 떠나기 전에 출발하면 안 된다";
      EXPECT_LE(departed_at, 31.0 + 0.25);
      EXPECT_GT(holds, 150);
    } else {
      EXPECT_GE(departed_at, 0.0);
      EXPECT_LT(departed_at, 16.0) << "음성 대조: grid 없는 동안 점유된 목표로 출발";
    }
  }
}

// 리뷰 #5: 빠른 respawn — latched update 의 나이가 start_grace (5 s) 안쪽 (4.5 s) 이다. 나이 규칙만 있으면 적용해 유령이
//   영구히 남는다 (이웃은 떠났고 더 오는 update 가 없다). 출처 규칙 (구독보다 먼저 찍힘) 이면 버린다.
TEST(MirrorSyncFlow, FastRespawnLatchedUpdateDroppedByOrigin)
{
  const unsigned SX = 100, SY = 80;
  const int64_t sub_ns = 60LL * 1000000000LL;                      // 새 instance 구독 시각
  auto latched = makeUpdate(40, 30, 10, 10, sub_ns - 4500000000LL + 0, 254);   // 죽기 직전 것 (나이 4.5 s)
  const double age = 4.5, grace = 5.0;
  for (bool origin_rule : {true, false}) {
    CostmapMirror global, mask(scOpts());
    ASSERT_TRUE(global.applyFull(makeFull(SX, SY, 0.05, 0, 0, 1, std::vector<uint8_t>(SX * SY, 0))).applied());
    const int64_t stamp_ns = sub_ns - 4500000000LL;
    const bool before_sub = origin_rule && stamp_ns < sub_ns - 200000000LL;
    const bool drop = replan_monitor::dropLatchedMaskUpdate(true, false, 0.3, 20.0, age, grace, before_sub);
    ASSERT_TRUE(mask.seedZero(global.stats().sig).seeded);
    if (!drop) {mask.applyUpdate(latched);}
    const bool ghost = grab(mask) != std::vector<uint8_t>(SX * SY, 0);
    std::printf("[FastRespawn] origin_rule=%d dropped=%d ghost=%d\n", origin_rule ? 1 : 0, drop ? 1 : 0, ghost ? 1 : 0);
    EXPECT_EQ(ghost, !origin_rule);
  }
}

TEST(ReadyHold, DecisionTable)
{
  EXPECT_TRUE(replan_monitor::readyHoldWithoutGlobalGrid(true, false));
  EXPECT_FALSE(replan_monitor::readyHoldWithoutGlobalGrid(true, true));
  EXPECT_FALSE(replan_monitor::readyHoldWithoutGlobalGrid(false, false));   // param 으로 끄면 예전 (SAFE)
  EXPECT_FALSE(replan_monitor::readyHoldWithoutGlobalGrid(false, true));
}

// ============================== [10-10 W1-4] global source 선택 ==============================

using replan_monitor::GlobalSourceSelector;
using SrcAction = GlobalSourceSelector::Action;

// diff layer 가 없는 설정: Nav2 가 t=8 에 뜨고 (stock publisher 생김) diff publisher 는 끝까지 없다 → t=8+15 에 fallback 한 번.
//   Nav2 가 뜨기 전 (둘 다 0) 은 세지 않는다. 조건이 한 번 끊기면 (stock 이 잠깐 사라짐) 처음부터 다시 센다.
TEST(GlobalSourceSelector, FallbackOnlyAfterContinuousCondition)
{
  GlobalSourceSelector sel;   // fallback_sec 15, confirm 1
  double switched_at = -1.0;
  for (double t = 0.0; t < 60.0; t += 0.5) {
    const int stock = (t >= 8.0 && !(t >= 12.0 && t < 12.6)) ? 1 : 0;   // t=12.0, 12.5 tick 에 끊김 → 13.0 에 다시 보임
    const auto a = sel.tick(T(t), 0, stock);
    if (a == SrcAction::kSwitchToFallback) {
      EXPECT_LT(switched_at, 0.0) << "fallback 은 한 번뿐";
      switched_at = t;
    }
  }
  std::printf("[SourceFallback] switched_at=%.1f s (stock 재등장 13.0 s + 15)\n", switched_at);
  EXPECT_NEAR(switched_at, 13.0 + 15.0, 0.01);
  EXPECT_EQ(sel.active(), GlobalSourceSelector::Source::kFallback);
  EXPECT_TRUE(sel.fallenBack());
}

// fallback 중에 diff publisher 가 생기면 confirm (1 s) 뒤 한 번 돌아가고, 그 뒤 diff 가 다시 사라져도 (stock 은 있음) 바꾸지 않는다.
//   0.5 s 만 보였다 사라지는 것은 복귀하지 않는다.
TEST(GlobalSourceSelector, SwitchBackOnceThenNeverAgain)
{
  GlobalSourceSelector sel;
  int fallbacks = 0, backs = 0;
  double back_at = -1.0;
  for (double t = 0.0; t < 200.0; t += 0.5) {
    int diff = 0;
    if (t >= 30.0 && t < 30.5) {diff = 1;}            // 0.5 s 깜빡임 → 복귀 안 함
    if (t >= 40.0 && t < 100.0) {diff = 1;}           // 복귀
    const auto a = sel.tick(T(t), diff, 1);
    if (a == SrcAction::kSwitchToFallback) {++fallbacks;}
    if (a == SrcAction::kSwitchToPrimary) {++backs; back_at = t;}
  }
  std::printf("[SourceSwitchBack] fallbacks=%d backs=%d back_at=%.1f s\n", fallbacks, backs, back_at);
  EXPECT_EQ(fallbacks, 1);
  EXPECT_EQ(backs, 1);
  EXPECT_NEAR(back_at, 41.0, 0.01);
  EXPECT_EQ(sel.active(), GlobalSourceSelector::Source::kPrimary);   // t=100 뒤 diff 가 사라져도 그대로
  EXPECT_TRUE(sel.switchedBack());
}

// planner_server 재기동: diff·stock 이 같이 사라졌다 (30 s) 같이 돌아온다. stock 이 0.4 s 먼저 보여도 fallback 하지 않는다.
//   음성 대조: "diff 없음" 만 세면 (재기동 동안 쌓인 시간으로) stock 이 보이는 순간 fallback 한다.
TEST(GlobalSourceSelector, PlannerRestartDoesNotFallBack)
{
  GlobalSourceSelector sel;
  int fallbacks = 0;
  double diff_absent_since = -1.0;
  int naive_fallbacks = 0;
  for (double t = 0.0; t < 120.0; t += 0.1) {
    const bool down = t >= 50.0 && t < 80.0;
    const int stock = down ? 0 : 1;
    const int diff = (down || (t >= 80.0 && t < 80.4)) ? 0 : 1;
    if (sel.tick(T(t), diff, stock) == SrcAction::kSwitchToFallback) {++fallbacks;}
    // 음성 대조
    if (diff == 0) {
      if (diff_absent_since < 0.0) {diff_absent_since = t;}
      if (stock > 0 && t - diff_absent_since >= 15.0) {++naive_fallbacks; diff_absent_since = 1e9;}
    } else {
      diff_absent_since = -1.0;
    }
  }
  std::printf("[SourceRestart] fallbacks=%d naive_fallbacks=%d\n", fallbacks, naive_fallbacks);
  EXPECT_EQ(fallbacks, 0);
  EXPECT_EQ(naive_fallbacks, 1);
  EXPECT_EQ(sel.active(), GlobalSourceSelector::Source::kPrimary);
}

TEST(GlobalSourceSelector, DisabledConfigs)
{
  GlobalSourceSelector::Config c;
  c.fallback_sec = 0.0;
  GlobalSourceSelector off1(c);
  c.fallback_sec = 15.0;
  c.enabled = false;   // fallback topic 비움 / primary 와 같음
  GlobalSourceSelector off2(c);
  for (double t = 0.0; t < 100.0; t += 0.5) {
    EXPECT_EQ(off1.tick(T(t), 0, 1), SrcAction::kNone);
    EXPECT_EQ(off2.tick(T(t), 0, 1), SrcAction::kNone);
  }
  EXPECT_FALSE(off1.enabled());
  EXPECT_FALSE(off2.enabled());
}

// watchdog: diff source (liveness) 는 몇 분 조용해도 정상, publisher 가 사라지면 stale_sec 뒤 kWatchdogNoPublisher.
//   본 적 없는 publisher 는 source 시작 뒤 no_grid_sec 까지 기다린다 (Nav2 가 늦게 뜸·fallback 판정 15 s 보다 길게).
//   resync 연속 실패는 두 기준 모두에서 본다. 음성 대조: 같은 침묵을 stock 기준 (무갱신) 으로 보면 kWatchdogSilent.
TEST(GlobalWatchdog, LivenessForDiffSourceSilenceForStock)
{
  using namespace replan_monitor;
  GlobalWatchdogInput in;
  in.stale_sec = 10.0; in.no_grid_sec = 50.0; in.fail_threshold = 2;
  in.has_grid = true; in.any_rx = true; in.since_last_rx_sec = 600.0;   // 10 분 동안 값이 안 바뀜
  // [10-10 W1-4][리뷰] 여기서는 heartbeat_sec 기본값 0 (끔) — heartbeat 는 DiffSourceHeartbeatFlagsSilentPublisher 에서 본다
  in.since_start_sec = 900.0; in.since_source_sec = 900.0;
  in.liveness_mode = true; in.publisher_present = true; in.publisher_ever_seen = true; in.since_publisher_seen_sec = 0.0;
  EXPECT_EQ(evaluateGlobalWatchdog(in), kWatchdogOk);
  // 음성 대조: 무갱신 기준이면 ERROR
  in.liveness_mode = false;
  EXPECT_EQ(evaluateGlobalWatchdog(in), kWatchdogSilent);
  in.liveness_mode = true;
  // publisher 사라짐: 9 s 는 아직, 10 s 부터
  in.publisher_present = false; in.since_publisher_seen_sec = 9.0;
  EXPECT_EQ(evaluateGlobalWatchdog(in), kWatchdogOk);
  in.since_publisher_seen_sec = 10.0;
  EXPECT_EQ(evaluateGlobalWatchdog(in), kWatchdogNoPublisher);
  // + resync 연속 실패
  in.resync_consecutive_failures = 2;
  EXPECT_EQ(evaluateGlobalWatchdog(in), kWatchdogNoPublisher | kWatchdogResyncFailing);
  in.publisher_present = true; in.since_publisher_seen_sec = 0.0;
  EXPECT_EQ(evaluateGlobalWatchdog(in), kWatchdogResyncFailing);
  in.resync_consecutive_failures = 0;
  // 시작 직후 publisher 를 본 적 없음 (Nav2 아직): 20 s 는 정상, 50 s 부터 (grid 도 없으면 NoGrid 도)
  in.has_grid = false; in.any_rx = false; in.publisher_present = false; in.publisher_ever_seen = false;
  in.since_start_sec = 20.0; in.since_source_sec = 20.0;
  EXPECT_EQ(evaluateGlobalWatchdog(in), kWatchdogOk);
  in.since_start_sec = 51.0; in.since_source_sec = 51.0;
  EXPECT_EQ(evaluateGlobalWatchdog(in), kWatchdogNoGrid | kWatchdogNoPublisher);
  // stock fallback 으로 바꾼 직후 (grid 있음, 아직 수신 없음): source 시작부터 무갱신을 센다 (since_start 가 아니라)
  in.liveness_mode = false; in.has_grid = true; in.any_rx = false;
  in.since_start_sec = 300.0; in.since_source_sec = 3.0;
  EXPECT_EQ(evaluateGlobalWatchdog(in), kWatchdogOk);
  in.since_source_sec = 10.5;
  EXPECT_EQ(evaluateGlobalWatchdog(in), kWatchdogSilent);
  // 기존 호출 (since_source_sec 안 줌 = -1) 은 W1-3 그대로
  GlobalWatchdogInput old;
  old.has_grid = true; old.any_rx = false; old.since_start_sec = 60.0;
  EXPECT_EQ(evaluateGlobalWatchdog(old), kWatchdogSilent);
  // 끔
  in.stale_sec = 0.0;
  EXPECT_EQ(evaluateGlobalWatchdog(in), kWatchdogOk);
}

TEST(ResyncScheduler, ResetForNewSourceDropsInFlightAndBackoff)
{
  ResyncScheduler s;
  s.trigger();
  ASSERT_EQ(s.tick(T(0), true, false), Tick::kSend);
  s.onSent(T(0));
  s.onResponseArrived();
  s.onResponseFailed(T(1));                  // backoff 30 s
  ASSERT_EQ(s.tick(T(2), true, false), Tick::kWaitBackoff);
  // backoff 뒤 다시 보내고 진행 중 + 내용 유실 표시
  ASSERT_EQ(s.tick(T(32), true, false), Tick::kSend);
  s.onSent(T(32));
  s.trigger(ResyncScheduler::Trigger::kContentLoss);
  ASSERT_TRUE(s.inFlight());
  ASSERT_TRUE(s.dirtyDuringFlight());
  // 전환
  EXPECT_TRUE(s.resetForNewSource());
  EXPECT_FALSE(s.inFlight());
  EXPECT_FALSE(s.dirtyDuringFlight());
  EXPECT_EQ(s.consecutiveFailures(), 0);
  EXPECT_TRUE(s.needed());
  // 바로 보낸다 (min_interval·backoff 없음). service 가 아직이면 기다림
  EXPECT_EQ(s.tick(T(33), false, false), Tick::kWaitService);
  EXPECT_EQ(s.tick(T(33.5), true, false), Tick::kSend);
  s.onSent(T(33.5));
  s.onResponseArrived();
  s.onResponseDefinitive();
  EXPECT_FALSE(s.needed());
  EXPECT_EQ(s.counters().source_resets, 1u);
  EXPECT_EQ(s.counters().dropped_in_flight_on_reset, 1u);
  EXPECT_EQ(s.counters().requests, 3u);
  // 진행 중 요청이 없을 때의 reset
  EXPECT_FALSE(s.resetForNewSource());
  EXPECT_EQ(s.counters().dropped_in_flight_on_reset, 1u);
}

// source 전환 흐름 (node 순서 흉내): stock fallback 중 get_costmap (MESSAGE_LOST 때문) 이 진행 중일 때 diff publisher 가 생겨
//   복귀한다. 새 규칙: 옛 요청 세대를 버리고 (scheduler reset + 세대 검사) diff service 로 resync 1 번, diff 쪽 동시 요청 최대 1.
//   음성 대조 (세대 검사 없음 — scheduler reset·mirror rebase 는 함): 늦게 온 옛 stock 응답이 diff 요청의 "진행 중" 을 지워
//   그 사이 트리거 (t=18, 범위 밖 update) 로 diff 에 두 번째 200 MB 요청이 겹쳐 나간다.
TEST(MirrorSyncFlow, SourceSwitchDropsOldInFlightResponse)
{
  using replan_monitor::isCurrentSourceGen;
  const unsigned SX = 60, SY = 40;
  auto ns = [](double t) {return 1000LL * 1000000000LL + static_cast<int64_t>(t * 1e9);};
  for (bool gen_rule : {true, false}) {
    CostmapMirror global;
    ResyncScheduler sched;
    uint64_t cur_gen = 0;
    std::vector<uint8_t> srv(static_cast<size_t>(SX) * SY, 0);
    srv[5 * SX + 5] = 254;
    struct Req {uint64_t gen; bool diff; double resp_t; int64_t stamp; std::vector<uint8_t> snap; uint64_t full_gen; bool done;};
    std::vector<Req> reqs;
    int diff_outstanding = 0, max_diff_outstanding = 0, diff_requests = 0, stale_dropped = 0, responses_seen = 0;
    bool on_diff = false;
    // stock (G=True) full t=0
    ASSERT_TRUE(global.applyFull(makeFull(SX, SY, 0.05, 0, 0, ns(0), srv)).applied());
    // t=1 MESSAGE_LOST → stock get_costmap (16 s)
    sched.trigger(ResyncScheduler::Trigger::kContentLoss);
    for (double t = 0.5; t < 60.0; t += 0.5) {
      // 서버 변화: t=3 (A), t=10 (C) — 지금 source 의 update 로 온다
      if (std::abs(t - 3.0) < 0.01) {srv[10 * SX + 10] = 254; global.applyUpdate(makeUpdate(10, 10, 1, 1, ns(t), 254));}
      if (std::abs(t - 10.0) < 0.01) {srv[20 * SX + 20] = 254; global.applyUpdate(makeUpdate(20, 20, 1, 1, ns(t), 254));}
      // t=6 diff 복귀 (selector confirm 뒤) — switchGlobalSource 순서
      if (std::abs(t - 6.0) < 0.01) {
        ++cur_gen;
        sched.resetForNewSource();
        global.beginNewSource();
        on_diff = true;
      }
      // diff latched FULL (t=6.5, activation 때 stamp — 옛 grid 상태) + 새 구독자 FULL (t=7.5, 지금 상태)
      if (std::abs(t - 6.5) < 0.01) {
        std::vector<uint8_t> act(static_cast<size_t>(SX) * SY, 0);
        act[5 * SX + 5] = 254; act[30 * SX + 30] = 254;   // 그때 있던 유령 (나중에 지워짐 — durable update 로 옴)
        global.applyFull(makeFull(SX, SY, 0.05, 0, 0, ns(-50), act));
        global.applyUpdate(makeUpdate(30, 30, 1, 1, ns(-40), 0));
        global.applyUpdate(makeUpdate(10, 10, 1, 1, ns(3), 254));
      }
      if (std::abs(t - 7.5) < 0.01) {global.applyFull(makeFull(SX, SY, 0.05, 0, 0, ns(7.4), srv));}
      // t=18 범위 밖 update (구조 트리거)
      if (std::abs(t - 18.0) < 0.01) {sched.trigger();}
      // 500 ms timer → resyncTick
      const auto k = sched.tick(T(t), true, false);
      if (k == Tick::kSend) {
        sched.onSent(T(t));
        Req r{cur_gen, on_diff, t + 16.0, ns(t + 0.1), srv, global.fullGeneration(), false};
        reqs.push_back(r);
        if (on_diff) {++diff_requests; ++diff_outstanding; max_diff_outstanding = std::max(max_diff_outstanding, diff_outstanding);}
      }
      // 응답 도착 (응답 콜백)
      for (auto & r : reqs) {
        if (r.done || t < r.resp_t - 1e-9) {continue;}
        r.done = true;
        if (r.diff) {--diff_outstanding;}
        if (gen_rule && !isCurrentSourceGen(r.gen, cur_gen)) {++stale_dropped; continue;}
        ++responses_seen;
        sched.onResponseArrived();
        const auto o = global.applyFull(makeFull(SX, SY, 0.05, 0, 0, r.stamp, r.snap), r.full_gen);
        sched.onResponseDefinitive();
        if (o.needs_resync) {sched.trigger(ResyncScheduler::Trigger::kContentLoss);}
      }
    }
    std::printf("[SourceSwitch] gen_rule=%d diff_requests=%d max_diff_outstanding=%d stale_dropped=%d responses=%d "
                "sched_requests=%lu resets=%lu dropped_in_flight=%lu mirror_ok=%d\n", gen_rule ? 1 : 0, diff_requests,
      max_diff_outstanding, stale_dropped, responses_seen, static_cast<unsigned long>(sched.counters().requests),
      static_cast<unsigned long>(sched.counters().source_resets),
      static_cast<unsigned long>(sched.counters().dropped_in_flight_on_reset), grab(global) == srv ? 1 : 0);
    EXPECT_EQ(grab(global), srv);   // 내용은 두 경우 모두 맞는다 (replay·stale 규칙)
    if (gen_rule) {
      EXPECT_EQ(stale_dropped, 1);
      EXPECT_EQ(diff_requests, 1);
      EXPECT_EQ(max_diff_outstanding, 1);
      EXPECT_EQ(sched.counters().dropped_in_flight_on_reset, 1u);
    } else {
      EXPECT_EQ(max_diff_outstanding, 2) << "음성 대조: 옛 응답이 진행 중 표시를 지워 diff 요청이 겹친다";
    }
  }
}

// ============================== [10-10 W1-4][리뷰] ==============================
// 빈 get_costmap 응답 (서버에 아직 데이터 없음) 은 실패가 아니다: 연속 실패·backoff (30 s) 없이 needed 를 남겨 min_interval (5 s)
//   뒤에 다시 묻는다. 음성 대조: 실패로 처리하면 backoff 30 s, 결론 (definitive) 으로 처리하면 needed 가 내려가 다시 묻지 않는다.
TEST(ResyncScheduler, NotReadyResponseRetriesAfterMinIntervalNoBackoff)
{
  for (int mode : {0, 1, 2}) {   // 0 = onResponseNotReady (새 규칙), 1 = onResponseFailed, 2 = onResponseDefinitive (예전 처리)
    ResyncScheduler s;
    s.trigger();
    ASSERT_EQ(s.tick(T(0), true, false), Tick::kSend);
    s.onSent(T(0));
    s.onResponseArrived();
    if (mode == 0) {s.onResponseNotReady();} else if (mode == 1) {s.onResponseFailed(T(0.1));} else {s.onResponseDefinitive();}
    double next_send = -1.0;
    for (double t = 0.5; t <= 60.0; t += 0.5) {
      if (s.tick(T(t), true, false) == Tick::kSend) {next_send = t; break;}
    }
    std::printf("[NotReady] mode=%d needed=%d failures=%d next_send=%.1f not_ready=%lu\n", mode, s.needed() ? 1 : 0,
      s.consecutiveFailures(), next_send, static_cast<unsigned long>(s.counters().not_ready));
    if (mode == 0) {
      EXPECT_TRUE(s.needed());
      EXPECT_EQ(s.consecutiveFailures(), 0);
      EXPECT_EQ(s.counters().failures, 0u);
      EXPECT_EQ(s.counters().not_ready, 1u);
      EXPECT_DOUBLE_EQ(next_send, 5.0);              // min_interval
    } else if (mode == 1) {
      EXPECT_EQ(s.consecutiveFailures(), 1);
      EXPECT_GE(next_send, 30.0);                    // backoff
    } else {
      EXPECT_FALSE(s.needed());
      EXPECT_LT(next_send, 0.0);                     // 다시 묻지 않는다 (빈 응답을 resync 완료로 봄)
    }
  }
}

// diff source 의 heartbeat: layer 의 publisher thread 는 update 구독자가 있으면 1 s 마다 sweep band 를 보낸다. publisher 는 graph 에
//   있는데 아무것도 오지 않으면 (F6: FULL 버퍼 할당 실패로 FULL token 이 큐 맨 앞에 멈춤) heartbeat_sec 뒤 무갱신.
//   음성 대조: heartbeat_sec 0 (= 리뷰 전 규칙) 은 10 분 침묵도 OK.
TEST(GlobalWatchdog, DiffSourceHeartbeatFlagsSilentPublisher)
{
  using namespace replan_monitor;
  GlobalWatchdogInput in;
  in.stale_sec = 10.0; in.no_grid_sec = 50.0; in.fail_threshold = 2;
  in.has_grid = true; in.any_rx = true; in.since_start_sec = 900.0; in.since_source_sec = 900.0;
  in.liveness_mode = true; in.publisher_present = true; in.publisher_ever_seen = true; in.since_publisher_seen_sec = 0.0;
  in.heartbeat_sec = 20.0;
  in.since_last_rx_sec = 1.0;    // 정상: sweep 1 s
  EXPECT_EQ(evaluateGlobalWatchdog(in), kWatchdogOk);
  in.since_last_rx_sec = 12.0;   // FULL ack 대기 (≤ 10 s) + 여유 안
  EXPECT_EQ(evaluateGlobalWatchdog(in), kWatchdogOk);
  in.since_last_rx_sec = 20.0;
  EXPECT_EQ(evaluateGlobalWatchdog(in), kWatchdogSilent);
  in.since_last_rx_sec = 600.0;
  EXPECT_EQ(evaluateGlobalWatchdog(in), kWatchdogSilent);
  // publisher 도 사라지면 둘 다
  in.publisher_present = false; in.since_publisher_seen_sec = 10.0;
  EXPECT_EQ(evaluateGlobalWatchdog(in), kWatchdogSilent | kWatchdogNoPublisher);
  in.publisher_present = true; in.since_publisher_seen_sec = 0.0;
  // 이 source 에서 아직 아무것도 못 받음 (grid 는 옛 source 것): source 시작부터 센다
  in.any_rx = false; in.since_source_sec = 19.0;
  EXPECT_EQ(evaluateGlobalWatchdog(in), kWatchdogOk);
  in.since_source_sec = 21.0;
  EXPECT_EQ(evaluateGlobalWatchdog(in), kWatchdogSilent);
  // 음성 대조 / 끔: heartbeat 0 = 리뷰 전 규칙 (침묵을 보지 않음)
  in.any_rx = true; in.since_last_rx_sec = 600.0; in.heartbeat_sec = 0.0;
  const unsigned off = evaluateGlobalWatchdog(in);
  EXPECT_EQ(off, kWatchdogOk);
  // stock source 는 heartbeat 와 무관하게 stale_sec (10 s)
  in.liveness_mode = false; in.heartbeat_sec = 20.0; in.since_last_rx_sec = 10.0;
  EXPECT_EQ(evaluateGlobalWatchdog(in), kWatchdogSilent);
  in.since_last_rx_sec = 9.0;
  EXPECT_EQ(evaluateGlobalWatchdog(in), kWatchdogOk);
  std::printf("[Heartbeat] diff 20 s 침묵 → 0x%x, heartbeat 끔 (리뷰 전) → 0x%x\n", kWatchdogSilent, off);
}

// 복귀 (stock → diff) 직후 Nav2 configure~activate 사이: diff layer 의 service 는 있지만 첫 cycle 전이라 빈 응답.
//   node 응답 처리 (path_validator_node.cpp get_costmap 응답 콜백) 를 그대로 흉내 낸다. 새 규칙: 빈 응답 → onResponseNotReady,
//   grid 는 옛 stock grid 그대로 (0x0 이 되지 않음), activation 뒤 topic FULL 이 기준이 되고 다음 요청이 진짜 snapshot 을 받는다.
//   음성 대조 (rule=0, 리뷰 전 응답 처리): 빈 응답을 결론으로 봐 needed 를 내림 → activation 뒤 get_costmap 0 번.
TEST(MirrorSyncFlow, SwitchBackEmptyResponseUntilLayerActive)
{
  const unsigned SX = 30, SY = 20;
  auto ns = [](double t) {return 2000LL * 1000000000LL + static_cast<int64_t>(t * 1e9);};
  const double active_at = 8.0;   // diff layer 첫 cycle (planner_server activate)
  for (int rule : {1, 0}) {
    CostmapMirror global;
    ResyncScheduler sched;
    std::vector<uint8_t> stock(static_cast<size_t>(SX) * SY, 0);
    stock[3 * SX + 3] = 254;
    ASSERT_TRUE(global.applyFull(makeFull(SX, SY, 0.05, 0, 0, ns(-5), stock)).applied());
    // t=0 복귀 (switchGlobalSource)
    global.beginNewSource();
    sched.resetForNewSource();
    std::vector<uint8_t> diff(stock);
    diff[7 * SX + 7] = 254;   // diff layer 가 activation 때 보는 지금 상태
    int empty_resp = 0, real_resp = 0, requests = 0, min_cells = SX * SY;
    double rebased_at = -1.0;
    for (double t = 0.5; t <= 40.0; t += 0.5) {
      if (std::abs(t - (active_at + 0.5)) < 1e-9) {   // activation 뒤 새 구독자 FULL (topic)
        const auto o = global.applyFull(makeFull(SX, SY, 0.05, 0, 0, ns(active_at), diff));
        if (o.applied() && rebased_at < 0) {rebased_at = t;}
      }
      if (sched.tick(T(t), true, false) == Tick::kSend) {
        sched.onSent(T(t));
        ++requests;
        const uint64_t req_full_gen = global.fullGeneration();
        // 응답은 0.5 s 뒤 (작은 지도). 첫 cycle 전이면 layer 는 빈 Costmap() 을 돌려준다.
        const bool ready = (t + 0.5) >= active_at;
        nav2_msgs::msg::Costmap resp = ready ? makeFull(SX, SY, 0.05, 0, 0, ns(t + 0.5), diff) : nav2_msgs::msg::Costmap();
        sched.onResponseArrived();
        const auto o = global.applyFull(resp, req_full_gen);
        if (o.result == CostmapMirror::FullResult::kRejectedEmpty) {
          ++empty_resp;
          if (rule) {sched.onResponseNotReady();} else {sched.onResponseDefinitive();}
        } else {
          if (o.applied()) {++real_resp; if (rebased_at < 0) {rebased_at = t;}}
          sched.onResponseDefinitive();
        }
      }
      auto g = global.read();
      ASSERT_TRUE(static_cast<bool>(g));
      min_cells = std::min<int>(min_cells, static_cast<int>(g->getSizeInCellsX() * g->getSizeInCellsY()));
    }
    std::printf("[SwitchBackEmpty] rule=%d requests=%d empty=%d real_resp=%d rebased_at=%.1f min_cells=%d baseline_pending=%d\n",
      rule, requests, empty_resp, real_resp, rebased_at, min_cells, global.baselinePending() ? 1 : 0);
    EXPECT_EQ(min_cells, static_cast<int>(SX * SY));   // 0x0 grid 가 된 적 없음 (mirror 규칙 — 두 경우 모두)
    EXPECT_FALSE(global.baselinePending());
    EXPECT_EQ(grab(global), diff);
    if (rule) {
      EXPECT_EQ(empty_resp, 2);        // t=0.5, 5.5 (min_interval 5 s, backoff 없음)
      EXPECT_EQ(real_resp, 1);         // t=10.5 — activation 뒤 진짜 snapshot 으로 resync 완료
      EXPECT_FALSE(sched.needed());
      EXPECT_EQ(sched.consecutiveFailures(), 0);
    } else {
      EXPECT_EQ(empty_resp, 1);
      EXPECT_EQ(real_resp, 0);         // 빈 응답을 resync 완료로 봐 다시 묻지 않았다
    }
  }
}
