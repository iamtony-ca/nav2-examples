// [10-10 W1-D] DiffPublisherLayer ↔ 실제 nav2_costmap_2d::CostmapSubscriber 연결 확인.
//   격리: ROS_DOMAIN_ID=229 + localhost discovery (돌고 있는 sim 들과 섞이지 않게), 노드 이름공간도 시험 전용.
//   확인: 토픽 이름(+"_updates"), QoS 호환, full + update 병합 결과 == master, get_costmap 응답, 늦게 붙은 구독자.
#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "ammr_costmap_diff_publisher/diff_publisher_layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace
{
const char * kNs = "w1d_test_global_costmap";

bool sameAsMaster(nav2_costmap_2d::CostmapSubscriber & sub, nav2_costmap_2d::Costmap2D & master)
{
  // getCostmap() 은 아직 full 이 없으면 throw 한다 (isCostmapReceived 는 protected)
  std::shared_ptr<nav2_costmap_2d::Costmap2D> cm;
  try {
    cm = sub.getCostmap();
  } catch (...) {
    return false;
  }
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> l1(*cm->getMutex());
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> l2(*master.getMutex());
  return cm->getSizeInCellsX() == master.getSizeInCellsX() &&
         cm->getSizeInCellsY() == master.getSizeInCellsY() &&
         std::memcmp(cm->getCharMap(), master.getCharMap(),
           size_t(master.getSizeInCellsX()) * master.getSizeInCellsY()) == 0;
}

template<typename Pred>
bool waitFor(Pred p, std::chrono::milliseconds timeout)
{
  const auto end = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < end) {
    if (p()) {
      return true;
    }
    std::this_thread::sleep_for(20ms);
  }
  return p();
}
}  // namespace

TEST(DiffPublisherLayerRos, CostmapSubscriberMirrorsMaster)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides(
  {
    rclcpp::Parameter("diff_publisher.max_rects", 8),
    rclcpp::Parameter("diff_publisher.tile_size", 16),
    rclcpp::Parameter("diff_publisher.log_stats_period_sec", 1.0),
    rclcpp::Parameter("diff_publisher.catchup_delays_sec", std::vector<double>{0.5, 2.0}),
  });
  auto node = std::make_shared<nav2_util::LifecycleNode>("global_costmap", kNs, opts);

  auto lc = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, false);
  lc->resizeMap(300, 200, 0.05, -1.0, -2.0);
  auto * master = lc->getCostmap();
  auto layer = std::make_shared<ammr_costmap_diff_publisher::DiffPublisherLayer>();
  lc->addPlugin(layer);
  auto cbg = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  layer->initialize(lc.get(), "diff_publisher", nullptr, node, cbg);
  layer->activate();

  std::mt19937 rng(5);
  auto cycle = [&](int x0, int y0, int x1, int y1, int edits) {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*master->getMutex());
      for (int k = 0; k < edits; ++k) {
        master->setCost(x0 + rng() % (x1 - x0), y0 + rng() % (y1 - y0),
          static_cast<unsigned char>(rng() % 256));
      }
      layer->updateCosts(*master, x0, y0, x1, y1);   // LayeredCostmap::updateMap 처럼 master lock 아래
    };

  // 첫 cycle (FULL)
  cycle(0, 0, 300, 200, 500);

  // 구독자 (smoother_server 와 같은 CostmapSubscriber)
  auto sub_node = std::make_shared<rclcpp::Node>("smoother_like", kNs);
  const std::string full_topic = std::string("/") + kNs + "/diff_publisher/costmap_raw";
  auto sub = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
    rclcpp::Node::WeakPtr(sub_node), full_topic);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(sub_node);
  std::atomic<bool> run{true};
  std::thread spin([&] {
      while (run && rclcpp::ok()) {
        exec.spin_some(10ms);
        std::this_thread::sleep_for(2ms);
      }
    });

  EXPECT_TRUE(waitFor([&] {return sameAsMaster(*sub, *master);}, 10s)) << "full 미수신/불일치";

  // 토픽 이름 확인
  auto topics = node->get_topic_names_and_types();
  EXPECT_TRUE(topics.count(full_topic)) << full_topic;
  EXPECT_TRUE(topics.count(full_topic + "_updates")) << full_topic + "_updates";
  EXPECT_EQ(layer->fullTopic(), "diff_publisher/costmap_raw");

  // update 여러 cycle
  for (int c = 0; c < 40; ++c) {
    const int x0 = rng() % 250, y0 = rng() % 150;
    cycle(x0, y0, x0 + 1 + rng() % 50, y0 + 1 + rng() % 50, 1 + rng() % 60);
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(waitFor([&] {return sameAsMaster(*sub, *master);}, 10s)) << "update 병합 불일치";

  // clear (reset) → 다음 cycle 은 작은 bounds 라도 지도 전체 비교
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*master->getMutex());
    master->resetMap(0, 0, 300, 200);
    layer->reset();
  }
  cycle(10, 10, 20, 20, 5);
  EXPECT_TRUE(waitFor([&] {return sameAsMaster(*sub, *master);}, 10s)) << "clear 뒤 불일치";

  // get_costmap 서비스 (전용 executor 에서 응답)
  auto client = sub_node->create_client<nav2_msgs::srv::GetCostmap>(
    std::string("/") + kNs + "/diff_publisher/get_costmap");
  ASSERT_TRUE(client->wait_for_service(5s));
  auto fut = client->async_send_request(std::make_shared<nav2_msgs::srv::GetCostmap::Request>());
  ASSERT_EQ(fut.wait_for(5s), std::future_status::ready);
  auto resp = fut.get();
  {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> l(*master->getMutex());
    ASSERT_EQ(resp->map.metadata.size_x, 300u);
    ASSERT_EQ(resp->map.metadata.size_y, 200u);
    EXPECT_EQ(0, std::memcmp(resp->map.data.data(), master->getCharMap(), 300 * 200));
    const int64_t st = int64_t(resp->map.header.stamp.sec) * 1000000000LL + resp->map.header.stamp.nanosec;
    EXPECT_EQ(st, layer->engine()->lastStamp());
    EXPECT_EQ(resp->map.header.frame_id, "map");
  }

  // 늦게 붙은 구독자 (respawn 흉내): 새 구독자 FULL + durable history + catch-up 으로 master 와 같아져야 한다
  for (int c = 0; c < 80; ++c) {   // durable depth(기본 10) 보다 많은 update
    const int x0 = rng() % 250, y0 = rng() % 150;
    cycle(x0, y0, x0 + 1 + rng() % 50, y0 + 1 + rng() % 50, 1 + rng() % 30);
  }
  auto late_node = std::make_shared<rclcpp::Node>("late_joiner", kNs);
  auto late = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
    rclcpp::Node::WeakPtr(late_node), full_topic);
  rclcpp::executors::SingleThreadedExecutor exec2;
  exec2.add_node(late_node);
  std::thread spin2([&] {
      while (run && rclcpp::ok()) {
        exec2.spin_some(10ms);
        std::this_thread::sleep_for(2ms);
      }
    });
  for (int c = 0; c < 20; ++c) {
    const int x0 = rng() % 250, y0 = rng() % 150;
    cycle(x0, y0, x0 + 1 + rng() % 50, y0 + 1 + rng() % 50, 1 + rng() % 30);
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_TRUE(waitFor([&] {return sameAsMaster(*late, *master);}, 10s)) << "늦은 구독자 불일치";
  EXPECT_TRUE(waitFor([&] {return sameAsMaster(*sub, *master);}, 10s)) << "기존 구독자 불일치";

  std::this_thread::sleep_for(1200ms);   // 통계 log (1 s 주기) 한 번 보기
  run = false;
  spin.join();
  spin2.join();
  layer->deactivate();
  lc.reset();      // plugin 해제 → layer 소멸자 (thread join)
  layer.reset();
}

// ---------------------------------------------------------------------------
// 실제 LayeredCostmap::updateMap + 실제 InflationLayer 위에서 bounds 불변식 확인.
//   audit_period_cycles=1 → 매 cycle 지도 전체를 비교해 "bounds 밖에서 바뀐 셀" 을 센다 (0 이어야 한다).
//   장애물 추가/삭제, clear (resetLayers 흉내: master 전체 reset → 각 layer reset) 를 섞는다.
// ---------------------------------------------------------------------------
namespace
{
// 장애물을 찍고 지우는 단순 layer (ObstacleLayer 흉내: 바뀐 곳만 extra bounds, updateWithMax)
class MarkLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  void onInitialize() override
  {
    current_ = true;
    enabled_ = true;
    default_value_ = nav2_costmap_2d::FREE_SPACE;
    matchSize();
  }
  void reset() override
  {
    resetMaps();
    full_bounds_next_ = reset_full_bounds_;
  }
  bool isClearable() override {return true;}
  void updateBounds(double, double, double, double * min_x, double * min_y, double * max_x, double * max_y) override
  {
    useExtraBounds(min_x, min_y, max_x, max_y);
    if (full_bounds_next_) {
      // static layer reset 처럼 지도 전체
      *min_x = std::min(*min_x, getOriginX());
      *min_y = std::min(*min_y, getOriginY());
      *max_x = std::max(*max_x, getOriginX() + getSizeInMetersX());
      *max_y = std::max(*max_y, getOriginY() + getSizeInMetersY());
      full_bounds_next_ = false;
    }
  }
  void updateCosts(nav2_costmap_2d::Costmap2D & master, int min_i, int min_j, int max_i, int max_j) override
  {
    updateWithMax(master, min_i, min_j, max_i, max_j);
  }
  void mark(unsigned int x, unsigned int y, unsigned char v)
  {
    setCost(x, y, v);
    double wx, wy;
    mapToWorld(x, y, wx, wy);
    addExtraBounds(wx, wy, wx, wy);
  }
  bool reset_full_bounds_{true};

private:
  bool full_bounds_next_{false};
};
}  // namespace

TEST(DiffPublisherLayerRos, BoundsInvariantWithRealInflationLayer)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides(
  {
    rclcpp::Parameter("inflation.inflation_radius", 0.3),
    rclcpp::Parameter("inflation.cost_scaling_factor", 5.0),
    rclcpp::Parameter("diff.audit_period_cycles", 1),
    rclcpp::Parameter("diff.log_stats_period_sec", 0.0),
    rclcpp::Parameter("diff.tile_size", 32),
  });
  auto node = std::make_shared<nav2_util::LifecycleNode>("global_costmap2", kNs, opts);
  auto lc = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, false);
  lc->resizeMap(600, 400, 0.05, 0.0, 0.0);
  auto cbg = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto mark = std::make_shared<MarkLayer>();
  auto infl = std::make_shared<nav2_costmap_2d::InflationLayer>();
  auto diff = std::make_shared<ammr_costmap_diff_publisher::DiffPublisherLayer>();
  lc->addPlugin(mark);
  lc->addPlugin(infl);
  lc->addPlugin(diff);
  mark->initialize(lc.get(), "mark", nullptr, node, cbg);
  infl->initialize(lc.get(), "inflation", nullptr, node, cbg);
  diff->initialize(lc.get(), "diff", nullptr, node, cbg);
  std::vector<geometry_msgs::msg::Point> fp(4);
  fp[0].x = 0.2; fp[0].y = 0.2; fp[1].x = 0.2; fp[1].y = -0.2;
  fp[2].x = -0.2; fp[2].y = -0.2; fp[3].x = -0.2; fp[3].y = 0.2;
  lc->setFootprint(fp);
  diff->activate();

  auto shadowEqualsMaster = [&]() {
      nav2_msgs::msg::Costmap snap;
      if (!diff->engine()->snapshot(snap)) {
        return false;
      }
      auto * m = lc->getCostmap();
      return snap.data.size() == size_t(600) * 400 &&
             std::memcmp(snap.data.data(), m->getCharMap(), snap.data.size()) == 0;
    };

  std::mt19937 rng(17);
  std::vector<std::pair<unsigned, unsigned>> obstacles;
  uint64_t outside = 0, changed = 0, cycles = 0;
  for (int c = 0; c < 300; ++c) {
    {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*lc->getCostmap()->getMutex());
      const int op = rng() % 10;
      if (op < 5 || obstacles.empty()) {
        // 덩어리 추가
        const unsigned cx = 20 + rng() % 560, cy = 20 + rng() % 360;
        for (int k = 0; k < 6; ++k) {
          const unsigned x = cx + rng() % 5, y = cy + rng() % 5;
          mark->mark(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
          obstacles.push_back({x, y});
        }
      } else if (op < 9) {
        // 몇 개 지우기 (ghost 가 사라지는 경우)
        for (int k = 0; k < 6 && !obstacles.empty(); ++k) {
          const size_t i = rng() % obstacles.size();
          mark->mark(obstacles[i].first, obstacles[i].second, nav2_costmap_2d::FREE_SPACE);
          obstacles.erase(obstacles.begin() + i);
        }
      } else {
        // ClearEntireCostmap (Costmap2DROS::resetLayers 와 같은 순서)
        mark->reset_full_bounds_ = (rng() % 2) == 0;   // 반은 static layer 없이 작은 bounds
        auto * top = lc->getCostmap();
        top->resetMap(0, 0, top->getSizeInCellsX(), top->getSizeInCellsY());
        for (auto & p : *lc->getPlugins()) {
          p->reset();
        }
        obstacles.clear();
      }
    }
    lc->updateMap(15.0, 10.0, 0.0);   // 내부에서 master lock
    ASSERT_TRUE(shadowEqualsMaster()) << "cycle " << c;
    const auto st = diff->engine()->takeStats();
    outside += st.outside_bounds_cells;
    changed += st.changed_cells;
    cycles += st.cycles;
  }
  std::printf("[bounds-invariant] cycles %lu, changed cells %lu, outside-bounds changes %lu\n",
    (unsigned long)cycles, (unsigned long)changed, (unsigned long)outside);
  EXPECT_GT(changed, 0u);
  // reset 직후 cycle 은 audit 대신 reset 전체 비교라서 outside 를 세지 않는다. 그 밖의 cycle 에서는 0 이어야 한다.
  EXPECT_EQ(outside, 0u);

  diff->deactivate();
  lc.reset();
}

// ---------------------------------------------------------------------------
// [10-10 W1-D][리뷰] (F2/F8) smoother 처럼 CostmapSubscriber 의 costmap mutex 가 잡혀 있는 동안 (SimpleSmoother::smooth,
//   getCostmap 의 FULL 적용) update 가 몰려 오면 reader KeepLast(10) 이 오래된 것을 덮는다. 그 손실이
//   (a) repair 를 끄면 남아 있고 (리뷰 지적 재현), (b) refresh 로, (c) sweep 만으로도 메워지는지 본다.
// ---------------------------------------------------------------------------
namespace
{
struct BlockedResult
{
  bool lost{false};        // mutex 를 놓은 직후 master 와 다름 (= update 를 잃음)
  bool healed{false};
  double heal_sec{-1.0};
};

BlockedResult runBlockedSubscriber(
  const std::string & layer_name, const std::vector<rclcpp::Parameter> & extra, double wait_sec)
{
  BlockedResult out;
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter(layer_name + ".tile_size", 16),
    rclcpp::Parameter(layer_name + ".log_stats_period_sec", 0.0),
  };
  params.insert(params.end(), extra.begin(), extra.end());
  rclcpp::NodeOptions opts;
  opts.parameter_overrides(params);
  auto node = std::make_shared<nav2_util::LifecycleNode>("global_costmap_" + layer_name, kNs, opts);
  auto lc = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, false);
  lc->resizeMap(400, 300, 0.05, 0.0, 0.0);
  auto * master = lc->getCostmap();
  auto layer = std::make_shared<ammr_costmap_diff_publisher::DiffPublisherLayer>();
  lc->addPlugin(layer);
  auto cbg = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  layer->initialize(lc.get(), layer_name, nullptr, node, cbg);
  layer->activate();
  auto cycle = [&](unsigned x, unsigned y, unsigned char v) {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*master->getMutex());
      for (unsigned yy = y; yy < y + 3; ++yy) {
        for (unsigned xx = x; xx < x + 3; ++xx) {
          master->setCost(xx, yy, v);
        }
      }
      layer->updateCosts(*master, x, y, x + 3, y + 3);
    };
  cycle(0, 0, 1);   // 첫 cycle → FULL

  auto sub_node = std::make_shared<rclcpp::Node>("smoother_like_" + layer_name, kNs);
  const std::string full_topic = std::string("/") + kNs + "/" + layer_name + "/costmap_raw";
  auto sub = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(rclcpp::Node::WeakPtr(sub_node), full_topic);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(sub_node);
  std::atomic<bool> run{true};
  std::thread spin([&] {
      while (run && rclcpp::ok()) {
        exec.spin_some(10ms);
        std::this_thread::sleep_for(1ms);
      }
    });
  EXPECT_TRUE(waitFor([&] {return sameAsMaster(*sub, *master);}, 10s)) << layer_name << " full 미수신";
  std::this_thread::sleep_for(500ms);   // 첫 catch-up 등이 끝나도록

  {
    // smoother 의 smooth() 처럼 subscriber 의 costmap mutex 를 1 s 쥔다
    auto cm = sub->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> hold(*cm->getMutex());
    for (int k = 0; k < 40; ++k) {   // 서로 떨어진 40 곳 → update 40 개
      cycle(10 + (k % 10) * 38, 10 + (k / 10) * 70, static_cast<unsigned char>(100 + k));
      std::this_thread::sleep_for(1ms);
    }
    std::this_thread::sleep_for(1000ms);
  }
  std::this_thread::sleep_for(200ms);   // 남은 10 개를 처리할 시간
  out.lost = !sameAsMaster(*sub, *master);
  const auto t0 = std::chrono::steady_clock::now();
  out.healed = waitFor([&] {return sameAsMaster(*sub, *master);},
      std::chrono::milliseconds(static_cast<int>(wait_sec * 1000)));
  if (out.healed) {
    out.heal_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  }
  run = false;
  spin.join();
  layer->deactivate();
  lc.reset();
  layer.reset();
  return out;
}
}  // namespace

TEST(DiffPublisherLayerRos, BlockedSubscriberLosesUpdatesWithoutRepair)
{
  auto r = runBlockedSubscriber(
    "diff_norepair", {
      rclcpp::Parameter("diff_norepair.refresh_period_sec", 0.0),
      rclcpp::Parameter("diff_norepair.sweep_period_sec", 0.0),
      rclcpp::Parameter("diff_norepair.catchup_delays_sec", std::vector<double>{}),
    }, 3.0);
  std::printf("[blocked/no-repair] lost %d, healed %d\n", r.lost, r.healed);
  EXPECT_TRUE(r.lost);      // reader KeepLast(10) 에서 덮였다 (리뷰 F2/F8 재현)
  EXPECT_FALSE(r.healed);   // repair 가 없으면 그대로 남는다
}

TEST(DiffPublisherLayerRos, BlockedSubscriberHealsByRefresh)
{
  auto r = runBlockedSubscriber(
    "diff_refresh", {
      rclcpp::Parameter("diff_refresh.refresh_period_sec", 1.0),
      rclcpp::Parameter("diff_refresh.sweep_period_sec", 0.0),
    }, 6.0);
  std::printf("[blocked/refresh 1 s] lost %d, healed %d in %.2f s\n", r.lost, r.healed, r.heal_sec);
  EXPECT_TRUE(r.lost);
  EXPECT_TRUE(r.healed);
}

TEST(DiffPublisherLayerRos, BlockedSubscriberHealsBySweepOnly)
{
  auto r = runBlockedSubscriber(
    "diff_sweep", {
      rclcpp::Parameter("diff_sweep.refresh_period_sec", 0.0),
      rclcpp::Parameter("diff_sweep.sweep_period_sec", 2.0),
      rclcpp::Parameter("diff_sweep.sweep_tick_sec", 0.1),
    }, 6.0);
  std::printf("[blocked/sweep 2 s] lost %d, healed %d in %.2f s\n", r.lost, r.healed, r.heal_sec);
  EXPECT_TRUE(r.lost);
  EXPECT_TRUE(r.healed);
}

int main(int argc, char ** argv)
{
  // 격리 domain (sim 은 43/44/88, 개발 셸은 21)
  setenv("ROS_DOMAIN_ID", "229", 1);
  setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST", 1);
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int r = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return r;
}
