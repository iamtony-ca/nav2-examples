// [10-10 W1-5] FastClearLayer 시험.
//   격리: ROS_DOMAIN_ID=229 + localhost discovery (돌고 있는 sim 과 섞이지 않게), 노드 이름공간도 시험 전용.
//
//   (1) scanNonDefaultExtent 단위 시험 (무작위, default 0/255, 끝 열·행).
//   (2) 동등성: 같은 스택 두 벌 (A, B) 을 똑같이 돌린 뒤
//         A = 표준 ClearEntireCostmap (Costmap2DROS::resetLayers 복사: master resetMap + 모든 plugin/filter reset) + updateMap x2
//         B = FastClearLayer::clearNow() + updateMap x2
//       → master byte 동일, B 에서 창 (extent + inflation 반경) 밖 셀은 clear 전과 byte 동일, ghost 는 지워지고 현재 관측은 남는다.
//       스택 = 현장 global 순서: static(실제 StaticLayer) → lane_filter(실제 KeepoutFilter, plugin) → zed_front_layer(ObstacleLayer,
//       footprint clearing) → denoise(실제 DenoiseLayer, 5/8) → obstacle_layer(ObstacleLayer) → agent_like(시험용 칠하기 layer) →
//       fast_clear → inflation(실제 InflationLayer 0.61 m, 10.0).
//   (3) track_unknown_space=true (default NO_INFORMATION, raytrace 로 FREE 가 칠해진 셀도 extent 에 들어가야 한다).
//   (4) fast_clear 를 inflation 뒤에 둔 경우 (스스로 반경만큼 넓힌다) 도 동등.
//   (5) CostmapLayer 가 아닌 clearable layer → 지도 전체로 (표준과 같은 결과).
//   (6) 큰 지도 (기본 20000x10000 = 2e8 셀, FAST_CLEAR_BIG_X/Y 로 바꿈, FAST_CLEAR_BIG=0 이면 건너뜀) 에서 시간 비교 + 동등성
//       (L1: ghost 가 로봇 지나온 30 m 안, L2: 지도 양 끝 — 최악).
//   (7) 동시성: update thread 가 updateMap 을 계속 도는 동안 ROS 서비스로 clear 를 여러 번 → 모두 응답, 마지막 결과 = 표준 결과.
//   [10-10 W1-5][리뷰]
//   (9) ghost 의 inflation 링 (21 번째 셀 포함) 을 clear 전에 칠해 둔 이력 → A==B (창 여유가 반경 m 이면 21 번째 셀이 남는다).
//   (10) keepout mask 가 지도 전체 cycle 뒤에 도착 → FastClearLayer 가 한 번 지도 전체를 다시 계산하게 한다.
//   (11) deactivate 뒤 clear 요청은 아무것도 안 하고, 요청이 쏟아지는 중 deactivate → LayeredCostmap 파괴에도 죽지 않는다.
#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "ammr_costmap_diff_publisher/fast_clear_layer.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_filters/keepout_filter.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/denoise_layer.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "nav2_costmap_2d/static_layer.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;
using ammr_costmap_diff_publisher::CellBox;
using ammr_costmap_diff_publisher::FastClearLayer;
using ammr_costmap_diff_publisher::FastClearReport;
using nav2_costmap_2d::Costmap2D;

namespace
{

double msSince(std::chrono::steady_clock::time_point t)
{
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count();
}

// StaticLayer 의 protected incomingMap 을 시험에서 부른다 (지도 topic 없이)
class TestStaticLayer : public nav2_costmap_2d::StaticLayer
{
public:
  void feed(const nav_msgs::msg::OccupancyGrid::SharedPtr m) {incomingMap(m);}
};

// AgentLayer 흉내: 고정 사각형을 bounds 에 넣고 (매 cycle) 창 안에서만 master 에 LETHAL 을 max 로 칠한다. reset() 은 아무것도 안 함
// (pv_w4 AgentLayer::reset 은 current_=true 만), isClearable false (pv_w3+).
class PaintLayer : public nav2_costmap_2d::Layer
{
public:
  CellBox rect;
  void onInitialize() override {current_ = true;}
  void reset() override {}
  bool isClearable() override {return false;}
  void matchSize() override {}
  void updateBounds(double, double, double, double * min_x, double * min_y, double * max_x, double * max_y) override
  {
    auto * m = layered_costmap_->getCostmap();
    if (!rect.valid() || m->getSizeInCellsX() == 0) {
      return;
    }
    double wx0, wy0, wx1, wy1;
    m->mapToWorld(rect.x0, rect.y0, wx0, wy0);
    m->mapToWorld(rect.x1 - 1, rect.y1 - 1, wx1, wy1);
    *min_x = std::min(*min_x, wx0);
    *min_y = std::min(*min_y, wy0);
    *max_x = std::max(*max_x, wx1);
    *max_y = std::max(*max_y, wy1);
  }
  void updateCosts(Costmap2D & master, int min_i, int min_j, int max_i, int max_j) override
  {
    const int x0 = std::max(min_i, rect.x0), x1 = std::min(max_i, rect.x1);
    const int y0 = std::max(min_j, rect.y0), y1 = std::min(max_j, rect.y1);
    for (int y = y0; y < y1; ++y) {
      for (int x = x0; x < x1; ++x) {
        master.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }
};

// 큰 지도용 KeepoutFilter 흉내 (topic 으로 2e8 mask 를 보내지 않으려고): 같은 규칙 (NO_INFORMATION 건너뜀, 더 크면 덮음), 창 안만.
class KeepoutLikeLayer : public nav2_costmap_2d::Layer
{
public:
  std::vector<unsigned char> mask;   // master 와 같은 크기, NO_INFORMATION = 없음
  void onInitialize() override {current_ = true;}
  void reset() override {}
  bool isClearable() override {return false;}
  void matchSize() override {}
  void updateBounds(double, double, double, double *, double *, double *, double *) override {}
  void updateCosts(Costmap2D & master, int min_i, int min_j, int max_i, int max_j) override
  {
    if (mask.size() != size_t(master.getSizeInCellsX()) * master.getSizeInCellsY()) {
      return;
    }
    unsigned char * a = master.getCharMap();
    const size_t sx = master.getSizeInCellsX();
    for (int y = min_j; y < max_j; ++y) {
      for (int x = min_i; x < max_i; ++x) {
        const size_t i = size_t(y) * sx + x;
        const unsigned char d = mask[i];
        if (d == nav2_costmap_2d::NO_INFORMATION) {
          continue;
        }
        if (d > a[i] || a[i] == nav2_costmap_2d::NO_INFORMATION) {
          a[i] = d;
        }
      }
    }
  }
};

// CostmapLayer 가 아닌 clearable layer (extent 를 못 구하는 경우)
class OpaqueClearableLayer : public nav2_costmap_2d::Layer
{
public:
  int resets{0};
  void onInitialize() override {current_ = true;}
  void reset() override {++resets;}
  bool isClearable() override {return true;}
  void matchSize() override {}
  void updateBounds(double, double, double, double *, double *, double *, double *) override {}
  void updateCosts(Costmap2D &, int, int, int, int) override {}
};

void addPoints(
  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> ol, const std::vector<std::pair<double, double>> & pts,
  bool clearing, double ox, double oy)
{
  sensor_msgs::msg::PointCloud2 cloud;
  sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(pts.size());
  sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x"), iy(cloud, "y"), iz(cloud, "z");
  for (const auto & p : pts) {
    *ix = static_cast<float>(p.first);
    *iy = static_cast<float>(p.second);
    *iz = 0.5f;
    ++ix; ++iy; ++iz;
  }
  geometry_msgs::msg::Point o;
  o.x = ox;
  o.y = oy;
  o.z = 0.5;
  nav2_costmap_2d::Observation obs(o, cloud, 1e6, 0.0, 1e6, 0.0);
  ol->addStaticObservation(obs, true, clearing);
}

// 셀 (cx,cy) 부터 w x h 블록의 셀 중심 world 좌표
std::vector<std::pair<double, double>> blob(Costmap2D & m, int cx, int cy, int w, int h)
{
  std::vector<std::pair<double, double>> v;
  for (int y = cy; y < cy + h; ++y) {
    for (int x = cx; x < cx + w; ++x) {
      double wx, wy;
      m.mapToWorld(x, y, wx, wy);
      v.emplace_back(wx, wy);
    }
  }
  return v;
}

struct StackOptions
{
  std::string ns;
  unsigned int sx{2000}, sy{1000};
  double res{0.03};
  double ox{-5.0}, oy{-3.0};
  bool track_unknown{false};
  bool real_keepout{true};          // false 면 KeepoutLikeLayer
  bool denoise{true};
  bool fast_after_inflation{false};
  bool opaque_clearable{false};
  bool fast_enabled{true};
  std::string filter_info_topic{"/fc_test/filter_info"};
};

// 현장 global costmap 과 같은 순서의 스택
struct Stack
{
  StackOptions opt;
  nav2_util::LifecycleNode::SharedPtr node;
  std::unique_ptr<tf2_ros::Buffer> tf;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> lc;
  std::shared_ptr<TestStaticLayer> stat;
  std::shared_ptr<nav2_costmap_2d::KeepoutFilter> keepout;
  std::shared_ptr<KeepoutLikeLayer> keepout_like;
  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> zed, obst;
  std::shared_ptr<nav2_costmap_2d::DenoiseLayer> denoise;
  std::shared_ptr<PaintLayer> agent;
  std::shared_ptr<OpaqueClearableLayer> opaque;
  std::shared_ptr<FastClearLayer> fast;
  std::shared_ptr<nav2_costmap_2d::InflationLayer> infl;

  explicit Stack(const StackOptions & o)
  : opt(o)
  {
    rclcpp::NodeOptions nopt;
    nopt.parameter_overrides(
    {
      rclcpp::Parameter("zed_front_layer.footprint_clearing_enabled", true),
      rclcpp::Parameter("obstacle_layer.footprint_clearing_enabled", false),
      rclcpp::Parameter("denoise_layer.minimal_group_size", 5),
      rclcpp::Parameter("denoise_layer.group_connectivity_type", 8),
      rclcpp::Parameter("inflation_layer.inflation_radius", 0.61),
      rclcpp::Parameter("inflation_layer.cost_scaling_factor", 10.0),
      rclcpp::Parameter("lane_filter.filter_info_topic", o.filter_info_topic),
      rclcpp::Parameter("lane_filter.transform_tolerance", 0.5),
      rclcpp::Parameter("fast_clear.enabled", o.fast_enabled),
    });
    node = std::make_shared<nav2_util::LifecycleNode>("global_costmap", o.ns, nopt);
    // Costmap2DROS 가 선언하는 node 공통 param (nav2 obstacle_tests.cpp 와 같은 방식)
    node->declare_parameter("map_topic", rclcpp::ParameterValue(std::string("map")));
    node->declare_parameter("track_unknown_space", rclcpp::ParameterValue(o.track_unknown));
    node->declare_parameter("use_maximum", rclcpp::ParameterValue(false));
    node->declare_parameter("lethal_cost_threshold", rclcpp::ParameterValue(100));
    node->declare_parameter(
      "unknown_cost_value", rclcpp::ParameterValue(static_cast<int>(nav2_costmap_2d::NO_INFORMATION)));
    node->declare_parameter("trinary_costmap", rclcpp::ParameterValue(true));
    node->declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.3));
    tf = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    lc = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, o.track_unknown);

    auto add = [&](std::shared_ptr<nav2_costmap_2d::Layer> l, const std::string & name) {
        lc->addPlugin(l);
        l->initialize(lc.get(), name, tf.get(), node, nullptr);
      };
    stat = std::make_shared<TestStaticLayer>();
    add(stat, "static_layer");
    if (o.real_keepout) {
      keepout = std::make_shared<nav2_costmap_2d::KeepoutFilter>();
      add(keepout, "lane_filter");
    } else {
      keepout_like = std::make_shared<KeepoutLikeLayer>();
      add(keepout_like, "lane_filter");
    }
    zed = std::make_shared<nav2_costmap_2d::ObstacleLayer>();
    add(zed, "zed_front_layer");
    if (o.denoise) {
      denoise = std::make_shared<nav2_costmap_2d::DenoiseLayer>();
      add(denoise, "denoise_layer");
    }
    obst = std::make_shared<nav2_costmap_2d::ObstacleLayer>();
    add(obst, "obstacle_layer");
    if (o.opaque_clearable) {
      opaque = std::make_shared<OpaqueClearableLayer>();
      add(opaque, "opaque_layer");
    }
    agent = std::make_shared<PaintLayer>();
    add(agent, "agent_layer");
    fast = std::make_shared<FastClearLayer>();
    infl = std::make_shared<nav2_costmap_2d::InflationLayer>();
    if (o.fast_after_inflation) {
      add(infl, "inflation_layer");
      add(fast, "fast_clear");
    } else {
      add(fast, "fast_clear");
      add(infl, "inflation_layer");
    }
    std::vector<geometry_msgs::msg::Point> fp;
    for (auto xy : std::vector<std::pair<double, double>>{{0.32, 0.315}, {0.32, -0.315}, {-0.315, -0.315}, {-0.315, 0.315}}) {
      geometry_msgs::msg::Point p;
      p.x = xy.first;
      p.y = xy.second;
      fp.push_back(p);
    }
    lc->setFootprint(fp);
  }

  Costmap2D & master() {return *lc->getCostmap();}

  void standardClear()
  {
    // Costmap2DROS::resetLayers (fork costmap_2d_ros.cpp:682-700) + ClearCostmapService::clearEntirely 의 lock
    Costmap2D * top = lc->getCostmap();
    std::unique_lock<Costmap2D::mutex_t> lock(*top->getMutex());
    top->resetMap(0, 0, top->getSizeInCellsX(), top->getSizeInCellsY());
    for (auto & p : *lc->getPlugins()) {
      p->reset();
    }
    for (auto & f : *lc->getFilters()) {
      f->reset();
    }
  }

  std::vector<unsigned char> snapshot()
  {
    std::unique_lock<Costmap2D::mutex_t> lock(*master().getMutex());
    return std::vector<unsigned char>(
      master().getCharMap(), master().getCharMap() + size_t(master().getSizeInCellsX()) * master().getSizeInCellsY());
  }
};

// 정적 지도: 테두리 4 셀 + 가로 선반 벽 (6 셀 두께, 가운데 틈) + 작은 고립 점 (denoise 대상) — 0 / 100 / -1(unknown)
nav_msgs::msg::OccupancyGrid::SharedPtr makeStaticMap(const StackOptions & o, int shelf_pitch)
{
  auto m = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  m->header.frame_id = "map";
  m->info.width = o.sx;
  m->info.height = o.sy;
  m->info.resolution = static_cast<float>(o.res);
  m->info.origin.position.x = o.ox;
  m->info.origin.position.y = o.oy;
  m->info.origin.orientation.w = 1.0;
  m->data.assign(size_t(o.sx) * o.sy, 0);
  auto set = [&](int x, int y, int8_t v) {m->data[size_t(y) * o.sx + x] = v;};
  for (unsigned int y = 0; y < o.sy; ++y) {
    for (unsigned int x = 0; x < o.sx; ++x) {
      if (x < 4 || y < 4 || x >= o.sx - 4 || y >= o.sy - 4) {
        set(x, y, 100);
      }
    }
  }
  const int gap0 = static_cast<int>(o.sx) / 2 - 50, gap1 = static_cast<int>(o.sx) / 2 + 50;
  for (int r = shelf_pitch; r + 6 < static_cast<int>(o.sy) - 20; r += shelf_pitch) {
    for (int y = r; y < r + 6; ++y) {
      for (int x = 100; x < static_cast<int>(o.sx) - 100; ++x) {
        if (x < gap0 || x >= gap1) {
          set(x, y, 100);
        }
      }
    }
  }
  if (o.track_unknown) {
    // unknown 띠 (x 1050..1350, y 200..262): raytrace 가 FREE 로 칠하면 master 도 FREE 가 되는 자리
    for (int y = 200; y < 262; ++y) {
      for (int x = 1050; x < 1350; ++x) {
        set(x, y, -1);
      }
    }
  }
  return m;
}

nav_msgs::msg::OccupancyGrid::SharedPtr makeKeepoutMask(const StackOptions & o)
{
  auto m = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  m->header.frame_id = "map";
  m->info.width = o.sx;
  m->info.height = o.sy;
  m->info.resolution = static_cast<float>(o.res);
  m->info.origin.position.x = o.ox;
  m->info.origin.position.y = o.oy;
  m->info.origin.orientation.w = 1.0;
  m->data.assign(size_t(o.sx) * o.sy, 0);
  // lethal 차선 블록 두 개 + 50 % 비용 띠 + unknown 띠
  auto fill = [&](int x0, int y0, int x1, int y1, int8_t v) {
      for (int y = y0; y < y1; ++y) {
        for (int x = x0; x < x1; ++x) {
          m->data[size_t(y) * o.sx + x] = v;
        }
      }
    };
  fill(500, 560, 560, 570, 100);
  fill(1400, 700, 1500, 708, 100);
  fill(200, 300, 1800, 304, 50);
  fill(700, 100, 800, 110, -1);
  return m;
}

void spinUntil(nav2_util::LifecycleNode::SharedPtr node, std::function<bool()> pred, std::chrono::seconds to)
{
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(node->get_node_base_interface());
  const auto end = std::chrono::steady_clock::now() + to;
  while (!pred() && std::chrono::steady_clock::now() < end) {
    ex.spin_some(20ms);
    std::this_thread::sleep_for(5ms);
  }
  ex.remove_node(node->get_node_base_interface());
}

struct Scenario
{
  // 두 obstacle layer 에 남길 ghost (셀 좌표 블록), 현재 관측 (마지막 관측, 다시 찍힘)
  std::vector<std::array<int, 4>> ghosts_zed, ghosts_obst;
  std::array<int, 2> current_obstacle{0, 0};
  std::array<int, 2> robot_ghost_time{0, 0}, robot_now{0, 0};
};

// 두 스택에 같은 이력을 준다 (A, B 공통)
void runHistory(Stack & s, const Scenario & sc)
{
  auto & m = s.master();
  double rx, ry;
  m.mapToWorld(sc.robot_ghost_time[0], sc.robot_ghost_time[1], rx, ry);
  s.lc->updateMap(rx, ry, 0.0);
  s.lc->updateMap(rx, ry, 0.0);
  // ghost 를 찍는다 (clearing 없이 — raytrace 가 ghost 를 다시 지우지 않게)
  for (const auto & g : sc.ghosts_zed) {
    addPoints(s.zed, blob(m, g[0], g[1], g[2], g[3]), false, rx, ry);
  }
  for (const auto & g : sc.ghosts_obst) {
    addPoints(s.obst, blob(m, g[0], g[1], g[2], g[3]), false, rx, ry);
  }
  s.lc->updateMap(rx, ry, 0.0);
  // 센서가 더는 그 자리를 보지 않는다 → ghost 는 layer 에 남는다. 로봇은 이동, 지금 보이는 장애물 하나.
  s.zed->clearStaticObservations(true, true);
  s.obst->clearStaticObservations(true, true);
  double nx, ny;
  m.mapToWorld(sc.robot_now[0], sc.robot_now[1], nx, ny);
  double cx, cy;
  m.mapToWorld(sc.current_obstacle[0], sc.current_obstacle[1], cx, cy);
  addPoints(s.obst, {{cx, cy}}, s.opt.track_unknown, nx, ny);
  s.lc->updateMap(nx, ny, 0.0);
  s.lc->updateMap(nx, ny, 0.0);
}

struct CompareResult
{
  size_t diff_ab{0};
  // A!=B 셀 분류: 창 안 (빠른 clear 가 다시 계산한 곳 — 0 이어야 한다) / 창 밖이고 B == clear 전 (= 빠른 clear 가 건드리지 않은 셀을
  //   표준 clear 의 지도 전체 재계산만 바꿨다 = clear 전부터 있던 증분 갱신 흔적) / 그 밖 (0 이어야 한다)
  size_t diff_in_window{0}, diff_preexisting{0}, diff_other{0};
  size_t changed_outside_window{0};
  size_t changed_inside{0};
  CellBox window;   // extent+반경 창 ∪ 실제 창
  CellBox actual;   // FastClearLayer::postClearWindow()
  double a_clear_ms{0}, a_up1_ms{0}, a_up2_ms{0};
  double b_clear_ms{0}, b_up1_ms{0}, b_up2_ms{0};
  FastClearReport rep;
};

CompareResult clearAndCompare(Stack & A, Stack & B, std::array<int, 2> robot_now)
{
  CompareResult r;
  double nx, ny;
  B.master().mapToWorld(robot_now[0], robot_now[1], nx, ny);
  auto before = B.snapshot();

  auto t = std::chrono::steady_clock::now();
  A.standardClear();
  r.a_clear_ms = msSince(t);
  t = std::chrono::steady_clock::now();
  A.lc->updateMap(nx, ny, 0.0);
  r.a_up1_ms = msSince(t);
  t = std::chrono::steady_clock::now();
  A.lc->updateMap(nx, ny, 0.0);
  r.a_up2_ms = msSince(t);

  t = std::chrono::steady_clock::now();
  r.rep = B.fast->clearNow();
  r.b_clear_ms = msSince(t);
  t = std::chrono::steady_clock::now();
  B.lc->updateMap(nx, ny, 0.0);
  r.b_up1_ms = msSince(t);
  t = std::chrono::steady_clock::now();
  B.lc->updateMap(nx, ny, 0.0);
  r.b_up2_ms = msSince(t);

  auto a = A.snapshot();
  auto b = B.snapshot();
  EXPECT_EQ(a.size(), b.size());
  // 창 = LayeredCostmap::updateMap 이 실제로 쓰는 창과 같은 계산: clear 가 넘긴 world 사각형 (셀 중심 ± margin) 을
  //   InflationLayer::updateBounds 가 inflation_radius (m) 만큼 넓히고, worldToMapEnforceBounds + max 쪽 +1.
  const int sx = static_cast<int>(B.master().getSizeInCellsX()), sy = static_cast<int>(B.master().getSizeInCellsY());
  if (r.rep.extent.valid()) {
    r.actual = B.fast->postClearWindow();
    const double res = B.master().getResolution(), ox = B.master().getOriginX(), oy = B.master().getOriginY();
    const double grow = r.rep.margin_m + (r.rep.inflation_after ? 0.61 : 0.0);
    int x0, y0, x1, y1;
    B.master().worldToMapEnforceBounds(
      ox + (r.rep.extent.x0 + 0.5) * res - grow, oy + (r.rep.extent.y0 + 0.5) * res - grow, x0, y0);
    B.master().worldToMapEnforceBounds(
      ox + (r.rep.extent.x1 - 0.5) * res + grow, oy + (r.rep.extent.y1 - 0.5) * res + grow, x1, y1);
    r.window = CellBox{std::max(0, x0), std::max(0, y0), std::min(sx, x1 + 1), std::min(sy, y1 + 1)};
    // 실제 창 (clear 뒤 2 cycle 에 updateCosts 가 받은 창의 합집합) 은 extent+반경 창을 포함해야 한다.
    //   더 넓은 부분 = 같은 cycle 의 다른 bounds (로봇 footprint·현재 관측) 와의 bounding box 합집합 (LayeredCostmap 은 사각형 하나).
    EXPECT_TRUE(r.actual.x0 <= r.window.x0 && r.actual.y0 <= r.window.y0 && r.actual.x1 >= r.window.x1 &&
      r.actual.y1 >= r.window.y1) << "실제 창이 extent+반경 창을 덮지 않는다";
    r.window.merge(r.actual);
  }
  // [10-10 W1-5][리뷰] 지워진 범위 (extent) 에서 cell_inflation_radius (올림 — InflationLayer 가 실제로 칠하는 거리) 안의 셀은
  //   ghost 의 inflation 이 닿을 수 있는 곳이다. 그런 셀이 창 밖에서 A!=B 면 "clear 전부터의 흔적" 이 아니라 빠른 clear 의 잔여로 센다.
  const int cell_ir = static_cast<int>(B.master().cellDistance(0.61));
  auto nearCleared = [&](int x, int y) {
      if (!r.rep.extent.valid()) {
        return false;
      }
      const int dx = std::max({r.rep.extent.x0 - x, 0, x - (r.rep.extent.x1 - 1)});
      const int dy = std::max({r.rep.extent.y0 - y, 0, y - (r.rep.extent.y1 - 1)});
      return dx * dx + dy * dy <= cell_ir * cell_ir;
    };
  for (size_t i = 0; i < std::min(a.size(), b.size()); ++i) {
    if (a[i] != b[i]) {
      const int x = static_cast<int>(i % sx), y = static_cast<int>(i / sx);
      const bool in_win = r.window.valid() && x >= r.window.x0 && x < r.window.x1 && y >= r.window.y0 && y < r.window.y1;
      const char * kind;
      if (in_win) {
        ++r.diff_in_window;
        kind = "창 안";
      } else if (b[i] == before[i] && !nearCleared(x, y)) {
        ++r.diff_preexisting;
        kind = "창 밖·clear 전부터 (증분 갱신 흔적)";
      } else {
        ++r.diff_other;
        kind = b[i] == before[i] ? "그 밖 (창 밖이지만 지운 범위의 inflation 거리 안 = 빠른 clear 잔여)" : "그 밖";
      }
      if (r.diff_ab < 10) {
        std::printf(
          "[10-10 W1-5] A!=B 셀 (%d,%d): A=%d B=%d clear 전=%d — %s\n", x, y, a[i], b[i], before[i], kind);
      }
      ++r.diff_ab;
    }
  }
  // 로봇 주변 (현재 관측·footprint) 은 매 cycle 바뀔 수 있으므로 창 밖 검사에서 뺀다: 로봇 ± 2 m
  const int rr = static_cast<int>(2.0 / B.opt.res);
  for (int y = 0; y < sy && !r.rep.standard; ++y) {   // enabled=false (표준) 이면 창이 지도 전체라 분류하지 않는다
    for (int x = 0; x < sx; ++x) {
      const size_t i = size_t(y) * sx + x;
      if (before[i] == b[i]) {
        continue;
      }
      const bool in_win = r.window.valid() && x >= r.window.x0 && x < r.window.x1 && y >= r.window.y0 && y < r.window.y1;
      const bool near_robot = std::abs(x - robot_now[0]) <= rr && std::abs(y - robot_now[1]) <= rr;
      if (in_win || near_robot) {
        ++r.changed_inside;
      } else {
        if (r.changed_outside_window < 5) {
          std::printf(
            "[10-10 W1-5] 창 밖 변경 셀 (%d,%d): clear 전=%d B=%d A=%d (창 [%d,%d)x[%d,%d))\n", x, y, before[i], b[i], a[i],
            r.window.x0, r.window.x1, r.window.y0, r.window.y1);
        }
        ++r.changed_outside_window;
      }
    }
  }
  return r;
}

void printTimes(const char * tag, const CompareResult & r, unsigned int sx, unsigned int sy)
{
  std::printf(
    "[10-10 W1-5][%s] %ux%u: 표준 clear %.1f ms + update1 %.1f ms + update2 %.1f ms = %.1f ms | "
    "빠른 clear %.1f ms (scan %.1f, reset %.1f) + update1 %.1f ms + update2 %.1f ms = %.1f ms | "
    "extent %dx%d (%zu 셀) window %dx%d, A!=B %zu (창 안 %zu / clear 전부터 %zu / 그 밖 %zu), 창 밖 변경 %zu, 창 안 변경 %zu\n",
    tag, sx, sy, r.a_clear_ms, r.a_up1_ms, r.a_up2_ms, r.a_clear_ms + r.a_up1_ms + r.a_up2_ms,
    r.b_clear_ms, r.rep.scan_ms, r.rep.reset_ms, r.b_up1_ms, r.b_up2_ms, r.b_clear_ms + r.b_up1_ms + r.b_up2_ms,
    r.rep.extent.valid() ? r.rep.extent.x1 - r.rep.extent.x0 : 0,
    r.rep.extent.valid() ? r.rep.extent.y1 - r.rep.extent.y0 : 0, r.rep.extent.area(),
    r.window.valid() ? r.window.x1 - r.window.x0 : 0, r.window.valid() ? r.window.y1 - r.window.y0 : 0,
    r.diff_ab, r.diff_in_window, r.diff_preexisting, r.diff_other, r.changed_outside_window, r.changed_inside);
  std::fflush(stdout);
}

// 두 스택 만들고 정적 지도·keepout 을 준다
void prepare(Stack & s, rclcpp::Node::SharedPtr pub_node, nav_msgs::msg::OccupancyGrid::SharedPtr map,
  nav_msgs::msg::OccupancyGrid::SharedPtr mask)
{
  s.stat->feed(map);
  if (s.keepout) {
    s.keepout->activate();
    spinUntil(s.node, [&] {return s.keepout->isActive();}, 20s);
    ASSERT_TRUE(s.keepout->isActive()) << "keepout mask 미수신";
  } else if (s.keepout_like && mask) {
    s.keepout_like->mask.resize(mask->data.size());
    for (size_t i = 0; i < mask->data.size(); ++i) {
      const int8_t d = mask->data[i];
      s.keepout_like->mask[i] = d < 0 ? nav2_costmap_2d::NO_INFORMATION :
        static_cast<unsigned char>(std::round(d * 254.0 / 100.0));
    }
  }
  (void)pub_node;
}

struct KeepoutPublisher
{
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr info_pub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_pub;
  KeepoutPublisher(const std::string & info_topic, const std::string & mask_topic, nav_msgs::msg::OccupancyGrid::SharedPtr mask)
  {
    node = std::make_shared<rclcpp::Node>("fc_keepout_pub", "fc_test");
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    info_pub = node->create_publisher<nav2_msgs::msg::CostmapFilterInfo>(info_topic, qos);
    mask_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>(mask_topic, qos);
    nav2_msgs::msg::CostmapFilterInfo info;
    info.type = 0;
    info.filter_mask_topic = mask_topic;
    info.base = 0.0;
    info.multiplier = 1.0;
    info_pub->publish(info);
    mask_pub->publish(*mask);
  }
};

}  // namespace

// ---------------------------------------------------------------- (1)
TEST(FastClearExtent, MatchesBruteForce)
{
  std::mt19937 rng(11);
  for (int it = 0; it < 300; ++it) {
    const unsigned int sx = 1 + rng() % 70, sy = 1 + rng() % 40;
    const unsigned char dv = (it % 2) ? nav2_costmap_2d::NO_INFORMATION : nav2_costmap_2d::FREE_SPACE;
    std::vector<unsigned char> d(size_t(sx) * sy, dv);
    const int n = rng() % 6;
    for (int k = 0; k < n; ++k) {
      d[rng() % d.size()] = static_cast<unsigned char>(rng() % 256);
    }
    if (it % 7 == 0) {
      d[size_t(sy - 1) * sx + (sx - 1)] = dv ^ 1;   // 마지막 셀
    }
    CellBox bf;
    for (unsigned int y = 0; y < sy; ++y) {
      for (unsigned int x = 0; x < sx; ++x) {
        if (d[size_t(y) * sx + x] != dv) {
          bf.merge(CellBox{int(x), int(y), int(x) + 1, int(y) + 1});
        }
      }
    }
    const CellBox got = ammr_costmap_diff_publisher::scanNonDefaultExtent(d.data(), sx, sy, dv);
    ASSERT_EQ(bf.valid(), got.valid()) << it;
    if (bf.valid()) {
      EXPECT_EQ(bf.x0, got.x0) << it;
      EXPECT_EQ(bf.x1, got.x1) << it;
      EXPECT_EQ(bf.y0, got.y0) << it;
      EXPECT_EQ(bf.y1, got.y1) << it;
    }
  }
}

// ---------------------------------------------------------------- (2)
TEST(FastClearEquivalence, FieldOrderStackByteIdentical)
{
  StackOptions o;
  auto map = makeStaticMap(o, 166);
  auto mask = makeKeepoutMask(o);
  KeepoutPublisher kp(o.filter_info_topic, "/fc_test/keepout_mask", mask);
  StackOptions oa = o, ob = o;
  oa.ns = "fc_eq_A";
  ob.ns = "fc_eq_B";
  Stack A(oa), B(ob);
  prepare(A, kp.node, map, mask);
  prepare(B, kp.node, map, mask);

  Scenario sc;
  // 통로 가운데 (벽 행 150,316,482,648,814 사이): 4x4 ghost, 3x1 (denoise 가 master 에서 지우는 크기), 1 셀, 멀리 떨어진 것
  sc.ghosts_zed = {{300, 231, 4, 4}, {1700, 729, 4, 4}, {600, 398, 1, 1}};
  sc.ghosts_obst = {{1000, 563, 3, 1}, {1250, 231, 5, 3}};
  sc.robot_ghost_time = {900, 399};
  sc.robot_now = {400, 565};
  sc.current_obstacle = {440, 565};
  A.agent->rect = CellBox{1200, 395, 1220, 405};
  B.agent->rect = A.agent->rect;
  runHistory(A, sc);
  runHistory(B, sc);
  ASSERT_EQ(A.snapshot(), B.snapshot()) << "clear 전 두 스택이 같아야 한다";

  // ghost 가 master 에 있는지 (4x4 는 denoise 에 살아남는다)
  EXPECT_EQ(B.master().getCost(301, 232), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(B.master().getCost(1701, 730), nav2_costmap_2d::LETHAL_OBSTACLE);

  auto r = clearAndCompare(A, B, sc.robot_now);
  printTimes("field-order 2000x1000", r, o.sx, o.sy);
  EXPECT_EQ(r.diff_ab, 0u) << "표준 clear 와 master 가 다르다";
  EXPECT_EQ(r.changed_outside_window, 0u) << "창 밖 셀이 바뀌었다";
  EXPECT_GT(r.changed_inside, 0u);
  EXPECT_FALSE(r.rep.standard);
  EXPECT_TRUE(r.rep.inflation_after);
  // [10-10 W1-5][리뷰] inflation 이 뒤에서 0.61 m 를 더하므로 이 layer 는 올림 몫 (21 x 0.03 − 0.61 = 0.02 m) 만 더한다
  EXPECT_EQ(r.rep.cell_inflation_radius, 21u);
  EXPECT_NEAR(r.rep.margin_m, 21 * 0.03 - 0.61, 1e-6);   // master 해상도는 float (0.03f)
  // keepout mask 가 첫 cycle 전에 와 있었다 → 첫 (어차피 지도 전체인) cycle 에 한 번
  EXPECT_EQ(B.fast->maskRepaints(), 1u);
  // 지운 layer = clearable 인 두 obstacle layer 뿐 (static/keepout/denoise/agent/inflation/fast_clear 제외)
  ASSERT_EQ(r.rep.layers.size(), 2u);
  EXPECT_EQ(r.rep.layers[0].name, "zed_front_layer");
  EXPECT_EQ(r.rep.layers[1].name, "obstacle_layer");
  // ghost 는 지워지고 지금 보이는 장애물은 남는다
  EXPECT_NE(B.master().getCost(301, 232), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_NE(B.master().getCost(1701, 730), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(B.master().getCost(440, 565), nav2_costmap_2d::LETHAL_OBSTACLE);
  // 두 번째 clear (ghost 없음) → extent 는 지금 관측 하나뿐
  auto r2 = clearAndCompare(A, B, sc.robot_now);
  printTimes("field-order 2000x1000 (2회째)", r2, o.sx, o.sy);
  EXPECT_EQ(r2.diff_ab, 0u);
  EXPECT_EQ(r2.changed_outside_window, 0u);
  EXPECT_LE(r2.rep.extent.area(), 4u);
}

// ---------------------------------------------------------------- (3)
TEST(FastClearEquivalence, TrackUnknownSpaceFreeCellsCount)
{
  StackOptions o;
  o.track_unknown = true;
  o.real_keepout = false;
  o.denoise = false;
  auto map = makeStaticMap(o, 166);
  auto mask = makeKeepoutMask(o);
  StackOptions oa = o, ob = o;
  oa.ns = "fc_tu_A";
  ob.ns = "fc_tu_B";
  Stack A(oa), B(ob);
  // keepout mask 없음: mask 의 0 은 unknown master 를 FREE 로 덮는다 (KeepoutFilter 규칙) → unknown 띠를 보려고 뺀다
  (void)mask;
  prepare(A, nullptr, map, nullptr);
  prepare(B, nullptr, map, nullptr);
  Scenario sc;
  sc.ghosts_zed = {{300, 231, 4, 4}};
  sc.ghosts_obst = {{1500, 729, 4, 4}};
  sc.robot_ghost_time = {900, 399};
  sc.robot_now = {400, 565};
  sc.current_obstacle = {470, 565};    // clearing 관측 → 로봇~장애물 사이가 FREE 로 칠해진다 (default 255 와 다름)
  runHistory(A, sc);
  runHistory(B, sc);
  // ghost 를 찍을 때도 raytrace 로 FREE 를 칠한다 (track_unknown 에서 FREE 는 default 가 아니다)
  {
    double rx, ry, gx, gy;
    for (Stack * s : {&A, &B}) {
      s->master().mapToWorld(1100, 231, rx, ry);
      s->master().mapToWorld(1300, 231, gx, gy);
      addPoints(s->zed, {{gx, gy}}, true, rx, ry);
      double nx, ny;
      s->master().mapToWorld(sc.robot_now[0], sc.robot_now[1], nx, ny);
      s->lc->updateMap(nx, ny, 0.0);
      s->zed->clearStaticObservations(true, true);
      s->lc->updateMap(nx, ny, 0.0);
    }
  }
  ASSERT_EQ(A.snapshot(), B.snapshot());
  EXPECT_EQ(B.master().getCost(1200, 231), nav2_costmap_2d::FREE_SPACE) << "raytrace FREE 가 master 에 있어야 한다";
  auto r = clearAndCompare(A, B, sc.robot_now);
  printTimes("track_unknown 2000x1000", r, o.sx, o.sy);
  EXPECT_EQ(r.diff_ab, 0u);
  EXPECT_EQ(r.changed_outside_window, 0u);
  // FREE 로 칠해졌던 띠 (x 1100..1300) 도 extent 에 들어갔어야 한다
  EXPECT_LE(r.rep.extent.x0, 1100);
  EXPECT_GE(r.rep.extent.x1, 1300);
  EXPECT_EQ(B.master().getCost(1200, 231), nav2_costmap_2d::NO_INFORMATION);
}

// ---------------------------------------------------------------- (4)
TEST(FastClearEquivalence, FastLayerAfterInflationSelfMargin)
{
  StackOptions o;
  o.real_keepout = false;
  o.fast_after_inflation = true;
  auto map = makeStaticMap(o, 166);
  auto mask = makeKeepoutMask(o);
  StackOptions oa = o, ob = o;
  oa.ns = "fc_ai_A";
  ob.ns = "fc_ai_B";
  Stack A(oa), B(ob);
  prepare(A, nullptr, map, mask);
  prepare(B, nullptr, map, mask);
  Scenario sc;
  sc.ghosts_zed = {{300, 231, 4, 4}};
  sc.ghosts_obst = {{1700, 729, 6, 6}};
  sc.robot_ghost_time = {900, 399};
  sc.robot_now = {400, 565};
  sc.current_obstacle = {440, 565};
  runHistory(A, sc);
  runHistory(B, sc);
  auto r = clearAndCompare(A, B, sc.robot_now);
  printTimes("fast_clear 가 inflation 뒤", r, o.sx, o.sy);
  EXPECT_FALSE(r.rep.inflation_after);
  // [10-10 W1-5][리뷰] 스스로 넓히는 여유 = cell_inflation_radius 셀 (올림 21 셀 = 0.63 m), 반경 m (0.61) 이 아니다
  EXPECT_EQ(r.rep.cell_inflation_radius, 21u);
  EXPECT_NEAR(r.rep.margin_m, 21 * 0.03, 1e-6);   // master 해상도는 float (0.03f)
  EXPECT_EQ(r.diff_ab, 0u);
  EXPECT_EQ(r.changed_outside_window, 0u);
}

// ---------------------------------------------------------------- (5)
TEST(FastClearEquivalence, OpaqueClearableLayerFallsBackToFullMap)
{
  StackOptions o;
  o.sx = 600;
  o.sy = 400;
  o.real_keepout = false;
  o.denoise = false;
  o.opaque_clearable = true;
  auto map = makeStaticMap(o, 120);
  StackOptions oa = o, ob = o;
  oa.ns = "fc_op_A";
  ob.ns = "fc_op_B";
  Stack A(oa), B(ob);
  prepare(A, nullptr, map, nullptr);
  prepare(B, nullptr, map, nullptr);
  Scenario sc;
  sc.ghosts_zed = {{100, 60, 4, 4}};
  sc.ghosts_obst = {{500, 300, 4, 4}};
  sc.robot_ghost_time = {300, 180};
  sc.robot_now = {200, 180};
  sc.current_obstacle = {230, 180};
  runHistory(A, sc);
  runHistory(B, sc);
  auto r = clearAndCompare(A, B, sc.robot_now);
  EXPECT_EQ(r.diff_ab, 0u);
  EXPECT_EQ(B.opaque->resets, 1);
  EXPECT_EQ(r.rep.extent.area(), size_t(o.sx) * o.sy);
  bool found = false;
  for (const auto & l : r.rep.layers) {
    if (l.name == "opaque_layer") {
      found = true;
      EXPECT_TRUE(l.full);
    }
  }
  EXPECT_TRUE(found);
}

// ---------------------------------------------------------------- (6)
TEST(FastClearBigMap, TimingAndEquivalence)
{
  const char * en = std::getenv("FAST_CLEAR_BIG");
  if (en && std::string(en) == "0") {
    GTEST_SKIP() << "FAST_CLEAR_BIG=0";
  }
  StackOptions o;
  o.sx = 20000;
  o.sy = 10000;
  if (const char * e = std::getenv("FAST_CLEAR_BIG_X")) {
    o.sx = static_cast<unsigned int>(std::atoi(e));
  }
  if (const char * e = std::getenv("FAST_CLEAR_BIG_Y")) {
    o.sy = static_cast<unsigned int>(std::atoi(e));
  }
  o.real_keepout = false;   // 2e8 mask 를 topic 으로 보내지 않는다 (KeepoutLikeLayer 는 같은 규칙, 좌표 변환 없이 더 싸다)
  o.denoise = true;
  auto t0 = std::chrono::steady_clock::now();
  auto map = makeStaticMap(o, 166);
  auto mask = std::make_shared<nav_msgs::msg::OccupancyGrid>(*map);
  std::fill(mask->data.begin(), mask->data.end(), 0);
  for (unsigned int y = 1000; y < o.sy - 1000; y += 500) {      // 차선 keepout 띠
    for (unsigned int x = 300; x < o.sx - 300; ++x) {
      mask->data[size_t(y) * o.sx + x] = 100;
      mask->data[size_t(y + 1) * o.sx + x] = 100;
    }
  }
  StackOptions oa = o, ob = o;
  oa.ns = "fc_big_A";
  ob.ns = "fc_big_B";
  Stack A(oa), B(ob);
  prepare(A, nullptr, map, mask);
  prepare(B, nullptr, map, mask);
  map.reset();
  mask.reset();
  std::printf("[10-10 W1-5][big] 준비 %.1f s\n", msSince(t0) / 1000.0);

  // L1: 로봇이 지나온 30 m 안 ghost 여러 개 (pitch 166 셀 통로 가운데 행 = 166k + 83)
  Scenario sc;
  const int cx = static_cast<int>(o.sx) / 3, cy = 166 * 20 + 83;
  for (int k = 0; k < 8; ++k) {
    sc.ghosts_zed.push_back({cx + k * 120, cy, 4, 4});
    sc.ghosts_obst.push_back({cx + 60 + k * 120, cy + 166, 3, 5});
  }
  sc.robot_ghost_time = {cx + 500, cy};
  sc.robot_now = {cx + 1000, cy};
  sc.current_obstacle = {cx + 1040, cy};
  auto th = std::chrono::steady_clock::now();
  runHistory(A, sc);
  runHistory(B, sc);
  std::printf("[10-10 W1-5][big] 이력 %.1f s\n", msSince(th) / 1000.0);
  auto r1 = clearAndCompare(A, B, sc.robot_now);
  printTimes("big L1 (30 m 안)", r1, o.sx, o.sy);
  // 창 안은 byte 동일. 창 밖 A!=B 는 "B == clear 전" 인 셀만 허용 — 표준 clear 의 지도 전체 재계산이 clear 전부터 있던 증분 갱신 흔적
  //   (InflationLayer::updateBounds 는 반경을 m 로 넓히는데 updateCosts 는 cell_inflation_radius_ (올림) 셀까지 칠한다 →
  //   창 가장자리 바로 밖 1 셀이 증분 갱신에서 빠진다) 을 고친 것이다. 빠른 clear 는 그 셀을 건드리지 않는다. W1_STEP5.md §4.
  EXPECT_EQ(r1.diff_in_window, 0u);
  EXPECT_EQ(r1.diff_other, 0u);
  EXPECT_EQ(r1.changed_outside_window, 0u);
  EXPECT_LT(r1.b_clear_ms + r1.b_up1_ms + r1.b_up2_ms, r1.a_clear_ms + r1.a_up1_ms + r1.a_up2_ms);

  // L2: 지도 양 끝 (최악: 합집합이 지도 거의 전체)
  Scenario s2;
  s2.ghosts_zed = {{200, 83 + 166, 4, 4}};
  s2.ghosts_obst = {{static_cast<int>(o.sx) - 300, 166 * ((static_cast<int>(o.sy) - 400) / 166) + 83, 4, 4}};
  s2.robot_ghost_time = sc.robot_now;
  s2.robot_now = sc.robot_now;
  s2.current_obstacle = sc.current_obstacle;
  runHistory(A, s2);
  runHistory(B, s2);
  auto r2 = clearAndCompare(A, B, s2.robot_now);
  printTimes("big L2 (지도 양 끝)", r2, o.sx, o.sy);
  EXPECT_EQ(r2.diff_in_window, 0u);
  EXPECT_EQ(r2.diff_other, 0u);
  EXPECT_EQ(r2.changed_outside_window, 0u);
}

// ---------------------------------------------------------------- (7)
TEST(FastClearConcurrency, ServiceWhileUpdating)
{
  StackOptions o;
  o.ns = "fc_cc";
  o.real_keepout = false;
  auto map = makeStaticMap(o, 166);
  auto mask = makeKeepoutMask(o);
  Stack S(o);
  prepare(S, nullptr, map, mask);
  Scenario sc;
  sc.ghosts_zed = {{300, 231, 4, 4}};
  sc.ghosts_obst = {{1700, 729, 4, 4}};
  sc.robot_ghost_time = {900, 399};
  sc.robot_now = {400, 565};
  sc.current_obstacle = {440, 565};
  runHistory(S, sc);

  std::atomic<bool> run{true};
  std::atomic<int> cycles{0};
  std::thread upd([&] {
      std::mt19937 rng(3);
      double nx, ny;
      S.master().mapToWorld(sc.robot_now[0], sc.robot_now[1], nx, ny);
      while (run) {
        // 가끔 새 ghost (update thread 안에서만 관측을 바꾼다 — 실제 노드에서도 관측은 updateBounds 가 읽는다)
        if (rng() % 5 == 0) {
          const int gx = 100 + rng() % 1800;
          const int row = 83 + 166 * static_cast<int>(rng() % 5);
          addPoints(S.zed, blob(S.master(), gx, row, 2, 2), false, nx, ny);
          S.lc->updateMap(nx, ny, 0.0);
          S.zed->clearStaticObservations(true, true);
        }
        S.lc->updateMap(nx, ny, 0.0);
        ++cycles;
      }
    });

  auto cli_node = std::make_shared<rclcpp::Node>("fc_client", "fc_test");
  auto client = cli_node->create_client<nav2_msgs::srv::ClearEntireCostmap>("/fc_cc/fast_clear/clear_entirely");
  ASSERT_TRUE(client->wait_for_service(10s)) << S.fast->serviceName();
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(cli_node);
  int ok = 0;
  for (int k = 0; k < 20; ++k) {
    auto fut = client->async_send_request(std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>());
    if (ex.spin_until_future_complete(fut, 10s) == rclcpp::FutureReturnCode::SUCCESS) {
      ++ok;
    }
    std::this_thread::sleep_for(30ms);
  }
  run = false;
  upd.join();
  EXPECT_EQ(ok, 20);
  EXPECT_GE(S.fast->calls(), 20u);
  EXPECT_GT(cycles.load(), 20);

  // 마지막 빠른 clear + 2 cycle 결과 == 같은 layer 상태에서 표준 clear + 2 cycle 결과
  double nx, ny;
  S.master().mapToWorld(sc.robot_now[0], sc.robot_now[1], nx, ny);
  S.fast->clearNow();
  S.lc->updateMap(nx, ny, 0.0);
  S.lc->updateMap(nx, ny, 0.0);
  auto fast_snap = S.snapshot();
  S.standardClear();
  S.lc->updateMap(nx, ny, 0.0);
  S.lc->updateMap(nx, ny, 0.0);
  auto std_snap = S.snapshot();
  size_t diff = 0;
  for (size_t i = 0; i < fast_snap.size(); ++i) {
    diff += fast_snap[i] != std_snap[i];
  }
  EXPECT_EQ(diff, 0u);
  std::printf("[10-10 W1-5][concurrency] 서비스 %d/20 응답, update cycle %d, 최종 표준 대비 다른 셀 %zu\n", ok, cycles.load(), diff);
}

// ---------------------------------------------------------------- (8) 증분 갱신 흔적 (빠른 clear 와 무관, 표준 clear 는 지도 전체로 고친다)
// InflationLayer::updateBounds 는 bounds 를 inflation_radius_ (0.61 m = 20.33 셀) 만큼 넓히는데 updateCosts 는 cell_inflation_radius_
// (= ceil(20.33) = 21 셀) 까지 칠한다 (fork inflation_layer.cpp:619-622, Costmap2D::cellDistance). 그래서 새 장애물의 21 번째 셀
// (비용 10) 은 증분 갱신 창 바로 밖이라 master 에 들어가지 않는다. 표준 clear (지도 전체 재계산) 는 그 셀을 채운다.
// [10-10 W1-5][리뷰] 빠른 clear 창은 이제 extent ± 21 셀 (cell_inflation_radius) 이라 extent 안 장애물 (clear 전 layer 에 있던 것) 의
// 21 번째 셀도 다시 계산한다 → 이 경우 빠른 clear 도 표준과 같이 채운다. (예전 창 ± 20 셀에서는 0 으로 남아 표준과 달랐다.)
TEST(FastClearEquivalence, PreexistingInflationEdgeArtifact)
{
  StackOptions o;
  o.ns = "fc_edge";
  o.sx = 600;
  o.sy = 400;
  o.real_keepout = false;
  o.denoise = false;
  auto map = makeStaticMap(o, 120);
  Stack S(o);
  prepare(S, nullptr, map, nullptr);
  double rx, ry, px, py;
  S.master().mapToWorld(200, 180, rx, ry);
  S.master().mapToWorld(300, 180, px, py);
  S.lc->updateMap(rx, ry, 0.0);
  S.lc->updateMap(rx, ry, 0.0);
  addPoints(S.obst, {{px, py}}, false, rx, ry);   // 계속 보이는 장애물 (clear 뒤에도 다시 찍힌다)
  S.lc->updateMap(rx, ry, 0.0);
  S.lc->updateMap(rx, ry, 0.0);
  EXPECT_EQ(S.master().getCost(300, 180), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_GT(S.master().getCost(320, 180), 0) << "20 번째 셀은 창 안";
  EXPECT_EQ(S.master().getCost(321, 180), 0) << "21 번째 셀은 증분 갱신 창 밖 (흔적)";
  S.fast->clearNow();
  S.lc->updateMap(rx, ry, 0.0);
  S.lc->updateMap(rx, ry, 0.0);
  const int fast = S.master().getCost(321, 180);
  EXPECT_GT(fast, 0) << "빠른 clear: 창이 extent ± 21 셀 → 21 번째 셀도 다시 계산";
  EXPECT_GT(S.master().getCost(279, 180), 0);
  S.standardClear();
  S.lc->updateMap(rx, ry, 0.0);
  S.lc->updateMap(rx, ry, 0.0);
  const int full = S.master().getCost(321, 180);
  EXPECT_GT(full, 0) << "표준 clear: 지도 전체 재계산이 21 번째 셀을 채운다";
  EXPECT_EQ(fast, full);
  std::printf("[10-10 W1-5][edge] 21 번째 셀: 증분 = 0, 빠른 clear = %d, 표준 clear = %d\n", fast, full);
}

// ---------------------------------------------------------------- enabled=false (표준 스위치)
TEST(FastClearEquivalence, DisabledMeansStandard)
{
  StackOptions o;
  o.sx = 600;
  o.sy = 400;
  o.real_keepout = false;
  o.denoise = false;
  auto map = makeStaticMap(o, 120);
  StackOptions oa = o, ob = o;
  oa.ns = "fc_dis_A";
  ob.ns = "fc_dis_B";
  ob.fast_enabled = false;
  Stack A(oa), B(ob);
  prepare(A, nullptr, map, nullptr);
  prepare(B, nullptr, map, nullptr);
  Scenario sc;
  sc.ghosts_zed = {{100, 60, 4, 4}};
  sc.ghosts_obst = {{500, 300, 4, 4}};
  sc.robot_ghost_time = {300, 180};
  sc.robot_now = {200, 180};
  sc.current_obstacle = {230, 180};
  runHistory(A, sc);
  runHistory(B, sc);
  auto r = clearAndCompare(A, B, sc.robot_now);
  EXPECT_TRUE(r.rep.standard);
  EXPECT_EQ(r.diff_ab, 0u);
}

// ---------------------------------------------------------------- (9) [10-10 W1-5][리뷰] ghost 의 21 번째 셀이 clear 전에 칠해져 있던 이력
// 현장은 marking 관측이 clearing:true·raytrace 2-5 m 라서 ghost 를 찍는 cycle 의 창이 ghost + 21 셀을 덮고, 로봇이 근처에 있는 동안
// 다른 cycle 도 그렇다 → ghost 의 inflation 링이 21 번째 셀 (비용 ~10) 까지 master 에 들어간다. 여기서는 marking 뒤 지도 전체 cycle
// (static reset) 을 한 번 돌려 그 상태를 만든다. 창 여유가 inflation_radius (m, 20.33 셀) 뿐이면 창은 extent ± 20 셀이라 21 번째 셀이
// 표준 clear (0) 와 달리 10 으로 남는다. fast_clear 가 inflation 앞 (inflation 이 넓힘) / 뒤 (스스로 넓힘) 두 경우.
TEST(FastClearEquivalence, GhostRingPaintedBeforeClear)
{
  for (const bool after : {false, true}) {
    StackOptions o;
    o.real_keepout = false;
    o.fast_after_inflation = after;
    auto map = makeStaticMap(o, 166);
    auto mask = makeKeepoutMask(o);
    StackOptions oa = o, ob = o;
    oa.ns = after ? "fc_ring_after_A" : "fc_ring_A";
    ob.ns = after ? "fc_ring_after_B" : "fc_ring_B";
    Stack A(oa), B(ob);
    prepare(A, nullptr, map, mask);
    prepare(B, nullptr, map, mask);
    Scenario sc;
    sc.ghosts_zed = {{300, 231, 4, 4}, {1700, 729, 4, 4}, {600, 398, 1, 1}};
    sc.ghosts_obst = {{1000, 563, 3, 1}, {1250, 231, 5, 3}};
    sc.robot_ghost_time = {900, 399};
    sc.robot_now = {400, 565};
    sc.current_obstacle = {440, 565};
    A.agent->rect = CellBox{1200, 395, 1220, 405};
    B.agent->rect = A.agent->rect;
    runHistory(A, sc);
    runHistory(B, sc);
    for (Stack * s : {&A, &B}) {
      double nx, ny;
      s->master().mapToWorld(sc.robot_now[0], sc.robot_now[1], nx, ny);
      s->stat->reset();   // 다음 cycle bounds = 지도 전체 → ghost 링 전체 (21 번째 셀 포함) 가 master 에 칠해진다
      s->lc->updateMap(nx, ny, 0.0);
      s->lc->updateMap(nx, ny, 0.0);
    }
    ASSERT_EQ(A.snapshot(), B.snapshot());
    // ghost (300..303, 231..234) 의 왼쪽 21 번째 셀 / 오른쪽 21 번째 셀
    const int ring_l = B.master().getCost(279, 232), ring_r = B.master().getCost(324, 232);
    EXPECT_GT(ring_l, 0) << "clear 전 ghost 링 21 번째 셀이 칠해져 있어야 시험이 의미가 있다";
    EXPECT_GT(ring_r, 0);
    auto r = clearAndCompare(A, B, sc.robot_now);
    printTimes(after ? "ghost 링 칠한 뒤 (fast_clear 가 inflation 뒤)" : "ghost 링 칠한 뒤", r, o.sx, o.sy);
    std::printf(
      "[10-10 W1-5][ring] after=%d 21 번째 셀 clear 전 %d/%d → 빠른 clear 뒤 %d/%d (표준 %d/%d), margin %.3f m\n", after, ring_l,
      ring_r, B.master().getCost(279, 232), B.master().getCost(324, 232), A.master().getCost(279, 232),
      A.master().getCost(324, 232), r.rep.margin_m);
    EXPECT_EQ(r.diff_ab, 0u) << "ghost 링 잔여";
    EXPECT_EQ(r.diff_preexisting, 0u);
    EXPECT_EQ(r.changed_outside_window, 0u);
    EXPECT_EQ(B.master().getCost(279, 232), 0);
    EXPECT_EQ(B.master().getCost(324, 232), 0);
  }
}

// ---------------------------------------------------------------- (10) [10-10 W1-5][리뷰] keepout mask 가 지도 전체 cycle 뒤에 도착
// KeepoutFilter::maskCallback 은 mask 를 저장만 하고 bounds 를 넓히지 않는다 → 늦게 온 mask 는 이후 창 안에서만 master 에 들어간다.
// 예전에는 명령마다 표준 clear 가 지도 전체를 다시 칠해 고쳤다. FastClearLayer 는 keepout 이 inactive → active 로 바뀐 것을 보면
// 한 번 지도 전체 bounds 를 요구한다.
TEST(FastClearEquivalence, LateKeepoutMaskRepainted)
{
  StackOptions o;
  o.ns = "fc_late";
  o.denoise = false;
  o.filter_info_topic = "/fc_test/late_filter_info";
  auto map = makeStaticMap(o, 166);
  auto mask = makeKeepoutMask(o);
  Stack S(o);
  S.stat->feed(map);
  S.keepout->activate();
  double nx, ny;
  S.master().mapToWorld(400, 565, nx, ny);
  S.lc->updateMap(nx, ny, 0.0);   // 지도 전체 cycle 두 번 — mask 없이
  S.lc->updateMap(nx, ny, 0.0);
  S.lc->updateMap(nx, ny, 0.0);   // 증분
  ASSERT_FALSE(S.keepout->isActive());
  EXPECT_NE(S.master().getCost(1450, 704), nav2_costmap_2d::LETHAL_OBSTACLE);
  KeepoutPublisher kp(o.filter_info_topic, "/fc_test/late_keepout_mask", mask);
  spinUntil(S.node, [&] {return S.keepout->isActive();}, 20s);
  ASSERT_TRUE(S.keepout->isActive()) << "keepout mask 미수신";
  S.lc->updateMap(nx, ny, 0.0);
  // 로봇 (400,565) 에서 먼 keepout 블록 두 개
  EXPECT_EQ(S.master().getCost(1450, 704), nav2_costmap_2d::LETHAL_OBSTACLE) << "늦게 온 keepout mask 가 master 에 없다";
  EXPECT_EQ(S.master().getCost(530, 565), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(S.master().getCost(1000, 302), static_cast<unsigned char>(std::round(50 * 254.0 / 100.0)));
  EXPECT_EQ(S.fast->maskRepaints(), 1u);
  // 한 번만: InflationLayer 의 last_* 때문에 바로 다음 cycle 도 지도 전체 (표준 clear·시작 때와 같은 2 cycle) 이고,
  //   그 뒤로는 작은 창 (지도 전체 재계산이 반복되지 않는다) — 시간으로 확인
  auto t = std::chrono::steady_clock::now();
  S.lc->updateMap(nx, ny, 0.0);
  const double second_ms = msSince(t);
  t = std::chrono::steady_clock::now();
  S.lc->updateMap(nx, ny, 0.0);
  S.lc->updateMap(nx, ny, 0.0);
  const double small_ms = msSince(t) / 2.0;
  S.stat->reset();
  t = std::chrono::steady_clock::now();
  S.lc->updateMap(nx, ny, 0.0);
  const double full_ms = msSince(t);
  std::printf(
    "[10-10 W1-5][late-mask] mask 뒤 둘째 cycle (inflation last_*) %.2f ms, 그 뒤 증분 cycle %.2f ms, 지도 전체 cycle %.2f ms\n",
    second_ms, small_ms, full_ms);
  EXPECT_LT(small_ms * 3.0, full_ms) << "mask 도착 뒤에도 지도 전체 cycle 이 반복된다";
  EXPECT_EQ(S.fast->maskRepaints(), 1u);
}

// ---------------------------------------------------------------- (11) [10-10 W1-5][리뷰] deactivate 뒤에는 clear 서비스가 layer 목록을 건드리지 않는다
// Costmap2DROS::on_cleanup 은 layered_costmap_.reset() → ~LayeredCostmap 이 lock 없이 plugins_ 를 pop_back 한다. 그 전에
// on_deactivate → stop() → 모든 plugin deactivate() 가 불린다. deactivate 가 돌아온 뒤 clear 요청은 아무것도 하지 않아야 하고,
// deactivate 는 진행 중인 clear 가 끝날 때까지 기다려야 한다.
TEST(FastClearLifecycle, DeactivateStopsClear)
{
  StackOptions o;
  o.ns = "fc_deact";
  o.sx = 600;
  o.sy = 400;
  o.real_keepout = false;
  o.denoise = false;
  auto map = makeStaticMap(o, 120);
  Stack S(o);
  prepare(S, nullptr, map, nullptr);
  Scenario sc;
  sc.ghosts_zed = {{100, 60, 4, 4}};
  sc.ghosts_obst = {{500, 300, 4, 4}};
  sc.robot_ghost_time = {300, 180};
  sc.robot_now = {200, 180};
  sc.current_obstacle = {230, 180};
  runHistory(S, sc);
  double nx, ny;
  S.master().mapToWorld(sc.robot_now[0], sc.robot_now[1], nx, ny);
  ASSERT_EQ(S.master().getCost(101, 61), nav2_costmap_2d::LETHAL_OBSTACLE);

  S.fast->deactivate();
  EXPECT_TRUE(S.fast->clearNow().skipped);
  S.lc->updateMap(nx, ny, 0.0);
  S.lc->updateMap(nx, ny, 0.0);
  EXPECT_EQ(S.master().getCost(101, 61), nav2_costmap_2d::LETHAL_OBSTACLE) << "deactivate 뒤 clear 가 layer 를 지웠다";
  EXPECT_EQ(S.zed->getCost(101, 61), nav2_costmap_2d::LETHAL_OBSTACLE);

  S.fast->activate();
  S.fast->clearNow();
  S.lc->updateMap(nx, ny, 0.0);
  S.lc->updateMap(nx, ny, 0.0);
  EXPECT_NE(S.master().getCost(101, 61), nav2_costmap_2d::LETHAL_OBSTACLE) << "activate 뒤 clear 가 다시 동작해야 한다";

  // 서비스 요청을 계속 보내는 중에 deactivate → LayeredCostmap 파괴 (on_deactivate → on_cleanup 순서). 죽지 않아야 한다.
  auto cli_node = std::make_shared<rclcpp::Node>("fc_deact_client", "fc_test");
  auto client = cli_node->create_client<nav2_msgs::srv::ClearEntireCostmap>("/fc_deact/fast_clear/clear_entirely");
  ASSERT_TRUE(client->wait_for_service(10s));
  std::atomic<bool> run{true};
  std::atomic<int> sent{0};
  std::thread spam([&] {
      rclcpp::executors::SingleThreadedExecutor ex;
      ex.add_node(cli_node);
      while (run) {
        auto fut = client->async_send_request(std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>());
        ex.spin_until_future_complete(fut, 200ms);
        ++sent;
      }
    });
  std::this_thread::sleep_for(300ms);
  const uint64_t calls_before = S.fast->calls();
  S.fast->deactivate();
  const uint64_t calls_at_deact = S.fast->calls();
  S.lc.reset();   // ~LayeredCostmap: plugins_ pop_back (lock 없음)
  std::this_thread::sleep_for(300ms);
  run = false;
  spam.join();
  std::printf(
    "[10-10 W1-5][deactivate] 요청 %d 회, deactivate 전 clear %lu 회, deactivate 시점 %lu, 끝 %lu, 무시 %lu\n", sent.load(),
    static_cast<unsigned long>(calls_before), static_cast<unsigned long>(calls_at_deact),
    static_cast<unsigned long>(S.fast->calls()), static_cast<unsigned long>(S.fast->skippedCalls()));
  EXPECT_GT(calls_before, 0u);
  EXPECT_EQ(S.fast->calls(), calls_at_deact) << "deactivate 뒤에도 clear 가 돌았다";
  EXPECT_GT(S.fast->skippedCalls(), 1u) << "deactivate 뒤 요청이 skipped 로 처리되어야 한다";
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
