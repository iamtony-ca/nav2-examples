// [10-10 W1-2] AgentMaskCanvas 동등성 시험.
//
//   기준(reference) = amhs 원본 agent_layer.cpp (md5 27f4da73) 의 updateCosts 1)~3) 와 fillFootprintAt /
//   dilateFootprintDirectional 을 이 파일에 글자 그대로 옮긴 것 (ref* 함수). 새 코드(AgentMaskCanvas +
//   rasterizeFootprint)와 legacy 모드를 같은 무작위 입력으로 돌려 매 사이클 viz grid·master grid·meta_hits 가
//   바이트 단위로 같은지, publish 사각형이 바뀐 셀을 모두 덮는지, 사이클 작업량이 칠한 사각형에 묶이는지 본다.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <random>
#include <set>
#include <utility>
#include <vector>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "multi_agent_nav2/agent_mask_canvas.hpp"

// [10-10 W1-2][리뷰] publish 접착 코드 → fork publisher 모형 → path_validator 의 실제 CostmapMirror 까지 잇는 시험용
#include <nav2_msgs/msg/costmap.hpp>
#include <nav2_msgs/msg/costmap_update.hpp>
#ifdef AGENT_MASK_TEST_WITH_MIRROR
#include "replan_monitor/costmap_mirror.hpp"
#include "replan_monitor/mirror_sync_policy.hpp"   // [10-10 W1-3]
#endif

using multi_agent_nav2::AgentMaskCanvas;
using multi_agent_nav2::CellRect;
using Costmap2D = nav2_costmap_2d::Costmap2D;
using Hits = std::vector<std::pair<unsigned int, unsigned int>>;

namespace
{

rclcpp::Logger testLogger() {return rclcpp::get_logger("test_agent_mask_canvas");}
rclcpp::Clock::SharedPtr testClock()
{
  static auto clk = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  return clk;
}

// ======================= 기준: amhs 원본 그대로 =======================
// agent_layer.cpp:521-574 (원본) 그대로
static inline std::vector<geometry_msgs::msg::Point>
refDilateFootprintDirectional(const std::vector<geometry_msgs::msg::Point32> & in,
                           double iso_dilate_m,
                           double forward_len_m,
                           double rear_len_m,            // [NEW] rear(-x) 확장
                           rclcpp::Logger logger,
                           rclcpp::Clock::SharedPtr clock)
{
  std::vector<geometry_msgs::msg::Point> out; out.reserve(in.size());
  if (in.empty()) return out;

  double cx=0, cy=0;
  for (auto & p : in) { cx += p.x; cy += p.y; }
  cx /= static_cast<double>(in.size());
  cy /= static_cast<double>(in.size());

  // 다각형 중심에서 가장 가까운 변(또는 꼭짓점)까지의 거리 계산
  double min_dist = 1e9;
  for (const auto & p : in) {
      min_dist = std::min(min_dist, std::hypot(p.x - cx, p.y - cy));
  }

  // 사용자의 파라미터 에러 방어 (너무 큰 음수값 제한)
  double applied_dilate = iso_dilate_m;
  if (applied_dilate < 0.0 && std::abs(applied_dilate) >= min_dist) {
      RCLCPP_WARN_THROTTLE(logger, *clock, 2000,
          "[AgentLayer] Negative dilation (%.2f) exceeds min radius (%.2f). Clamping to safe boundary to prevent bow-tie effect.",
          iso_dilate_m, min_dist);
      // 최소 1cm(0.01m) 형체는 남겨두도록 강제 제한
      applied_dilate = -min_dist + 0.01;
  }

  for (auto & p : in) {
    double vx = p.x - cx, vy = p.y - cy;
    double n = std::hypot(vx, vy); if (n < 1e-6) n = 1.0;

    double x_local = p.x + applied_dilate * (vx / n);
    double y_local = p.y + applied_dilate * (vy / n);

    // forward: 중심보다 앞쪽(+x) 정점만 앞으로
    if (forward_len_m > 1e-6 && (p.x - cx) >= 0.0) {
      x_local += forward_len_m;
    }
    // [NEW] rear: 중심보다 뒤쪽(-x) 정점만 뒤로
    if (rear_len_m > 1e-6 && (p.x - cx) < 0.0) {
      x_local -= rear_len_m;
    }

    geometry_msgs::msg::Point q;
    q.x = x_local; q.y = y_local; q.z = 0.0;
    out.push_back(q);
  }
  return out;
}

// agent_layer.cpp:576-638 (원본) 그대로. touch_* 갱신(:640-649)은 grid 와 무관해서 뺐다.
void refFillFootprintAt(const std::vector<geometry_msgs::msg::Point32> & fp_points,
                        const geometry_msgs::msg::Pose & pose,
                        double extra_dilation_m,
                        double forward_len_m,
                        double rear_len_m,
                        nav2_costmap_2d::Costmap2D * grid,
                        unsigned char cost,
                        std::vector<std::pair<unsigned int,unsigned int>> * meta_hits)
{
  auto poly = refDilateFootprintDirectional(
      fp_points, extra_dilation_m, forward_len_m, rear_len_m,  // [NEW]
      testLogger(), testClock());

  const double yaw = tf2::getYaw(pose.orientation);
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (auto & p : poly) {
    const double x = p.x, y = p.y;
    p.x = pose.position.x + c * x - s * y;
    p.y = pose.position.y + s * x + c * y;
  }

  double minx=1e9, miny=1e9, maxx=-1e9, maxy=-1e9;
  for (auto & p : poly) {
    minx = std::min(minx, p.x);
    miny = std::min(miny, p.y);
    maxx = std::max(maxx, p.x);
    maxy = std::max(maxy, p.y);
  }

  int min_i, min_j, max_i, max_j;
  grid->worldToMapEnforceBounds(minx, miny, min_i, min_j);
  grid->worldToMapEnforceBounds(maxx, maxy, max_i, max_j);

  for (int j = min_j; j <= max_j; ++j) {
    for (int i = min_i; i <= max_i; ++i) {
      double wx, wy; grid->mapToWorld(i, j, wx, wy);

      bool inside = false;
      const size_t n = poly.size();
      for (size_t k=0, h=n-1; k<n; h=k++) {
        const double xi = poly[k].x, yi = poly[k].y;
        const double xh = poly[h].x, yh = poly[h].y;

        const bool hit = ((yi > wy) != (yh > wy)) &&
                         (wx < (xh - xi) * (wy - yi) / (yh - yi) + xi);
        if (hit) {
            inside = !inside;
        }
      }

      if (inside) {
        const unsigned char old_raw = grid->getCost(i, j);
        const int old = (old_raw == nav2_costmap_2d::NO_INFORMATION) ? 0 : static_cast<int>(old_raw);
        const int cand = static_cast<int>(cost);
        if (cand > old) {
          grid->setCost(static_cast<unsigned int>(i),
                        static_cast<unsigned int>(j), cand);
        }
        if (meta_hits) meta_hits->emplace_back(
            static_cast<unsigned int>(i), static_cast<unsigned int>(j));
      }
    }
  }
}

// agent_layer.cpp:742-761 (원본 updateCosts 1)) 그대로
void refBeginCycle(Costmap2D & viz_costmap_, const Costmap2D & master_grid)
{
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(viz_costmap_.getMutex()));

  if (viz_costmap_.getSizeInCellsX() != master_grid.getSizeInCellsX() ||
      viz_costmap_.getSizeInCellsY() != master_grid.getSizeInCellsY() ||
      viz_costmap_.getResolution()   != master_grid.getResolution()   ||
      viz_costmap_.getOriginX()      != master_grid.getOriginX()      ||
      viz_costmap_.getOriginY()      != master_grid.getOriginY())
  {
    viz_costmap_.resizeMap(master_grid.getSizeInCellsX(),
                           master_grid.getSizeInCellsY(),
                           master_grid.getResolution(),
                           master_grid.getOriginX(),
                           master_grid.getOriginY());
  }

  unsigned char * char_map = viz_costmap_.getCharMap();
  const unsigned int n = viz_costmap_.getSizeInCellsX() * viz_costmap_.getSizeInCellsY();
  std::memset(char_map, 0, n * sizeof(unsigned char));
}

// agent_layer.cpp:773-790 (원본 updateCosts 3)) 그대로
void refMerge(const Costmap2D & viz_costmap_, Costmap2D & master_grid,
              int min_i, int min_j, int max_i, int max_j)
{
  const int gx_min = std::max(0, min_i);
  const int gy_min = std::max(0, min_j);
  const int gx_max = std::min<int>(master_grid.getSizeInCellsX(), max_i);
  const int gy_max = std::min<int>(master_grid.getSizeInCellsY(), max_j);

  for (int j = gy_min; j < gy_max; ++j) {
    for (int i = gx_min; i < gx_max; ++i) {
      const unsigned char our = viz_costmap_.getCost(i, j);
      if (our == 0) continue;  // 우리가 안 그린 셀은 master 그대로 둠

      const unsigned char master_raw = master_grid.getCost(i, j);
      const int master_val = (master_raw == nav2_costmap_2d::NO_INFORMATION)
                                ? 0 : static_cast<int>(master_raw);
      if (static_cast<int>(our) > master_val) {
        master_grid.setCost(i, j, our);
      }
    }
  }
}
// ===================== 기준 끝 =====================

std::vector<unsigned char> snapshot(const Costmap2D & cm)
{
  const size_t n = static_cast<size_t>(cm.getSizeInCellsX()) * cm.getSizeInCellsY();
  return std::vector<unsigned char>(cm.getCharMap(), cm.getCharMap() + n);
}

// publisher 쪽 사각형을 구독자 배열에 복사 (Costmap2DPublisher::createCostmapUpdateMsg + CostmapMirror::copyRect 와 같은 일)
void applyRect(const Costmap2D & src, std::vector<unsigned char> & sub, const CellRect & r)
{
  const size_t sx = src.getSizeInCellsX();
  for (unsigned int j = r.y0; j < r.yn; ++j) {
    std::memcpy(sub.data() + j * sx + r.x0, src.getCharMap() + j * sx + r.x0, r.xn - r.x0);
  }
}

geometry_msgs::msg::Pose makePose(double x, double y, double yaw)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x; p.position.y = y;
  tf2::Quaternion q; q.setRPY(0, 0, yaw);
  p.orientation.x = q.x(); p.orientation.y = q.y(); p.orientation.z = q.z(); p.orientation.w = q.w();
  return p;
}

std::vector<geometry_msgs::msg::Point32> toP32(const std::vector<geometry_msgs::msg::Point> & pts)
{
  std::vector<geometry_msgs::msg::Point32> out;
  for (const auto & p : pts) {
    geometry_msgs::msg::Point32 q; q.x = static_cast<float>(p.x); q.y = static_cast<float>(p.y); q.z = 0.f;
    out.push_back(q);
  }
  return out;
}

// 시험용 무작위 agent. AgentLayer::rasterizeAgentPath 처럼 몸체 1 번 + 경로 점 여러 번 칠한다.
struct SimAgent
{
  bool alive{false};
  std::vector<geometry_msgs::msg::Point32> fp;
  double x{0}, y{0}, yaw{0}, v{0}, w{0};
  unsigned char body_cost{254};
  double iso{0.05}, fwd{0.0}, rear{0.0}, path_dil{-0.01};
  int path_n{0};
  double path_step{0.2};
  int base_cost{254}, end_cost{254};
};

struct Paint
{
  std::vector<geometry_msgs::msg::Point32> fp;
  geometry_msgs::msg::Pose pose;
  double iso, fwd, rear;
  unsigned char cost;
};

std::vector<Paint> paintsFor(const std::vector<SimAgent> & agents)
{
  std::vector<Paint> out;
  for (const auto & a : agents) {
    if (!a.alive) {continue;}
    out.push_back({a.fp, makePose(a.x, a.y, a.yaw), a.iso, a.fwd, a.rear, a.body_cost});
    for (int i = 0; i < a.path_n; ++i) {
      const double d = a.path_step * (i + 1);
      // 경로 점의 자세 방향은 몸체 방향 (rasterizeAgentPath: ps.orientation = current orientation)
      const double px = a.x + d * std::cos(a.yaw + 0.15 * i * a.w);
      const double py = a.y + d * std::sin(a.yaw + 0.15 * i * a.w);
      const double ratio = (a.path_n > 1) ? static_cast<double>(i) / (a.path_n - 1) : 0.0;
      int dc = a.base_cost - static_cast<int>((a.base_cost - a.end_cost) * ratio);
      dc = std::clamp(dc, 0, 254);
      if (dc == 0) {continue;}
      out.push_back({a.fp, makePose(px, py, a.yaw), a.path_dil, 0.0, 0.0, static_cast<unsigned char>(dc)});
    }
  }
  return out;
}

class Scenario
{
public:
  explicit Scenario(unsigned seed)
  : rng_(seed) {}

  double uni(double a, double b) {return std::uniform_real_distribution<double>(a, b)(rng_);}
  int uint(int a, int b) {return std::uniform_int_distribution<int>(a, b)(rng_);}

  void spawn(SimAgent & a, const Costmap2D & m)
  {
    a.alive = true;
    const double wx0 = m.getOriginX(), wy0 = m.getOriginY();
    const double wx1 = wx0 + m.getSizeInCellsX() * m.getResolution();
    const double wy1 = wy0 + m.getSizeInCellsY() * m.getResolution();
    // 일부는 지도 가장자리 밖·걸침 (worldToMapEnforceBounds 클램프 경로)
    a.x = uni(wx0 - 1.0, wx1 + 1.0);
    a.y = uni(wy0 - 1.0, wy1 + 1.0);
    a.yaw = uni(-M_PI, M_PI);
    a.v = uni(0.0, 0.25);
    a.w = uni(-0.3, 0.3);
    const int kind = uint(0, 2);
    if (kind == 0) {
      a.fp = toP32(nav2_costmap_2d::makeFootprintFromRadius(uni(0.15, 0.6)));
    } else if (kind == 1) {
      const double hx = uni(0.2, 0.5), hy = uni(0.15, 0.45);
      a.fp = {};
      const double corners[4][2] = {{hx, hy}, {hx, -hy}, {-hx, -hy}, {-hx, hy}};
      for (const auto & cxy : corners) {
        geometry_msgs::msg::Point32 q; q.x = static_cast<float>(cxy[0]); q.y = static_cast<float>(cxy[1]);
        a.fp.push_back(q);
      }
    } else {
      // 비대칭 오각형
      a.fp = {};
      const double r = uni(0.2, 0.5);
      for (int k = 0; k < 5; ++k) {
        const double ang = 2 * M_PI * k / 5 + uni(-0.2, 0.2);
        geometry_msgs::msg::Point32 q;
        q.x = static_cast<float>(r * std::cos(ang) + 0.1); q.y = static_cast<float>(r * 0.8 * std::sin(ang));
        a.fp.push_back(q);
      }
    }
    a.body_cost = static_cast<unsigned char>(uint(1, 254));
    a.iso = uni(-0.6, 0.3);         // 큰 음수 → bow-tie clamp 경로도 탄다
    a.fwd = uint(0, 1) ? uni(0.0, 0.6) : 0.0;
    a.rear = uint(0, 2) == 0 ? uni(0.0, 0.5) : 0.0;
    a.path_dil = uni(-0.3, 0.2);
    a.path_n = uint(0, 12);
    a.path_step = uni(0.05, 0.5);
    a.base_cost = uint(0, 254);
    a.end_cost = uint(0, 254);
  }

  void step(std::vector<SimAgent> & agents, const Costmap2D & m)
  {
    for (auto & a : agents) {
      if (!a.alive) {
        if (uint(0, 9) == 0) {spawn(a, m);}   // 나타남
        continue;
      }
      if (uint(0, 29) == 0) {a.alive = false; continue;}   // 사라짐
      a.yaw += a.w * 0.2;
      a.x += a.v * std::cos(a.yaw);
      a.y += a.v * std::sin(a.yaw);
      if (uint(0, 19) == 0) {a.iso = uni(-0.6, 0.3);}            // 파라미터 변경 (dilation)
      if (uint(0, 19) == 0) {a.path_n = uint(0, 12);}            // 경로 길이 변경
      if (uint(0, 39) == 0) {a.fwd = uni(0.0, 0.6);}
    }
  }

  // 하위 layer 가 master 에 얹은 값 (NO_INFORMATION·lethal 포함)
  void fillLower(Costmap2D & m)
  {
    const size_t n = static_cast<size_t>(m.getSizeInCellsX()) * m.getSizeInCellsY();
    unsigned char * d = m.getCharMap();
    for (size_t k = 0; k < n; ++k) {
      const int r = uint(0, 99);
      d[k] = r < 70 ? 0 : (r < 80 ? 255 : static_cast<unsigned char>(uint(1, 254)));
    }
  }

  // updateCosts 의 bounds (음수·크기 초과·빈 범위 포함)
  void bounds(const Costmap2D & m, int & min_i, int & min_j, int & max_i, int & max_j)
  {
    const int sx = static_cast<int>(m.getSizeInCellsX()), sy = static_cast<int>(m.getSizeInCellsY());
    if (uint(0, 3) == 0) {min_i = 0; min_j = 0; max_i = sx; max_j = sy; return;}
    min_i = uint(-5, sx / 2); max_i = uint(min_i - 3, sx + 5);
    min_j = uint(-5, sy / 2); max_j = uint(min_j - 3, sy + 5);
  }

  std::mt19937 & rng() {return rng_;}

private:
  std::mt19937 rng_;
};

struct Geometry
{
  unsigned int sx, sy; double res, ox, oy;
};

}  // namespace

// 무작위 agent 집합(이동·출현·소멸·footprint/dilation 변화·지도 밖)을 기준(원본 memset), legacy, 새 방식으로
// 돌려 매 사이클 viz grid / master grid / meta_hits 가 같은지, update 사각형이 바뀐 셀을 모두 덮는지 본다.
// 도중에 크기+원점 변경 1 번, 원점만 변경 1 번.
TEST(AgentMaskCanvas, RandomCyclesMatchOriginalFullMemset)
{
  for (unsigned seed : {1u, 7u, 42u, 2026u}) {
    SCOPED_TRACE("seed " + std::to_string(seed));
    Scenario sc(seed);

    std::vector<Geometry> geoms = {
      {260, 180, 0.05, -3.2, 1.7},
      {300, 140, 0.05, -6.0, -2.0},   // 크기 + 원점 변경
      {300, 140, 0.05, -5.5, -2.0},   // 원점만 변경
    };
    const int cycles = 420;
    const int change1 = 140, change2 = 280;

    Costmap2D ref_viz, legacy_viz, new_viz, new2_viz;
    // [10-10 W1-2][리뷰] Costmap2D() 는 default_value_ 를 초기화하지 않는다 (예전 주석 "default value = 0" 은 틀렸고 쓰레기 값에
    //   기대고 있었다). AgentLayer 처럼 0 으로 둔다. canvas 는 생성자에서 스스로 0 으로 두지만 ref_viz 는 canvas 가 없다.
    ref_viz.setDefaultValue(0);
    AgentMaskCanvas legacy(legacy_viz), cand(new_viz), cand2(new2_viz);
    legacy.setLegacyFullClear(true);
    cand2.setPublishHistory(2);

    std::vector<SimAgent> agents(7);
    std::vector<unsigned char> sub, sub2, sub2_drop, sub1_drop;   // update 를 받는 구독자 흉내
    bool prev_dropped = false;
    uint64_t total_cleared = 0, total_merge = 0, total_pub = 0, map_cells_total = 0;
    uint64_t ghost_cycles_h1 = 0;
    int empty_pub_cycles = 0;

    for (int cyc = 0; cyc < cycles; ++cyc) {
      const Geometry & g = geoms[cyc < change1 ? 0 : (cyc < change2 ? 1 : 2)];
      Costmap2D lower(g.sx, g.sy, g.res, g.ox, g.oy, 0);
      sc.fillLower(lower);
      Costmap2D ref_master(lower), legacy_master(lower), new_master(lower), new2_master(lower);

      sc.step(agents, lower);
      // 가끔 전원 사라짐 (빈 사이클 연속)
      if (cyc % 97 == 50 || cyc % 97 == 51) {
        for (auto & a : agents) {a.alive = false;}
      }
      const auto paints = paintsFor(agents);

      const auto before = new_viz.getSizeInCellsX() ? snapshot(new_viz) : std::vector<unsigned char>{};
      const CellRect prev_painted = cand.painted();

      refBeginCycle(ref_viz, ref_master);
      legacy.beginCycle(legacy_master);
      const bool resized = cand.beginCycle(new_master);
      cand2.beginCycle(new2_master);

      // 작업량: 새 방식은 직전 사각형만 지운다
      if (!resized) {
        ASSERT_EQ(cand.lastStats().cleared_cells, prev_painted.area());
        ASSERT_EQ(cand.previous().area(), prev_painted.area());
      } else {
        ASSERT_EQ(cand.lastStats().cleared_cells, 0u);
      }
      ASSERT_EQ(legacy.lastStats().cleared_cells, static_cast<uint64_t>(g.sx) * g.sy);
      ASSERT_EQ(resized, cyc == 0 || cyc == change1 || cyc == change2);

      Hits ref_hits, legacy_hits, new_hits, new2_hits;
      for (const auto & p : paints) {
        refFillFootprintAt(p.fp, p.pose, p.iso, p.fwd, p.rear, &ref_viz, p.cost, &ref_hits);
        multi_agent_nav2::rasterizeFootprint(p.fp, p.pose, p.iso, p.fwd, p.rear, legacy, p.cost, &legacy_hits,
          testLogger(), testClock());
        multi_agent_nav2::rasterizeFootprint(p.fp, p.pose, p.iso, p.fwd, p.rear, cand, p.cost, &new_hits,
          testLogger(), testClock());
        multi_agent_nav2::rasterizeFootprint(p.fp, p.pose, p.iso, p.fwd, p.rear, cand2, p.cost, &new2_hits,
          testLogger(), testClock());
      }
      ASSERT_EQ(ref_hits, new_hits);
      ASSERT_EQ(ref_hits, legacy_hits);
      ASSERT_EQ(ref_hits, new2_hits);

      int min_i, min_j, max_i, max_j;
      sc.bounds(lower, min_i, min_j, max_i, max_j);
      refMerge(ref_viz, ref_master, min_i, min_j, max_i, max_j);
      legacy.mergeInto(legacy_master, min_i, min_j, max_i, max_j);
      cand.mergeInto(new_master, min_i, min_j, max_i, max_j);
      cand2.mergeInto(new2_master, min_i, min_j, max_i, max_j);

      // viz grid 와 master grid 가 바이트 단위로 같다
      const auto ref_s = snapshot(ref_viz);
      ASSERT_EQ(ref_s, snapshot(legacy_viz)) << "cycle " << cyc;
      ASSERT_EQ(ref_s, snapshot(new_viz)) << "cycle " << cyc;
      ASSERT_EQ(ref_s, snapshot(new2_viz)) << "cycle " << cyc;
      ASSERT_EQ(snapshot(ref_master), snapshot(legacy_master)) << "cycle " << cyc;
      ASSERT_EQ(snapshot(ref_master), snapshot(new_master)) << "cycle " << cyc;
      ASSERT_EQ(snapshot(ref_master), snapshot(new2_master)) << "cycle " << cyc;

      // 칠한 사각형은 0 이 아닌 셀을 모두 담는다 (다음 사이클에 지울 범위가 충분하다)
      const CellRect painted = cand.painted();
      const CellRect pub = cand.publishRect();
      {
        const auto & s = ref_s;
        for (unsigned int j = 0; j < g.sy; ++j) {
          for (unsigned int i = 0; i < g.sx; ++i) {
            if (s[j * g.sx + i] != 0) {ASSERT_TRUE(painted.contains(i, j)) << i << "," << j;}
          }
        }
      }
      // 병합은 painted ∩ bounds 만 본다
      ASSERT_LE(cand.lastStats().merge_visited_cells, painted.area());
      // publish 사각형 = 직전 ∪ 이번, grid 안
      {
        CellRect u = painted; u.unite(prev_painted);
        if (!resized) {
          ASSERT_EQ(pub.x0, u.x0); ASSERT_EQ(pub.xn, u.xn); ASSERT_EQ(pub.y0, u.y0); ASSERT_EQ(pub.yn, u.yn);
        }
        if (!pub.empty()) {ASSERT_LE(pub.xn, g.sx); ASSERT_LE(pub.yn, g.sy);}
      }
      if (pub.empty()) {++empty_pub_cycles;}

      // 바뀐 셀은 모두 publish 사각형 안 (크기가 바뀐 사이클은 publisher 가 full 을 보낸다)
      const auto after = snapshot(new_viz);
      if (!resized) {
        for (unsigned int j = 0; j < g.sy; ++j) {
          for (unsigned int i = 0; i < g.sx; ++i) {
            const size_t k = j * g.sx + i;
            if (before[k] != after[k]) {ASSERT_TRUE(pub.contains(i, j)) << "changed cell outside rect " << i << "," << j;}
          }
        }
      }

      // 구독자 흉내: full(크기 변경) + 사각형 update 로 grid 를 그대로 재현해야 한다
      if (resized) {
        sub = after;
        sub2 = snapshot(new2_viz);
        sub2_drop = sub2;
        sub1_drop = after;
        prev_dropped = false;
      } else {
        applyRect(new_viz, sub, pub);
        ASSERT_EQ(sub, after) << "subscriber diverged at cycle " << cyc;
        // history 2: 가끔 update 하나를 잃어도 (연속은 아님) 다음 update 로 다시 같아진다
        const bool drop = !prev_dropped && std::uniform_int_distribution<int>(0, 6)(sc.rng()) == 0;
        if (!drop) {
          applyRect(new2_viz, sub2_drop, cand2.publishRect());
          ASSERT_EQ(sub2_drop, snapshot(new2_viz)) << "history-2 subscriber not healed at cycle " << cyc;
          applyRect(new_viz, sub1_drop, pub);   // 같은 유실을 history 1 로 겪으면 (음성 대조)
          if (sub1_drop != after) {++ghost_cycles_h1;}
        }
        prev_dropped = drop;
      }

      total_cleared += cand.lastStats().cleared_cells;
      total_merge += cand.lastStats().merge_visited_cells;
      total_pub += pub.area();
      map_cells_total += static_cast<uint64_t>(g.sx) * g.sy;
    }
    // 빈 사이클이 연속 두 번 이상 있었다 → publish 사각형이 비는 경우도 거쳤다
    EXPECT_GT(empty_pub_cycles, 0);
    // history 1 로는 유실된 update 의 유령 셀이 남는 경우가 실제로 생긴다 (남은 위험의 증거)
    EXPECT_GT(ghost_cycles_h1, 0u);
    std::printf("[seed %u] cycles=%d cleared=%lu merge_visited=%lu publish_cells=%lu vs legacy memset=%lu "
                "(%.2f%%), empty_publish_cycles=%d, ghost_cycles(history1, drops)=%lu\n",
      seed, cycles, static_cast<unsigned long>(total_cleared), static_cast<unsigned long>(total_merge),
      static_cast<unsigned long>(total_pub), static_cast<unsigned long>(map_cells_total),
      100.0 * static_cast<double>(total_cleared) / static_cast<double>(map_cells_total),
      empty_pub_cycles, static_cast<unsigned long>(ghost_cycles_h1));
  }
}

// 새 방식은 직전 사각형 밖을 건드리지 않는다: 사각형 밖에 몰래 심은 값(canary)이 살아남는다.
// legacy 는 전체 memset 이라 지워진다. 크기가 바뀌면 둘 다 지워진다 (resizeMap).
TEST(AgentMaskCanvas, ClearTouchesOnlyPreviousRect)
{
  Costmap2D master(200, 100, 0.05, 0.0, 0.0, 0);
  Costmap2D v_new, v_old;
  AgentMaskCanvas cand(v_new), legacy(v_old);
  legacy.setLegacyFullClear(true);

  std::vector<geometry_msgs::msg::Point32> fp = toP32(nav2_costmap_2d::makeFootprintFromRadius(0.3));
  for (AgentMaskCanvas * c : {&cand, &legacy}) {
    c->beginCycle(master);
    multi_agent_nav2::rasterizeFootprint(fp, makePose(2.0, 2.0, 0.3), 0.0, 0.0, 0.0, *c, 254, nullptr,
      testLogger(), testClock());
  }
  const CellRect painted = cand.painted();
  ASSERT_FALSE(painted.empty());
  ASSERT_LT(painted.area(), 200u * 100u / 20u);
  // canary: 사각형 밖
  v_new.setCost(190, 90, 77);
  v_old.setCost(190, 90, 77);

  cand.beginCycle(master);
  legacy.beginCycle(master);
  EXPECT_EQ(v_new.getCost(190, 90), 77);   // 새 방식은 사각형 밖을 지우지 않았다
  EXPECT_EQ(v_old.getCost(190, 90), 0);    // legacy 는 전체를 지웠다
  EXPECT_EQ(cand.lastStats().cleared_cells, painted.area());
  for (unsigned int j = painted.y0; j < painted.yn; ++j) {
    for (unsigned int i = painted.x0; i < painted.xn; ++i) {
      ASSERT_EQ(v_new.getCost(i, j), 0);
    }
  }
  // 아무것도 안 칠한 사이클 두 번: 두 번째의 publish 사각형은 비고, 지운 셀도 0
  EXPECT_FALSE(cand.publishRect().empty());   // 직전에 칠한 것을 지운 사이클 → 사각형 있음
  cand.beginCycle(master);
  EXPECT_TRUE(cand.publishRect().empty());
  EXPECT_EQ(cand.lastStats().cleared_cells, 0u);
  // 크기 변경 → resizeMap 이 전체를 0 으로
  Costmap2D master2(220, 100, 0.05, 0.0, 0.0, 0);
  EXPECT_TRUE(cand.beginCycle(master2));
  EXPECT_EQ(v_new.getCost(190, 90), 0);
  EXPECT_TRUE(cand.publishRect().empty());
}

// CellRect 경계: intersect 의 음수·초과·빈 범위
TEST(AgentMaskCanvas, CellRectIntersectAndUnite)
{
  CellRect r; EXPECT_TRUE(r.empty()); EXPECT_EQ(r.area(), 0u);
  r.include(5, 7); EXPECT_EQ(r.area(), 1u); EXPECT_TRUE(r.contains(5, 7));
  r.include(2, 9); EXPECT_EQ(r.x0, 2u); EXPECT_EQ(r.xn, 6u); EXPECT_EQ(r.y0, 7u); EXPECT_EQ(r.yn, 10u);
  EXPECT_TRUE(r.intersect(-5, -5, 3, 8).contains(2, 7));
  EXPECT_EQ(r.intersect(-5, -5, 3, 8).area(), 1u);
  EXPECT_TRUE(r.intersect(6, 0, 100, 100).empty());
  EXPECT_TRUE(r.intersect(0, 0, -1, 100).empty());
  CellRect e; e.unite(CellRect{}); EXPECT_TRUE(e.empty());
  e.unite(r); EXPECT_EQ(e.area(), r.area());
}

// ============================================================================================
// [10-10 W1-2][리뷰] finishCycle (refresh / 강제 full) 과 publish 접착 코드의 끝에서 끝까지 시험
// ============================================================================================
namespace
{

// fork Costmap2DPublisher (amhs nav2_costmap_2d/src/costmap_2d_publisher.cpp) 의 publish 경로를 그대로 옮긴 모형.
//   updateBounds (hpp:57-63), 생성자의 사각형 초기화 (cpp:107-109), updateGridParams (cpp:123-130),
//   prepareCostmap (cpp:162-189), createCostmapUpdateMsg (cpp:212-231), publishCostmap (cpp:233-265).
//   OccupancyGrid 쪽은 빼고 raw (nav2_msgs) 쪽만 둔다. publish 는 out 에 쌓는다 (active=false 면 LifecyclePublisher 처럼 버린다).
//   saved_origin_x_ / grid_width_ 등은 fork 생성자가 초기화하지 않는다 → 시험이 "첫 geometry 와 우연히 같음" 을 고를 수 있다.
struct PubMsg
{
  std::shared_ptr<nav2_msgs::msg::Costmap> full;
  std::shared_ptr<nav2_msgs::msg::CostmapUpdate> update;
};

class ForkPublisherModel
{
public:
  ForkPublisherModel(Costmap2D * costmap, bool always_send_full_costmap)
  : costmap_(costmap), always_send_full_costmap_(always_send_full_costmap)
  {
    xn_ = yn_ = 0;
    x0_ = costmap_->getSizeInCellsX();
    y0_ = costmap_->getSizeInCellsY();
  }
  void setUninitialized(double ox, double oy, float res, unsigned w, unsigned h)
  {
    saved_origin_x_ = ox; saved_origin_y_ = oy; grid_resolution_ = res; grid_width_ = w; grid_height_ = h;
  }

  void updateBounds(unsigned int x0, unsigned int xn, unsigned int y0, unsigned int yn)
  {
    x0_ = std::min(x0, x0_);
    xn_ = std::max(xn, xn_);
    y0_ = std::min(y0, y0_);
    yn_ = std::max(yn, yn_);
  }

  void publishCostmap()
  {
    float resolution = costmap_->getResolution();
    if (always_send_full_costmap_ || grid_resolution_ != resolution ||
      grid_width_ != costmap_->getSizeInCellsX() ||
      grid_height_ != costmap_->getSizeInCellsY() ||
      saved_origin_x_ != costmap_->getOriginX() ||
      saved_origin_y_ != costmap_->getOriginY())
    {
      updateGridParams();
      if (full_subs > 0) {
        emit(PubMsg{prepareCostmap(), nullptr});
      }
    } else if (x0_ < xn_) {
      if (update_subs > 0) {
        emit(PubMsg{nullptr, createCostmapUpdateMsg()});
      }
    }
    xn_ = yn_ = 0;
    x0_ = costmap_->getSizeInCellsX();
    y0_ = costmap_->getSizeInCellsY();
  }

  bool active{true};
  int full_subs{1}, update_subs{1};
  int64_t now_ns{0};
  std::vector<PubMsg> out;
  uint64_t inactive_drops{0};

private:
  void emit(PubMsg m)
  {
    if (!active) {++inactive_drops; return;}   // rclcpp_lifecycle: "publisher is not activated" → 버려짐
    out.push_back(std::move(m));
  }
  void updateGridParams()
  {
    saved_origin_x_ = costmap_->getOriginX();
    saved_origin_y_ = costmap_->getOriginY();
    grid_resolution_ = costmap_->getResolution();
    grid_width_ = costmap_->getSizeInCellsX();
    grid_height_ = costmap_->getSizeInCellsY();
  }
  builtin_interfaces::msg::Time stampNow() const
  {
    builtin_interfaces::msg::Time t;
    t.sec = static_cast<int32_t>(now_ns / 1000000000LL);
    t.nanosec = static_cast<uint32_t>(now_ns % 1000000000LL);
    return t;
  }
  std::shared_ptr<nav2_msgs::msg::Costmap> prepareCostmap()
  {
    double resolution = costmap_->getResolution();
    auto c = std::make_shared<nav2_msgs::msg::Costmap>();
    c->header.frame_id = "map";
    c->header.stamp = stampNow();
    c->metadata.layer = "master";
    c->metadata.resolution = resolution;
    c->metadata.size_x = costmap_->getSizeInCellsX();
    c->metadata.size_y = costmap_->getSizeInCellsY();
    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    c->metadata.origin.position.x = wx - resolution / 2;
    c->metadata.origin.position.y = wy - resolution / 2;
    c->metadata.origin.orientation.w = 1.0;
    c->data.resize(c->metadata.size_x * c->metadata.size_y);
    std::memcpy(c->data.data(), costmap_->getCharMap(), c->data.size());
    return c;
  }
  std::shared_ptr<nav2_msgs::msg::CostmapUpdate> createCostmapUpdateMsg()
  {
    auto msg = std::make_shared<nav2_msgs::msg::CostmapUpdate>();
    msg->header.stamp = stampNow();
    msg->header.frame_id = "map";
    msg->x = x0_;
    msg->y = y0_;
    msg->size_x = xn_ - x0_;
    msg->size_y = yn_ - y0_;
    msg->data.resize(msg->size_x * msg->size_y);
    std::uint32_t i = 0;
    for (std::uint32_t y = y0_; y < yn_; y++) {
      for (std::uint32_t x = x0_; x < xn_; x++) {
        msg->data[i++] = costmap_->getCost(x, y);
      }
    }
    return msg;
  }

  Costmap2D * costmap_;
  bool always_send_full_costmap_;
  unsigned int x0_, xn_, y0_, yn_;
  double saved_origin_x_{-12345.0}, saved_origin_y_{-12345.0};
  float grid_resolution_{-1.f};
  unsigned int grid_width_{0}, grid_height_{0};
};

// 구독자 1: full 은 통째로, update 는 사각형만 복사 (RViz·IsGoalsOccupied 같은 일반 구독자, step 2 리뷰 전 mirror 와 같은 규칙)
struct PlainSub
{
  unsigned sx{0}, sy{0};
  std::vector<unsigned char> g;
  void onFull(const nav2_msgs::msg::Costmap & m)
  {
    sx = m.metadata.size_x; sy = m.metadata.size_y;
    g.assign(m.data.begin(), m.data.end());
  }
  void onUpdate(const nav2_msgs::msg::CostmapUpdate & u)
  {
    if (static_cast<uint64_t>(u.x) + u.size_x > sx || static_cast<uint64_t>(u.y) + u.size_y > sy) {return;}
    for (unsigned r = 0; r < u.size_y; ++r) {
      std::memcpy(&g[(static_cast<size_t>(u.y) + r) * sx + u.x], &u.data[static_cast<size_t>(r) * u.size_x], u.size_x);
    }
  }
};

}  // namespace

// finishCycle: 평소 = publishRect, refresh 주기 = 최근 두 구간 사각형 더함, requestFullRect = grid 전체 한 번, legacy = grid 전체.
TEST(AgentMaskCanvas, FinishCycleRefreshAndForcedFull)
{
  Costmap2D master(120, 80, 0.05, 0.0, 0.0, 0);
  Costmap2D viz;
  AgentMaskCanvas c(viz);
  c.setRefreshCycles(3);
  const auto fp = toP32(nav2_costmap_2d::makeFootprintFromRadius(0.2));
  auto paint = [&](double x, double y) {
      multi_agent_nav2::rasterizeFootprint(fp, makePose(x, y, 0.0), 0.0, 0.0, 0.0, c, 254, nullptr,
        testLogger(), testClock());
    };
  // cycle 1 (resize): A 칠함
  EXPECT_TRUE(c.beginCycle(master)); paint(1.0, 1.0);
  const CellRect a = c.painted();
  CellRect r = c.finishCycle();
  EXPECT_FALSE(c.lastStats().refresh_boundary);
  EXPECT_EQ(r.area(), a.area());
  // cycle 2: 아무것도 안 칠함 → 지우기 update = A
  c.beginCycle(master);
  r = c.finishCycle();
  EXPECT_EQ(r.area(), a.area());
  // cycle 3: 빈 사이클, refresh 주기 → 지난 구간에 보낸 A 를 한 번 더
  c.beginCycle(master);
  EXPECT_TRUE(c.publishRect().empty());
  r = c.finishCycle();
  EXPECT_TRUE(c.lastStats().refresh_boundary);
  EXPECT_TRUE(c.lastStats().refreshed);
  EXPECT_EQ(r.x0, a.x0); EXPECT_EQ(r.xn, a.xn); EXPECT_EQ(r.y0, a.y0); EXPECT_EQ(r.yn, a.yn);
  // cycle 4,5: 빈 사이클 → 보낼 것 없음
  for (int k = 0; k < 2; ++k) {c.beginCycle(master); EXPECT_TRUE(c.finishCycle().empty());}
  // cycle 6: refresh 주기 → 직전 구간(sent_prev) 의 A 를 한 번 더 (앞 refresh 를 잃었을 때 대비)
  c.beginCycle(master);
  r = c.finishCycle();
  EXPECT_TRUE(c.lastStats().refresh_boundary);
  EXPECT_EQ(r.area(), a.area());
  // cycle 7~9: 두 구간 지나면 더는 없다
  for (int k = 0; k < 3; ++k) {c.beginCycle(master); EXPECT_TRUE(c.finishCycle().empty()) << k;}
  // 강제 full: 한 번만 grid 전체
  c.beginCycle(master);
  c.requestFullRect();
  r = c.finishCycle();
  EXPECT_TRUE(c.lastStats().forced_full);
  EXPECT_EQ(r.x0, 0u); EXPECT_EQ(r.xn, 120u); EXPECT_EQ(r.y0, 0u); EXPECT_EQ(r.yn, 80u);
  c.beginCycle(master);
  EXPECT_TRUE(c.finishCycle().empty());
  // 강제 full 사각형은 refresh 구간에 쌓지 않는다 (다음 refresh 가 grid 전체가 되지 않게)
  for (int k = 0; k < 6; ++k) {c.beginCycle(master); EXPECT_TRUE(c.finishCycle().empty()) << k;}
  // 크기 변경: 옛 좌표의 refresh 기록은 버린다
  c.beginCycle(master); paint(5.0, 3.5); c.finishCycle();
  Costmap2D master2(60, 40, 0.05, 0.0, 0.0, 0);
  EXPECT_TRUE(c.beginCycle(master2));
  for (int k = 0; k < 6; ++k) {
    const CellRect rr = c.finishCycle();
    EXPECT_TRUE(rr.empty()) << k;
    c.beginCycle(master2);
  }
  // legacy: 언제나 grid 전체
  Costmap2D viz2;
  AgentMaskCanvas lg(viz2);
  lg.setLegacyFullClear(true);
  lg.beginCycle(master);
  r = lg.finishCycle();
  EXPECT_EQ(r.area(), 120u * 80u);
}

// 끝에서 끝까지: 무작위 agent → canvas → publishCycle(접착 코드, AgentLayer 와 같은 함수) → fork publisher 모형 →
//   (DDS 가 알리지 않는 유실 / deactivate 중 버려짐) → 구독자 (일반 복사, 실제 CostmapMirror 일반·self-contained 모드).
//   - self-contained mirror 는 메시지를 하나 받을 때마다 viz 와 같다 (유실과 무관).
//   - 일반 구독자는 유실 뒤 refresh 두 주기가 지나면 viz 와 같다 (refresh 를 연달아 두 번 잃지 않는 한).
//   - activate 뒤 강제 full 사각형은 fork publisher 의 초기화 안 된 geometry 가 첫 geometry 와 같아 첫 full 이 빠진
//     경우(옛 실행의 쓰레기를 가진 구독자)와 deactivate 중 버려진 update 를 그 사이클에 고친다.
//   - 음성 대조: refresh 0 이면 떠난 이웃의 마지막 지우기를 잃었을 때 유령 셀이 남고, 강제 full 이 없으면 쓰레기가 남는다.
TEST(AgentMaskPublish, EndToEndThroughForkPublisherModel)
{
  struct Cfg {int refresh; bool force;};
  for (const Cfg cfg : {Cfg{10, true}, Cfg{0, true}, Cfg{10, false}}) {
    for (unsigned seed : {5u, 77u}) {
      SCOPED_TRACE("refresh " + std::to_string(cfg.refresh) + " force " + std::to_string(cfg.force) +
        " seed " + std::to_string(seed));
      Scenario sc(seed);
      std::vector<Geometry> geoms = {
        {263, 171, 0.05, -3.2, 1.7},
        {301, 143, 0.05, -6.0, -2.0},   // 크기 + 원점
        {301, 143, 0.05, -5.5, -2.0},   // 원점만
      };
      const int cycles = 600;
      Costmap2D viz;
      AgentMaskCanvas canvas(viz);
      canvas.setRefreshCycles(cfg.refresh);
      ForkPublisherModel pub(&viz, false);   // AgentLayer 생성 시점: viz 는 아직 크기 0
      // 최악: 초기화 안 된 geometry 가 첫 geometry 와 같다 → 첫 publish 가 full 이 아니라 update 가지로 간다
      pub.setUninitialized(geoms[0].ox, geoms[0].oy, static_cast<float>(geoms[0].res), geoms[0].sx, geoms[0].sy);

      // 구독자는 옛 실행(cleanup→configure 전)의 grid 를 들고 있다: 같은 geometry, 여기저기 옛 agent 셀
      std::vector<unsigned char> junk(static_cast<size_t>(geoms[0].sx) * geoms[0].sy, 0);
      for (size_t k = 0; k < junk.size(); k += 997) {junk[k] = 254;}
      nav2_msgs::msg::Costmap junk_full;
      junk_full.header.stamp.sec = 0;
      junk_full.metadata.size_x = geoms[0].sx; junk_full.metadata.size_y = geoms[0].sy;
      junk_full.metadata.resolution = static_cast<float>(geoms[0].res);
      junk_full.metadata.origin.position.x = geoms[0].ox; junk_full.metadata.origin.position.y = geoms[0].oy;
      junk_full.data = junk;
      PlainSub plain; plain.onFull(junk_full);
#ifdef AGENT_MASK_TEST_WITH_MIRROR
      replan_monitor::CostmapMirror::Options sco; sco.self_contained_updates = true;
      replan_monitor::CostmapMirror m_sc(sco), m_plain;
      ASSERT_TRUE(m_sc.applyFull(junk_full).applied());
      ASSERT_TRUE(m_plain.applyFull(junk_full).applied());
      auto grab = [](const replan_monitor::CostmapMirror & m) {
          auto g = m.read();
          const size_t n = static_cast<size_t>(g->getSizeInCellsX()) * g->getSizeInCellsY();
          return std::vector<unsigned char>(g->getCharMap(), g->getCharMap() + n);
        };
#endif

      std::vector<SimAgent> agents(6);
      std::mt19937 drop_rng(seed * 31 + 7);
      int last_loss = 0;          // 마지막 유실 사이클
      int suppress_until = -1;    // 이 사이클까지 agent 없음 (떠난 뒤 조용한 구간)
      bool last_refresh_dropped = false;
      int checkpoints = 0, plain_bad = 0, msc_bad = 0, mplain_bad = 0, msc_after_msg_bad = 0;
      int forced_heals = 0, sc_msgs = 0, refresh_msgs = 0;
      bool first_cycle_plain_ok = false;   // 첫 사이클 직후 일반 구독자 == viz (옛 실행의 쓰레기가 지워졌나)
      uint64_t refresh_area = 0, regular_area = 0;
      const std::set<int> departures = {100, 330, 520};
      const std::set<int> deactivations = {150, 450};
      for (int c = 0; c < cycles; ++c) {
        const Geometry & g = geoms[c < 200 ? 0 : (c < 400 ? 1 : 2)];
        Costmap2D master(g.sx, g.sy, g.res, g.ox, g.oy, 0);
        pub.now_ns = 1000000000LL + static_cast<int64_t>(c) * 200000000LL;
        const bool departure = departures.count(c) > 0;
        if (departure) {suppress_until = c + 45;}
        if (c <= suppress_until) {
          for (auto & a : agents) {a.alive = false;}
        } else {
          sc.step(agents, master);
        }
        const auto paints = paintsFor(agents);
        canvas.beginCycle(master);
        for (const auto & p : paints) {
          multi_agent_nav2::rasterizeFootprint(p.fp, p.pose, p.iso, p.fwd, p.rear, canvas, p.cost, nullptr,
            testLogger(), testClock());
        }
        // AgentLayer::updateCosts 4) 와 같은 순서: activate 뒤면 강제 full 요청 → publishCycle
        const bool activation = (c == 0) || deactivations.count(c - 1) > 0;
        if (activation && cfg.force) {canvas.requestFullRect();}
        pub.active = deactivations.count(c) == 0;   // deactivate 가 이 사이클 publish 도중에 들어옴
        pub.out.clear();
        const CellRect sent = multi_agent_nav2::publishCycle(canvas, pub);
        if (canvas.lastStats().refresh_boundary && !sent.empty()) {++refresh_msgs; refresh_area += sent.area();}
        else if (!sent.empty()) {regular_area += sent.area();}
        if (!pub.active && !sent.empty()) {last_loss = c;}

        const auto truth = snapshot(viz);
        for (const auto & m : pub.out) {
          if (m.full) {
            plain.onFull(*m.full);
#ifdef AGENT_MASK_TEST_WITH_MIRROR
            ASSERT_TRUE(m_sc.applyFull(*m.full).applied());
            ASSERT_TRUE(m_plain.applyFull(*m.full).applied());
#endif
            continue;
          }
          // DDS 가 알리지 않는 유실: 시끄러운 구간에만, refresh 를 연달아 두 번 잃지는 않게. 떠난 사이클의 지우기는 일부러 잃는다.
          const bool noisy = (c % 60) < 30 && c > 0;
          bool drop = departure || (noisy && drop_rng() % 5 == 0);
          if (canvas.lastStats().refresh_boundary) {
            if (last_refresh_dropped) {drop = false;}
            last_refresh_dropped = drop;
          }
          if (drop) {last_loss = c; continue;}
          plain.onUpdate(*m.update);
#ifdef AGENT_MASK_TEST_WITH_MIRROR
          ASSERT_EQ(m_sc.applyUpdate(m.update), replan_monitor::CostmapMirror::UpdateResult::kApplied);
          ASSERT_EQ(m_plain.applyUpdate(m.update), replan_monitor::CostmapMirror::UpdateResult::kApplied);
          ++sc_msgs;
          if (grab(m_sc) != truth) {++msc_after_msg_bad;}
#endif
          if (activation && canvas.lastStats().forced_full && plain.g == truth) {++forced_heals;}
        }
        if (c == 0) {first_cycle_plain_ok = (plain.g == truth);}
        // 점검 시점: 마지막 유실 뒤 refresh 두 주기(+1) 가 지났다
        if (c - last_loss > 2 * 10 + 1 && c % 5 == 4) {
          ++checkpoints;
          if (plain.g != truth) {++plain_bad;}
#ifdef AGENT_MASK_TEST_WITH_MIRROR
          if (grab(m_sc) != truth) {++msc_bad;}
          if (grab(m_plain) != truth) {++mplain_bad;}
#endif
        }
      }
      std::printf("[E2E refresh=%d force=%d seed=%u] checkpoints=%d plain_bad=%d mirror_plain_bad=%d mirror_sc_bad=%d "
                  "sc_after_msg_bad=%d/%d first_cycle_plain_ok=%d forced_heals=%d inactive_drops=%lu refresh_msgs=%d "
                  "avg_refresh_area=%.0f avg_regular_area=%.0f\n",
        cfg.refresh, cfg.force ? 1 : 0, seed, checkpoints, plain_bad, mplain_bad, msc_bad, msc_after_msg_bad, sc_msgs,
        first_cycle_plain_ok ? 1 : 0,
        forced_heals, static_cast<unsigned long>(pub.inactive_drops), refresh_msgs,
        refresh_msgs ? static_cast<double>(refresh_area) / refresh_msgs : 0.0,
        static_cast<double>(regular_area) / std::max(1, cycles));
      EXPECT_GT(checkpoints, 30);
      EXPECT_GE(pub.inactive_drops, 1u);
#ifdef AGENT_MASK_TEST_WITH_MIRROR
      EXPECT_EQ(msc_after_msg_bad, 0);   // self-contained: 받은 메시지마다 정확 (유실·쓰레기와 무관)
#endif
      EXPECT_EQ(first_cycle_plain_ok, cfg.force);   // 강제 full 이 있으면 첫 사이클에 바로 맞고, 없으면 쓰레기가 남는다
      if (cfg.refresh > 0 && cfg.force) {
        EXPECT_EQ(plain_bad, 0);
        EXPECT_EQ(mplain_bad, 0);
        EXPECT_EQ(msc_bad, 0);
        EXPECT_GE(forced_heals, 1);
      } else if (cfg.refresh == 0) {
        EXPECT_GT(plain_bad, 0);   // 음성 대조: 떠난 이웃의 마지막 지우기 유실 → 유령 셀이 남는다
#ifdef AGENT_MASK_TEST_WITH_MIRROR
        EXPECT_GT(msc_bad, 0);     // self-contained 도 뒤따르는 update 가 없으면 못 고친다 → refresh 가 필요한 이유
#endif
      } else {
        // 음성 대조 (강제 full 없음): 옛 실행의 쓰레기는 위 first_cycle_plain_ok 가 본다. 그 뒤에는 agent 가 지도 전체를 돌아
        //   우연히 덮이기도 해서 (seed 에 따라 plain_bad 0 일 수 있음) 개수는 보지 않는다.
        EXPECT_EQ(msc_bad, 0);     // self-contained mirror 는 첫 update 에서 grid 를 훑어 지운다
      }
    }
  }
}

// [10-10 W1-3] S2-OPS-1/W2-1 배포 guard: update 방식인데 full topic 구독자만 있으면 (예전 path_validator) 의심.
TEST(AgentMaskPublishGuard, LegacyListenerSuspected)
{
  using multi_agent_nav2::legacyMaskListenerSuspected;
  // 새 PV: full + update 둘 다 구독 → 정상
  EXPECT_FALSE(legacyMaskListenerSuspected(false, 300.0, 120.0, 1, 1));
  // 예전 PV: full 만 → 120 s 뒤 의심
  EXPECT_TRUE(legacyMaskListenerSuspected(false, 300.0, 120.0, 1, 0));
  EXPECT_FALSE(legacyMaskListenerSuspected(false, 60.0, 120.0, 1, 0));    // activation 직후는 기다린다
  // 아무도 없음 → 경고 안 함 (PV 가 아직 안 떴다)
  EXPECT_FALSE(legacyMaskListenerSuspected(false, 300.0, 120.0, 0, 0));
  // 예전 방식 (full_mask_publish=true) 은 해당 없음
  EXPECT_FALSE(legacyMaskListenerSuspected(true, 300.0, 120.0, 1, 0));
}

#ifdef AGENT_MASK_TEST_WITH_MIRROR
// [10-10 W1-3] 현장 시작 순서 E2E: Nav2 (AgentLayer) 가 먼저 activation 되고 path_validator 는 60 s 뒤에 붙는다.
//   - activation 때 구독자가 없어 첫 full·강제 full 사각형 모두 나가지 않는다 (fork publisher 는 구독자 수를 먼저 본다).
//   - PV 는 mask 를 global geometry (= master geometry) 로 seed 하고, 붙은 뒤 오는 update 를 바로 적용한다.
//   - 이웃이 있으면 첫 update 까지 "모름", 그 뒤 받은 update 마다 mirror == viz.
//   - 그 다음 PV 가 죽었다가 (구독자 0, 5 s) respawn: 그사이 이웃이 모두 떠나 보낼 update 가 없다. 옛 latched update 를
//     적용하면 유령 몸체가 남고 (음성 대조), 구독 직후 grace 보다 오래된 latched update 를 버리면 0 grid 가 맞다.
TEST(AgentMaskPublish, FieldOrderLateJoinSeededAndRespawn)
{
  const Costmap2D master(240, 160, 0.05, -2.0, 1.0, 0);
  Costmap2D viz;
  AgentMaskCanvas canvas(viz);
  canvas.setRefreshCycles(10);
  ForkPublisherModel pub(&viz, false);
  pub.full_subs = 0; pub.update_subs = 0;   // PV 아직 없음
  Scenario sc(2026);
  std::vector<SimAgent> agents(4);
  const auto fp = toP32(nav2_costmap_2d::makeFootprintFromRadius(0.3));
  for (size_t k = 0; k < agents.size(); ++k) {
    auto & a = agents[k];
    sc.spawn(a, master); a.fp = fp; a.path_n = 3; a.iso = 0.05;
    a.x = 0.5 + 2.2 * static_cast<double>(k); a.y = 2.5 + 1.3 * static_cast<double>(k);   // 지도 안 (x -2~10, y 1~9)
  }

  replan_monitor::CostmapSignature gsig;   // global mirror geometry (topic full 식 origin)
  gsig.size_x = master.getSizeInCellsX(); gsig.size_y = master.getSizeInCellsY();
  gsig.resolution = master.getResolution(); gsig.origin_x = master.getOriginX(); gsig.origin_y = master.getOriginY();

  replan_monitor::CostmapMirror::Options sco; sco.self_contained_updates = true;
  auto mirror = std::make_unique<replan_monitor::CostmapMirror>(sco);
  std::shared_ptr<nav2_msgs::msg::CostmapUpdate> latched;   // publisher 의 transient_local KeepLast(1)
  int latched_cycle = -1;
  const int join = 300, die = 600, respawn = 625, leave = 605, end = 700;
  int unknown_cycles_after_join = 0, bad_after_first_update = 0, msgs = 0, fulls_seen = 0;
  bool first_update_seen = false;
  int ghost_rule = -1, ghost_norule = -1;
  for (int c = 0; c < end; ++c) {
    pub.now_ns = 1000000000LL + static_cast<int64_t>(c) * 200000000LL;
    if (c >= leave && c < respawn + 40) {
      for (auto & a : agents) {a.alive = false;}   // PV 가 죽어 있는 동안 이웃이 모두 떠난다
    } else if (c < leave) {
      const double dx = ((c / 100) % 2 == 0) ? 0.02 : -0.02;   // 왕복 (지도 밖으로 나가지 않게)
      for (auto & a : agents) {a.x += dx; if (!a.alive) {a.alive = true;}}
    }
    canvas.beginCycle(master);
    for (const auto & p : paintsFor(agents)) {
      multi_agent_nav2::rasterizeFootprint(p.fp, p.pose, p.iso, p.fwd, p.rear, canvas, p.cost, nullptr,
        testLogger(), testClock());
    }
    if (c == 0) {canvas.requestFullRect();}   // activate 뒤 첫 사이클
    const bool pv_alive = (c >= join && c < die) || c >= respawn;
    pub.full_subs = pub.update_subs = pv_alive ? 1 : 0;
    pub.out.clear();
    multi_agent_nav2::publishCycle(canvas, pub);
    const auto truth = snapshot(viz);
    for (const auto & m : pub.out) {
      if (m.full) {++fulls_seen; continue;}
      latched = m.update;
      latched_cycle = c;
    }
    if (c == join) {
      // PV 시작: global grid 는 latched global full 로 바로 있다 → mask seed
      ASSERT_TRUE(mirror->seedZero(gsig).seeded);
    }
    if (c == respawn) {
      // respawn: 새 mirror. 구독 직후 latched update (die 직전 것, 5 s 전) 가 먼저 온다.
      // latched update 의 나이 = 죽기 전 마지막 update 부터 respawn(5 s) + 기동까지 (현장: respawn_delay 5.0 + 기동 > grace 5.0)
      const double age = (respawn - latched_cycle) * 0.2;
      ASSERT_GT(age, 5.0);
      for (bool rule : {true, false}) {
        replan_monitor::CostmapMirror m2(sco);
        ASSERT_TRUE(m2.seedZero(gsig).seeded);
        if (!(rule && replan_monitor::dropLatchedMaskUpdate(true, false, 0.2, 20.0, age, 5.0))) {
          m2.applyUpdate(latched);
        }
        // 이웃이 없으니 publisher 는 보낼 것이 없다 (refresh 도 두 구간 지나 비었다)
        const bool ghost = [&] {
            auto g = m2.read();
            return std::vector<unsigned char>(g->getCharMap(),
              g->getCharMap() + static_cast<size_t>(g->getSizeInCellsX()) * g->getSizeInCellsY()) != truth;
          }();
        (rule ? ghost_rule : ghost_norule) = ghost ? 1 : 0;
      }
      EXPECT_TRUE(pub.out.empty()) << "respawn 직후 보낼 update 가 없어야 음성 대조가 뜻이 있다";
    }
    if (c >= join && c < die) {
      for (const auto & m : pub.out) {
        if (m.update) {
          ASSERT_EQ(mirror->applyUpdate(m.update), replan_monitor::CostmapMirror::UpdateResult::kApplied);
          first_update_seen = true;
          ++msgs;
        }
      }
      const bool neighbours = true;
      const auto mode = replan_monitor::decideAgentMaskMode(true, mirror->hasGrid(), mirror->contentKnown(), neighbours);
      if (mode == replan_monitor::AgentMaskMode::kUnknown) {++unknown_cycles_after_join;}
      if (first_update_seen) {
        auto g = mirror->read();
        const std::vector<unsigned char> mine(g->getCharMap(),
          g->getCharMap() + static_cast<size_t>(g->getSizeInCellsX()) * g->getSizeInCellsY());
        if (mine != truth) {++bad_after_first_update;}
      }
    }
  }
  std::printf("[FieldOrder E2E] fulls_seen=%d msgs=%d unknown_cycles_after_join=%d bad_after_first_update=%d "
              "respawn ghost(rule)=%d ghost(no rule)=%d\n",
    fulls_seen, msgs, unknown_cycles_after_join, bad_after_first_update, ghost_rule, ghost_norule);
  EXPECT_EQ(fulls_seen, 0);                 // 현장 순서: mask full 은 한 번도 PV 에 가지 않는다
  EXPECT_GT(msgs, 100);
  EXPECT_LE(unknown_cycles_after_join, 0);  // 붙은 사이클에 바로 update 가 온다 (이웃이 칠해져 있으면)
  EXPECT_EQ(bad_after_first_update, 0);
  EXPECT_EQ(ghost_rule, 0);
  EXPECT_EQ(ghost_norule, 1);               // 음성 대조
}
#endif

