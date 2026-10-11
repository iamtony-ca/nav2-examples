// [10-10 W1-2] AgentMaskCanvas 구현. 설명은 agent_mask_canvas.hpp 참고.

#include "multi_agent_nav2/agent_mask_canvas.hpp"

#include <cmath>
#include <cstring>
#include <mutex>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>   // tf2::getYaw(geometry_msgs Quaternion) 의 fromMsg

namespace multi_agent_nav2
{

bool AgentMaskCanvas::beginCycle(const nav2_costmap_2d::Costmap2D & master)
{
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(grid_.getMutex()));
  stats_ = CycleStats{};

  // 예전 코드와 같은 조건으로 크기/원점 동기화
  if (grid_.getSizeInCellsX() != master.getSizeInCellsX() ||
    grid_.getSizeInCellsY() != master.getSizeInCellsY() ||
    grid_.getResolution() != master.getResolution() ||
    grid_.getOriginX() != master.getOriginX() ||
    grid_.getOriginY() != master.getOriginY())
  {
    grid_.resizeMap(
      master.getSizeInCellsX(),
      master.getSizeInCellsY(),
      master.getResolution(),
      master.getOriginX(),
      master.getOriginY());
    stats_.resized = true;
  }

  if (legacy_) {
    // 예전 그대로: grid 전체 memset
    unsigned char * char_map = grid_.getCharMap();
    const unsigned int n = grid_.getSizeInCellsX() * grid_.getSizeInCellsY();
    std::memset(char_map, 0, n * sizeof(unsigned char));
    stats_.cleared_cells = n;
    past_.clear();
    painted_ = CellRect{};
    return stats_.resized;
  }

  if (stats_.resized) {
    // resizeMap 이 grid 전체를 default(0) 로 채웠다. 옛 좌표의 사각형은 더 이상 뜻이 없다.
    past_.clear();
    painted_ = CellRect{};
    return true;
  }

  // 직전 사이클 사각형을 기록하고 그 사각형만 0 으로 만든다 (불변식: 0 이 아닌 셀은 모두 그 안에 있다).
  past_.push_front(painted_);
  while (past_.size() > history_) {
    past_.pop_back();
  }
  const CellRect & prev = past_.front();
  if (!prev.empty()) {
    unsigned char * char_map = grid_.getCharMap();
    const size_t sx = grid_.getSizeInCellsX();
    const size_t w = prev.xn - prev.x0;
    for (unsigned int j = prev.y0; j < prev.yn; ++j) {
      std::memset(char_map + static_cast<size_t>(j) * sx + prev.x0, 0, w);
    }
    stats_.cleared_cells = prev.area();
  }
  painted_ = CellRect{};
  return false;
}

void AgentMaskCanvas::mergeInto(
  nav2_costmap_2d::Costmap2D & master, int min_i, int min_j, int max_i, int max_j)
{
  // 예전 3) 과 같은 bounds 계산
  const int gx_min = std::max(0, min_i);
  const int gy_min = std::max(0, min_j);
  const int gx_max = std::min<int>(master.getSizeInCellsX(), max_i);
  const int gy_max = std::min<int>(master.getSizeInCellsY(), max_j);

  int x0 = gx_min, y0 = gy_min, xn = gx_max, yn = gy_max;
  if (!legacy_) {
    // bounds ∩ painted_. painted_ 밖은 불변식상 0 이라 예전 루프에서도 continue 로 넘어가던 셀이다.
    const CellRect r = painted_.intersect(gx_min, gy_min, gx_max, gy_max);
    if (r.empty()) {
      stats_.merge_visited_cells = 0;
      return;
    }
    x0 = static_cast<int>(r.x0); xn = static_cast<int>(r.xn);
    y0 = static_cast<int>(r.y0); yn = static_cast<int>(r.yn);
  }

  uint64_t visited = 0;
  for (int j = y0; j < yn; ++j) {
    for (int i = x0; i < xn; ++i) {
      ++visited;
      const unsigned char our = grid_.getCost(i, j);
      if (our == 0) {continue;}  // 우리가 안 그린 셀은 master 그대로 둠

      const unsigned char master_raw = master.getCost(i, j);
      const int master_val = (master_raw == nav2_costmap_2d::NO_INFORMATION) ?
        0 : static_cast<int>(master_raw);
      if (static_cast<int>(our) > master_val) {
        master.setCost(i, j, our);
      }
    }
  }
  stats_.merge_visited_cells = visited;
}

CellRect AgentMaskCanvas::publishRect() const
{
  CellRect r = painted_;
  for (const auto & p : past_) {
    r.unite(p);
  }
  return r;
}

// [10-10 W1-2][리뷰] 설명은 agent_mask_canvas.hpp 머리 주석 참고.
CellRect AgentMaskCanvas::finishCycle()
{
  CellRect full;
  full.x0 = 0; full.xn = grid_.getSizeInCellsX();
  full.y0 = 0; full.yn = grid_.getSizeInCellsY();
  if (legacy_) {
    force_full_ = false;
    return full;   // 예전 그대로 updateBounds(0..size)
  }
  if (stats_.resized) {
    // 옛 좌표의 사각형은 뜻이 없다. 이 사이클은 publisher 가 full 을 보낸다.
    sent_cur_ = CellRect{};
    sent_prev_ = CellRect{};
    cycles_since_refresh_ = 0;
  }
  CellRect r = publishRect();
  sent_cur_.unite(r);
  if (refresh_cycles_ > 0 && ++cycles_since_refresh_ >= refresh_cycles_) {
    cycles_since_refresh_ = 0;
    stats_.refresh_boundary = true;
    const CellRect before = r;
    r.unite(sent_cur_);
    r.unite(sent_prev_);
    stats_.refreshed = !(r.x0 == before.x0 && r.xn == before.xn && r.y0 == before.y0 && r.yn == before.yn);
    // 이번 구간에 보낸 것은 다음 refresh 에서 한 번 더 덮는다 (이 refresh update 자체를 잃는 경우 대비).
    sent_prev_ = sent_cur_;
    sent_cur_ = CellRect{};
  }
  if (force_full_) {
    force_full_ = false;
    stats_.forced_full = true;
    return full.empty() ? r : full;
  }
  return r;
}

// ========================================================================
// 다각형 꼬임(Bow-tie) 방지를 위한 안전한 음수 팽창 로직
// [10-10 W1-2] agent_layer.cpp 에서 그대로 옮김 (static → namespace 함수)
// ========================================================================
std::vector<geometry_msgs::msg::Point>
dilateFootprintDirectional(
  const std::vector<geometry_msgs::msg::Point32> & in,
  double iso_dilate_m,
  double forward_len_m,
  double rear_len_m,
  rclcpp::Logger logger,
  rclcpp::Clock::SharedPtr clock)
{
  std::vector<geometry_msgs::msg::Point> out; out.reserve(in.size());
  if (in.empty()) {return out;}

  double cx = 0, cy = 0;
  for (auto & p : in) {cx += p.x; cy += p.y;}
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
    double n = std::hypot(vx, vy); if (n < 1e-6) {n = 1.0;}

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

// [10-10 W1-2] 예전 AgentLayer::fillFootprintAt 본문 (touch_* 갱신 제외). 계산 순서·조건은 그대로다.
WorldBox rasterizeFootprint(
  const std::vector<geometry_msgs::msg::Point32> & footprint,
  const geometry_msgs::msg::Pose & pose,
  double extra_dilation_m,
  double forward_len_m,
  double rear_len_m,
  AgentMaskCanvas & canvas,
  unsigned char cost,
  std::vector<std::pair<unsigned int, unsigned int>> * meta_hits,
  const rclcpp::Logger & logger,
  const rclcpp::Clock::SharedPtr & clock)
{
  nav2_costmap_2d::Costmap2D * grid = &canvas.grid();

  auto poly = dilateFootprintDirectional(
    footprint, extra_dilation_m, forward_len_m, rear_len_m,
    logger, clock);

  const double yaw = tf2::getYaw(pose.orientation);
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (auto & p : poly) {
    const double x = p.x, y = p.y;
    p.x = pose.position.x + c * x - s * y;
    p.y = pose.position.y + s * x + c * y;
  }

  WorldBox box;
  for (auto & p : poly) {
    box.minx = std::min(box.minx, p.x);
    box.miny = std::min(box.miny, p.y);
    box.maxx = std::max(box.maxx, p.x);
    box.maxy = std::max(box.maxy, p.y);
  }

  int min_i, min_j, max_i, max_j;
  grid->worldToMapEnforceBounds(box.minx, box.miny, min_i, min_j);
  grid->worldToMapEnforceBounds(box.maxx, box.maxy, max_i, max_j);

  for (int j = min_j; j <= max_j; ++j) {
    for (int i = min_i; i <= max_i; ++i) {
      double wx, wy; grid->mapToWorld(i, j, wx, wy);

      bool inside = false;
      const size_t n = poly.size();
      for (size_t k = 0, h = n - 1; k < n; h = k++) {
        const double xi = poly[k].x, yi = poly[k].y;
        const double xh = poly[h].x, yh = poly[h].y;

        const bool hit = ((yi > wy) != (yh > wy)) &&
          (wx < (xh - xi) * (wy - yi) / (yh - yi) + xi);
        if (hit) {
          inside = !inside;
        }
      }

      if (inside) {
        // [10-10 W1-2] 예전: getCost → (NO_INFORMATION 이면 0) → cand > old 일 때 setCost. 같은 규칙을 paintMax 가 하고
        //   실제로 쓴 셀만 사각형에 넣는다.
        canvas.paintMax(static_cast<unsigned int>(i), static_cast<unsigned int>(j), static_cast<int>(cost));
        if (meta_hits) {
          meta_hits->emplace_back(
            static_cast<unsigned int>(i), static_cast<unsigned int>(j));
        }
      }
    }
  }
  return box;
}

}  // namespace multi_agent_nav2
