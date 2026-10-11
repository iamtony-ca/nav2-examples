// [10-10 W1-2] AgentLayer 의 viz/mask grid(viz_costmap_) 관리.
//
//   예전 updateCosts 는 매 사이클 master 크기의 viz grid 전체를 memset 하고, 다 칠한 뒤 grid 전체를
//   bounds 로 잡아 always_send_full=true publisher 로 보냈다 (2e8 cells 지도면 사이클마다 200 MB).
//   이 클래스는 "실제로 칠한 셀" 의 사각형을 기록해 두었다가 다음 사이클에는 그 사각형만 지운다.
//
//   불변식: grid 의 0 이 아닌 셀 ⊆ 직전 사이클에 칠한 사각형 (painted_).
//     - grid 에 값을 쓰는 곳은 paintMax() 하나뿐이고, 쓸 때마다 painted_ 를 넓힌다.
//     - resizeMap() 은 grid 전체를 default(0) 로 채운다 (AgentLayer 가 setDefaultValue(0)).
//     - 그래서 beginCycle() 에서 직전 사각형만 0 으로 만들면 grid 전체가 0 이 된다 = 예전 memset 결과와 같다.
//   enable/disable, reset(), deactivate/activate, 파라미터 변경(ROI·dilation 등)은 grid 에 쓰지 않으므로
//   불변식을 깨지 않는다 (사각형은 다음 beginCycle 까지 그대로 남아 그때 지워진다).
//
//   publishRect() = 이번 사각형 ∪ 직전 history 개 사이클의 사각형. 이번 사이클에 값이 바뀐 셀은 모두
//   (직전 사각형에서 지운 셀 ∪ 이번에 칠한 셀) 안에 있으므로 history ≥ 1 이면 update 가 바뀐 셀을 모두 덮는다.
//   history 를 N 으로 키우면 연속 N-1 개의 update 를 잃어도 다음 update 가 유령 셀을 지운다.
//
//   legacy(full_mask_publish=true) 이면 예전과 똑같이 memset 전체 + bounds 전체 병합을 한다 (A/B 용).
//
//   [10-10 W1-2][리뷰] publisher 에 넘길 사각형은 finishCycle() 이 정한다 (layer 는 publishCycle() 로 넘긴다).
//     - 평소: publishRect().
//     - refresh: refresh_cycles 사이클마다 "최근 두 구간에 보낸 사각형 전체" 를 한 번 더 넣는다. 지운 사각형이 담긴
//       update 를 DDS 가 알리지 않고 잃었는데 그 뒤로 보낼 것이 없으면(떠난 이웃의 마지막 지우기) 구독자에 유령 셀이
//       영원히 남는다. 이 refresh 가 그 셀을 refresh_cycles 사이클 안에 다시 덮는다. 비용은 O(ROI + 이동 거리).
//     - requestFullRect(): 다음 한 사이클은 grid 전체 사각형 (activate 뒤 한 번). deactivate 중 inactive
//       LifecyclePublisher 에 버려진 update 와, fork publisher 의 초기화 안 된 geometry 멤버 때문에 첫 full 이 빠지는
//       경우를 덮는다. 비용은 activation 당 grid 전체 update 1 개 (예전 방식은 매 사이클 full).
//     어느 경우든 보내는 사각형 ⊇ painted_ 이므로 "0 이 아닌 셀 ⊆ 보낸 사각형" 이 모든 update 에서 성립한다
//     (구독자가 update 하나만으로 mask 전체를 복원할 수 있다 — path_validator 의 self-contained mirror 가 이것을 쓴다).

#ifndef MULTI_AGENT_NAV2__AGENT_MASK_CANVAS_HPP_
#define MULTI_AGENT_NAV2__AGENT_MASK_CANVAS_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <utility>
#include <vector>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/cost_values.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

namespace multi_agent_nav2
{

// [10-10 W1-2] 셀 사각형. half-open [x0, xn) × [y0, yn) — Costmap2DPublisher::updateBounds(x0, xn, y0, yn) 와 같은 규약.
struct CellRect
{
  unsigned int x0{0}, xn{0}, y0{0}, yn{0};

  bool empty() const noexcept {return x0 >= xn || y0 >= yn;}
  uint64_t area() const noexcept
  {
    return empty() ? 0u : static_cast<uint64_t>(xn - x0) * static_cast<uint64_t>(yn - y0);
  }
  bool contains(unsigned int i, unsigned int j) const noexcept
  {
    return i >= x0 && i < xn && j >= y0 && j < yn;
  }
  void include(unsigned int i, unsigned int j) noexcept
  {
    if (empty()) {
      x0 = i; xn = i + 1; y0 = j; yn = j + 1;
      return;
    }
    x0 = std::min(x0, i); xn = std::max(xn, i + 1);
    y0 = std::min(y0, j); yn = std::max(yn, j + 1);
  }
  void unite(const CellRect & o) noexcept
  {
    if (o.empty()) {return;}
    if (empty()) {*this = o; return;}
    x0 = std::min(x0, o.x0); xn = std::max(xn, o.xn);
    y0 = std::min(y0, o.y0); yn = std::max(yn, o.yn);
  }
  // [gx0, gxn) × [gy0, gyn) 와의 교집합 (int 범위, 음수/빈 범위 허용)
  CellRect intersect(int gx0, int gy0, int gxn, int gyn) const noexcept
  {
    CellRect r;
    if (empty() || gx0 >= gxn || gy0 >= gyn) {return r;}
    const int64_t ax0 = std::max<int64_t>(x0, gx0), axn = std::min<int64_t>(xn, gxn);
    const int64_t ay0 = std::max<int64_t>(y0, gy0), ayn = std::min<int64_t>(yn, gyn);
    if (ax0 >= axn || ay0 >= ayn) {return r;}
    r.x0 = static_cast<unsigned int>(ax0); r.xn = static_cast<unsigned int>(axn);
    r.y0 = static_cast<unsigned int>(ay0); r.yn = static_cast<unsigned int>(ayn);
    return r;
  }
};

class AgentMaskCanvas
{
public:
  // 시험·진단용 사이클 작업량
  struct CycleStats
  {
    bool resized{false};              // 이번 beginCycle 에서 크기/원점이 바뀌어 resizeMap 했다
    uint64_t cleared_cells{0};        // beginCycle 에서 0 으로 만든 셀 수 (legacy 면 grid 전체)
    uint64_t merge_visited_cells{0};  // mergeInto 가 들여다본 셀 수
    bool refresh_boundary{false};     // [10-10 W1-2][리뷰] 이번 finishCycle 이 refresh 주기에 걸렸다
    bool refreshed{false};            // [10-10 W1-2][리뷰] 이번 finishCycle 이 refresh 사각형을 더했다 (사각형이 커졌다)
    bool forced_full{false};          // [10-10 W1-2][리뷰] 이번 finishCycle 이 grid 전체 사각형을 돌려줬다
  };

  // [10-10 W1-2][리뷰] grid 의 default 값을 0 으로 둔다 (불변식: resizeMap 이 grid 를 0 으로 채움). Costmap2D() 기본 생성자는
  //   default_value_ 를 초기화하지 않는다 — AgentLayer 는 onInitialize 에서 setDefaultValue(0) 을 하지만 canvas 가 스스로 지킨다.
  //   (시험은 이 초기화 없이 쓰레기 값에 기대고 있었다: 클래스 크기가 바뀌자 resizeMap 이 255 로 채워 실패했다.)
  explicit AgentMaskCanvas(nav2_costmap_2d::Costmap2D & grid)
  : grid_(grid) {grid_.setDefaultValue(0);}

  AgentMaskCanvas(const AgentMaskCanvas &) = delete;
  AgentMaskCanvas & operator=(const AgentMaskCanvas &) = delete;

  // legacy=true: 예전 동작 (매 사이클 grid 전체 memset, bounds 전체 병합). 첫 사이클 전에만 바꾼다.
  void setLegacyFullClear(bool legacy) {legacy_ = legacy;}
  bool legacy() const {return legacy_;}
  // publishRect 에 넣을 직전 사이클 사각형 수 (최소 1). 첫 사이클 전에만 바꾼다.
  void setPublishHistory(size_t n) {history_ = std::max<size_t>(1, n);}
  size_t publishHistory() const {return history_;}
  // [10-10 W1-2][리뷰] refresh 주기 (사이클 수, 0 = 끔). 첫 사이클 전에만 바꾼다.
  void setRefreshCycles(int n) {refresh_cycles_ = std::max(0, n);}
  int refreshCycles() const {return refresh_cycles_;}
  // [10-10 W1-2][리뷰] 다음 finishCycle 이 grid 전체 사각형을 돌려주게 한다 (map update thread 에서 부를 것).
  void requestFullRect() {force_full_ = true;}

  // 1) master 와 크기/해상도/원점을 맞추고 2) 직전 사이클에 칠한 셀을 지운다.
  //    grid mutex 는 이 안에서만 쥔다 (예전 코드의 1) 블록과 같은 범위).
  //    반환: 이번에 resizeMap 했는지 (= publisher 가 full 을 보낼 사이클).
  bool beginCycle(const nav2_costmap_2d::Costmap2D & master);

  // 예전 fillFootprintAt 의 쓰기 규칙 그대로: NO_INFORMATION 은 0 으로 보고, 더 큰 값만 쓴다.
  // 실제로 쓴 셀만 painted_ 에 넣는다.
  inline void paintMax(unsigned int i, unsigned int j, int cand)
  {
    const unsigned char old_raw = grid_.getCost(i, j);
    const int old = (old_raw == nav2_costmap_2d::NO_INFORMATION) ? 0 : static_cast<int>(old_raw);
    if (cand > old) {
      grid_.setCost(i, j, static_cast<unsigned char>(cand));
      painted_.include(i, j);
    }
  }

  // 예전 3) 병합: bounds [min_i,max_i)×[min_j,max_j) 안에서 grid 의 0 이 아닌 셀을 master 에 max 로 얹는다.
  // 새 방식은 bounds ∩ painted_ 만 본다 (밖은 불변식상 모두 0 이라 결과가 같다).
  void mergeInto(nav2_costmap_2d::Costmap2D & master, int min_i, int min_j, int max_i, int max_j);

  // 이번 사이클에 칠한 사각형
  const CellRect & painted() const {return painted_;}
  // 직전 사이클에 칠한 사각형 (이번 beginCycle 에서 지운 사각형). 없으면 빈 사각형.
  CellRect previous() const {return past_.empty() ? CellRect{} : past_.front();}
  // publisher 에 줄 사각형 = painted_ ∪ 직전 history_ 개 사이클 사각형
  CellRect publishRect() const;
  // [10-10 W1-2][리뷰] 칠하기가 끝난 뒤 사이클마다 한 번 부른다. 이번에 publisher 에 넘길 사각형을 돌려준다 (비면 넘기지
  //   않는다). = publishRect() (+ refresh 사이클이면 최근 두 구간 사각형, + requestFullRect 뒤면 grid 전체).
  //   legacy 면 예전처럼 grid 전체.
  CellRect finishCycle();

  const CycleStats & lastStats() const {return stats_;}
  nav2_costmap_2d::Costmap2D & grid() {return grid_;}
  const nav2_costmap_2d::Costmap2D & grid() const {return grid_;}

private:
  nav2_costmap_2d::Costmap2D & grid_;
  bool legacy_{false};
  size_t history_{1};
  CellRect painted_;
  std::deque<CellRect> past_;   // 직전 사이클들의 painted_, 최신이 앞. 크기 ≤ history_
  CycleStats stats_;
  // [10-10 W1-2][리뷰] refresh / 강제 full 사각형
  int refresh_cycles_{0};
  int cycles_since_refresh_{0};
  CellRect sent_cur_;    // 이번 refresh 구간에 보낸 사각형의 합
  CellRect sent_prev_;   // 직전 refresh 구간에 보낸 사각형의 합
  bool force_full_{false};
};

// [10-10 W1-2][리뷰] updateCosts 4) 의 publish 접착 코드. AgentLayer 는 실제 Costmap2DPublisher 로, 시험은 같은 동작의
//   모형으로 부른다 (updateBounds 인자 순서 x0, xn, y0, yn 과 "빈 사각형이면 updateBounds 를 부르지 않는다" 를 시험이 본다).
//   publishCostmap 은 사각형이 비어도 부른다: 크기/원점이 바뀐 사이클이면 publisher 가 bounds 와 무관하게 full 을 보낸다.
template<class PublisherT>
CellRect publishCycle(AgentMaskCanvas & canvas, PublisherT & pub)
{
  const CellRect r = canvas.finishCycle();
  if (!r.empty()) {
    pub.updateBounds(r.x0, r.xn, r.y0, r.yn);
  }
  pub.publishCostmap();
  return r;
}

// [10-10 W1-3] S2-OPS-1/W2-1 배포 guard: full_mask_publish=false (update 만 보냄) 인데 activation 뒤 grace 가 지나도
//   full topic(<topic>_raw) 구독자는 있고 update topic(<topic>_raw_updates) 구독자는 없으면, full 만 받는 예전 소비자
//   (amhs·pv_w1 path_validator, IsGoalsOccupiedAgentCondition 등) 가 듣고 있는 것이다. 그 소비자의 mask 는 얼어붙는다
//   (현장 순서면 null). layer 는 WARN 만 낸다 (동작은 바꾸지 않는다).
inline bool legacyMaskListenerSuspected(
  bool full_mask_publish, double since_activate_sec, double grace_sec,
  size_t raw_full_subscribers, size_t raw_update_subscribers)
{
  return !full_mask_publish && since_activate_sec >= grace_sec &&
         raw_full_subscribers > 0 && raw_update_subscribers == 0;
}

// [10-10 W1-2] 예전 agent_layer.cpp 의 static dilateFootprintDirectional 를 그대로 옮겼다 (시험에서 같은 코드를 쓰려고).
std::vector<geometry_msgs::msg::Point>
dilateFootprintDirectional(
  const std::vector<geometry_msgs::msg::Point32> & in,
  double iso_dilate_m,
  double forward_len_m,
  double rear_len_m,
  rclcpp::Logger logger,
  rclcpp::Clock::SharedPtr clock);

// [10-10 W1-2] 예전 AgentLayer::fillFootprintAt 의 래스터화 부분을 그대로 옮겼다. grid 쓰기만 canvas.paintMax 로 바꿨다.
// 반환: 회전·이동한 다각형의 world bbox (AgentLayer 가 touch_* 갱신에 쓴다).
struct WorldBox
{
  double minx{1e9}, miny{1e9}, maxx{-1e9}, maxy{-1e9};
};

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
  const rclcpp::Clock::SharedPtr & clock);

}  // namespace multi_agent_nav2

#endif  // MULTI_AGENT_NAV2__AGENT_MASK_CANVAS_HPP_
