// [10-10 W1-5] FastClearLayer 구현. 설계는 fast_clear_layer.hpp 머리말과 pv_w5_ws/W1_STEP5.md 참고.
#include "ammr_costmap_diff_publisher/fast_clear_layer.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <utility>

#include "nav2_costmap_2d/costmap_filters/keepout_filter.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/plugin_container_layer.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace ammr_costmap_diff_publisher
{

using namespace std::chrono_literals;

void CellBox::merge(const CellBox & o)
{
  if (!o.valid()) {
    return;
  }
  if (!valid()) {
    *this = o;
    return;
  }
  x0 = std::min(x0, o.x0);
  y0 = std::min(y0, o.y0);
  x1 = std::max(x1, o.x1);
  y1 = std::max(y1, o.y1);
}

namespace
{
inline double msSince(std::chrono::steady_clock::time_point t)
{
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count();
}

// 행 하나가 전부 default 인가 (8 byte 단위)
inline bool rowAllDefault(const unsigned char * row, unsigned int n, uint64_t pat8, unsigned char dv)
{
  unsigned int i = 0;
  for (; i + 8 <= n; i += 8) {
    uint64_t w;
    std::memcpy(&w, row + i, 8);
    if (w != pat8) {
      return false;
    }
  }
  for (; i < n; ++i) {
    if (row[i] != dv) {
      return false;
    }
  }
  return true;
}
}  // namespace

CellBox scanNonDefaultExtent(
  const unsigned char * data, unsigned int size_x, unsigned int size_y, unsigned char default_value)
{
  CellBox box;
  if (!data || size_x == 0 || size_y == 0) {
    return box;
  }
  uint64_t pat8 = 0;
  std::memset(&pat8, default_value, sizeof(pat8));
  int min_x = std::numeric_limits<int>::max(), max_x = -1;
  int min_y = -1, max_y = -1;
  for (unsigned int y = 0; y < size_y; ++y) {
    const unsigned char * row = data + size_t(y) * size_x;
    if (rowAllDefault(row, size_x, pat8, default_value)) {
      continue;
    }
    if (min_y < 0) {
      min_y = static_cast<int>(y);
    }
    max_y = static_cast<int>(y);
    // 이 행의 첫 / 마지막 non-default 셀 (이미 찾은 범위 밖만 보면 된다)
    int first = -1;
    const int lim_l = std::min<int>(min_x, static_cast<int>(size_x));
    for (int x = 0; x < lim_l; ++x) {
      if (row[x] != default_value) {
        first = x;
        break;
      }
    }
    if (first >= 0) {
      min_x = first;
    }
    for (int x = static_cast<int>(size_x) - 1; x > max_x; --x) {
      if (row[x] != default_value) {
        max_x = x;
        break;
      }
    }
  }
  if (min_y < 0 || max_x < 0) {
    return box;
  }
  box.x0 = min_x;
  box.x1 = max_x + 1;
  box.y0 = min_y;
  box.y1 = max_y + 1;
  return box;
}

FastClearLayer::~FastClearLayer()
{
  active_.store(false);
  srv_running_.store(false);
  if (srv_executor_) {
    srv_executor_->cancel();
  }
  if (srv_thread_.joinable()) {
    srv_thread_.join();
  }
  if (srv_executor_ && srv_cb_group_) {
    try {
      srv_executor_->remove_callback_group(srv_cb_group_);
    } catch (...) {
    }
  }
  service_.reset();
  srv_executor_.reset();
}

void FastClearLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"[10-10 W1-5] FastClearLayer: node lock 실패"};
  }
  declareParameter("enabled", rclcpp::ParameterValue(true));
  // inflation layer 뒤 여유와 별도로 더 넓히고 싶을 때 (m). 기본 0.
  declareParameter("extra_margin_m", rclcpp::ParameterValue(0.0));
  node->get_parameter(getFullName("enabled"), enabled_);
  node->get_parameter(getFullName("extra_margin_m"), extra_margin_m_);
  extra_margin_m_ = std::max(0.0, extra_margin_m_);

  // 서비스: costmap 노드 기준 상대 이름 → /global_costmap/<name>/clear_entirely
  service_name_ = name_ + "/clear_entirely";
  // 전용 callback group (node 기본 executor 에 자동으로 붙지 않게 false) + 전용 executor thread.
  //   node 기본 executor 는 표준 clear_*_costmap·parameter 서비스를, Layer 에 넘어온 callback_group_ 은 layer 구독을 돌린다.
  //   이 서비스는 master lock 을 기다릴 수 있으므로 (updateMap 진행 중) 그 둘을 막지 않게 따로 둔다.
  srv_cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  service_ = node->create_service<nav2_msgs::srv::ClearEntireCostmap>(
    service_name_,
    std::bind(
      &FastClearLayer::serviceCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
    rclcpp::ServicesQoS(), srv_cb_group_);
  srv_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  srv_executor_->add_callback_group(srv_cb_group_, node->get_node_base_interface());
  srv_thread_ = std::thread(
    [this, exec = srv_executor_]() {
      auto last_err = std::chrono::steady_clock::time_point{};
      while (srv_running_.load() && rclcpp::ok()) {
        try {
          exec->spin_once(100ms);
        } catch (const std::exception & e) {
          const auto now = std::chrono::steady_clock::now();
          if (now - last_err > 5s) {
            last_err = now;
            RCLCPP_ERROR(logger_, "[10-10 W1-5] %s clear 서비스 executor 예외 (계속 돈다): %s", name_.c_str(), e.what());
          }
          std::this_thread::sleep_for(100ms);
        } catch (...) {
          std::this_thread::sleep_for(100ms);
        }
      }
    });

  current_ = true;
  RCLCPP_INFO(
    logger_, "[10-10 W1-5] FastClearLayer '%s' 초기화: enabled=%d service=%s extra_margin=%.2fm "
    "(enabled=false 면 표준 clearEntirely 와 같은 동작)",
    name_.c_str(), enabled_, service_->get_service_name(), extra_margin_m_);
}

void FastClearLayer::activate()
{
  active_.store(true);
}

void FastClearLayer::deactivate()
{
  // [10-10 W1-5][리뷰] on_deactivate → stop() 이 부른다 (on_cleanup 의 ~LayeredCostmap 보다 먼저).
  //   flag 를 내리고 clear_mtx_ 를 한 번 잡아 진행 중인 clearNow 가 끝나기를 기다린다 → 그 뒤에는 plugins/filters 를 순회하지 않는다.
  active_.store(false);
  std::lock_guard<std::mutex> l(clear_mtx_);
  RCLCPP_INFO(logger_, "[10-10 W1-5][리뷰] %s deactivate: 이후 clear 요청은 무시한다 (activate 때 다시 동작)", name_.c_str());
}

void FastClearLayer::watchFilterMasks(double * min_x, double * min_y, double * max_x, double * max_y)
{
  // [10-10 W1-5][리뷰] KeepoutFilter::maskCallback 은 mask 를 저장만 하고 bounds 를 넓히지 않는다 (fork keepout_filter.cpp:123-144).
  //   늦게 온 mask 는 이후 창 안에서만 master 에 들어가는데, 예전에는 명령마다 표준 clear 의 지도 전체 재계산이 그것을 고쳤다.
  //   빠른 clear 는 그러지 않으므로 keepout 이 inactive → active 로 바뀐 cycle 에 한 번 지도 전체 bounds 를 요구한다.
  //   (mask 가 같은 cycle 의 updateBounds 뒤·process 전에 도착해도 다음 cycle 에 active 로 보이므로 그때 지도 전체가 된다.)
  //   한계: 이미 active 인 채로 mask 가 바뀌는 경우 (mask 서버 재게시) 는 KeepoutFilter 가 알려 주지 않아 잡지 못한다 (W1_STEP5.md 리뷰 반영 표).
  if (!filters_scanned_) {
    filters_scanned_ = true;
    auto scan = [&](std::vector<std::shared_ptr<nav2_costmap_2d::Layer>> * v) {
        if (!v) {
          return;
        }
        for (auto & l : *v) {
          if (std::dynamic_pointer_cast<nav2_costmap_2d::KeepoutFilter>(l)) {
            WatchedMask w;
            w.layer = l;
            w.name = l->getName();
            watched_masks_.push_back(w);
          }
        }
      };
    scan(layered_costmap_->getPlugins());
    scan(layered_costmap_->getFilters());
  }
  bool repaint = false;
  for (auto & w : watched_masks_) {
    auto l = w.layer.lock();
    auto k = std::dynamic_pointer_cast<nav2_costmap_2d::KeepoutFilter>(l);
    if (!k) {
      continue;
    }
    const bool now = k->isEnabled() && k->isActive();
    if (now && !w.active) {
      repaint = true;
      RCLCPP_INFO(
        logger_, "[10-10 W1-5][리뷰] %s: %s 의 keepout mask 가 이제 적용 가능 → 이번 cycle 한 번 지도 전체를 다시 계산한다 "
        "(늦게 온 mask 를 로봇 주변 창 밖까지 반영)", name_.c_str(), w.name.c_str());
    }
    w.active = now;
  }
  if (repaint) {
    mask_repaints_.fetch_add(1);
    *min_x = std::numeric_limits<double>::lowest();
    *min_y = std::numeric_limits<double>::lowest();
    *max_x = std::numeric_limits<double>::max();
    *max_y = std::numeric_limits<double>::max();
  }
}

CellBox FastClearLayer::postClearWindow()
{
  std::lock_guard<std::mutex> wl(window_mtx_);
  return post_clear_window_;
}

bool FastClearLayer::hasPending()
{
  std::lock_guard<std::mutex> l(pending_mtx_);
  return pending_;
}

void FastClearLayer::updateBounds(
  double, double, double, double * min_x, double * min_y, double * max_x, double * max_y)
{
  // clear 가 남긴 사각형이 있을 때 한 번만 넓힌다. 다른 때는 bounds 를 건드리지 않는다 (keepout mask 가 막 도착한 cycle 제외).
  if (enabled_) {
    watchFilterMasks(min_x, min_y, max_x, max_y);
  }
  std::lock_guard<std::mutex> l(pending_mtx_);
  if (!pending_) {
    return;
  }
  *min_x = std::min(*min_x, p_min_x_);
  *min_y = std::min(*min_y, p_min_y_);
  *max_x = std::max(*max_x, p_max_x_);
  *max_y = std::max(*max_y, p_max_y_);
  pending_ = false;
}

void FastClearLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  // master 는 건드리지 않는다. clear 뒤 2 cycle 동안 실제로 다시 계산된 창 크기만 남긴다 (진단용).
  //   2 cycle 인 이유: InflationLayer::updateBounds 가 직전 cycle 의 bounds 를 한 번 더 합친다 (fork inflation_layer.cpp:611-622).
  //   clearable layer 의 reset() 이 지도 전체 bounds 를 요구하면 (이 기능의 목적을 깨는 경우) 여기서 WARN 으로 드러난다.
  int left = watch_cycles_.load();
  if (left <= 0) {
    return;
  }
  watch_cycles_.store(left - 1);
  {
    std::lock_guard<std::mutex> wl(window_mtx_);
    post_clear_window_.merge(CellBox{min_i, min_j, max_i, max_j});
  }
  const int sx = static_cast<int>(master_grid.getSizeInCellsX()), sy = static_cast<int>(master_grid.getSizeInCellsY());
  const double map_cells = double(sx) * double(sy);
  const double win = double(std::max(0, max_i - min_i)) * double(std::max(0, max_j - min_j));
  const double pct = map_cells > 0 ? 100.0 * win / map_cells : 0.0;
  // 창이 지도 전체인데 clear 가 남긴 범위는 전체가 아니었다 = 다른 layer 가 지도 전체 bounds 를 요구했다
  const bool whole = min_i <= 0 && min_j <= 0 && max_i >= sx && max_j >= sy;
  if (whole && !last_extent_whole_.load()) {
    RCLCPP_WARN(
      logger_, "[10-10 W1-5] %s clear 뒤 cycle %d: 다시 계산한 창이 지도 전체 [%d,%d)x[%d,%d) 다 — clear 가 남긴 범위보다 넓다. "
      "다른 layer 가 지도 전체 bounds 를 요구했다 (static 새 지도·footprint 변경·표준 clear 등). 이번 clear 는 이득이 없다.",
      name_.c_str(), 3 - left, min_i, max_i, min_j, max_j);
  } else {
    RCLCPP_INFO(
      logger_, "[10-10 W1-5] %s clear 뒤 cycle %d: 다시 계산한 창 [%d,%d)x[%d,%d) = %.0f 셀 (지도의 %.4f %%)",
      name_.c_str(), 3 - left, min_i, max_i, min_j, max_j, win, pct);
  }
}

FastClearReport FastClearLayer::clearNow()
{
  FastClearReport rep;
  const auto t0 = std::chrono::steady_clock::now();
  // [10-10 W1-5][리뷰] deactivate 와 배타. deactivate 뒤에는 layered_costmap_ (곧 lock 없이 파괴될 수 있다) 를 건드리지 않는다.
  std::lock_guard<std::mutex> cl(clear_mtx_);
  if (!active_.load()) {
    rep.skipped = true;
    skipped_calls_.fetch_add(1);
    rep.total_ms = msSince(t0);
    return rep;
  }
  nav2_costmap_2d::Costmap2D * master = layered_costmap_->getCostmap();
  // ClearCostmapService::clearEntirely 와 같은 lock (fork clear_costmap_service.cpp:150). updateMap 도 이 lock 을 cycle 내내 쥔다.
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(master->getMutex()));
  rep.lock_wait_ms = msSince(t0);
  rep.size_x = master->getSizeInCellsX();
  rep.size_y = master->getSizeInCellsY();

  auto * plugins = layered_costmap_->getPlugins();
  auto * filters = layered_costmap_->getFilters();

  if (!enabled_) {
    // 표준 Costmap2DROS::resetLayers 와 똑같이 (fork costmap_2d_ros.cpp:682-700)
    rep.standard = true;
    const auto ts = std::chrono::steady_clock::now();
    master->resetMap(0, 0, master->getSizeInCellsX(), master->getSizeInCellsY());
    for (auto & p : *plugins) {
      p->reset();
    }
    for (auto & f : *filters) {
      f->reset();
    }
    rep.reset_ms = msSince(ts);
    rep.total_ms = msSince(t0);
    calls_.fetch_add(1);
    return rep;
  }

  // 이 layer 의 위치와 inflation layer 위치: inflation 이 뒤에 있으면 그 updateBounds 가 반경만큼 넓힌다
  bool seen_self = false;
  for (auto & p : *plugins) {
    if (p.get() == this) {
      seen_self = true;
      continue;
    }
    auto infl = std::dynamic_pointer_cast<nav2_costmap_2d::InflationLayer>(p);
    if (infl && infl->isEnabled()) {
      rep.inflation_radius_m = std::max(rep.inflation_radius_m, infl->getInflationRadius());
      if (seen_self) {
        rep.inflation_after = true;
        rep.inflation_after_radius_m = std::max(rep.inflation_after_radius_m, infl->getInflationRadius());
      }
    }
  }
  // [10-10 W1-5][리뷰] InflationLayer::updateBounds 는 반경을 m 로 더하지만 (0.61 = 20.33 셀 → 창은 extent ± 20 셀)
  //   updateCosts/enqueue 는 cell_inflation_radius_ = cellDistance(반경) = ceil(20.33) = 21 셀까지 칠한다 (fork inflation_layer.cpp:578·801).
  //   지운 ghost 의 21 번째 셀이 창 밖에 남지 않게 여유를 셀 단위로 잡는다: 창 = extent ± cell_inflation_radius 셀.
  //   셀 중심 기준 ± (cell_inflation_radius) 셀 → worldToMapEnforceBounds 의 버림에 0.5 셀 여유가 생긴다.
  rep.cell_inflation_radius = rep.inflation_radius_m > 0.0 ? master->cellDistance(rep.inflation_radius_m) : 0u;
  {
    const double need_m = static_cast<double>(rep.cell_inflation_radius) * master->getResolution();
    const double added_by_inflation = rep.inflation_after ? rep.inflation_after_radius_m : 0.0;
    rep.margin_m = extra_margin_m_ + std::max(0.0, need_m - added_by_inflation);
  }

  const unsigned int sx = rep.size_x, sy = rep.size_y;
  const CellBox whole{0, 0, static_cast<int>(sx), static_cast<int>(sy)};
  double scan_ms = 0.0, reset_ms = 0.0;

  auto handle = [&](const std::shared_ptr<nav2_costmap_2d::Layer> & layer) {
      if (!layer || layer.get() == this || !layer->isClearable()) {
        return;
      }
      FastClearLayerReport lr;
      lr.name = layer->getName();
      auto cl = std::dynamic_pointer_cast<nav2_costmap_2d::CostmapLayer>(layer);
      const bool container = static_cast<bool>(
        std::dynamic_pointer_cast<nav2_costmap_2d::PluginContainerLayer>(layer));
      if (!cl || container) {
        lr.full = true;
        lr.why = !cl ? "CostmapLayer 아님" : "PluginContainerLayer";
      } else {
        // layer 자체 mutex 도 잡는다 (ClearCostmapService::clearLayerRegion 이 이 mutex 만 쥐고 char map 을 고친다).
        // 순서 master → layer 는 updateMap(master) → ObstacleLayer::updateBounds(layer) 와 같다.
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> ll(*(cl->getMutex()));
        if (cl->getSizeInCellsX() != sx || cl->getSizeInCellsY() != sy ||
          cl->getOriginX() != master->getOriginX() || cl->getOriginY() != master->getOriginY() ||
          cl->getResolution() != master->getResolution())
        {
          lr.full = true;
          lr.why = "geometry 가 master 와 다름";
        } else {
          const auto ts = std::chrono::steady_clock::now();
          lr.box = scanNonDefaultExtent(cl->getCharMap(), sx, sy, cl->getDefaultValue());
          scan_ms += msSince(ts);
          const auto tr = std::chrono::steady_clock::now();
          layer->reset();   // 표준과 같은 per-layer 효과
          reset_ms += msSince(tr);
        }
      }
      if (lr.full) {
        const auto tr = std::chrono::steady_clock::now();
        layer->reset();
        reset_ms += msSince(tr);
        lr.box = whole;
      }
      rep.extent.merge(lr.box);
      rep.layers.push_back(std::move(lr));
    };
  for (auto & p : *plugins) {
    handle(p);
  }
  for (auto & f : *filters) {
    handle(f);   // CostmapFilter::isClearable() 은 false — 표준 clearRegion 과 같은 기준이라 실제로는 건너뛴다
  }
  rep.scan_ms = scan_ms;
  rep.reset_ms = reset_ms;

  if (rep.extent.valid()) {
    // 셀 중심 좌표로 바꾼다 → LayeredCostmap::updateMap 의 worldToMapEnforceBounds(min) = x0, (max) = x1-1 (+1 → x1) 이 정확히 나온다
    const double res = master->getResolution();
    const double ox = master->getOriginX(), oy = master->getOriginY();
    const double wx0 = ox + (rep.extent.x0 + 0.5) * res - rep.margin_m;
    const double wy0 = oy + (rep.extent.y0 + 0.5) * res - rep.margin_m;
    const double wx1 = ox + (rep.extent.x1 - 0.5) * res + rep.margin_m;
    const double wy1 = oy + (rep.extent.y1 - 0.5) * res + rep.margin_m;
    std::lock_guard<std::mutex> l(pending_mtx_);
    if (pending_) {
      p_min_x_ = std::min(p_min_x_, wx0);
      p_min_y_ = std::min(p_min_y_, wy0);
      p_max_x_ = std::max(p_max_x_, wx1);
      p_max_y_ = std::max(p_max_y_, wy1);
    } else {
      p_min_x_ = wx0;
      p_min_y_ = wy0;
      p_max_x_ = wx1;
      p_max_y_ = wy1;
      pending_ = true;
    }
    last_extent_whole_.store(rep.extent.x0 <= 0 && rep.extent.y0 <= 0 &&
      rep.extent.x1 >= static_cast<int>(sx) && rep.extent.y1 >= static_cast<int>(sy));
    {
      std::lock_guard<std::mutex> wl(window_mtx_);
      post_clear_window_ = CellBox{};
    }
    watch_cycles_.store(2);
  }
  rep.total_ms = msSince(t0);
  calls_.fetch_add(1);
  return rep;
}

void FastClearLayer::logReport(const FastClearReport & r)
{
  if (r.skipped) {
    // 5 s 에 한 번만 (deactivate 중 요청이 몰려도 log 가 넘치지 않게)
    const auto now = std::chrono::steady_clock::now();
    if (skip_log_last_ == std::chrono::steady_clock::time_point{} || now - skip_log_last_ > std::chrono::seconds(5)) {
      skip_log_last_ = now;
      RCLCPP_WARN(
        logger_, "[10-10 W1-5][리뷰] %s clear 요청 무시: layer 가 deactivate 상태 (lifecycle deactivate/cleanup 중), 누적 %lu 회",
        name_.c_str(), static_cast<unsigned long>(skipped_calls_.load()));
    }
    return;
  }
  if (r.standard) {
    RCLCPP_INFO(
      logger_, "[10-10 W1-5] %s clear (enabled=false → 표준 clearEntirely: master 전체 reset + 모든 layer reset) "
      "%ux%u, lock 대기 %.1f ms, reset %.1f ms, 합계 %.1f ms", name_.c_str(), r.size_x, r.size_y,
      r.lock_wait_ms, r.reset_ms, r.total_ms);
    return;
  }
  std::string layers;
  size_t n_full = 0;
  for (const auto & l : r.layers) {
    char b[160];
    if (l.full) {
      ++n_full;
      std::snprintf(b, sizeof(b), "%s(전체: %s) ", l.name.c_str(), l.why.c_str());
    } else if (l.box.valid()) {
      std::snprintf(
        b, sizeof(b), "%s[%d,%d)x[%d,%d) ", l.name.c_str(), l.box.x0, l.box.x1, l.box.y0, l.box.y1);
    } else {
      std::snprintf(b, sizeof(b), "%s(빈) ", l.name.c_str());
    }
    layers += b;
  }
  const double map_cells = double(r.size_x) * double(r.size_y);
  const double pct = map_cells > 0 ? 100.0 * double(r.extent.area()) / map_cells : 0.0;
  const char * infl = r.inflation_after ? "inflation layer 가 뒤에서 반경만큼 넓힘 + 올림 셀 몫" :
    "inflation 이 앞/없음 → 이 layer 가 올림 반경만큼 넓힘";
  if (n_full > 0) {
    RCLCPP_WARN(
      logger_, "[10-10 W1-5] %s clear: layer %zu 개 reset (지도 전체로 잡은 layer %zu 개 — 표준과 같은 비용) %s| extent %ux%u 중 "
      "%dx%d (%zu 셀, %.4f %%) + 여유 %.2f m (%s, 반경 %.2f m = %u 셀) | scan %.1f ms, reset %.1f ms, lock 대기 %.1f ms, 합계 %.1f ms",
      name_.c_str(), r.layers.size(), n_full, layers.c_str(), r.size_x, r.size_y,
      r.extent.x1 - r.extent.x0, r.extent.y1 - r.extent.y0, r.extent.area(), pct, r.margin_m, infl,
      r.inflation_radius_m, r.cell_inflation_radius, r.scan_ms, r.reset_ms, r.lock_wait_ms, r.total_ms);
  } else {
    RCLCPP_INFO(
      logger_, "[10-10 W1-5] %s clear: layer %zu 개 reset %s| extent %ux%u 중 %dx%d (%zu 셀, %.4f %%) + 여유 %.2f m (%s, 반경 %.2f m = %u 셀) "
      "| scan %.1f ms, reset %.1f ms, lock 대기 %.1f ms, 합계 %.1f ms",
      name_.c_str(), r.layers.size(), layers.c_str(), r.size_x, r.size_y,
      r.extent.valid() ? r.extent.x1 - r.extent.x0 : 0, r.extent.valid() ? r.extent.y1 - r.extent.y0 : 0,
      r.extent.area(), pct, r.margin_m, infl, r.inflation_radius_m, r.cell_inflation_radius, r.scan_ms, r.reset_ms,
      r.lock_wait_ms,
      r.total_ms);
  }
}

void FastClearLayer::serviceCallback(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Request>,
  std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Response>)
{
  try {
    const FastClearReport r = clearNow();
    logReport(r);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "[10-10 W1-5] %s clear 실패: %s", name_.c_str(), e.what());
  }
}

}  // namespace ammr_costmap_diff_publisher

PLUGINLIB_EXPORT_CLASS(ammr_costmap_diff_publisher::FastClearLayer, nav2_costmap_2d::Layer)
