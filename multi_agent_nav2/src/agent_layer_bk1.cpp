#include "multi_agent_nav2/agent_layer.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>

#include <geometry_msgs/msg/point32.hpp>
#include <multi_agent_msgs/msg/agent_status.hpp>
#include <multi_agent_msgs/msg/agent_layer_cell_meta.hpp>

PLUGINLIB_EXPORT_CLASS(multi_agent_nav2::AgentLayer, nav2_costmap_2d::Layer)


namespace multi_agent_nav2
{

AgentLayer::AgentLayer() {}

void AgentLayer::onInitialize()
{
  node_shared_ = node_.lock();
  if (!node_shared_) {
    throw std::runtime_error("AgentLayer: failed to lock lifecycle node");
  }

  // Declare parameters (defaults)
  node_shared_->declare_parameter("enabled", rclcpp::ParameterValue(true));
  node_shared_->declare_parameter("topic", rclcpp::ParameterValue(std::string("/multi_agent_infos")));
  node_shared_->declare_parameter("self_machine_id", rclcpp::ParameterValue(0));
  node_shared_->declare_parameter("self_type_id", rclcpp::ParameterValue(std::string("")));
  node_shared_->declare_parameter("use_path_header_frame", rclcpp::ParameterValue(true));
  node_shared_->declare_parameter("roi_range_m", rclcpp::ParameterValue(12.0));
  node_shared_->declare_parameter("time_decay_sec", rclcpp::ParameterValue(1.0));
  node_shared_->declare_parameter("lethal_cost", rclcpp::ParameterValue(254));
  node_shared_->declare_parameter("moving_cost", rclcpp::ParameterValue(180));
  node_shared_->declare_parameter("waiting_cost", rclcpp::ParameterValue(200));
  node_shared_->declare_parameter("manual_cost_bias", rclcpp::ParameterValue(30));
  node_shared_->declare_parameter("dilation_m", rclcpp::ParameterValue(0.05));
  node_shared_->declare_parameter("forward_smear_m", rclcpp::ParameterValue(0.25));
  node_shared_->declare_parameter("sigma_k", rclcpp::ParameterValue(2.0));
  node_shared_->declare_parameter("publish_meta", rclcpp::ParameterValue(true));
  node_shared_->declare_parameter("meta_stride", rclcpp::ParameterValue(3));
  node_shared_->declare_parameter("freshness_timeout_ms", rclcpp::ParameterValue(800));
  node_shared_->declare_parameter("max_poses", rclcpp::ParameterValue(40));
  node_shared_->declare_parameter("qos_reliable", rclcpp::ParameterValue(true));

  // Get parameters
  node_shared_->get_parameter("enabled", enabled_);
  node_shared_->get_parameter("topic", topic_);
  {
    int tmp = 0;
    node_shared_->get_parameter("self_machine_id", tmp);
    self_machine_id_ = static_cast<uint16_t>(tmp);
  }
  node_shared_->get_parameter("self_type_id", self_type_id_);
  node_shared_->get_parameter("use_path_header_frame", use_path_header_frame_);
  node_shared_->get_parameter("roi_range_m", roi_range_m_);
  node_shared_->get_parameter("time_decay_sec", time_decay_sec_);
  {
    int tmp = 254; node_shared_->get_parameter("lethal_cost", tmp);
    lethal_cost_ = static_cast<unsigned char>(std::clamp(tmp, 0, 254));
  }
  {
    int tmp = 180; node_shared_->get_parameter("moving_cost", tmp);
    moving_cost_ = static_cast<unsigned char>(std::clamp(tmp, 0, 254));
  }
  {
    int tmp = 200; node_shared_->get_parameter("waiting_cost", tmp);
    waiting_cost_ = static_cast<unsigned char>(std::clamp(tmp, 0, 254));
  }
  node_shared_->get_parameter("manual_cost_bias", manual_cost_bias_);
  node_shared_->get_parameter("dilation_m", dilation_m_);
  node_shared_->get_parameter("forward_smear_m", forward_smear_m_);
  node_shared_->get_parameter("sigma_k", sigma_k_);
  node_shared_->get_parameter("publish_meta", publish_meta_);
  node_shared_->get_parameter("meta_stride", meta_stride_);
  node_shared_->get_parameter("freshness_timeout_ms", freshness_timeout_ms_);
  node_shared_->get_parameter("max_poses", max_poses_);
  node_shared_->get_parameter("qos_reliable", qos_reliable_);

  current_ = true;
  matchSize();

  if (enabled_) activate();
}

void AgentLayer::activate()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  if (qos_reliable_) qos.reliable(); else qos.best_effort();

  sub_ = node_shared_->create_subscription<multi_agent_msgs::msg::MultiAgentInfoArray>(
      topic_, qos, std::bind(&AgentLayer::infosCallback, this, std::placeholders::_1));

  if (publish_meta_) {
    meta_pub_ = node_shared_->create_publisher<multi_agent_msgs::msg::AgentLayerMetaArray>(
        "agent_layer_meta", rclcpp::QoS(1).reliable().transient_local());
  }
}

void AgentLayer::deactivate()
{
  sub_.reset();
  meta_pub_.reset();
}

void AgentLayer::infosCallback(
  const multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(data_mtx_);
  last_infos_ = msg;
  last_stamp_ = msg->header.stamp;
}

bool AgentLayer::stale(const rclcpp::Time & stamp) const
{
  return (node_shared_->now() - stamp) >
         rclcpp::Duration::from_nanoseconds(
           static_cast<int64_t>(freshness_timeout_ms_) * 1000000LL);
}

bool AgentLayer::isSelf(const multi_agent_msgs::msg::MultiAgentInfo & a) const
{
  return (a.machine_id == self_machine_id_) && (a.type_id == self_type_id_);
}

unsigned char AgentLayer::computeCost(const multi_agent_msgs::msg::MultiAgentInfo & a) const
{
  using S = multi_agent_msgs::msg::AgentStatus;
  const uint8_t p = a.status.phase;
  const bool is_moving =
      (p == S::STATUS_MOVING) || (p == S::STATUS_PATH_SEARCHING);

  unsigned char base = is_moving ? moving_cost_ : waiting_cost_;

  if (a.mode == "manual") {
    int c = static_cast<int>(base) + manual_cost_bias_;
    return static_cast<unsigned char>(std::clamp(c, 0, 254));
  }
  return base;
}

double AgentLayer::computeDilation(const multi_agent_msgs::msg::MultiAgentInfo & a) const
{
  double r = dilation_m_;
  // 위치 불확실도 반영
  if (a.pos_std_m >= 0.0) r += sigma_k_ * a.pos_std_m;
  // 진행방향 스미어(단순 가산)
  if (forward_smear_m_ > 0.0 &&
      a.status.phase == multi_agent_msgs::msg::AgentStatus::STATUS_MOVING) {
    r += forward_smear_m_;
  }
  return r;
}

void AgentLayer::updateBounds(double robot_x, double robot_y, double /*robot_yaw*/,
                              double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_) return;

  touched_ = false;
  touch_min_x_ =  1e9; touch_min_y_ =  1e9;
  touch_max_x_ = -1e9; touch_max_y_ = -1e9;

  std::vector<multi_agent_msgs::msg::MultiAgentInfo> infos;
  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    if (!last_infos_ || stale(last_stamp_)) return;
    infos.assign(last_infos_->agents.begin(), last_infos_->agents.end());
  }

  const std::string & global_frame = layered_costmap_->getGlobalFrameID();

  for (const auto & a : infos) {
    if (isSelf(a)) continue;

    // ROI by distance from our robot
    const double dx = a.current_pose.pose.position.x - robot_x;
    const double dy = a.current_pose.pose.position.y - robot_y;
    if (std::hypot(dx, dy) > roi_range_m_) continue;

    // frame check (optional)
    if (use_path_header_frame_ && a.truncated_path.header.frame_id != global_frame) {
      continue;
    }

    const int limit = std::min<int>(a.truncated_path.poses.size(), max_poses_);
    for (int i = 0; i < limit; ++i) {
      const auto & p = a.truncated_path.poses[i].pose.position;
      if (p.x < touch_min_x_) touch_min_x_ = p.x;
      if (p.y < touch_min_y_) touch_min_y_ = p.y;
      if (p.x > touch_max_x_) touch_max_x_ = p.x;
      if (p.y > touch_max_y_) touch_max_y_ = p.y;
      touched_ = true;
    }
  }

  if (touched_) {
    *min_x = std::min(*min_x, touch_min_x_);
    *min_y = std::min(*min_y, touch_min_y_);
    *max_x = std::max(*max_x, touch_max_x_);
    *max_y = std::max(*max_y, touch_max_y_);
  }
}

void AgentLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                             int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
  if (!enabled_) return;

  std::vector<multi_agent_msgs::msg::MultiAgentInfo> infos;
  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    if (!last_infos_ || stale(last_stamp_)) return;
    infos.assign(last_infos_->agents.begin(), last_infos_->agents.end());
  }

  std::vector<std::pair<unsigned int,unsigned int>> meta_hits;
  meta_hits.reserve(256);

  for (const auto & a : infos) {
    if (isSelf(a)) continue;
    rasterizeAgentPath(a, &master_grid, meta_hits);
  }

  if (publish_meta_ && meta_pub_) {
    multi_agent_msgs::msg::AgentLayerMetaArray arr;
    arr.header.frame_id = layered_costmap_->getGlobalFrameID();
    arr.header.stamp = node_shared_->now();

    for (size_t k = 0; k < meta_hits.size(); k += std::max(1, meta_stride_)) {
      auto [mx, my] = meta_hits[k];
      double wx, wy; master_grid.mapToWorld(mx, my, wx, wy);
      multi_agent_msgs::msg::AgentLayerCellMeta cm;
      cm.header = arr.header;

      // 간단 채움(원하면 rasterize에서 셀별 주체 agent 식별 추가 가능)
      cm.machine_id = 0;
      cm.phase = 0;
      cm.mode = "";
      cm.reroute = false;
      cm.re_path_search = false;
      cm.transferring = false;
      cm.area_id = 0;

      cm.position.x = wx; cm.position.y = wy; cm.position.z = 0.0;
      cm.mx = mx; cm.my = my;

      cm.t_first_hit = -1.0f;
      cm.sigma = 0.0f;

      arr.cells.emplace_back(std::move(cm));
    }
    meta_pub_->publish(std::move(arr));
  }
}

static inline std::vector<geometry_msgs::msg::Point>
dilatePolygon(const std::vector<geometry_msgs::msg::Point32> & in, double d)
{
  std::vector<geometry_msgs::msg::Point> out; out.reserve(in.size());
  if (in.empty()) return out;

  // centroid
  double cx=0, cy=0;
  for (auto & p : in) { cx += p.x; cy += p.y; }
  cx /= static_cast<double>(in.size());
  cy /= static_cast<double>(in.size());

  for (auto & p : in) {
    double vx = p.x - cx, vy = p.y - cy;
    double n = std::hypot(vx, vy); if (n < 1e-6) n = 1.0;
    geometry_msgs::msg::Point q;
    q.x = p.x + d * (vx / n);
    q.y = p.y + d * (vy / n);
    q.z = 0.0;
    out.push_back(q);
  }
  return out;
}

void AgentLayer::rasterizeAgentPath(
  const multi_agent_msgs::msg::MultiAgentInfo & a,
  nav2_costmap_2d::Costmap2D * grid,
  std::vector<std::pair<unsigned int,unsigned int>> & meta_hits)
{
  if (a.truncated_path.poses.empty()) return;

  const unsigned char cost = computeCost(a);
  const double extra = computeDilation(a);
  const auto & fp = a.footprint;
  const int limit = std::min<int>(a.truncated_path.poses.size(), max_poses_);

  for (int i = 0; i < limit; ++i) {
    const auto & ps = a.truncated_path.poses[i].pose;
    fillFootprintAt(fp, ps, extra, grid, cost, &meta_hits);
  }
}

void AgentLayer::fillFootprintAt(const geometry_msgs::msg::PolygonStamped & fp,
                                 const geometry_msgs::msg::Pose & pose,
                                 double extra_dilation_m,
                                 nav2_costmap_2d::Costmap2D * grid,
                                 unsigned char cost,
                                 std::vector<std::pair<unsigned int,unsigned int>> * meta_hits)
{
  // 1) inflate local polygon
  auto poly = dilatePolygon(fp.polygon.points, extra_dilation_m);

  // 2) transform to world by pose
  const double yaw = tf2::getYaw(pose.orientation);
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (auto & p : poly) {
    const double x = p.x, y = p.y;
    p.x = pose.position.x + c * x - s * y;
    p.y = pose.position.y + s * x + c * y;
  }

  // 3) bbox in world
  double minx=1e9, miny=1e9, maxx=-1e9, maxy=-1e9;
  for (auto & p : poly) {
    if (p.x < minx) minx = p.x;
    if (p.y < miny) miny = p.y;
    if (p.x > maxx) maxx = p.x;
    if (p.y > maxy) maxy = p.y;
  }

  // 4) bbox→map index (enforce bounds uses int&)
  int min_i, min_j, max_i, max_j;
  grid->worldToMapEnforceBounds(minx, miny, min_i, min_j);
  grid->worldToMapEnforceBounds(maxx, maxy, max_i, max_j);

  // 5) fill by point-in-polygon
  for (int j = min_j; j <= max_j; ++j) {
    for (int i = min_i; i <= max_i; ++i) {
      double wx, wy; grid->mapToWorld(i, j, wx, wy);

      // ray-casting
      bool inside = false;
      const size_t n = poly.size();
      for (size_t k=0, h=n-1; k<n; h=k++) {
        const double xi = poly[k].x, yi = poly[k].y;
        const double xh = poly[h].x, yh = poly[h].y;
        const bool hit = ((yi > wy) != (yh > wy)) &&
                         (wx < (xh - xi) * (wy - yi) / std::max(1e-12, (yh - yi)) + xi);
        if (hit) inside = !inside;
      }

      if (inside) {
        grid->setCost(static_cast<unsigned int>(i),
                      static_cast<unsigned int>(j), cost);
        if (meta_hits) meta_hits->emplace_back(
            static_cast<unsigned int>(i), static_cast<unsigned int>(j));
      }
    }
  }

  // 6) bounds bookkeeping for updateBounds
  if (!touched_) {
    touch_min_x_ = minx; touch_min_y_ = miny;
    touch_max_x_ = maxx; touch_max_y_ = maxy;
    touched_ = true;
  } else {
    touch_min_x_ = std::min(touch_min_x_, minx);
    touch_min_y_ = std::min(touch_min_y_, miny);
    touch_max_x_ = std::max(touch_max_x_, maxx);
    touch_max_y_ = std::max(touch_max_y_, maxy);
  }
}

bool AgentLayer::pointInPolygon(const std::vector<geometry_msgs::msg::Point> & poly,
                                double x, double y)
{
  bool inside = false;
  const size_t n = poly.size();
  for (size_t i=0, j=n-1; i<n; j=i++) {
    const double xi = poly[i].x, yi = poly[i].y;
    const double xj = poly[j].x, yj = poly[j].y;
    const bool hit = ((yi > y) != (yj > y)) &&
                     (x < (xj - xi) * (y - yi) / std::max(1e-12, (yj - yi)) + xi);
    if (hit) inside = !inside;
  }
  return inside;
}

} // namespace multi_agent_nav2

// pluginlib export (cpp 마지막에 단 한 번)
// PLUGINLIB_EXPORT_CLASS(multi_agent_nav2::AgentLayer, nav2_costmap_2d::Layer)

