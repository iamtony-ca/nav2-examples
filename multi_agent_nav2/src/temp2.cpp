#include "multi_agent_nav2/agent_layer.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <nav2_costmap_2d/footprint.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <multi_agent_msgs/msg/agent_status.hpp>
#include <multi_agent_msgs/msg/agent_layer_cell_meta.hpp>
#include "tf2_ros/buffer.h"

namespace multi_agent_nav2
{

// ==========================================
// Helper Implementations
// ==========================================

std::vector<geometry_msgs::msg::Point32> AgentLayer::toPoint32(
    const std::vector<geometry_msgs::msg::Point>& points)
{
    std::vector<geometry_msgs::msg::Point32> points32;
    points32.reserve(points.size());
    for (const auto& p : points) {
        geometry_msgs::msg::Point32 p32;
        p32.x = static_cast<float>(p.x);
        p32.y = static_cast<float>(p.y);
        p32.z = 0.0f; 
        points32.push_back(p32);
    }
    return points32;
}

geometry_msgs::msg::PolygonStamped 
AgentLayer::getFootprintForAgent(const multi_agent_msgs::msg::MultiAgentInfo & a)
{
    geometry_msgs::msg::PolygonStamped fp_stamped;
    
    auto it = agent_footprints_.find(a.machine_id);
    if (it == agent_footprints_.end()) {
        RCLCPP_WARN_ONCE(logger_, "No footprint data found in YAML for machine_id %u", a.machine_id);
        return fp_stamped; 
    }

    const auto& data = it->second;
    if (data.use_radius) {
        std::vector<geometry_msgs::msg::Point> points = 
            nav2_costmap_2d::makeFootprintFromRadius(data.radius);
        fp_stamped.polygon.points = toPoint32(points);
    } else {
        fp_stamped.polygon.points = data.points;
    }

    fp_stamped.header.frame_id = "base_link"; 
    fp_stamped.header.stamp = a.current_pose.header.stamp;
    return fp_stamped;
}

// [FIX] TF 변환 시 '최신 시간(Time(0))'을 사용하여 Local Costmap 표시 문제 해결
bool AgentLayer::transformAgentInfo(
    const multi_agent_msgs::msg::MultiAgentInfo & agent_in_map,
    multi_agent_msgs::msg::MultiAgentInfo & agent_in_costmap_frame,
    const std::string & source_frame_id,
    const std::string & target_frame_id) const
{
  if (source_frame_id == target_frame_id) {
    agent_in_costmap_frame = agent_in_map;
    return true;
  }

  agent_in_costmap_frame = agent_in_map; 
  agent_in_costmap_frame.truncated_path.poses.clear(); 

  // [KEY FIX] TF Lookup을 위한 시간 설정: rclcpp::Time(0) = Latest Available
  rclcpp::Time latest_time(0);

  try {
    // --- Transform Current Pose ---
    geometry_msgs::msg::PoseStamped pose_to_transform = agent_in_map.current_pose;
    pose_to_transform.header.frame_id = source_frame_id;
    pose_to_transform.header.stamp = latest_time; 

    geometry_msgs::msg::PoseStamped transformed_pose;
    tf_->transform(pose_to_transform, transformed_pose, target_frame_id);
    agent_in_costmap_frame.current_pose = transformed_pose;

    // --- Transform Path Poses ---
    for (const auto& pose_stamped_in_map : agent_in_map.truncated_path.poses) {
      geometry_msgs::msg::PoseStamped path_pose_to_transform = pose_stamped_in_map;
      
      path_pose_to_transform.header.frame_id = source_frame_id;
      path_pose_to_transform.header.stamp = latest_time; 

      geometry_msgs::msg::PoseStamped pose_in_costmap_frame;
      tf_->transform(path_pose_to_transform, pose_in_costmap_frame, target_frame_id);
      
      agent_in_costmap_frame.truncated_path.poses.push_back(pose_in_costmap_frame);
    }

    agent_in_costmap_frame.truncated_path.header.frame_id = target_frame_id;
    agent_in_costmap_frame.truncated_path.header.stamp = transformed_pose.header.stamp;
    return true;

  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node_shared_->get_clock(), 2000,
      "TF Fail agent %u ('%s'->'%s'): %s",
      agent_in_map.machine_id, source_frame_id.c_str(), target_frame_id.c_str(), ex.what());
    return false;
  }
}

// ==========================================
// AgentLayer Implementation
// ==========================================

AgentLayer::AgentLayer()
{
  // costmap_ = nullptr; // CostmapLayer가 아니므로 사용 안 함
}

AgentLayer::~AgentLayer()
{
}

void AgentLayer::onInitialize()
{
  node_shared_ = node_.lock();
  if (!node_shared_) {
    throw std::runtime_error("AgentLayer: failed to lock lifecycle node");
  }

  // --- Parameters ---
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("/multi_agent_infos")));
  declareParameter("self_machine_id", rclcpp::ParameterValue(0));
  declareParameter("self_type_id", rclcpp::ParameterValue(std::string("")));
  declareParameter("use_path_header_frame", rclcpp::ParameterValue(true));
  declareParameter("roi_range_m", rclcpp::ParameterValue(12.0));
  declareParameter("time_decay_sec", rclcpp::ParameterValue(1.0));
  declareParameter("lethal_cost", rclcpp::ParameterValue(254));
  declareParameter("moving_cost", rclcpp::ParameterValue(180));
  declareParameter("waiting_cost", rclcpp::ParameterValue(200));
  declareParameter("manual_cost_bias", rclcpp::ParameterValue(30));
  declareParameter("dilation_m", rclcpp::ParameterValue(0.05));
  declareParameter("forward_smear_m", rclcpp::ParameterValue(0.005));
  declareParameter("sigma_k", rclcpp::ParameterValue(2.0));
  declareParameter("publish_meta", rclcpp::ParameterValue(true));
  declareParameter("meta_stride", rclcpp::ParameterValue(3));
  declareParameter("freshness_timeout_ms", rclcpp::ParameterValue(800));
  declareParameter("max_poses", rclcpp::ParameterValue(10000));
  declareParameter("qos_reliable", rclcpp::ParameterValue(true));

  node_shared_->get_parameter(name_ + "." + "enabled", enabled_);
  node_shared_->get_parameter(name_ + "." + "topic", topic_);
  {
    int tmp = 0;
    node_shared_->get_parameter(name_ + "." + "self_machine_id", tmp);
    self_machine_id_ = static_cast<uint16_t>(tmp);
  }
  node_shared_->get_parameter(name_ + "." + "self_type_id", self_type_id_);
  node_shared_->get_parameter(name_ + "." + "use_path_header_frame", use_path_header_frame_);
  node_shared_->get_parameter(name_ + "." + "roi_range_m", roi_range_m_);
  node_shared_->get_parameter(name_ + "." + "time_decay_sec", time_decay_sec_);
  {
    int tmp = 254; 
    node_shared_->get_parameter(name_ + "." + "lethal_cost", tmp);
    lethal_cost_ = static_cast<unsigned char>(std::clamp(tmp, 0, 254));
  }
  {
    int tmp = 180; 
    node_shared_->get_parameter(name_ + "." + "moving_cost", tmp);
    moving_cost_ = static_cast<unsigned char>(std::clamp(tmp, 0, 254));
  }
  {
    int tmp = 200; 
    node_shared_->get_parameter(name_ + "." + "waiting_cost", tmp);
    waiting_cost_ = static_cast<unsigned char>(std::clamp(tmp, 0, 254));
  }
  node_shared_->get_parameter(name_ + "." + "manual_cost_bias", manual_cost_bias_);
  node_shared_->get_parameter(name_ + "." + "dilation_m", dilation_m_);
  node_shared_->get_parameter(name_ + "." + "forward_smear_m", forward_smear_m_);
  node_shared_->get_parameter(name_ + "." + "sigma_k", sigma_k_);
  node_shared_->get_parameter(name_ + "." + "publish_meta", publish_meta_);
  node_shared_->get_parameter(name_ + "." + "meta_stride", meta_stride_);
  node_shared_->get_parameter(name_ + "." + "freshness_timeout_ms", freshness_timeout_ms_);
  node_shared_->get_parameter(name_ + "." + "max_poses", max_poses_);
  node_shared_->get_parameter(name_ + "." + "qos_reliable", qos_reliable_);

  // --- Robot Footprints ---
  declareParameter("robot_ids", rclcpp::ParameterValue(std::vector<std::string>({})));
  std::vector<std::string> robot_ids;
  node_shared_->get_parameter(name_ + "." + "robot_ids", robot_ids);

  agent_footprints_.clear();
  for (const auto & id_str : robot_ids) {
    std::string id_ns = name_ + "." + id_str;
    
    int machine_id_int = 0;
    node_shared_->get_parameter(id_ns + ".machine_id", machine_id_int);
    if (machine_id_int == 0) continue; 

    uint16_t machine_id = static_cast<uint16_t>(machine_id_int);
    
    AgentFootprintData data;
    std::string footprint_str;
    node_shared_->get_parameter(id_ns + ".footprint", footprint_str);
    node_shared_->get_parameter(id_ns + ".robot_radius", data.radius);

    std::vector<geometry_msgs::msg::Point> footprint_points;
    if (nav2_costmap_2d::makeFootprintFromString(footprint_str, footprint_points) &&
        footprint_points.size() >= 3)
    {
      data.points = toPoint32(footprint_points); 
      data.use_radius = false;
    } else {
      data.use_radius = true;
      RCLCPP_INFO(logger_, 
        "Using radius (%.2f) for machine_id %u (footprint string: '%s')",
        data.radius, machine_id, footprint_str.c_str());
    }
    agent_footprints_[machine_id] = data;
  }

  RCLCPP_INFO(logger_, 
      "AgentLayer '%s' initialized. Topic: %s", name_.c_str(), topic_.c_str());

  // [VISUALIZATION] 시각화용 맵 및 퍼블리셔 초기화
  viz_costmap_.setDefaultValue(0); // 배경은 투명(0)

  // 토픽 이름: /global_costmap/agent_layer/costmap
  costmap_pub_ = std::make_unique<nav2_costmap_2d::Costmap2DPublisher>(
      node_shared_, 
      &viz_costmap_, 
      layered_costmap_->getGlobalFrameID(), 
      name_ + "/costmap", 
      false);

  current_ = true;
  
  // [CRITICAL] 활성화를 강제로 수행하여 Subscriber가 반드시 생성되도록 함
  if (enabled_) {
    activate();
  }
}

void AgentLayer::activate()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  if (qos_reliable_) qos.reliable(); else qos.best_effort();

  // Subscriber 생성
  sub_ = node_shared_->create_subscription<multi_agent_msgs::msg::MultiAgentInfoArray>(
      topic_, qos, std::bind(&AgentLayer::infosCallback, this, std::placeholders::_1));

  if (publish_meta_) {
    meta_pub_ = node_shared_->create_publisher<multi_agent_msgs::msg::AgentLayerMetaArray>(
        "agent_layer_meta", rclcpp::QoS(1).reliable().transient_local());
  }

  // [VISUALIZATION] 퍼블리셔 활성화
  if (costmap_pub_) costmap_pub_->on_activate();
}

void AgentLayer::deactivate()
{
  sub_.reset();
  meta_pub_.reset();

  // [VISUALIZATION] 퍼블리셔 비활성화
  if (costmap_pub_) costmap_pub_->on_deactivate();
}

void AgentLayer::reset()
{
  current_ = true;
  touched_ = false;
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
  const bool is_moving = (p == S::STATUS_MOVING) || (p == S::STATUS_PATH_SEARCHING);

  unsigned char base = is_moving ? moving_cost_ : waiting_cost_;

  if (a.mode == "manual") {
    int c = static_cast<int>(base) + manual_cost_bias_;
    return static_cast<unsigned char>(std::clamp(c, 0, 254));
  }
  return base;
}

double AgentLayer::computeDilation(const multi_agent_msgs::msg::MultiAgentInfo & a) const
{
  using S = multi_agent_msgs::msg::AgentStatus;
  const uint8_t phase = a.status.phase;
  double r = dilation_m_;

  switch (phase) {
    case S::STATUS_AUTORECOVERY:
    case S::STATUS_ERROR:
    case S::STATUS_PAUSE:
    case S::STATUS_WAITING_FOR_SAFETY:
    case S::STATUS_WAITING_FOR_FLOWCONTROL:
    case S::STATUS_WAITING_FOR_ROS_STATUS:
      r = std::max(r, 0.0); 
      break;

    case S::STATUS_LOADING:
    case S::STATUS_UNLOADING:
    case S::STATUS_UNLOADED:
    case S::STATUS_LOADED:
    case S::STATUS_RECOVERING:
    case S::STATUS_UNKNOWN:
    case S::STATUS_MANUAL_RUNNING:
    case S::STATUS_MANUAL_COMPLETE:
      r = std::max(r, 0.1);  
      break;
    default:
      break;
  }
  return r;
}

void AgentLayer::updateBounds(double robot_x, double robot_y, double /*robot_yaw*/,
                              double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_) return;

  cached_robot_x_ = robot_x;
  cached_robot_y_ = robot_y;

  touched_ = false;
  touch_min_x_ = 1e9; touch_min_y_ = 1e9;
  touch_max_x_ = -1e9; touch_max_y_ = -1e9;

  std::vector<multi_agent_msgs::msg::MultiAgentInfo> infos;
  std::string source_frame;
  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    if (!last_infos_ || stale(last_stamp_)) return;
    source_frame = last_infos_->header.frame_id;
    infos.assign(last_infos_->agents.begin(), last_infos_->agents.end());
  }

  const std::string & target_frame = layered_costmap_->getGlobalFrameID();

  for (const auto & a_map : infos) {
    if (isSelf(a_map)) continue;

    multi_agent_msgs::msg::MultiAgentInfo a;
    // TF 변환 수행
    if (!transformAgentInfo(a_map, a, source_frame, target_frame)) continue;

    const double dx = a.current_pose.pose.position.x - robot_x;
    const double dy = a.current_pose.pose.position.y - robot_y;
    if (std::hypot(dx, dy) > roi_range_m_) continue;

    // Bounds 계산
    {
      const auto & p = a.current_pose.pose.position;
      touch_min_x_ = std::min(touch_min_x_, p.x);
      touch_min_y_ = std::min(touch_min_y_, p.y);
      touch_max_x_ = std::max(touch_max_x_, p.x);
      touch_max_y_ = std::max(touch_max_y_, p.y);
      touched_ = true;
    }

    const int limit = std::min<int>(a.truncated_path.poses.size(), max_poses_);
    for (int i = 0; i < limit; ++i) {
      const auto & p = a.truncated_path.poses[i].pose.position; 
      touch_min_x_ = std::min(touch_min_x_, p.x);
      touch_min_y_ = std::min(touch_min_y_, p.y);
      touch_max_x_ = std::max(touch_max_x_, p.x);
      touch_max_y_ = std::max(touch_max_y_, p.y);
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

// Helper: Directional Footprint Dilation
static inline std::vector<geometry_msgs::msg::Point>
dilateFootprintDirectional(const std::vector<geometry_msgs::msg::Point32> & in,
                           double iso_dilate_m,
                           double forward_len_m)
{
  std::vector<geometry_msgs::msg::Point> out; out.reserve(in.size());
  if (in.empty()) return out;

  double cx=0, cy=0;
  for (auto & p : in) { cx += p.x; cy += p.y; }
  cx /= static_cast<double>(in.size());
  cy /= static_cast<double>(in.size());

  for (auto & p : in) {
    double vx = p.x - cx, vy = p.y - cy;
    double n = std::hypot(vx, vy); if (n < 1e-6) n = 1.0;

    double x_local = p.x + iso_dilate_m * (vx / n);
    double y_local = p.y + iso_dilate_m * (vy / n);

    if (forward_len_m > 1e-6 && (p.x - cx) >= 0.0) {
      x_local += forward_len_m;
    }

    geometry_msgs::msg::Point q;
    q.x = x_local; q.y = y_local; q.z = 0.0;
    out.push_back(q);
  }
  return out;
}

void AgentLayer::fillFootprintAt(const geometry_msgs::msg::PolygonStamped & fp,
                                 const geometry_msgs::msg::Pose & pose,
                                 double extra_dilation_m,
                                 double forward_len_m,
                                 nav2_costmap_2d::Costmap2D * grid,
                                 unsigned char cost,
                                 std::vector<std::pair<unsigned int,unsigned int>> * meta_hits)
{
  auto poly = dilateFootprintDirectional(fp.polygon.points, extra_dilation_m, forward_len_m);

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
  // grid는 master_grid 혹은 viz_costmap_
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
                         (wx < (xh - xi) * (wy - yi) / std::max(1e-12, (yh - yi)) + xi);
        if (hit) inside = !inside;
      }

      if (inside) {
        // [IMPORTANT] Master Grid에 직접 쓸 때는 기존 값과 비교(Max-Merge)
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

void AgentLayer::rasterizeAgentPath(
  const multi_agent_msgs::msg::MultiAgentInfo & a,
  nav2_costmap_2d::Costmap2D * grid,
  std::vector<std::pair<unsigned int,unsigned int>> & meta_hits)
{
  geometry_msgs::msg::PolygonStamped fp = getFootprintForAgent(a);
  if (fp.polygon.points.empty()) return;

  const unsigned char cost_now = computeCost(a);
  const double iso_extra = computeDilation(a);
  const double forward_len = isMovingPhase(a.status.phase) ? forward_smear_m_ : 0.0;

  fillFootprintAt(fp, a.current_pose.pose, iso_extra, forward_len,
                  grid, cost_now, &meta_hits);

  const int limit = std::min<int>(a.truncated_path.poses.size(), max_poses_);
  for (int i = 0; i < limit; ++i) {
    const auto & ps = a.truncated_path.poses[i].pose;
    fillFootprintAt(fp, ps, iso_extra * 0.5, 0.0, grid, cost_now, &meta_hits);
  }
}

void AgentLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                             int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) return;
  
  // =================================================================
  // [VISUALIZATION] 시각화용 맵(viz_costmap_) 동기화 및 초기화
  // =================================================================
  {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(viz_costmap_.getMutex()));
      if (viz_costmap_.getSizeInCellsX() != master_grid.getSizeInCellsX() ||
          viz_costmap_.getSizeInCellsY() != master_grid.getSizeInCellsY() ||
          viz_costmap_.getResolution() != master_grid.getResolution() ||
          viz_costmap_.getOriginX() != master_grid.getOriginX() ||
          viz_costmap_.getOriginY() != master_grid.getOriginY())
      {
        viz_costmap_.resizeMap(master_grid.getSizeInCellsX(),
                              master_grid.getSizeInCellsY(),
                              master_grid.getResolution(),
                              master_grid.getOriginX(),
                              master_grid.getOriginY());
      }
      viz_costmap_.resetMaps(); // 시각화용은 매번 0으로 클리어
  }

  // 데이터 준비
  std::vector<multi_agent_msgs::msg::MultiAgentInfo> infos;
  std::string source_frame;
  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    if (!last_infos_ || stale(last_stamp_)) return;
    source_frame = last_infos_->header.frame_id;
    infos.assign(last_infos_->agents.begin(), last_infos_->agents.end());
  }

  std::vector<std::pair<unsigned int,unsigned int>> meta_hits;
  meta_hits.reserve(256);
  std::vector<std::pair<unsigned int,unsigned int>> dummy_hits; // 시각화용 더미

  const double robot_x = cached_robot_x_;
  const double robot_y = cached_robot_y_;
  const std::string & target_frame = layered_costmap_->getGlobalFrameID();

  for (const auto & a_map : infos) {
    if (isSelf(a_map)) continue;

    multi_agent_msgs::msg::MultiAgentInfo a;
    // TF 변환 (Time(0) 적용됨 -> Local Costmap 문제 해결)
    if (!transformAgentInfo(a_map, a, source_frame, target_frame)) continue;

    const double dx = a.current_pose.pose.position.x - robot_x;
    const double dy = a.current_pose.pose.position.y - robot_y;
    if (std::hypot(dx, dy) > roi_range_m_) continue;

    // 1. [NAVIGATION] Master Grid에 직접 그리기 (안전한 회피용)
    rasterizeAgentPath(a, &master_grid, meta_hits);

    // 2. [VISUALIZATION] Viz Grid에 따로 그리기 (화면 표시용)
    rasterizeAgentPath(a, &viz_costmap_, dummy_hits);
  }

  // [VISUALIZATION] 별도 Costmap 발행
  if (costmap_pub_) {
      // 전체 영역 발행
      costmap_pub_->updateBounds(0, viz_costmap_.getSizeInCellsX(),
                                 0, viz_costmap_.getSizeInCellsY());
      costmap_pub_->publishCostmap();
  }

  // Meta Data 발행
  if (publish_meta_ && meta_pub_) {
    multi_agent_msgs::msg::AgentLayerMetaArray arr;
    arr.header.frame_id = target_frame;
    arr.header.stamp = node_shared_->now();

    for (size_t k = 0; k < meta_hits.size(); k += std::max(1, meta_stride_)) {
      auto [mx, my] = meta_hits[k];
      double wx, wy; master_grid.mapToWorld(mx, my, wx, wy);
      multi_agent_msgs::msg::AgentLayerCellMeta cm;
      cm.header = arr.header;
      cm.machine_id = 0; 
      cm.position.x = wx; cm.position.y = wy; cm.position.z = 0.0;
      cm.mx = mx; cm.my = my;
      arr.cells.emplace_back(std::move(cm));
    }
    meta_pub_->publish(std::move(arr));
  }
}

} // namespace multi_agent_nav2

// Export Plugin
PLUGINLIB_EXPORT_CLASS(multi_agent_nav2::AgentLayer, nav2_costmap_2d::Layer)