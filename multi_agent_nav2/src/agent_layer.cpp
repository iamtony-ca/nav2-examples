#include "multi_agent_nav2/agent_layer.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>

// [NEW] For makeFootprintFromString and makeFootprintFromRadius
#include <nav2_costmap_2d/footprint.hpp>

#include <geometry_msgs/msg/point32.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <multi_agent_msgs/msg/agent_status.hpp>
#include <multi_agent_msgs/msg/agent_layer_cell_meta.hpp>

// #include <cmath>
// #include <algorithm>
// #include <utility>


namespace multi_agent_nav2
{

// [NEW] Implementation of static helper
std::vector<geometry_msgs::msg::Point32> AgentLayer::toPoint32(
    const std::vector<geometry_msgs::msg::Point>& points)
{
    std::vector<geometry_msgs::msg::Point32> points32;
    points32.reserve(points.size());
    for (const auto& p : points) {
        geometry_msgs::msg::Point32 p32;
        p32.x = static_cast<float>(p.x);
        p32.y = static_cast<float>(p.y);
        p32.z = 0.0f; // Z is not used in 2D footprint
        points32.push_back(p32);
    }
    return points32;
}

// [NEW] Implementation of helper to get footprint from loaded map
geometry_msgs::msg::PolygonStamped 
AgentLayer::getFootprintForAgent(const multi_agent_msgs::msg::MultiAgentInfo & a)
{
    geometry_msgs::msg::PolygonStamped fp_stamped;
    
    // Find the data loaded from YAML
    auto it = agent_footprints_.find(a.machine_id);
    if (it == agent_footprints_.end()) {
        // If not found, log a warning once and return empty
        RCLCPP_WARN_ONCE(logger_, 
          "No footprint data found in YAML for machine_id %u. Cannot draw agent.",
          a.machine_id);
        return fp_stamped; // Return empty
    }

    const auto& data = it->second;

    if (data.use_radius) {
        // Generate circular footprint from radius
        std::vector<geometry_msgs::msg::Point> points = 
            nav2_costmap_2d::makeFootprintFromRadius(data.radius);
        fp_stamped.polygon.points = toPoint32(points);
    } else {
        // Use pre-loaded polygon points
        fp_stamped.polygon.points = data.points;
    }

    // Header is mostly for convention; frame_id should be local (base_link)
    fp_stamped.header.frame_id = "base_link";
    fp_stamped.header.stamp = a.current_pose.header.stamp;
    return fp_stamped;
}



AgentLayer::AgentLayer() {}

void AgentLayer::onInitialize()
{
  node_shared_ = node_.lock();
  if (!node_shared_) {
    throw std::runtime_error("AgentLayer: failed to lock lifecycle node");
  }
// --- [수정] ---
  // Layer의 내장 헬퍼를 사용하거나, name_을 수동으로 붙입니다.
  // Layer의 내장 헬퍼(declareParameter)를 사용하는 것을 권장합니다.
  // 이 함수는 내부적으로 name_을 처리해 줍니다.
  
  // declareParameter는 base class (Layer)의 헬퍼 함수를 사용합니다.
  // 이 함수는 내부적으로 name_을 붙여줍니다. (node_shared_-> 불필요)
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("/multi_agent_infos")));
  declareParameter("self_machine_id", rclcpp::ParameterValue(0));
  declareParameter("self_type_id", rclcpp::ParameterValue(std::string("")));
  declareParameter("use_path_header_frame", rclcpp::ParameterValue(true));
  declareParameter("roi_range_m", rclcpp::ParameterValue(12.0));
  declareParameter("time_decay_sec", rclcpp::ParameterValue(1.0));
  declareParameter("lethal_cost", rclcpp::ParameterValue(254));
  declareParameter("moving_cost", rclcpp::ParameterValue(254)); // 180
  declareParameter("waiting_cost", rclcpp::ParameterValue(254));
  declareParameter("manual_cost_bias", rclcpp::ParameterValue(30));
  declareParameter("dilation_m", rclcpp::ParameterValue(0.05));
  declareParameter("forward_smear_m", rclcpp::ParameterValue(0.005));
  declareParameter("sigma_k", rclcpp::ParameterValue(2.0));
  declareParameter("publish_meta", rclcpp::ParameterValue(true));
  declareParameter("meta_stride", rclcpp::ParameterValue(3));
  declareParameter("freshness_timeout_ms", rclcpp::ParameterValue(800));
  declareParameter("max_poses", rclcpp::ParameterValue(10000));
  declareParameter("qos_reliable", rclcpp::ParameterValue(true));


  // Get parameters
  // get_parameter는 헬퍼가 없으므로, node_shared_-> 와 'name_' 을 명시적으로 사용합니다.
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


// [NEW] Add declaration for the robot list
  declareParameter("robot_ids", rclcpp::ParameterValue(std::vector<std::string>({})));

  // [NEW] Get the list of robot IDs
  std::vector<std::string> robot_ids;
  node_shared_->get_parameter(name_ + "." + "robot_ids", robot_ids);

  // [NEW] Loop 1: Declare all sub-parameters for each robot_id
  for (const auto & id_str : robot_ids) {
    std::string id_ns = name_ + "." + id_str; // e.g., "agent_layer.robot_001"
    // declareParameter(id_ns + ".type_id", rclcpp::ParameterValue(std::string(""))); // Not strictly needed
    declareParameter(id_ns + ".machine_id", rclcpp::ParameterValue(0));
    declareParameter(id_ns + ".robot_radius", rclcpp::ParameterValue(0.0));
    declareParameter(id_ns + ".footprint", rclcpp::ParameterValue(std::string("[]")));
  }

  // [NEW] Loop 2: Get parameters and populate the map
  agent_footprints_.clear();
  for (const auto & id_str : robot_ids) {
    std::string id_ns = name_ + "." + id_str;
    
    int machine_id_int = 0;
    node_shared_->get_parameter(id_ns + ".machine_id", machine_id_int);
    if (machine_id_int == 0) continue; // Skip invalid ID

    uint16_t machine_id = static_cast<uint16_t>(machine_id_int);
    
    AgentFootprintData data;
    std::string footprint_str;
    node_shared_->get_parameter(id_ns + ".footprint", footprint_str);
    node_shared_->get_parameter(id_ns + ".robot_radius", data.radius);

    std::vector<geometry_msgs::msg::Point> footprint_points;
    // Use nav2_costmap_2d helper to parse footprint string
    if (nav2_costmap_2d::makeFootprintFromString(footprint_str, footprint_points) &&
        footprint_points.size() >= 3)
    {
      data.points = toPoint32(footprint_points); // Convert Point to Point32
      data.use_radius = false;
    } else {
      data.use_radius = true;
      // data.radius is already set
      RCLCPP_INFO(logger_, 
        "Using radius (%.2f) for machine_id %u (footprint string: '%s')",
        data.radius, machine_id, footprint_str.c_str());
    }

    agent_footprints_[machine_id] = data;
    
    RCLCPP_INFO(logger_, 
      "Loaded footprint for machine_id %u: use_radius=%s, points=%zu",
      machine_id, (data.use_radius ? "true" : "false"), data.points.size());
  }




  RCLCPP_INFO(
      logger_,  // Layer 기본 클래스에서 상속받은 logger_ 사용
      "AgentLayer '%s' initialized: self_machine_id=%u, self_type_id='%s', moving_cost=%u",
      name_.c_str(),
      self_machine_id_,
      self_type_id_.c_str(),
      static_cast<unsigned int>(moving_cost_)
    );


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

// [CHANGED] 등방성(모든 방향) 팽창만 반환. 전방 스미어는 여기서 제외!
double AgentLayer::computeDilation(const multi_agent_msgs::msg::MultiAgentInfo & a) const
{
  double r = dilation_m_;
  // if (a.pos_std_m >= 0.0) r += sigma_k_ * a.pos_std_m;   // need to edit
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

    // 현재 위치 + 트렁케이트 경로를 모두 bounds에 반영
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

// === 내부 헬퍼: 로컬 프레임에서 등방성 + 전방(+x) 스미어 적용 ===
// [NEW]
static inline std::vector<geometry_msgs::msg::Point>
dilateFootprintDirectional(const std::vector<geometry_msgs::msg::Point32> & in,
                           double iso_dilate_m,
                           double forward_len_m)
{
  std::vector<geometry_msgs::msg::Point> out; out.reserve(in.size());
  if (in.empty()) return out;

  // 로컬 폴리곤의 중심
  double cx=0, cy=0;
  for (auto & p : in) { cx += p.x; cy += p.y; }
  cx /= static_cast<double>(in.size());
  cy /= static_cast<double>(in.size());

  for (auto & p : in) {
    // 1) 등방성(모든 방향) 기본 여유
    double vx = p.x - cx, vy = p.y - cy;
    double n = std::hypot(vx, vy); if (n < 1e-6) n = 1.0;

    double x_local = p.x + iso_dilate_m * (vx / n);
    double y_local = p.y + iso_dilate_m * (vy / n);

    // 2) 전방(+x) 여유: 중심 기준 전방측 점들만 +x로 평행 이동
    if (forward_len_m > 1e-6 && (p.x - cx) >= 0.0) {
      x_local += forward_len_m;
    }

    geometry_msgs::msg::Point q;
    q.x = x_local; q.y = y_local; q.z = 0.0;
    out.push_back(q);
  }
  return out;
}

// [CHANGED] forward_len_m을 인자로 받아 전방(+x)으로만 스미어 적용
void AgentLayer::fillFootprintAt(const geometry_msgs::msg::PolygonStamped & fp,
                                 const geometry_msgs::msg::Pose & pose,
                                 double extra_dilation_m,
                                 double forward_len_m,   // [NEW]
                                 nav2_costmap_2d::Costmap2D * grid,
                                 unsigned char cost,
                                 std::vector<std::pair<unsigned int,unsigned int>> * meta_hits)
{
  // 1) 로컬 폴리곤을 등방성 + 전방(+x)으로만 늘리기
  auto poly = dilateFootprintDirectional(fp.polygon.points, extra_dilation_m, forward_len_m);

  // 2) 로컬→월드 변환
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
    minx = std::min(minx, p.x);
    miny = std::min(miny, p.y);
    maxx = std::max(maxx, p.x);
    maxy = std::max(maxy, p.y);
  }

  // 4) bbox → map index
  int min_i, min_j, max_i, max_j;
  grid->worldToMapEnforceBounds(minx, miny, min_i, min_j);
  grid->worldToMapEnforceBounds(maxx, maxy, max_i, max_j);

  // 5) 래스터 채우기 (Max-merge)
  for (int j = min_j; j <= max_j; ++j) {
    for (int i = min_i; i <= max_i; ++i) {
      double wx, wy; grid->mapToWorld(i, j, wx, wy);

      // ray-casting (point-in-polygon)
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
        // const unsigned char old = grid->getCost(i, j);
        // if (cost > old) {
        // 치사 또는 강한 footprint 코스트: TrueOverwrite(footprint 우선) 느낌으로 덮되
        // Max-merge로도 충분히 강함
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



/* ****************************************
 * [AFTER] agent_layer.cpp - rasterizeAgentPath()
 * ****************************************
 */
void AgentLayer::rasterizeAgentPath(
  const multi_agent_msgs::msg::MultiAgentInfo & a,
  nav2_costmap_2d::Costmap2D * grid,
  std::vector<std::pair<unsigned int,unsigned int>> & meta_hits)
{
  // [NEW] Get footprint from YAML map using machine_id
  geometry_msgs::msg::PolygonStamped fp = getFootprintForAgent(a);
  if (fp.polygon.points.empty()) {
    return; // No footprint found (logged in helper), skip rasterizing this agent
  }

  // 코스트 & 등방성 팽창
  const unsigned char cost_now = computeCost(a);
  const double iso_extra = computeDilation(a);

  // 이동 여부에 따라 전방 스미어 적용
  const double forward_len = isMovingPhase(a.status.phase) ? forward_smear_m_ : 0.0;

  // 1) 에이전트 현재 footprint 찍기 (전방 스미어 조건부 적용)
  fillFootprintAt(fp, a.current_pose.pose, iso_extra, forward_len, // [CHANGED]
                  grid, cost_now, &meta_hits);

  // 2) (선택) truncated_path의 각 pose에서도 footprint를 얇게/간격 띄워서 찍고 싶다면
  //    아래 루프를 활성화하세요. 지금은 과도한 차단을 피하기 위해 "현재 위치만" 반영.
  //
  const int limit = std::min<int>(a.truncated_path.poses.size(), max_poses_);
  for (int i = 0; i < limit; ++i) {
    const auto & ps = a.truncated_path.poses[i].pose;
    // 경로상의 footprint는 등방성만 소량(예: iso_extra*0.5), 전방 스미어는 0.0로 권장
    fillFootprintAt(fp, ps, iso_extra * 0.5, 0.0, grid, cost_now, &meta_hits); // [CHANGED]
  }
}



// // [CHANGED] 이동 중이면 forward_smear_m_ 사용, 아니면 0.0
// void AgentLayer::rasterizeAgentPath(
//   const multi_agent_msgs::msg::MultiAgentInfo & a,
//   nav2_costmap_2d::Costmap2D * grid,
//   std::vector<std::pair<unsigned int,unsigned int>> & meta_hits)
// {
//   // 코스트 & 등방성 팽창
//   const unsigned char cost_now = computeCost(a);
//   const double iso_extra = computeDilation(a);

//   // 이동 여부에 따라 전방 스미어 적용
//   const double forward_len = isMovingPhase(a.status.phase) ? forward_smear_m_ : 0.0;

//   // 1) 에이전트 현재 footprint 찍기 (전방 스미어 조건부 적용)
//   fillFootprintAt(a.footprint, a.current_pose.pose, iso_extra, forward_len,
//                   grid, cost_now, &meta_hits);

//   // 2) (선택) truncated_path의 각 pose에서도 footprint를 얇게/간격 띄워서 찍고 싶다면
//   //    아래 루프를 활성화하세요. 지금은 과도한 차단을 피하기 위해 "현재 위치만" 반영.
//   //
//   const int limit = std::min<int>(a.truncated_path.poses.size(), max_poses_);
//   for (int i = 0; i < limit; ++i) {
//     const auto & ps = a.truncated_path.poses[i].pose;
//     // 경로상의 footprint는 등방성만 소량(예: iso_extra*0.5), 전방 스미어는 0.0로 권장
//     fillFootprintAt(a.footprint, ps, iso_extra * 0.5, 0.0, grid, cost_now, &meta_hits);
//   }
// }

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

} // namespace multi_agent_nav2

// pluginlib export
PLUGINLIB_EXPORT_CLASS(multi_agent_nav2::AgentLayer, nav2_costmap_2d::Layer)
