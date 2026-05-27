#include "multi_agent_nav2/agent_layer.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <nav2_costmap_2d/footprint.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <robot_interfaces/msg/agent_status.hpp>
#include <robot_interfaces/msg/agent_layer_cell_meta.hpp>
#include "tf2_ros/buffer.h"
#include <cstring> 

namespace multi_agent_nav2
{

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
AgentLayer::getFootprintForAgent(const robot_interfaces::msg::MultiAgentInfo & a)
{
    geometry_msgs::msg::PolygonStamped fp_stamped;
    
    auto it = agent_footprints_.find(a.machine_id);
    if (it == agent_footprints_.end()) {
        RCLCPP_WARN_ONCE(logger_, 
          "No footprint data found in YAML for machine_id %u. Cannot draw agent.",
          a.machine_id);
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

bool AgentLayer::transformAgentInfo(
    const robot_interfaces::msg::MultiAgentInfo & agent_in_map,
    robot_interfaces::msg::MultiAgentInfo & agent_in_costmap_frame,
    const std::string & costmap_frame) const
{
  const std::string& map_frame = last_infos_->header.frame_id;

  if (map_frame.empty()) return false;

  if (map_frame == costmap_frame) {
    agent_in_costmap_frame = agent_in_map;
    return true;
  }

  agent_in_costmap_frame = agent_in_map; 
  agent_in_costmap_frame.truncated_path.poses.clear();

  rclcpp::Time latest_time(0);

  try {
    geometry_msgs::msg::PoseStamped pose_to_transform = agent_in_map.current_pose;
    pose_to_transform.header.frame_id = map_frame;
    pose_to_transform.header.stamp = latest_time; 

    geometry_msgs::msg::PoseStamped transformed_pose;
    tf_->transform(pose_to_transform, transformed_pose, costmap_frame);
    agent_in_costmap_frame.current_pose = transformed_pose;

    for (const auto& pose_stamped_in_map : agent_in_map.truncated_path.poses) {
      geometry_msgs::msg::PoseStamped path_pose_to_transform;
      path_pose_to_transform.header.frame_id = map_frame;
      path_pose_to_transform.header.stamp = latest_time; 
      path_pose_to_transform.pose = pose_stamped_in_map.pose; 

      geometry_msgs::msg::PoseStamped pose_in_costmap_frame;
      tf_->transform(path_pose_to_transform, pose_in_costmap_frame, costmap_frame);
      
      agent_in_costmap_frame.truncated_path.poses.push_back(pose_in_costmap_frame);
    }

    agent_in_costmap_frame.truncated_path.header.frame_id = costmap_frame;
    agent_in_costmap_frame.truncated_path.header.stamp = transformed_pose.header.stamp;
    return true;

  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(logger_, *node_shared_->get_clock(), 2000,
      "Failed to transform agent %u from '%s' to '%s': %s",
      agent_in_map.machine_id, map_frame.c_str(), costmap_frame.c_str(), ex.what());
    return false;
  }
}

AgentLayer::AgentLayer() {}

void AgentLayer::onInitialize()
{
  node_shared_ = node_.lock();
  if (!node_shared_) {
    throw std::runtime_error("AgentLayer: failed to lock lifecycle node");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("/multi_agent_infos")));
  declareParameter("self_machine_id", rclcpp::ParameterValue(0));
  declareParameter("self_type_id", rclcpp::ParameterValue(std::string("")));
  declareParameter("use_path_header_frame", rclcpp::ParameterValue(true));
  declareParameter("roi_range_m", rclcpp::ParameterValue(12.0));
  declareParameter("time_decay_sec", rclcpp::ParameterValue(1.0));
  declareParameter("lethal_cost", rclcpp::ParameterValue(254));
  declareParameter("moving_cost", rclcpp::ParameterValue(254));
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

  // ========================================================
  // 경로용 파라미터 등록
  // ========================================================
  declareParameter("path_dilation_m", rclcpp::ParameterValue(-0.01));
  declareParameter("path_base_cost", rclcpp::ParameterValue(254));
  declareParameter("path_end_cost", rclcpp::ParameterValue(254));
  declareParameter("ignore_higher_machine_id_path", rclcpp::ParameterValue(true));
  declareParameter("load_rear_smear_m", rclcpp::ParameterValue(0.4));


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
  
  { int tmp = 254; node_shared_->get_parameter(name_ + "." + "lethal_cost", tmp); lethal_cost_ = static_cast<unsigned char>(std::clamp(tmp, 0, 254)); }
  { int tmp = 180; node_shared_->get_parameter(name_ + "." + "moving_cost", tmp); moving_cost_ = static_cast<unsigned char>(std::clamp(tmp, 0, 254)); }
  { int tmp = 200; node_shared_->get_parameter(name_ + "." + "waiting_cost", tmp); waiting_cost_ = static_cast<unsigned char>(std::clamp(tmp, 0, 254)); }
  
  node_shared_->get_parameter(name_ + "." + "manual_cost_bias", manual_cost_bias_);
  node_shared_->get_parameter(name_ + "." + "dilation_m", dilation_m_);
  node_shared_->get_parameter(name_ + "." + "forward_smear_m", forward_smear_m_);
  node_shared_->get_parameter(name_ + "." + "sigma_k", sigma_k_);

  node_shared_->get_parameter(name_ + "." + "path_dilation_m", path_dilation_m_);
  node_shared_->get_parameter(name_ + "." + "path_base_cost", path_base_cost_);
  node_shared_->get_parameter(name_ + "." + "path_end_cost", path_end_cost_);

  node_shared_->get_parameter(name_ + "." + "publish_meta", publish_meta_);
  node_shared_->get_parameter(name_ + "." + "meta_stride", meta_stride_);
  node_shared_->get_parameter(name_ + "." + "freshness_timeout_ms", freshness_timeout_ms_);
  node_shared_->get_parameter(name_ + "." + "max_poses", max_poses_);
  node_shared_->get_parameter(name_ + "." + "qos_reliable", qos_reliable_);

  node_shared_->get_parameter(name_ + "." + "ignore_higher_machine_id_path", ignore_higher_machine_id_path_);
  node_shared_->get_parameter(name_ + "." + "load_rear_smear_m", load_rear_smear_m_);



  declareParameter("robot_ids", rclcpp::ParameterValue(std::vector<std::string>({})));

  std::vector<std::string> robot_ids;
  node_shared_->get_parameter(name_ + "." + "robot_ids", robot_ids);

  for (const auto & id_str : robot_ids) {
    declareParameter(id_str + ".machine_id", rclcpp::ParameterValue(0));
    declareParameter(id_str + ".robot_radius", rclcpp::ParameterValue(0.0));
    declareParameter(id_str + ".footprint", rclcpp::ParameterValue(std::string("[]")));
  }

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

      double max_dist = 0.0;
      for (const auto& p : footprint_points) {
        max_dist = std::max(max_dist, std::hypot(p.x, p.y));
      }
      data.radius = max_dist;
    } else {
      data.use_radius = true;
    }
    agent_footprints_[machine_id] = data;
  }

  RCLCPP_INFO(logger_, "AgentLayer initialized. Body dilation: %.2f, Path dilation: %.2f", dilation_m_, path_dilation_m_);

  viz_costmap_.setDefaultValue(0); 
  costmap_pub_ = std::make_unique<nav2_costmap_2d::Costmap2DPublisher>(
      node_shared_, &viz_costmap_, layered_costmap_->getGlobalFrameID(), 
      name_ + "/raw_costmap", true);

  // ====== [NEW] Dynamic parameter callback 등록 ======
  dyn_params_handler_ = node_shared_->add_on_set_parameters_callback(
      std::bind(&AgentLayer::dynamicParametersCallback, this, std::placeholders::_1));
  // ===================================================

  current_ = true;
  matchSize();
}

void AgentLayer::activate()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  if (qos_reliable_) qos.reliable(); else qos.best_effort();

  sub_ = node_shared_->create_subscription<robot_interfaces::msg::MultiAgentInfoArray>(
      topic_, qos, std::bind(&AgentLayer::infosCallback, this, std::placeholders::_1));

  if (publish_meta_) {
    meta_pub_ = node_shared_->create_publisher<robot_interfaces::msg::AgentLayerMetaArray>(
        "agent_layer_meta", rclcpp::QoS(1).reliable().transient_local());
  }

  if (costmap_pub_) costmap_pub_->on_activate();
}

void AgentLayer::deactivate()
{
  // ====== [NEW] Callback handle 해제 ======
  // node가 살아있을 때 안전하게 해제
  auto node = node_shared_;
  if (node && dyn_params_handler_) {
    node->remove_on_set_parameters_callback(dyn_params_handler_.get());
  }
  dyn_params_handler_.reset();
  // =========================================

  sub_.reset();
  meta_pub_.reset();
  if (costmap_pub_) costmap_pub_->on_deactivate();
}

void AgentLayer::infosCallback(const robot_interfaces::msg::MultiAgentInfoArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(data_mtx_);
  last_infos_ = msg;
  last_stamp_ = msg->header.stamp;
}

bool AgentLayer::stale(const rclcpp::Time & stamp) const
{
  return (node_shared_->now() - stamp) >
         rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(freshness_timeout_ms_) * 1000000LL);
}

bool AgentLayer::isSelf(const robot_interfaces::msg::MultiAgentInfo & a) const
{
  return (a.machine_id == self_machine_id_) && (a.type_id == self_type_id_);
}

unsigned char AgentLayer::computeCost(const robot_interfaces::msg::MultiAgentInfo & a) const
{
  using S = robot_interfaces::msg::AgentStatus;
  const uint8_t p = a.status.phase;
  const bool is_moving = (p == S::STATUS_MOVING) || (p == S::STATUS_PATH_SEARCHING);

  unsigned char base = is_moving ? moving_cost_ : waiting_cost_;
  if (a.mode == "manual") {
    int c = static_cast<int>(base) + manual_cost_bias_;
    return static_cast<unsigned char>(std::clamp(c, 0, 254));
  }
  return base;
}

double AgentLayer::computeDilation(const robot_interfaces::msg::MultiAgentInfo & a) const
{
  using S = robot_interfaces::msg::AgentStatus;
  const uint8_t phase = a.status.phase;
  double r = dilation_m_;

  switch (phase) {
    case S::STATUS_AUTORECOVERY:
    case S::STATUS_ERROR:
    case S::STATUS_PAUSE:
    case S::STATUS_WAITING_FOR_SAFETY:
    case S::STATUS_WAITING_FOR_FLOWCONTROL:
    case S::STATUS_WAITING_FOR_ROS_STATUS:
    case S::STATUS_LOADING:
    case S::STATUS_UNLOADING:
    case S::STATUS_UNLOADED:
    case S::STATUS_LOADED:    
      r = std::max(r, 0.0);
      break;

    case S::STATUS_RECOVERING:
    case S::STATUS_UNKNOWN:
    case S::STATUS_MANUAL_RUNNING:
    case S::STATUS_MANUAL_COMPLETE:
      r = std::max(r, 0.05); 
      break;
    default:
      break;
  }
  return r;
}

double AgentLayer::computeRearSmear(const robot_interfaces::msg::MultiAgentInfo & a) const
{
  using S = robot_interfaces::msg::AgentStatus;
  const uint8_t phase = a.status.phase;
  if (phase == S::STATUS_LOADING || phase == S::STATUS_UNLOADING) {
    return load_rear_smear_m_;
  }
  return 0.0;
}



void AgentLayer::updateBounds(double robot_x, double robot_y, double /*robot_yaw*/,
                              double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_) return;

  // [NEW] 파라미터 변경 race 방지
  std::lock_guard<std::mutex> param_lock(param_mtx_);

  transformed_agents_.clear();
  cached_robot_x_ = robot_x;
  cached_robot_y_ = robot_y;

  if (last_touched_) {
    *min_x = std::min(*min_x, last_min_x_);
    *min_y = std::min(*min_y, last_min_y_);
    *max_x = std::max(*max_x, last_max_x_);
    *max_y = std::max(*max_y, last_max_y_);
  }

  touched_ = false;
  touch_min_x_ =  1e9; touch_min_y_ =  1e9;
  touch_max_x_ = -1e9; touch_max_y_ = -1e9;

  std::vector<robot_interfaces::msg::MultiAgentInfo> infos;
  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    if (!last_infos_ || stale(last_stamp_)) return;
    infos.assign(last_infos_->agents.begin(), last_infos_->agents.end());
  }

  const std::string & costmap_frame = layered_costmap_->getGlobalFrameID();

  for (const auto & a_map : infos) { 
    if (isSelf(a_map)) continue;

    robot_interfaces::msg::MultiAgentInfo a; 
    if (!transformAgentInfo(a_map, a, costmap_frame)) continue; 

    const double dx = a.current_pose.pose.position.x - robot_x;
    const double dy = a.current_pose.pose.position.y - robot_y;
    if (std::hypot(dx, dy) > roi_range_m_) continue;

    transformed_agents_.push_back(a);

    double base_radius = 0.5; // fallback
    auto it = agent_footprints_.find(a.machine_id);
    if (it != agent_footprints_.end()) {
        base_radius = it->second.radius;
    }

    // 본체(body)와 경로(path)의 bounding box 확장 반경을 서로 분리
    double actual_body_dilation = computeDilation(a);
    double actual_smear = isMovingPhase(a.status.phase) ? forward_smear_m_ : 0.0;

    double rear_smear = computeRearSmear(a);   // [NEW]

    // body_extent에 rear 확장분 포함 (등방 bbox라 큰 쪽으로 통일)
    double body_extent = base_radius + actual_body_dilation
                       + std::max(actual_smear, rear_smear) + 0.1;  // [수정]    

    double path_extent = base_radius + std::max(0.0, path_dilation_m_) + 0.1;

    // 본체 Bounds
    {
      const auto & p = a.current_pose.pose.position;
      touch_min_x_ = std::min(touch_min_x_, p.x - body_extent);
      touch_min_y_ = std::min(touch_min_y_, p.y - body_extent);
      touch_max_x_ = std::max(touch_max_x_, p.x + body_extent);
      touch_max_y_ = std::max(touch_max_y_, p.y + body_extent);
      touched_ = true;
    }

    // 경로 Bounds
    const int limit = std::min<int>(a.truncated_path.poses.size(), max_poses_);
    for (int i = 0; i < limit; ++i) {
      const auto & p = a.truncated_path.poses[i].pose.position; 
      touch_min_x_ = std::min(touch_min_x_, p.x - path_extent);
      touch_min_y_ = std::min(touch_min_y_, p.y - path_extent);
      touch_max_x_ = std::max(touch_max_x_, p.x + path_extent);
      touch_max_y_ = std::max(touch_max_y_, p.y + path_extent);
      touched_ = true;
    }
  }

  if (touched_) {
    *min_x = std::min(*min_x, touch_min_x_);
    *min_y = std::min(*min_y, touch_min_y_);
    *max_x = std::max(*max_x, touch_max_x_);
    *max_y = std::max(*max_y, touch_max_y_);

    last_min_x_ = touch_min_x_ - 0.1;
    last_min_y_ = touch_min_y_ - 0.1;
    last_max_x_ = touch_max_x_ + 0.1;
    last_max_y_ = touch_max_y_ + 0.1;
    last_touched_ = true;
  } else {
    last_touched_ = false;
  }
}

// ========================================================================
// 다각형 꼬임(Bow-tie) 방지를 위한 안전한 음수 팽창 로직
// ========================================================================
static inline std::vector<geometry_msgs::msg::Point>
dilateFootprintDirectional(const std::vector<geometry_msgs::msg::Point32> & in,
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

void AgentLayer::fillFootprintAt(const geometry_msgs::msg::PolygonStamped & fp,
                                 const geometry_msgs::msg::Pose & pose,
                                 double extra_dilation_m,
                                 double forward_len_m,
                                 double rear_len_m,            // [NEW]
                                 nav2_costmap_2d::Costmap2D * grid,
                                 unsigned char cost,
                                 std::vector<std::pair<unsigned int,unsigned int>> * meta_hits)
{
  auto poly = dilateFootprintDirectional(
      fp.polygon.points, extra_dilation_m, forward_len_m, rear_len_m,  // [NEW]
      logger_, node_shared_->get_clock());

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

// ========================================================================
// 본체와 경로 팽창 분리 및 경로 선형 코스트 감소
// ========================================================================
void AgentLayer::rasterizeAgentPath(
  const robot_interfaces::msg::MultiAgentInfo & a,
  nav2_costmap_2d::Costmap2D * grid,
  std::vector<std::pair<unsigned int,unsigned int>> & meta_hits)
{
  geometry_msgs::msg::PolygonStamped fp = getFootprintForAgent(a);
  if (fp.polygon.points.empty()) {
    return; 
  }

  const unsigned char cost_now = computeCost(a);
  const double body_iso_extra = computeDilation(a);
  const double forward_len = isMovingPhase(a.status.phase) ? forward_smear_m_ : 0.0;  

  const double rear_len = computeRearSmear(a);   // [NEW]

  // 1. 에이전트 본체 그리기
  fillFootprintAt(fp, a.current_pose.pose, body_iso_extra, forward_len,
                  rear_len,                       // [NEW]
                  grid, cost_now, &meta_hits);

  // =========================================================================
  // 우선순위(ID 비교) 기반 경로 반영 로직
  // =========================================================================
  if ((self_machine_id_ < a.machine_id) && ignore_higher_machine_id_path_) {
     return;
  }

  // 2. 미래 경로 그리기 (경로 전용 파라미터 적용)
  const int limit = std::min<int>(a.truncated_path.poses.size(), max_poses_);
  for (int i = 0; i < limit; ++i) {
    auto ps = a.truncated_path.poses[i].pose;
    ps.orientation = a.current_pose.pose.orientation;

    // 선형적인 코스트 감소(Linear Cost Decay)
    double ratio = (limit > 1) ? static_cast<double>(i) / (limit - 1) : 0.0;
    int decayed_cost_int = path_base_cost_ - static_cast<int>((path_base_cost_ - path_end_cost_) * ratio);
    
    unsigned char decay_cost = static_cast<unsigned char>(std::clamp(decayed_cost_int, 0, 255));

    if (decay_cost == 0) continue;
    
    // [NEW] 경로 팽창에도 후방 smear 0.0 적용
    fillFootprintAt(fp, ps, path_dilation_m_, 0.0,
                    0.0,                          // [NEW] rear 미적용
                    grid, decay_cost, &meta_hits);
  }
}

void AgentLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                             int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
  if (!enabled_) return;

  // [NEW] 파라미터 변경 race 방지
  std::lock_guard<std::mutex> param_lock(param_mtx_);

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
      
      unsigned char* char_map = viz_costmap_.getCharMap();
      unsigned int size_x = viz_costmap_.getSizeInCellsX();
      unsigned int size_y = viz_costmap_.getSizeInCellsY();
      std::memset(char_map, 0, size_x * size_y * sizeof(unsigned char));
  }

  std::vector<std::pair<unsigned int,unsigned int>> meta_hits;
  meta_hits.reserve(256);
  std::vector<std::pair<unsigned int,unsigned int>> dummy_hits; 

  for (const auto & a : transformed_agents_) { 
    rasterizeAgentPath(a, &master_grid, meta_hits);
    rasterizeAgentPath(a, &viz_costmap_, dummy_hits);
  }

  if (costmap_pub_) {
      costmap_pub_->updateBounds(0, viz_costmap_.getSizeInCellsX(),
                                 0, viz_costmap_.getSizeInCellsY());
      costmap_pub_->publishCostmap();
  }

  if (publish_meta_ && meta_pub_) {
    robot_interfaces::msg::AgentLayerMetaArray arr;
    arr.header.frame_id = layered_costmap_->getGlobalFrameID();
    arr.header.stamp = node_shared_->now();

    for (size_t k = 0; k < meta_hits.size(); k += std::max(1, meta_stride_)) {
      auto [mx, my] = meta_hits[k];
      double wx, wy; master_grid.mapToWorld(mx, my, wx, wy);
      robot_interfaces::msg::AgentLayerCellMeta cm;
      cm.header = arr.header;
      cm.machine_id = 0;
      cm.position.x = wx; cm.position.y = wy; cm.position.z = 0.0;
      cm.mx = mx; cm.my = my;
      arr.cells.emplace_back(std::move(cm));
    }
    meta_pub_->publish(std::move(arr));
  }
}

// ========================================================================
// [NEW] Dynamic parameter callback 구현
// ========================================================================
rcl_interfaces::msg::SetParametersResult
AgentLayer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // updateBounds / updateCosts 와의 race 방지
  std::lock_guard<std::mutex> lock(param_mtx_);

  for (const auto & param : parameters) {
    const auto & full_name = param.get_name();
    const auto type = param.get_type();

    // 우리 layer 의 파라미터만 처리 (이름 prefix 확인: "<name_>.")
    const std::string prefix = name_ + ".";
    if (full_name.rfind(prefix, 0) != 0) {
      continue;  // 다른 layer/노드 파라미터는 패스
    }
    const std::string key = full_name.substr(prefix.size());

    try {
      if (type == rclcpp::ParameterType::PARAMETER_BOOL) {
        if (key == "enabled") {
          enabled_ = param.as_bool();
          current_ = false;  // 다음 cycle에서 재계산 트리거
        } else if (key == "use_path_header_frame") {
          use_path_header_frame_ = param.as_bool();
        } else if (key == "publish_meta") {
          publish_meta_ = param.as_bool();
        } else if (key == "qos_reliable") {
          // QoS는 subscription 재생성이 필요하므로 경고만
          RCLCPP_WARN(logger_,
            "qos_reliable is set, but it requires re-activation to take effect.");
          qos_reliable_ = param.as_bool();
        } else if (key == "ignore_higher_machine_id_path") {
          ignore_higher_machine_id_path_ = param.as_bool();
        }
      }
      else if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (key == "roi_range_m") {
          if (param.as_double() < 0.0) {
            result.successful = false;
            result.reason = "roi_range_m must be >= 0";
            return result;
          }
          roi_range_m_ = param.as_double();
        } else if (key == "time_decay_sec") {
          time_decay_sec_ = param.as_double();
        } else if (key == "dilation_m") {
          dilation_m_ = param.as_double();
        } else if (key == "forward_smear_m") {
          forward_smear_m_ = std::max(0.0, param.as_double());
        } else if (key == "sigma_k") {
          sigma_k_ = param.as_double();
        } else if (key == "path_dilation_m") {
          path_dilation_m_ = param.as_double();
        } else if (key == "load_rear_smear_m") {
          load_rear_smear_m_ = std::max(0.0, param.as_double());
        }
      }
      else if (type == rclcpp::ParameterType::PARAMETER_INTEGER) {
        const int v = static_cast<int>(param.as_int());

        if (key == "self_machine_id") {
          if (v < 0 || v > 65535) {
            result.successful = false;
            result.reason = "self_machine_id out of uint16 range";
            return result;
          }
          self_machine_id_ = static_cast<uint16_t>(v);
        } else if (key == "lethal_cost") {
          lethal_cost_ = static_cast<unsigned char>(std::clamp(v, 0, 254));
        } else if (key == "moving_cost") {
          moving_cost_ = static_cast<unsigned char>(std::clamp(v, 0, 254));
        } else if (key == "waiting_cost") {
          waiting_cost_ = static_cast<unsigned char>(std::clamp(v, 0, 254));
        } else if (key == "manual_cost_bias") {
          manual_cost_bias_ = v;
        } else if (key == "path_base_cost") {
          path_base_cost_ = std::clamp(v, 0, 254);
        } else if (key == "path_end_cost") {
          path_end_cost_ = std::clamp(v, 0, 254);
        } else if (key == "meta_stride") {
          meta_stride_ = std::max(1, v);
        } else if (key == "freshness_timeout_ms") {
          freshness_timeout_ms_ = std::max(0, v);
        } else if (key == "max_poses") {
          max_poses_ = std::max(0, v);
        }
      }
      else if (type == rclcpp::ParameterType::PARAMETER_STRING) {
        if (key == "self_type_id") {
          self_type_id_ = param.as_string();
        } else if (key == "topic") {
          // topic도 subscription 재생성이 필요
          RCLCPP_WARN(logger_,
            "topic change requires re-activation to take effect.");
          topic_ = param.as_string();
        }
      }

      RCLCPP_INFO(logger_, "[AgentLayer] Param updated: %s", full_name.c_str());
    }
    catch (const std::exception & e) {
      result.successful = false;
      result.reason = std::string("Exception while setting param: ") + e.what();
      return result;
    }
  }

  return result;
}

} // namespace multi_agent_nav2

// pluginlib export
PLUGINLIB_EXPORT_CLASS(multi_agent_nav2::AgentLayer, nav2_costmap_2d::Layer)
