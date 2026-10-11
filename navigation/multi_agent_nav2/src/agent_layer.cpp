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
  // [FIX AL-1] 배열 header 가 아니라 항목 자신의 frame 을 본다 (캐시가 항목 단위라
  // 배열 header 는 더 이상 없고, 예전 코드는 data_mtx_ 밖에서 배열을 읽고 있었다).
  const std::string& map_frame = agent_in_map.header.frame_id;

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
  // [V2]
  declareParameter("tube_horizon_sec", rclcpp::ParameterValue(4.0));
  declareParameter("tube_min_m", rclcpp::ParameterValue(0.5));
  declareParameter("tube_max_m", rclcpp::ParameterValue(4.0));
  declareParameter("agent_moving_mps", rclcpp::ParameterValue(0.05));
  declareParameter("paint_stopped_agent_path", rclcpp::ParameterValue(false));
  // [10-10 W1-2] mask publish 방식. false = 칠한 사각형만 지우고 CostmapUpdate 로 바뀐 사각형만 보냄,
  //   true = 예전 그대로 (매 사이클 viz 전체 memset + 전체 full publish).
  // [10-10 W1-3] S2-OPS-1/W2-1: code 기본값 false → **true**. 새 .so 를 params 변경 없이 넣어도 예전 path_validator
  //   (full topic 만 구독) 의 mask 가 얼지 않게 한다. update 방식은 두 agent_layer block 에 full_mask_publish: false 를 명시해
  //   켠다 (W1-3 path_validator 가 들어간 뒤에만 — W1_STEP3.md 배포 순서).
  declareParameter("full_mask_publish", rclcpp::ParameterValue(true));
  // [10-10 W1-2] update 사각형에 넣을 직전 사이클 수 (1 = 직전 ∪ 이번). 2 이상이면 연속 유실 N-1 개를 다음 update 가 메운다.
  declareParameter("mask_update_history", rclcpp::ParameterValue(1));
  // [10-10 W1-2][리뷰] 이 사이클 수마다 최근 두 구간에 보낸 사각형 전체를 한 번 더 보낸다 (0 = 끔). global 5 Hz 면 10 = 2 s.
  declareParameter("mask_refresh_cycles", rclcpp::ParameterValue(10));


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
  node_shared_->get_parameter(name_ + "." + "tube_horizon_sec", tube_horizon_sec_);
  node_shared_->get_parameter(name_ + "." + "tube_min_m", tube_min_m_);
  node_shared_->get_parameter(name_ + "." + "tube_max_m", tube_max_m_);
  node_shared_->get_parameter(name_ + "." + "agent_moving_mps", agent_moving_mps_);
  node_shared_->get_parameter(name_ + "." + "paint_stopped_agent_path", paint_stopped_agent_path_);
  RCLCPP_INFO(logger_, "[AgentLayer V2] time-aware tube: horizon %.1f s, %.1f~%.1f m, moving > %.2f m/s, stopped path %s",
              tube_horizon_sec_, tube_min_m_, tube_max_m_, agent_moving_mps_, paint_stopped_agent_path_ ? "painted" : "body only");
  node_shared_->get_parameter(name_ + "." + "load_rear_smear_m", load_rear_smear_m_);
  node_shared_->get_parameter(name_ + "." + "full_mask_publish", full_mask_publish_);      // [10-10 W1-2]
  node_shared_->get_parameter(name_ + "." + "mask_update_history", mask_update_history_);  // [10-10 W1-2]
  mask_update_history_ = std::max(1, mask_update_history_);
  node_shared_->get_parameter(name_ + "." + "mask_refresh_cycles", mask_refresh_cycles_);  // [10-10 W1-2][리뷰]
  mask_refresh_cycles_ = std::max(0, mask_refresh_cycles_);



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
  // [10-10 W1-2] 예전: always_send_full=true 하드코딩 → 사이클마다 master 크기 full (2e8 cells 지도면 200 MB).
  //   이제 기본은 false: full 은 첫 publish·크기/원점 변경 때만 (구독자가 있을 때), 그 밖에는 updateCosts 가 넘긴
  //   사각형만 <topic>_raw_updates (nav2_msgs/CostmapUpdate) 로 보낸다. service 이름은 그대로
  //   "get_" + name_ + "/raw_costmap" (global costmap 노드 기준 /global_costmap/get_agent_layer/raw_costmap).
  mask_canvas_.setLegacyFullClear(full_mask_publish_);
  mask_canvas_.setPublishHistory(static_cast<size_t>(mask_update_history_));
  mask_canvas_.setRefreshCycles(mask_refresh_cycles_);   // [10-10 W1-2][리뷰]
  costmap_pub_ = std::make_unique<nav2_costmap_2d::Costmap2DPublisher>(
      node_shared_, &viz_costmap_, layered_costmap_->getGlobalFrameID(), 
      name_ + "/raw_costmap", full_mask_publish_);
  RCLCPP_INFO(logger_,
    "[10-10 W1-2] AgentLayer mask publish: %s (update history %d, refresh %d cycles) [10-10 W1-3 code 기본값 true]",
    full_mask_publish_ ? "full_mask_publish=true — 예전 방식 (매 사이클 전체 memset + full)"
                       : "full_mask_publish=false — 칠한 사각형만 지우고 CostmapUpdate 로 바뀐 사각형만 보냄 "
                         "(W1-3 path_validator 필요)",
    mask_update_history_, mask_refresh_cycles_);

  // ====== [NEW] Dynamic parameter callback 등록 ======
  dyn_params_handler_ = node_shared_->add_on_set_parameters_callback(
      std::bind(&AgentLayer::dynamicParametersCallback, this, std::placeholders::_1));
  // ===================================================

  current_ = true;
  matchSize();
}

void AgentLayer::activate()
{
  // [10-10 W1-b] 사용자 결정 (권장대로): deactivate() 가 dynamic parameter callback 을 지우므로
  //   Nav2 lifecycle pause → resume (RViz Navigation2 panel Pause/Resume) 뒤에는 다시 등록해야 한다.
  //   예전 (amhs 원본 포함) 에는 다시 등록하지 않아 roi_range_manager 의 agent_layer.roi_range_m set 이
  //   성공으로 돌아오지만 적용되지 않았고, full_mask_publish 등 read-only guard 도 사라졌다.
  //   첫 activate (onInitialize 직후) 는 이미 등록돼 있으므로 건너뛴다.
  if (!dyn_params_handler_ && node_shared_) {
    dyn_params_handler_ = node_shared_->add_on_set_parameters_callback(
        std::bind(&AgentLayer::dynamicParametersCallback, this, std::placeholders::_1));
    RCLCPP_INFO(logger_, "[10-10 W1-b] AgentLayer 재활성화: dynamic parameter callback 다시 등록");
  }

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  if (qos_reliable_) qos.reliable(); else qos.best_effort();

  // [10-10 W1-3] W2-N1: 구독을 layer 의 callback_group_ 에 둔다 (fork ObstacleLayer 와 같은 방식, obstacle_layer.cpp:131-132).
  //   예전에는 costmap 노드의 default group 이라 planner_server 의 costmap_thread_ (NodeThread, planner_server.cpp:97) 가
  //   GetCostmap(200 MB 응답)·clear_entirely·parameter service 와 한 SingleThreadedExecutor 로 돌렸다 → 200 MB 응답을 만드는
  //   동안 /multi_agent_infos 를 받지 못하고, 800 ms 를 넘기면 updateBounds 가 이웃을 cache 에서 지웠다.
  //   callback_group_ 은 Costmap2DROS::on_configure 가 automatically_add_to_executor_with_node=false 로 만들어
  //   (costmap_2d_ros.cpp:155-156) 전용 executor_ 에 넣고 executor_thread_ 가 돌린다 (:268-270). plugin initialize 가 넘겨
  //   준다 (:189-190). activate() 는 그 executor 가 이미 돌 때 불리지만, ObstacleLayer::activate 도 같은 group 에 구독을 다시
  //   만든다 (obstacle_layer.cpp:717-726 → message_filters subscribe(sub_opt)). rclcpp(Jazzy 28.x) 는 group 에 entity 를
  //   더하면 group 의 notify guard condition 을 올려 executor 가 다시 모은다 (callback_group.hpp:259-264).
  //   순서상 결과: infosCallback 은 이제 그 전용 executor thread 에서 돌고 (MutuallyExclusive), 같은 group 의 다른 layer
  //   (obstacle/zed/denoise 의 point cloud·scan 콜백, tf CreateTimerROS timer) 와 줄을 선다. map update thread 와는 예전처럼
  //   data_mtx_ 로만 만난다. dynamic parameter callback·GetCostmap·clear service 는 default group 에 그대로 남는다.
  //   callback_group_ 이 비어 있으면 (initialize 를 group 없이 부른 경우) 예전처럼 default group 이다.
  rclcpp::SubscriptionOptions sub_opt;
  sub_opt.callback_group = callback_group_;
  sub_ = node_shared_->create_subscription<robot_interfaces::msg::MultiAgentInfoArray>(
      topic_, qos, std::bind(&AgentLayer::infosCallback, this, std::placeholders::_1), sub_opt);
  RCLCPP_INFO(logger_, "[10-10 W1-3] AgentLayer %s 구독: %s",
    topic_.c_str(), callback_group_ ? "layer callback_group (Costmap2DROS 전용 executor)" : "default group (callback_group 없음)");

  if (publish_meta_) {
    meta_pub_ = node_shared_->create_publisher<robot_interfaces::msg::AgentLayerMetaArray>(
        "agent_layer_meta", rclcpp::QoS(1).reliable().transient_local());
  }

  if (costmap_pub_) costmap_pub_->on_activate();
  // [10-10 W1-2][리뷰] 다음 updateCosts 는 grid 전체 사각형을 update 로 보낸다 (layer 헤더의 force_full_rect_ 주석).
  //   reactivate 전 deactivate 가 publish 도중에 들어와 버려진 update 가 있어도 구독자 mask 가 여기서 다시 맞는다.
  force_full_rect_.store(true);

  // [10-10 W1-3] S2-OPS-1/W2-1 배포 guard: update 방식인데 full-only 소비자만 듣고 있으면 WARN (checkLegacyMaskListener).
  activated_at_ = std::chrono::steady_clock::now();
  legacy_listener_warned_ = false;
  if (!full_mask_publish_) {
    legacy_listener_timer_ = node_shared_->create_wall_timer(
      std::chrono::seconds(30), std::bind(&AgentLayer::checkLegacyMaskListener, this), callback_group_);
  }
}

// [10-10 W1-3] activation 120 s 뒤부터 30 s 마다: <name>/raw_costmap_raw 구독자는 있는데 _raw_updates 구독자가 없으면 WARN.
//   예전 path_validator (amhs·pv_w1) 와 IsGoalsOccupiedAgentCondition 은 full topic 만 구독한다. full 은 첫 publish·geometry
//   변경 때만 나가므로 그 소비자의 mask 는 얼어붙는다 (현장 순서면 처음부터 비어 있다). 동작은 바꾸지 않고 알리기만 한다.
void AgentLayer::checkLegacyMaskListener()
{
  if (!node_shared_ || full_mask_publish_) {
    return;
  }
  const double since = std::chrono::duration<double>(std::chrono::steady_clock::now() - activated_at_).count();
  const std::string base = name_ + "/raw_costmap";
  size_t raw_subs = 0, upd_subs = 0;
  try {
    raw_subs = node_shared_->count_subscribers(base + "_raw");
    upd_subs = node_shared_->count_subscribers(base + "_raw_updates");
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(logger_, "[10-10 W1-3] 구독자 수 조회 실패: %s", e.what());
    return;
  }
  if (legacyMaskListenerSuspected(full_mask_publish_, since, 120.0, raw_subs, upd_subs)) {
    if (!legacy_listener_warned_) {
      legacy_listener_warned_ = true;
      RCLCPP_WARN(logger_,
        "[10-10 W1-3] full_mask_publish=false 인데 %s_raw 구독자 %zu, %s_raw_updates 구독자 0 — full topic 만 받는 예전 소비자"
        "(amhs/pv_w1 path_validator 등) 가 듣고 있다. 그 mask 는 갱신되지 않는다. W1-3 path_validator 를 넣거나 "
        "full_mask_publish: true 로 되돌리고 Nav2 재기동", base.c_str(), raw_subs, base.c_str());
    } else {
      RCLCPP_WARN_THROTTLE(logger_, *node_shared_->get_clock(), 300000,
        "[10-10 W1-3] (계속) %s_raw 구독자 %zu, _raw_updates 구독자 0 — full-only 소비자", base.c_str(), raw_subs);
    }
  } else if (legacy_listener_warned_ && upd_subs > 0) {
    legacy_listener_warned_ = false;
    RCLCPP_INFO(logger_, "[10-10 W1-3] %s_raw_updates 구독자 %zu — full-only 소비자 경고 해제", base.c_str(), upd_subs);
  }
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
  if (legacy_listener_timer_) {   // [10-10 W1-3]
    legacy_listener_timer_->cancel();
    legacy_listener_timer_.reset();
  }
  if (costmap_pub_) costmap_pub_->on_deactivate();
}

void AgentLayer::infosCallback(const robot_interfaces::msg::MultiAgentInfoArray::SharedPtr msg)
{
  // [FIX AL-1] 통째로 교체하지 않고 machine_id 별로 병합한다. (헤더의 agent_cache_ 주석 참고)
  // 만료(freshness_timeout_ms)는 updateBounds 에서 읽을 때 건다 — 병합만 하면
  // 떠난 이웃이 영원히 남는다.
  std::lock_guard<std::mutex> lk(data_mtx_);
  for (const auto & a_in : msg->agents) {
    CachedAgent & slot = agent_cache_[a_in.machine_id];
    slot.info = a_in;
    // 항목 header 가 비어 있으면 배열 header 로 채운다. transformAgentInfo 는
    // 이제 항목의 frame_id 만 보므로 여기서 반드시 채워져 있어야 한다.
    if (slot.info.header.frame_id.empty()) {
      slot.info.header.frame_id = msg->header.frame_id;
    }
    const rclcpp::Time item_stamp(a_in.header.stamp);
    slot.seen_at = (item_stamp.nanoseconds() > 0) ? item_stamp
                                                  : rclcpp::Time(msg->header.stamp);
    // [V2] 속도 추정: 0.6 s 이상 떨어진 이전 표본과의 변위 / 시간 (지수 평활)
    const rclcpp::Time now_rx = node_shared_->now();
    const double px = a_in.current_pose.pose.position.x, py = a_in.current_pose.pose.position.y;
    if (slot.hist_t.nanoseconds() == 0) {
      slot.hist_x = px; slot.hist_y = py; slot.hist_t = now_rx;
    } else {
      const double dt = (now_rx - slot.hist_t).seconds();
      if (dt >= 0.6) {
        const double v = std::hypot(px - slot.hist_x, py - slot.hist_y) / dt;
        slot.speed_mps = (slot.speed_mps < 0.0) ? v : (0.5 * slot.speed_mps + 0.5 * v);
        slot.hist_x = px; slot.hist_y = py; slot.hist_t = now_rx;
      } else if (dt < 0.0) {
        slot.hist_x = px; slot.hist_y = py; slot.hist_t = now_rx;   // 시계가 되돌아감 (sim 리셋)
      }
    }
  }
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
    // [FIX AL-1] 만료된 이웃은 로봇별로 걷어내고, 남은 전부를 그린다.
    // 예전에는 마지막 메시지 하나의 stamp 로 전체를 한 번에 버렸다.
    for (auto it = agent_cache_.begin(); it != agent_cache_.end(); ) {
      if (stale(it->second.seen_at)) {
        RCLCPP_WARN(logger_,
          "[AgentLayer] agent %u 소식 끊김 (%d ms). 캐시에서 제거한다.",
          static_cast<unsigned>(it->first), freshness_timeout_ms_);
        it = agent_cache_.erase(it);
      } else {
        infos.push_back(it->second.info);
        agent_speed_[it->first] = it->second.speed_mps;   // [V2]
        ++it;
      }
    }
    if (infos.empty()) return;
  }

  const std::string & costmap_frame = layered_costmap_->getGlobalFrameID();



  for (const auto & a_map : infos) { 
    if (isSelf(a_map)) continue;

    robot_interfaces::msg::MultiAgentInfo a; 
    if (!transformAgentInfo(a_map, a, costmap_frame)) continue; 

    // 본체가 ROI 안인지
    const bool body_in_roi =
        inRoi(a.current_pose.pose.position.x, a.current_pose.pose.position.y);

    // ROI 안에 들어오는 path 점이 하나라도 있는지 미리 확인
    const int limit = std::min<int>(a.truncated_path.poses.size(), max_poses_);
    bool any_path_in_roi = false;
    for (int i = 0; i < limit; ++i) {
      const auto & p = a.truncated_path.poses[i].pose.position;
      if (inRoi(p.x, p.y)) { any_path_in_roi = true; break; }
    }

    // 본체도 path도 ROI 밖이면 이 agent는 통째로 스킵
    if (!body_in_roi && !any_path_in_roi) continue;

    transformed_agents_.push_back(a);

    double base_radius = 0.5; // fallback
    auto it = agent_footprints_.find(a.machine_id);
    if (it != agent_footprints_.end()) {
        base_radius = it->second.radius;
    }

    double actual_body_dilation = computeDilation(a);
    double actual_smear = isMovingPhase(a.status.phase) ? forward_smear_m_ : 0.0;
    double rear_smear = computeRearSmear(a);
    double body_extent = base_radius + actual_body_dilation
                       + std::max(actual_smear, rear_smear) + 0.1;
    double path_extent = base_radius + std::max(0.0, path_dilation_m_) + 0.1;

    // 본체 Bounds (본체가 ROI 안일 때만)
    if (body_in_roi) {
      const auto & p = a.current_pose.pose.position;
      touch_min_x_ = std::min(touch_min_x_, p.x - body_extent);
      touch_min_y_ = std::min(touch_min_y_, p.y - body_extent);
      touch_max_x_ = std::max(touch_max_x_, p.x + body_extent);
      touch_max_y_ = std::max(touch_max_y_, p.y + body_extent);
      touched_ = true;
    }

    // 경로 Bounds (ROI 안의 path 점만)
    for (int i = 0; i < limit; ++i) {
      const auto & p = a.truncated_path.poses[i].pose.position; 
      if (!inRoi(p.x, p.y)) continue;   // [핵심] ROI 밖 path 점 제외
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

// [10-10 W1-2] dilateFootprintDirectional 은 agent_mask_canvas.cpp 로 옮겼다 (내용 동일, 시험에서 같은 코드를 쓰려고).

void AgentLayer::fillFootprintAt(const geometry_msgs::msg::PolygonStamped & fp,
                                 const geometry_msgs::msg::Pose & pose,
                                 double extra_dilation_m,
                                 double forward_len_m,
                                 double rear_len_m,            // [NEW]
                                 AgentMaskCanvas & canvas,     // [10-10 W1-2]
                                 unsigned char cost,
                                 std::vector<std::pair<unsigned int,unsigned int>> * meta_hits)
{
  // [10-10 W1-2] 래스터화 본문은 rasterizeFootprint (agent_mask_canvas.cpp) 로 옮겼다. 계산은 그대로이고
  //   grid 쓰기만 canvas.paintMax 로 바꿔 실제로 쓴 셀의 사각형을 기록한다.
  const WorldBox box = rasterizeFootprint(
      fp.polygon.points, pose, extra_dilation_m, forward_len_m, rear_len_m,
      canvas, cost, meta_hits, logger_, node_shared_->get_clock());
  const double minx = box.minx, miny = box.miny, maxx = box.maxx, maxy = box.maxy;

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
  AgentMaskCanvas & canvas,   // [10-10 W1-2]
  std::vector<std::pair<unsigned int,unsigned int>> & meta_hits)
{
  geometry_msgs::msg::PolygonStamped fp = getFootprintForAgent(a);
  if (fp.polygon.points.empty()) {
    return; 
  }

  const unsigned char cost_now = computeCost(a);
  const double body_iso_extra = computeDilation(a);
  const double forward_len = isMovingPhase(a.status.phase) ? forward_smear_m_ : 0.0;
  const double rear_len = computeRearSmear(a);

  // 1. 본체: current_pose가 ROI 안일 때만 그리기
  if (inRoi(a.current_pose.pose.position.x, a.current_pose.pose.position.y)) {
    fillFootprintAt(fp, a.current_pose.pose, body_iso_extra, forward_len,
                    rear_len, canvas, cost_now, &meta_hits);   // [10-10 W1-2]
  }

  // 우선순위(ID 비교) 기반 경로 반영 로직
  // 단, 상대가 MARKING 상태이면 ID와 무관하게 path를 반영
  using S = robot_interfaces::msg::AgentStatus;
  const bool is_marking = (a.status.phase == S::STATUS_MARKING);

  if (!is_marking &&
      (self_machine_id_ < a.machine_id) && ignore_higher_machine_id_path_) {
     return;
  }
    
  // // 우선순위(ID 비교) 기반 경로 반영 로직
  // if ((self_machine_id_ < a.machine_id) && ignore_higher_machine_id_path_) {
  //    return;
  // }

  // [V2] 시간 인지형 튜브: 정지 상대는 몸체만, 주행 상대는 속도×horizon 길이만
  double paint_len = tube_max_m_;
  {
    auto sit = agent_speed_.find(a.machine_id);
    const double sp = (sit != agent_speed_.end()) ? sit->second : -1.0;
    if (sp >= 0.0) {
      if (sp < agent_moving_mps_ && !paint_stopped_agent_path_) {
        return;                                       // 정지 상대: 경로 튜브 없음
      }
      paint_len = std::clamp(sp * tube_horizon_sec_, tube_min_m_, tube_max_m_);
    }
  }
  double along = 0.0;

  // 2. 미래 경로: ROI 안의 점만 그리기
  const int limit = std::min<int>(a.truncated_path.poses.size(), max_poses_);
  for (int i = 0; i < limit; ++i) {
    const auto & ppos = a.truncated_path.poses[i].pose.position;
    if (i > 0) {
      const auto & prev = a.truncated_path.poses[i - 1].pose.position;
      along += std::hypot(ppos.x - prev.x, ppos.y - prev.y);
      if (along > paint_len) break;                   // [V2] horizon 밖은 칠하지 않는다
    }
    if (!inRoi(ppos.x, ppos.y)) continue;   // [핵심] ROI 밖 path 점 제외

    auto ps = a.truncated_path.poses[i].pose;
    ps.orientation = a.current_pose.pose.orientation;

    double ratio = (limit > 1) ? static_cast<double>(i) / (limit - 1) : 0.0;
    int decayed_cost_int = path_base_cost_ - static_cast<int>((path_base_cost_ - path_end_cost_) * ratio);
    unsigned char decay_cost = static_cast<unsigned char>(std::clamp(decayed_cost_int, 0, 254));

    if (decay_cost == 0) continue;

    fillFootprintAt(fp, ps, path_dilation_m_, 0.0, 0.0, canvas, decay_cost, &meta_hits);   // [10-10 W1-2]
  }
}



void AgentLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                             int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) return;

  // RCLCPP_INFO_THROTTLE(logger_, *node_shared_->get_clock(), 1000,
  //   "[AgentLayer] updateCosts called, agents=%zu, bounds=[%d,%d,%d,%d]",
  //   transformed_agents_.size(), min_i, min_j, max_i, max_j);
  // std::lock_guard<std::mutex> param_lock(param_mtx_);

  // 1) viz_costmap 크기/원점 동기화 + 직전 그림 지우기
  //    [10-10 W1-2] 예전: 매 사이클 viz 전체 memset (master 크기, 2e8 cells 지도면 200 MB).
  //    이제: 직전 사이클에 실제로 칠한 사각형만 지운다 (크기/원점이 바뀌면 resizeMap 이 전체를 0 으로 채움).
  //    full_mask_publish=true 이면 예전 그대로 전체 memset.
  mask_canvas_.beginCycle(master_grid);

  // 2) 우리 cost는 viz_costmap_에만 그린다 (master_grid에는 직접 안 씀)
  std::vector<std::pair<unsigned int,unsigned int>> meta_hits;
  meta_hits.reserve(256);

  for (const auto & a : transformed_agents_) {
    rasterizeAgentPath(a, mask_canvas_, meta_hits);   // [10-10 W1-2]
  }

  // 3) bounds 영역만큼 viz_costmap_ -> master_grid로 OR 합성
  //    (max() 방식으로 합성: 하위 레이어가 얹은 cost를 보존하면서 우리 cost 추가)
  //    [10-10 W1-2] bounds ∩ 이번에 칠한 사각형만 본다 (밖은 모두 0 이라 예전에도 건너뛰던 셀). legacy 면 bounds 전체.
  mask_canvas_.mergeInto(master_grid, min_i, min_j, max_i, max_j);

  // 4) 시각화 publish
  //    [10-10 W1-2] 예전: updateBounds(0..size) + always_send_full publisher → 매 사이클 full.
  //    이제: 직전 ∪ 이번 사각형만 bounds 로 넘긴다 → publisher 가 그 사각형만 CostmapUpdate 로 보낸다 (O(칠한 면적)).
  //    사각형이 비면 updateBounds 를 부르지 않는다. 그러면 publishCostmap 은 x0_ < xn_ 이 거짓이라 아무것도 보내지
  //    않는다 (크기/원점이 바뀐 사이클이면 bounds 와 무관하게 full 을 보낸다).
  //    [10-10 W1-2][리뷰] 사각형은 canvas.finishCycle() 이 정한다 (legacy = grid 전체, 평소 = 직전 ∪ 이번,
  //    refresh 사이클 = + 최근 두 구간, activate 뒤 첫 사이클 = grid 전체). 접착 코드는 시험과 같은 publishCycle().
  if (force_full_rect_.exchange(false) && !full_mask_publish_) {
    mask_canvas_.requestFullRect();
    // [10-10 W1-3] W2-N2: 로그를 실제 동작에 맞춘다. 새 instance (bringup·respawn·bond reset) 의 첫 사이클은 beginCycle 이
    //   resize 한 사이클이고, publisher 는 geometry 변경으로 full 경로를 탄다 (_raw 구독자가 있을 때만 full 하나). 그때 이
    //   사각형은 쓰이지 않는다 (fork publisher 의 초기화 안 된 geometry 멤버가 우연히 같을 때만 update 로 나간다 — 그 경우를
    //   덮으려고 요청은 그대로 둔다, 비용 없음). 같은 instance 의 deactivate→activate (resize 없음) 만 grid 전체 update 를
    //   만든다 (update 구독자가 있을 때만, 2e8 cells 면 master lock 안에서 약 1 s).
    if (mask_canvas_.lastStats().resized) {
      RCLCPP_INFO(logger_,
        "[10-10 W1-3] activate 뒤 첫 사이클이 resize 사이클 (%ux%u) — publisher 가 full 경로를 타므로 강제 update 는 생략된다 "
        "(full 은 _raw 구독자가 있을 때만)",
        viz_costmap_.getSizeInCellsX(), viz_costmap_.getSizeInCellsY());
    } else {
      RCLCPP_INFO(logger_,
        "[10-10 W1-2][리뷰] activate 뒤 첫 사이클 (재활성화): mask grid 전체(%ux%u)를 update 로 한 번 보낸다 "
        "(update 구독자가 있을 때만 만든다)",
        viz_costmap_.getSizeInCellsX(), viz_costmap_.getSizeInCellsY());
    }
  }
  if (costmap_pub_) {
    publishCycle(mask_canvas_, *costmap_pub_);
  } else {
    mask_canvas_.finishCycle();
  }

  // 5) meta publish (기존 그대로) — meta_hits는 viz_costmap_ 기준으로 채워졌으므로 좌표는 동일
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

  // [10-10 W1-2][리뷰] 재시작해야 적용되는 key 는 값을 바꾸려 하면 거절한다 (예전: WARN 만 남기고 successful=true 라
  //   parameter server 에는 새 값이 저장돼 `ros2 param get` 이 실제와 다른 방식을 보여 줬다). 같은 값은 받아 준다.
  //   다른 key 를 하나도 적용하기 전에 먼저 검사한다 (거절된 묶음의 일부만 멤버에 반영되지 않도록).
  {
    const std::string prefix = name_ + ".";
    for (const auto & param : parameters) {
      const auto & full_name = param.get_name();
      if (full_name.rfind(prefix, 0) != 0) {
        continue;
      }
      const std::string key = full_name.substr(prefix.size());
      bool same = true;
      if (key == "full_mask_publish" && param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        same = (param.as_bool() == full_mask_publish_);
      } else if (key == "mask_update_history" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        same = (std::max<int64_t>(1, param.as_int()) == mask_update_history_);
      } else if (key == "mask_refresh_cycles" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        same = (std::max<int64_t>(0, param.as_int()) == mask_refresh_cycles_);
      }
      if (!same) {
        result.successful = false;
        result.reason = key + " 는 재시작해야 적용 (read-only)";
        RCLCPP_WARN(logger_, "[10-10 W1-2][리뷰] %s 변경 거절: 재시작해야 적용된다 (read-only)", full_name.c_str());
        return result;
      }
    }
  }

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
        // [10-10 W1-2][리뷰] full_mask_publish 는 위에서 같은 값만 통과한다 (publisher 생성자 인자라 실행 중 변경 불가).
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
        // [10-10 W1-2][리뷰] mask_update_history / mask_refresh_cycles 는 위에서 같은 값만 통과한다
        //   (updateCosts 는 param_mtx_ 를 쥐지 않으므로 실행 중에는 canvas 설정을 바꾸지 않는다).
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