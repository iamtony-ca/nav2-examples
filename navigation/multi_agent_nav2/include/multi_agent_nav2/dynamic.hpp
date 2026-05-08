#pragma once

#include <mutex>
#include <vector>
#include <string>
#include <utility>
#include <algorithm>
#include <cmath>
#include <map>
#include <memory> // unique_ptr

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_publisher.hpp> 

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <robot_interfaces/msg/multi_agent_info_array.hpp>
#include <robot_interfaces/msg/agent_layer_meta_array.hpp>
#include <robot_interfaces/msg/agent_status.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// [NEW] Dynamic parameter callback 지원
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace multi_agent_nav2
{

class AgentLayer : public nav2_costmap_2d::Layer
{
public:
  AgentLayer();

  // lifecycle
  void onInitialize() override;
  void activate() override;
  void deactivate() override;

  void reset() override { 
    current_ = true; 
    last_touched_ = false; 
    touched_ = false; 
  }

  // costmap callbacks
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;

  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                   int min_i, int min_j, int max_i, int max_j) override;

  bool isClearable() override { return true; }

private:
  // ====== [NEW] Dynamic parameter 지원 ======
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
      std::vector<rclcpp::Parameter> parameters);

  // 파라미터 변경과 updateBounds/updateCosts 간 race 방지용
  std::mutex param_mtx_;

  // Callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr 
      dyn_params_handler_;
  // ==========================================

  // node handle
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_shared_;

  // I/O
  rclcpp::Subscription<robot_interfaces::msg::MultiAgentInfoArray>::SharedPtr sub_;
  rclcpp::Publisher<robot_interfaces::msg::AgentLayerMetaArray>::SharedPtr meta_pub_;

  // 시각화 전용
  nav2_costmap_2d::Costmap2D viz_costmap_;
  std::unique_ptr<nav2_costmap_2d::Costmap2DPublisher> costmap_pub_;

  // last data
  std::mutex data_mtx_;
  robot_interfaces::msg::MultiAgentInfoArray::SharedPtr last_infos_;
  rclcpp::Time last_stamp_;

  // parameters
  bool        enabled_{true};
  std::string topic_{"/multi_agent_infos"};
  uint16_t    self_machine_id_{0};
  std::string self_type_id_{};
  bool        use_path_header_frame_{true};
  double      roi_range_m_{12.0};
  double      time_decay_sec_{1.0};
  unsigned char lethal_cost_{254};
  unsigned char moving_cost_{180};
  unsigned char waiting_cost_{200};
  int         manual_cost_bias_{30};

  double      dilation_m_{0.05};          
  double      forward_smear_m_{0.25};     
  double      sigma_k_{2.0};              

  // ========================================================
  // 경로(Path) 전용 파라미터
  // ========================================================
  double      path_dilation_m_{-0.1}; // 본체와 분리된 경로 전용 팽창값 (음수 허용)
  int         path_base_cost_{200};   // 로봇 바로 앞 경로 코스트
  int         path_end_cost_{50};     // 멀어질수록 도달하는 끝점 코스트
  // ========================================================

  bool        publish_meta_{true};
  int         meta_stride_{3};
  int         freshness_timeout_ms_{800};
  int         max_poses_{40};
  bool        qos_reliable_{true};

  // bounds cache for this cycle
  double touch_min_x_{0.0}, touch_min_y_{0.0}, touch_max_x_{0.0}, touch_max_y_{0.0};
  bool   touched_{false};

  // 이전 사이클의 Bounds (잔상 지우기용)
  double last_min_x_{1e9}, last_min_y_{1e9}, last_max_x_{-1e9}, last_max_y_{-1e9};
  bool   last_touched_{false};

  // Cached robot pose
  double cached_robot_x_{0.0};
  double cached_robot_y_{0.0};

  bool ignore_higher_machine_id_path_{true};

  // TF 시간차 캐시
  std::vector<robot_interfaces::msg::MultiAgentInfo> transformed_agents_;

  struct AgentFootprintData
  {
    std::vector<geometry_msgs::msg::Point32> points;
    double radius{0.0};
    bool use_radius{true};
  };
  std::map<uint16_t, AgentFootprintData> agent_footprints_;

  // Helpers
  geometry_msgs::msg::PolygonStamped 
  getFootprintForAgent(const robot_interfaces::msg::MultiAgentInfo & a);

  static std::vector<geometry_msgs::msg::Point32> toPoint32(
      const std::vector<geometry_msgs::msg::Point>& points);

  bool transformAgentInfo(
      const robot_interfaces::msg::MultiAgentInfo & agent_in_map,
      robot_interfaces::msg::MultiAgentInfo & agent_in_costmap_frame,
      const std::string & costmap_frame) const;

  void infosCallback(const robot_interfaces::msg::MultiAgentInfoArray::SharedPtr msg);
  bool isSelf(const robot_interfaces::msg::MultiAgentInfo & a) const;
  bool stale(const rclcpp::Time & stamp) const;

  void rasterizeAgentPath(const robot_interfaces::msg::MultiAgentInfo & a,
                          nav2_costmap_2d::Costmap2D * grid,
                          std::vector<std::pair<unsigned int,unsigned int>> & meta_hits);

  void fillFootprintAt(const geometry_msgs::msg::PolygonStamped & fp,
                       const geometry_msgs::msg::Pose & pose,
                       double extra_dilation_m,
                       double forward_len_m,
                       nav2_costmap_2d::Costmap2D * grid,
                       unsigned char cost,
                       std::vector<std::pair<unsigned int,unsigned int>> * meta_hits = nullptr);

  static bool pointInPolygon(const std::vector<geometry_msgs::msg::Point> & poly,
                             double x, double y);

  double computeDilation(const robot_interfaces::msg::MultiAgentInfo & a) const;

  unsigned char computeCost(const robot_interfaces::msg::MultiAgentInfo & a) const;

  static inline bool isMovingPhase(uint8_t phase)
  {
    using S = robot_interfaces::msg::AgentStatus;
    return phase == S::STATUS_MOVING || phase == S::STATUS_PATH_SEARCHING;
  }
};

} // namespace multi_agent_nav2
