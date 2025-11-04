#pragma once

#include <mutex>
#include <vector>
#include <string>
#include <utility>
#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

#include <multi_agent_msgs/msg/multi_agent_info_array.hpp>
#include <multi_agent_msgs/msg/agent_layer_meta_array.hpp>

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
  void reset() override { current_ = true; }

  // costmap callbacks
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;

  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                   int min_i, int min_j, int max_i, int max_j) override;

  bool isClearable() override { return true; }

private:
  // node handle (lock base weak_ptr)
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_shared_;

  // I/O
  rclcpp::Subscription<multi_agent_msgs::msg::MultiAgentInfoArray>::SharedPtr sub_;
  rclcpp::Publisher<multi_agent_msgs::msg::AgentLayerMetaArray>::SharedPtr meta_pub_;

  // last data
  std::mutex data_mtx_;
  multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr last_infos_;
  rclcpp::Time last_stamp_;

  // parameters (기본 동작)
  bool        enabled_{true};
  std::string topic_{"/multi_agent_infos"};
  uint16_t    self_machine_id_{0};
  std::string self_type_id_{};
  bool        use_path_header_frame_{true};
  double      roi_range_m_{12.0};
  double      time_decay_sec_{1.0}; // reserved
  unsigned char lethal_cost_{254};
  unsigned char moving_cost_{180};
  unsigned char waiting_cost_{200};
  int         manual_cost_bias_{30};
  double      dilation_m_{0.05};       // footprint 기본 팽창(안전 여유)
  double      forward_smear_m_{0.25};  // 이동중 footprint 전방 여유
  double      sigma_k_{2.0};           // 위치 표준편차 가중
  bool        publish_meta_{true};
  int         meta_stride_{3};
  int         freshness_timeout_ms_{800};
  int         max_poses_{40};
  bool        qos_reliable_{true};

  // 경로 소프트필드 튜닝(이번 수정 핵심)
  bool   soft_path_only_when_moving_{true}; // 이동 중일 때만 경로 코스트 생성
  double path_sigma_lat_m_{0.25};           // 경로 횡방향 표준편차(좁게)
  double path_lambda_long_m_{1.5};          // 경로 진행방향 감쇠 길이(짧게)
  int    path_cost_base_{180};              // 경로 코스트 바닥값(약하게)
  int    path_cost_cap_{230};               // 경로 코스트 상한(LETHAL 미만)
  double path_cone_boost_{0.0};             // 전방 콘 부스트(기본 0 → 비활성)

  // bounds cache for this cycle
  double touch_min_x_{0.0}, touch_min_y_{0.0}, touch_max_x_{0.0}, touch_max_y_{0.0};
  bool   touched_{false};

  // helpers
  void infosCallback(const multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr msg);
  bool isSelf(const multi_agent_msgs::msg::MultiAgentInfo & a) const;
  bool stale(const rclcpp::Time & stamp) const;

  void rasterizeAgentPath(const multi_agent_msgs::msg::MultiAgentInfo & a,
                          nav2_costmap_2d::Costmap2D * grid,
                          std::vector<std::pair<unsigned int,unsigned int>> & meta_hits);

  void fillFootprintAt(const geometry_msgs::msg::PolygonStamped & fp,
                       const geometry_msgs::msg::Pose & pose,
                       double extra_dilation_m,
                       nav2_costmap_2d::Costmap2D * grid,
                       unsigned char cost,
                       std::vector<std::pair<unsigned int,unsigned int>> * meta_hits = nullptr);

  static bool pointInPolygon(const std::vector<geometry_msgs::msg::Point> & poly,
                             double x, double y);

  unsigned char computeCost(const multi_agent_msgs::msg::MultiAgentInfo & a) const;
  double computeDilation(const multi_agent_msgs::msg::MultiAgentInfo & a) const;

  static inline bool isMovingPhase(uint8_t phase);
};

} // namespace multi_agent_nav2