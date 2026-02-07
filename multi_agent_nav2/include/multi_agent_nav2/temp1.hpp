#pragma once

#include <mutex>
#include <vector>
#include <string>
#include <utility>
#include <algorithm>
#include <cmath>
#include <map>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

// [Core] Layer 상속 (CostmapLayer가 아님)
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <multi_agent_msgs/msg/multi_agent_info_array.hpp>
#include <multi_agent_msgs/msg/agent_layer_meta_array.hpp>
#include <multi_agent_msgs/msg/agent_status.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace multi_agent_nav2
{

// [CHANGED] CostmapLayer 대신 가벼운 Layer 상속
class AgentLayer : public nav2_costmap_2d::Layer
{
public:
  AgentLayer();
  virtual ~AgentLayer();

  // Lifecycle
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override;

  // Costmap callbacks
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;

  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                   int min_i, int min_j, int max_i, int max_j) override;

  bool isClearable() override { return true; }

private:
  // Node handle
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_shared_;

  // I/O
  rclcpp::Subscription<multi_agent_msgs::msg::MultiAgentInfoArray>::SharedPtr sub_;
  rclcpp::Publisher<multi_agent_msgs::msg::AgentLayerMetaArray>::SharedPtr meta_pub_;
  
  // [REMOVED] costmap_pub_ (Removed internal publisher)

  // Last data
  std::mutex data_mtx_;
  multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr last_infos_;
  rclcpp::Time last_stamp_;

  // Parameters
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

  bool        publish_meta_{true};
  int         meta_stride_{3};
  int         freshness_timeout_ms_{800};
  int         max_poses_{40};
  bool        qos_reliable_{true};

  // [REMOVED] last_min_x_ logic (Optimization C1 removed for stability)

  // Bounds cache for current cycle
  double touch_min_x_{0.0}, touch_min_y_{0.0}, touch_max_x_{0.0}, touch_max_y_{0.0};
  bool   touched_{false};

  // Cached robot pose
  double cached_robot_x_{0.0};
  double cached_robot_y_{0.0};

  // Footprint cache
  struct AgentFootprintData
  {
    std::vector<geometry_msgs::msg::Point32> points;
    double radius{0.0};
    bool use_radius{true};
  };
  std::map<uint16_t, AgentFootprintData> agent_footprints_;

  // Helpers
  geometry_msgs::msg::PolygonStamped 
  getFootprintForAgent(const multi_agent_msgs::msg::MultiAgentInfo & a);

  static std::vector<geometry_msgs::msg::Point32> toPoint32(
      const std::vector<geometry_msgs::msg::Point>& points);

  bool transformAgentInfo(
      const multi_agent_msgs::msg::MultiAgentInfo & agent_in_map,
      multi_agent_msgs::msg::MultiAgentInfo & agent_in_costmap_frame,
      const std::string & source_frame_id,
      const std::string & target_frame_id) const;

  void infosCallback(const multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr msg);
  bool isSelf(const multi_agent_msgs::msg::MultiAgentInfo & a) const;
  bool stale(const rclcpp::Time & stamp) const;

  void rasterizeAgentPath(const multi_agent_msgs::msg::MultiAgentInfo & a,
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

  double computeDilation(const multi_agent_msgs::msg::MultiAgentInfo & a) const;
  unsigned char computeCost(const multi_agent_msgs::msg::MultiAgentInfo & a) const;

  static inline bool isMovingPhase(uint8_t phase)
  {
    using S = multi_agent_msgs::msg::AgentStatus;
    return phase == S::STATUS_MOVING || phase == S::STATUS_PATH_SEARCHING;
  }
};

} // namespace multi_agent_nav2