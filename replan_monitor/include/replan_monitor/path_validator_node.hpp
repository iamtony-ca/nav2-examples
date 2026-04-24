#ifndef PATH_VALIDATOR_NODE_HPP
#define PATH_VALIDATOR_NODE_HPP

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <chrono>
#include <algorithm>

// [NEW] Added for std::map
#include <map>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_msgs/msg/costmap.hpp"

#include "multi_agent_msgs/msg/multi_agent_info_array.hpp"
#include "multi_agent_msgs/msg/multi_agent_info.hpp"
#include "multi_agent_msgs/msg/agent_status.hpp"
#include "multi_agent_msgs/msg/path_agent_collision_info.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace replan_monitor
{

struct ObstacleInfo
{
  rclcpp::Time first_seen;
  rclcpp::Time last_seen;
};

struct CostmapSignature
{
  unsigned int size_x{0};
  unsigned int size_y{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};

  bool operator==(const CostmapSignature & other) const
  {
    return size_x == other.size_x && size_y == other.size_y &&
           resolution == other.resolution &&
           origin_x == other.origin_x && origin_y == other.origin_y;
  }
};

class PathValidatorNode : public rclcpp::Node
{
public:
  PathValidatorNode();

private:
  // ========= Callbacks =========
  void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
  void agentMaskCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
  void robotStatusCallback(const std_msgs::msg::String::SharedPtr msg);
  void validatePathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void updateObstacleDatabase();
  void agentsCallback(const multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr msg);

  // ========= Helpers / Utils =========
  bool getCurrentPoseFromTF(geometry_msgs::msg::Pose & pose_out) const;
  bool transformToGlobal(const geometry_msgs::msg::PoseStamped & in,
                         geometry_msgs::msg::PoseStamped & out) const;
  void transformPathToGlobal(const nav_msgs::msg::Path & in,
                             std::vector<geometry_msgs::msg::PoseStamped> & out) const;

  inline uint64_t packKey(unsigned int mx, unsigned int my) const
  {
    return (static_cast<uint64_t>(mx) << 32) | static_cast<uint64_t>(my);
  }

  bool isBlockedCellKernel(unsigned int mx, unsigned int my) const;

  void validateWithFootprint(const std::vector<geometry_msgs::msg::PoseStamped> & gpath);
  void validateWithPoints(const std::vector<geometry_msgs::msg::PoseStamped> & gpath);
  void validatePathOptimized(const std::vector<geometry_msgs::msg::PoseStamped> & gpath);


  inline bool masterCellBlocked(unsigned int mx, unsigned int my, unsigned char thr) const;
  inline bool agentCellBlockedNear(unsigned int mx, unsigned int my,
                                   unsigned char thr, int manhattan_buf) const;

  static bool pointInPolygon(const std::vector<geometry_msgs::msg::Point> & poly,
                             double x, double y);

// ========= (NEW) Timer & Path Storage =========
  void validationTimerCallback();
  
  rclcpp::TimerBase::SharedPtr validation_timer_;
  std::vector<geometry_msgs::msg::PoseStamped> latest_global_path_;
  mutable std::mutex path_mutex_;


  // === Agent 충돌 식별 ===
  struct AgentHit {
    uint16_t machine_id{0};
    std::string type_id;
    double x{0.0}, y{0.0};
    float ttc_first{-1.0f};
    std::string note;
  };

  // [NEW] Map to store footprint data from YAML
  struct AgentFootprintData
  {
    // existing code uses Point32, so we store Point32
    std::vector<geometry_msgs::msg::Point32> points;
    double radius{0.0};
    bool use_radius{true};
  };
  // Map from machine_id to its footprint/radius data
  std::map<uint16_t, AgentFootprintData> agent_footprints_;

  // [NEW] Helper to get footprint for a given agent
  std::vector<geometry_msgs::msg::Point32> 
  getFootprintForAgent(const multi_agent_msgs::msg::MultiAgentInfo & a) const;

  // [NEW] Helper to convert nav2_costmap_2d::makeFootprint... results
  static std::vector<geometry_msgs::msg::Point32> toPoint32(
      const std::vector<geometry_msgs::msg::Point>& points);



  // wx, wy를 커버하는 agent의 footprint 또는 truncated_path 튜브를 찾아 리턴
  std::vector<AgentHit> whoCoversPoint(double wx, double wy) const;

  // 내부: 경로 튜브(footprint를 얇게 확장)에서 포함 여부 검사
  static bool pathTubeCoversPoint(const multi_agent_msgs::msg::MultiAgentInfo & a,
                                  double wx, double wy,
                                  double stride_m, double dilate_m,
                                  int max_poses, double frame_yaw,
                                  const std::string & global_frame);

  static double headingTo(const geometry_msgs::msg::Pose & pose, double wx, double wy);
  static double speedAlong(const geometry_msgs::msg::Twist & tw, double heading_rad);

  void triggerReplan(const std::string & reason);
  void publishAgentCollisionList(const std::vector<AgentHit> & hits);
// [NEW] 아무 충돌이 없을 때 안전 상태를 퍼블리시하는 함수
  void publishSafeStatus();


  std::vector<AgentHit> findNearestAgent(double wx, double wy, double max_allowed_dist) const;



  // ========= Callback Groups =========
  rclcpp::CallbackGroup::SharedPtr subs_callback_group_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

  // ========= ROS I/O =========
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pruned_path_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr agent_mask_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_status_sub_;
  rclcpp::Subscription<multi_agent_msgs::msg::MultiAgentInfoArray>::SharedPtr agents_sub_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr replan_pub_;
  rclcpp::Publisher<multi_agent_msgs::msg::PathAgentCollisionInfo>::SharedPtr agent_collision_pub_;

  rclcpp::TimerBase::SharedPtr obstacle_db_update_timer_;
  rclcpp::TimerBase::SharedPtr flag_reset_timer_;

  // ========= State =========
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  mutable std::mutex costmap_mutex_;
  CostmapSignature last_costmap_sig_;

  std::shared_ptr<nav2_costmap_2d::Costmap2D> agent_mask_;
  mutable std::mutex agent_mask_mutex_;
  CostmapSignature last_agent_sig_;

  // 최신 MultiAgentInfoArray
  multi_agent_msgs::msg::MultiAgentInfoArray::SharedPtr last_agents_;
  rclcpp::Time last_agents_stamp_;
  mutable std::mutex agents_mutex_;

  std::atomic<bool> is_robot_in_driving_state_{false};
  rclcpp::Time last_replan_time_;        // replan 쿨다운 기준
  rclcpp::Time last_agent_block_time_;   // 에이전트 홀드 기준

  std::unordered_map<uint64_t, ObstacleInfo> obstacle_db_;
  mutable std::mutex obstacle_db_mutex_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ========= Parameters =========
  std::string global_frame_;
  std::string base_frame_;

// [add] 본인 식별용 ID 변수
  uint16_t self_machine_id_{0};

  double cooldown_sec_;
  size_t consecutive_threshold_;
  double obstacle_persistence_sec_;
  double max_speed_;
  double lookahead_time_sec_;
  double min_lookahead_m_;
  double cost_threshold_;
  bool ignore_unknown_;

  // DB/ROI
  double db_update_frequency_;
  double obstacle_prune_timeout_sec_;
  int db_stride_;
  double cone_angle_deg_;
  int kernel_half_size_;

  // Path checking
  double path_check_distance_m_;

  // Replan flag pulse
  bool publish_false_pulse_;
  int flag_pulse_ms_;

  // Footprint / Agent mask / Output
  bool use_footprint_check_;
  double footprint_step_m_;
  bool compare_agent_mask_;
  std::string agent_mask_topic_;
  double agent_cost_threshold_;
  int agent_mask_manhattan_buffer_;

  // 충돌 메시지
  bool publish_agent_collision_;
  std::string agent_collision_topic_;

  // MultiAgent 구독
  std::string agents_topic_;
  int agents_freshness_timeout_ms_;
  double agent_match_dilate_m_;

  // Nav2 footprint
  std::string footprint_str_;
  std::vector<geometry_msgs::msg::Point> footprint_;
  bool use_radius_{true};
  double robot_radius_m_{0.1};

  // 에이전트 홀드 파라미터
  double agent_block_hold_sec_{2.0};
  double agent_block_max_wait_sec_{8.0};

  // === NEW: 에이전트 경로 튜브 매칭 파라미터 ===
  bool   agent_path_hit_enable_{true};
  double agent_path_hit_stride_m_{0.35};
  double agent_path_hit_dilate_m_{0.02};
  int    agent_path_hit_max_poses_{200};

  // [NEW] 우선순위에 따른 경로 검사 옵션
  bool respect_higher_priority_path_{false};  
};

}  // namespace replan_monitor

#endif  // PATH_VALIDATOR_NODE_HPP
