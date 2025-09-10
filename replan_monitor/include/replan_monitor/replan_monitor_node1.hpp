#ifndef REPLAN_MONITOR_NODE_HPP
#define REPLAN_MONITOR_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include <unordered_map>
#include <mutex>
#include <cmath>
#include <limits>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ADDED: nav2_costmap_2d와 nav2_msgs::msg::Costmap 헤더 포함
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_msgs/msg/costmap.hpp"

class ReplanMonitorNode : public rclcpp::Node {
public:
    ReplanMonitorNode();

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    // CHANGED: 콜백 함수의 파라미터 타입 변경
    void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
    void evaluateReplanCondition();
    bool getCurrentPoseFromTF(geometry_msgs::msg::Pose &pose_out);

    // CHANGED: 구독자 타입 변경
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr replan_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Path current_path_;
    // CHANGED: Costmap을 nav2_costmap_2d::Costmap2D 객체로 관리
    std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
    std::string costmap_frame_;

    rclcpp::Time last_replan_time_;

    std::unordered_map<int, rclcpp::Time> obstacle_seen_time_;
    std::unordered_map<int, double> obstacle_distance_history_;
    std::mutex data_mutex_;
    size_t closest_index = 0;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string target_frame_ = "base_link";
    std::string source_frame_ = "map";

    // Parameters
    double cooldown_sec_ = 1.0;
    size_t blocked_threshold_ = 2;
    double passed_pose_ignore_dist_ = 0.9;
    double obstacle_duration_threshold_sec_ = 0.5;
    double approach_threshold_dist_ = 0.1;
    double max_speed_ = 0.5;
    double lookahead_time_sec_ = 15;
    double goal_ignore_radius_ = 0.2;
    // NOTE: 이 값은 이제 0~255 스케일의 raw cost와 비교됩니다.
    double cost_threshold_ = 20; // e.g., nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE (253) 보다 낮은 값
    double immediate_block_dist_ = 0.7;
    bool immediate_replan = false;
};

#endif // REPLAN_MONITOR_NODE_HPP