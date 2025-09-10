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
#include <atomic>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

class ReplanMonitorNode : public rclcpp::Node {
public:
    ReplanMonitorNode();

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void costmapCallback(const nav2_msgs::msg/Costmap::SharedPtr msg);
    void evaluateReplanCondition();
    bool getCurrentPoseFromTF(geometry_msgs::msg::Pose &pose_out);

    void navToPoseStatusCallback(const action_msgs::msg::GoalStatusArray::SharedPtr msg);
    void navThroughPosesStatusCallback(const action_msgs::msg::GoalStatusArray::SharedPtr msg);
    void updateGoalStatus(const action_msgs::msg::GoalStatusArray::SharedPtr msg, std::atomic<bool> & is_active_flag);
    
    // ADDED: 콜백 그룹 멤버 변수 선언
    rclcpp::CallbackGroup::SharedPtr subs_callback_group_;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr replan_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav_to_pose_status_sub_;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav_through_poses_status_sub_;

    nav_msgs::msg::Path current_path_;
    std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
    std::string costmap_frame_;
    
    std::atomic<bool> is_nav_to_pose_active_{false};
    std::atomic<bool> is_nav_through_poses_active_{false};

    rclcpp::Time last_replan_time_;

    std::unordered_map<int, rclcpp::Time> obstacle_seen_time_;
    std::unordered_map<int, double> obstacle_distance_history_;
    std::mutex data_mutex_; // 여러 구독 콜백이 공유하는 데이터(path, costmap) 보호용
    size_t closest_index = 0;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string target_frame_ = "base_link";
    std::string source_frame_ = "map";

    // ... Parameters ...
    double cooldown_sec_ = 1.0;
    size_t blocked_threshold_ = 2;
    double passed_pose_ignore_dist_ = 0.9;
    double obstacle_duration_threshold_sec_ = 0.5;
    double approach_threshold_dist_ = 0.1;
    double max_speed_ = 0.5;
    double lookahead_time_sec_ = 15;
    double goal_ignore_radius_ = 0.2;
    double cost_threshold_ = 20;
    double immediate_block_dist_ = 0.7;
    bool immediate_replan = false;
};

#endif // REPLAN_MONITOR_NODE_HPP