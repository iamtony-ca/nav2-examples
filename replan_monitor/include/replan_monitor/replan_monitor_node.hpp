#ifndef REPLAN_MONITOR_NODE_HPP
#define REPLAN_MONITOR_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include <atomic>
#include <mutex>


#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp" 
#include "std_msgs/msg/bool.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace replan_monitor
{

class ReplanMonitorNode : public rclcpp::Node {
public:
    ReplanMonitorNode();

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
    void evaluateReplanCondition();
    bool getCurrentPoseFromTF(geometry_msgs::msg::Pose &pose_out);

    
    void robotStatusCallback(const std_msgs::msg::String::SharedPtr msg);

    rclcpp::CallbackGroup::SharedPtr subs_callback_group_;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr replan_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_status_sub_;
    
    

    nav_msgs::msg::Path current_path_;
    std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
    std::string costmap_frame_;
    
    std::atomic<bool> is_robot_in_driving_state_{false};

    
    
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
    size_t blocked_threshold_ = 3;  
    double passed_pose_ignore_dist_ = 0.9;
    double obstacle_duration_threshold_sec_ = 1.0;  // 0.5;
    double approach_threshold_dist_ = 0.1;
    double max_speed_ = 0.5;
    double lookahead_time_sec_ = 15;
    double goal_ignore_radius_ = 0.2;
    double cost_threshold_ = 200; // 20   // 0~255 스케일의 raw cost
    double immediate_block_dist_ = 0.7;
    bool immediate_replan = false;    

    
};

}  // namespace replan_monitor


#endif // REPLAN_MONITOR_NODE_HPP