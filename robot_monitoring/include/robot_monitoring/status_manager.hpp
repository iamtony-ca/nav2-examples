#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp" // 'rclcpp::action' 타입을 위해 필수
#include "action_msgs/msg/goal_status_array.hpp"
#include "nav2_msgs/msg/behavior_tree_log.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include <string>
#include <mutex>
#include <optional>

// 최종 RobotStatus Enum
enum class RobotStatus : uint8_t {
    IDLE,
    RECEIVED_GOAL,
    PLANNING,
    DRIVING,
    FOLLOWING_WAYPOINTS,
    PAUSED,
    COLLISION_IMMINENT,
    RECOVERY_CLEARING,
    RECOVERY_PLANNING,
    RECOVERY_DRIVING,
    SUCCEEDED,
    FAILED,
    CANCELED
};

enum class RobotContext : uint8_t {
    NORMAL,
    RECOVERY
};

class StatusManager : public rclcpp::Node
{
public:
    using GoalStatusArray = action_msgs::msg::GoalStatusArray;
    using BehaviorTreeLog = nav2_msgs::msg::BehaviorTreeLog;
    using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
    using GoalHandleThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

    explicit StatusManager(const rclcpp::NodeOptions & options);

private:
    // 콜백 함수 선언부
    void timer_callback();
    void main_nav_status_callback(const GoalStatusArray::SharedPtr msg);
    void compute_path_status_callback(const GoalStatusArray::SharedPtr msg);
    void follow_path_status_callback(const GoalStatusArray::SharedPtr msg);
    void bt_log_callback(const BehaviorTreeLog::SharedPtr msg);
    void collision_callback(const std_msgs::msg::Bool::SharedPtr msg);
    
    //  NavigateThroughPoses 관련 콜백 선언 추가
    void feedback_callback_through_poses(GoalHandleThroughPoses::SharedPtr, const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback);

    // 유틸리티 함수
    void set_status(RobotStatus new_status);
    void publish_status_locked();
    std::string status_to_string(RobotStatus status);
    bool is_goal_active(const GoalStatusArray::SharedPtr msg);

    // 공유 자원 및 Mutex
    std::mutex status_mutex_;
    RobotStatus current_status_{RobotStatus::IDLE};
    RobotContext current_context_{RobotContext::NORMAL};
    bool is_main_task_active_{false};

    //  ROS 인터페이스 멤버 변수 선언 (이름 일치시킴)
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::Subscription<GoalStatusArray>::SharedPtr nav_to_pose_status_sub_;
    rclcpp::Subscription<GoalStatusArray>::SharedPtr nav_through_poses_status_sub_; //  추가
    rclcpp::Subscription<GoalStatusArray>::SharedPtr compute_path_status_sub_;
    rclcpp::Subscription<GoalStatusArray>::SharedPtr follow_path_status_sub_;
    rclcpp::Subscription<BehaviorTreeLog>::SharedPtr bt_log_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collision_sub_;
    // rclcpp::action::Client<NavigateThroughPoses>::SharedPtr nav_through_poses_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    //  콜백 그룹 선언 (이름 일치시킴)
    rclcpp::CallbackGroup::SharedPtr group_subscribers_;
    rclcpp::CallbackGroup::SharedPtr group_timer_;
};