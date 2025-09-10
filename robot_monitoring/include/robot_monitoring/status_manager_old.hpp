#pragma once

// ... (기존 include 와 enum 선언은 동일) ...
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/msg/behavior_tree_log.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include <string>
#include <mutex>
#include <optional>
#include <vector>
#include <map>
#include <atomic>
// #include <set> // For storing previous BT node states

enum class RobotStatus : uint8_t {
    IDLE,
    RECEIVED_GOAL,
    PLANNING,
    DRIVING,
    FOLLOWING_WAYPOINTS,
    PAUSED,
    COLLISION_IMMINENT,
    RECOVERY_FAILURE,      // RECOVERY_CLEARING -> RECOVERY_FAILURE
    RECOVERY_RUNNING,      // RECOVERY_PLANNING -> RECOVERY_RUNNING
    RECOVERY_SUCCESS,    // RECOVERY_DRIVING -> RECOVERY_SUCCEEDED
    SUCCEEDED,
    FAILED,
    CANCELED
};


class StatusManager : public rclcpp::Node
{
public:
    using GoalStatusArray = action_msgs::msg::GoalStatusArray;
    using BehaviorTreeLog = nav2_msgs::msg::BehaviorTreeLog;

    explicit StatusManager(const rclcpp::NodeOptions & options);

private:
    // --- 주요 로직 ---
    void evaluate_and_publish_if_changed();
    RobotStatus determine_current_status(RobotStatus last_known_status); // last_known_status 인자 추가

    // --- 콜백 함수 ---
    void timer_callback(); // 주기적 발행을 위한 콜백
    void nav_to_pose_status_callback(const GoalStatusArray::SharedPtr msg);
    void waypoints_status_callback(const GoalStatusArray::SharedPtr msg);
    void compute_path_status_callback(const GoalStatusArray::SharedPtr msg);
    void follow_path_status_callback(const GoalStatusArray::SharedPtr msg);
    void bt_log_callback(const BehaviorTreeLog::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void collision_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void pause_flag_callback(const std_msgs::msg::Bool::SharedPtr msg);

    // --- 유틸리티 및 초기화 함수 ---
    void query_initial_node_states();
    std::string status_to_string(RobotStatus status);
    bool is_bt_node_running(const std::string& node_name) const;

    // --- 상태 저장을 위한 멤버 변수 ---
    std::recursive_mutex status_mutex_; // 재귀 뮤텍스로 변경하여 유연성 확보
    RobotStatus current_status_{RobotStatus::IDLE};

    // ... (나머지 멤버 변수들은 이전과 동일) ...
    std::atomic<bool> are_core_nodes_active_{false};
    std::atomic<bool> is_robot_stopped_{true};
    std::atomic<bool> is_collision_imminent_{false};
    std::atomic<bool> is_paused_{false};
    std::atomic<bool> is_nav_executing_{false};
    std::atomic<bool> is_waypoints_executing_{false};
    std::atomic<bool> is_planning_sub_action_executing_{false};
    std::atomic<bool> is_driving_sub_action_executing_{false};
    std::vector<std::string> running_bt_nodes_;
    std::atomic<bool> is_in_recovery_context_{false};
    std::optional<action_msgs::msg::GoalStatus> latest_terminal_status_;
    // 현재 활성 navigate_to_pose 목표의 ID를 추적하기 위한 변수 ***
    std::optional<unique_identifier_msgs::msg::UUID> active_nav_goal_id_;

    
    // --- ROS 인터페이스 ---
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::Subscription<GoalStatusArray>::SharedPtr nav_to_pose_status_sub_;
    rclcpp::Subscription<GoalStatusArray>::SharedPtr waypoints_status_sub_;
    rclcpp::Subscription<GoalStatusArray>::SharedPtr compute_path_status_sub_;
    rclcpp::Subscription<GoalStatusArray>::SharedPtr follow_path_status_sub_;
    rclcpp::Subscription<BehaviorTreeLog>::SharedPtr bt_log_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collision_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr> lifecycle_clients_;
    std::vector<std::string> core_node_names_ = {
      "bt_navigator", "planner_server", "controller_server", "amcl"
    };
};



























// #pragma once

// // ... (기존 include 와 enum 선언은 동일) ...
// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include "action_msgs/msg/goal_status_array.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "nav2_msgs/msg/behavior_tree_log.hpp"
// #include "lifecycle_msgs/srv/get_state.hpp"
// #include "std_msgs/msg/string.hpp"
// #include "std_msgs/msg/bool.hpp"
// #include <string>
// #include <mutex>
// #include <optional>
// #include <vector>
// #include <map>
// #include <atomic>

// enum class RobotStatus : uint8_t {
//     IDLE,
//     RECEIVED_GOAL,
//     PLANNING,
//     DRIVING,
//     FOLLOWING_WAYPOINTS,
//     PAUSED,
//     COLLISION_IMMINENT,
//     RECOVERY_CLEARING,
//     RECOVERY_PLANNING,
//     RECOVERY_DRIVING,
//     SUCCEEDED,
//     FAILED,
//     CANCELED
// };


// class StatusManager : public rclcpp::Node
// {
// public:
//     using GoalStatusArray = action_msgs::msg::GoalStatusArray;
//     using BehaviorTreeLog = nav2_msgs::msg::BehaviorTreeLog;

//     explicit StatusManager(const rclcpp::NodeOptions & options);

// private:
//     // --- 주요 로직 ---
//     void evaluate_and_publish_if_changed();
//     RobotStatus determine_current_status(RobotStatus last_known_status); // last_known_status 인자 추가

//     // --- 콜백 함수 ---
//     void timer_callback(); // 주기적 발행을 위한 콜백
//     void nav_to_pose_status_callback(const GoalStatusArray::SharedPtr msg);
//     void waypoints_status_callback(const GoalStatusArray::SharedPtr msg);
//     void compute_path_status_callback(const GoalStatusArray::SharedPtr msg);
//     void follow_path_status_callback(const GoalStatusArray::SharedPtr msg);
//     void bt_log_callback(const BehaviorTreeLog::SharedPtr msg);
//     void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
//     void collision_callback(const std_msgs::msg::Bool::SharedPtr msg);
//     void pause_flag_callback(const std_msgs::msg::Bool::SharedPtr msg);

//     // --- 유틸리티 및 초기화 함수 ---
//     void query_initial_node_states();
//     std::string status_to_string(RobotStatus status);
//     bool is_bt_node_running(const std::string& node_name) const;

//     // --- 상태 저장을 위한 멤버 변수 ---
//     std::recursive_mutex status_mutex_; // 재귀 뮤텍스로 변경하여 유연성 확보
//     RobotStatus current_status_{RobotStatus::IDLE};

//     // ... (나머지 멤버 변수들은 이전과 동일) ...
//     std::atomic<bool> are_core_nodes_active_{false};
//     std::atomic<bool> is_robot_stopped_{true};
//     std::atomic<bool> is_collision_imminent_{false};
//     std::atomic<bool> is_paused_{false};
//     std::atomic<bool> is_nav_executing_{false};
//     std::atomic<bool> is_waypoints_executing_{false};
//     std::atomic<bool> is_planning_sub_action_executing_{false};
//     std::atomic<bool> is_driving_sub_action_executing_{false};
//     std::vector<std::string> running_bt_nodes_;
//     std::atomic<bool> is_in_recovery_context_{false};
//     std::optional<action_msgs::msg::GoalStatus> latest_terminal_status_;
//     // *** FIX: 현재 활성 navigate_to_pose 목표의 ID를 추적하기 위한 변수 ***
//     std::optional<unique_identifier_msgs::msg::UUID> active_nav_goal_id_;

    
//     // --- ROS 인터페이스 ---
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
//     rclcpp::Subscription<GoalStatusArray>::SharedPtr nav_to_pose_status_sub_;
//     rclcpp::Subscription<GoalStatusArray>::SharedPtr waypoints_status_sub_;
//     rclcpp::Subscription<GoalStatusArray>::SharedPtr compute_path_status_sub_;
//     rclcpp::Subscription<GoalStatusArray>::SharedPtr follow_path_status_sub_;
//     rclcpp::Subscription<BehaviorTreeLog>::SharedPtr bt_log_sub_;
//     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
//     rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collision_sub_;
//     rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_sub_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     std::vector<rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr> lifecycle_clients_;
//     std::vector<std::string> core_node_names_ = {
//       "bt_navigator", "planner_server", "controller_server", "amcl"
//     };
// };



