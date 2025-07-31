#include "robot_monitoring/status_manager.hpp"
#include "action_msgs/msg/goal_status.hpp"

using GoalStatus = action_msgs::msg::GoalStatus;

StatusManager::StatusManager(const rclcpp::NodeOptions & options)
: Node("status_manager", options)
{
    // 콜백 그룹 생성
    group_subscribers_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    group_timer_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = group_subscribers_;

    // ROS 인터페이스 초기화
    status_publisher_ = this->create_publisher<std_msgs::msg::String>("/robot_status", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&StatusManager::timer_callback, this), group_timer_);

    nav_to_pose_status_sub_ = this->create_subscription<GoalStatusArray>(
        "/navigate_to_pose/_action/status", 10, std::bind(&StatusManager::main_nav_status_callback, this, std::placeholders::_1), sub_options);
    nav_through_poses_status_sub_ = this->create_subscription<GoalStatusArray>(
        "/navigate_through_poses/_action/status", 10, std::bind(&StatusManager::main_nav_status_callback, this, std::placeholders::_1), sub_options);
    compute_path_status_sub_ = this->create_subscription<GoalStatusArray>(
        "/compute_path_to_pose/_action/status", 10, std::bind(&StatusManager::compute_path_status_callback, this, std::placeholders::_1), sub_options);
    follow_path_status_sub_ = this->create_subscription<GoalStatusArray>(
        "/follow_path/_action/status", 10, std::bind(&StatusManager::follow_path_status_callback, this, std::placeholders::_1), sub_options);
    bt_log_sub_ = this->create_subscription<BehaviorTreeLog>(
        "/behavior_tree_log", 10, std::bind(&StatusManager::bt_log_callback, this, std::placeholders::_1), sub_options);
    collision_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/is_collision_imminent", rclcpp::SystemDefaultsQoS(), std::bind(&StatusManager::collision_callback, this, std::placeholders::_1), sub_options);

    // nav_through_poses_client_ = rclcpp_action::create_client<NavigateThroughPoses>(
    //     this, "navigate_through_poses", group_subscribers_);
        
    RCLCPP_INFO(this->get_logger(), "StatusManager Node has been started (Deadlock-Free Mode).");
}

void StatusManager::set_status(RobotStatus new_status)
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (current_status_ == RobotStatus::SUCCEEDED ||
        current_status_ == RobotStatus::FAILED ||
        current_status_ == RobotStatus::CANCELED)
    {
        if (new_status != RobotStatus::RECEIVED_GOAL && new_status != RobotStatus::IDLE) {
            return;
        }
    }
    if (current_status_ != new_status) {
        current_status_ = new_status;
        publish_status_locked();
    }
}

void StatusManager::timer_callback()
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (!is_main_task_active_ &&
        current_status_ != RobotStatus::IDLE &&
        current_status_ != RobotStatus::SUCCEEDED &&
        current_status_ != RobotStatus::FAILED &&
        current_status_ != RobotStatus::CANCELED)
    {
        current_status_ = RobotStatus::IDLE;
    }
    publish_status_locked();
}

void StatusManager::publish_status_locked()
{
    auto msg = std_msgs::msg::String();
    msg.data = status_to_string(current_status_);
    status_publisher_->publish(msg);
}

void StatusManager::main_nav_status_callback(const GoalStatusArray::SharedPtr msg)
{
    bool task_is_currently_active = is_goal_active(msg);
    for (const auto& status : msg->status_list) {
        if (status.status == GoalStatus::STATUS_CANCELED) {
            set_status(RobotStatus::CANCELED);
            is_main_task_active_ = false;
            return;
        }
    }
    if (!is_main_task_active_ && task_is_currently_active) {
        set_status(RobotStatus::RECEIVED_GOAL);
    }
    is_main_task_active_ = task_is_currently_active;
}



// void StatusManager::main_nav_status_callback(const GoalStatusArray::SharedPtr msg)
// {
//     bool task_is_currently_active = is_goal_active(msg);

//     for (const auto& status : msg->status_list) {
//         // 최종 상태를 먼저 확인
//         if (status.status == GoalStatus::STATUS_SUCCEEDED) {
//             set_status(RobotStatus::SUCCEEDED);
//             is_main_task_active_ = false;
//             return;
//         }
//         if (status.status == GoalStatus::STATUS_ABORTED) {
//             set_status(RobotStatus::FAILED);
//             is_main_task_active_ = false;
//             return;
//         }
//         if (status.status == GoalStatus::STATUS_CANCELED) {
//             set_status(RobotStatus::CANCELED);
//             is_main_task_active_ = false;
//             return;
//         }
//     }

//     // 최종 상태가 아니라면, 작업 시작 여부만 판단
//     if (!is_main_task_active_ && task_is_currently_active) {
//         set_status(RobotStatus::RECEIVED_GOAL);
//     }
    
//     is_main_task_active_ = task_is_currently_active;
// }







// 교착 상태를 유발하는 lock 제거
void StatusManager::compute_path_status_callback(const GoalStatusArray::SharedPtr msg)
{
    if (is_goal_active(msg)) {
        if (current_context_ == RobotContext::RECOVERY) {
            set_status(RobotStatus::RECOVERY_PLANNING);
        } else {
            set_status(RobotStatus::PLANNING);
        }
    }
}

// 교착 상태를 유발하는 lock 제거
void StatusManager::follow_path_status_callback(const GoalStatusArray::SharedPtr msg)
{
    if (is_goal_active(msg)) {
        if (current_context_ == RobotContext::RECOVERY) {
            set_status(RobotStatus::RECOVERY_DRIVING);
        } else {
            set_status(RobotStatus::DRIVING);
        }
    }
}

//  교착 상태를 유발하는 lock 제거
void StatusManager::bt_log_callback(const BehaviorTreeLog::SharedPtr msg)
{
    for (const auto& event : msg->event_log) {
        const std::string& node_name = event.node_name;
        
        if (event.current_status == "RUNNING") {
            if (node_name.find("Recovery") != std::string::npos) {
                current_context_ = RobotContext::RECOVERY;
            }
            if (node_name == "CheckPauseCondition") {
                set_status(RobotStatus::PAUSED);
            } else if (node_name.find("Clear") != std::string::npos && node_name.find("Costmap") != std::string::npos) {
                set_status(RobotStatus::RECOVERY_CLEARING);
            }
        } else if (event.current_status == "SUCCESS" || event.current_status == "FAILURE") {
            if (node_name.find("Recovery") != std::string::npos) {
                current_context_ = RobotContext::NORMAL;
            }
            if (node_name == "NavigateRecovery" || node_name == "MainTree") {
                if (event.current_status == "SUCCESS") {
                    set_status(RobotStatus::SUCCEEDED);
                } else {
                    set_status(RobotStatus::FAILED);
                }
                return;
            }
        }
    }
}




// void StatusManager::bt_log_callback(const BehaviorTreeLog::SharedPtr msg)
// {
//     for (const auto& event : msg->event_log) {
//         const std::string& node_name = event.node_name;
        
//         if (event.current_status == "RUNNING") {
//             // 문맥 설정
//             if (node_name.find("Recovery") != std::string::npos) {
//                 std::lock_guard<std::mutex> lock(status_mutex_);
//                 current_context_ = RobotContext::RECOVERY;
//             }
//             // BT에서만 알 수 있는 상태 직접 설정
//             if (node_name == "CheckPauseCondition") {
//                 set_status(RobotStatus::PAUSED);
//             } else if (node_name.find("Clear") != std::string::npos && node_name.find("Costmap") != std::string::npos) {
//                 set_status(RobotStatus::RECOVERY_CLEARING);
//             }
//         } else if (event.current_status == "SUCCESS" || event.current_status == "FAILURE") {
//              // 문맥 해제
//             if (node_name.find("Recovery") != std::string::npos) {
//                 std::lock_guard<std::mutex> lock(status_mutex_);
//                 current_context_ = RobotContext::NORMAL;
//             }
//             //  최종 상태 감지 로직은 여기서 제거됨
//         }
//     }
// }



//  교착 상태를 유발하는 lock 제거
void StatusManager::feedback_callback_through_poses(
  GoalHandleThroughPoses::SharedPtr, const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
{
    if ((current_status_ == RobotStatus::DRIVING || current_status_ == RobotStatus::FOLLOWING_WAYPOINTS) 
        && feedback->number_of_poses_remaining > 0) 
    {
        set_status(RobotStatus::FOLLOWING_WAYPOINTS);
    } 
    else if (current_status_ == RobotStatus::FOLLOWING_WAYPOINTS && feedback->number_of_poses_remaining == 0) 
    {
        set_status(RobotStatus::DRIVING);
    }
}

//  교착 상태를 유발하는 lock 제거
void StatusManager::collision_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        set_status(RobotStatus::COLLISION_IMMINENT);
    } else {
        if (current_status_ == RobotStatus::COLLISION_IMMINENT) {
            set_status(RobotStatus::IDLE);
        }
    }
}

bool StatusManager::is_goal_active(const GoalStatusArray::SharedPtr msg)
{
    for (const auto& status : msg->status_list) {
        if (status.status == GoalStatus::STATUS_ACCEPTED || status.status == GoalStatus::STATUS_EXECUTING) {
            return true;
        }
    }
    return false;
}

std::string StatusManager::status_to_string(RobotStatus status)
{
    switch (status) {
        case RobotStatus::IDLE:               return "IDLE";
        case RobotStatus::RECEIVED_GOAL:      return "RECEIVED_GOAL";
        case RobotStatus::PLANNING:           return "PLANNING";
        case RobotStatus::DRIVING:            return "DRIVING";
        case RobotStatus::FOLLOWING_WAYPOINTS:return "FOLLOWING_WAYPOINTS";
        case RobotStatus::PAUSED:             return "PAUSED";
        case RobotStatus::COLLISION_IMMINENT: return "COLLISION_IMMINENT";
        case RobotStatus::RECOVERY_CLEARING:  return "RECOVERY_CLEARING";
        case RobotStatus::RECOVERY_PLANNING:  return "RECOVERY_PLANNING";
        case RobotStatus::RECOVERY_DRIVING:   return "RECOVERY_DRIVING";
        case RobotStatus::SUCCEEDED:          return "SUCCEEDED";
        case RobotStatus::FAILED:             return "FAILED";
        case RobotStatus::CANCELED:           return "CANCELED";
        default:                              return "UNKNOWN";
    }
}