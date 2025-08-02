#include "robot_monitoring/status_manager.hpp"
#include "action_msgs/msg/goal_status.hpp"
#include <thread>

using GoalStatus = action_msgs::msg::GoalStatus;

StatusManager::StatusManager(const rclcpp::NodeOptions & options)
: Node("status_manager", options)
{
    // ... (생성자 내용은 이전과 동일하게 유지) ...
    status_publisher_ = this->create_publisher<std_msgs::msg::String>("/robot_status", rclcpp::SystemDefaultsQoS());
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&StatusManager::timer_callback, this));

    nav_to_pose_status_sub_ = this->create_subscription<GoalStatusArray>(
        "/navigate_to_pose/_action/status", 10, std::bind(&StatusManager::nav_to_pose_status_callback, this, std::placeholders::_1));
    waypoints_status_sub_ = this->create_subscription<GoalStatusArray>(
        "/follow_waypoints/_action/status", 10, std::bind(&StatusManager::waypoints_status_callback, this, std::placeholders::_1));
    compute_path_status_sub_ = this->create_subscription<GoalStatusArray>(
        "/compute_path_to_pose/_action/status", 10, std::bind(&StatusManager::compute_path_status_callback, this, std::placeholders::_1));
    follow_path_status_sub_ = this->create_subscription<GoalStatusArray>(
        "/follow_path/_action/status", 10, std::bind(&StatusManager::follow_path_status_callback, this, std::placeholders::_1));
    bt_log_sub_ = this->create_subscription<BehaviorTreeLog>(
        "/behavior_tree_log", 10, std::bind(&StatusManager::bt_log_callback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&StatusManager::odom_callback, this, std::placeholders::_1));
    collision_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/is_collision_imminent", rclcpp::SystemDefaultsQoS(), std::bind(&StatusManager::collision_callback, this, std::placeholders::_1));
    pause_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/pause_robot", rclcpp::SystemDefaultsQoS(), std::bind(&StatusManager::pause_flag_callback, this, std::placeholders::_1));

    for (const auto& name : core_node_names_) {
        lifecycle_clients_.push_back(this->create_client<lifecycle_msgs::srv::GetState>(name + "/get_state"));
    }
    std::thread{ [this]() { this->query_initial_node_states(); }}.detach();
    RCLCPP_INFO(this->get_logger(), "StatusManager Node has been started.");
}

// 주기적 발행을 담당하는 타이머 콜백
void StatusManager::timer_callback()
{
    std::lock_guard<std::recursive_mutex> lock(status_mutex_);
    // 현재 상태를 주기적으로 발행
    auto msg = std_msgs::msg::String();
    msg.data = status_to_string(current_status_);
    status_publisher_->publish(msg);
}

// 상태 변화 감지 및 발행을 담당하는 메인 함수
void StatusManager::evaluate_and_publish_if_changed()
{
    std::lock_guard<std::recursive_mutex> lock(status_mutex_);
    
    RobotStatus new_status = determine_current_status(current_status_);

    if (current_status_ != new_status) {
        current_status_ = new_status;
        auto msg = std_msgs::msg::String();
        msg.data = status_to_string(current_status_);
        RCLCPP_INFO(this->get_logger(), "Robot Status Changed -> %s", msg.data.c_str());
        status_publisher_->publish(msg);
    }
}

RobotStatus StatusManager::determine_current_status(RobotStatus last_known_status)
{
    // ... (1 ~ 4번 로직은 이전과 동일) ...
    // 1. 최우선: 안전/명령 오버라이드
    if (is_collision_imminent_.load()) return RobotStatus::COLLISION_IMMINENT;
    if (is_paused_.load()) return RobotStatus::PAUSED;

    // 2. 진행 중인 작업: navigate_to_pose
    if (is_nav_executing_.load()) {
        if (is_in_recovery_context_.load()) {
            if (is_bt_node_running("ClearEntireCostmap")) return RobotStatus::RECOVERY_CLEARING;
            if (is_bt_node_running("ComputePathToPose")) return RobotStatus::RECOVERY_PLANNING;
            if (is_bt_node_running("FollowPath")) return RobotStatus::RECOVERY_DRIVING;
            return RobotStatus::RECOVERY_CLEARING;
        }

        if (is_driving_sub_action_executing_.load()) return RobotStatus::DRIVING;
        if (is_planning_sub_action_executing_.load()) return RobotStatus::PLANNING;
        
        if (last_known_status == RobotStatus::PLANNING || last_known_status == RobotStatus::DRIVING) {
            return last_known_status;
        }
        
        if (is_bt_node_running("NavigateWithReplanning")) return RobotStatus::RECEIVED_GOAL;
        
        return RobotStatus::RECEIVED_GOAL;
    }

    // 3. 진행 중인 작업: follow_waypoints
    if (is_waypoints_executing_.load()) {
        if (!is_in_recovery_context_.load()) {
            return RobotStatus::FOLLOWING_WAYPOINTS;
        }
    }

    // 4. 작업 종료 상태
    if (latest_terminal_status_.has_value()) {
        auto status_val = latest_terminal_status_->status;
        latest_terminal_status_.reset();
        switch (status_val) {
            case GoalStatus::STATUS_SUCCEEDED: return RobotStatus::SUCCEEDED;
            case GoalStatus::STATUS_CANCELED: return RobotStatus::CANCELED;
            case GoalStatus::STATUS_ABORTED: return RobotStatus::FAILED;
        }
    }


    // 5. 기본 상태: IDLE
    if (are_core_nodes_active_.load() && is_robot_stopped_.load()) {
        return RobotStatus::IDLE;
    }

    // *** 6. 수정된 최종 폴백 ***
    // 핵심 노드가 비활성화된 명백한 실패 상황일 때만 FAILED 반환
    if (!are_core_nodes_active_.load()) {
        return RobotStatus::FAILED;
    }

    // 그 외의 경우는 아직 IDLE로 안정화되지 않은 과도기 상태이므로,
    // FAILED로 단정하지 않고 이전 상태를 유지하여 불필요한 변경 방지
    return last_known_status;
}

// UUID 비교를 위한 헬퍼 함수
bool is_same_goal_id(const unique_identifier_msgs::msg::UUID& id1, const unique_identifier_msgs::msg::UUID& id2) {
    return std::equal(std::begin(id1.uuid), std::end(id1.uuid), std::begin(id2.uuid));
}

void StatusManager::nav_to_pose_status_callback(const GoalStatusArray::SharedPtr msg)
{
    bool was_active = is_nav_executing_.load();
    bool is_active_now = false;
    std::optional<action_msgs::msg::GoalStatus> final_status;

    // 1. 현재 활성화된 goal_id를 찾음
    if (!active_nav_goal_id_.has_value()) {
        for (const auto& status : msg->status_list) {
            if (status.status == GoalStatus::STATUS_EXECUTING) {
                is_active_now = true;
                std::lock_guard<std::recursive_mutex> lock(status_mutex_);
                active_nav_goal_id_ = status.goal_info.goal_id;
                break; // EXECUTING 상태를 찾으면 바로 ID 저장하고 루프 종료
            }
        }
    } else {
        // 이미 ID를 추적 중이면, 해당 ID가 여전히 EXECUTING인지 확인
        is_active_now = true; 
    }

    // 2. 추적 중인 ID의 최종 상태를 찾되, CANCELED를 최우선으로 함
    if (active_nav_goal_id_.has_value()) {
        for (const auto& status : msg->status_list) {
            if (is_same_goal_id(status.goal_info.goal_id, active_nav_goal_id_.value())) {
                // CANCELED는 가장 높은 우선순위를 가짐
                if (status.status == GoalStatus::STATUS_CANCELED) {
                    final_status = status;
                    break; // CANCELED를 찾았으면 더 이상 볼 필요 없음
                }
                // 다른 최종 상태들도 저장해두지만, CANCELED가 나오면 덮어써짐
                if (status.status == GoalStatus::STATUS_SUCCEEDED || status.status == GoalStatus::STATUS_ABORTED) {
                    final_status = status;
                }
            }
        }
    }

    // 3. is_nav_executing_ 플래그 및 최종 상태 업데이트
    if (final_status.has_value()) {
        is_active_now = false; // 최종 상태가 있으면 더 이상 활성 상태가 아님
        std::lock_guard<std::recursive_mutex> lock(status_mutex_);
        latest_terminal_status_ = final_status;
        active_nav_goal_id_.reset(); // 목표가 끝났으므로 ID 초기화
    }

    if (was_active != is_active_now) {
        is_nav_executing_.store(is_active_now);
    }
    
    if (!was_active && is_active_now) {
        std::lock_guard<std::recursive_mutex> lock(status_mutex_);
        latest_terminal_status_.reset();
    }

    evaluate_and_publish_if_changed();
}

void StatusManager::waypoints_status_callback(const GoalStatusArray::SharedPtr msg)
{
    // ... (이전과 동일한 로직) ...
    bool is_active = false;
    for (const auto& status : msg->status_list) {
        if (status.status == GoalStatus::STATUS_EXECUTING) {
            is_active = true;
            break;
        }
    }
    is_waypoints_executing_.store(is_active);
    evaluate_and_publish_if_changed();
}

void StatusManager::compute_path_status_callback(const GoalStatusArray::SharedPtr msg)
{
    // ... (이전과 동일한 로직) ...
    bool is_active = false;
    for (const auto& status : msg->status_list) {
        if (status.status == GoalStatus::STATUS_EXECUTING) {
            is_active = true;
            break;
        }
    }
    is_planning_sub_action_executing_.store(is_active);
    evaluate_and_publish_if_changed();
}

void StatusManager::follow_path_status_callback(const GoalStatusArray::SharedPtr msg)
{
    // ... (이전과 동일한 로직) ...
    bool is_active = false;
    for (const auto& status : msg->status_list) {
        if (status.status == GoalStatus::STATUS_EXECUTING) {
            is_active = true;
            break;
        }
    }
    is_driving_sub_action_executing_.store(is_active);
    evaluate_and_publish_if_changed();
}

void StatusManager::bt_log_callback(const BehaviorTreeLog::SharedPtr msg)
{
    // ... (이전과 동일한 로직) ...
    std::lock_guard<std::recursive_mutex> lock(status_mutex_);
    running_bt_nodes_.clear();
    bool recovery_flag = false;
    for (const auto& event : msg->event_log) {
        if (event.current_status == "RUNNING") {
            running_bt_nodes_.push_back(event.node_name);
            if (event.node_name == "NavigateRecovery" || event.node_name.find("Recovery") != std::string::npos) {
                recovery_flag = true;
            }
        }
    }
    is_in_recovery_context_.store(recovery_flag);
    evaluate_and_publish_if_changed();
}

void StatusManager::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // ... (이전과 동일한 로직) ...
    double linear_vel = msg->twist.twist.linear.x;
    double angular_vel = msg->twist.twist.angular.z;
    is_robot_stopped_.store(std::abs(linear_vel) < 0.01 && std::abs(angular_vel) < 0.01);
}

void StatusManager::collision_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    is_collision_imminent_.store(msg->data);
    evaluate_and_publish_if_changed();
}

void StatusManager::pause_flag_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    is_paused_.store(msg->data);
    evaluate_and_publish_if_changed();
}

// ... (query_initial_node_states, is_bt_node_running, status_to_string 함수는 이전과 동일) ...
void StatusManager::query_initial_node_states()
{
    bool all_active = true;
    for (size_t i = 0; i < core_node_names_.size(); ++i) {
        while (!lifecycle_clients_[i]->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for %s service.", core_node_names_[i].c_str());
                are_core_nodes_active_.store(false);
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for %s service...", core_node_names_[i].c_str());
        }

        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        auto result = lifecycle_clients_[i]->async_send_request(request).get();
        if (result->current_state.id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            all_active = false;
            RCLCPP_WARN(this->get_logger(), "Node %s is not active.", core_node_names_[i].c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Node %s is active.", core_node_names_[i].c_str());
        }
    }
    are_core_nodes_active_.store(all_active);
    RCLCPP_INFO(this->get_logger(), "Core nodes active status: %s", all_active ? "true" : "false");
    evaluate_and_publish_if_changed();
}

bool StatusManager::is_bt_node_running(const std::string& node_name) const
{
    for (const auto& name : running_bt_nodes_) {
        if (name == node_name) {
            return true;
        }
    }
    return false;
}

std::string StatusManager::status_to_string(RobotStatus status)
{
    switch (status) {
        case RobotStatus::IDLE:                return "IDLE";
        case RobotStatus::RECEIVED_GOAL:       return "RECEIVED_GOAL";
        case RobotStatus::PLANNING:            return "PLANNING";
        case RobotStatus::DRIVING:             return "DRIVING";
        case RobotStatus::FOLLOWING_WAYPOINTS: return "FOLLOWING_WAYPOINTS";
        case RobotStatus::PAUSED:              return "PAUSED";
        case RobotStatus::COLLISION_IMMINENT:  return "COLLISION_IMMINENT";
        case RobotStatus::RECOVERY_CLEARING:   return "RECOVERY_CLEARING";
        case RobotStatus::RECOVERY_PLANNING:   return "RECOVERY_PLANNING";
        case RobotStatus::RECOVERY_DRIVING:    return "RECOVERY_DRIVING";
        case RobotStatus::SUCCEEDED:           return "SUCCEEDED";
        case RobotStatus::FAILED:              return "FAILED";
        case RobotStatus::CANCELED:            return "CANCELED";
        default:                               return "UNKNOWN";
    }
}































// #include "robot_monitoring/status_manager.hpp"
// #include "action_msgs/msg/goal_status.hpp"
// #include <thread>

// using GoalStatus = action_msgs::msg::GoalStatus;

// StatusManager::StatusManager(const rclcpp::NodeOptions & options)
// : Node("status_manager", options)
// {
//     // ... (생성자 내용은 이전과 동일하게 유지) ...
//     status_publisher_ = this->create_publisher<std_msgs::msg::String>("/robot_status", rclcpp::SystemDefaultsQoS());
//     timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&StatusManager::timer_callback, this));

//     nav_to_pose_status_sub_ = this->create_subscription<GoalStatusArray>(
//         "/navigate_to_pose/_action/status", 10, std::bind(&StatusManager::nav_to_pose_status_callback, this, std::placeholders::_1));
//     waypoints_status_sub_ = this->create_subscription<GoalStatusArray>(
//         "/follow_waypoints/_action/status", 10, std::bind(&StatusManager::waypoints_status_callback, this, std::placeholders::_1));
//     compute_path_status_sub_ = this->create_subscription<GoalStatusArray>(
//         "/compute_path_to_pose/_action/status", 10, std::bind(&StatusManager::compute_path_status_callback, this, std::placeholders::_1));
//     follow_path_status_sub_ = this->create_subscription<GoalStatusArray>(
//         "/follow_path/_action/status", 10, std::bind(&StatusManager::follow_path_status_callback, this, std::placeholders::_1));
//     bt_log_sub_ = this->create_subscription<BehaviorTreeLog>(
//         "/behavior_tree_log", 10, std::bind(&StatusManager::bt_log_callback, this, std::placeholders::_1));
//     odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
//         "/odom", 10, std::bind(&StatusManager::odom_callback, this, std::placeholders::_1));
//     collision_sub_ = this->create_subscription<std_msgs::msg::Bool>(
//         "/is_collision_imminent", rclcpp::SystemDefaultsQoS(), std::bind(&StatusManager::collision_callback, this, std::placeholders::_1));
//     pause_sub_ = this->create_subscription<std_msgs::msg::Bool>(
//         "/pause_robot", rclcpp::SystemDefaultsQoS(), std::bind(&StatusManager::pause_flag_callback, this, std::placeholders::_1));

//     for (const auto& name : core_node_names_) {
//         lifecycle_clients_.push_back(this->create_client<lifecycle_msgs::srv::GetState>(name + "/get_state"));
//     }
//     std::thread{ [this]() { this->query_initial_node_states(); }}.detach();
//     RCLCPP_INFO(this->get_logger(), "StatusManager Node has been started.");
// }

// // 주기적 발행을 담당하는 타이머 콜백
// void StatusManager::timer_callback()
// {
//     std::lock_guard<std::recursive_mutex> lock(status_mutex_);
//     // 현재 상태를 주기적으로 발행
//     auto msg = std_msgs::msg::String();
//     msg.data = status_to_string(current_status_);
//     status_publisher_->publish(msg);
// }

// // 상태 변화 감지 및 발행을 담당하는 메인 함수
// void StatusManager::evaluate_and_publish_if_changed()
// {
//     std::lock_guard<std::recursive_mutex> lock(status_mutex_);
    
//     RobotStatus new_status = determine_current_status(current_status_);

//     if (current_status_ != new_status) {
//         current_status_ = new_status;
//         auto msg = std_msgs::msg::String();
//         msg.data = status_to_string(current_status_);
//         RCLCPP_INFO(this->get_logger(), "Robot Status Changed -> %s", msg.data.c_str());
//         status_publisher_->publish(msg);
//     }
// }

// // *** 핵심 수정: RECEIVED_GOAL 반복 문제 해결 ***
// RobotStatus StatusManager::determine_current_status(RobotStatus last_known_status)
// {
//     // 1. 최우선: 안전/명령 오버라이드
//     if (is_collision_imminent_.load()) return RobotStatus::COLLISION_IMMINENT;
//     if (is_paused_.load()) return RobotStatus::PAUSED;

//     // 2. 진행 중인 작업: navigate_to_pose
//     if (is_nav_executing_.load()) {
//         if (is_in_recovery_context_.load()) {
//             if (is_bt_node_running("ClearEntireCostmap")) return RobotStatus::RECOVERY_CLEARING;
//             if (is_bt_node_running("ComputePathToPose")) return RobotStatus::RECOVERY_PLANNING;
//             if (is_bt_node_running("FollowPath")) return RobotStatus::RECOVERY_DRIVING;
//             return RobotStatus::RECOVERY_CLEARING;
//         }

//         if (is_driving_sub_action_executing_.load()) return RobotStatus::DRIVING;
//         if (is_planning_sub_action_executing_.load()) return RobotStatus::PLANNING;
        
//         // --- 문제 해결 로직 ---
//         // 이미 PLANNING 또는 DRIVING 상태라면, 잠시 조건이 비더라도 RECEIVED_GOAL로 돌아가지 않음
//         if (last_known_status == RobotStatus::PLANNING || last_known_status == RobotStatus::DRIVING) {
//             return last_known_status; // 현재 상태 유지
//         }
        
//         if (is_bt_node_running("NavigateWithReplanning")) return RobotStatus::RECEIVED_GOAL;
        
//         return RobotStatus::RECEIVED_GOAL;
//     }

//     // 3. 진행 중인 작업: follow_waypoints
//     if (is_waypoints_executing_.load()) {
//         if (!is_in_recovery_context_.load()) {
//             return RobotStatus::FOLLOWING_WAYPOINTS;
//         }
//     }

//     // 4. 작업 종료 상태
//     if (latest_terminal_status_.has_value()) {
//         auto status_val = latest_terminal_status_->status;
//         latest_terminal_status_.reset();
//         switch (status_val) {
//             case GoalStatus::STATUS_SUCCEEDED: return RobotStatus::SUCCEEDED;
//             case GoalStatus::STATUS_CANCELED: return RobotStatus::CANCELED;
//             case GoalStatus::STATUS_ABORTED: return RobotStatus::FAILED;
//         }
//     }

//     // 5. 기본 상태: IDLE
//     if (are_core_nodes_active_.load() && is_robot_stopped_.load()) {
//         return RobotStatus::IDLE;
//     }

//     // 6. 최종 폴백: 시스템 실패
//     return RobotStatus::FAILED;
// }

// // UUID 비교를 위한 헬퍼 함수
// bool is_same_goal_id(const unique_identifier_msgs::msg::UUID& id1, const unique_identifier_msgs::msg::UUID& id2) {
//     return std::equal(std::begin(id1.uuid), std::end(id1.uuid), std::begin(id2.uuid));
// }

// void StatusManager::nav_to_pose_status_callback(const GoalStatusArray::SharedPtr msg)
// {
//     bool was_active = is_nav_executing_.load();
//     bool is_active_now = false;
//     std::optional<action_msgs::msg::GoalStatus> terminal_status_for_our_goal;

//     // 현재 활성화된 goal_id를 찾고, 그 goal_id에 대한 최종 상태를 확인
//     for (const auto& status : msg->status_list) {
//         if (status.status == GoalStatus::STATUS_EXECUTING) {
//             is_active_now = true;
//             // *** FIX: 활성 목표의 ID를 저장 ***
//             {
//                 std::lock_guard<std::recursive_mutex> lock(status_mutex_);
//                 active_nav_goal_id_ = status.goal_info.goal_id;
//             }
//         }
//     }
    
//     // 활성 목표가 있다면, 그 목표의 최종 상태를 찾아야 함
//     if (active_nav_goal_id_.has_value()) {
//         for (const auto& status : msg->status_list) {
//             // *** FIX: 저장된 ID와 일치하는 상태만 처리 ***
//             if (is_same_goal_id(status.goal_info.goal_id, active_nav_goal_id_.value())) {
//                 if (status.status == GoalStatus::STATUS_SUCCEEDED ||
//                     status.status == GoalStatus::STATUS_CANCELED ||
//                     status.status == GoalStatus::STATUS_ABORTED)
//                 {
//                     terminal_status_for_our_goal = status;
//                 }
//             }
//         }
//     }

//     // is_nav_executing_ 플래그 업데이트
//     if (was_active != is_active_now) {
//         is_nav_executing_.store(is_active_now);
//     }

//     // 우리 목표의 최종 상태가 감지되면 latest_terminal_status_ 업데이트
//     if(terminal_status_for_our_goal.has_value()){
//         std::lock_guard<std::recursive_mutex> lock(status_mutex_);
//         latest_terminal_status_ = terminal_status_for_our_goal;
//         // 목표가 끝났으므로 추적하던 ID 초기화
//         active_nav_goal_id_.reset();
//     }
    
//     // 새 작업 시작 시 이전 종료 상태 초기화
//     if (!was_active && is_active_now) {
//         std::lock_guard<std::recursive_mutex> lock(status_mutex_);
//         latest_terminal_status_.reset();
//     }

//     evaluate_and_publish_if_changed();
// }

// void StatusManager::waypoints_status_callback(const GoalStatusArray::SharedPtr msg)
// {
//     // ... (이전과 동일한 로직) ...
//     bool is_active = false;
//     for (const auto& status : msg->status_list) {
//         if (status.status == GoalStatus::STATUS_EXECUTING) {
//             is_active = true;
//             break;
//         }
//     }
//     is_waypoints_executing_.store(is_active);
//     evaluate_and_publish_if_changed();
// }

// void StatusManager::compute_path_status_callback(const GoalStatusArray::SharedPtr msg)
// {
//     // ... (이전과 동일한 로직) ...
//     bool is_active = false;
//     for (const auto& status : msg->status_list) {
//         if (status.status == GoalStatus::STATUS_EXECUTING) {
//             is_active = true;
//             break;
//         }
//     }
//     is_planning_sub_action_executing_.store(is_active);
//     evaluate_and_publish_if_changed();
// }

// void StatusManager::follow_path_status_callback(const GoalStatusArray::SharedPtr msg)
// {
//     // ... (이전과 동일한 로직) ...
//     bool is_active = false;
//     for (const auto& status : msg->status_list) {
//         if (status.status == GoalStatus::STATUS_EXECUTING) {
//             is_active = true;
//             break;
//         }
//     }
//     is_driving_sub_action_executing_.store(is_active);
//     evaluate_and_publish_if_changed();
// }

// void StatusManager::bt_log_callback(const BehaviorTreeLog::SharedPtr msg)
// {
//     // ... (이전과 동일한 로직) ...
//     std::lock_guard<std::recursive_mutex> lock(status_mutex_);
//     running_bt_nodes_.clear();
//     bool recovery_flag = false;
//     for (const auto& event : msg->event_log) {
//         if (event.current_status == "RUNNING") {
//             running_bt_nodes_.push_back(event.node_name);
//             if (event.node_name == "NavigateRecovery" || event.node_name.find("Recovery") != std::string::npos) {
//                 recovery_flag = true;
//             }
//         }
//     }
//     is_in_recovery_context_.store(recovery_flag);
//     evaluate_and_publish_if_changed();
// }

// void StatusManager::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
// {
//     // ... (이전과 동일한 로직) ...
//     double linear_vel = msg->twist.twist.linear.x;
//     double angular_vel = msg->twist.twist.angular.z;
//     is_robot_stopped_.store(std::abs(linear_vel) < 0.01 && std::abs(angular_vel) < 0.01);
// }

// void StatusManager::collision_callback(const std_msgs::msg::Bool::SharedPtr msg)
// {
//     is_collision_imminent_.store(msg->data);
//     evaluate_and_publish_if_changed();
// }

// void StatusManager::pause_flag_callback(const std_msgs::msg::Bool::SharedPtr msg)
// {
//     is_paused_.store(msg->data);
//     evaluate_and_publish_if_changed();
// }

// // ... (query_initial_node_states, is_bt_node_running, status_to_string 함수는 이전과 동일) ...
// void StatusManager::query_initial_node_states()
// {
//     bool all_active = true;
//     for (size_t i = 0; i < core_node_names_.size(); ++i) {
//         while (!lifecycle_clients_[i]->wait_for_service(std::chrono::seconds(1))) {
//             if (!rclcpp::ok()) {
//                 RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for %s service.", core_node_names_[i].c_str());
//                 are_core_nodes_active_.store(false);
//                 return;
//             }
//             RCLCPP_INFO(this->get_logger(), "Waiting for %s service...", core_node_names_[i].c_str());
//         }

//         auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
//         auto result = lifecycle_clients_[i]->async_send_request(request).get();
//         if (result->current_state.id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
//             all_active = false;
//             RCLCPP_WARN(this->get_logger(), "Node %s is not active.", core_node_names_[i].c_str());
//         } else {
//             RCLCPP_INFO(this->get_logger(), "Node %s is active.", core_node_names_[i].c_str());
//         }
//     }
//     are_core_nodes_active_.store(all_active);
//     RCLCPP_INFO(this->get_logger(), "Core nodes active status: %s", all_active ? "true" : "false");
//     evaluate_and_publish_if_changed();
// }

// bool StatusManager::is_bt_node_running(const std::string& node_name) const
// {
//     for (const auto& name : running_bt_nodes_) {
//         if (name == node_name) {
//             return true;
//         }
//     }
//     return false;
// }

// std::string StatusManager::status_to_string(RobotStatus status)
// {
//     switch (status) {
//         case RobotStatus::IDLE:                return "IDLE";
//         case RobotStatus::RECEIVED_GOAL:       return "RECEIVED_GOAL";
//         case RobotStatus::PLANNING:            return "PLANNING";
//         case RobotStatus::DRIVING:             return "DRIVING";
//         case RobotStatus::FOLLOWING_WAYPOINTS: return "FOLLOWING_WAYPOINTS";
//         case RobotStatus::PAUSED:              return "PAUSED";
//         case RobotStatus::COLLISION_IMMINENT:  return "COLLISION_IMMINENT";
//         case RobotStatus::RECOVERY_CLEARING:   return "RECOVERY_CLEARING";
//         case RobotStatus::RECOVERY_PLANNING:   return "RECOVERY_PLANNING";
//         case RobotStatus::RECOVERY_DRIVING:    return "RECOVERY_DRIVING";
//         case RobotStatus::SUCCEEDED:           return "SUCCEEDED";
//         case RobotStatus::FAILED:              return "FAILED";
//         case RobotStatus::CANCELED:            return "CANCELED";
//         default:                               return "UNKNOWN";
//     }
// }


