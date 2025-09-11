#include "replan_monitor/replan_monitor_node.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

// REMOVED: is_same_goal_id 헬퍼 함수 불필요

ReplanMonitorNode::ReplanMonitorNode() : Node("replan_monitor_node") 
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    subs_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto subs_options = rclcpp::SubscriptionOptions();
    subs_options.callback_group = subs_callback_group_;

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/plan", rclcpp::QoS(10), std::bind(&ReplanMonitorNode::pathCallback, this, std::placeholders::_1), subs_options);

    costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
        "/global_costmap/costmap_raw", rclcpp::QoS(rclcpp::SystemDefaultsQoS()).transient_local().reliable(), 
        std::bind(&ReplanMonitorNode::costmapCallback, this, std::placeholders::_1), subs_options);
    
    // ADDED: /robot_status 구독자 생성
    robot_status_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/robot_status", rclcpp::SystemDefaultsQoS(),
        std::bind(&ReplanMonitorNode::robotStatusCallback, this, std::placeholders::_1), subs_options);

    // REMOVED: Nav2 action status 구독자 생성 코드 모두 제거

    replan_pub_ = this->create_publisher<std_msgs::msg::Bool>("/replan_flag", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&ReplanMonitorNode::evaluateReplanCondition, this),
        timer_callback_group_);

    RCLCPP_INFO(this->get_logger(), "ReplanMonitorNode initialized. Subscribing to /robot_status.");
}

// ... pathCallback, costmapCallback, getCurrentPoseFromTF 함수는 이전과 동일 ...

// ADDED: robotStatusCallback 구현
void ReplanMonitorNode::robotStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
    const std::string& status = msg->data;
    // 로봇이 'DRIVING' 또는 'PLANNING' 상태일 때만 활성으로 간주
    if (status == "DRIVING" || status == "PLANNING") {
        is_robot_in_driving_state_.store(true);
    } else {
        is_robot_in_driving_state_.store(false);
    }
}

// REMOVED: updateGoalStatus, navToPoseStatusCallback, navThroughPosesStatusCallback 함수 모두 제거

void ReplanMonitorNode::evaluateReplanCondition() {
    // CHANGED: 가드 조건을 단순화된 플래그로 확인
    if (!is_robot_in_driving_state_) {
        return;
    }
    
    // ... 이하 로직은 이전 버전과 완벽하게 동일합니다 ...
    // ...
}




// <depend>std_msgs</depend> ```

// **`CMakeLists.txt`**
// ```cmake
// # find_package(action_msgs REQUIRED) # 제거
// # find_package(unique_identifier_msgs REQUIRED) # 제거
// find_package(std_msgs REQUIRED) # 있는지 확인
// # ...
// ament_target_dependencies(replan_monitor_node
//   # ... other dependencies
//   # action_msgs # 제거
//   # unique_identifier_msgs # 제거
//   std_msgs
// )