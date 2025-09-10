#include "replan_monitor/replan_monitor_node.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

ReplanMonitorNode::ReplanMonitorNode()
: Node("replan_monitor_node") {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // CHANGED: 콜백 그룹 생성 및 할당
    subs_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto subs_options = rclcpp::SubscriptionOptions();
    subs_options.callback_group = subs_callback_group_;

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/plan", rclcpp::QoS(10), std::bind(&ReplanMonitorNode::pathCallback, this, std::placeholders::_1), subs_options);

    costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
        "/global_costmap/costmap_raw", rclcpp::QoS(rclcpp::SystemDefaultsQoS()).transient_local().reliable(), 
        std::bind(&ReplanMonitorNode::costmapCallback, this, std::placeholders::_1), subs_options);
    
    nav_to_pose_status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
        "/navigate_to_pose/_action/status", rclcpp::SystemDefaultsQoS(),
        std::bind(&ReplanMonitorNode::navToPoseStatusCallback, this, std::placeholders::_1), subs_options);

    nav_through_poses_status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
        "/navigate_through_poses/_action/status", rclcpp::SystemDefaultsQoS(),
        std::bind(&ReplanMonitorNode::navThroughPosesStatusCallback, this, std::placeholders::_1), subs_options);

    replan_pub_ = this->create_publisher<std_msgs::msg::Bool>("/replan_flag", 10);

    // CHANGED: 타이머 생성 시 별도의 콜백 그룹 지정
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&ReplanMonitorNode::evaluateReplanCondition, this),
        timer_callback_group_);

    last_replan_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "ReplanMonitorNode initialized with concurrent callback groups");
}

// ... pathCallback, costmapCallback, getCurrentPoseFromTF, updateGoalStatus, nav...StatusCallback 함수들은 이전과 동일 ...
// (이 함수들은 짧게 끝나므로 하나의 MutuallyExclusive 그룹에서 순차 실행되어도 문제없음)
// ... (생략된 동일한 코드들) ...

void ReplanMonitorNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_path_ = *msg;
    closest_index = 0;
    obstacle_seen_time_.clear();
    obstacle_distance_history_.clear();
}
void ReplanMonitorNode::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>(
        msg->metadata.size_x, msg->metadata.size_y,
        msg->metadata.resolution, msg->metadata.origin.position.x,
        msg->metadata.origin.position.y);
    
    unsigned char* char_map = costmap_->getCharMap();
    memcpy(char_map, &msg->data[0], msg->data.size() * sizeof(unsigned char));
    costmap_frame_ = msg->header.frame_id;
}
void ReplanMonitorNode::updateGoalStatus(const action_msgs::msg::GoalStatusArray::SharedPtr msg, std::atomic<bool> & is_active_flag)
{
    if (msg->status_list.empty()) {
        is_active_flag = false;
        return;
    }
    const auto & latest_goal_status = msg->status_list.back();
    int status = latest_goal_status.status;
    if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
        status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
    {
        is_active_flag = true;
    } else {
        is_active_flag = false;
    }
}
void ReplanMonitorNode::navToPoseStatusCallback(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
{
    updateGoalStatus(msg, is_nav_to_pose_active_);
}
void ReplanMonitorNode::navThroughPosesStatusCallback(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
{
    updateGoalStatus(msg, is_nav_through_poses_active_);
}
bool ReplanMonitorNode::getCurrentPoseFromTF(geometry_msgs::msg::Pose &pose_out) {
    try {
        geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
            source_frame_, target_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
        pose_out.position.x = tf.transform.translation.x;
        pose_out.position.y = tf.transform.translation.y;
        pose_out.position.z = tf.transform.translation.z;
        pose_out.orientation = tf.transform.rotation;
        return true;
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
        return false;
    }
}


// evaluateReplanCondition 함수는 변경할 필요 없음
// (내부의 mutex/atomic 사용이 이미 동시성 문제를 고려하고 있음)
void ReplanMonitorNode::evaluateReplanCondition() {
    if (!is_nav_to_pose_active_ && !is_nav_through_poses_active_) {
        return;
    }
    
    std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
    nav_msgs::msg::Path path;

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (current_path_.poses.empty() || !costmap_) {
            return;
        }
        costmap = costmap_;
        path = current_path_;
    }

    geometry_msgs::msg::Pose current_pose;
    if (!getCurrentPoseFromTF(current_pose)) {
        return;
    }
    // ... (이하 모든 계산 로직은 이전과 동일) ...
    immediate_replan = false;
    std_msgs::msg::Bool flag_msg;
    flag_msg.data = false;
    size_t closest_index_start = (closest_index > 5) ? closest_index - 5 : 0;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = closest_index_start; i < path.poses.size(); ++i) {
        const auto &p = path.poses[i].pose;
        double dx = p.position.x - current_pose.position.x;
        double dy = p.position.y - current_pose.position.y;
        double dist = std::hypot(dx, dy);
        if (dist < min_dist) {
            min_dist = dist;
            this->closest_index = i;
        }
    }
    rclcpp::Time now = this->now();
    double lookahead_distance = max_speed_ * lookahead_time_sec_;
    const auto &goal_pose = path.poses.back().pose;
    if (!std::isfinite(goal_pose.position.x) || !std::isfinite(goal_pose.position.y)) return;
    size_t blocked = 0;
    for (size_t i = this->closest_index; i < path.poses.size(); ++i) {
        const auto &pose = path.poses[i].pose;
        double dist = std::hypot(pose.position.x - current_pose.position.x, pose.position.y - current_pose.position.y);
        if (dist < passed_pose_ignore_dist_ || dist > lookahead_distance) continue;
        double goal_dist = std::hypot(pose.position.x - goal_pose.position.x, pose.position.y - goal_pose.position.y);
        if (goal_dist < goal_ignore_radius_) continue;
        unsigned int mx, my;
        if (!costmap->worldToMap(pose.position.x, pose.position.y, mx, my)) {
            continue;
        }
        unsigned char c = costmap->getCost(mx, my);
        int index = costmap->getIndex(mx, my);
        if (c >= cost_threshold_) {
            if (dist < immediate_block_dist_) {
                immediate_replan = true;
            }
            auto it = obstacle_seen_time_.find(index);
            if (it == obstacle_seen_time_.end()) {
                obstacle_seen_time_[index] = now;
                obstacle_distance_history_[index] = dist;
                continue;
            }
            rclcpp::Duration duration = now - it->second;
            bool is_approaching = false;
            if (obstacle_distance_history_.count(index)) {
                double prev_dist = obstacle_distance_history_[index];
                if (prev_dist - dist > approach_threshold_dist_) is_approaching = true;
                obstacle_distance_history_[index] = dist;
            }
            if (duration.seconds() >= obstacle_duration_threshold_sec_ && is_approaching) {
                blocked++;
            }
        } else {
            obstacle_seen_time_.erase(index);
            obstacle_distance_history_.erase(index);
        }
        if (immediate_replan) {
            flag_msg.data = true;
            break;
        }
    }
    if (flag_msg.data || (blocked >= blocked_threshold_ && (now - last_replan_time_).seconds() > cooldown_sec_)) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        flag_msg.data = true;
        last_replan_time_ = now;
        replan_pub_->publish(flag_msg);
        if (immediate_replan) {
            RCLCPP_WARN(this->get_logger(), "Triggering replan: immediate block detected.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Triggering replan: %zu path points consistently blocked.", blocked);
        }
    }
}