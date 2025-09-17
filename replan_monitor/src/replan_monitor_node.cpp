#include "replan_monitor/replan_monitor_node.hpp"
#include "nav2_costmap_2d/cost_values.hpp"


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
    
    robot_status_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/robot_status", rclcpp::SystemDefaultsQoS(),
        std::bind(&ReplanMonitorNode::robotStatusCallback, this, std::placeholders::_1), subs_options);


    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local().reliable();
    replan_pub_ = this->create_publisher<std_msgs::msg::Bool>("/replan_flag", qos);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&ReplanMonitorNode::evaluateReplanCondition, this),
        timer_callback_group_);

    last_replan_time_ = this->now();
        
    RCLCPP_INFO(this->get_logger(), "ReplanMonitorNode initialized. Subscribing to /robot_status.");
}


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
            // if (duration.seconds() >= obstacle_duration_threshold_sec_) {
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

