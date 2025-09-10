#include "replan_monitor/replan_monitor_node.hpp"
#include "nav2_costmap_2d/cost_values.hpp" // ADDED: LETHAL_OBSTACLE 등 cost 값 상수를 사용하기 위함

ReplanMonitorNode::ReplanMonitorNode()
: Node("replan_monitor_node") {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::SubscriptionOptions sub_options;
    auto callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options.callback_group = callback_group;

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/plan", rclcpp::QoS(10), std::bind(&ReplanMonitorNode::pathCallback, this, std::placeholders::_1), sub_options);

    // CHANGED: 구독 토픽과 메시지 타입 변경
    costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
        "/global_costmap/costmap_raw", rclcpp::QoS(rclcpp::SystemDefaultsQoS()).transient_local().reliable(), 
        std::bind(&ReplanMonitorNode::costmapCallback, this, std::placeholders::_1), sub_options);

    replan_pub_ = this->create_publisher<std_msgs::msg::Bool>("/replan_flag", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&ReplanMonitorNode::evaluateReplanCondition, this));

    last_replan_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "ReplanMonitorNode initialized");
}

void ReplanMonitorNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_path_ = *msg;
    closest_index = 0;
    obstacle_seen_time_.clear();
    obstacle_distance_history_.clear();
}

// CHANGED: 콜백 함수 전체 로직 변경
void ReplanMonitorNode::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    // Costmap 메시지를 nav2_costmap_2d::Costmap2D 객체로 변환
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>(
        msg->metadata.size_x, msg->metadata.size_y,
        msg->metadata.resolution, msg->metadata.origin.position.x,
        msg->metadata.origin.position.y);
    
    // memcpy를 사용하여 원본 cost 데이터를 복사
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

void ReplanMonitorNode::evaluateReplanCondition() {
    std::unique_lock<std::mutex> lock(data_mutex_);

    // 로컬 스코프에 costmap 포인터를 복사하여 안전하게 사용
    auto costmap = costmap_;
    if (current_path_.poses.empty() || !costmap) {
        return;
    }

    geometry_msgs::msg::Pose current_pose;
    if (!getCurrentPoseFromTF(current_pose)) {
        return;
    }

    lock.unlock(); // TF 조회 후 뮤텍스 락 해제 (긴 계산 동안 다른 콜백 방해하지 않도록)

    immediate_replan = false;
    std_msgs::msg::Bool flag_msg;
    flag_msg.data = false;

    // ... (가장 가까운 경로점 찾는 로직은 동일)
    size_t closest_index_start = (closest_index > 5) ? closest_index - 5 : 0;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = closest_index_start; i < current_path_.poses.size(); ++i) {
        const auto &p = current_path_.poses[i].pose;
        double dx = p.position.x - current_pose.position.x;
        double dy = p.position.y - current_pose.position.y;
        double dist = std::hypot(dx, dy);
        if (dist < min_dist) {
            min_dist = dist;
            this->closest_index = i; // Use member variable
        }
    }
    
    rclcpp::Time now = this->now();
    double lookahead_distance = max_speed_ * lookahead_time_sec_;
    const auto &goal_pose = current_path_.poses.back().pose;
    if (!std::isfinite(goal_pose.position.x) || !std::isfinite(goal_pose.position.y)) return;
    size_t blocked = 0;

    for (size_t i = this->closest_index; i < current_path_.poses.size(); ++i) {
        const auto &pose = current_path_.poses[i].pose;
        double dist = std::hypot(pose.position.x - current_pose.position.x, pose.position.y - current_pose.position.y);
        if (dist < passed_pose_ignore_dist_ || dist > lookahead_distance) continue;

        double goal_dist = std::hypot(pose.position.x - goal_pose.position.x, pose.position.y - goal_pose.position.y);
        if (goal_dist < goal_ignore_radius_) continue;

        // CHANGED: Costmap2D 유틸리티를 사용하여 cost 값 획득
        unsigned int mx, my;
        if (!costmap->worldToMap(pose.position.x, pose.position.y, mx, my)) {
            continue; // 경로가 costmap 밖에 있는 경우 무시
        }
        
        unsigned char cost = costmap->getCost(mx, my);
        int index = costmap->getIndex(mx, my); // 고유 식별을 위해 맵 인덱스 사용

        if (cost >= cost_threshold_) {
            if (dist < immediate_block_dist_) {
                immediate_replan = true;
            }
            // ... (나머지 장애물 지속 시간 및 접근 감지 로직은 동일)
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
            break; // 즉시 replan이므로 더 이상 검사할 필요 없음
        }
    }

    // ... (replan 발행 로직은 거의 동일, last_replan_time_ 접근 시 뮤텍스 필요)
    lock.lock(); // 멤버 변수 접근 전 다시 락
    if (flag_msg.data || (blocked >= blocked_threshold_ && (now - last_replan_time_).seconds() > cooldown_sec_)) {
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


// <depend>nav2_msgs</depend>

// find_package(nav2_msgs REQUIRED)
// # ...
// ament_target_dependencies(replan_monitor_node
//   # ... other dependencies
//   nav2_msgs
// )