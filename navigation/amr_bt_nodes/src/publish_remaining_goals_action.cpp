// src/publish_remaining_goals_action.cpp

#include "amr_bt_nodes/publish_remaining_goals_action.hpp"


namespace amr_bt_nodes
{

PublishRemainingGoalsAction::PublishRemainingGoalsAction(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf), initialized_(false)
{
}

void PublishRemainingGoalsAction::initialize()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
    throw std::runtime_error("Failed to get 'node' from blackboard in PublishRemainingGoalsAction");
  }

  // Node의 로거를 캐싱하여 tick()에서 매번 get_logger()를 호출하는 오버헤드 제거
  logger_ = node_->get_logger();

  std::string topic_name;
  if (!getInput("topic_name", topic_name)) {
    topic_name = "/remaining_goals";
  }

  // 요청하신 대로 reliable QoS 프로파일 적용
  rclcpp::QoS qos_profile(10);
  qos_profile.reliable();
  publisher_ = node_->create_publisher<nav_msgs::msg::Path>(topic_name, qos_profile);

  last_publish_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  initialized_ = true;

  RCLCPP_INFO(
    logger_,
    "[PublishRemainingGoalsAction] Initialized. Publishing to: '%s'", topic_name.c_str());
}

BT::PortsList PublishRemainingGoalsAction::providedPorts()
{
  return {
    BT::InputPort<Goals>("input_goals", "Remaining goals to publish as nav_msgs/msg/Path"),
    BT::InputPort<std::string>("topic_name", "/remaining_goals", "Topic name to publish the path"),
    BT::InputPort<double>("publish_interval", 0.2, "The minimum interval in seconds between publishes")
  };
}

BT::NodeStatus PublishRemainingGoalsAction::tick()
{
  if (!initialized_) {
    initialize();
  }

  double interval;
  if (!getInput("publish_interval", interval)) {
     RCLCPP_ERROR(logger_, "[PublishRemainingGoalsAction] Missing required input 'publish_interval'.");
     return BT::NodeStatus::FAILURE;
  }

  rclcpp::Time current_time = node_->get_clock()->now();
  
  if ((current_time - last_publish_time_).seconds() >= interval) {
    
    Goals goal_poses;
    if (!getInput("input_goals", goal_poses)) {
      return BT::NodeStatus::SUCCESS;
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = current_time;
    
    if (!goal_poses.empty()) {
      path_msg.header.frame_id = goal_poses[0].header.frame_id;
    } else {
      path_msg.header.frame_id = "map";
    }

    path_msg.poses = goal_poses;
    publisher_->publish(path_msg);
    last_publish_time_ = current_time;

    // 매 tick 마다 반복되는 로그는 DEBUG로 변경하여 평상시 오버헤드 방지
    RCLCPP_DEBUG(
      logger_, 
      "[PublishRemainingGoalsAction] Published %zu remaining goals.", goal_poses.size());
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace amr_bt_nodes

// 플러그인 등록
#include "behaviortree_cpp/bt_factory.h"
extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<amr_bt_nodes::PublishRemainingGoalsAction>("PublishRemainingGoalsAction");
}