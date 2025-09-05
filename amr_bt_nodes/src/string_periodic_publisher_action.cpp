// src/string_periodic_publisher_action.cpp

#include "amr_bt_nodes/string_periodic_publisher_action.hpp"
#include "behaviortree_cpp/bt_factory.h"

namespace amr_bt_nodes
{

StringPeriodicPublisherAction::StringPeriodicPublisherAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  if (!getInput("node", node_)) {
    throw BT::RuntimeError("[StringPeriodicPublisherAction] Missing required input 'node'");
  }

  // 생성자에서 last_burst_time_을 초기화하여 첫 tick에서는 항상 발행되도록 함
  last_burst_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());

  std::string topic_name;
  getInput("topic_name", topic_name);
  if (topic_name.empty()) {
    topic_name = "/bt_string_publisher";
    RCLCPP_WARN(
      node_->get_logger(),
      "[StringPeriodicPublisherAction] 'topic_name' is not provided. Using default: '%s'", topic_name.c_str());
  }
  
  rclcpp::QoS qos_profile(10);
  qos_profile.reliable();
  publisher_ = node_->create_publisher<std_msgs::msg::String>(topic_name, qos_profile);

  RCLCPP_INFO(
    node_->get_logger(),
    "[StringPeriodicPublisherAction] Created. Publishing to topic: '%s'", topic_name.c_str());
}

BT::PortsList StringPeriodicPublisherAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "The ROS 2 node."),
    BT::InputPort<std::string>("topic_name", "/bt_string_publisher", "Topic name to publish."),
    BT::InputPort<std::string>("message", "The string message to publish."),
    BT::InputPort<int>("num_publishes", 1, "Number of messages to publish in each burst."),
    BT::InputPort<double>("publish_interval", 1.0, "The minimum interval in seconds between bursts.")
  };
}

BT::NodeStatus StringPeriodicPublisherAction::tick()
{
  double interval;
  if (!getInput("publish_interval", interval)) {
     RCLCPP_ERROR(node_->get_logger(), "[StringPeriodicPublisherAction] Missing required input 'publish_interval'.");
     return BT::NodeStatus::FAILURE;
  }

  // 시간 간격 체크 로직
  rclcpp::Time current_time = node_->get_clock()->now();
  if ((current_time - last_burst_time_).seconds() >= interval) {
    // 시간이 되면 Burst 발행을 수행
    int num_publishes;
    if (!getInput("num_publishes", num_publishes) || num_publishes <= 0) {
      RCLCPP_ERROR(node_->get_logger(), "[StringPeriodicPublisherAction] 'num_publishes' must be a positive integer.");
      return BT::NodeStatus::FAILURE;
    }

    std::string message_to_publish;
    if (!getInput("message", message_to_publish)) {
      RCLCPP_ERROR(node_->get_logger(), "[StringPeriodicPublisherAction] Missing required input 'message'.");
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "[StringPeriodicPublisherAction] Interval met. Publishing '%s' %d times.",
      message_to_publish.c_str(), num_publishes);

    std_msgs::msg::String msg;
    msg.data = message_to_publish;
    for (int i = 0; i < num_publishes; ++i) {
      publisher_->publish(msg);
    }

    // 마지막 Burst 발행 시간을 현재 시간으로 갱신
    last_burst_time_ = current_time;
  }
  
  // 실제 발행 여부와 관계없이 항상 SUCCESS를 반환하여 BT의 흐름을 막지 않음
  return BT::NodeStatus::SUCCESS;
}

} // namespace amr_bt_nodes

// 플러그인 등록
extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<amr_bt_nodes::StringPeriodicPublisherAction>("StringPeriodicPublisherAction");
}