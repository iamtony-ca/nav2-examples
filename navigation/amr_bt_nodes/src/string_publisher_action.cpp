// src/string_publisher_action.cpp

#include "amr_bt_nodes/string_publisher_action.hpp"

// #include "behaviortree_cpp/plugins.h"

namespace amr_bt_nodes
{

StringPublisherAction::StringPublisherAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  if (!getInput("node", node_)) {
    throw BT::RuntimeError("[StringPublisherAction] Missing required input 'node'");
  }

  std::string topic_name;
  getInput("topic_name", topic_name);
  if (topic_name.empty()) {
    topic_name = "/bt_string_publisher";
    RCLCPP_WARN(
      node_->get_logger(),
      "[StringPublisherAction] 'topic_name' is not provided. Using default: '%s'", topic_name.c_str());
  }
  
  // QoS 설정
  rclcpp::QoS qos_profile(10); // Burst publish를 위해 약간의 큐 사이즈를 둠
  qos_profile.reliable();

  publisher_ = node_->create_publisher<std_msgs::msg::String>(topic_name, qos_profile);

  RCLCPP_INFO(
    node_->get_logger(),
    "[StringPublisherAction] Created. Publishing to topic: '%s'", topic_name.c_str());
}

BT::PortsList StringPublisherAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "The ROS 2 node."),
    BT::InputPort<std::string>("topic_name", "/bt_string_publisher", "Topic name to publish the string to."),
    BT::InputPort<std::string>("message", "The string message to publish."),
    BT::InputPort<int>("num_publishes", 1, "Number of times to publish the message in a burst.")
  };
}

BT::NodeStatus StringPublisherAction::tick()
{
  int num_publishes;
  if (!getInput("num_publishes", num_publishes) || num_publishes <= 0) {
    RCLCPP_ERROR(node_->get_logger(), "[StringPublisherAction] 'num_publishes' must be a positive integer.");
    return BT::NodeStatus::FAILURE;
  }

  std::string message_to_publish;
  if (!getInput("message", message_to_publish)) {
    RCLCPP_ERROR(node_->get_logger(), "[StringPublisherAction] Missing required input 'message'.");
    return BT::NodeStatus::FAILURE;
  }

  // for 루프를 사용하여 N회 연속 발행
  std_msgs::msg::String msg;
  msg.data = message_to_publish;

  RCLCPP_INFO(
    node_->get_logger(),
    "[StringPublisherAction] Publishing message '%s' %d times in a burst.", 
    message_to_publish.c_str(), num_publishes);
    
  for (int i = 0; i < num_publishes; ++i) {
    publisher_->publish(msg);
  }

  // 모든 작업이 tick() 안에서 완료되었으므로 SUCCESS 반환
  return BT::NodeStatus::SUCCESS;
}

} // namespace amr_bt_nodes

// 플러그인 등록
// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<amr_bt_nodes::StringPublisherAction>("StringPublisherAction");
// }

#include "behaviortree_cpp/bt_factory.h"


extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<amr_bt_nodes::StringPublisherAction>("StringPublisherAction");
} 




