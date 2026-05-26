// src/uint8_topic_publisher_action.cpp

#include "amr_bt_nodes/uint8_topic_publisher_action.hpp"
#include <cstdint>

namespace amr_bt_nodes
{

Uint8TopicPublisherAction::Uint8TopicPublisherAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  if (!getInput("node", node_)) {
    throw BT::RuntimeError("[Uint8TopicPublisherAction] Missing required input 'node'");
  }

  std::string topic_name;
  getInput("topic_name", topic_name);
  if (topic_name.empty()) {
    topic_name = "/bt_uint8_publisher"; // 범용 기본 토픽명
    RCLCPP_WARN(
      node_->get_logger(),
      "[Uint8TopicPublisherAction] 'topic_name' is not provided. Using default: '%s'", topic_name.c_str());
  }

  rclcpp::QoS qos_profile(10);
  qos_profile.reliable();

  publisher = node_->create_publisher<std_msgs::msg::UInt8>(topic_name, qos_profile);

  RCLCPP_INFO(
    node_->get_logger(),
    "[Uint8TopicPublisherAction] Created. Publishing to topic: '%s'", topic_name.c_str());
}

BT::PortsList Uint8TopicPublisherAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "The ROS 2 node."),
    BT::InputPort<std::string>("topic_name", "/bt_uint8_publisher", "Topic name to publish the uint8 message to."),
    // 입력 포트 이름을 의도에 맞게 'value'로 변경하여 범용성 확보
    BT::InputPort<uint8_t>("value", "The uint8 value to publish."),
    BT::InputPort<int>("num_publishes", 1, "Number of times to publish the message.")
  };
}

BT::NodeStatus Uint8TopicPublisherAction::tick()
{
  // 0으로 초기화 (블랙보드 변수 미설정 시 방어용 기본값)
  uint8_t value_to_publish = 0;

  // 블랙보드에 'value' 키가 없으면 0 유지, 있으면 해당 값으로 업데이트
  getInput("value", value_to_publish);

  // 값이 0(미설정 혹은 명령 없음)이면 발행을 스킵하고 SUCCESS 반환
  if (value_to_publish == 0) {
    RCLCPP_DEBUG(node_->get_logger(), "[Uint8TopicPublisherAction] Value is 0 or not set. Skipping publication.");
    return BT::NodeStatus::SUCCESS;
  }

  int num_publishes;
  if (!getInput("num_publishes", num_publishes) || num_publishes <= 0) {
    RCLCPP_ERROR(node_->get_logger(), "[Uint8TopicPublisherAction] 'num_publishes' must be a positive integer.");
    return BT::NodeStatus::FAILURE;
  }

  std_msgs::msg::UInt8 msg;
  msg.data = value_to_publish;

  RCLCPP_INFO(
    node_->get_logger(),
    "[Uint8TopicPublisherAction] Publishing value '%u' %d times.",
    static_cast<unsigned int>(value_to_publish), num_publishes);

  for (int i = 0; i < num_publishes; ++i) {
    publisher_->publish(msg);
  }

  return BT::NodeStatus::SUCCESS;
}

} // namespace amr_bt_nodes

#include "behaviortree_cpp/bt_factory.h"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<amr_bt_nodes::Uint8TopicPublisherAction>("Uint8TopicPublisherAction");
}