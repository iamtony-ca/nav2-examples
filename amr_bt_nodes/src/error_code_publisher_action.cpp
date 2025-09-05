// src/error_code_publisher_action.cpp

#include "amr_bt_nodes/error_code_publisher_action.hpp"
#include <cstdint> // uint16_t 사용을 위해 추가

namespace amr_bt_nodes
{

ErrorCodePublisherAction::ErrorCodePublisherAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  if (!getInput("node", node_)) {
    throw BT::RuntimeError("[ErrorCodePublisherAction] Missing required input 'node'");
  }

  std::string topic_name;
  getInput("topic_name", topic_name);
  if (topic_name.empty()) {
    topic_name = "/bt_error_code";
    RCLCPP_WARN(
      node_->get_logger(),
      "[ErrorCodePublisherAction] 'topic_name' is not provided. Using default: '%s'", topic_name.c_str());
  }

  rclcpp::QoS qos_profile(10);
  qos_profile.reliable();

  publisher_ = node_->create_publisher<std_msgs::msg::UInt16>(topic_name, qos_profile);

  RCLCPP_INFO(
    node_->get_logger(),
    "[ErrorCodePublisherAction] Created. Publishing to topic: '%s'", topic_name.c_str());
}

BT::PortsList ErrorCodePublisherAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "The ROS 2 node."),
    BT::InputPort<std::string>("topic_name", "/bt_error_code", "Topic name to publish the error code to."),
    // 변경점 1: 입력 포트의 타입을 uint16_t로 명시적으로 변경하여 타입 안전성 확보
    BT::InputPort<uint16_t>("error_code", "The uint16 error code to publish."),
    BT::InputPort<int>("num_publishes", 1, "Number of times to publish the message.")
  };
}

BT::NodeStatus ErrorCodePublisherAction::tick()
{
  // 변경점: 변수를 0으로 먼저 초기화합니다.
  // ActionResult::NONE (에러 없음) 상태를 기본값으로 설정합니다.
  uint16_t error_code_to_publish = 0;

  // getInput의 반환값을 체크하지 않습니다.
  // 만약 블랙보드에 'error_code' 키가 있으면, error_code_to_publish 변수는 그 값으로 덮어써집니다.
  // 만약 키가 없으면, 변수는 초기값인 0을 그대로 유지합니다.
  getInput("error_code", error_code_to_publish);

  // 이제 이 조건문 하나로 두 가지 경우를 모두 처리할 수 있습니다.
  // 1. 블랙보드에 키가 없어서 error_code_to_publish가 0인 경우
  // 2. 블랙보드에 키가 있고, 그 값이 0 (ActionResult::NONE)인 경우
  if (error_code_to_publish == 0) {
    RCLCPP_DEBUG(node_->get_logger(), "[ErrorCodePublisherAction] No valid error code (0 or not set). Skipping publication.");
    return BT::NodeStatus::SUCCESS;
  }

  int num_publishes;
  if (!getInput("num_publishes", num_publishes) || num_publishes <= 0) {
    RCLCPP_ERROR(node_->get_logger(), "[ErrorCodePublisherAction] 'num_publishes' must be a positive integer.");
    return BT::NodeStatus::FAILURE;
  }

  std_msgs::msg::UInt16 msg;
  msg.data = error_code_to_publish;

  RCLCPP_INFO(
    node_->get_logger(),
    "[ErrorCodePublisherAction] Publishing error_code '%u' %d times.",
    error_code_to_publish, num_publishes);

  for (int i = 0; i < num_publishes; ++i) {
    publisher_->publish(msg);
  }

  return BT::NodeStatus::SUCCESS;
}

} // namespace amr_bt_nodes

#include "behaviortree_cpp/bt_factory.h"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<amr_bt_nodes::ErrorCodePublisherAction>("ErrorCodePublisherAction");
}