// src/bool_publisher_action.cpp

#include "amr_bt_nodes/bool_publisher_action.hpp"

namespace amr_bt_nodes
{

BoolPublisherAction::BoolPublisherAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  if (!getInput("node", node_)) {
    throw BT::RuntimeError("[BoolPublisherAction] Missing required input 'node'");
  }

  std::string topic_name;
  getInput("topic_name", topic_name);
  if (topic_name.empty()) {
    // 기본 토픽 이름 변경
    topic_name = "/bt_bool_publisher";
    RCLCPP_WARN(
      node_->get_logger(),
      "[BoolPublisherAction] 'topic_name' is not provided. Using default: '%s'", topic_name.c_str());
  }
  
  // QoS 설정
  rclcpp::QoS qos_profile(10);
  qos_profile.reliable();

  // 퍼블리셔 타입을 Bool로 변경
  publisher_ = node_->create_publisher<std_msgs::msg::Bool>(topic_name, qos_profile);

  RCLCPP_INFO(
    node_->get_logger(),
    "[BoolPublisherAction] Created. Publishing to topic: '%s'", topic_name.c_str());
}

BT::PortsList BoolPublisherAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "The ROS 2 node handle."),
    BT::InputPort<std::string>("topic_name", "/bt_bool_publisher", "Topic name to publish the bool to."),
    // message 포트의 타입을 bool로 변경
    BT::InputPort<bool>("message", "The boolean message to publish."),
    BT::InputPort<int>("num_publishes", 1, "Number of times to publish the message in a burst.")
  };
}

BT::NodeStatus BoolPublisherAction::tick()
{
  int num_publishes;
  if (!getInput("num_publishes", num_publishes) || num_publishes <= 0) {
    RCLCPP_ERROR(node_->get_logger(), "[BoolPublisherAction] 'num_publishes' must be a positive integer.");
    return BT::NodeStatus::FAILURE;
  }

  bool message_to_publish;
  if (!getInput("message", message_to_publish)) {
    RCLCPP_ERROR(node_->get_logger(), "[BoolPublisherAction] Missing required input 'message'.");
    return BT::NodeStatus::FAILURE;
  }

  // 메시지 타입을 std_msgs::msg::Bool로 변경
  std_msgs::msg::Bool msg;
  msg.data = message_to_publish;

  // 로그 메시지에서 bool 값을 명확하게 (true/false) 출력하도록 수정
  RCLCPP_INFO(
    node_->get_logger(),
    "[BoolPublisherAction] Publishing message '%s' %d times in a burst.",
    message_to_publish ? "true" : "false", num_publishes);
    
  for (int i = 0; i < num_publishes; ++i) {
    publisher_->publish(msg);
  }

  // 모든 작업이 tick() 안에서 완료되었으므로 SUCCESS 반환
  return BT::NodeStatus::SUCCESS;
}

} // namespace amr_bt_nodes


// 플러그인 등록
#include "behaviortree_cpp/bt_factory.h"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  // 클래스 이름을 BoolPublisherAction으로 변경하여 등록
  factory.registerNodeType<amr_bt_nodes::BoolPublisherAction>("BoolPublisherAction");
}