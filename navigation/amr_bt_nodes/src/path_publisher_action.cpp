#include "amr_bt_nodes/path_publisher_action.hpp"

namespace amr_bt_nodes
{

PathPublisherAction::PathPublisherAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  if (!getInput("node", node_)) {
    throw BT::RuntimeError("[PathPublisherAction] Missing required input 'node'");
  }

  std::string topic_name;
  getInput("topic_name", topic_name);
  if (topic_name.empty()) {
     topic_name = "/path_for_viz";
     RCLCPP_WARN(
      node_->get_logger(),
      "[PathPublisherAction] 'topic_name' is not provided. Using default: '%s'", topic_name.c_str());
  }
  
  publisher_ = node_->create_publisher<nav_msgs::msg::Path>(topic_name, 1);

  // last_publish_time_을 아주 오래된 시간으로 초기화하여 첫 번째 tick()에서는 항상 발행되도록 함
  last_publish_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());

  RCLCPP_INFO(
    node_->get_logger(),
    "[PathPublisherAction] Created. Publishing to topic: '%s'", topic_name.c_str());
}

BT::PortsList PathPublisherAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "The ROS 2 node."),
    BT::InputPort<nav_msgs::msg::Path>("input_path", "Path to be published."),
    BT::InputPort<std::string>("topic_name", "/path_for_viz", "Topic name to publish the path to."),
    // 새로운 입력 포트 추가: 발행 주기 (초 단위)
    BT::InputPort<double>("publish_period", 0.0, "The minimum period (in seconds) between publishes. 0.0 means publish on every tick.")
  };
}

BT::NodeStatus PathPublisherAction::tick()
{
  // 1. 입력 포트에서 발행 주기를 읽어옵니다.
  double publish_period;
  getInput("publish_period", publish_period);

  // 2. 현재 시간을 가져옵니다.
  auto current_time = node_->get_clock()->now();

  // 3. 주기가 0 이하이거나, 마지막 발행 시간으로부터 충분한 시간이 지났는지 확인합니다.
  if (publish_period <= 0.0 || (current_time - last_publish_time_).seconds() >= publish_period)
  {
    nav_msgs::msg::Path path_to_publish;
    if (!getInput("input_path", path_to_publish)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "[PathPublisherAction] Failed to get 'input_path' from the blackboard.");
      return BT::NodeStatus::FAILURE;
    }
    
    // 4. 토픽을 발행하고, 마지막 발행 시간을 현재 시간으로 업데이트합니다.
    RCLCPP_INFO(
    node_->get_logger(),
    "[PathPublisherAction] Publish Path");
    publisher_->publish(path_to_publish);
    last_publish_time_ = current_time;
  }
  
  // 토픽 발행 여부와 관계없이 항상 SUCCESS를 반환하여 BT 흐름을 막지 않습니다.
  return BT::NodeStatus::SUCCESS;
}

}  // namespace amr_bt_nodes

#include "behaviortree_cpp/bt_factory.h"

// BT_REGISTER_NODES(factory)
// {
//     factory.registerNodeType<amr_bt_nodes::PathPublisherAction>("PathPublisherAction");
// }


extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<amr_bt_nodes::PathPublisherAction>("PathPublisherAction");
} 
