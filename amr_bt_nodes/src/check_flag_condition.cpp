#include "amr_bt_nodes/check_flag_condition.hpp"

namespace amr_bt_nodes
{

CheckFlagCondition::CheckFlagCondition(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config), flag_ok_(true)
{
  if (!getInput("node", node_)) {
    throw BT::RuntimeError("[CheckFlagCondition] Missing required input [node]");
  }

  if (!getInput("flag_topic", flag_topic_)) {
    flag_topic_ = "/mission_flag";
  }

  rclcpp::QoS qos(10);
  qos.best_effort();

  flag_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    flag_topic_, qos,
    std::bind(&CheckFlagCondition::flagCallback, this, std::placeholders::_1));

  SharedExecutor::ensureExecutorStarted(node_);

  RCLCPP_INFO(node_->get_logger(), "[CheckFlagCondition] Subscribed to topic: %s", flag_topic_.c_str());
}

CheckFlagCondition::~CheckFlagCondition()
{
}

void CheckFlagCondition::flagCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(flag_mutex_);
    flag_ok_ = msg->data;
  }
  RCLCPP_INFO(node_->get_logger(), "[CheckFlagCondition] Received flag: %s", msg->data ? "true" : "false");

}

BT::NodeStatus CheckFlagCondition::tick()
{
  current_flag = false;
  // bool current_flag = false;
  {
    std::lock_guard<std::mutex> lock(flag_mutex_);
    current_flag = flag_ok_;
    flag_ok_ = false;
  }
  

  RCLCPP_DEBUG(node_->get_logger(), "[CheckFlagCondition] Flag: %s", current_flag ? "true" : "false");

  return current_flag ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace amr_bt_nodes

#include "behaviortree_cpp/bt_factory.h"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<amr_bt_nodes::CheckFlagCondition>("CheckFlagCondition");
}
