// run_if_flag_true_decorator.cpp
#include "amr_bt_nodes/run_if_flag_true_decorator.hpp"

namespace amr_bt_nodes
{

RunIfFlagTrueDecorator::RunIfFlagTrueDecorator(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::DecoratorNode(name, config), flag_ok_(false)
{
  if (!getInput("node", node_)) {
    throw BT::RuntimeError("[RunIfFlagTrueDecorator] Missing required input [node]");
  }

  if (!getInput("flag_topic", flag_topic_)) {
    flag_topic_ = "/mission_flag";
  }

  rclcpp::QoS qos_profile(10);
  qos_profile.best_effort();

  flag_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    flag_topic_, qos_profile,
    std::bind(&RunIfFlagTrueDecorator::flagCallback, this, std::placeholders::_1));

  SharedExecutor::ensureExecutorStarted(node_);

  RCLCPP_INFO(node_->get_logger(), "[RunIfFlagTrueDecorator] Subscriber initialized on topic: %s", flag_topic_.c_str());
}

RunIfFlagTrueDecorator::~RunIfFlagTrueDecorator()
{
  // No explicit shutdown needed. SharedExecutor handles lifecycle.
}

void RunIfFlagTrueDecorator::flagCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(flag_mutex_);
    flag_ok_ = msg->data;
  }
  RCLCPP_INFO(node_->get_logger(), "[RunIfFlagTrueDecorator] Received flag: %s", msg->data ? "true" : "false");
}

BT::NodeStatus RunIfFlagTrueDecorator::tick()
{
  if (!child_node_) {
    throw BT::RuntimeError("[RunIfFlagTrueDecorator] Child node is null.");
  }

  bool current_flag = false;
  {
    std::lock_guard<std::mutex> lock(flag_mutex_);
    current_flag = flag_ok_;
  }

  if (!current_flag)
  {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "[RunIfFlagTrueDecorator] Flag is FALSE. Skipping child node.");
    return BT::NodeStatus::FAILURE;
  }

  return child_node_->executeTick();
}

void RunIfFlagTrueDecorator::halt()
{
  {
    std::lock_guard<std::mutex> lock(flag_mutex_);
    flag_ok_ = false;
  }
  BT::DecoratorNode::halt();
}

}  // namespace amr_bt_nodes

#include "behaviortree_cpp/bt_factory.h"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<amr_bt_nodes::RunIfFlagTrueDecorator>("RunIfFlagTrueDecorator");
} 
