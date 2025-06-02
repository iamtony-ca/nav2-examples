// trigger_planner_decorator.cpp
#include "amr_bt_nodes/trigger_planner_decorator.hpp"

namespace amr_bt_nodes
{

using namespace std::chrono_literals;

TriggerPlannerDecorator::TriggerPlannerDecorator(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::DecoratorNode(name, config)
{
  if (!getInput("node", node_)) {
    throw BT::RuntimeError("[TriggerPlannerDecorator] Missing input [node]");
  }
  if (!getInput("flag_topic", flag_topic_)) {
    flag_topic_ = "/replan_flag";
  }

  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::QoS qos(10);
  qos.best_effort();

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;
  flag_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    flag_topic_, qos,
    std::bind(&TriggerPlannerDecorator::flagCallback, this, std::placeholders::_1),
    sub_options);

  ensureSharedExecutor(node_);

  RCLCPP_INFO(node_->get_logger(), "[TriggerPlannerDecorator] Subscribed to: %s", flag_topic_.c_str());
}

void TriggerPlannerDecorator::ensureSharedExecutor(rclcpp::Node::SharedPtr node)
{
  SharedExecutor::start(node);
}

void TriggerPlannerDecorator::flagCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (msg->data) {
    triggered_ = true;
  }
  last_flag_ = msg->data;
  RCLCPP_INFO(node_->get_logger(), "[TriggerPlannerDecorator] Triggered by flag");

}

BT::NodeStatus TriggerPlannerDecorator::tick()
{
  if (!child_node_) {
    throw BT::RuntimeError("[TriggerPlannerDecorator] Child node is null");
  }

  geometry_msgs::msg::PoseStamped current_goal;
  if (getInput("goal", current_goal)) {
    if (!has_last_goal_ ||
        current_goal.pose.position.x != last_goal_.pose.position.x ||
        current_goal.pose.position.y != last_goal_.pose.position.y ||
        current_goal.pose.orientation.z != last_goal_.pose.orientation.z ||
        current_goal.pose.orientation.w != last_goal_.pose.orientation.w) {
      triggered_ = true;
      last_goal_ = current_goal;
      has_last_goal_ = true;
      RCLCPP_INFO(node_->get_logger(), "[TriggerPlannerDecorator] Goal changed → Triggered");
    }
  }

  if (triggered_) {
    auto status = child_node_->executeTick();
    if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
      triggered_ = false;
    }
    return status;
  }

  return BT::NodeStatus::SUCCESS;
}

void TriggerPlannerDecorator::halt()
{
  BT::DecoratorNode::halt();
}

}  // namespace amr_bt_nodes

#include "behaviortree_cpp/bt_factory.h"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<amr_bt_nodes::TriggerPlannerDecorator>("TriggerPlannerDecorator");
}