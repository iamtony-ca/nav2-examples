// trigger_planner_decorator.hpp
#pragma once

#include <behaviortree_cpp/decorator_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <mutex>

namespace amr_bt_nodes
{

class TriggerPlannerDecorator : public BT::DecoratorNode
{
public:
  TriggerPlannerDecorator(const std::string & name, const BT::NodeConfiguration & config);
  ~TriggerPlannerDecorator() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<rclcpp::Node::SharedPtr>("node", "Shared ROS node"),
      BT::InputPort<std::string>("flag_topic", "/decor_flag", "Bool flag topic"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Target goal pose")
    };
  }

  BT::NodeStatus tick() override;
  void halt() override;

private:
  void flagCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void ensureSharedExecutor(rclcpp::Node::SharedPtr node);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flag_sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  std::string flag_topic_;

  std::mutex mutex_;
  bool last_flag_ = false;
  bool triggered_ = true;  // 최초 1회는 실행
  bool has_last_goal_ = false;
  geometry_msgs::msg::PoseStamped last_goal_;
};

class SharedExecutor
{
public:
  static void start(rclcpp::Node::SharedPtr node)
  {
    static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
    static std::once_flag flag;
    std::call_once(flag, [&]() {
      executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
      executor->add_node(node);
      std::thread([executor]() {
        executor->spin();
      }).detach();
    });
  }
};

}  // namespace amr_bt_nodes


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
    flag_topic_ = "/decor_flag";
  }

  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

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
  if (msg->data && !last_flag_) {
    triggered_ = true;
    RCLCPP_INFO(node_->get_logger(), "[TriggerPlannerDecorator] Triggered by flag");
  }
  last_flag_ = msg->data;
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
