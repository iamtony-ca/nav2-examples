#include "amr_bt_nodes/check_flag_condition.hpp"

namespace amr_bt_nodes
{

CheckFlagCondition::CheckFlagCondition(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  // 1. Get the node from the blackboard
  if (!getInput("node", node_)) {
    throw BT::RuntimeError("[CheckFlagCondition] Missing required input [node]");
  }
  getInput("flag_topic", flag_topic_);

  // 2. Create a callback group that is not automatically added to an executor
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  
  // 3. Add the callback group to our private executor
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  // 4. Create the subscription with our callback group
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;
  
  rclcpp::QoS qos(10);
  flag_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    flag_topic_, qos,
    std::bind(&CheckFlagCondition::flagCallback, this, std::placeholders::_1),
    sub_options);

  RCLCPP_INFO(node_->get_logger(), "[CheckFlagCondition] Subscribed to topic: %s", flag_topic_.c_str());
}

void CheckFlagCondition::flagCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  last_flag_.store(msg->data);
}

BT::NodeStatus CheckFlagCondition::tick()
{
  // 5. Manually spin our private executor on each tick to process callbacks
  callback_group_executor_.spin_some();

  if (last_flag_.load()) {
    // last_flag_.store(false);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace amr_bt_nodes

// #include "behaviortree_cpp/bt_factory.h"

// // BT_REGISTER_NODES는 그대로 유지합니다.
// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<amr_bt_nodes::CheckFlagCondition>("CheckFlagCondition");
// }

#include "behaviortree_cpp/bt_factory.h"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<amr_bt_nodes::CheckFlagCondition>("CheckFlagCondition");
}
















// #include "amr_bt_nodes/check_flag_condition.hpp"

// namespace amr_bt_nodes
// {

// CheckFlagCondition::CheckFlagCondition(
//   const std::string & name,
//   const BT::NodeConfiguration & config)
// : BT::ConditionNode(name, config), flag_ok_(false)
// {
//   if (!getInput("node", node_)) {
//     throw BT::RuntimeError("[CheckFlagCondition] Missing required input [node]");
//   }

//   if (!getInput("flag_topic", flag_topic_)) {
//     flag_topic_ = "/mission_flag";
//   }

//   // 1. 콜백 그룹 생성 (MutuallyExclusive는 한 번에 하나의 콜백만 실행되도록 보장)
//   callback_group_ = node_->create_callback_group(
//     rclcpp::CallbackGroupType::MutuallyExclusive);

//   // 2. Subscriber 옵션 생성 및 콜백 그룹 할당
//   auto sub_options = rclcpp::SubscriptionOptions();
//   sub_options.callback_group = callback_group_;


//   rclcpp::QoS qos(10);
//   // qos.best_effort();

//   flag_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
//     flag_topic_, qos,
//     std::bind(&CheckFlagCondition::flagCallback, this, std::placeholders::_1), sub_options);

//   // SharedExecutor::ensureExecutorStarted(node_);

//   RCLCPP_INFO(node_->get_logger(), "[CheckFlagCondition] Subscribed to topic: %s", flag_topic_.c_str());
// }

// CheckFlagCondition::~CheckFlagCondition()
// {
// }

// void CheckFlagCondition::flagCallback(const std_msgs::msg::Bool::SharedPtr msg)
// {
//   {
//     std::lock_guard<std::mutex> lock(flag_mutex_);
//     flag_ok_ = msg->data;
//   }
//   RCLCPP_INFO(node_->get_logger(), "[CheckFlagCondition] Received flag: %s", msg->data ? "true" : "false");

// }

// BT::NodeStatus CheckFlagCondition::tick()
// {
//   current_flag = false;
//   // bool current_flag = false;
//   {
//     std::lock_guard<std::mutex> lock(flag_mutex_);
//     current_flag = flag_ok_;
//     // flag_ok_ = false; // testttt..
//   }
  

//   RCLCPP_DEBUG(node_->get_logger(), "[CheckFlagCondition] Flag: %s", current_flag ? "true" : "false");

//   return current_flag ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
// }

// }  // namespace amr_bt_nodes

// #include "behaviortree_cpp/bt_factory.h"

// extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
// {
//   factory.registerNodeType<amr_bt_nodes::CheckFlagCondition>("CheckFlagCondition");
// }
