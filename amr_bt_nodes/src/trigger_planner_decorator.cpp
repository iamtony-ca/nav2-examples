// trigger_planner_decorator.cpp
#include "amr_bt_nodes/trigger_planner_decorator.hpp"

namespace amr_bt_nodes
{

TriggerPlannerDecorator::TriggerPlannerDecorator(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::DecoratorNode(name, config)
{
  if (!getInput("node", node_)) {
    throw BT::RuntimeError("[TriggerPlannerDecorator] Missing input [node]");
  }
  getInput("flag_topic", flag_topic_);

  // 1. 독립적인 콜백 그룹 생성
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  
  // 2. 전용 Executor에 콜백 그룹 할당
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  // 3. Subscriber 생성 시 콜백 그룹 옵션 사용
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;

  rclcpp::QoS qos(10);
  // qos.best_effort();
  
  flag_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    flag_topic_, qos,
    std::bind(&TriggerPlannerDecorator::flagCallback, this, std::placeholders::_1),
    sub_options);

  RCLCPP_INFO(node_->get_logger(), "[TriggerPlannerDecorator] Subscribed to: %s", flag_topic_.c_str());
}

void TriggerPlannerDecorator::flagCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    // std::atomic의 store 함수를 사용하여 스레드 안전하게 값 변경
    triggered_.store(true); 
    RCLCPP_INFO(node_->get_logger(), "[TriggerPlannerDecorator] Triggered by flag");
  }
}

BT::NodeStatus TriggerPlannerDecorator::tick()
{
  // 4. 매 tick마다 전용 Executor를 수동으로 실행하여 콜백 처리 보장
  callback_group_executor_.spin_some();

  if (!child_node_) {
    throw BT::RuntimeError("[TriggerPlannerDecorator] Child node is null");
  }

  // Goal 변경 시 트리거 로직은 그대로 유지
  geometry_msgs::msg::PoseStamped current_goal;
  if (getInput("goal", current_goal)) {
    if (!has_last_goal_ ||
        current_goal.pose.position.x != last_goal_.pose.position.x ||
        current_goal.pose.position.y != last_goal_.pose.position.y ||
        current_goal.pose.orientation.z != last_goal_.pose.orientation.z ||
        current_goal.pose.orientation.w != last_goal_.pose.orientation.w) {
      triggered_.store(true);
      last_goal_ = current_goal;
      has_last_goal_ = true;
      RCLCPP_INFO(node_->get_logger(), "[TriggerPlannerDecorator] Goal changed -> Triggered");
    }
  }

  // triggered_ 플래그 상태 확인 (std::atomic의 load 함수 사용)
  if (triggered_.load()) {
    setStatus(BT::NodeStatus::RUNNING);
    auto status = child_node_->executeTick();
    if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
      triggered_.store(false); // 자식 노드가 완료되면 트리거 리셋
    }
    return status;
  }

  // 트리거되지 않았을 때의 기본 행동 (기존 로직 유지)
  return BT::NodeStatus::SUCCESS;
}

void TriggerPlannerDecorator::halt()
{
  // halt()가 호출될 때 트리거 상태를 리셋하여 예기치 않은 재실행 방지
  triggered_.store(false);
  DecoratorNode::halt();
}

}  // namespace amr_bt_nodes

// #include "behaviortree_cpp/bt_factory.h"

// // BT_REGISTER_NODES는 그대로 유지합니다.
// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<amr_bt_nodes::TriggerPlannerDecorator>("TriggerPlannerDecorator");
// }

#include "behaviortree_cpp/bt_factory.h"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<amr_bt_nodes::TriggerPlannerDecorator>("TriggerPlannerDecorator");
}

















// // trigger_planner_decorator.cpp
// #include "amr_bt_nodes/trigger_planner_decorator.hpp"

// namespace amr_bt_nodes
// {

// using namespace std::chrono_literals;

// TriggerPlannerDecorator::TriggerPlannerDecorator(
//   const std::string & name,
//   const BT::NodeConfiguration & config)
// : BT::DecoratorNode(name, config)
// {
//   if (!getInput("node", node_)) {
//     throw BT::RuntimeError("[TriggerPlannerDecorator] Missing input [node]");
//   }
//   if (!getInput("flag_topic", flag_topic_)) {
//     flag_topic_ = "/replan_flag";
//   }

//   callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

//   rclcpp::QoS qos(10);
//   qos.best_effort();

//   rclcpp::SubscriptionOptions sub_options;
//   sub_options.callback_group = callback_group_;
//   flag_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
//     flag_topic_, qos,
//     std::bind(&TriggerPlannerDecorator::flagCallback, this, std::placeholders::_1),
//     sub_options);

//   // ensureSharedExecutor(node_);

//   RCLCPP_INFO(node_->get_logger(), "[TriggerPlannerDecorator] Subscribed to: %s", flag_topic_.c_str());
// }

// // void TriggerPlannerDecorator::ensureSharedExecutor(rclcpp::Node::SharedPtr node)
// // {
// //   SharedExecutor::start(node);
// // }

// void TriggerPlannerDecorator::flagCallback(const std_msgs::msg::Bool::SharedPtr msg)
// {
//   std::lock_guard<std::mutex> lock(mutex_);
//   if (msg->data) {
//     triggered_ = true;
//   }
//   last_flag_ = msg->data;
//   RCLCPP_INFO(node_->get_logger(), "[TriggerPlannerDecorator] Triggered by flag");

// }

// BT::NodeStatus TriggerPlannerDecorator::tick()
// {
//   if (!child_node_) {
//     throw BT::RuntimeError("[TriggerPlannerDecorator] Child node is null");
//   }

//   geometry_msgs::msg::PoseStamped current_goal;
//   if (getInput("goal", current_goal)) {
//     if (!has_last_goal_ ||
//         current_goal.pose.position.x != last_goal_.pose.position.x ||
//         current_goal.pose.position.y != last_goal_.pose.position.y ||
//         current_goal.pose.orientation.z != last_goal_.pose.orientation.z ||
//         current_goal.pose.orientation.w != last_goal_.pose.orientation.w) {
//       triggered_ = true;
//       last_goal_ = current_goal;
//       has_last_goal_ = true;
//       RCLCPP_INFO(node_->get_logger(), "[TriggerPlannerDecorator] Goal changed → Triggered");
//     }
//   }

//   if (triggered_) {
//     auto status = child_node_->executeTick();
//     if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
//       triggered_ = false;
//     }
//     return status;
//   }

//   return BT::NodeStatus::SUCCESS;
// }

// void TriggerPlannerDecorator::halt()
// {
//   BT::DecoratorNode::halt();
// }

// }  // namespace amr_bt_nodes

// #include "behaviortree_cpp/bt_factory.h"

// extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
// {
//   factory.registerNodeType<amr_bt_nodes::TriggerPlannerDecorator>("TriggerPlannerDecorator");
// }