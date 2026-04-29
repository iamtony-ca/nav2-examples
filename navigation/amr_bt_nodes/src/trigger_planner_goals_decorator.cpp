// trigger_planner_goals_decorator.cpp
#include "amr_bt_nodes/trigger_planner_goals_decorator.hpp"

namespace amr_bt_nodes
{

TriggerPlannerGoalsDecorator::TriggerPlannerGoalsDecorator(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::DecoratorNode(name, config)
{
  if (!getInput("node", node_)) {
    throw BT::RuntimeError("[TriggerPlannerGoalsDecorator] Missing input [node]");
  }
  getInput("flag_topic", flag_topic_);

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();
  
  flag_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    flag_topic_, qos,
    std::bind(&TriggerPlannerGoalsDecorator::flagCallback, this, std::placeholders::_1),
    sub_options);

  RCLCPP_INFO(node_->get_logger(), "[TriggerPlannerGoalsDecorator] Subscribed to: %s", flag_topic_.c_str());
}

void TriggerPlannerGoalsDecorator::flagCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    triggered_.store(true); 
    RCLCPP_INFO(node_->get_logger(), "[TriggerPlannerGoalsDecorator] Triggered by flag");
  }
}

BT::NodeStatus TriggerPlannerGoalsDecorator::tick()
{
  callback_group_executor_.spin_some();

  if (!child_node_) {
    throw BT::RuntimeError("[TriggerPlannerGoalsDecorator] Child node is null");
  }

  // Goals 변경 시 트리거 로직
  std::vector<geometry_msgs::msg::PoseStamped> current_goals;
  if (getInput("goals", current_goals)) {
    bool goals_are_different = false;

    // 1. 이전 goals 정보가 없거나, goals의 개수가 다르면 무조건 변경된 것으로 간주
    if (!has_last_goals_ || current_goals.size() != last_goals_.size()) {
      goals_are_different = true;
    } else {
      // 2. goals의 개수가 같다면, 각 pose를 순회하며 내용 비교
      for (size_t i = 0; i < current_goals.size(); ++i) {
        const auto& current_pose = current_goals[i].pose;
        const auto& last_pose = last_goals_[i].pose;
        if (current_pose.position.x != last_pose.position.x ||
            current_pose.position.y != last_pose.position.y ||
            current_pose.position.z != last_pose.position.z || // 3D 환경을 위해 z도 비교
            current_pose.orientation.x != last_pose.orientation.x ||
            current_pose.orientation.y != last_pose.orientation.y ||
            current_pose.orientation.z != last_pose.orientation.z ||
            current_pose.orientation.w != last_pose.orientation.w)
        {
          goals_are_different = true;
          break; // 하나라도 다르면 더 이상 비교할 필요 없음
        }
      }
    }

    if (goals_are_different) {
      triggered_.store(true);
      last_goals_ = current_goals;
      has_last_goals_ = true;
      RCLCPP_INFO(
        node_->get_logger(), "[TriggerPlannerGoalsDecorator] Goals changed (%zu waypoints) -> Triggered",
        current_goals.size());
    }
  }

  if (triggered_.load()) {
    setStatus(BT::NodeStatus::RUNNING);
    auto status = child_node_->executeTick();
    if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
      triggered_.store(false); // 자식 노드가 완료되면 트리거 리셋
    }
    return status;
  }

  return BT::NodeStatus::SUCCESS;
}

void TriggerPlannerGoalsDecorator::halt()
{
  triggered_.store(false);
  DecoratorNode::halt();
}

}  // namespace amr_bt_nodes

#include "behaviortree_cpp/bt_factory.h"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<amr_bt_nodes::TriggerPlannerGoalsDecorator>("TriggerPlannerGoalsDecorator");
}