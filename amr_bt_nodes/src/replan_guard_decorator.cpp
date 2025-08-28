// replan_guard_decorator.cpp
#include "amr_bt_nodes/replan_guard_decorator.hpp"

namespace amr_bt_nodes
{

ReplanGuardDecorator::ReplanGuardDecorator(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::DecoratorNode(name, config)
{
  if (!getInput("node", node_)) {
    throw BT::RuntimeError("[ReplanGuardDecorator] Missing input [node]");
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
    std::bind(&ReplanGuardDecorator::flagCallback, this, std::placeholders::_1),
    sub_options);

  RCLCPP_INFO(node_->get_logger(), "[ReplanGuardDecorator] Subscribed to: %s", flag_topic_.c_str());
}

void ReplanGuardDecorator::flagCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    triggered_.store(true); 
    RCLCPP_INFO(node_->get_logger(), "[ReplanGuardDecorator] Triggered by flag");
  }
}

BT::NodeStatus ReplanGuardDecorator::tick()
{
  callback_group_executor_.spin_some();

  if (!child_node_) {
    throw BT::RuntimeError("[ReplanGuardDecorator] Child node is null");
  }

  std::string blackboard_status;
  // getInput은 포트가 연결되지 않았을 경우 기본값을 반환합니다.
  getInput("bt_blackboard_status", blackboard_status);
  if (blackboard_status == "recovery_mode") {
    triggered_.store(true);
    RCLCPP_INFO(node_->get_logger(), "[ReplanGuardDecorator] Triggered by recovery_mode status");
  }

  std::vector<geometry_msgs::msg::PoseStamped> current_goals;
  if (getInput("goals", current_goals)) {
    bool trigger_due_to_goal = false;

    if ((!has_last_goals_ && !current_goals.empty()) ||
        (!last_goals_.empty() && !current_goals.empty()))
    {
      if (has_last_goals_) {
        const auto& current_last_pose = current_goals.back().pose;
        const auto& previous_last_pose = last_goals_.back().pose;
        if (current_last_pose.position.x != previous_last_pose.position.x ||
            current_last_pose.position.y != previous_last_pose.position.y ||
            current_last_pose.position.z != previous_last_pose.position.z ||
            current_last_pose.orientation.x != previous_last_pose.orientation.x ||
            current_last_pose.orientation.y != previous_last_pose.orientation.y ||
            current_last_pose.orientation.z != previous_last_pose.orientation.z ||
            current_last_pose.orientation.w != previous_last_pose.orientation.w) {
          trigger_due_to_goal = true;
        }
      } else {
        trigger_due_to_goal = true;
      }
    }
    
    if (trigger_due_to_goal) {
      triggered_.store(true);
      RCLCPP_INFO(node_->get_logger(), "[ReplanGuardDecorator] Last goal changed -> Triggered");
    }

    last_goals_ = current_goals;
    has_last_goals_ = true;
  }

  if (triggered_.load()) {
    setOutput("bt_blackboard_status", "normal");
    RCLCPP_INFO(node_->get_logger(), "[ReplanGuardDecorator] Setting bt_blackboard_status to 'normal'");

    setStatus(BT::NodeStatus::RUNNING);
    auto status = child_node_->executeTick();
    if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
      triggered_.store(false); 
    }
    return status;
  }

  return BT::NodeStatus::SUCCESS;
}

void ReplanGuardDecorator::halt()
{
  triggered_.store(false);
  DecoratorNode::halt();
}

}  // namespace amr_bt_nodes


#include "behaviortree_cpp/bt_factory.h"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<amr_bt_nodes::ReplanGuardDecorator>("ReplanGuardDecorator");
}