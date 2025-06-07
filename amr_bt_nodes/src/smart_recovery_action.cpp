#include "amr_bt_nodes/smart_recovery_action.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <thread>

namespace amr_bt_nodes
{
// ── SharedExecutor ------------------------------------------------
void SharedExecutor::start(const rclcpp::Node::SharedPtr& node)
{
  static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec;
  static std::once_flag once;
  std::call_once(once, [&]{
    exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    std::thread([local_exec = exec]{ local_exec->spin(); }).detach();
  });
  exec->add_node(node);
}

// ── Constructor ---------------------------------------------------
SmartRecoveryAction::SmartRecoveryAction(const std::string& name,
                                         const BT::NodeConfiguration& cfg)
: BT::StatefulActionNode(name, cfg)
{
  node_     = rclcpp::Node::make_shared("smart_recovery_bt_node");
  cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  action_client_ =
      rclcpp_action::create_client<SmartRecovery>(node_, "smart_recovery",
                                                  cb_group_);

  SharedExecutor::start(node_);
}

// ── onStart -------------------------------------------------------
BT::NodeStatus SmartRecoveryAction::onStart()
{
  if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(node_->get_logger(), "smart_recovery action server not available");
    return BT::NodeStatus::FAILURE;
  }

  SmartRecovery::Goal goal;
  getInput("backup_distance", goal.backup_distance);
  getInput("speed",           goal.speed);
  getInput("goal_pose",       goal.goal_pose);

  rclcpp_action::Client<SmartRecovery>::SendGoalOptions opts;
  opts.feedback_callback =
      std::bind(&SmartRecoveryAction::feedbackCb, this,
                std::placeholders::_1, std::placeholders::_2);

  auto gh_future = action_client_->async_send_goal(goal, opts);
  goal_handle_   = gh_future.get();
  if (!goal_handle_) {
    RCLCPP_ERROR(node_->get_logger(), "Goal rejected by SmartRecoveryServer");
    return BT::NodeStatus::FAILURE;
  }

  result_future_ = action_client_->async_get_result(goal_handle_);
  return BT::NodeStatus::RUNNING;
}

// ── onRunning -----------------------------------------------------
BT::NodeStatus SmartRecoveryAction::onRunning()
{
  if (result_future_.valid() &&
      result_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
  {
    auto res = result_future_.get();
    return (res.code == rclcpp_action::ResultCode::SUCCEEDED)
           ? BT::NodeStatus::SUCCESS
           : BT::NodeStatus::FAILURE;
  }
  setOutput("progress", last_progress_);
  return BT::NodeStatus::RUNNING;
}

// ── onHalted ------------------------------------------------------
void SmartRecoveryAction::onHalted()
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (goal_handle_)
    action_client_->async_cancel_goal(goal_handle_);
}

// ── feedback callback --------------------------------------------
void SmartRecoveryAction::feedbackCb(
    GoalHandleRecovery::SharedPtr /*gh*/,
    SmartRecovery::Feedback::ConstSharedPtr fb)
{
  std::lock_guard<std::mutex> lk(mtx_);
  last_progress_ = fb->progress;
  RCLCPP_DEBUG(node_->get_logger(), "SmartRecovery progress %.1f %%", fb->progress * 100.0f);
}

}  // namespace amr_bt_nodes

// ── BT plugin registration ---------------------------------------
extern "C"
void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<amr_bt_nodes::SmartRecoveryAction>("SmartRecoveryAction");
}
