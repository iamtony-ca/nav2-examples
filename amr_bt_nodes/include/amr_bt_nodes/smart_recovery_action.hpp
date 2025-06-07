#pragma once
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_interfaces/action/smart_recovery.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mutex>

namespace amr_bt_nodes
{
using SmartRecovery      = robot_interfaces::action::SmartRecovery;
using GoalHandleRecovery = rclcpp_action::ClientGoalHandle<SmartRecovery>;
using WrappedResult      = GoalHandleRecovery::WrappedResult;
using WrappedResultFuture= std::shared_future<WrappedResult>;

class SharedExecutor
{
public:
  static void start(const rclcpp::Node::SharedPtr& node);
};

class SmartRecoveryAction : public BT::StatefulActionNode
{
public:
  SmartRecoveryAction(const std::string& name,
                      const BT::NodeConfiguration& cfg);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<float>("backup_distance", 0.5f,
                           "meters to reverse"),
      BT::InputPort<float>("speed", 0.2f,
                           "reverse speed"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose",
                           "NavigateToPose goal"),
      BT::OutputPort<float>("progress", "0 – 1")
    };
  }

private:
  BT::NodeStatus onStart()   override;
  BT::NodeStatus onRunning() override;
  void           onHalted()  override;

  void feedbackCb(GoalHandleRecovery::SharedPtr,
                  SmartRecovery::Feedback::ConstSharedPtr);

  rclcpp::Node::SharedPtr                         node_;
  rclcpp::CallbackGroup::SharedPtr                cb_group_;
  rclcpp_action::Client<SmartRecovery>::SharedPtr action_client_;

  GoalHandleRecovery::SharedPtr  goal_handle_;
  WrappedResultFuture            result_future_;
  std::mutex                     mtx_;
  float                          last_progress_{0.f};
};
}  // namespace amr_bt_nodes
