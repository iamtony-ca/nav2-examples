#pragma once

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <mutex>
#include <atomic>

namespace amr_bt_nodes
{

class CheckFlagCondition : public BT::ConditionNode
{
public:
  CheckFlagCondition(const std::string &name, const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("flag_topic", "/mission_flag", "Topic name to subscribe for flag"),
      BT::InputPort<rclcpp::Node::SharedPtr>("node", "Shared ROS2 node")
    };
  }

  BT::NodeStatus tick() override;

private:
  void flagCallback(const std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flag_sub_;
  std::string flag_topic_;
  std::atomic<bool> last_flag_{false}; // Use std::atomic for thread safety

  // PlannerSelector 패턴 적용
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
};

}  // namespace amr_bt_nodes













// #pragma once

// #include <behaviortree_cpp/condition_node.h>
// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/bool.hpp>
// #include <mutex>

// namespace amr_bt_nodes
// {

// class CheckFlagCondition : public BT::ConditionNode
// {
// public:
//   CheckFlagCondition(const std::string &name, const BT::NodeConfiguration &config);
//   ~CheckFlagCondition() override;

//   static BT::PortsList providedPorts()
//   {
//     return {
//       BT::InputPort<std::string>("flag_topic", "/mission_flag", "Topic name to subscribe for flag"),
//       BT::InputPort<rclcpp::Node::SharedPtr>("node", "Shared ROS2 node")
//     };
//   }

//   BT::NodeStatus tick() override;

// private:
//   void flagCallback(const std_msgs::msg::Bool::SharedPtr msg);

//   rclcpp::Node::SharedPtr node_;
//   rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flag_sub_;
//   rclcpp::CallbackGroup::SharedPtr callback_group_; // Callback group for subscription

//   std::string flag_topic_;
//   std::atomic_bool flag_ok_{true};  // Default to true for initial state
//   std::mutex flag_mutex_;
//   bool current_flag = true;  // Default to true for initial state
// };

// // class SharedExecutor
// // {
// // public:
// //   static void ensureExecutorStarted(rclcpp::Node::SharedPtr node)
// //   {
// //     static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
// //     static std::once_flag flag;

// //     std::call_once(flag, [&]() {
// //       executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
// //       executor->add_node(node);
// //       std::thread([]() {
// //         executor->spin();
// //       }).detach();
// //     });
// //   }
// // };

// }  // namespace amr_bt_nodes
