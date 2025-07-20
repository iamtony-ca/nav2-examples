// trigger_planner_decorator.hpp
#pragma once

#include <behaviortree_cpp/decorator_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <atomic> // std::mutex 대신 std::atomic 사용

namespace amr_bt_nodes
{

class TriggerPlannerDecorator : public BT::DecoratorNode
{
public:
  TriggerPlannerDecorator(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<rclcpp::Node::SharedPtr>("node", "Shared ROS node"),
      BT::InputPort<std::string>("flag_topic", "/replan_flag", "Bool flag topic"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Target goal pose")
    };
  }

  BT::NodeStatus tick() override;
  void halt() override;

private:
  void flagCallback(const std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flag_sub_;
  
  // 전용 Executor 패턴 적용
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  std::string flag_topic_;

  // 스레드 안전성을 위해 std::atomic 사용
  std::atomic<bool> triggered_{true};  // 최초 1회는 실행
  
  // goal 비교 로직을 위한 멤버 변수들
  bool has_last_goal_{false};
  geometry_msgs::msg::PoseStamped last_goal_;
};

}  // namespace amr_bt_nodes


// // trigger_planner_decorator.hpp
// #pragma once

// #include <behaviortree_cpp/decorator_node.h>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/bool.hpp>
// #include <mutex>

// namespace amr_bt_nodes
// {

// class TriggerPlannerDecorator : public BT::DecoratorNode
// {
// public:
//   TriggerPlannerDecorator(const std::string & name, const BT::NodeConfiguration & config);
//   ~TriggerPlannerDecorator() override = default;

//   static BT::PortsList providedPorts()
//   {
//     return {
//       BT::InputPort<rclcpp::Node::SharedPtr>("node", "Shared ROS node"),
//       BT::InputPort<std::string>("flag_topic", "/replan_flag", "Bool flag topic"),
//       BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Target goal pose")
//     };
//   }

//   BT::NodeStatus tick() override;
//   void halt() override;

// private:
//   void flagCallback(const std_msgs::msg::Bool::SharedPtr msg);
//   // void ensureSharedExecutor(rclcpp::Node::SharedPtr node);

//   rclcpp::Node::SharedPtr node_;
//   rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flag_sub_;
//   rclcpp::CallbackGroup::SharedPtr callback_group_;

//   std::string flag_topic_;

//   std::mutex mutex_;
//   bool last_flag_ = false;
//   bool triggered_ = true;  // 최초 1회는 실행
//   bool has_last_goal_ = false;
//   geometry_msgs::msg::PoseStamped last_goal_;
// };

// // class SharedExecutor
// // {
// // public:
// //   static void start(rclcpp::Node::SharedPtr node)
// //   {
// //     static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
// //     static std::once_flag flag;
// //     std::call_once(flag, [&]() {
// //       executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
// //       executor->add_node(node);
// //       std::thread([executor]() {
// //         executor->spin();
// //       }).detach();
// //     });
// //   }
// // };

// }  // namespace amr_bt_nodes