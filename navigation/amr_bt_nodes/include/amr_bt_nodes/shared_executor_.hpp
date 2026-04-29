#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <mutex>
#include <thread>

namespace amr_bt_nodes
{

class SharedExecutor
{
public:
  static void ensureExecutorStarted(rclcpp::Node::SharedPtr node)
  {
    static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
    static std::once_flag flag;

    std::call_once(flag, [&]() {
      executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
      executor->add_node(node);
      std::thread([]() {
        executor->spin();
      }).detach();
    });
  }
};

}  // namespace amr_bt_nodes
