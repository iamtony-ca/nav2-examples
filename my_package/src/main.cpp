#include "my_package/my_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create the node
  auto node = std::make_shared<MyNode>();

  // Create a MultiThreadedExecutor to allow parallel callback execution
  rclcpp::executors::MultiThreadedExecutor executor;

  // Register the node to the executor
  executor.add_node(node);

  // Spin
  executor.spin();

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}