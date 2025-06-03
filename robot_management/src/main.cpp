#include "rclcpp/rclcpp.hpp"
#include "robot_management/task_management.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);


  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<TaskManagement>();
  node->initialize();  // Initialize parameters and other setup
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}