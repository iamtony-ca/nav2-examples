#include "rclcpp/rclcpp.hpp"
#include "replan_monitor/path_validator_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // MultiThreadedExecutor to respect callback groups
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<replan_monitor::PathValidatorNode>();
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
