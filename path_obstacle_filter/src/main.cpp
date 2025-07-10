#include "rclcpp/rclcpp.hpp"
#include "path_obstacle_filter/path_obstacle_filter_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  

  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<path_obstacle_filter::PathObstacleFilterNode>(rclcpp::NodeOptions());
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}



// #include "path_obstacle_filter/path_obstacle_filter_node.hpp"
// #include "rclcpp/rclcpp.hpp"

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<path_obstacle_filter::PathObstacleFilterNode>(rclcpp::NodeOptions());
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }