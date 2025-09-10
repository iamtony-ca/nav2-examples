#include "rclcpp/rclcpp.hpp"
#include "replan_monitor/replan_monitor_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  

  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<ReplanMonitorNode>();
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}



// #include "rclcpp/rclcpp.hpp"
// #include "replan_monitor/replan_monitor_node.hpp" // 자신의 노드 헤더

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
  
//   auto node = std::make_shared<ReplanMonitorNode>();
  
//   // CHANGED: MultiThreadedExecutor 사용
//   // 기본적으로 CPU 코어 수만큼 스레드를 생성합니다.
//   rclcpp::executors::MultiThreadedExecutor executor;
//   executor.add_node(node);
//   executor.spin();
  
//   rclcpp::shutdown();
//   return 0;
// }