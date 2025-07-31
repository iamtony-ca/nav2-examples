#include "rclcpp/rclcpp.hpp"
#include "robot_monitoring/status_manager.hpp" // 패키지 이름에 맞게 경로 수정

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  rclcpp::executors::MultiThreadedExecutor executor;
  auto status_manager_node = std::make_shared<StatusManager>(rclcpp::NodeOptions());
  
  executor.add_node(status_manager_node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}





// #include "velocity_modifier/velocity_modifier_node.hpp"
// #include "rclcpp/rclcpp.hpp"

// int main(int argc, char * argv[])
// {
//   // ROS 2 초기화
//   rclcpp::init(argc, argv);

//   // 병렬 콜백 처리를 위해 MultiThreadedExecutor를 생성
//   rclcpp::executors::MultiThreadedExecutor executor;
  
//   // 노드가 Composable 하도록 NodeOptions를 전달
//   rclcpp::NodeOptions options;
//   auto node = std::make_shared<velocity_modifier::VelocityModifierNode>(options);

//   // 생성한 노드를 Executor에 추가
//   executor.add_node(node);

//   // Executor를 실행하여 노드의 콜백 처리를 시작
//   executor.spin();

//   // ROS 2 종료
//   rclcpp::shutdown();
//   return 0;
// }


