#include "velocity_modifier/velocity_modifier_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  // ROS 2 초기화
  rclcpp::init(argc, argv);

  // 병렬 콜백 처리를 위해 MultiThreadedExecutor를 생성
  rclcpp::executors::MultiThreadedExecutor executor;
  
  // 노드가 Composable 하도록 NodeOptions를 전달
  rclcpp::NodeOptions options;
  auto node = std::make_shared<velocity_modifier::VelocityModifierNode>(options);

  // 생성한 노드를 Executor에 추가
  executor.add_node(node);

  // Executor를 실행하여 노드의 콜백 처리를 시작
  executor.spin();

  // ROS 2 종료
  rclcpp::shutdown();
  return 0;
}