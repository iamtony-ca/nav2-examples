#include "rclcpp/rclcpp.hpp"
#include "my_dummy_node/dummy_agent_publisher.hpp"

int main(int argc, char * argv[])
{
  // 1. ROS 2 컨텍스트 초기화
  rclcpp::init(argc, argv);

  // 2. 노드 생성 (스마트 포인터)
  auto node = std::make_shared<my_dummy_node::DummyAgentPublisher>();

  // 3. 실행 (스핀)
  rclcpp::spin(node);

  // 4. 종료 처리
  rclcpp::shutdown();
  return 0;
}