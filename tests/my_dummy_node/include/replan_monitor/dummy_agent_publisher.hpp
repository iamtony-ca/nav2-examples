#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

// Custom Messages
#include "multi_agent_msgs/msg/multi_agent_info_array.hpp"
#include "multi_agent_msgs/msg/agent_status.hpp"

// Standard ROS Messages
#include "geometry_msgs/msg/point32.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace my_dummy_node
{

class DummyAgentPublisher : public rclcpp::Node
{
public:
  /**
   * @brief 생성자
   */
  explicit DummyAgentPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief 소멸자
   */
  virtual ~DummyAgentPublisher();

private:
  /**
   * @brief 주기적으로 실행되는 타이머 콜백
   */
  void timer_callback();

  /**
   * @brief 개별 에이전트의 더미 데이터를 생성하는 헬퍼 함수
   * * @param id 에이전트 ID
   * @param type 에이전트 타입 (예: LIFT, TOW)
   * @param radius 회전 반경
   * @param speed 이동 속도 계수
   * @return 생성된 MultiAgentInfo 메시지
   */
  multi_agent_msgs::msg::MultiAgentInfo create_dummy_agent(
    uint16_t id, const std::string & type, double radius, double speed);

  // 멤버 변수
  rclcpp::Publisher<multi_agent_msgs::msg::MultiAgentInfoArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

} // namespace my_dummy_node