#pragma once

// BT.CPP 액션 노드 베이스 클래스
#include <behaviortree_cpp/action_node.h>
// ROS2 핵심 라이브러리
#include <rclcpp/rclcpp.hpp>
// ROS2 Action 클라이언트를 위한 Nav2의 베이스 클래스 (NodeBase를 포함)
#include <nav2_behavior_tree/bt_action_node.hpp> // BtActionNodeBase가 포함될 수 있음
#include <nav2_util/lifecycle_node.hpp> // 노드 라이프사이클 관리를 위해

// Standard C++ 라이브러리
#include <mutex>          // std::mutex, std::lock_guard
#include <thread>         // std::thread (SharedExecutor에서 사용)
#include <functional>     // std::bind, std::placeholders
#include <chrono>         // std::chrono::seconds, milliseconds (wait_for, sleep_for)
#include <future>         // std::shared_future, std::future_status

// ROS2 메시지 및 서비스 타입 (당신의 패키지 이름으로 변경)
#include "robot_interfaces/msg/navigation_command.hpp" // 토픽용 메시지
#include "robot_interfaces/srv/set_navigation_mode.hpp" // 서비스용 메시지
#include <std_msgs/msg/string.hpp>                    // 응답 메시지 타입

namespace amr_bt_nodes
{

// SharedExecutor 클래스: ROS2 노드를 별도의 스레드에서 스핀하여 비동기 콜백 처리
// 이 클래스는 BT 노드 컨텍스트 외부에서, BT가 실행되는 메인 ROS2 노드에서 관리될 수 있도록 설계됩니다.
// 여기서는 `TriggerReceiverAndParser` 내부에 직접 포함시키기 보다는,
// Nav2의 `BehaviorTreeEngine`이 제공하는 `rclcpp::Node::SharedPtr`를 사용하여
// 해당 노드를 MultiThreadedExecutor에 추가하고 스핀하는 방식을 가정합니다.
// 따라서 이 클래스 자체는 이 헤더에 직접 포함되기 보단, BT를 로드하는 메인 애플리케이션에 정의될 수 있습니다.
// 하지만 예시를 위해 여기에 그대로 두겠습니다.
class SharedExecutor
{
public:
  static void start(rclcpp::Node::SharedPtr node)
  {
    // 정적 변수들은 한 번만 초기화됩니다.
    static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
    static std::once_flag flag;

    // std::call_once를 사용하여 멀티스레드 환경에서 단 한 번만 실행되도록 보장
    std::call_once(flag, [&]() {
      executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
      executor->add_node(node); // 주어진 ROS2 노드를 실행기에 추가

      // 새로운 스레드를 생성하여 실행기를 스핀합니다.
      // detach()를 사용하여 스레드를 분리합니다.
      std::thread([executor]() {
        executor->spin();
      }).detach();
      RCLCPP_INFO(node->get_logger(), "Shared ROS2 MultiThreadedExecutor started in background.");
    });
  }
};


// TriggerReceiverAndParser 액션 노드 정의
// Nav2의 BtActionNode는 ROS 액션에 특화되므로, 토픽/서비스를 위해 ActionNodeBase를 직접 상속
// 하지만, ROS 노드 포인터를 받는 방식은 Nav2의 노드와 유사하게 구현합니다.
class TriggerReceiverAndParser : public BT::ActionNodeBase
{
public:
  // 노드 생성자: 이름과 설정을 받고, ROS2 노드 포인터를 명시적으로 받음 (XML 포트 'node'를 통해)
  TriggerReceiverAndParser(const std::string& name, const BT::NodeConfiguration& config);

  // 소멸자 (리소스 해제)
  ~TriggerReceiverAndParser() override;

  // // BT.CPP에 노드의 포트(입력/출력)를 선언하는 정적 메서드
  // static BT::PortsList providedPorts();

  // // BT 노드의 핵심 로직 (매 틱마다 호출됨)
  // BT::NodeStatus tick() override;

  // // BT 노드가 중단될 때 호출되는 메서드
  // void halt() override;

private:
  // // ROS2 노드 핸들 (생성자에서 초기화하며, BtActionNode와 유사하게 관리)
  // // Nav2 컨텍스트 내의 노드를 참조하므로 shared_ptr로 관리
  // rclcpp::Node::SharedPtr node_;

  // // --- 토픽 관련 멤버 ---
  // rclcpp::Subscription<robot_interfaces::msg::NavigationCommand>::SharedPtr subscription_;
  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  // robot_interfaces::msg::NavigationCommand::SharedPtr last_received_topic_msg_;
  // bool new_topic_msg_received_; // 새 토픽 메시지 수신 여부 플래그
  // std::string input_topic_name_;
  // std::string output_topic_name_;
  // // 토픽 콜백 함수
  // void triggerMessageCallback(const robot_interfaces::msg::NavigationCommand::SharedPtr msg);

  // // --- 서비스 관련 멤버 ---
  // rclcpp::Client<robot_interfaces::srv::SetNavigationMode>::SharedPtr service_client_;
  // rclcpp::Client<robot_interfaces::srv::SetNavigationMode>::SharedPtr::SharedFuture service_future_;
  // bool service_request_sent_; // 서비스 요청을 보냈고 응답을 기다리는 중인지 여부
  // std::string service_name_;
  // std::string service_trigger_key_; // Blackboard에서 서비스 호출을 트리거할 키 이름
  // std::string service_mode_key_;    // 서비스 요청에 사용할 모드 (Blackboard 키 이름)

  // // --- 공통 멤버 ---
  // std::string output_planner_id_key_;   // Blackboard 출력 키 이름
  // std::string output_controller_id_key_; // Blackboard 출력 키 이름

  // // 공유 데이터 보호를 위한 뮤텍스
  // std::mutex mutex_;

  // // 콜백 그룹 (Reentrant 타입으로 생성하여 멀티스레드 Executor에서 병렬 처리 가능하게 함)
  // rclcpp::CallbackGroup::SharedPtr callback_group_;
};

} // namespace amr_bt_nodes