#include <string>
#include <optional> // std::optional을 사용하기 위해 필요 (이전 코드와의 호환성을 위해 유지할 수도 있지만, 여기서는 BT::Expected를 사용)

#include "amr_bt_nodes/log_text_action.hpp" // 자체 헤더 파일 포함
#include "behaviortree_cpp/bt_factory.h"    // 노드 등록 매크로를 위해 필요
#include "rclcpp/rclcpp.hpp"                // 로깅 매크로 (RCLCPP_INFO 등) 사용을 위해 필요

// BehaviorTree.CPP v3에서 BT::Expected 사용을 위한 헤더 (명시적으로 포함하는 것이 좋습니다)
#include "behaviortree_cpp/basic_types.h" // BT::Expected를 포함할 가능성이 높습니다.

namespace amr_bt_nodes
{

LogTextAction::LogTextAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  // 생성자에서는 특별한 초기화가 필요하지 않습니다.
  // 필요한 ROS 노드 핸들 등은 tick()에서 blackboard를 통해 접근할 수 있습니다.
}

BT::PortsList LogTextAction::providedPorts()
{
  // "message"라는 이름의 Input Port를 정의합니다.
  // 이 포트를 통해 BT XML에서 로깅할 텍스트를 전달받을 것입니다.
  return {
    BT::InputPort<std::string>("message", "Text message to be logged to the terminal")
  };
}

BT::NodeStatus LogTextAction::tick()
{
  // Blackboard에서 rclcpp::Node의 공유 포인터를 가져옵니다.
  // Nav2 환경에서는 'node'라는 이름으로 기본적으로 제공됩니다.
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node) {
      RCLCPP_ERROR(rclcpp::get_logger("LogTextAction"), "Failed to get ROS 2 node from blackboard.");
      return BT::NodeStatus::FAILURE;
  }

  // "message" 포트에서 문자열 데이터를 읽어옵니다.
  // getInput()은 BT::Expected<T>를 반환합니다.
  // BT::Expected<T>는 값이 성공적으로 로드되었는지 (has_value()),
  // 아니면 에러가 발생했는지 (error())를 나타냅니다.
  BT::Expected<std::string> msg_expected = getInput<std::string>("message");

  // 메시지가 유효한지 확인합니다.
  // has_value()를 사용하여 값이 존재하는지 확인합니다.
  if (!msg_expected.has_value()) {
    // 메시지가 없거나 유효하지 않으면 에러를 로깅하고 실패를 반환합니다.
    // error() 메서드로 에러 문자열을 가져올 수 있습니다.
    RCLCPP_ERROR(
      node->get_logger(),
      "LogTextAction: Missing or invalid 'message' input port: %s",
      msg_expected.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  // 성공적으로 메시지를 가져왔으므로 터미널에 출력합니다.
  // value() 메서드로 실제 값을 가져옵니다.
  RCLCPP_INFO(node->get_logger(), "BT Log: %s", msg_expected.value().c_str());

  // 로깅 작업은 항상 성공했다고 가정하고 SUCCESS를 반환합니다.
  return BT::NodeStatus::SUCCESS;
}

}  // namespace amr_bt_nodes


// 외부 C 함수로 노드를 등록합니다.
// BehaviorTreeFactory가 이 함수를 호출하여 플러그인을 로드합니다.
extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<amr_bt_nodes::LogTextAction>("LogTextAction");
}