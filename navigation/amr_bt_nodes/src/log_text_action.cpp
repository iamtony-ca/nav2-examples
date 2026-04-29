#include "amr_bt_nodes/log_text_action.hpp"

#include <string>
#include <memory>
#include <typeinfo>
#include <typeindex>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/basic_types.h"

namespace {
/**
 * @brief Blackboard Entry의 타입을 직접 확인하여 값은 문자열로 변환합니다.
 * 이 파일 내에서만 사용되는 static 헬퍼 함수처럼 동작합니다.
 * @param entry 확인할 Blackboard Entry
 * @return 값의 문자열 표현
 */
std::string getEntryValueAsString(const BT::Blackboard::Entry& entry)
{
  const std::type_index& type = entry.info.type();

  // 기본 타입들을 직접 확인합니다.
  if (type == typeid(std::string)) {
    return entry.value.cast<std::string>();
  }
  if (type == typeid(int)) {
    return std::to_string(entry.value.cast<int>());
  }
  if (type == typeid(double)) {
    return std::to_string(entry.value.cast<double>());
  }
  if (type == typeid(float)) {
    return std::to_string(entry.value.cast<float>());
  }
  if (type == typeid(bool)) {
    return entry.value.cast<bool>() ? "true" : "false";
  }
  if (type == typeid(long)) {
    return std::to_string(entry.value.cast<long>());
  }
  // 여기에 필요한 다른 기본 타입들을 추가할 수 있습니다.

  // 지원하지 않는 타입의 경우, 타입 이름이라도 출력해줍니다.
  return std::string("[Object of type: ") + BT::demangle(type) + "]";
}
} // end of anonymous namespace

namespace amr_bt_nodes
{

LogTextAction::LogTextAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config),
  last_log_time_(0, 0, RCL_ROS_TIME)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
      RCLCPP_FATAL(rclcpp::get_logger("LogTextAction"), "Failed to get ROS 2 node from blackboard in constructor.");
  }
}

BT::PortsList LogTextAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("message", "Text message to be logged."),
    BT::InputPort<float>("interval_s", 1.0f, "Logging interval in seconds."),
    BT::InputPort<std::string>("bb_key", "Optional: The key of the blackboard entry to log.")
  };
}

BT::NodeStatus LogTextAction::tick()
{
  if (!node_) {
      RCLCPP_ERROR(rclcpp::get_logger("LogTextAction"), "LogTextAction: ROS 2 node is not available.");
      return BT::NodeStatus::FAILURE;
  }

  BT::Expected<std::string> msg_expected = getInput<std::string>("message");
  if (!msg_expected) {
    RCLCPP_ERROR(
      node_->get_logger(), "LogTextAction: Missing 'message' input port: %s", msg_expected.error().c_str());
    return BT::NodeStatus::FAILURE;
  }
  const std::string& message = msg_expected.value();
  const float interval_s = getInput<float>("interval_s").value();
  
  rclcpp::Time current_time = node_->now();
  if ((current_time - last_log_time_).seconds() >= interval_s) {
    
    auto bb_key_expected = getInput<std::string>("bb_key");

    if (bb_key_expected) {
        const std::string& key = bb_key_expected.value();
        
        std::shared_ptr<BT::Blackboard::Entry> entry = config().blackboard->getEntry(key);

        if (entry) {
            // 익명 네임스페이스에 있는 헬퍼 함수를 호출합니다.
            std::string value_str = getEntryValueAsString(*entry);
            RCLCPP_INFO(
              node_->get_logger(), "BT Log: %s | %s: %s",
              message.c_str(), key.c_str(), value_str.c_str());
        } else {
            RCLCPP_WARN(
              node_->get_logger(), "BT Log: %s | Blackboard key '%s' not found.",
              message.c_str(), key.c_str());
        }
    } else {
        RCLCPP_INFO(node_->get_logger(), "BT Log: %s", message.c_str());
    }

    last_log_time_ = current_time;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace amr_bt_nodes


// 플러그인 등록
extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<amr_bt_nodes::LogTextAction>("LogTextAction");
}






// #include <string>
// #include <optional> // std::optional (BT::Expected 대신 사용 시)

// #include "amr_bt_nodes/log_text_action.hpp" // 자체 헤더 파일 포함
// #include "behaviortree_cpp/bt_factory.h"    // 노드 등록 매크로를 위해 필요
// #include "rclcpp/rclcpp.hpp"                // 로깅 매크로 (RCLCPP_INFO 등) 사용을 위해 필요
// #include "behaviortree_cpp/basic_types.h" // BT::Expected를 위해 필요

// namespace amr_bt_nodes
// {

// LogTextAction::LogTextAction(
//   const std::string & name,
//   const BT::NodeConfiguration & config)
// : BT::SyncActionNode(name, config),
//   last_log_time_(0, 0, RCL_ROS_TIME) // 초기 시간 0으로 설정
// {
//   // 생성자에서 ROS 2 노드 핸들을 Blackboard로부터 가져옵니다.
//   // tick() 메서드에서 매번 가져오는 것보다 효율적입니다.
//   node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
//   if (!node_) {
//       RCLCPP_FATAL(rclcpp::get_logger("LogTextAction"), "Failed to get ROS 2 node from blackboard in constructor.");
//       // 생성자에서는 FAILURE를 반환할 수 없으므로, FATAL 로깅 후 문제가 있음을 알립니다.
//       // 실제 BT 실행 시 tick()에서 다시 확인하여 FAILURE를 반환하게 됩니다.
//   }
// }

// BT::PortsList LogTextAction::providedPorts()
// {
//   // "message" Input Port: 로깅할 텍스트 메시지 (std::string)
//   // "interval_s" Input Port: 로깅 주기를 나타내는 float 값 (초 단위, 기본값 1.0초)
//   return {
//     BT::InputPort<std::string>("message", "Text message to be logged to the terminal"),
//     BT::InputPort<float>("interval_s", 1.0f, "Logging interval in seconds (e.g., 0.1 for 100ms)") // float 포트 추가 및 기본값 설정
//   };
// }

// BT::NodeStatus LogTextAction::tick()
// {
//   if (!node_) {
//       RCLCPP_ERROR(rclcpp::get_logger("LogTextAction"), "LogTextAction: ROS 2 node is not available.");
//       return BT::NodeStatus::FAILURE;
//   }

//   // 1. "message" 포트에서 문자열 데이터를 읽어옵니다.
//   BT::Expected<std::string> msg_expected = getInput<std::string>("message");
//   if (!msg_expected.has_value()) {
//     RCLCPP_ERROR(
//       node_->get_logger(),
//       "LogTextAction: Missing or invalid 'message' input port: %s",
//       msg_expected.error().c_str());
//     return BT::NodeStatus::FAILURE;
//   }
//   const std::string& message = msg_expected.value(); // 참조로 가져와 복사 방지

//   // 2. "interval_s" 포트에서 float 간격 데이터를 읽어옵니다.
//   BT::Expected<float> interval_expected = getInput<float>("interval_s");
//   if (!interval_expected.has_value()) {
//     RCLCPP_WARN(
//       node_->get_logger(),
//       "LogTextAction: Missing or invalid 'interval_s' input port. Using default interval (1.0s). Error: %s",
//       interval_expected.error().c_str());
//     // 포트가 없거나 유효하지 않으면, providedPorts()에서 설정한 기본값 (1.0f)이 사용됩니다.
//     // 하지만 확실성을 위해 한 번 더 경고를 줍니다.
//   }
//   // 기본값을 포함하여 유효한 float 값을 가져옵니다.
//   const float interval_s = interval_expected.value();

//   // 3. 현재 시간 가져오기
//   // MonoClock을 사용하여 시스템 시간으로, 시스템 시간 변경에 영향을 받지 않도록 합니다.
//   rclcpp::Time current_time = node_->now();

//   // 4. 로깅 주기 확인
//   // (current_time - last_log_time_).seconds()는 시간 차이를 float (초)로 반환합니다.
//   if ((current_time - last_log_time_).seconds() >= interval_s) {
//     // 설정된 간격 이상 시간이 지났으면 로깅을 수행합니다.
//     RCLCPP_INFO(node_->get_logger(), "BT Log: %s", message.c_str());

//     // 마지막 로깅 시간을 현재 시간으로 업데이트합니다.
//     last_log_time_ = current_time;
//   }

//   // 로깅 여부와 관계없이 노드는 항상 SUCCESS를 반환하여 Behavior Tree의 흐름을 방해하지 않습니다.
//   // 로깅은 부가적인 기능이므로, 로깅이 안 되었다고 해서 BT 전체가 실패할 필요는 없습니다.
//   return BT::NodeStatus::SUCCESS;
// }

// }  // namespace amr_bt_nodes

// extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
// {
//   factory.registerNodeType<amr_bt_nodes::LogTextAction>("LogTextAction");
// }



// <LogTextAction name="InitialLog" message="Starting the navigation task." interval_s="3.0" />
