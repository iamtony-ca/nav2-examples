#include "amr_bt_nodes/is_blackboard_changed_condition.hpp"

namespace amr_bt_nodes
{


IsBlackboardChangedCondition::IsBlackboardChangedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
    // Nav2 BT 노드들은 일반적으로 'node'라는 이름으로 rclcpp::Node::SharedPtr를 블랙보드에서 가져옵니다.
    // 로깅을 위해 이를 활용합니다.
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    // logger_ = node_->get_logger();
}

// IsBlackboardChangedCondition::IsBlackboardChangedCondition(
//   const std::string & condition_name,
//   const BT::NodeConfiguration & conf)
// : BT::ConditionNode(condition_name, conf),
//   node_(config().blackboard->get<rclcpp::Node::SharedPtr>("node")),
//   logger_(node_->get_logger())
// {
// }

BT::NodeStatus IsBlackboardChangedCondition::tick()
{
  // *** 수정된 부분: std::any 대신 std::string 타입으로 입력을 받습니다. ***
  BT::Expected<std::string> get_result = getInput<std::string>("blackboard_entry");
  if (!get_result) {
    RCLCPP_ERROR(
      node_->get_logger(), "IsBlackboardChangedCondition: failed to get input [blackboard_entry]: %s",
      get_result.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  const std::string& current_value = get_result.value();

  // 첫 번째 tick에서는 이전 값이 없으므로, 현재 값을 저장하고 SUCCESS를 반환합니다.
  if (!first_tick_done_) {
    previous_value_ = current_value;
    first_tick_done_ = true;
    RCLCPP_INFO(node_->get_logger(), "IsBlackboardChangedCondition: First tick, storing initial value: [%s]", current_value.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  // *** 수정된 부분: 타입 검사 없이 바로 문자열을 비교합니다. ***
  if (previous_value_ != current_value) {
    RCLCPP_INFO(
      node_->get_logger(),
      "IsBlackboardChangedCondition: Blackboard entry value changed from [%s] to [%s]. Triggering FAILURE.",
      previous_value_.c_str(), current_value.c_str());
    previous_value_ = current_value; // 값 업데이트
    return BT::NodeStatus::FAILURE;
  }

  // 값이 변경되지 않았으면 SUCCESS를 반환합니다.
  RCLCPP_INFO(
      node_->get_logger(),
      "IsBlackboardChangedCondition: Blackboard entry value unchanged from [%s]. Returning SUCCESS.",
      current_value.c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace amr_bt_nodes

// #include "behaviortree_cpp/bt_factory.h"

// // 플러그인을 등록하기 위한 정식 방법
// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<amr_bt_nodes::IsBlackboardChangedCondition>("IsBlackboardChangedCondition");
// }


#include "behaviortree_cpp/bt_factory.h"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<amr_bt_nodes::IsBlackboardChangedCondition>("IsBlackboardChangedCondition");
}















// #include "amr_bt_nodes/is_blackboard_changed_condition.hpp" 
// #include <typeindex>

// namespace amr_bt_nodes
// {

// IsBlackboardChangedCondition::IsBlackboardChangedCondition(
//   const std::string & condition_name,
//   const BT::NodeConfiguration & conf)
// : BT::ConditionNode(condition_name, conf)
// {
//     // Nav2 BT 노드들은 일반적으로 'node'라는 이름으로 rclcpp::Node::SharedPtr를 블랙보드에서 가져옵니다.
//     // 로깅을 위해 이를 활용합니다.
//     node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
//     // logger_ = node_->get_logger();
// }

// BT::NodeStatus IsBlackboardChangedCondition::tick()
// {
//   // 블랙보드에서 모니터링할 엔트리 값을 가져옵니다.
//   BT::Expected<std::any> get_result = getInput<std::any>("blackboard_entry");
//   if (!get_result) {
//     RCLCPP_ERROR(
//       node_->get_logger(), "IsBlackboardChangedCondition: failed to get input [blackboard_entry]: %s",
//       get_result.error().c_str());
//     return BT::NodeStatus::FAILURE;
//   }

//   const std::any& current_value = get_result.value();

//   // 첫 번째 tick에서는 이전 값이 없으므로, 현재 값을 저장하고 SUCCESS를 반환합니다.
//   // 이렇게 해야 BT가 처음 실행될 때 불필요하게 FAILURE를 반환하지 않습니다.
//   if (!first_tick_done_) {
//     previous_value_ = current_value;
//     first_tick_done_ = true;
//     RCLCPP_DEBUG(node_->get_logger(), "IsBlackboardChangedCondition: First tick, storing initial value.");
//     return BT::NodeStatus::SUCCESS;
//   }

//   // 이전 값과 현재 값의 타입이 다른지 확인합니다.
//   if (previous_value_.type() != current_value.type()) {
//     RCLCPP_INFO(
//       node_->get_logger(),
//       "IsBlackboardChangedCondition: Value type changed. Old: %s, New: %s. Triggering FAILURE.",
//       previous_value_.type().name(), current_value.type().name());
//     previous_value_ = current_value; // 다음 비교를 위해 현재 값을 저장합니다.
//     return BT::NodeStatus::FAILURE;
//   }

//   // 타입이 같다면 실제 값을 비교합니다.
//   // std::any는 직접 비교가 불가능하므로, 타입을 확인하고 캐스팅해야 합니다.
//   // 여기서는 가장 일반적인 타입인 std::string을 예시로 비교합니다.
//   if (current_value.type() == typeid(std::string)) {
//     const auto& prev_str = std::any_cast<const std::string&>(previous_value_);
//     const auto& curr_str = std::any_cast<const std::string&>(current_value);
//     if (prev_str != curr_str) {
//       RCLCPP_INFO(
//         node_->get_logger(),
//         "IsBlackboardChangedCondition: Blackboard entry value changed from [%s] to [%s]. Triggering FAILURE.",
//         prev_str.c_str(), curr_str.c_str());
//       previous_value_ = current_value; // 값 업데이트
//       return BT::NodeStatus::FAILURE;
//     }
//   }
//   // 다른 타입(int, double 등)에 대한 비교 로직을 필요에 따라 추가할 수 있습니다.
//   // else if (current_value.type() == typeid(int)) { ... }
//   // else if (current_value.type() == typeid(double)) { ... }

//   // 값이 변경되지 않았으면 SUCCESS를 반환합니다.
//   return BT::NodeStatus::SUCCESS;
// }

// }  // namespace amr_bt_nodes

// // #include "behaviortree_cpp/bt_factory.h"

// // // 플러그인을 등록하기 위한 정식 방법
// // BT_REGISTER_NODES(factory)
// // {
// //   factory.registerNodeType<amr_bt_nodes::IsBlackboardChangedCondition>("IsBlackboardChangedCondition");
// // }

// #include "behaviortree_cpp/bt_factory.h"

// extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
// {
//   factory.registerNodeType<amr_bt_nodes::IsBlackboardChangedCondition>("IsBlackboardChangedCondition");
// }