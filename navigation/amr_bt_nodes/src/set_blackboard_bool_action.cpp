// Copyright 2025 Your Name
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "amr_bt_nodes/set_blackboard_bool_action.hpp" // 경로에 맞게 수정 필요

#include <string>

namespace amr_bt_nodes
{

SetBlackboardBoolAction::SetBlackboardBoolAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SetBlackboardBoolAction::providedPorts()
{
  return {
    BT::InputPort<bool>("value", "Boolean value to be written to the blackboard"),
    BT::InputPort<std::string>("output_key", "Name of the blackboard entry to write to")
  };
}

BT::NodeStatus SetBlackboardBoolAction::tick()
{
  std::string output_key;
  if (!getInput("output_key", output_key)) {
    throw BT::RuntimeError("missing required input port [output_key]");
  }

  bool value;
  if (!getInput("value", value)) {
    throw BT::RuntimeError("missing required input port [value]");
  }

  config().blackboard->set<bool>(output_key, value);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace amr_bt_nodes

#include "behaviortree_cpp/bt_factory.h"
// BT_REGISTER_NODES(factory)
// {
//   // XML에서 사용할 태그 이름은 간결하게 "SetBlackboardBool"을 그대로 유지하는 것을 추천합니다.
//   factory.registerNodeType<amr_bt_nodes::SetBlackboardBoolAction>("SetBlackboardBool");
// }

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<amr_bt_nodes::SetBlackboardBoolAction>("SetBlackboardBoolAction");
}
