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

#ifndef AMR_BT_NODES__PLUGINS__ACTION__SET_BLACKBOARD_BOOL_ACTION_HPP_
#define AMR_BT_NODES__PLUGINS__ACTION__SET_BLACKBOARD_BOOL_ACTION_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"

namespace amr_bt_nodes
{

/**
 * @brief The SetBlackboardBoolAction is an action that sets a boolean value
 * on the blackboard. It is more type-safe than the generic SetBlackboard
 * for boolean types.
 *
 * It can take a value directly or from another blackboard entry.
 *
 * Example usage:
 *
 * <SetBlackboardBool output_key="is_battery_low" value="true" />
 *
 * or copying from another port:
 *
 * <SetBlackboardBool output_key="task_completed" value="{is_done}" />
 *
 */
class SetBlackboardBoolAction : public BT::SyncActionNode
{
public:
  /**
   * @brief A constructor for amr_bt_nodes::SetBlackboardBoolAction
   * @param name The name of the node
   * @param config The configuration of the node
   */
  SetBlackboardBoolAction(const std::string & name, const BT::NodeConfig & config);

  /**
   * @brief Provides the ports of the node
   * @return PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts();

private:
  /**
   * @brief The main execution routine.
   * @return NodeStatus SUCCESS or FAILURE
   */
  BT::NodeStatus tick() override;
};

}  // namespace amr_bt_nodes

#endif  // AMR_BT_NODES__PLUGINS__ACTION__SET_BLACKBOARD_BOOL_ACTION_HPP_