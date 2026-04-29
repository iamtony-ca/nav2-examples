// Copyright 2025 YourName <youremail@example.com>
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

#pragma once

#include "behaviortree_cpp/decorator_node.h"
#include <rclcpp/rclcpp.hpp>


namespace amr_bt_nodes
{

/**
 * @brief Executes the child node for a specified number of ticks each time a new goal/execution cycle begins.
 * After N ticks, this node will continuously return SUCCESS for the remainder of the
 * current execution cycle without ticking its child.
 * The internal tick count is reset to zero when the node is halted, making it ready
 * for the next execution cycle (e.g., a new navigation goal).
 */
class RunNTimesDecorator : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for amr_bt_nodes::RunNTimesDecorator
   * @param name The name of the node
   * @param config The node configuration
   */
  RunNTimesDecorator(
    const std::string & name,
    const BT::NodeConfiguration & config);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts();

  /**
   * @brief The halt method, called when the node is preempted.
   */
  void halt() override;

private:
  /**
   * @brief The main tick behaviour
   * @return BT::NodeStatus The status of the node
   */
  BT::NodeStatus tick() override;

  unsigned int tick_count_{0};
  bool completed_for_this_goal_{false};
  int num_ticks_to_run_{1};
};

}  // namespace amr_bt_nodes