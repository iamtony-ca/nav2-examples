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

#include "amr_bt_nodes/run_n_times_decorator.hpp"

namespace amr_bt_nodes
{

RunNTimesDecorator::RunNTimesDecorator(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::DecoratorNode(name, config)
{
}

BT::PortsList RunNTimesDecorator::providedPorts()
{
  return {
    BT::InputPort<int>("num_ticks", 1, "Number of ticks to execute the child")
  };
}

void RunNTimesDecorator::halt()
{
  // Reset the state for the next goal/execution cycle.
  tick_count_ = 0;
  completed_for_this_goal_ = false;
  DecoratorNode::halt();
}

BT::NodeStatus RunNTimesDecorator::tick()
{
  // If the required number of ticks has already been completed for this goal,
  // do not execute the child and return SUCCESS immediately.
  if (completed_for_this_goal_) {
    return BT::NodeStatus::SUCCESS;
  }

  // On the first tick of a new execution cycle, read the 'num_ticks' parameter.
  if (tick_count_ == 0) {
    if (!getInput<int>("num_ticks", num_ticks_to_run_) || num_ticks_to_run_ < 1) {
      throw BT::RuntimeError(
              "[RunNTimesDecorator] Missing or invalid required input port [num_ticks]");
    }
  }

  tick_count_++;

  // Tick the child and propagate its status.
  setStatus(BT::NodeStatus::RUNNING);
  const auto child_status = child_node_->executeTick();

  // After ticking, check if we have now completed the required number of ticks.
  if (tick_count_ >= num_ticks_to_run_) {
    completed_for_this_goal_ = true;
  }

  return child_status;
}

}  // namespace amr_bt_nodes

// BehaviorTree.CPP 플러그인 등록을 위한 외부 함수
#include "behaviortree_cpp/bt_factory.h"
extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<amr_bt_nodes::RunNTimesDecorator>("RunNTimesDecorator");
}