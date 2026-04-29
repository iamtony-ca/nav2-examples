// Copyright (c) 2025 Open Navigation LLC
// Copyright (c) 2025 Your Name/Company
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

// [Name Change] 헤더 파일 경로를 get_next_few_goals_action.hpp로 변경
#include "amr_bt_nodes/get_next_few_goals_action.hpp"

namespace amr_bt_nodes
{

// [Name Change] 클래스 및 생성자 이름을 GetNextFewGoalsAction으로 변경
GetNextFewGoalsAction::GetNextFewGoalsAction(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
}

// [Name Change] 메서드 소속 클래스 이름을 GetNextFewGoalsAction으로 변경
BT::NodeStatus GetNextFewGoalsAction::tick()
{
  auto input_goals_wrapper = getInput<std::vector<geometry_msgs::msg::PoseStamped>>("input_goals");
  if (!input_goals_wrapper) {
    RCLCPP_ERROR(
      config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
      "Input goals not provided to GetNextFewGoalsAction node. Failing.");
    return BT::NodeStatus::FAILURE;
  }
  const auto & input_goals = input_goals_wrapper.value();

  int num_goals_to_extract;
  getInput("num_goals", num_goals_to_extract);

  if (input_goals.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  size_t num_to_copy = std::min(
    static_cast<size_t>(num_goals_to_extract),
    input_goals.size());

  auto start_it = input_goals.begin();
  auto end_it = input_goals.begin() + num_to_copy;

  std::vector<geometry_msgs::msg::PoseStamped> output_goals(start_it, end_it);
  
  setOutput("output_goals", output_goals);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace amr_bt_nodes

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  // [Name Change] 클래스 및 등록 이름을 GetNextFewGoalsAction으로 변경
  factory.registerNodeType<amr_bt_nodes::GetNextFewGoalsAction>("GetNextFewGoalsAction");
}