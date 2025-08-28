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

#ifndef AMR_BT_NODES__GET_NEXT_FEW_GOALS_ACTION_HPP_
#define AMR_BT_NODES__GET_NEXT_FEW_GOALS_ACTION_HPP_

#include <string>
#include <vector>
#include <algorithm>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp/behavior_tree.h"

namespace amr_bt_nodes
{

/**
 * @brief A BT::SyncActionNode that gets a subset of goals from a list of goals.
 */
// [Name Change] 클래스 이름을 GetNextFewGoalsAction으로 변경
class GetNextFewGoalsAction : public BT::SyncActionNode
{
public:
  /**
   * @brief A constructor for amr_bt_nodes::GetNextFewGoalsAction
   * @param name The name of the node
   * @param conf The node configuration
   */
  // [Name Change] 생성자 이름을 GetNextFewGoalsAction으로 변경
  GetNextFewGoalsAction(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief The main execution function.
   * @return BT::NodeStatus SUCCESS if goals are extracted, FAILURE otherwise.
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
        "input_goals", "Full list of goals to process"),
      BT::InputPort<int>("num_goals", "Number of goals to extract from the input list"),
      BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
        "output_goals", "Subset of goals extracted")
    };
  }
};

}  // namespace amr_bt_nodes

#endif  // AMR_BT_NODES__GET_NEXT_FEW_GOALS_ACTION_HPP_