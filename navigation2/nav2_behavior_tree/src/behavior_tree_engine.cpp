// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Florian Gramss
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

#include "nav2_behavior_tree/behavior_tree_engine.hpp"

#include <memory>
#include <string>
#include <vector>
#include <algorithm> // std::find 를 위해 추가 // ADD chang.gwak

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/utils/shared_library.h"

namespace nav2_behavior_tree
{

BehaviorTreeEngine::BehaviorTreeEngine(
  const std::vector<std::string> & plugin_libraries, rclcpp::Node::SharedPtr node)
{
  BT::SharedLibrary loader;
  for (const auto & p : plugin_libraries) {
    factory_.registerFromPlugin(loader.getOSName(p));
  }

  // clock for throttled debug log
  clock_ = node->get_clock();

  // FIXME: the next two line are needed for back-compatibility with BT.CPP 3.8.x
  // Note that the can be removed, once we migrate from BT.CPP 4.5.x to 4.6+
  BT::ReactiveSequence::EnableException(false);
  BT::ReactiveFallback::EnableException(false);
}

BtStatus
BehaviorTreeEngine::run(
  BT::Tree * tree,
  std::function<void()> onLoop,
  std::function<bool()> cancelRequested,
  std::chrono::milliseconds loopTimeout)
{
  rclcpp::WallRate loopRate(loopTimeout);
  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  // Loop until something happens with ROS or the node completes
  try {
    while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
      if (cancelRequested()) {
        tree->haltTree();
        return BtStatus::CANCELED;
      }

      result = tree->tickOnce();

      onLoop();

      if (!loopRate.sleep()) {
        RCLCPP_DEBUG_THROTTLE(
          rclcpp::get_logger("BehaviorTreeEngine"),
          *clock_, 1000,
          "Behavior Tree tick rate %0.2f was exceeded!",
          1.0 / (loopRate.period().count() * 1.0e-9));
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("BehaviorTreeEngine"),
      "Behavior tree threw exception: %s. Exiting with failure.", ex.what());
    return BtStatus::FAILED;
  }

  return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}

BT::Tree
BehaviorTreeEngine::createTreeFromText(
  const std::string & xml_string,
  BT::Blackboard::Ptr blackboard)
{
  return factory_.createTreeFromText(xml_string, blackboard);
}

BT::Tree
BehaviorTreeEngine::createTreeFromFile(
  const std::string & file_path,
  BT::Blackboard::Ptr blackboard)
{
  return factory_.createTreeFromFile(file_path, blackboard);
}

// In order to re-run a Behavior Tree, we must be able to reset all nodes to the initial state
void
BehaviorTreeEngine::haltAllActions(BT::Tree & tree)
{
  // this halt signal should propagate through the entire tree.
  tree.haltTree();
}

// ADD chang.gwak
// Function to attach a standard cout logger to the BT
void
BehaviorTreeEngine::addStdCoutLogger(const BT::Tree & tree)
{
  // 이전 로거가 있다면 초기화하고 새로 생성합니다.
  cout_logger_.reset();
  cout_logger_ = std::make_unique<BT::StdCoutLogger>(tree);
}

void
BehaviorTreeEngine::addFileLogger2(const BT::Tree & tree, const std::string & filepath)
{
  file_logger2_ = std::make_unique<BT::FileLogger2>(tree, filepath);
}


// void
// BehaviorTreeEngine::addStdCoutLogger(
//   const BT::Tree & tree,
//   const std::vector<std::string> & nodes_to_log)
// {
//   // 이전 StdCoutLogger 생성 코드를 아래 내용으로 교체합니다.
//   cout_logger_.reset(); // 이전 로거 초기화

//   // StatusChangeLogger를 사용하여 직접 콜백을 만듭니다.
//   auto custom_callback =
//     [this, nodes_to_log](BT::Duration timestamp, const BT::TreeNode &node,
//                          BT::NodeStatus prev_status, BT::NodeStatus status)
//     {
//       // 모니터링 목록이 비어있거나, 현재 노드가 목록에 있을 때만 로그 출력
//       if (nodes_to_log.empty() ||
//         std::find(nodes_to_log.begin(), nodes_to_log.end(), node.registrationName()) != nodes_to_log.end())
//       {
//         // StdCoutLogger의 출력과 유사한 형식으로 직접 출력
//         double since_epoch = std::chrono::duration<double>(timestamp).count();
//         RCLCPP_INFO(
//           rclcpp::get_logger("FilteredBtLogger"), "[%.3f]: %s: %s -> %s",
//           since_epoch, node.name().c_str(),
//           BT::toStr(prev_status, true).c_str(), BT::toStr(status, true).c_str());
//       }
//     };

//   cout_logger_ = std::make_unique<BT::StatusChangeLogger>(tree.rootNode(), custom_callback);
// }


/////////////// done ADD chang.gwak


}  // namespace nav2_behavior_tree
