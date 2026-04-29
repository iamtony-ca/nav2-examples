#include "amr_bt_nodes/reactive_round_robin_control.hpp"

namespace amr_bt_nodes
{

ReactiveRoundRobinControl::ReactiveRoundRobinControl(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ControlNode(name, config)
{
}

void ReactiveRoundRobinControl::halt()
{
  // halt() is called when the tree is reset (e.g., new goal received).
  // This ensures that for every new action, we start from the first child.
  ControlNode::halt();
  current_child_idx_ = 0;
}

BT::NodeStatus ReactiveRoundRobinControl::tick()
{
  const size_t num_children = children_nodes_.size();

  if (num_children == 0) {
    return BT::NodeStatus::SUCCESS;
  }

  setStatus(BT::NodeStatus::RUNNING);

  // This while loop allows the node to try multiple children within a single tick if they fail.
  while (current_child_idx_ < num_children) {
    TreeNode * child_node = children_nodes_[current_child_idx_];
    const BT::NodeStatus child_status = child_node->executeTick();

    switch (child_status) {
      case BT::NodeStatus::RUNNING:
        // If a child is still running, the entire node is still running.
        return BT::NodeStatus::RUNNING;

      case BT::NodeStatus::SUCCESS:
        // A child succeeded. Return SUCCESS for this tick.
        // Increment the index to prepare for the *next* time this node is ticked.
        current_child_idx_++;
        // If we have completed the cycle, wrap around.
        if (current_child_idx_ == num_children) {
          current_child_idx_ = 0;
        }
        return BT::NodeStatus::SUCCESS;

      case BT::NodeStatus::FAILURE:
        {
          // A child failed. Check if it was the last one in the sequence.
          if (current_child_idx_ == num_children - 1) {
            // The last child failed. This entire recovery cycle fails.
            // Reset index for the next full attempt and return FAILURE.
            current_child_idx_ = 0;
            return BT::NodeStatus::FAILURE;
          }
          // It was not the last child. Increment index and continue the while loop
          // to try the next child immediately in this same tick.
          current_child_idx_++;
        }
        break;

      default:  // SKIPPED
        // Treat SKIPPED the same as FAILURE.
        if (current_child_idx_ == num_children - 1) {
            current_child_idx_ = 0;
            return BT::NodeStatus::FAILURE;
        }
        current_child_idx_++;
        break;
    }  // end switch
  }  // end while

  // This part of the code should not be reachable, but serves as a safeguard.
  halt();
  return BT::NodeStatus::FAILURE;
}

}  // namespace amr_bt_nodes


#include "behaviortree_cpp/bt_factory.h"

// 이 함수는 BehaviorTreeFactory의 클래스 로더에 의해 호출됩니다.
extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  // 다른 노드 등록...
  factory.registerNodeType<amr_bt_nodes::ReactiveRoundRobinControl>("ReactiveRoundRobinControl");
}