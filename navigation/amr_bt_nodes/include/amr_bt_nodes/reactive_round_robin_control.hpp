#ifndef AMR_BT_NODES__REACTIVE_ROUND_ROBIN_CONTROL_HPP_
#define AMR_BT_NODES__REACTIVE_ROUND_ROBIN_CONTROL_HPP_

#include <string>
#include "behaviortree_cpp/control_node.h"
// #include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"


namespace amr_bt_nodes
{

/**
 * @class ReactiveRoundRobinControl
 * @brief A stateful, cycling control node designed for recovery sequences.
 *
 * This node ticks its children sequentially based on its internal state.
 * Its behavior is a hybrid between a Fallback and a stateful Sequence.
 *
 * - Rule 1 & 6 (Stateful & Cycling): It remembers the last executed child index
 * and will cycle back to the first child after the sequence is complete. It is
 * reset to the first child when halt() is called (e.g., at the start of a new action).
 *
 * - Rule 2 & 4 (Execution Policy): It attempts to find a successful child within a
 * single tick. If a child fails, it immediately tries the next child in the
 * same tick. It only stops execution in a tick if a child returns SUCCESS or RUNNING,
 * or if the final child fails.
 *
 * - Rule 3 (Success Condition): If any child returns SUCCESS, this node
 * immediately returns SUCCESS.
 *
 * - Rule 5 (Failure Condition): This node returns FAILURE only if the VERY LAST
 * child in the sequence is ticked and it returns FAILURE.
 */
class ReactiveRoundRobinControl : public BT::ControlNode
{
public:
  explicit ReactiveRoundRobinControl(const std::string & name, const BT::NodeConfiguration & config);

  ReactiveRoundRobinControl() = delete;

  /**
   * @brief The main execution logic for the node.
   * @return The status of the node after execution.
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Called when the node is halted. Resets the internal state.
   */
  void halt() override;

  /**
   * @brief Defines the ports for this node.
   * @return A list of ports. This node has no ports.
   */
  static BT::PortsList providedPorts() {return {};}

private:
  unsigned int current_child_idx_{0};
};

}  // namespace amr_bt_nodes

#endif  // AMR_BT_NODES__REACTIVE_ROUND_ROBIN_CONTROL_HPP_