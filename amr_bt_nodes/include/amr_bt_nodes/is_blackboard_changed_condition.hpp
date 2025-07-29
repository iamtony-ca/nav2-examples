#ifndef IS_BLACKBOARD_CHANGED_CONDITION_HPP_
#define IS_BLACKBOARD_CHANGED_CONDITION_HPP_

#include <string>
#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp" // For logging

namespace amr_bt_nodes
{

/**
 * @brief A BT::ConditionNode that checks if a string value on the blackboard has changed
 * since the last tick.
 *
 * This node will return SUCCESS if the value is unchanged or on the first tick.
 * It will return FAILURE if the value has changed, which can be used to trigger
 * a different branch in the tree (e.g., replanning).
 */
class IsBlackboardChangedCondition : public BT::ConditionNode
{
public:
  IsBlackboardChangedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsBlackboardChangedCondition() = delete;

  /**
   * @brief The main override required by a BT::ConditionNode.
   * @return BT::NodeStatus SUCCESS if the blackboard value is unchanged, FAILURE otherwise.
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    // *** 수정된 부분: std::any 대신 std::string을 명시적으로 사용합니다. ***
    return {
      BT::InputPort<std::string>("blackboard_entry", "The string blackboard entry to monitor for changes.")
    };
  }

private:
  // *** 수정된 부분: std::any 대신 std::string을 사용합니다. ***
  std::string previous_value_;
  // Flag to track the first successful tick
  bool first_tick_done_ = false;
  // Logger
  rclcpp::Node::SharedPtr node_;
  // rclcpp::Logger logger_;
};

}  // namespace amr_bt_nodes

#endif  // IS_BLACKBOARD_CHANGED_CONDITION_HPP_













// #ifndef IS_BLACKBOARD_CHANGED_CONDITION_HPP_
// #define IS_BLACKBOARD_CHANGED_CONDITION_HPP_

// #include <string>
// #include <any>
// #include "behaviortree_cpp/condition_node.h"
// #include "rclcpp/rclcpp.hpp" // For logging

// namespace amr_bt_nodes
// {

// /**
//  * @brief A BT::ConditionNode that checks if a value on the blackboard has changed
//  * since the last tick.
//  *
//  * This node will return SUCCESS if the value is unchanged or on the first tick.
//  * It will return FAILURE if the value has changed, which can be used to trigger
//  * a different branch in the tree (e.g., replanning).
//  */
// class IsBlackboardChangedCondition : public BT::ConditionNode
// {
// public:
//   IsBlackboardChangedCondition(
//     const std::string & condition_name,
//     const BT::NodeConfiguration & conf);

//   IsBlackboardChangedCondition() = delete;

//   /**
//    * @brief The main override required by a BT::ConditionNode.
//    * @return BT::NodeStatus SUCCESS if the blackboard value is unchanged, FAILURE otherwise.
//    */
//   BT::NodeStatus tick() override;

//   /**
//    * @brief Creates list of BT ports
//    * @return BT::PortsList Containing basic ports along with node-specific ports
//    */
//   static BT::PortsList providedPorts()
//   {
//     // This node needs to read a value from a blackboard entry.
//     // The key of the entry is passed at construction time.
//     // Since we don't know the type, we use a "generic" Any port.
//     return {
//       BT::InputPort<std::any>("blackboard_entry", "The blackboard entry to monitor for changes.")
//     };
//   }

// private:
//   // previous_value_ stores the value from the last tick to compare against the current one.
//   // std::any allows us to store any type.
//   std::any previous_value_;
//   // Flag to track the first successful tick
//   bool first_tick_done_ = false;
//   // Logger
//   rclcpp::Node::SharedPtr node_;
//   // rclcpp::Logger logger_;
// };

// }  // namespace amr_bt_nodes

// #endif  // IS_BLACKBOARD_CHANGED_CONDITION_HPP_
