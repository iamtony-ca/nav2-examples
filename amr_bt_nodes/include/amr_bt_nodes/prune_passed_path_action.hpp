#ifndef AMR_BT_NODES__PRUNE_PASSED_PATH_NODE_HPP_
#define AMR_BT_NODES__PRUNE_PASSED_PATH_NODE_HPP_

#include <string>
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "behaviortree_cpp/action_node.h"

namespace amr_bt_nodes
{

class PrunePassedPathAction : public BT::SyncActionNode
{
public:
  PrunePassedPathAction(
    const std::string & name,
    const BT::NodeConfiguration & config);

  PrunePassedPathAction() = delete;

  // 이 노드가 사용하는 포트 정의
  static BT::PortsList providedPorts();

  // 메인 실행 로직
  BT::NodeStatus tick() override;
};

}  // namespace amr_bt_nodes

#endif  // AMR_BT_NODES__PRUNE_PASSED_PATH_NODE_HPP_