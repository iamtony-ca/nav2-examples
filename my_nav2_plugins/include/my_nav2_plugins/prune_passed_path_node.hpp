#ifndef MY_NAV2_PLUGINS__PRUNE_PASSED_PATH_NODE_HPP_
#define MY_NAV2_PLUGINS__PRUNE_PASSED_PATH_NODE_HPP_

#include <string>
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "behaviortree_cpp/action_node.h"

namespace my_nav2_plugins
{

class PrunePassedPath : public BT::SyncActionNode
{
public:
  PrunePassedPath(
    const std::string & name,
    const BT::NodeConfiguration & config);

  PrunePassedPath() = delete;

  // 이 노드가 사용하는 포트 정의
  static BT::PortsList providedPorts();

  // 메인 실행 로직
  BT::NodeStatus tick() override;
};

}  // namespace my_nav2_plugins

#endif  // MY_NAV2_PLUGINS__PRUNE_PASSED_PATH_NODE_HPP_