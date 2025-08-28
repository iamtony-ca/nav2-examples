// [수정] 인클루드 가드 및 파일 경로 컨벤션 변경
#ifndef AMR_BT_NODES__REMOVE_FIRST_GOAL_ACTION_HPP_
#define AMR_BT_NODES__REMOVE_FIRST_GOAL_ACTION_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

// [수정] 네임스페이스 변경
namespace amr_bt_nodes
{

// [수정] 클래스 이름 변경
class RemoveFirstGoalAction : public BT::SyncActionNode
{
public:
  // [수정] 생성자 이름 변경
  RemoveFirstGoalAction(
    const std::string & name,
    const BT::NodeConfiguration & config);

  // BT 노드에 필요한 정적 Port 리스트 정의
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
        "input_goals", "Original list of goals to process"),
      // [개선] 출력 포트 이름을 더 명확하게 변경 (output_goals -> remaining_goals)
      BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
        "remaining_goals", "List of goals with the first element removed")
    };
  }

  // 노드가 Tick될 때 실행되는 메인 로직
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
};

}  // namespace amr_bt_nodes

#endif  // AMR_BT_NODES__REMOVE_FIRST_GOAL_ACTION_HPP_