#ifndef AMR_BT_NODES__LOG_TEXT_ACTION_HPP_
#define AMR_BT_NODES__LOG_TEXT_ACTION_HPP_

#include <string>
#include "behaviortree_cpp/action_node.h" // BT::SyncActionNode를 사용하기 위해 필요
#include "rclcpp/rclcpp.hpp"             // RCLCPP_INFO 등을 사용하기 위해 필요

namespace amr_bt_nodes
{

/**
 * @brief LogTextAction은 Behavior Tree에서 주어진 텍스트 메시지를 터미널에 출력하는 Action 노드입니다.
 * 주로 디버깅 및 정보 로깅 목적으로 사용됩니다.
 */
class LogTextAction : public BT::SyncActionNode
{
public:
  /**
   * @brief LogTextAction 클래스의 생성자입니다.
   * @param name 노드의 이름
   * @param config 노드 설정 (BT::NodeConfiguration)
   */
  LogTextAction(
    const std::string & name,
    const BT::NodeConfiguration & config);

  // 기본 생성자 삭제 (NodeConfiguration이 필수이므로)
  LogTextAction() = delete;

  /**
   * @brief 이 노드가 사용하는 포트를 정의합니다.
   * "message"라는 이름의 Input Port를 통해 로깅할 텍스트를 받습니다.
   * @return 포트 리스트
   */
  static BT::PortsList providedPorts();

  /**
   * @brief 노드의 메인 실행 로직입니다.
   * 입력 포트에서 메시지를 읽어 터미널에 출력합니다.
   * @return 노드 실행 결과 (SUCCESS 또는 FAILURE)
   */
  BT::NodeStatus tick() override;
};

}  // namespace amr_bt_nodes

#endif  // AMR_BT_NODES__LOG_TEXT_ACTION_HPP_