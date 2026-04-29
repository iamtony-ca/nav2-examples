#ifndef AMR_BT_NODES__LOG_TEXT_ACTION_HPP_
#define AMR_BT_NODES__LOG_TEXT_ACTION_HPP_

#include <string>
#include "behaviortree_cpp/action_node.h" // BT::SyncActionNode를 사용하기 위해 필요
#include "rclcpp/rclcpp.hpp"             // RCLCPP_INFO 등을 사용하기 위해 필요

namespace amr_bt_nodes
{

/**
 * @brief LogTextAction은 Behavior Tree에서 주어진 텍스트 메시지를 터미널에 출력하는 Action 노드입니다.
 * 'interval_s' 포트를 통해 설정된 시간 간격으로만 메시지를 로깅합니다.
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
   * "message" Input Port: 로깅할 텍스트 메시지
   * "interval_s" Input Port: 로깅 주기를 나타내는 float 값 (초 단위)
   * @return 포트 리스트
   */
  static BT::PortsList providedPorts();

  /**
   * @brief 노드의 메인 실행 로직입니다.
   * 입력 포트에서 메시지와 간격을 읽어와, 설정된 간격으로만 메시지를 로깅합니다.
   * @return 노드 실행 결과 (SUCCESS 또는 FAILURE)
   */
  BT::NodeStatus tick() override;

private:
  // 마지막으로 로깅된 시간 기록 (MonoClock 사용)
  rclcpp::Time last_log_time_;
  // 노드 생성 시 ROS 2 노드 핸들을 받아서 로거를 초기화하는 용도
  rclcpp::Node::SharedPtr node_;
};

}  // namespace amr_bt_nodes

#endif  // AMR_BT_NODES__LOG_TEXT_ACTION_HPP_