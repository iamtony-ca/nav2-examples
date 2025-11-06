#!/usr/bin/env python3

"""
robot_status_manager.py

ROS 2 (Jazzy) 노드로, Nav2 Behavior Tree의 로그를 구독하여
로봇의 상위 레벨 상태를 추론하고 0.5초 주기로 'robot_status' 토픽에 게시합니다.

bt.xml의 특정 노드 이름에 의존하여 상태를 매핑합니다.
- /behavior_tree_log (nav2_msgs/msg/BehaviorTreeLog) -> 구독
- /robot_status (std_msgs/msg/String) -> 발행
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from nav2_msgs.msg import BehaviorTreeLog
from std_msgs.msg import String

# 1. 요청하신 로봇 상태 목록을 클래스로 정의
class RobotStatus:
    IDLE = "IDLE"
    RECEIVED_GOAL = "RECEIVED_GOAL"
    PLANNING = "PLANNING"
    DRIVING = "DRIVING"
    PAUSED = "PAUSED"
    RECOVERY_FAILURE = "RECOVERY_FAILURE"
    RECOVERY_RUNNING = "RECOVERY_RUNNING"
    RECOVERY_SUCCESS = "RECOVERY_SUCCESS"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELED = "CANCELED"
    
    # IDLE 상태로 자동 복귀하면 안 되는 최종 상태 집합
    TERMINAL_STATES = {SUCCEEDED, FAILED, CANCELED}


class RobotStatusManager(Node):
    """
    BT 로그를 파싱하여 로봇의 현재 상태를 게시하는 노드입니다.
    """
    def __init__(self):
        super().__init__('robot_status_manager')

        # --- 파라미터 ---
        # BT 로그가 수신되지 않을 때 IDLE로 복귀하기까지의 타임아웃 (초)
        self.declare_parameter('idle_timeout_sec', 2.0)
        self.idle_timeout_sec = self.get_parameter('idle_timeout_sec').get_parameter_value().double_value
        
        # 요청하신 0.5초 발행 주기
        self.publish_period = 0.5

        # --- 상태 변수 ---
        self.current_status = RobotStatus.IDLE
        self.last_log_time = self.get_clock().now() - Duration(seconds=self.idle_timeout_sec * 2.0)

        # --- ROS 2 통신 ---
        
        # BT 로그 구독
        self.subscription = self.create_subscription(
            BehaviorTreeLog,
            '/behavior_tree_log',
            self.log_callback,
            10)
        
        # 로봇 상태 발행
        self.status_publisher = self.create_publisher(
            String,
            'robot_status',
            10)
        
        # 0.5초 주기 타이머
        self.status_timer = self.create_timer(
            self.publish_period,
            self.publish_status_callback)

        self.get_logger().info('Robot Status Manager가 시작되었습니다.')

    def log_callback(self, msg: BehaviorTreeLog):
        """
        /behavior_tree_log 토픽에서 메시지를 수신할 때마다 호출됩니다.
        """
        # 마지막 로그 수신 시간 업데이트
        self.last_log_time = Time.from_msg(msg.timestamp)
        
        running_nodes = set()
        terminal_nodes = {}  # key: node_name, value: status ("SUCCESS", "FAILURE", "IDLE")

        # 로그 메시지 내의 모든 이벤트를 파싱하여 최신 노드 상태를 구축합니다.
        for event in msg.event_log:
            if event.current_status == "RUNNING":
                running_nodes.add(event.node_name)
                # 만약 터미널 상태였다가 다시 실행되면 (예: 루프) 터미널 맵에서 제거
                if event.node_name in terminal_nodes:
                    del terminal_nodes[event.node_name]
            elif event.current_status in ["SUCCESS", "FAILURE", "IDLE"]:
                terminal_nodes[event.node_name] = event.current_status
                # 만약 실행 중이었다가 종료되면 실행 셋에서 제거
                if event.node_name in running_nodes:
                    running_nodes.remove(event.node_name)

        # --- 상태 추론 로직 (우선순위 기반) ---
        
        new_status = self.current_status

        # 1. 전체 트리의 최종 상태 확인 (가장 높은 우선순위)
        # 'NavigateRecovery'는 메인 트리의 루트 복구 노드입니다.
        if "NavigateRecovery" in terminal_nodes:
            status = terminal_nodes["NavigateRecovery"]
            if status == "SUCCESS":
                new_status = RobotStatus.SUCCEEDED
            elif status == "FAILURE":
                new_status = RobotStatus.FAILED
            elif status == "IDLE" and self.current_status not in RobotStatus.TERMINAL_STATES:
                # 'IDLE' 상태는 BT가 외부(Nav2)에 의해 중지(Halt)되었음을 의미합니다.
                # 이는 일반적으로 목표 취소(Cancel)시 발생합니다.
                new_status = RobotStatus.CANCELED
        
        # 2. 트리가 최종 상태가 아니라면, 활성 상태를 확인합니다. (우선순위 순서대로)
        if new_status not in RobotStatus.TERMINAL_STATES:
            if "CancelControl" in running_nodes:
                # 'CancelControl' 노드는 Pause 로직의 핵심입니다.
                new_status = RobotStatus.PAUSED
            
            elif "IntelligentRecoverySubtree" in running_nodes:
                # 복구 서브트리가 실행 중입니다.
                new_status = RobotStatus.RECOVERY_RUNNING
            
            elif "IntelligentRecoverySubtree" in terminal_nodes:
                # 복구 서브트리가 방금 완료되었습니다. (매우 짧은 순간)
                if terminal_nodes["IntelligentRecoverySubtree"] == "SUCCESS":
                    new_status = RobotStatus.RECOVERY_SUCCESS
                elif terminal_nodes["IntelligentRecoverySubtree"] == "FAILURE":
                    new_status = RobotStatus.RECOVERY_FAILURE
            
            elif "FollowPath_Main" in running_nodes:
                # 메인 경로 추종 노드가 실행 중입니다.
                new_status = RobotStatus.DRIVING
            
            elif "ComputePathThroughPoses_Main" in running_nodes:
                # 메인 경로 계획 노드가 실행 중입니다.
                new_status = RobotStatus.PLANNING
            
            elif "Initilizing..." in running_nodes:
                # BT가 시작되고 초기화 중입니다.
                new_status = RobotStatus.RECEIVED_GOAL
        
        # 상태가 변경된 경우에만 로그를 출력합니다.
        if new_status != self.current_status:
            self.get_logger().info(f'로봇 상태 변경: {self.current_status} -> {new_status}')
            self.current_status = new_status

    def publish_status_callback(self):
        """
        0.5초마다 호출되어 현재 상태를 게시합니다.
        """
        now = self.get_clock().now()
        elapsed = now - self.last_log_time
        
        # BT 로그 수신이 일정 시간(idle_timeout_sec) 동안 끊겼는지 확인합니다.
        if elapsed.nanoseconds / 1e9 > self.idle_timeout_sec:
            # 단, SUCCEEDED, FAILED, CANCELED 같은 최종 상태가 아닐 경우에만 IDLE로 변경합니다.
            if self.current_status not in RobotStatus.TERMINAL_STATES:
                if self.current_status != RobotStatus.IDLE:
                    self.get_logger().warn(
                        f'BT 로그 수신 없음 ({self.idle_timeout_sec}초 초과). 상태를 IDLE로 설정합니다.')
                    self.current_status = RobotStatus.IDLE
        
        # 현재 상태를 String 메시지에 담아 게시합니다.
        status_msg = String()
        status_msg.data = self.current_status
        self.status_publisher.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 노드 및 rclpy 종료
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
