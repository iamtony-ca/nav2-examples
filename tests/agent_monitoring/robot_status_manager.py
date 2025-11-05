#!/usr/bin/env python3

"""
robot_status_manager.py (v3 - 즉시 발행)

ROS 2 (Jazzy) 노드로, BT 로그와 액션 상태를 구독합니다.
- 상태 변경이 감지되면 *즉시* '/robot_status'를 발행합니다.
- 0.5초마다 현재 상태를 *주기적으로* 발행합니다.

- /behavior_tree_log (nav2_msgs/msg/BehaviorTreeLog) -> 구독
- /navigate_through_poses/_action/status (action_msgs/msg/GoalStatusArray) -> 구독
- /robot_status (std_msgs/msg/String) -> 발행
"""

import rclpy
import threading
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from std_msgs.msg import String
from nav2_msgs.msg import BehaviorTreeLog
from action_msgs.msg import GoalStatusArray, GoalStatus
from unique_identifier_msgs.msg import UUID

# 1. 로봇 상태 목록
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
    BT 로그와 액션 상태를 파싱하여 로봇의 현재 상태를 게시하는 노드입니다.
    """
    def __init__(self):
        super().__init__('robot_status_manager')

        # --- 파라미터 ---
        self.declare_parameter('idle_timeout_sec', 2.0)
        self.idle_timeout_sec = self.get_parameter('idle_timeout_sec').get_parameter_value().double_value
        
        # 0.5초 발행 주기
        self.publish_period = 0.5

        # --- 스레드 안전성 ---
        self.state_lock = threading.RLock()

        # --- 상태 변수 (공유됨) ---
        self.current_status = RobotStatus.IDLE
        self.last_log_time = self.get_clock().now() - Duration(seconds=self.idle_timeout_sec * 2.0)
        self.bt_running_nodes = set()
        self.bt_terminal_nodes = {} # {node_name: "SUCCESS" or "FAILURE"}
        self.active_goal_id = None  # type: UUID | None
        self.latest_terminal_status_code = None  # type: int | None
        self.is_action_executing = False

        # --- ROS 2 통신 ---
        self.bt_log_sub = self.create_subscription(
            BehaviorTreeLog,
            '/behavior_tree_log',
            self.log_callback,
            10)
        
        self.action_status_sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_through_poses/_action/status',
            self.action_status_callback,
            10)
        
        self.status_publisher = self.create_publisher(
            String,
            'robot_status',
            10)
        
        # 0.5초 주기 타이머 (상태 추론 및 발행 담당)
        self.status_timer = self.create_timer(
            self.publish_period,
            self.publish_status_callback)

        self.get_logger().info('Robot Status Manager (즉시 발행 v3)가 시작되었습니다.')

    def log_callback(self, msg: BehaviorTreeLog):
        """
        BT 로그 수신 시, BT 상태를 캐시하고 즉시 상태 재평가를 트리거합니다.
        """
        with self.state_lock:
            self.last_log_time = Time.from_msg(msg.header.stamp)
            
            for event in msg.event_log:
                if event.current_status == "RUNNING":
                    self.bt_running_nodes.add(event.node_name)
                    if event.node_name in self.bt_terminal_nodes:
                        del self.bt_terminal_nodes[event.node_name]
                elif event.current_status in ["SUCCESS", "FAILURE", "IDLE"]:
                    self.bt_terminal_nodes[event.node_name] = event.current_status
                    if event.node_name in self.bt_running_nodes:
                        self.bt_running_nodes.remove(event.node_name)
            
            # [즉시 발행] BT 이벤트로 인한 상태 변경을 즉시 평가하고 게시
            self._evaluate_and_publish_if_changed(reason="BT Log Event")

    def action_status_callback(self, msg: GoalStatusArray):
        """
        액션 상태 수신 시, 액션 상태를 캐시하고 즉시 상태 재평가를 트리거합니다.
        """
        with self.state_lock:
            is_active_now = False
            final_status_code = None
            executing_goal_id = None

            if self.active_goal_id:
                found_tracked_goal = False
                for status in msg.status_list:
                    if tuple(status.goal_info.goal_id.uuid) == tuple(self.active_goal_id.uuid):
                        found_tracked_goal = True
                        if status.status == GoalStatus.STATUS_EXECUTING:
                            is_active_now = True
                        elif status.status == GoalStatus.STATUS_CANCELED:
                            final_status_code = GoalStatus.STATUS_CANCELED
                            break
                        elif status.status in [GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED]:
                            final_status_code = status.status
                
                if not found_tracked_goal and self.is_action_executing:
                     self.get_logger().warn("추적 중인 액션 골이 상태 목록에서 사라졌습니다.")
                
                if final_status_code is not None:
                    is_active_now = False
            else:
                for status in msg.status_list:
                    if status.status == GoalStatus.STATUS_EXECUTING:
                        is_active_now = True
                        executing_goal_id = status.goal_info.goal_id
                        break
            
            # --- 상태 변수 업데이트 ---
            self.is_action_executing = is_active_now
            
            event_triggered = False
            if final_status_code is not None:
                self.active_goal_id = None
                self.is_action_executing = False
                if self.latest_terminal_status_code != final_status_code:
                    self.latest_terminal_status_code = final_status_code
                    event_triggered = True # 새 터미널 이벤트
            elif is_active_now and self.active_goal_id is None and executing_goal_id:
                self.active_goal_id = executing_goal_id
                self.latest_terminal_status_code = None
                event_triggered = True # 새 골 시작 이벤트
            elif not is_active_now and self.active_goal_id:
                self.active_goal_id = None
                self.is_action_executing = False
                event_triggered = True # 알 수 없는 이유로 중지

            # [즉시 발행] 액션 이벤트로 인한 상태 변경을 즉시 평가하고 게시
            if event_triggered:
                self._evaluate_and_publish_if_changed(reason="Action Status Event")

    def _evaluate_and_publish_if_changed(self, reason: str):
        """
        [핵심 로직]
        현재 캐시된 모든 상태를 기반으로 로봇의 상태를 추론합니다.
        상태가 변경된 경우에만 로그를 남기고 *즉시* 게시합니다.
        
        참고: 이 함수는 self.state_lock이 잠긴 상태에서 호출되어야 합니다.
        """
        now = self.get_clock().now()
        new_status = self.current_status

        # --- 1. 로컬 스냅샷 생성 ---
        terminal_code = self.latest_terminal_status_code
        running_nodes = self.bt_running_nodes
        terminal_nodes = self.bt_terminal_nodes
        action_is_running = self.is_action_executing
        elapsed_bt = now - self.last_log_time

        # --- 2. 일회성 트리거 소비 ---
        # 터미널 상태는 한 번만 처리되어야 합니다.
        self.latest_terminal_status_code = None  

        # --- 3. 상태 추론 (우선순위 순) ---
        if terminal_code is not None:
            if terminal_code == GoalStatus.STATUS_SUCCEEDED:
                new_status = RobotStatus.SUCCEEDED
            elif terminal_code == GoalStatus.STATUS_ABORTED:
                new_status = RobotStatus.FAILED
            elif terminal_code == GoalStatus.STATUS_CANCELED:
                new_status = RobotStatus.CANCELED
        
        elif new_status not in RobotStatus.TERMINAL_STATES:
            if "CancelControl" in running_nodes:
                new_status = RobotStatus.PAUSED
            
            elif "IntelligentRecoverySubtree" in running_nodes:
                new_status = RobotStatus.RECOVERY_RUNNING
            
            elif "IntelligentRecoverySubtree" in terminal_nodes:
                status = terminal_nodes.pop("IntelligentRecoverySubtree") # 이벤트 소비
                if status == "SUCCESS":
                    new_status = RobotStatus.RECOVERY_SUCCESS
                elif status == "FAILURE":
                    new_status = RobotStatus.RECOVERY_FAILURE
            
            elif "FollowPath_Main" in running_nodes:
                new_status = RobotStatus.DRIVING
            
            elif "ComputePathThroughPoses_Main" in running_nodes:
                new_status = RobotStatus.PLANNING
            
            elif action_is_running or "Initilizing..." in running_nodes:
                if new_status == RobotStatus.IDLE:
                    new_status = RobotStatus.RECEIVED_GOAL
            
            elif terminal_nodes.get("NavigateRecovery") == "IDLE":
                terminal_nodes.pop("NavigateRecovery") # 이벤트 소비
                new_status = RobotStatus.CANCELED

            elif elapsed_bt.nanoseconds / 1e9 > self.idle_timeout_sec:
                if new_status != RobotStatus.IDLE:
                    self.get_logger().warn(
                        f'BT 로그 수신 없음 ({self.idle_timeout_sec}초 초과). 상태를 IDLE로 설정합니다.')
                    new_status = RobotStatus.IDLE

        # --- 4. 상태 변경 시 즉시 게시 ---
        if self.current_status != new_status:
            self.get_logger().info(f'로봇 상태 변경: {self.current_status} -> {new_status} (Reason: {reason})')
            self.current_status = new_status
            
            status_msg = String()
            status_msg.data = self.current_status
            self.status_publisher.publish(status_msg) # 즉시 발행!

        # --- 5. 최종 상태 도달 시 상태 초기화 ---
        if self.current_status in RobotStatus.TERMINAL_STATES:
            self.active_goal_id = None
            self.is_action_executing = False
            self.bt_running_nodes.clear()
            self.bt_terminal_nodes.clear()


    def publish_status_callback(self):
        """
        0.5초마다 호출됩니다.
        1. 타임아웃과 같은 시간 기반 상태 변경을 확인합니다.
        2. 현재 상태를 주기적으로 게시합니다.
        """
        with self.state_lock:
            # 1. 타임아웃 (IDLE) 등 시간 기반 변경 사항을 평가
            self._evaluate_and_publish_if_changed(reason="Timer Tick")
            
            # 2. (요청사항) 0.5초마다 현재 상태를 *주기적*으로 게시
            #    (위의 즉시 발행과 별개로, 상태가 동일하더라도 계속 발행)
            status_msg = String()
            status_msg.data = self.current_status
            self.status_publisher.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    
    # 참고: rclpy.spin()은 기본적으로 싱글 스레드입니다.
    # 만약 MultiThreadedExecutor를 사용한다면, 콜백에서의 RLock이 필수적입니다.
    # 싱글 스레드에서도 이 코드는 안전하게 동작합니다.
    node = RobotStatusManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()