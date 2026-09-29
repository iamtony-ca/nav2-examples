#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, UInt8, Bool
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class StuckManagerNode(Node):
    def __init__(self):
        super().__init__('stuck_manager_node')

        # 파라미터 선언 (타입 명시)
        self.declare_parameter(
            'timeout_sec', 
            300.0,
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, description='Timeout limit in seconds')
        )
        self.declare_parameter(
            'min_distance_m', 
            0.3,
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, description='Minimum distance to move in meters')
        )
        self.declare_parameter(
            'stop_command_value', 
            3,
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, description='Value to publish on timeout')
        )

        self.timeout_sec = self.get_parameter('timeout_sec').value
        self.min_distance_m = self.get_parameter('min_distance_m').value
        self.stop_command_value = self.get_parameter('stop_command_value').value

        # 상태 관리 변수 및 Thread Lock
        self.state_lock = threading.Lock()
        self.is_tracking = False
        self.reference_pose = None
        self.reference_time = None
        # [09-28 관제 우선] 관제 pause(/nav_pause_flag) 동안은 무진전 시계를 멈춘다 (사용자 결정).
        # fleet 자체 pause(controller_pause_flag)는 이 플래그를 올리지 않으므로 지금처럼 센다.
        self.nav_paused = False
        self.nav_pause_since = None

        # Callback Groups (멀티스레드 병렬 처리용)
        self.status_cb_group = MutuallyExclusiveCallbackGroup()
        self.pose_cb_group = MutuallyExclusiveCallbackGroup()

        # Subscriber & Publisher 설정
        self.sub_status = self.create_subscription(
            String, 
            '/robot_status', 
            self.status_callback, 
            10,
            callback_group=self.status_cb_group
        )
        self.sub_pose = self.create_subscription(
            PoseStamped, 
            '/tracked_pose', 
            self.pose_callback, 
            10,
            callback_group=self.pose_cb_group
        )
        self.pub_stop = self.create_publisher(UInt8, 'stop_command', 10)
        # navigation_manager 가 TRANSIENT_LOCAL·depth 1 로 발행한다 — 늦게 붙어도 현재 pause 상태를 받는다.
        qos_nav_pause = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.sub_nav_pause = self.create_subscription(
            Bool,
            '/nav_pause_flag',
            self.nav_pause_callback,
            qos_nav_pause,
            callback_group=self.status_cb_group
        )

        self.get_logger().info(
            f'Stuck Detector Initialized. '
            f'Timeout: {self.timeout_sec}s, Min Distance: {self.min_distance_m}m'
        )

    def status_callback(self, msg: String):
        """로봇의 상태를 모니터링하여 트래킹 시작/종료를 결정합니다."""
        # [수정] 리스트가 대문자이므로, 데이터도 대문자로 변환하여 비교 (Critical Bug Fix)
        status = msg.data.upper()
        
        with self.state_lock:
            if status in ['RECEIVED_GOAL', 'PLANNING', 'DRIVING', 'PAUSED', 'RECOVERY_FAILURE', 'RECOVERY_RUNNING', 'RECOVERY_SUCCESS']:
                if not self.is_tracking:
                    self.get_logger().info('Goal received: 위치 트래킹을 시작합니다.')
                    self.is_tracking = True
                    self.reference_pose = None
                    self.reference_time = None
                    
            elif status in ['IDLE', 'SUCCEEDED', 'FAILED', 'CANCELED']:
                if self.is_tracking:
                    self.get_logger().info(f'Status [{status}]: 위치 트래킹을 중지합니다.')
                    self.is_tracking = False
                    self.reference_pose = None
                    self.reference_time = None

    PAUSE_LOG_MIN_SEC = 5.0          # [09-28 미결 4 초안] 이보다 짧은 관제 pause 의 시계 정지·해제는 DEBUG 로만 남긴다

    def nav_pause_callback(self, msg: Bool):
        """[09-28] 관제 pause 동안 시계를 멈추고, 해제되면 pause 길이만큼 기준 시각을 뒤로 민다."""
        paused = bool(msg.data)
        with self.state_lock:
            if paused == self.nav_paused:
                return
            now = self.get_clock().now()
            self.nav_paused = paused
            if paused:
                self.nav_pause_since = now
                # [09-28 미결 4 초안] 교차로 흐름제어의 짧은 pause(0.1~1 s)도 같은 경로라 교차로마다 두 줄씩 남았다.
                # 시작은 DEBUG 로, 해제 줄에서 길이가 PAUSE_LOG_MIN_SEC 이상일 때만 INFO 로 남긴다 (동작은 같다).
                self.get_logger().debug('관제 pause: 무진전 시계를 멈춥니다.')
                return
            since, self.nav_pause_since = self.nav_pause_since, None
            if since is not None and self.reference_time is not None:
                paused_dur = now - since
                self.reference_time = self.reference_time + paused_dur
                sec = paused_dur.nanoseconds / 1e9
                line = f'관제 pause 해제: {sec:.1f}초를 빼고 이어서 셉니다.'
                if sec >= self.PAUSE_LOG_MIN_SEC:
                    self.get_logger().info(line)
                else:
                    self.get_logger().debug(line)

    def pose_callback(self, msg: PoseStamped):
        """실시간 Pose를 받아 이동 거리를 계산하고 Timeout을 판별합니다."""
        
        stop_info = None

        with self.state_lock:
            if not self.is_tracking:
                return

            current_time = self.get_clock().now()
            
            if current_time.nanoseconds == 0:
                self.get_logger().warn('Clock not yet initialized, skipping pose update.', throttle_duration_sec=2.0)
                return

            current_pose = msg.pose

            # [09-28] 관제 pause 중에는 판정하지 않는다 (시계는 해제 때 이어서 센다)
            if self.nav_paused:
                return

            if self.reference_pose is None or self.reference_time is None:
                self.reference_pose = current_pose
                self.reference_time = current_time
                return

            # 거리 계산
            dx = current_pose.position.x - self.reference_pose.position.x
            dy = current_pose.position.y - self.reference_pose.position.y
            distance = math.hypot(dx, dy)

            # Stuck 판단 로직 (연속 슬라이딩 윈도우)
            if distance >= self.min_distance_m:
                # 목표 거리 이상 이동했으므로 기준점 갱신
                self.reference_pose = current_pose
                self.reference_time = current_time
            else:
                # [수정] 불필요한 elif 제거하고 else로 처리. elapsed_sec 계산.
                elapsed_time_duration = current_time - self.reference_time
                elapsed_sec = elapsed_time_duration.nanoseconds / 1e9
                
                # [수정] 로그 메시지 포매팅 개선 (소수점 자릿수 지정 및 문구 다듬기)
                self.get_logger().warn(
                    f'Stuck Monitoring: {elapsed_sec:.2f} / {self.timeout_sec} 초 경과 '
                    f'(현재 이동 거리: {distance:.2f} / {self.min_distance_m} m)',
                    throttle_duration_sec=2.0
                )

                if elapsed_sec >= self.timeout_sec:
                    stop_info = (elapsed_sec, distance, int(self.stop_command_value))
                    self.is_tracking = False  

        # Lock 해제 후 퍼블리시 및 로그 포매팅 실행
        if stop_info is not None:
            elapsed, dist, cmd = stop_info
            self.get_logger().warn(
                f'TIMEOUT 발생! 로봇이 {elapsed:.2f}초 동안 {dist:.2f}m 반경 내에 갇혀있습니다. '
                f'Stop command(UInt8: {cmd})를 발행합니다.'
            )
            
            stop_msg = UInt8()
            stop_msg.data = cmd
            self.pub_stop.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StuckManagerNode()
    
    # 스레드 4개 명시적 지정 잘 하셨습니다.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()