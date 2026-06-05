#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, UInt8
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class StuckManagerNode(Node):
    def __init__(self):
        super().__init__('stuck_manager_node')

        # 파라미터 선언 (타입 명시)
        self.declare_parameter(
            'timeout_sec', 
            200.0,
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