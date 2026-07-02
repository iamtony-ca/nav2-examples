#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""move_command 연속 발행 테스터.

트리거 토픽(/test_trigger, std_msgs/Empty)을 받으면
NavigationCommand 를 N초 간격으로 M회 연속 발행한다.

# 기본값(1초 간격, 2회)으로 실행
ros2 run <your_pkg> move_command_tester

# N/M 지정해서 실행 (예: 0.5초 간격, 3회, 명령당 goal 2개)
ros2 run <your_pkg> move_command_tester --ros-args \
  -p interval_sec:=0.5 -p repeat_count:=3 -p goal_cnt:=2

# 트리거 (다른 터미널에서)
ros2 topic pub --once /test_trigger std_msgs/msg/Empty "{}"


"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import Empty
from geometry_msgs.msg import Pose

from navigation_command_msgs.msg import NavigationCommand


def yaw_to_quat(yaw: float):
    """2D yaw(rad) -> quaternion (z, w)."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class MoveCommandTester(Node):
    def __init__(self) -> None:
        super().__init__('move_command_tester')

        # ---- 파라미터 ----
        self.declare_parameter('interval_sec', 1.0)   # N초 간격
        self.declare_parameter('repeat_count', 2)     # M회 반복
        self.declare_parameter('goal_cnt', 1)         # 각 명령당 goal 개수

        self._interval = self.get_parameter('interval_sec').value
        self._repeat = int(self.get_parameter('repeat_count').value)
        self._goal_cnt = int(self.get_parameter('goal_cnt').value)

        self._cb_group = MutuallyExclusiveCallbackGroup()

        # ---- 퍼블리셔 / 트리거 구독 ----
        self._move_pub = self.create_publisher(
            NavigationCommand, 'move_command', 10)

        self._trigger_sub = self.create_subscription(
            Empty, '/test_trigger',
            self._trigger_callback, 10, callback_group=self._cb_group)

        # 중복 트리거로 발행 스레드가 겹치지 않도록 가드
        self._busy_lock = threading.Lock()
        self._busy = False

        self.get_logger().info(
            f'move_command_tester ready. '
            f'interval={self._interval}s, repeat={self._repeat}, '
            f'goal_cnt={self._goal_cnt}. '
            f'Trigger with: ros2 topic pub --once /test_trigger std_msgs/msg/Empty "{{}}"')

    def _trigger_callback(self, _msg: Empty) -> None:
        with self._busy_lock:
            if self._busy:
                self.get_logger().warn('Publish sequence already running; ignoring trigger.')
                return
            self._busy = True

        # 구독 콜백을 오래 점유하지 않도록 별도 스레드에서 연속 발행
        threading.Thread(target=self._publish_sequence, daemon=True).start()

    def _publish_sequence(self) -> None:
        try:
            self.get_logger().info(
                f'--- Start publishing {self._repeat} move_command(s) '
                f'every {self._interval}s ---')

            for i in range(self._repeat):
                msg = self._build_command(seq=i)
                self._move_pub.publish(msg)
                self.get_logger().info(
                    f'[{i + 1}/{self._repeat}] published move_command '
                    f'(cmd_seq_num={msg.cmd_seq_num}, goal_cnt={msg.goal_cnt})')

                # 마지막 발행 뒤에는 대기하지 않음
                if i < self._repeat - 1:
                    # rclpy.ok() 확인하며 인터럽트 가능하게 대기
                    end_t = self.get_clock().now().nanoseconds + int(self._interval * 1e9)
                    while rclpy.ok() and self.get_clock().now().nanoseconds < end_t:
                        threading.Event().wait(0.02)

            self.get_logger().info('--- Publish sequence done ---')
        finally:
            with self._busy_lock:
                self._busy = False

    def _build_command(self, seq: int) -> NavigationCommand:
        """테스트용 NavigationCommand 생성. 실제 좌표/노드ID는 필요에 맞게 수정."""
        msg = NavigationCommand()
        msg.goal_cnt = self._goal_cnt
        # cmd_seq_num 은 uint8 이므로 0~255 로 래핑
        msg.cmd_seq_num = seq % 256

        msg.goal_poses = []
        msg.from_node_id = []
        msg.to_node_id = []

        for j in range(self._goal_cnt):
            pose = Pose()
            # 더미 좌표: 명령마다/goal마다 살짝 다르게
            pose.position.x = 1.0 + seq * 0.5 + j * 0.1
            pose.position.y = 0.0
            pose.position.z = 0.0
            qz, qw = yaw_to_quat(0.0)
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = qz
            pose.orientation.w = qw
            msg.goal_poses.append(pose)

            msg.from_node_id.append(100000 + seq)
            msg.to_node_id.append(100001 + seq)

        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveCommandTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
