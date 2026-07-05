#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ZED 카메라 이미지 토픽을 모니터링하는 ROS 2 Jazzy 노드.

동작 요약
    - 이미지가 정상 수신되면 status_topic 에 Bool(True) 를 주기적으로 발행
    - timeout_sec 동안 이미지가 한 번도 들어오지 않으면 Bool(False) 를 주기적으로 발행
    - alive(True) -> dead(False) 로 바뀌는 하강 에지에서 stop_command(UInt8) 를 즉시 1회 발행하고,
      dead 가 지속되는 동안 stop_repeat_period_sec 간격으로 stop_command 를 반복 발행

주의
    - 단일 스레드 executor(rclpy.spin) 기준으로 작성됨. image 콜백과 두 타이머 콜백이
      상태 변수를 공유하지만 단일 스레드이므로 경합이 없음.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, UInt8


class ZedMonitoringNode(Node):
    def __init__(self) -> None:
        super().__init__('zed_monitoring')

        # ---------------- 파라미터 ----------------
        # 실제 ZED wrapper 토픽에 맞게 image_topic 을 조정하세요.
        self.declare_parameter('image_topic', '/zed/zed_node/rgb/image_rect_color')
        self.declare_parameter('status_topic', 'camera_alive')
        self.declare_parameter('stop_command_topic', 'stop_command')
        self.declare_parameter('timeout_sec', 120.0)            # 미수신 판정 시간
        self.declare_parameter('check_period_sec', 1.0)         # 상태 점검 및 status 발행 주기
        self.declare_parameter('stop_repeat_period_sec', 5.0)   # dead 상태에서 stop_command 재발행 주기
        self.declare_parameter('status_log_period_sec', 5.0)    # camera_alive 상태 로깅 주기
        self.declare_parameter('stop_command_value', 1)         # stop_command 에 실을 값

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        status_topic = self.get_parameter('status_topic').get_parameter_value().string_value
        stop_topic = self.get_parameter('stop_command_topic').get_parameter_value().string_value
        self._timeout_sec: float = self.get_parameter('timeout_sec').value
        check_period: float = self.get_parameter('check_period_sec').value
        stop_repeat_period: float = self.get_parameter('stop_repeat_period_sec').value
        self._status_log_period: float = self.get_parameter('status_log_period_sec').value
        self._stop_value: int = int(self.get_parameter('stop_command_value').value)

        # ---------------- 상태 변수 ----------------
        # 노드 시작 시점을 기준으로 카운트다운 시작 (초기 grace period 역할)
        self._last_msg_time = self.get_clock().now()
        # 이전 tick 의 alive 상태. 초기값 True 로 두어 timeout 후 '최초 False 전환'을 에지로 감지
        self._prev_alive: bool = True
        # 현재 생존 상태. stop 반복 타이머가 발행 여부를 판단하는 게이트로 사용
        self._is_alive: bool = True

        # ---------------- 통신 인터페이스 ----------------
        # 카메라 이미지는 보통 best-effort 로 발행되므로 sensor_data QoS 로 맞춰서 구독
        self._image_sub = self.create_subscription(
            Image, image_topic, self._image_callback, qos_profile_sensor_data
        )
        # status/stop_command 는 default QoS(KEEP_LAST, RELIABLE, VOLATILE) 사용.
        # create_publisher 에 정수를 넘기면 그 값이 history depth 로 적용됨.
        # status: 0.5s 주기로 계속 발행하므로 depth 1 로 충분 (latch 불필요)
        self._status_pub = self.create_publisher(Bool, status_topic, 1)
        # stop_command: dead 동안 반복 발행되므로 depth 5 의 소량 버퍼면 충분
        self._stop_pub = self.create_publisher(UInt8, stop_topic, 5)

        # ---------------- 타이머 ----------------
        # 1) 생존 판정 + status 발행 타이머
        self._check_timer = self.create_timer(check_period, self._check_timer_callback)
        # 2) dead 상태에서 stop_command 를 주기적으로 재발행하는 타이머
        #    (alive 일 때는 콜백 내부 게이트에서 막힘)
        self._stop_timer = self.create_timer(stop_repeat_period, self._stop_timer_callback)

        # ---------------- 파라미터 로깅 ----------------
        # __init__ 마지막에 적용된 모든 파라미터 값을 출력
        self.get_logger().info(
            '[zed_monitoring] 파라미터 로드 완료:\n'
            f'  image_topic            = {image_topic}\n'
            f'  status_topic           = {status_topic}\n'
            f'  stop_command_topic     = {stop_topic}\n'
            f'  timeout_sec            = {self._timeout_sec}\n'
            f'  check_period_sec       = {check_period}\n'
            f'  stop_repeat_period_sec = {stop_repeat_period}\n'
            f'  status_log_period_sec  = {self._status_log_period}\n'
            f'  stop_command_value     = {self._stop_value}'
        )

    def _image_callback(self, msg: Image) -> None:
        """이미지가 들어올 때마다 마지막 수신 시각만 갱신 (가볍게 유지)."""
        self._last_msg_time = self.get_clock().now()

    def _check_timer_callback(self) -> None:
        """주기적으로 생존 여부를 판정하고 status 를 발행."""
        elapsed = (self.get_clock().now() - self._last_msg_time).nanoseconds * 1e-9
        is_alive = elapsed < self._timeout_sec

        # 1) 상태를 주기적으로 발행 (alive 든 dead 든 매 주기)
        self._status_pub.publish(Bool(data=is_alive))

        # 2) camera_alive 상태 로깅 (throttle: status_log_period_sec 마다 최대 1회)
        #    alive: INFO, dead: WARN. INFO/WARN 은 서로 다른 callsite 라 throttle 타이머가 독립적임.
        #    기본 시간 소스는 steady time(벽시계). sim time 에 묶고 싶으면
        #    throttle_time_source_type=self.get_clock() 을 추가.
        if is_alive:
            self.get_logger().info(
                '[zed_monitoring] camera_alive = true',
                throttle_duration_sec=self._status_log_period,
            )
        else:
            self.get_logger().warn(
                '[zed_monitoring] camera_alive = false (이미지 미수신)',
                throttle_duration_sec=self._status_log_period,
            )

        # 3) alive -> dead 하강 에지: 즉시 1회 발행하고 재발행 타이머를 리셋해
        #    다음 반복 발행이 정확히 stop_repeat_period 뒤부터 시작되도록 함
        if (not is_alive) and self._prev_alive:
            self._publish_stop_command('dead 전환')
            self._stop_timer.reset()

        self._prev_alive = is_alive
        self._is_alive = is_alive

    def _stop_timer_callback(self) -> None:
        """dead 상태가 지속되는 동안 stop_command 를 주기적으로 재발행."""
        if not self._is_alive:
            self._publish_stop_command('dead 지속')

    def _publish_stop_command(self, reason: str) -> None:
        self._stop_pub.publish(UInt8(data=self._stop_value))
        self.get_logger().warn(
            f'[zed_monitoring] {reason} → stop_command({self._stop_value}) 발행'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZedMonitoringNode()
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