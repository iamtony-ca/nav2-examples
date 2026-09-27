#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
roi_range_setter.py

robot_status 토픽 (std_msgs/String)을 구독해서,
값에 따라 /global_costmap/global_costmap 노드의
agent_layer.roi_range_m 파라미터를 동적으로 변경하는 노드.

매핑 규칙:
  - "goal_received" -> 0.3
  - "driving"       -> 0.5
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterValue, ParameterType
from std_msgs.msg import String


class RoiRangeManagerNode(Node):
    """robot_status 에 따라 agent_layer.roi_range_m 을 변경하는 노드."""

    # 상태별 ROI 매핑
    STATUS_TO_ROI = {
        'RECEIVED_GOAL': 3.0,
        'DRIVING': 5.0,
        'IDLE': 3.0,
        'SUCCEEDED': 3.0,
        'FAILED': 3.0,
        'CANCELED': 3.0,
    }

    def __init__(self) -> None:
        super().__init__('roi_range_manager_node')

        # 파라미터 (필요 시 launch에서 override 가능)
        self.declare_parameter('target_node', '/global_costmap/global_costmap')
        self.declare_parameter('param_name', 'agent_layer.roi_range_m')
        self.declare_parameter('status_topic', '/robot_status')

        self.target_node_ = self.get_parameter('target_node').value
        self.param_name_ = self.get_parameter('param_name').value
        status_topic = self.get_parameter('status_topic').value

        # 같은 콜백 그룹에서 sub callback과 service call이 동시에 가능하도록
        # ReentrantCallbackGroup 사용 (MultiThreadedExecutor와 함께)
        self.cb_group_ = ReentrantCallbackGroup()

        # SetParameters 서비스 client
        # Nav2 lifecycle node 도 표준 ROS 2 parameter service 를 제공함
        srv_name = f'{self.target_node_}/set_parameters'
        self.cli_ = self.create_client(
            SetParameters,
            srv_name,
            callback_group=self.cb_group_,
        )

        # robot_status 구독
        self.sub_ = self.create_subscription(
            String,
            status_topic,
            self.status_callback,
            10,
            callback_group=self.cb_group_,
        )

        # 같은 값 중복 set 방지를 위한 캐시
        self.last_value_: float | None = None

        self.get_logger().info(
            f'RoiRangeSetter started. '
            f'target={self.target_node_}, param={self.param_name_}, '
            f'topic={status_topic}'
        )

    def status_callback(self, msg: String) -> None:
        """robot_status 메시지 수신 시 호출."""
        status = msg.data.strip()

        if status not in self.STATUS_TO_ROI:
            # 매핑되지 않는 상태는 무시 (필요시 다른 정책으로 변경)
            self.get_logger().debug(f'Ignored status: "{status}"')
            return

        target_value = self.STATUS_TO_ROI[status]

        # 동일 값 재요청 방지
        if self.last_value_ is not None and \
                abs(self.last_value_ - target_value) < 1e-9:
            return

        self.set_roi_range(target_value, status)

    def set_roi_range(self, value: float, reason: str) -> None:
        """SetParameters 서비스 호출."""
        if not self.cli_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                f'Service "{self.cli_.srv_name}" not available yet.'
            )
            return

        # rclpy.parameter.Parameter 를 사용해 안전하게 메시지 변환
        param = Parameter(
            name=self.param_name_,
            type_=Parameter.Type.DOUBLE,
            value=float(value),
        )

        request = SetParameters.Request()
        request.parameters = [param.to_parameter_msg()]

        # 비동기 호출 (callback_group 덕분에 sub callback 안에서 호출 가능)
        future = self.cli_.call_async(request)
        future.add_done_callback(
            lambda f: self._on_set_done(f, value, reason)
        )

    def _on_set_done(self, future, value: float, reason: str) -> None:
        """SetParameters 응답 처리."""
        try:
            response = future.result()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Service call failed: {e}')
            return

        if not response.results:
            self.get_logger().error('Empty SetParameters response.')
            return

        result = response.results[0]
        if result.successful:
            self.last_value_ = value
            self.get_logger().info(
                f'[{reason}] {self.param_name_} -> {value:.3f}'
            )
        else:
            self.get_logger().error(
                f'Failed to set {self.param_name_}: {result.reason}'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoiRangeManagerNode()

    # service call 과 subscription callback 이 같은 thread 에서
    # 동기적으로 충돌하지 않도록 MultiThreadedExecutor 사용
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
