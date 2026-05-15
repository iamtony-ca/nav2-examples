#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Navigation Manager node.

Combines the clean structural choices of the user's draft (dataclass for
internal command state, match/case status mapping, ``wait_for_server``
guard) with a stricter multi-threaded safety model (RLock, single-block
critical sections, snapshot-then-publish for the timer).

Behavior is functionally equivalent to the original C++ node:

* Topics, QoS depths, 100 ms timer period.
* ``nav_pause_flag`` is a Bool consumed by a custom BT node that only
  cancels ``FollowPath`` (so ``resume`` can continue execution without
  re-issuing the goal).
* Monitoring fields keep the original (typo-preserving) names from the
  shared ROS interface.
"""

import copy
import threading
from dataclasses import dataclass, field
from typing import List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from std_msgs.msg import Bool, UInt8

from robot_interfaces.msg import NavigationCommand
from robot_interfaces.msg import NavigationMonitoring

# [주의] 커스텀 메시지 패키지 경로가 다를 경우 아래 임포트를 환경에 맞게 수정해주세요.
from robot_interfaces.msg import PathAgentCollisionInfo, PathStaticCollisionInfo


# ---------------------------------------------------------------------- #
# Internal state container
# ---------------------------------------------------------------------- #
@dataclass
class NavCommandData:
    """In-process mirror of the latest accepted navigation command.

    Decoupled from the ROS message type so internal logic does not break
    if the IDL evolves.
    """

    goal_cnt: int = 0
    cmd_seq_num: int = 0
    from_node_id: List[int] = field(default_factory=list)
    to_node_id: List[int] = field(default_factory=list)
    goal_poses: List[Pose] = field(default_factory=list)


# ---------------------------------------------------------------------- #
# Node
# ---------------------------------------------------------------------- #
class NavigationManager(Node):
    """Manages navigation commands and monitoring."""

    # ------------------------------------------------------------------ #
    # Construction / destruction
    # ------------------------------------------------------------------ #
    def __init__(self) -> None:
        super().__init__('navigation_manager_node')

        # ----- Concurrency primitives --------------------------------- #
        # RLock so a method holding the lock can call helpers that
        # *also* take the lock without deadlocking.
        self._state_lock = threading.RLock()

        self._cmd_cb_group = MutuallyExclusiveCallbackGroup()
        self._action_cb_group = ReentrantCallbackGroup()
        self._timer_cb_group = MutuallyExclusiveCallbackGroup()

        # ----- Internal state ----------------------------------------- #
        self._nav2_cmd_data: NavCommandData = NavCommandData()
        self._nav2_monitoring_data: NavigationMonitoring = NavigationMonitoring()
        self._goal_status: int = GoalStatus.STATUS_UNKNOWN
        self._goal_handle: Optional[ClientGoalHandle] = None
        
        self._stop_in_flight: bool = False

        # 글로벌 변수 초기화 (추가된 C++ 변수들)
        self._controller_pause_flag: bool = False
        self._path_static_collision: bool = False
        self._path_agent_collision: bool = False

        # ----- Subscriptions (Command group) -------------------------- #
        self._move_subscription = self.create_subscription(
            NavigationCommand, 'move_command',
            self._move_callback, 10, callback_group=self._cmd_cb_group)
        self._pause_subscription = self.create_subscription(
            UInt8, 'pause_command',
            self._pause_callback, 10, callback_group=self._cmd_cb_group)
        self._resume_subscription = self.create_subscription(
            UInt8, 'resume_command',
            self._resume_callback, 10, callback_group=self._cmd_cb_group)
        self._stop_subscription = self.create_subscription(
            UInt8, 'stop_command',
            self._stop_callback, 10, callback_group=self._cmd_cb_group)

        # 1) /controller_pause_flag (Bool, TRANSIENT_LOCAL)
        qos_pause = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._controller_pause_sub = self.create_subscription(
            Bool, '/controller_pause_flag',
            self._controller_pause_callback, qos_pause, callback_group=self._cmd_cb_group)

        # 2) /path_agent_collision_info
        self._agent_collision_sub = self.create_subscription(
            PathAgentCollisionInfo, '/path_agent_collision_info',
            self._agent_collision_callback, 10, callback_group=self._cmd_cb_group)

        # 3) /path_static_collision_info (custom, VOLATILE)
        self._static_collision_sub = self.create_subscription(
            PathStaticCollisionInfo, '/path_static_collision_info',
            self._static_collision_callback, 10, callback_group=self._cmd_cb_group)

        # ----- Action clients (Action group) -------------------------- #
        self._nav2_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._action_cb_group)
        self._nav2_through_poses_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses',
            callback_group=self._action_cb_group)

        # ----- Publishers --------------------------------------------- #
        self._monitoring_publisher = self.create_publisher(
            NavigationMonitoring, 'ros2_nav2_monitoring_data', 10)
        self._pause_resume_publisher = self.create_publisher(
            Bool, 'nav_pause_flag', 10)
        self._stop_complete_publisher = self.create_publisher(
            Bool, 'nav_stop_complete', 10)

        # ----- Initial state ------------------------------------------ #
        self._clear_nav2_command_data_locked()
        self._clear_nav2_monitoring_data_locked()

        # ----- Timer (10 Hz, system clock) ---------------------------- #
        self._timer = self.create_timer(
            0.1,
            self._timer_callback,
            clock=Clock(clock_type=ClockType.SYSTEM_TIME),
            callback_group=self._timer_cb_group,
        )

        # ----- Optional: warn if Nav2 is not up yet ------------------- #
        if not self._nav2_through_poses_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn(
                'navigate_through_poses action server not available yet; '
                'will be re-checked on each move command.')

        self.get_logger().info('excute, navigation_manager_node')

    def destroy_node(self) -> bool:
        """Mirror the C++ destructor's cleanup behavior."""
        with self._state_lock:
            handle = self._goal_handle
            self._clear_nav2_command_data_locked()
            self._clear_nav2_monitoring_data_locked()
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'cancel on shutdown failed: {exc}')
        return super().destroy_node()

    # ------------------------------------------------------------------ #
    # Periodic publishing
    # ------------------------------------------------------------------ #
    def _timer_callback(self) -> None:
        with self._state_lock:
            self._update_nav2_status(self._goal_status)

            # C++ 타이머 콜백 로직 통합 (파이썬의 Type Check를 위해 반드시 True/False 사용)
            if self._controller_pause_flag or self._path_agent_collision:
                self._nav2_monitoring_data.ros_nav_pause = True
            else:
                self._nav2_monitoring_data.ros_nav_pause = False

            if self._path_static_collision:
                self._nav2_monitoring_data.ros_nav_obstacle_detected = True
            else:
                self._nav2_monitoring_data.ros_nav_obstacle_detected = False

            # Throttle log (1초 주기, 파이썬에서는 float 대신 int 캐스팅으로 0/1 출력)
            self.get_logger().info(
                f"{int(self._controller_pause_flag)}/{int(self._path_agent_collision)}/{int(self._path_static_collision)}",
                throttle_duration_sec=1.0
            )

            snapshot = copy.deepcopy(self._nav2_monitoring_data)
            
        self._monitoring_publisher.publish(snapshot)

    # ------------------------------------------------------------------ #
    # New Collision / Pause Callbacks
    # ------------------------------------------------------------------ #
    def _controller_pause_callback(self, msg: Bool) -> None:
        with self._state_lock:
            self._controller_pause_flag = msg.data

    def _agent_collision_callback(self, msg: PathAgentCollisionInfo) -> None:
        with self._state_lock:
            n = len(msg.machine_id)
            # C++ 구현 방식과 완벽히 동일하게 작성 (배열을 순회하며 덮어쓰기)
            for i in range(n):
                if msg.note[i] == "non_collision":
                    self._path_agent_collision = True
                else:
                    self._path_agent_collision = False

    def _static_collision_callback(self, msg: PathStaticCollisionInfo) -> None:
        with self._state_lock:
            self._path_static_collision = msg.replan_request

    # ------------------------------------------------------------------ #
    # Topic callbacks (Command group)
    # ------------------------------------------------------------------ #
    def _stop_callback(self, msg: UInt8) -> None:
        self.get_logger().info('stop_callback!')
        self._nav2_monitoring_data.ros_nav_driving_abort = False
        with self._state_lock:
            
            if self._goal_handle is None:
                self.get_logger().info('not cancle goal, stop_callback')
                return
            handle = self._goal_handle
            self._clear_nav2_command_data_locked()
            self._nav2_cmd_data.cmd_seq_num = msg.data
            self._goal_status = GoalStatus.STATUS_CANCELED
            self._stop_in_flight = True
            self._controller_pause_flag = False
            self._path_static_collision = False
            self._path_agent_collision = False

        handle.cancel_goal_async()
        self.get_logger().info('cancle goal, stop_callback')

    def _move_callback(self, msg: NavigationCommand) -> None:
        self.get_logger().info('move_callback')

        if not msg.goal_poses:
            self.get_logger().warn('Received empty multi goal list')
            return

        if not (len(msg.goal_poses) == msg.goal_cnt
                == len(msg.from_node_id) == len(msg.to_node_id)):
            self.get_logger().error(
                f'inconsistent NavigationCommand sizes: '
                f'goal_cnt={msg.goal_cnt}, '
                f'goal_poses={len(msg.goal_poses)}, '
                f'from_node_id={len(msg.from_node_id)}, '
                f'to_node_id={len(msg.to_node_id)}')
            return

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = []

        with self._state_lock:
            self._clear_nav2_command_data_locked()
            self._nav2_cmd_data.goal_cnt = msg.goal_cnt
            self._nav2_cmd_data.cmd_seq_num = msg.cmd_seq_num

            for i in range(msg.goal_cnt):
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = 'map'
                pose_stamped.header.stamp = self.get_clock().now().to_msg()
                pose_stamped.pose = msg.goal_poses[i]
                goal_msg.poses.append(pose_stamped)

                self._nav2_cmd_data.goal_poses.append(msg.goal_poses[i])
                self._nav2_cmd_data.from_node_id.append(msg.from_node_id[i])
                self._nav2_cmd_data.to_node_id.append(msg.to_node_id[i])

                self.get_logger().info(
                    f'[move_callback] seq:{msg.cmd_seq_num}, '
                    f'from:{msg.from_node_id[i]}, to:{msg.to_node_id[i]}, '
                    f'goal[{msg.goal_poses[i].position.x:.4f}/'
                    f'{msg.goal_poses[i].position.y:.4f}/'
                    f'{msg.goal_poses[i].position.z:.4f}]')

        if not self._nav2_through_poses_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                'navigate_through_poses action server not available!')
            return

        self.get_logger().info('Request sending NavigateThroughPoses goal')
        send_goal_future = self._nav2_through_poses_client.send_goal_async(
            goal_msg, feedback_callback=self._move_feedback_callback)
        send_goal_future.add_done_callback(self._move_response_callback)

    def _pause_callback(self, msg: UInt8) -> None:
        self.get_logger().info('pause_callback')

        with self._state_lock:
            if self._goal_handle is None:
                self.get_logger().warn('not cancle goal, pause_callback')
                return
            self._nav2_cmd_data.cmd_seq_num = msg.data
            self._nav2_monitoring_data.ros_nav_cmd_seq_num = msg.data

        pause_msg = Bool()
        pause_msg.data = True
        self._pause_resume_publisher.publish(pause_msg)
        self.get_logger().info('pause flag published (FollowPath canceled in BT)')

    def _resume_callback(self, msg: UInt8) -> None:
        with self._state_lock:
            self._nav2_cmd_data.cmd_seq_num = msg.data
            self._nav2_monitoring_data.ros_nav_cmd_seq_num = msg.data

        pause_msg = Bool()
        pause_msg.data = False
        self._pause_resume_publisher.publish(pause_msg)
        self.get_logger().info('resume_callback')

    # ------------------------------------------------------------------ #
    # Action callbacks (Action group)
    # ------------------------------------------------------------------ #
    def _move_response_callback(self, future) -> None:
        goal_handle: Optional[ClientGoalHandle] = future.result()

        with self._state_lock:
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().warn('Rejected goal')
                self._goal_status = GoalStatus.STATUS_CANCELED
                self._goal_handle = None
                return

            self._goal_handle = goal_handle
            self._goal_status = GoalStatus.STATUS_ACCEPTED
            self._nav2_monitoring_data.ros_nav_cmd_seq_num = \
                self._nav2_cmd_data.cmd_seq_num

        self.get_logger().info('Accepted goal')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._move_result_callback)

    def _move_feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        number_of_poses_remaining = int(feedback.number_of_poses_remaining)
        distance_remaining = float(feedback.distance_remaining)

        with self._state_lock:
            if self._goal_status in (
                GoalStatus.STATUS_SUCCEEDED,
                GoalStatus.STATUS_ABORTED,
                GoalStatus.STATUS_CANCELED,
            ):
                return

            self._goal_status = GoalStatus.STATUS_EXECUTING

            if self._nav2_cmd_data.goal_cnt != 0:
                current_id_index = (
                    self._nav2_cmd_data.goal_cnt - number_of_poses_remaining)

                if 0 <= current_id_index < self._nav2_cmd_data.goal_cnt:
                    self._nav2_monitoring_data.ros_nav_current_node_id = \
                        self._nav2_cmd_data.from_node_id[current_id_index]
                    self._nav2_monitoring_data.ros_nav_next_node_id = \
                        self._nav2_cmd_data.to_node_id[current_id_index]
                elif current_id_index == self._nav2_cmd_data.goal_cnt:
                    pass
                else:
                    self.get_logger().warn(
                        f' current id index is not correct '
                        f'({self._nav2_cmd_data.goal_cnt}/'
                        f'{number_of_poses_remaining})')

            self._nav2_monitoring_data.ros_nav_distance_remaining = \
                distance_remaining
            self._nav2_monitoring_data.ros_nav_number_of_poses_remaining = \
                number_of_poses_remaining

            current_id = self._nav2_monitoring_data.ros_nav_current_node_id
            next_id = self._nav2_monitoring_data.ros_nav_next_node_id

        self.get_logger().info(
            f'Distance_remaining: {distance_remaining:.2f} m, '
            f'Goal: {number_of_poses_remaining}, '
            f'c_id:{current_id}, n_id:{next_id}',
            throttle_duration_sec=1.0)

    def _move_result_callback(self, future) -> None:
        result = future.result()
        status = result.status


        publish_stop_complete = False

        with self._state_lock:
            if status == GoalStatus.STATUS_SUCCEEDED:
                self._goal_status = GoalStatus.STATUS_SUCCEEDED
                self._nav2_monitoring_data.ros_nav_current_node_id = (
                    self._nav2_cmd_data.to_node_id[-1]
                    if self._nav2_cmd_data.to_node_id else 0)
                self._nav2_monitoring_data.ros_nav_next_node_id = 0
                self._nav2_monitoring_data.ros_nav_distance_remaining = 0.0
                self._nav2_monitoring_data.ros_nav_number_of_poses_remaining = 0
                self.get_logger().info('SUCCEEDED')

            elif status == GoalStatus.STATUS_ABORTED:
                self._goal_status = GoalStatus.STATUS_ABORTED
                self.get_logger().error('ABORTED')

            elif status == GoalStatus.STATUS_CANCELED:
                self._goal_status = GoalStatus.STATUS_CANCELED
                self._nav2_monitoring_data.ros_nav_cmd_seq_num = \
                    self._nav2_cmd_data.cmd_seq_num
                self.get_logger().warn('CANCELED')
                self._nav2_monitoring_data.ros_nav_driving_abort = True
                if self._stop_in_flight:
                    publish_stop_complete = True
                    self._stop_in_flight = False

            else:
                self._goal_status = GoalStatus.STATUS_UNKNOWN
                self.get_logger().error(f'Unknown result status: {status}')

            self._goal_handle = None
            if status != GoalStatus.STATUS_CANCELED:
                self._stop_in_flight = False

        if publish_stop_complete:
            done_msg = Bool()
            done_msg.data = True
            self._stop_complete_publisher.publish(done_msg)
            self.get_logger().info('nav_stop_complete published')

    # ------------------------------------------------------------------ #
    # Helpers — *_locked variants assume the caller already holds the lock
    # ------------------------------------------------------------------ #
    def _clear_nav2_command_data_locked(self) -> None:
        self._nav2_cmd_data = NavCommandData()
        self.get_logger().debug('clear_nav2_command_data')

    def _clear_nav2_monitoring_data_locked(self) -> None:
        self._nav2_monitoring_data = NavigationMonitoring()
        self.get_logger().debug('clear_nav2_monitoring_data')

    def _update_nav2_status(self, status: int) -> None:
        """Map action GoalStatus -> monitoring flags."""
        m = self._nav2_monitoring_data
        
        # NOTE: ros_nav_pause와 ros_nav_obstacle_detected는
        # 이제 타이머 콜백에서 직접 제어하므로 이곳에서 False로 강제 초기화하지 않습니다.
        m.ros_nav_driving = False
        m.ros_nav_acvtivation = False
        m.ros_nav_is_destination_reached = False
        m.ros_nav_path_search = False
        # m.ros_nav_driving_abort = False

        match status:
            case GoalStatus.STATUS_SUCCEEDED:
                m.ros_nav_is_destination_reached = True
            case GoalStatus.STATUS_ABORTED:
                m.ros_nav_driving_abort = True
            case GoalStatus.STATUS_ACCEPTED:
                m.ros_nav_acvtivation = True
            case GoalStatus.STATUS_EXECUTING:
                m.ros_nav_driving = True
                m.ros_nav_acvtivation = True
            # case GoalStatus.STATUS_CANCELED:
                # m.ros_nav_driving_abort = True
            case _:
                pass


# ---------------------------------------------------------------------- #
# Entry point
# ---------------------------------------------------------------------- #
def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationManager()

    # 4 threads = 3 callback groups + headroom for action client
    # internals. Increase if you add more groups.
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()