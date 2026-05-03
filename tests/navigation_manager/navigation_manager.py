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

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from std_msgs.msg import Bool, UInt8

from amr_navigation_interfaces.msg import NavigationCommand
from amr_navigation_interfaces.msg import NavigationMonitoring


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

        # Three callback groups so command / action / timer callbacks
        # can run on different executor threads.
        #
        # - Commands & timer: MutuallyExclusive — within each group
        #   ordering is preserved and we never want the timer or two
        #   user commands to interleave with themselves.
        #
        # - Action: Reentrant. ``rclpy.ActionClient`` internally
        #   multiplexes several services (send_goal / cancel_goal /
        #   get_result) and a feedback subscription. Chaining
        #   ``add_done_callback`` on result futures from inside another
        #   action-related callback effectively creates a hidden
        #   callback dependency. With a MutuallyExclusive group those
        #   hidden callbacks can sit behind a long-running feedback
        #   handler and stall — in the worst case deadlock. Reentrant
        #   lets the executor schedule them freely.
        #
        # Thread-safety of *our* shared state is guaranteed by
        # ``self._state_lock``, not by callback-group serialization.
        self._cmd_cb_group = MutuallyExclusiveCallbackGroup()
        self._action_cb_group = ReentrantCallbackGroup()
        self._timer_cb_group = MutuallyExclusiveCallbackGroup()

        # ----- Internal state ----------------------------------------- #
        self._nav2_cmd_data: NavCommandData = NavCommandData()
        self._nav2_monitoring_data: NavigationMonitoring = NavigationMonitoring()
        self._goal_status: int = GoalStatus.STATUS_UNKNOWN
        self._goal_handle: Optional[ClientGoalHandle] = None
        # True between a user-initiated stop and the corresponding
        # CANCELED result. Used to publish ``nav_stop_complete`` only
        # for stops the user actually requested (not preemption /
        # external cancels).
        self._stop_in_flight: bool = False

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

        # ----- Action clients (Action group) -------------------------- #
        # NavigateToPose kept for parity with the original C++ node even
        # though the active code-path only uses NavigateThroughPoses.
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
        # Edge-trigger published once per stop-induced cancel completion.
        # depth=1 + transient_local would also be reasonable if late
        # subscribers must catch the latest event; keeping depth=10 +
        # default volatile QoS for symmetry with the other publishers.
        self._stop_complete_publisher = self.create_publisher(
            Bool, 'nav_stop_complete', 10)

        # ----- Initial state ------------------------------------------ #
        self._clear_nav2_command_data_locked()
        self._clear_nav2_monitoring_data_locked()

        # ----- Timer (10 Hz, system clock) ---------------------------- #
        # SYSTEM_TIME mirrors C++ ``create_wall_timer`` semantics so the
        # monitoring publish rate is independent of /clock and
        # use_sim_time.
        self._timer = self.create_timer(
            0.1,
            self._timer_callback,
            clock=Clock(clock_type=ClockType.SYSTEM_TIME),
            callback_group=self._timer_cb_group,
        )

        # ----- Optional: warn if Nav2 is not up yet ------------------- #
        # Non-blocking probe — actual server availability is re-checked
        # in ``_move_callback`` before each goal send.
        if not self._nav2_through_poses_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn(
                'navigate_through_poses action server not available yet; '
                'will be re-checked on each move command.')

        self.get_logger().info('excute, navigation_manager_node')

    def destroy_node(self) -> bool:
        """Mirror the C++ destructor's cleanup behavior."""
        with self._state_lock:
            # Best-effort cancel of any in-flight goal so Nav2 doesn't
            # keep driving after this node exits.
            handle = self._goal_handle
            self._clear_nav2_command_data_locked()
            self._clear_nav2_monitoring_data_locked()
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(
                    f'cancel on shutdown failed: {exc}')
        return super().destroy_node()

    # ------------------------------------------------------------------ #
    # Periodic publishing
    # ------------------------------------------------------------------ #
    def _timer_callback(self) -> None:
        # Update flags + take a deep copy under the lock, then publish
        # OUTSIDE the lock. publish() can block on DDS internals and we
        # don't want to stall command/action callbacks.
        with self._state_lock:
            self._update_nav2_status(self._goal_status)
            snapshot = copy.deepcopy(self._nav2_monitoring_data)
        self._monitoring_publisher.publish(snapshot)

    # ------------------------------------------------------------------ #
    # Topic callbacks (Command group)
    # ------------------------------------------------------------------ #
    def _stop_callback(self, msg: UInt8) -> None:
        self.get_logger().info('stop_callback')

        # Single critical section so the goal_handle check, state reset
        # and status update happen atomically. The cancel RPC itself is
        # issued OUTSIDE the lock to avoid holding it across middleware.
        with self._state_lock:
            if self._goal_handle is None:
                self.get_logger().info('not cancle goal, stop_callback')
                return
            handle = self._goal_handle
            self._clear_nav2_command_data_locked()
            self._nav2_cmd_data.cmd_seq_num = msg.data
            self._goal_status = GoalStatus.STATUS_CANCELED
            # Mark this cancel as user-initiated so the result callback
            # knows to fire ``nav_stop_complete`` once Nav2 actually
            # finishes terminating the goal.
            self._stop_in_flight = True

        handle.cancel_goal_async()
        self.get_logger().info('cancle goal, stop_callback')

    def _move_callback(self, msg: NavigationCommand) -> None:
        self.get_logger().info('move_callback')

        if not msg.goal_poses:
            self.get_logger().warn('Received empty multi goal list')
            return

        # Defensive length validation — the C++ original would crash on
        # mismatched lengths; we'd rather log and bail.
        if not (len(msg.goal_poses) == msg.goal_cnt
                == len(msg.from_node_id) == len(msg.to_node_id)):
            self.get_logger().error(
                f'inconsistent NavigationCommand sizes: '
                f'goal_cnt={msg.goal_cnt}, '
                f'goal_poses={len(msg.goal_poses)}, '
                f'from_node_id={len(msg.from_node_id)}, '
                f'to_node_id={len(msg.to_node_id)}')
            return

        # Build the action goal in a local variable first.
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = []

        with self._state_lock:
            self._clear_nav2_command_data_locked()
            self._nav2_cmd_data.goal_cnt = msg.goal_cnt
            self._nav2_cmd_data.cmd_seq_num = msg.cmd_seq_num

            for i in range(msg.goal_cnt):
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = 'map'
                # Stamp per iteration to mirror C++ ``this->now()`` exactly.
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

        # Server availability check + send_goal_async OUTSIDE the lock.
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

        # Read+update under lock; publish outside.
        with self._state_lock:
            if self._goal_handle is None:
                self.get_logger().warn('not cancle goal, pause_callback')
                return
            self._nav2_cmd_data.cmd_seq_num = msg.data
            self._nav2_monitoring_data.ros_nav_cmd_seq_num = msg.data

        # NOTE: do NOT cancel the action goal here. The custom BT node
        # consuming ``nav_pause_flag`` only cancels the FollowPath child,
        # so the parent goal stays alive and ``resume`` can continue
        # without re-issuing it.
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
        """Fired when the action server accepts/rejects the goal."""
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
        # Chain the result future OUTSIDE the lock.
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._move_result_callback)

    def _move_feedback_callback(self, feedback_msg) -> None:
        """Fired for each NavigateThroughPoses feedback message."""
        feedback = feedback_msg.feedback
        number_of_poses_remaining = int(feedback.number_of_poses_remaining)
        distance_remaining = float(feedback.distance_remaining)

        with self._state_lock:
            # Reentrant action group means a late feedback can race the
            # result callback. Don't downgrade a terminal status back
            # to EXECUTING.
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
                    # Final node — handled in the result callback's
                    # SUCCEEDED branch, no-op here.
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

        # Throttled to ~1 Hz, matching RCLCPP_INFO_THROTTLE(1000).
        self.get_logger().info(
            f'Distance_remaining: {distance_remaining:.2f} m, '
            f'Goal: {number_of_poses_remaining}, '
            f'c_id:{current_id}, n_id:{next_id}',
            throttle_duration_sec=1.0)

    def _move_result_callback(self, future) -> None:
        """Fired once the action terminates (success/abort/cancel)."""
        result = future.result()
        status = result.status

        # Captured under lock, used outside.
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
                # Only fire nav_stop_complete for a user-initiated stop;
                # consume the flag so duplicate publishes can't happen.
                if self._stop_in_flight:
                    publish_stop_complete = True
                    self._stop_in_flight = False

            else:
                self._goal_status = GoalStatus.STATUS_UNKNOWN
                self.get_logger().error(f'Unknown result status: {status}')

            self._goal_handle = None
            # If termination wasn't a CANCELED (e.g. SUCCEEDED arrived
            # before our cancel could take effect), drop the flag so a
            # later unrelated cancel doesn't accidentally publish.
            if status != GoalStatus.STATUS_CANCELED:
                self._stop_in_flight = False

        # Publish OUTSIDE the lock — DDS publish can briefly block.
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
        """Map action GoalStatus -> monitoring flags.

        Caller must hold ``self._state_lock``. Default-zero block first,
        then per-status overrides — keeps the table compact without
        changing the truth-table from the C++ original.
        """
        m = self._nav2_monitoring_data
        m.ros_nav_driving = 0
        m.ros_nav_acvtivation = 0
        m.ros_nav_is_destination_reached = 0
        m.ros_nav_pause = 0
        m.ros_nav_path_search = 0
        m.ros_nav_driving_abort = 0

        match status:
            case GoalStatus.STATUS_SUCCEEDED:
                m.ros_nav_is_destination_reached = 1
            case GoalStatus.STATUS_ABORTED:
                m.ros_nav_driving_abort = 1
            case GoalStatus.STATUS_ACCEPTED:
                m.ros_nav_acvtivation = 1
            case GoalStatus.STATUS_EXECUTING:
                m.ros_nav_driving = 1
                m.ros_nav_acvtivation = 1
            case _:
                # STATUS_CANCELED, STATUS_CANCELING, STATUS_UNKNOWN, ...
                # all leave the default-zero block intact.
                pass


# ---------------------------------------------------------------------- #
# Entry point
# ---------------------------------------------------------------------- #
def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationManager()

    # 4 threads = 3 callback groups + headroom for action client
    # internals. Increase if you add more groups.
    executor = MultiThreadedExecutor(num_threads=4)
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