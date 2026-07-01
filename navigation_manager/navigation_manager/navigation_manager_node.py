#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Navigation Manager node.

Combines the clean structural choices of the user's draft (dataclass for
internal command state, match/case status mapping, ``wait_for_server``
guard) with a stricter multi-threaded safety model (RLock, single-block
critical sections, snapshot-then-publish for the timer).
"""

import copy
import time  # while 문 내부 대기를 위해 추가
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
from std_msgs.msg import Bool, UInt8, String
from nav2_msgs.msg import BehaviorTreeLog, BehaviorTreeStatusChange
from nav2_msgs.srv import ClearEntireCostmap


# Path 메시지 임포트
from nav_msgs.msg import Path

from navigation_command_msgs.msg import NavigationCommand
from navigation_monitoring_msgs.msg import NavigationMonitoring

# [주의] 커스텀 메시지 패키지 경로
from robot_interfaces.msg import PathAgentCollisionInfo, PathStaticCollisionInfo


# ---------------------------------------------------------------------- #
# Internal state container
# ---------------------------------------------------------------------- #
@dataclass
class NavCommandData:
    goal_cnt: int = 0
    cmd_seq_num: int = 0
    from_node_id: List[int] = field(default_factory=list)
    to_node_id: List[int] = field(default_factory=list)
    goal_poses: List[Pose] = field(default_factory=list)


# ---------------------------------------------------------------------- #
# Node
# ---------------------------------------------------------------------- #
class NavigationManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('navigation_manager_node')

        self._state_lock = threading.RLock()

        # [중요 변경] 데드락 방지를 위해 콜백 그룹을 분리했습니다.
        # _move_callback은 _cmd_cb_group에서 block 되며, 
        # 상태 업데이트와 stop명령은 _state_update_cb_group에서 독립적으로 스레드를 점유해 실행됩니다.
        self._cmd_cb_group = MutuallyExclusiveCallbackGroup()
        self._state_update_cb_group = MutuallyExclusiveCallbackGroup()
        self._action_cb_group = ReentrantCallbackGroup()
        self._timer_cb_group = MutuallyExclusiveCallbackGroup()
        self._srv_cb_group = MutuallyExclusiveCallbackGroup()

        # ----- Internal state ----------------------------------------- #
        self._nav2_cmd_data: NavCommandData = NavCommandData()
        self._nav2_monitoring_data: NavigationMonitoring = NavigationMonitoring()
        self._goal_status: int = GoalStatus.STATUS_UNKNOWN
        self._goal_handle: Optional[ClientGoalHandle] = None
        
        self._stop_in_flight: bool = False

        # 글로벌 변수 초기화
        self._controller_pause_flag: bool = False
        self._path_static_collision: bool = False
        self._path_agent_collision: bool = False
        
        # 추가된 변수들 (while 문 조건용)
        self._robot_status: str = ""
        self._static_is_goal_occupied: bool = False
        self._static_is_last_goal_occupied: bool = False
        self._agent_is_goal_occupied: bool = False
        self._agent_is_last_goal_occupied: bool = False

        self._static_is_status_ready: bool = False 
        self._agent_is_status_ready: bool = False 

        self.nav_stop_command: bool = False  # nav_stop 명령 수신 여부를 나타내는 플래그

        # ----- Subscriptions ------------------------------------------ #
        # move만 _cmd_cb_group 할당 (block 발생 지점)
        self._move_subscription = self.create_subscription(
            NavigationCommand, 'move_command',
            self._move_callback, 10, callback_group=self._cmd_cb_group)
        
        # 나머지 제어 및 상태 업데이트는 _state_update_cb_group 할당 (block 방지)
        self._pause_subscription = self.create_subscription(
            UInt8, 'pause_command',
            self._pause_callback, 10, callback_group=self._state_update_cb_group)
        self._resume_subscription = self.create_subscription(
            UInt8, 'resume_command',
            self._resume_callback, 10, callback_group=self._state_update_cb_group)
        self._nav_stop_subscription = self.create_subscription(
            UInt8, 'stop_command',
            self._nav_stop_callback, 10, callback_group=self._state_update_cb_group)

        self._main_stop_subscription = self.create_subscription(
            UInt8, '/main_stop_command',
            self._main_stop_callback, 10, callback_group=self._state_update_cb_group)

        self._reset_subscription = self.create_subscription(
            UInt8, '/reset_command',
            self._reset_callback, 10, callback_group=self._state_update_cb_group)
        
        # /robot_status 추가
        self._robot_status_sub = self.create_subscription(
            String, '/robot_status',
            self._robot_status_callback, 10, callback_group=self._state_update_cb_group)

        qos_pause = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._controller_pause_sub = self.create_subscription(
            Bool, '/controller_pause_flag',
            self._controller_pause_callback, qos_pause, callback_group=self._state_update_cb_group)

        self._agent_collision_sub = self.create_subscription(
            PathAgentCollisionInfo, '/path_agent_collision_info',
            self._agent_collision_callback, 10, callback_group=self._state_update_cb_group)

        self._static_collision_sub = self.create_subscription(
            PathStaticCollisionInfo, '/path_static_collision_info',
            self._static_collision_callback, 10, callback_group=self._state_update_cb_group)


        self.client_local = self.create_client(
            ClearEntireCostmap, 
            '/local_costmap/clear_entirely_local_costmap',
            callback_group=self._srv_cb_group  # <--- 추가
        )
        self.client_global = self.create_client(
            ClearEntireCostmap, 
            '/global_costmap/clear_entirely_global_costmap',
            callback_group=self._srv_cb_group  # <--- 추가
        )


        # ----- Action clients ----------------------------------------- #
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
        self._bt_log_publisher = self.create_publisher(
            BehaviorTreeLog, '/behavior_tree_log', 10)
            
        # [추가된 부분] remaining_goals 퍼블리셔 생성
        self._remaining_goals_publisher = self.create_publisher(
            Path, '/remaining_goals', 10)

        # ----- Initial state ------------------------------------------ #
        self._clear_nav2_command_data_locked()
        self._clear_nav2_monitoring_data_locked()

        # ----- Timer -------------------------------------------------- #
        self._timer = self.create_timer(
            0.1,
            self._timer_callback,
            clock=Clock(clock_type=ClockType.SYSTEM_TIME),
            callback_group=self._timer_cb_group,
        )

        if not self._nav2_through_poses_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn(
                'navigate_through_poses action server not available yet; '
                'will be re-checked on each move command.')

        self.get_logger().info('excute, navigation_manager_node')

    def destroy_node(self) -> bool:
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

            if self._controller_pause_flag or self._path_agent_collision:
                self._nav2_monitoring_data.ros_nav_pause = True
            else:
                self._nav2_monitoring_data.ros_nav_pause = False

            if self._path_static_collision:
                self._nav2_monitoring_data.ros_nav_obstacle_detected = True
            else:
                self._nav2_monitoring_data.ros_nav_obstacle_detected = False

            self.get_logger().info(
                f"int(self._controller_pause_flag)/int(self._path_agent_collision)/int(self._path_static_collision): {int(self._controller_pause_flag)}/{int(self._path_agent_collision)}/{int(self._path_static_collision)}",
                throttle_duration_sec=3.0
            )
            snapshot = copy.deepcopy(self._nav2_monitoring_data)
            
        self._monitoring_publisher.publish(snapshot)

    # ------------------------------------------------------------------ #
    # New Collision / Pause Callbacks
    # ------------------------------------------------------------------ #
    def _robot_status_callback(self, msg: String) -> None:
        with self._state_lock:
            self._robot_status = msg.data

    def _controller_pause_callback(self, msg: Bool) -> None:
        with self._state_lock:
            self._controller_pause_flag = msg.data

    def _agent_collision_callback(self, msg: PathAgentCollisionInfo) -> None:
        with self._state_lock:
            n = len(msg.machine_id)
            for i in range(n):
                if msg.note[i] == "non_collision":
                    self._path_agent_collision = True
                else:
                    self._path_agent_collision = False
            
            # [가정] 커스텀 메시지에 아래 필드가 있다고 가정했습니다. 이름이 다르면 맞춰서 수정하세요.
            self._agent_is_goal_occupied = msg.is_goal_occupied
            self._agent_is_last_goal_occupied = msg.is_last_goal_occupied
            self._agent_is_status_ready = msg.is_status_ready

    def _static_collision_callback(self, msg: PathStaticCollisionInfo) -> None:
        with self._state_lock:
            self._path_static_collision = msg.replan_request
            
            # [가정] 커스텀 메시지에 아래 필드가 있다고 가정했습니다. 이름이 다르면 맞춰서 수정하세요.
            self._static_is_goal_occupied = msg.is_goal_occupied
            self._static_is_last_goal_occupied = msg.is_last_goal_occupied
            self._static_is_status_ready = msg.is_status_ready

    # ------------------------------------------------------------------ #
    # Topic callbacks
    # ------------------------------------------------------------------ #
    def _nav_stop_callback(self, msg: UInt8) -> None:
        self.get_logger().info(f'stop_callback!, cmd_seq_num: {msg.data}')
        
        with self._state_lock:
            # while 문 중단을 위한 플래그 설정
            self.nav_stop_command = True
            self._stop_in_flight = True
            self._nav2_monitoring_data.ros_nav_driving_abort = False
            
            if self._goal_handle is None:
                self.get_logger().info('not cancle goal, stop_callback')
                return
            handle = self._goal_handle
            self._clear_nav2_command_data_locked()
            self._nav2_cmd_data.cmd_seq_num = msg.data
            self._goal_status = GoalStatus.STATUS_CANCELED
            self._controller_pause_flag = False                       ### testing...
            self._path_static_collision = False                       ### testing...
            self._path_agent_collision = False                        ### testing...

        handle.cancel_goal_async()
        self.get_logger().info(f'cancle goal, stop_callback')

    def _main_stop_callback(self, msg: UInt8) -> None:
        self.get_logger().info(f'stop_callback!, cmd_seq_num: {msg.data}')
        self._nav2_monitoring_data.ros_nav_driving_abort = False
        with self._state_lock:
            # while 문 중단을 위한 플래그 설정
            self._stop_in_flight = True
            
            if self._goal_handle is None:
                self.get_logger().info('not cancle goal, stop_callback')
                return
            handle = self._goal_handle
            self._clear_nav2_command_data_locked()
            self._nav2_cmd_data.cmd_seq_num = msg.data
            self._goal_status = GoalStatus.STATUS_CANCELED
            self._controller_pause_flag = False                       ### testing...
            self._path_static_collision = False                       ### testing...
            self._path_agent_collision = False                        ### testing...

        handle.cancel_goal_async()
        self.get_logger().info(f'cancle goal, stop_callback')


    def _reset_callback(self, msg: UInt8) -> None:
        self.get_logger().info(f'reset_callback!, cmd_seq_num: {msg.data}')
        
        with self._state_lock:
            self._nav2_monitoring_data.ros_nav_driving_abort = False
   
        self.get_logger().info(f'reset abort status, reset_callback')

    

    def _move_callback(self, msg: NavigationCommand) -> None:
        self.get_logger().info('move_callback')
        self._stop_in_flight = False
        cond_ready = False
        cond_static = False
        cond_agent = False



        self.clear_both_costmaps()

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
            self._nav2_monitoring_data.ros_nav_driving_abort = False 
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

            
            self.get_logger().info(f'goal poses: {self._nav2_cmd_data.goal_poses}')





        # ==================================================================
        # [수정된 부분] 단일 while 문 기반의 조건 대기 및 타임아웃 로직
        # ==================================================================
        remaining_path_msg = Path()
        remaining_path_msg.header.frame_id = 'map'
        remaining_path_msg.poses = goal_msg.poses

        wait_start_time = None
        target_duration = 1.0  # 조건 충족 유지 시간 (초)
        
        # 타임아웃 설정 변수
        loop_start_time = self.get_clock().now()
        MAX_WAIT_TIMEOUT = 150.0  # 최대 대기 시간 (초)
        TIMEOUT_PUB_N_SEC = 0.2  # 타임아웃 발생 시 메시지 퍼블리시 유지 시간 (초)
        
        is_timed_out = False          # 타임아웃 상태를 나타내는 플래그
        timeout_pub_start = None      # 타임아웃 퍼블리시 시작 시간

        self.get_logger().info('Waiting for Ready and Non-collision conditions...')

        while rclpy.ok():
            # 1. Break + Return 조건 (stop_command 수신 시 즉시 종료)
            with self._state_lock:
                if self._stop_in_flight:
                    self.get_logger().warn('Move aborted due to stop command during wait phase.')
                    return

            # --------------------------------------------------------------
            # [상태 A] 타임아웃 발생 이후: N초 동안 빈 Goal과 IDLE 퍼블리시
            # --------------------------------------------------------------
            if is_timed_out:
                pub_elapsed = (self.get_clock().now() - timeout_pub_start).nanoseconds / 1e9
                if pub_elapsed > TIMEOUT_PUB_N_SEC:
                    # N초 퍼블리시가 끝났으므로 함수 자체를 종료 (Goal 전송 X)
                    self._nav2_monitoring_data.ros_nav_driving_abort = True       #### testing...

                    return 
                
                remaining_path_msg.header.stamp = self.get_clock().now().to_msg()
                self._remaining_goals_publisher.publish(remaining_path_msg)
                
                bt_log_msg = BehaviorTreeLog()
                bt_log_msg.timestamp = self.get_clock().now().to_msg()
                status_event = BehaviorTreeStatusChange()
                status_event.node_name = 'NavigationManagerReady'
                status_event.current_status = 'IDLE'
                bt_log_msg.event_log.append(status_event)
                self._bt_log_publisher.publish(bt_log_msg)

            # --------------------------------------------------------------
            # [상태 B] 정상 대기 상태: 목표 조건 대기 및 RUNNING 퍼블리시
            # --------------------------------------------------------------
            else:
                elapsed_total = (self.get_clock().now() - loop_start_time).nanoseconds / 1e9
                
                if elapsed_total > 3.0:
                    self.get_logger().warn(
                            f'waiting time: {elapsed_total:.2f} / {MAX_WAIT_TIMEOUT} sec bcs goals are occupied!', 
                            throttle_duration_sec=5.0)
                    self.get_logger().warn(
                            f'cond_ready: {cond_ready} / cond_static: {cond_static} / cond_agent: {cond_agent}', 
                            throttle_duration_sec=5.0)                    


                # 타임아웃 체크
                if elapsed_total > MAX_WAIT_TIMEOUT:
                    self.get_logger().error(f'Timeout ({MAX_WAIT_TIMEOUT}s) reached! Clearing goals and publishing IDLE for {TIMEOUT_PUB_N_SEC}s.')
                    is_timed_out = True
                    timeout_pub_start = self.get_clock().now()
                    remaining_path_msg.poses = []  # Goal 클리어
                    continue  # 즉시 다음 루프(상태 A)로 넘어가서 IDLE 퍼블리시 시작

                # 정상 퍼블리시 (RUNNING & Remaining Goals)
                remaining_path_msg.header.stamp = self.get_clock().now().to_msg()
                self._remaining_goals_publisher.publish(remaining_path_msg)

                bt_log_msg = BehaviorTreeLog()
                bt_log_msg.timestamp = self.get_clock().now().to_msg()
                status_event = BehaviorTreeStatusChange()
                status_event.node_name = 'NavigationManagerReady'
                status_event.current_status = 'RUNNING'
                bt_log_msg.event_log.append(status_event)
                self._bt_log_publisher.publish(bt_log_msg)

                # 조건 검사 로직
                with self._state_lock:
                    cond_ready = (self._robot_status == "READY")
                    cond_static = (not self._static_is_goal_occupied) and (not self._static_is_last_goal_occupied) and (self._static_is_status_ready)
                    cond_agent = (not self._agent_is_goal_occupied) and (not self._agent_is_last_goal_occupied) and (self._agent_is_status_ready)
                    
                    all_conditions_met = cond_ready and cond_static and cond_agent

                # 타겟 유지 시간 충족 확인
                if all_conditions_met:
                    if wait_start_time is None:
                        wait_start_time = self.get_clock().now()
                    else:
                        elapsed_sec = (self.get_clock().now() - wait_start_time).nanoseconds / 1e9
                        if elapsed_sec >= target_duration:
                            self.get_logger().info(f'All conditions met for {target_duration} sec. Breaking loop to send goal.')
                            break  # 대기 루프 탈출 -> Action 서버로 Goal 전송
                else:
                    wait_start_time = None

            # 10Hz 주기로 체크
            time.sleep(0.1)
        # ==================================================================



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
    # Action callbacks
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

            self._nav2_monitoring_data.ros_nav_distance_remaining = distance_remaining
            self._nav2_monitoring_data.ros_nav_number_of_poses_remaining = number_of_poses_remaining

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

                if self.nav_stop_command:
                    self.nav_stop_command = False
                    self._nav2_monitoring_data.ros_nav_driving_abort = True       #### testing...
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
    # Helpers
    # ------------------------------------------------------------------ #
    def _clear_nav2_command_data_locked(self) -> None:
        self._nav2_cmd_data = NavCommandData()
        self.get_logger().debug('clear_nav2_command_data')

    def _clear_nav2_monitoring_data_locked(self) -> None:
        self._nav2_monitoring_data = NavigationMonitoring()
        self.get_logger().debug('clear_nav2_monitoring_data')

    def _update_nav2_status(self, status: int) -> None:
        m = self._nav2_monitoring_data
        
        m.ros_nav_driving = False
        m.ros_nav_acvtivation = False
        m.ros_nav_is_destination_reached = False
        m.ros_nav_path_search = False

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
            case _:
                pass


    def wait_for_services(self) -> bool:
        """두 Costmap 서비스가 모두 활성화될 때까지 무한 대기합니다."""
        self.get_logger().info('Waiting for costmap clear services to become active...')
        
        # local_costmap 대기
        while not self.client_local.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for local_costmap service.')
                return False
            self.get_logger().info('Still waiting for /local_costmap/clear_entirely_local_costmap...')
            
        # global_costmap 대기
        while not self.client_global.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for global_costmap service.')
                return False
            self.get_logger().info('Still waiting for /global_costmap/clear_entirely_global_costmap...')
            
        self.get_logger().info('Both costmap services are now ready!')
        return True



    def _call_clear_costmap(self, client, name: str) -> bool:
        """단일 costmap clear 서비스를 호출하고 응답까지 안전하게 대기."""
        req = ClearEntireCostmap.Request()
        future = client.call_async(req)

        # 다른 콜백 그룹(_srv_cb_group)의 스레드가 응답을 처리하므로
        # 여기서는 spin 없이 Event 로 대기 가능.
        done_event = threading.Event()
        future.add_done_callback(lambda _f: done_event.set())

        if not done_event.wait(timeout=5.0):
            self.get_logger().error(f'Timed out clearing {name} costmap.')
            return False

        if future.exception() is not None:
            self.get_logger().error(
                f'Failed to clear {name} costmap: {future.exception()}')
            return False

        # ClearEntireCostmap.Response 는 빈 메시지지만, 성공 시 None 이 아님.
        self.get_logger().info(f'Successfully cleared {name} costmap.')
        return True


    def clear_both_costmaps(self) -> None:
        """두 Costmap을 모두 초기화합니다."""
        if not self.wait_for_services():
            return
        self._call_clear_costmap(self.client_local, 'local')
        self._call_clear_costmap(self.client_global, 'global')


    # def clear_both_costmaps(self):
    #     """두 Costmap을 모두 초기화합니다."""
    #     if not self.wait_for_services():
    #         return

    #     req = ClearEntireCostmap.Request()

    #     # Local Costmap 초기화 요청
    #     future_local = self.client_local.call_async(req)
    #     # rclpy.spin_until_future_complete(self, future_local)  <-- 삭제
    #     result_local = future_local.result()  # <-- 추가: MultiThread 환경에서 안전한 블로킹 대기
        
    #     if result_local is not None:
    #         self.get_logger().info('Successfully cleared Local Costmap.')
    #     else:
    #         self.get_logger().error('Failed to clear Local Costmap.')

    #     # Global Costmap 초기화 요청
    #     future_global = self.client_global.call_async(req)
    #     # rclpy.spin_until_future_complete(self, future_global) <-- 삭제
    #     result_global = future_global.result() # <-- 추가
        
    #     if result_global is not None:
    #         self.get_logger().info('Successfully cleared Global Costmap.')
    #     else:
    #         self.get_logger().error('Failed to clear Global Costmap.')


# ---------------------------------------------------------------------- #
# Entry point
# ---------------------------------------------------------------------- #
def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationManagerNode()

    executor = MultiThreadedExecutor(num_threads=9)
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
