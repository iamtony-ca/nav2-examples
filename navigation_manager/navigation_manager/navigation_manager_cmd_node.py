#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# [09-27] action/service 방식 navigation_manager (현장 0923 8cb26712 + amhs 0921 수정 병합 + B1 수정).
# navigation_manager_main 이 use_command_services=true 일 때 띄운다. false 면 navigation_manager_node.py(기존 토픽 방식).
# B2 수정(09-27): 대기 단계의 관제 STOP·PAUSE 를 처리한다. 하위 레이어 stop_command 는 보류. 토픽 방식 파일은 수정 안 함.
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
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
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
from navigation_command_msgs.action import NavigateCommand
from navigation_command_msgs.srv import NavigationControl
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


@dataclass
class _GoalCompletion:
    """Nav2 goal 하나의 완료를 기다리기 위한 레코드.

    [중요] 예전에는 단일 ``threading.Event`` (``_goal_finished_event``) 하나로
    이 일을 했다. 그런데 이제 그 신호를 기다리는 곳이 둘이다.
      1) 재라우팅 대기 - ``_move_callback`` 이 이전 goal 의 취소가 끝나기를 기다린다
      2) action execute - 자기 goal 이 완주/실패하기를 기다린다
    Event 가 하나면 두 대기자가 충돌한다. 재라우팅이 ``clear()`` 하는 순간
    action 이 기다리던 완료 신호가 삼켜지고, 반대로 남의 완료로 엉뚱하게 깨어난다.

    그래서 goal 을 보낼 때마다 레코드를 새로 만들고, 기다리는 쪽은 자기가 캡처한
    레코드만 본다. ``_move_result_callback`` 은 자기가 끝낸 goal 의 레코드에만
    결과를 쓰고 set 한다.
    """

    event: threading.Event = field(default_factory=threading.Event)
    status: int = GoalStatus.STATUS_UNKNOWN
    # NavigateCommand.Result 의 RESULT_* 중 하나. 관제에 "왜 끝났는지" 를 올린다.
    result_code: int = NavigateCommand.Result.RESULT_SUCCEEDED
    message: str = ''


# ---------------------------------------------------------------------- #
# Node
# ---------------------------------------------------------------------- #
class NavigationManagerNode(Node):
    # 재라우팅 시 이전 goal 의 취소가 끝나기를 기다리는 상한(초).
    # Nav2 의 취소는 BT 를 halt 하고 controller 를 정지시키는 데 보통 1초 안쪽이면
    # 끝난다. 이 시간을 넘기면 스택이 정상이 아니라고 보고 이번 명령을 포기한다.
    REROUTE_CANCEL_TIMEOUT = 10.0

    def __init__(self) -> None:
        super().__init__('navigation_manager_node')

        self._state_lock = threading.RLock()

        # [중요 변경] 데드락 방지를 위해 콜백 그룹을 분리했습니다.
        # _move_callback은 _cmd_cb_group에서 block 되며, 
        # 상태 업데이트와 stop명령은 _state_update_cb_group에서 독립적으로 스레드를 점유해 실행됩니다.
        self._cmd_cb_group = MutuallyExclusiveCallbackGroup()
        self._state_update_cb_group = MutuallyExclusiveCallbackGroup()
        self._pose_cb_group = ReentrantCallbackGroup()
        self._action_cb_group = ReentrantCallbackGroup()
        self._timer_cb_group = MutuallyExclusiveCallbackGroup()
        self._srv_cb_group = MutuallyExclusiveCallbackGroup()
        # [중요] 액션 서버 그룹은 반드시 Reentrant 여야 한다.
        # rclpy ActionServer 는 goal/cancel/result 요청을 내부 서비스로 처리하고
        # execute_callback 도 같은 그룹에서 돈다. MutuallyExclusive 를 주면
        # execute 가 목표 점유 대기(최대 150초)로 블록된 동안 cancel 요청이
        # 서비스되지 못한다 = 로봇이 정지 명령에 반응하지 않는다(안전 이슈).
        self._nav_action_cb_group = ReentrantCallbackGroup()

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
        self._move_in_progress: bool = False

        # [추가] 재라우팅(주행 중 새 move 수신) 처리용.
        # _reroute_in_flight 가 True 인 동안의 CANCELED 는 "주행 실패" 가 아니라
        # "새 명령을 위해 내가 일부러 취소한 것" 이다. 관제에 abort 로 보고하지 않는다.
        self._reroute_in_flight: bool = False
        # [변경] 이전 goal 의 완료를 기다리기 위한 레코드. 예전의 단일
        # _goal_finished_event 를 대체한다. 이유는 _GoalCompletion 의 주석 참조.
        # _run_move 가 send_goal_async 직전에 새로 만들고,
        # _move_result_callback 이 결과를 채워 set 한다.
        self._active_completion: Optional[_GoalCompletion] = None
        # 현재 실행 중인 NavigateCommand action goal 의 서버 핸들.
        # 토픽 경로로 들어온 move 에는 None 이다(그래서 피드백을 쏘지 않는다).
        self._active_action_goal = None
        # [B1 FIX 09-27] 주행 명령 세대 번호와 대기 단계 종료 신호.
        # action execute 는 Reentrant 그룹에서 동시에 돈다. 앞 goal A 가 목표 점유
        # 대기(최대 150초, _goal_handle 은 아직 None) 중에 새 goal B 가 오면, 예전
        # _overlap_gate 는 취소할 핸들이 없어 그냥 통과시켰다 -> A·B 대기 루프가 동시에
        # 돌고 둘 다 Nav2 로 goal 을 보낸다. A 의 결과 콜백이 _goal_handle 을 None 으로
        # 지워 이후 관제 STOP 이 NO_ACTIVE_GOAL 로 빠지고 로봇이 계속 달렸다.
        # 이제 게이트가 세대를 올리면 A 의 대기 루프는 RESULT_CANCELED_REROUTE 로 빠지고,
        # 게이트는 A 의 대기 단계가 실제로 끝날 때까지 기다린 뒤 B 를 진행시킨다.
        # (토픽 경로는 MutuallyExclusive 그룹이라 원래 줄을 섰다 - 동작 변화 없음)
        self._move_generation: int = 0
        self._wait_exit_event: Optional[threading.Event] = None
        # [B2 FIX 09-27, 관제 명령만] goal 발송 전(목표 점유 대기 단계)에는 _goal_handle 이 None 이라
        # 예전에는 관제 STOP/PAUSE 가 "no active goal" 로 버려지고, 대기가 끝나면 로봇이 그대로 출발했다
        # (sim 실측: FLECS 가 move 직후 0.2 s 에 보낸 "구간 앞 정지, 허가 대기" PAUSE 가 거부됨 → 허가 없이 교차로 진입 위험).
        #   - 관제 STOP  : 대기 중이면 즉시 끝낸다(_operator_stop_pending). 발송 직후 수락 응답 전이면 수락 즉시 취소한다(_cancel_on_accept).
        #   - 관제 PAUSE : 대기 중이면 보류 pause(_pending_pause). 출발 직전 래치를 풀지 않고 pause=true 로 goal 을 보낸다.
        #                  RESUME 이 풀고, 새 move 가 오면 버린다(기존 규칙).
        #   - 하위 레이어 stop_command(fleet/BT/stuck) 는 사용자 결정으로 보류 — 동작 변경 없음.
        # 토픽 방식(navigation_manager_node.py)은 검증본 그대로 둔다(수정 없음).
        self._operator_stop_pending: bool = False
        self._operator_stop_seq: Optional[int] = None
        self._cancel_on_accept: bool = False
        self._pending_pause: bool = False


        self.curr_x: float = 0.0
        self.curr_y: float = 0.0
        self.curr_z: float = 0.0
        self.curr_w: float = 0.0
        
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

        self._pose_tracked_subscription = self.create_subscription(
            PoseStamped, '/pose_tracked',
            self._pose_tracked_callback, 1, callback_group=self._pose_cb_group)

        
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

        # ----- 관제 명령 endpoint (동기) ------------------------------- #
        # winros_bridge 의 move_command / pause_command / resume_command /
        # /main_stop_command / /reset_command 다섯 토픽을 대체한다.
        # 토픽 경로는 이행 기간 동안 그대로 살려두고, bridge 쪽
        # use_command_services 파라미터로 전환한다.
        #
        # 장시간 명령(주행)은 action: accept/reject 가 곧 ack 이고, 취소와
        # 피드백이 프로토콜에 내장되어 있으며, 목표 점유 대기를 별도 워커에서
        # 돌릴 수 있다.
        self._navigate_action_server = ActionServer(
            self, NavigateCommand, 'navigate_command',
            goal_callback=self._nav_goal_callback,
            cancel_callback=self._nav_cancel_callback,
            execute_callback=self._nav_execute_callback,
            callback_group=self._nav_action_cb_group)

        # 순간 명령(pause/resume/stop/reset)은 service 하나로 묶고 command 를
        # 인자로 받는다. 대체 대상인 네 토픽 콜백과 같은 콜백 그룹을 쓰는 것이
        # 중요하다 - 그래야 직렬화 순서 의미가 바뀌지 않는다.
        self._nav_control_service = self.create_service(
            NavigationControl, 'navigation_control',
            self._nav_control_callback,
            callback_group=self._state_update_cb_group)

        # ----- Publishers --------------------------------------------- #
        self._monitoring_publisher = self.create_publisher(
            NavigationMonitoring, 'ros2_nav2_monitoring_data', 10)
        # [수정] nav_pause_flag 를 TRANSIENT_LOCAL 로 발행한다.
        #
        # 이 토픽은 pause/resume 이 일어난 "순간에만" 1회 발행된다. 기존처럼
        # VOLATILE 로 내보내면, 나중에 구독을 시작한 쪽은 현재 pause 상태를 알 방법이
        # 없다. moduler31 의 회복 pause 브랜치(ManeuverServerPause)가 이 토픽을
        # 봐야 하는데, amr_bt_nodes 의 CheckPauseCondition 은 항상 TRANSIENT_LOCAL 로
        # 구독하므로(= transient_local 포트가 durability 를 되돌리지 않는 결함이 있다)
        # VOLATILE 발행과는 QoS 가 맞지 않아 아예 수신되지 않는다.
        #
        # TRANSIENT_LOCAL 발행은 기존 구독자와도 호환된다
        # ("발행자가 제공하는 durability >= 구독자가 요구하는 durability").
        # controller_server 도 같은 이유로 TRANSIENT_LOCAL 구독으로 맞춰 두었다.
        # 그래야 컨트롤러가 재기동해도 직전 pause 상태를 그대로 이어받는다.
        #
        # depth 는 1 로 둔다. TRANSIENT_LOCAL 에서 depth 를 크게 잡으면 나중에 붙는
        # 구독자가 과거 샘플을 여러 개 돌려받는다. pause 플래그는 "현재 상태" 하나만
        # 의미가 있으므로 마지막 값만 유지하는 것이 맞다.
        # (fleet_decision_node 가 /controller_pause_flag 를 발행할 때 쓰는 프로파일과 같다)
        qos_nav_pause = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pause_resume_publisher = self.create_publisher(
            Bool, 'nav_pause_flag', qos_nav_pause)
        # [09-29 관제 우선] 마지막으로 발행한 /nav_pause_flag 값 = 관제 pause 중인지.
        # 관제 pause 동안에는 하위 레이어의 내부 정지(/stop_command: BT 405 경보·fleet·nav_stuck)가
        # goal 을 취소하지 못하게 _nav_stop_callback 이 이 값을 본다.
        self._nav_pause_latched: bool = False
        # [10-07 Q51 (다)] 관제 pause 중에는 Windows 로 올리는 pause·장애물 비트를 0 으로 둔다 (_timer_callback 참고).
        self.declare_parameter('mask_bits_during_operator_pause', True)
        self._mask_bits_in_op_pause: bool = bool(self.get_parameter('mask_bits_during_operator_pause').value)
        # self._pause_resume_publisher = self.create_publisher(
        #     Bool, '/controller_pause_flag', qos_pause)


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

            # [10-07 Q51 (다)] 관제 pause (/nav_pause_flag 래치) 중에는 정지 원인이 관제다 — 하위 레이어의 pause·장애물 비트를 0 으로 올린다.
            #   현장 Windows (DrivingControl.cpp executeGoTarget 972-1001) 는 TP resume 뒤 ros_nav_pause·ros_nav_obstacle_detected 가
            #   둘 다 꺼져야 8018 (resume) 을 보낸다. 관제 pause 중에는 fleet 판단·BT replan 이 멈춰 이 비트를 스스로 끌 수 없어,
            #   장애물 앞에서 관제 pause → resume 하면 영구 교착 (sim op_q45b_obs_then_pause_field 10-07: 장애물을 치워도 fleet pause 가 남아 안 풀림).
            #   0 으로 올리면 관제 resume 즉시 8018 → ROS 관제 pause 해제 → fleet·BT 가 다시 판단 (장애물이 남아 있으면 비트는 다시 켜진다).
            #   관제 pause 중 Windows 상태는 DrivingPause (15) 가 비트보다 먼저라 표시는 그대로다. 흐름제어 대기 (17) 는 비트를 보지 않는다.
            op_hold = self._mask_bits_in_op_pause and self._nav_pause_latched
            pause_bit = bool(self._controller_pause_flag or self._path_agent_collision)
            obs_bit = bool(self._path_static_collision)
            self._nav2_monitoring_data.ros_nav_pause = pause_bit and not op_hold
            self._nav2_monitoring_data.ros_nav_obstacle_detected = obs_bit and not op_hold

            self.get_logger().info(
                f"int(self._controller_pause_flag)/int(self._path_agent_collision)/int(self._path_static_collision): {int(self._controller_pause_flag)}/{int(self._path_agent_collision)}/{int(self._path_static_collision)}"
                + (" (관제 pause 중 — Windows 로는 0/0 으로 올림, Q51)" if op_hold and (pause_bit or obs_bit) else ""),
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


    def _pose_tracked_callback(self, msg: PoseStamped) -> None:
        # self.get_logger().info(f'reset_callback!, cmd_seq_num: {msg.data}')
        
        with self._state_lock:
            self.curr_x = round(msg.pose.position.x, 4)
            self.curr_y = round(msg.pose.position.y, 4)
            self.curr_z = round(msg.pose.orientation.z, 4)
            self.curr_w = round(msg.pose.orientation.w, 4)
   
        # self.get_logger().info(f'reset abort status, reset_callback')

    
    # ------------------------------------------------------------------ #
    # Topic callbacks
    # ------------------------------------------------------------------ #
    def _nav_stop_callback(self, msg: UInt8) -> None:
        self.get_logger().info(f'nav stop_callback!, cmd_seq_num: {msg.data}')
        
        with self._state_lock:
            # [09-29 관제 우선] 관제 pause 중에는 하위 레이어의 내부 정지로 goal 을 취소하지 않는다.
            # 관제의 pause/resume/stop 이 우선이고 하위 레이어는 그대로 따른다 (사용자 원칙).
            # (sim 실측 L5_dense r3 03:50:41: 교차로 flow control pause 중 BT 의 403 ping-pong 경보가
            #  /stop_command 2 를 내 goal 이 CANCELED 됐다.) 원인이 남아 있으면 resume 뒤 그 출처가 다시 판단한다.
            if self._nav_pause_latched:
                self.get_logger().warn(
                    f'[관제 우선] 관제 pause 중 내부 정지(/stop_command={msg.data}) 무시 — goal 유지')
                return
            # while 문 중단을 위한 플래그 설정
            self.nav_stop_command = True
            self._stop_in_flight = True
            self._nav2_monitoring_data.ros_nav_driving_abort = False
            
            if self._goal_handle is None:
                self.get_logger().info('not cancle goal, nav stop_callback')
                self.nav_stop_command = False
                self._stop_in_flight = False
                return
            handle = self._goal_handle
            # [FIX] msg.data 를 cmd_seq_num 에 넣으면 안 된다.
            #
            # /stop_command 는 관제가 쓰는 토픽이 아니다. 관제의 정지는
            # winros_bridge 가 /main_stop_command 로 넣어주고(_main_stop_callback),
            # 이 토픽은 로봇 내부 전용이다. 실제 발행자는 둘뿐이다.
            #   - 동작 트리의 405 에스컬레이션      -> 상수 2
            #   - fleet_decision 의 goal 점유 타임아웃 -> 상수 1
            # 그 상수를 cmd_seq_num 에 넣으면 관제가 발급한 시퀀스가 1 또는 2 로
            # 덮어써지고, _move_result_callback 의 CANCELED 처리가 그 값을
            # ros_nav_cmd_seq_num 으로 올려버린다. 관제는 자기가 예전에 보냈던
            # 1번/2번 명령이 끝난 것으로 오인한다.
            # (sim 실측: 관제가 seq=77 로 보낸 주행이 내부 정지 후 seq=2 로 보고됨)
            #
            # 진행 중이던 관제 시퀀스를 그대로 유지한다. 그러면 뒤이어 올라가는
            # driving_abort=True 가 "당신이 보낸 77번 명령이 실패했다" 는 뜻이 되어
            # 비로소 관제가 쓸 수 있는 정보가 된다.
            current_cmd_seq = self._nav2_cmd_data.cmd_seq_num
            self._clear_nav2_command_data_locked()
            self._nav2_cmd_data.cmd_seq_num = current_cmd_seq
            self._goal_status = GoalStatus.STATUS_CANCELED
            self._controller_pause_flag = False                       ### testing...
            self._path_static_collision = False                       ### testing...
            self._path_agent_collision = False                        ### testing...

        handle.cancel_goal_async()
        self.get_logger().info(
            f'cancle goal, nav stop_callback (내부 정지 값={msg.data}, '
            f'관제 seq={current_cmd_seq} 유지)')

    def _request_operator_stop(self, cmd_seq_num: int):
        """관제 정지의 실제 처리. 진입점 셋이 이 함수를 공유한다.
          - /main_stop_command 토픽 (레거시 경로)
          - NavigationControl service 의 COMMAND_STOP
          - NavigateCommand action 의 cancel 요청

        [중요] _nav_stop_callback(내부 정지)과 달리 self.nav_stop_command 를
        세우지 않는다. 관제의 정지는 주행 "실패" 가 아니므로 뒤이은 CANCELED 에
        driving_abort 를 올리지 않는다. 이 구분이 세 진입점에서 동일해야 하기
        때문에 함수 하나로 모았다.
        """
        R = NavigationControl.Response
        with self._state_lock:
            # while 문 중단을 위한 플래그 설정
            self._nav2_monitoring_data.ros_nav_driving_abort = False
            self._stop_in_flight = True

            if self._goal_handle is None:
                # [B2 FIX] (a) goal 을 막 보냈고 수락 응답을 기다리는 중: 수락되는 즉시 취소한다.
                pending = self._active_completion
                if pending is not None and not pending.event.is_set():
                    self._cancel_on_accept = True
                    self._pending_pause = False
                    self._clear_nav2_command_data_locked()
                    self._nav2_cmd_data.cmd_seq_num = cmd_seq_num
                    self._goal_status = GoalStatus.STATUS_CANCELED
                    self.get_logger().warn(
                        'operator stop before nav2 accepted the goal -> cancel on accept')
                    return True, R.RESULT_OK, 'stopping (cancel on accept)'
                # [B2 FIX] (b) 목표 점유 대기 단계: 대기 루프가 다음 주기(0.1 s)에 끝낸다. goal 은 보내지 않는다.
                if self._wait_exit_event is not None or self._move_in_progress:
                    # 위에서 세운 _stop_in_flight 는 "주행 중 취소" 용이다. 대기 단계는 보류 플래그로만 처리하고
                    # 이 값은 되돌린다 (남겨 두면 대기 루프의 옛 검사가 먼저 걸려 정지 완료 통지·seq 갱신이 빠진다).
                    self._stop_in_flight = False
                    self._operator_stop_pending = True
                    self._operator_stop_seq = cmd_seq_num
                    self._pending_pause = False
                    self.get_logger().warn('operator stop during wait phase -> move canceled before dispatch')
                    return True, R.RESULT_OK, 'stopping (wait phase)'
                self.get_logger().info('not cancle goal, main stop_callback')
                self._stop_in_flight = False
                return False, R.RESULT_NO_ACTIVE_GOAL, 'no active goal to stop'
            handle = self._goal_handle
            self._pending_pause = False
            self._clear_nav2_command_data_locked()
            self._nav2_cmd_data.cmd_seq_num = cmd_seq_num
            self._goal_status = GoalStatus.STATUS_CANCELED
            self._controller_pause_flag = False                       ### testing...
            self._path_static_collision = False                       ### testing...
            self._path_agent_collision = False                        ### testing...

        handle.cancel_goal_async()
        self.get_logger().info(f'cancle goal, main stop_callback')
        return True, R.RESULT_OK, 'stopping'

    def _do_reset(self, cmd_seq_num: int):
        """abort 상태 해제. 토픽과 service 가 공유한다."""
        R = NavigationControl.Response
        self._reset_abort_status_locked_body(cmd_seq_num)
        return True, R.RESULT_OK, 'abort status reset'

    def _main_stop_callback(self, msg: UInt8) -> None:
        self.get_logger().info(f'main_stop_callback!, cmd_seq_num: {msg.data}')
        self._request_operator_stop(msg.data)

    def _reset_callback(self, msg: UInt8) -> None:
        self.get_logger().info(f'reset_callback!, cmd_seq_num: {msg.data}')
        self._do_reset(msg.data)

    def _reset_abort_status_locked_body(self, cmd_seq_num: int) -> None:
        with self._state_lock:
            self._nav2_monitoring_data.ros_nav_driving_abort = False
            # [FIX] _goal_status가 STATUS_ABORTED로 남아 있으면 _timer_callback의
            # _update_nav2_status()가 매 tick마다 ros_nav_driving_abort를 다시 True로
            # 세워서, 위에서 지운 값이 publish 되기 전에 덮어써진다.
            # (_nav_stop_callback / _main_stop_callback이 STATUS_CANCELED를 넣는 것과 동일한 처리)
            #
            # [주의] 반드시 STATUS_ABORTED 일 때만 바꾼다. 조건 없이 CANCELED 를 넣으면
            # 주행 중에 reset 이 들어왔을 때 다음 두 가지가 같이 망가진다.
            #   1) _update_nav2_status(CANCELED) 가 ros_nav_driving 을 False 로 만든다.
            #   2) _move_feedback_callback 이 _goal_status in (SUCCEEDED/ABORTED/CANCELED)
            #      에서 조기 return 하므로 STATUS_EXECUTING 으로 되돌아오지 못하고,
            #      current_node_id / distance_remaining / poses_remaining 갱신도 멈춘다.
            # 그러면 로봇은 계속 주행하는데 상위 서버에는 "정지" 로 보인다.
            # (sim 실측: 주행 중 reset 후 6초간 cmd_vel 은 100% 비영인데
            #  ros_nav_driving 은 55tick 전부 False. 조건을 달면 55/55 True 로 정상.)
            if self._goal_status == GoalStatus.STATUS_ABORTED:
                self._goal_status = GoalStatus.STATUS_CANCELED
   
        self.get_logger().info(f'reset abort status, reset_callback')

    

    def _move_callback(self, msg: NavigationCommand) -> None:
        self.get_logger().info('move_callback')
        self.get_logger().info(f'goal_cnt: {msg.goal_cnt}, cmd_seq_num: {msg.cmd_seq_num}, from_node_id: {msg.from_node_id}, to_node_id: {msg.to_node_id}')

        # [주의] 예전엔 여기서 /nav_pause_flag=false 를 쐈다(주석 처리되어 있었다).
        # 이 자리는 아직 재라우팅 취소 대기도, 목표 점유 대기(최대 150초)도 시작하기
        # 전이라 pause 해제와 실제 주행 시작 사이가 너무 벌어진다.
        # 지금은 _run_move() 의 send_goal_async 직전으로 옮겼다.


        # [겹침 처리] 이전 move 가 아직 살아 있으면(action 진행 중) 재라우팅으로 본다.
        #
        # [FIX] 예전에는 여기서 이전 goal 을 취소하고 abort 를 세운 뒤 이번 명령을
        # 그냥 버렸다(return). 그래서 관제는 주행 중인 로봇의 경로를 바꿀 수 없었고,
        # 정상적인 재라우팅을 보내도 로봇이 서면서 driving_abort 가 올라왔다.
        # 게다가 _clear_nav2_command_data_locked() 가 cmd_seq_num 을 0 으로 만들어
        # 관제가 발급한 적 없는 seq=0 이 보고됐다.
        # (sim 실측: 주행 중 move seq=20 전송 -> seq=0 / abort=True / 명령 소실)
        #
        # 이제는 이전 goal 을 취소하고 그 취소가 실제로 끝날 때까지 기다린 뒤
        # 이번 명령을 그대로 수행한다.
        # [FIX] _move_in_progress 해제를 함수 전체를 감싸는 try/finally 로 올렸다.
        # 예전에는 _overlap_gate 안의 cancel_goal_async() 가 예외를 던지면
        # 그 예외가 아래 try/finally 에 닿기 전에 _move_callback 을 탈출해서
        # _move_in_progress 가 영구히 True 로 남았다. 그러면 이후의 모든 move 가
        # "겹침" 으로 오인되어 매번 불필요한 재라우팅 대기를 타게 된다.
        try:
            if not self._overlap_gate(msg.cmd_seq_num):
                return
            self._run_move(msg)
        finally:
            with self._state_lock:
                self._move_in_progress = False

    def _overlap_gate(self, cmd_seq_num: int) -> bool:
        """이전 move 가 살아 있으면 취소하고 그 취소가 끝날 때까지 기다린다.

        토픽 경로(_move_callback)와 action execute 가 공유한다.
        반환값 True 면 이번 명령을 진행해도 된다. False 면 포기해야 한다
        (이전 goal 의 취소가 제한 시간 안에 끝나지 않은 경우).

        성공 시 _move_in_progress 를 True 로 세워 둔다. 해제는 호출자의
        finally 가 책임진다.
        """
        with self._state_lock:
            # [B1 FIX] 매 진입마다 세대를 올린다. 대기 중인 앞 move 는 이 값이 바뀐 것을
            # 보고 스스로 빠진다(_run_move 의 대기 루프·발송 직전 검사).
            self._move_generation += 1
            prev_wait_exit = self._wait_exit_event
            # [B2 FIX] 새 주행 명령은 이전 명령에 걸려 있던 보류 pause·보류 정지를 무효로 한다.
            self._pending_pause = False
            self._operator_stop_pending = False
            self._operator_stop_seq = None
            self._cancel_on_accept = False
            overlapped = (
                self._goal_handle is not None
                or self._stop_in_flight
                or self._move_in_progress
            )
            handle = self._goal_handle
            # [변경] 예전에는 공용 _goal_finished_event 를 clear() 했다. 이제는
            # 이전 goal 의 레코드를 그대로 캡처한다. clear() 가 없으므로
            # 같은 시각 완주를 기다리던 action execute 의 신호를 삼키지 않는다.
            prev_completion = self._active_completion
            if overlapped:
                if handle is not None:
                    # 이 취소는 "실패" 가 아니라 재라우팅을 위한 것이다.
                    self._reroute_in_flight = True
            else:
                # 정상 진입: 이번 move가 점유 시작
                self._move_in_progress = True
                self._stop_in_flight = False

        if not overlapped:
            return True

        self.get_logger().warn(
            'Overlapping move_command received. Cancelling current goal '
            'and re-routing to the new goals.')

        # [B1 FIX] (1) 앞 move 가 목표 점유 대기 단계에 있으면 그 루프가 빠져나올 때까지
        # 기다린다. 세대가 이미 바뀌었으므로 다음 검사(0.1 s 주기)에서 빠진다.
        if prev_wait_exit is not None:
            if not prev_wait_exit.wait(timeout=self.REROUTE_CANCEL_TIMEOUT):
                with self._state_lock:
                    self._goal_status = GoalStatus.STATUS_ABORTED
                    self._nav2_monitoring_data.ros_nav_driving_abort = True
                self.get_logger().error(
                    f'Previous move did not leave its wait phase within '
                    f'{self.REROUTE_CANCEL_TIMEOUT}s. Dropping this move_command '
                    f'(cmd_seq_num={cmd_seq_num}).')
                return False
            with self._state_lock:
                handle = self._goal_handle
                prev_completion = self._active_completion

        # [B1 FIX] (2) 앞 move 가 goal 을 막 보냈고 응답(_move_response_callback)을
        # 아직 못 받은 경우: 핸들이 생길 때까지(또는 거부·종료될 때까지) 기다렸다가
        # 아래의 정상 취소 경로를 탄다. 핸들 없이 지나가면 Nav2 에 goal 이 둘 살아
        # 앞 goal 의 결과 콜백이 새 goal 의 상태를 지운다.
        if handle is None and prev_completion is not None and not prev_completion.event.is_set():
            deadline = time.monotonic() + self.REROUTE_CANCEL_TIMEOUT
            while time.monotonic() < deadline and not prev_completion.event.is_set():
                with self._state_lock:
                    handle = self._goal_handle
                if handle is not None:
                    break
                time.sleep(0.05)
            if handle is None and not prev_completion.event.is_set():
                with self._state_lock:
                    self._goal_status = GoalStatus.STATUS_ABORTED
                    self._nav2_monitoring_data.ros_nav_driving_abort = True
                self.get_logger().error(
                    f'Previous goal response did not arrive within '
                    f'{self.REROUTE_CANCEL_TIMEOUT}s. Dropping this move_command '
                    f'(cmd_seq_num={cmd_seq_num}).')
                return False
            if handle is not None:
                with self._state_lock:
                    self._reroute_in_flight = True

        if handle is not None:
            # [주의] _reroute_in_flight 는 어떤 경로로 빠져나가든 반드시 내려야 한다.
            # 켜진 채로 남으면 이후의 모든 CANCELED 가 "재라우팅" 으로 오인되어
            # 내부 정지의 driving_abort 와 cmd_seq_num 갱신이 영구히 사라진다.
            # cancel_goal_async() 는 액션 클라이언트가 정리 중이면 예외를 던질 수 있다.
            finished = False
            try:
                handle.cancel_goal_async()
                # 취소가 끝나야 새 goal 을 보낼 수 있다. 액션 콜백은 다른 콜백
                # 그룹(_action_cb_group, Reentrant)에서 돌아가므로 여기서 블록해도
                # _move_result_callback 은 정상적으로 실행된다.
                if prev_completion is None:
                    # goal_handle 은 있는데 레코드가 없다 = 이 노드가 보내지 않은
                    # goal 이거나 이미 정리된 상태다. 기다릴 대상이 없으므로 통과.
                    finished = True
                else:
                    finished = prev_completion.event.wait(
                        timeout=self.REROUTE_CANCEL_TIMEOUT)
            finally:
                with self._state_lock:
                    self._reroute_in_flight = False
            if not finished:
                # 취소가 제한 시간 안에 끝나지 않았다. 이 상태로 새 goal 을
                # 보내면 어느 goal 이 사는지 알 수 없으므로 포기하고, 그 사실을
                # 관제가 발급한 시퀀스와 함께 실패로 올린다(seq 는 건드리지 않는다).
                with self._state_lock:
                    self._goal_status = GoalStatus.STATUS_ABORTED
                    self._nav2_monitoring_data.ros_nav_driving_abort = True
                self.get_logger().error(
                    f'Cancel of the previous goal did not finish within '
                    f'{self.REROUTE_CANCEL_TIMEOUT}s. Dropping this move_command '
                    f'(cmd_seq_num={cmd_seq_num}).')
                return False

        with self._state_lock:
            self._move_in_progress = True
            self._stop_in_flight = False
            self.nav_stop_command = False
        return True



    def _run_move(self, msg: NavigationCommand, goal_handle=None):
        """Nav2 로 주행 goal 을 보낸다.

        [반환 계약 - 변경됨]
        예전에는 항상 None 을 반환했다. 그런데 이 함수는 fire-and-forget 이다:
        send_goal_async 직후 리턴하고 완주 여부는 _move_result_callback 이
        비동기로 받는다. 게다가 goal 을 아예 보내지 않고 리턴하는 경로가 5개나
        있는데 전부 None 이라 호출자가 구분할 수 없었다.
        action execute 가 결과를 보고해야 하므로 이제 구분해서 반환한다.

          - goal 을 보내지 못한 경우 -> NavigateCommand.Result.RESULT_* (int)
          - goal 을 보낸 경우        -> _GoalCompletion 레코드
                                       (호출자가 event 를 기다려 완주를 확인)

        토픽 경로(_move_callback)는 반환값을 무시한다 = 기존 동작 그대로.

        goal_handle 이 주어지면(action 경로) 목표 점유 대기 루프에서
        PHASE_WAITING_READY 피드백을 함께 발행한다.
        """
        RC = NavigateCommand.Result
        cond_ready = False
        cond_static = False
        cond_agent = False

        self.clear_both_costmaps()

        if not msg.goal_poses:
            self.get_logger().warn('Received empty multi goal list')
            return RC.RESULT_REJECTED_EMPTY

        if not (len(msg.goal_poses) == msg.goal_cnt
                == len(msg.from_node_id) == len(msg.to_node_id)):
            self.get_logger().error(
                f'inconsistent NavigationCommand sizes: '
                f'goal_cnt={msg.goal_cnt}, '
                f'goal_poses={len(msg.goal_poses)}, '
                f'from_node_id={len(msg.from_node_id)}, '
                f'to_node_id={len(msg.to_node_id)}')
            return RC.RESULT_REJECTED_SIZE_MISMATCH

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = []

        with self._state_lock:
            self._clear_nav2_command_data_locked()
            self._stop_in_flight = False
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


            self.get_logger().info(f'current pose: x: {self.curr_x}, y: {self.curr_y}, z: {self.curr_z}, w: {self.curr_w}')
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

        # [B1 FIX] 이 move 의 세대와 대기 단계 종료 신호. 신호는 대기 루프를 떠나
        # goal 발송 여부가 확정된 뒤(아래 finally) set 된다.
        wait_exit = threading.Event()
        with self._state_lock:
            my_generation = self._move_generation
            self._wait_exit_event = wait_exit
        try:
            return self._run_move_wait_and_dispatch(
                msg, goal_handle, goal_msg, remaining_path_msg, my_generation)
        finally:
            with self._state_lock:
                if self._wait_exit_event is wait_exit:
                    self._wait_exit_event = None
            wait_exit.set()

    def _run_move_wait_and_dispatch(self, msg, goal_handle, goal_msg,
                                    remaining_path_msg, my_generation: int):
        """[B1 FIX] _run_move 의 대기 루프 + goal 발송 부분 (본문은 예전과 같다).
        세대 검사 두 곳만 추가했다: 대기 루프 매 주기, 발송 직전(원자적)."""
        RC = NavigateCommand.Result
        cond_ready = False
        cond_static = False
        cond_agent = False
        wait_start_time = None
        target_duration = 1.0  # 조건 충족 유지 시간 (초)
        loop_start_time = self.get_clock().now()
        MAX_WAIT_TIMEOUT = 150.0  # 최대 대기 시간 (초)
        TIMEOUT_PUB_N_SEC = 0.2  # 타임아웃 발생 시 메시지 퍼블리시 유지 시간 (초)
        is_timed_out = False          # 타임아웃 상태를 나타내는 플래그
        timeout_pub_start = None      # 타임아웃 퍼블리시 시작 시간

        while rclpy.ok():
            # 1. Break + Return 조건 (stop_command 수신 시 즉시 종료)
            with self._state_lock:
                if self._stop_in_flight:
                    self.get_logger().warn('Move aborted due to stop command during wait phase.')
                    return RC.RESULT_CANCELED_OPERATOR
                # [B2 FIX] 대기 중 관제 STOP → goal 을 보내지 않고 끝낸다.
                if self._operator_stop_pending:
                    self._consume_operator_stop_locked(msg.cmd_seq_num)
                    stop_now = True
                else:
                    stop_now = False
            if stop_now:
                self._publish_stop_complete()
                return RC.RESULT_CANCELED_OPERATOR
            with self._state_lock:
                # [B1 FIX] 새 move 가 들어왔다(세대 변경) -> 이 move 는 대체됐다.
                if self._move_generation != my_generation:
                    self.get_logger().warn(
                        f'Move (cmd_seq_num={msg.cmd_seq_num}) superseded by a newer '
                        f'command during wait phase.')
                    return RC.RESULT_CANCELED_REROUTE

            # [중요] 대기 단계의 action cancel 은 여기서 직접 봐야 한다.
            # _nav_execute_callback 의 is_cancel_requested 폴링은 _run_move 가
            # 리턴한 뒤에야 도달하므로, 이 검사가 없으면 취소가 최대 150초 동안
            # 무시된다. 그리고 _request_operator_stop 도 도움이 안 된다 -
            # 대기 단계에는 _goal_handle 이 None 이라 _stop_in_flight 를 세우고
            # 곧바로 되돌린 뒤 RESULT_NO_ACTIVE_GOAL 로 빠져나오기 때문이다.
            if goal_handle is not None and goal_handle.is_cancel_requested:
                self.get_logger().warn(
                    'Move canceled by operator during wait phase.')
                return RC.RESULT_CANCELED_OPERATOR

            # action 경로면 관제에 "목표 점유로 대기 중" 을 알린다.
            # 이 정보는 지금까지 관제가 볼 수 없었다(최대 150초 동안 무소식).
            if goal_handle is not None:
                self._publish_wait_feedback(
                    goal_handle, msg.cmd_seq_num,
                    (self.get_clock().now() - loop_start_time).nanoseconds / 1e9)

            # --------------------------------------------------------------
            # [상태 A] 타임아웃 발생 이후: N초 동안 빈 Goal과 IDLE 퍼블리시
            # --------------------------------------------------------------
            if is_timed_out:
                pub_elapsed = (self.get_clock().now() - timeout_pub_start).nanoseconds / 1e9
                if pub_elapsed > TIMEOUT_PUB_N_SEC:
                    # N초 퍼블리시가 끝났으므로 함수 자체를 종료 (Goal 전송 X)
                    # [FIX] 이 쓰기가 _state_lock 밖에 있어서 10Hz _timer_callback 의
                    # deepcopy 와 경합했다.
                    with self._state_lock:
                        self._nav2_monitoring_data.ros_nav_driving_abort = True

                    return RC.RESULT_READY_TIMEOUT

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
            return RC.RESULT_NAV2_UNAVAILABLE

        # [FIX] goal 을 보내기 직전에 pause 래치를 반드시 내린다.
        #
        # controller_server 의 pause_flag_ 는 /nav_pause_flag 콜백에서만 쓰이고
        # (controller_server.cpp:709 가 유일한 쓰기), goal 시작/종료/취소 어디서도
        # 초기화되지 않는다. 그리고 false 를 내보내는 곳은 관제 RESUME 하나뿐이었다.
        # BT 의 InitSequence 가 goal 마다 내보내는 것은 /controller_pause_flag 로
        # 토픽이 다르므로 컨트롤러의 pause 를 풀지 못한다.
        #
        # 그래서 'pause -> 관제 stop -> goal 취소 -> 새 move' 처럼 RESUME 없이
        # pause 가 남은 채 다음 명령이 오면, 컨트롤러가 computeControl 루프의
        # pause 분기에 걸려 0 속도만 계속 발행한다(= 로봇이 영영 안 움직인다).
        #
        # 새 move 는 관제의 새 지시이므로 직전 pause 는 그 지시로 무효가 된다.
        # 여기(대기 루프와 wait_for_server 를 모두 통과한 뒤, send_goal_async 바로 앞)
        # 에 두어야 pause 해제와 실제 주행 시작 사이에 창이 생기지 않는다.
        #
        # /nav_pause_flag 는 TRANSIENT_LOCAL 이라 이 false 가 래치를 덮어쓴다.
        # 즉 이후 controller_server 가 재기동해도 옛 pause 로 되살아나지 않는다.
        # [B2 FIX] 대기 중 관제 PAUSE 가 있었으면 래치를 내리지 않고 pause=true 로 보낸다(컨트롤러는 멈춘 채
        # goal 을 받고 RESUME 을 기다린다). 판단·발행을 한 잠금 안에서 해 PAUSE/RESUME 발행과 순서가 섞이지 않게 한다.
        with self._state_lock:
            held_pause = self._pending_pause
            self._publish_nav_pause(held_pause)
        if held_pause:
            self.get_logger().warn('dispatching goal with held pause (waiting for RESUME)')

        # 이 goal 의 완료를 기다릴 레코드를 만든다. send_goal_async 보다 먼저
        # 만들어야 _move_result_callback 이 레코드 없는 상태로 도착하지 않는다.
        completion = _GoalCompletion()
        with self._state_lock:
            # [B1 FIX] 발송 여부를 세대 검사와 같은 잠금 안에서 확정한다. 게이트가
            # 세대를 올린 뒤라면 보내지 않는다. 게이트보다 먼저라면 게이트가 이
            # completion 을 보고 응답을 기다렸다가 정상 취소한다.
            if self._move_generation != my_generation:
                self.get_logger().warn(
                    f'Move (cmd_seq_num={msg.cmd_seq_num}) superseded right before '
                    f'dispatch - not sending.')
                return RC.RESULT_CANCELED_REROUTE
            # [B2 FIX] 발송 직전에 관제 STOP 이 와 있었으면 보내지 않는다.
            stop_now = self._operator_stop_pending
            if stop_now:
                self._consume_operator_stop_locked(msg.cmd_seq_num)
            else:
                self._active_completion = completion
        if stop_now:
            self._publish_stop_complete()
            return RC.RESULT_CANCELED_OPERATOR

        self.get_logger().info('Request sending NavigateThroughPoses goal')
        send_goal_future = self._nav2_through_poses_client.send_goal_async(
            goal_msg, feedback_callback=self._move_feedback_callback)
        send_goal_future.add_done_callback(self._move_response_callback)
        return completion

    def _consume_operator_stop_locked(self, move_seq: int) -> None:
        """[B2 FIX] 대기 단계에서 관제 STOP 을 소비한다 (_state_lock 안에서 부른다)."""
        stop_seq = self._operator_stop_seq
        self._operator_stop_pending = False
        self._operator_stop_seq = None
        self._pending_pause = False
        self._stop_in_flight = False
        self._goal_status = GoalStatus.STATUS_CANCELED
        self._nav2_monitoring_data.ros_nav_driving_abort = False    # 관제 정지는 실패가 아니다
        if stop_seq is not None:
            self._nav2_cmd_data.cmd_seq_num = stop_seq
            self._nav2_monitoring_data.ros_nav_cmd_seq_num = stop_seq
        self.get_logger().warn(
            f'Move (cmd_seq_num={move_seq}) stopped by operator during wait phase - not dispatched.')

    def _publish_stop_complete(self) -> None:
        self._stop_complete_publisher.publish(Bool(data=True))
        self.get_logger().info('nav_stop_complete published')

    def _publish_wait_feedback(self, goal_handle, cmd_seq_num: int,
                               wait_elapsed_sec: float) -> None:
        """목표 점유 대기 중임을 action 피드백으로 알린다."""
        fb = NavigateCommand.Feedback()
        fb.phase = NavigateCommand.Feedback.PHASE_WAITING_READY
        fb.cmd_seq_num = cmd_seq_num
        fb.wait_elapsed_sec = float(wait_elapsed_sec)
        try:
            goal_handle.publish_feedback(fb)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'wait feedback publish failed: {exc}')




    def _do_pause(self, cmd_seq_num: int):
        """관제 PAUSE 의 실제 처리. 토픽과 service 가 공유한다.

        Nav2 goal 을 취소하지 않는다는 점이 중요하다. nav_pause_flag 래치만
        올리면 controller_server 가 0 속도를 내며 goal 은 살아있게 둔다.
        """
        R = NavigationControl.Response
        with self._state_lock:
            if self._goal_handle is None:
                # [B2 FIX] 대기 단계(또는 발송 직후 수락 전)면 보류 pause 로 기록하고 래치도 지금 올린다.
                # 출발 직전 단계가 이 값을 보고 래치를 내리지 않는다. 같은 잠금 안에서 기록·발행해
                # 출발 직전의 발행과 순서가 뒤바뀌지 않게 한다.
                if self._wait_exit_event is not None or self._move_in_progress:
                    self._pending_pause = True
                    self._nav2_cmd_data.cmd_seq_num = cmd_seq_num
                    self._nav2_monitoring_data.ros_nav_cmd_seq_num = cmd_seq_num
                    self._publish_nav_pause(True)
                    self.get_logger().warn('pause during wait phase -> held; goal will be sent paused')
                    return True, R.RESULT_OK, 'paused (held until resume)'
                # [변경] 예전에는 조용히 return 해서 관제가 자기 PAUSE 가
                # 버려진 것을 몰랐다. 이제는 거부 사유를 돌려준다.
                self.get_logger().warn('not cancle goal, pause_callback')
                return False, R.RESULT_NO_ACTIVE_GOAL, 'no active goal to pause'
            self._nav2_cmd_data.cmd_seq_num = cmd_seq_num
            self._nav2_monitoring_data.ros_nav_cmd_seq_num = cmd_seq_num

        self._publish_nav_pause(True)
        self.get_logger().info('pause flag published (FollowPath canceled in BT)')
        return True, R.RESULT_OK, 'paused'

    def _do_resume(self, cmd_seq_num: int):
        """관제 RESUME 의 실제 처리. 토픽과 service 가 공유한다.

        _do_pause 와 달리 goal_handle 가드가 없다 - 기존 동작 그대로다.
        pause 래치를 내리는 것은 goal 이 없어도 해가 없고, 오히려 남은 래치를
        지우는 효과가 있다(nav_pause_flag 는 TRANSIENT_LOCAL 이다).
        """
        R = NavigationControl.Response
        with self._state_lock:
            self._pending_pause = False                 # [B2 FIX] 보류 pause 해제
            self._nav2_cmd_data.cmd_seq_num = cmd_seq_num
            self._nav2_monitoring_data.ros_nav_cmd_seq_num = cmd_seq_num
            # [B2 FIX] 같은 잠금 안에서 발행한다 (출발 직전 발행과 순서 보장)
            self._publish_nav_pause(False)


        self.get_logger().info('resume_callback')
        return True, R.RESULT_OK, 'resumed'

    def _publish_nav_pause(self, value: bool) -> None:
        """[09-29 관제 우선] /nav_pause_flag 발행은 모두 여기를 거친다 — 관제 pause 상태를 함께 기억한다."""
        self._nav_pause_latched = bool(value)
        self._pause_resume_publisher.publish(Bool(data=bool(value)))

    def _pause_callback(self, msg: UInt8) -> None:
        self.get_logger().info('pause_callback')
        self._do_pause(msg.data)

    def _resume_callback(self, msg: UInt8) -> None:
        self.get_logger().info('resume_callback start')
        self._do_resume(msg.data)

    # ------------------------------------------------------------------ #
    # Action callbacks
    # ------------------------------------------------------------------ #
    def _move_response_callback(self, future) -> None:
        goal_handle: Optional[ClientGoalHandle] = future.result()

        rejected_completion = None
        cancel_now = False
        with self._state_lock:
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().warn('Rejected goal')
                self._goal_status = GoalStatus.STATUS_CANCELED
                self._goal_handle = None
                # [중요] Nav2 가 goal 을 거부하면 get_result_async 를 걸지 않으므로
                # _move_result_callback 이 영영 오지 않는다. 그런데 _run_move 는
                # 이미 완료 레코드를 만들어 두었다. 여기서 깨워주지 않으면
                # action execute 의 대기 루프가 무한히 돌고(취소도 안 옴)
                # 관제는 결과를 영영 못 받으며 executor 스레드 하나가 영구 점유된다.
                rejected_completion = self._active_completion
                self._active_completion = None
            else:
                self._goal_handle = goal_handle
                # [B2 FIX] 수락 응답 전에 관제 STOP 이 왔으면 지금 취소한다.
                # (_stop_in_flight 는 _request_operator_stop 이 세워 두었다 → 결과는 CANCELED_OPERATOR, 정지 완료 통지)
                cancel_now = self._cancel_on_accept
                self._cancel_on_accept = False

        if rejected_completion is None and goal_handle is not None and goal_handle.accepted and cancel_now:
            self.get_logger().warn('canceling goal right after accept (operator stop arrived earlier)')
            goal_handle.cancel_goal_async()

        if rejected_completion is not None:
            rejected_completion.status = GoalStatus.STATUS_CANCELED
            rejected_completion.result_code = \
                NavigateCommand.Result.RESULT_NAV2_UNAVAILABLE
            rejected_completion.message = 'nav2 rejected the goal'
            rejected_completion.event.set()
            return
        if goal_handle is None or not goal_handle.accepted:
            return

        with self._state_lock:
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
            action_goal = self._active_action_goal
            cmd_seq = self._nav2_cmd_data.cmd_seq_num

        self.get_logger().info(
            f'Distance_remaining: {distance_remaining:.2f} m, '
            f'Goal: {number_of_poses_remaining}, '
            f'c_id:{current_id}, n_id:{next_id}',
            throttle_duration_sec=1.0)

        # action 경로면 같은 값을 action 피드백으로도 올린다. 이 함수가 이미
        # 필요한 값을 전부 계산해 두었으므로 중복 계산이 없다.
        if action_goal is not None:
            fb = NavigateCommand.Feedback()
            fb.phase = NavigateCommand.Feedback.PHASE_DRIVING
            fb.cmd_seq_num = cmd_seq
            fb.current_node_id = current_id
            fb.next_node_id = next_id
            fb.number_of_poses_remaining = number_of_poses_remaining
            fb.distance_remaining = distance_remaining
            try:
                action_goal.publish_feedback(fb)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'drive feedback publish failed: {exc}')

    def _move_result_callback(self, future) -> None:
        result = future.result()
        status = result.status

        publish_stop_complete = False
        RC = NavigateCommand.Result
        # 이 주행이 어떻게 끝났는지. action result 의 result_code 로 올라가서
        # 관제가 처음으로 "왜 멈췄는지" 를 직접 알 수 있게 된다.
        result_code = RC.RESULT_SUCCEEDED
        result_message = ''

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
                result_code = RC.RESULT_SUCCEEDED
                result_message = 'destination reached'

            elif status == GoalStatus.STATUS_ABORTED:
                self._goal_status = GoalStatus.STATUS_ABORTED
                self.get_logger().error('ABORTED')
                result_code = RC.RESULT_ABORTED
                result_message = 'nav2 aborted the drive'

            elif status == GoalStatus.STATUS_CANCELED:
                self._goal_status = GoalStatus.STATUS_CANCELED
                self.get_logger().warn('CANCELED')

                # 취소는 세 가지 의미가 있고 관제 입장에서 전혀 다르다.
                if self._reroute_in_flight:
                    result_code = RC.RESULT_CANCELED_REROUTE
                    result_message = 'superseded by a newer command'
                elif self.nav_stop_command:
                    # 내부 정지(BT 405 / fleet_decision / nav_stuck_manager)
                    result_code = RC.RESULT_CANCELED_INTERNAL
                    result_message = 'stopped by internal stop_command'
                else:
                    # 관제 정지(service COMMAND_STOP / action cancel /
                    # /main_stop_command). 실패가 아니다.
                    result_code = RC.RESULT_CANCELED_OPERATOR
                    result_message = 'stopped by operator'

                if self._reroute_in_flight:
                    # [FIX] 재라우팅을 위해 _move_callback 이 일부러 취소한 것이다.
                    # 관제 입장에서 주행이 실패한 것이 아니므로 driving_abort 를
                    # 세우지 않고, cmd_seq_num 도 손대지 않는다. 곧바로 새 goal 이
                    # 나가면서 _move_response_callback 이 새 시퀀스로 갱신한다.
                    self.get_logger().info('  (재라우팅을 위한 취소 - 실패 아님)')
                else:
                    self._nav2_monitoring_data.ros_nav_cmd_seq_num = \
                        self._nav2_cmd_data.cmd_seq_num

                    if self.nav_stop_command:
                        self.nav_stop_command = False
                        self._nav2_monitoring_data.ros_nav_driving_abort = True       #### testing...

            else:
                self._goal_status = GoalStatus.STATUS_UNKNOWN
                self.get_logger().error(f'Unknown result status: {status}')

            self._goal_handle = None

            # [FIX] 정지 완료 통지는 goal 의 종료 상태와 무관하게 반드시 내보낸다.
            #
            # 예전에는 이 블록이 CANCELED 분기 안에 있었다. 그래서 두 경우에 유실됐다.
            #   1) 재라우팅: 정지 진행 중(_stop_in_flight)에 새 move 가 들어와
            #      위에서 재라우팅으로 처리되는 경로.
            #   2) 경합: _nav_stop_callback / _main_stop_callback 이
            #      cancel_goal_async() 를 부른 뒤 서버가 취소를 처리하기 전에
            #      goal 이 스스로 SUCCEEDED / ABORTED 로 끝나는 경로.
            #
            # 통지가 유실되면 fleet_decision 은 nav_stop_complete_ 가 False 로
            # 영구 고착되고, 그 동안 check_collision_obstacle /
            # check_collision_agent / on_collision 을 전부 조기 return 시킨다.
            # 즉 그 로봇은 재기동 전까지 다른 로봇을 전혀 인지하지 못한다.
            # (fleet_decision_node.py 의 358 / 381 / 648 / 803 행)
            #
            # _stop_in_flight 는 여기서 항상 내려가므로, 기존의
            # "status != CANCELED 이면 강제 클리어" 와 최종 상태가 같다.
            if self._stop_in_flight:
                publish_stop_complete = True
                self._stop_in_flight = False

            # 이 goal 을 기다리던 레코드를 꺼낸다. 레코드는 goal 하나당 하나이므로
            # 여기서 비워도 다른 대기자의 신호를 건드리지 않는다.
            completion = self._active_completion
            self._active_completion = None

        # 이 goal 이 완전히 끝났음을 알린다. 기다리는 쪽은 둘 중 하나다.
        #   - 재라우팅 대기 중인 _overlap_gate
        #   - 완주를 기다리는 action execute
        # (락 밖에서 세운다 - 기다리는 쪽이 _state_lock 을 잡지 않은 채 깨어나도록)
        if completion is not None:
            completion.status = status
            completion.result_code = result_code
            completion.message = result_message
            completion.event.set()

        if publish_stop_complete:
            done_msg = Bool()
            done_msg.data = True
            self._stop_complete_publisher.publish(done_msg)
            self.get_logger().info('nav_stop_complete published')

    # ------------------------------------------------------------------ #
    # 관제 명령 endpoint (동기)
    # ------------------------------------------------------------------ #
    def _nav_goal_callback(self, goal_request) -> GoalResponse:
        """goal 수락/거부. 이 응답이 곧 관제가 기다리는 ack 이다.

        형식이 틀린 명령은 여기서 즉시 거부한다. _run_move 까지 내려가면
        costmap clear 와 목표 점유 대기를 지난 뒤에야 알 수 있어서 관제가
        한참 기다린 끝에 실패를 받는다.
        """
        cmd = goal_request.command
        if not cmd.goal_poses:
            self.get_logger().warn(
                f'NavigateCommand rejected: empty goal_poses '
                f'(cmd_seq_num={cmd.cmd_seq_num})')
            return GoalResponse.REJECT
        if not (len(cmd.goal_poses) == cmd.goal_cnt
                == len(cmd.from_node_id) == len(cmd.to_node_id)):
            self.get_logger().error(
                f'NavigateCommand rejected: inconsistent sizes '
                f'goal_cnt={cmd.goal_cnt}, poses={len(cmd.goal_poses)}, '
                f'from={len(cmd.from_node_id)}, to={len(cmd.to_node_id)}')
            return GoalResponse.REJECT

        self.get_logger().info(
            f'NavigateCommand accepted: goal_cnt={cmd.goal_cnt}, '
            f'cmd_seq_num={cmd.cmd_seq_num}, from_node_id={list(cmd.from_node_id)}, '
            f'to_node_id={list(cmd.to_node_id)}')
        return GoalResponse.ACCEPT

    def _nav_cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info('NavigateCommand cancel requested')
        return CancelResponse.ACCEPT

    def _nav_execute_callback(self, goal_handle):
        """주행 goal 실행. 최대 150초 대기 + 주행 전체 기간 동안 살아있다.

        [주의] 이 콜백은 _nav_action_cb_group(Reentrant)에서 돈다. 그래야
        여기서 블록하는 동안에도 cancel 요청이 서비스된다.
        """
        RC = NavigateCommand.Result
        cmd = goal_handle.request.command
        result = NavigateCommand.Result()
        result.cmd_seq_num = cmd.cmd_seq_num

        # 겹침 게이트를 통과했으면 _move_in_progress 점유를 이 goal 이 갖는다.
        # 해제 책임도 이쪽이다(아래 finally).
        owns_progress = False
        try:
            # 겹침/재라우팅 처리는 토픽 경로와 완전히 같은 함수를 쓴다.
            if not self._overlap_gate(cmd.cmd_seq_num):
                result.result_code = RC.RESULT_ABORTED
                result.success = False
                result.message = 'previous goal cancel timed out'
                goal_handle.abort()
                return result
            owns_progress = True

            # [주의] _active_action_goal 은 겹침 게이트를 통과한 뒤에 세운다.
            # 앞에 두면, 이 goal 이 이전 goal 을 취소하고 기다리는 동안 이전
            # goal 의 남은 _move_feedback_callback 틱이 이쪽 핸들로 발행되어
            # 아직 출발도 안 한 goal 의 첫 피드백이 남의 주행 수치로 채워진다.
            with self._state_lock:
                self._active_action_goal = goal_handle

            outcome = self._run_move(cmd, goal_handle)

            # goal 을 못 보낸 경우: _run_move 가 이유 코드를 그대로 돌려준다.
            if not isinstance(outcome, _GoalCompletion):
                result.result_code = int(outcome)
                result.success = False
                result.message = self._nav_result_message(int(outcome))
                self.get_logger().error(
                    f'NavigateCommand failed before dispatch: {result.message}')
                goal_handle.abort()
                return result

            # goal 을 보냈다. 완주/실패/취소를 기다리면서 취소 요청을 살핀다.
            stop_requested = False
            finished = False
            while not finished:
                finished = outcome.event.wait(timeout=0.1)
                if finished:
                    break
                if not rclpy.ok():
                    # 노드가 내려가는 중이다. 완료 신호는 오지 않는다.
                    break
                if goal_handle.is_cancel_requested and not stop_requested:
                    # action cancel = 관제 정지. 토픽/service 와 같은 함수를 쓴다.
                    stop_requested = True
                    self.get_logger().warn(
                        'NavigateCommand cancel -> requesting operator stop')
                    self._request_operator_stop(cmd.cmd_seq_num)

            if not finished:
                # [주의] 완료 신호 없이 빠져나온 경우 outcome 의 result_code 는
                # 아직 기본값(RESULT_SUCCEEDED)이다. 그대로 쓰면 종료 중에
                # "주행 성공" 을 관제에 보고해 버린다.
                result.result_code = RC.RESULT_ABORTED
                result.success = False
                result.message = 'node is shutting down'
            else:
                result.result_code = outcome.result_code
                result.message = outcome.message
                result.success = (outcome.result_code == RC.RESULT_SUCCEEDED)

            # [주의] rclpy 는 종료 상태를 정확히 한 번만 받아야 한다.
            # cancel 요청이 온 goal 은 반드시 canceled() 로 끝내야 하고,
            # 요청이 없었으면 canceled() 를 부르면 안 된다.
            try:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                elif result.success:
                    goal_handle.succeed()
                else:
                    # 재라우팅과 관제 정지는 "실패" 가 아니지만 SUCCEEDED 도 아니다.
                    # wire 상태는 ABORTED 로 가고, 진짜 의미는 result_code 가 싣는다.
                    # bridge 는 반드시 result_code 를 봐야 한다.
                    goal_handle.abort()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'terminal state transition failed: {exc}')
            return result

        finally:
            with self._state_lock:
                # [주의] 후임 goal 이 이미 점유를 가져갔다면 건드리지 않는다.
                # 무조건 내리면, 재라우팅으로 들어온 goal B 가 주행을 시작한 뒤
                # 취소된 goal A 의 execute 가 깨어나면서 B 의 점유 표시를 지운다.
                # (identity 로 판별한다 - B 가 _active_action_goal 을 가져갔으면
                #  _move_in_progress 는 B 의 것이다)
                if self._active_action_goal is goal_handle:
                    self._active_action_goal = None
                    if owns_progress:
                        self._move_in_progress = False
                elif owns_progress and self._active_action_goal is None:
                    # 이 goal 이 점유를 잡았지만 _active_action_goal 을 세우기
                    # 전에 빠져나온 경우(대기 단계 실패 등)까지 확실히 해제한다.
                    self._move_in_progress = False

    @staticmethod
    def _nav_result_message(result_code: int) -> str:
        RC = NavigateCommand.Result
        return {
            RC.RESULT_REJECTED_EMPTY: 'empty goal_poses',
            RC.RESULT_REJECTED_SIZE_MISMATCH: 'inconsistent NavigationCommand sizes',
            RC.RESULT_READY_TIMEOUT: 'timed out waiting for READY / free goals',
            RC.RESULT_NAV2_UNAVAILABLE: 'navigate_through_poses server unavailable',
            RC.RESULT_CANCELED_OPERATOR: 'stopped by operator during wait phase',
            RC.RESULT_CANCELED_REROUTE: 'superseded by a newer command',   # [B1 FIX]
        }.get(result_code, f'result_code={result_code}')

    def _nav_control_callback(self, request, response):
        """PAUSE / RESUME / STOP / RESET 을 command 인자로 받는다.

        네 개의 UInt8 토픽을 하나로 대체한다. 네 동작 모두 플래그 세팅과
        cancel_goal_async 뿐이라 즉시 리턴한다 - 절대 블로킹하지 않는다.
        """
        Req = NavigationControl.Request
        Res = NavigationControl.Response

        handlers = {
            Req.COMMAND_PAUSE: self._do_pause,
            Req.COMMAND_RESUME: self._do_resume,
            Req.COMMAND_STOP: self._request_operator_stop,
            Req.COMMAND_RESET: self._do_reset,
        }
        handler = handlers.get(request.command)
        if handler is None:
            self.get_logger().error(
                f'NavigationControl: unknown command {request.command}')
            response.accepted = False
            response.result_code = Res.RESULT_UNKNOWN_COMMAND
            response.message = f'unknown command {request.command}'
            return response

        self.get_logger().info(
            f'NavigationControl: command={request.command}, '
            f'cmd_seq_num={request.cmd_seq_num}')
        accepted, code, message = handler(request.cmd_seq_num)
        response.accepted = accepted
        response.result_code = code
        response.message = message
        return response

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


    # costmap clear 서비스를 기다리는 상한(초).
    # [FIX] 예전에는 상한이 없어서 costmap 서버가 없으면 _run_move 의 첫 줄
    # (clear_both_costmaps)에서 영원히 블록됐다. 그러면 관제 명령이 통째로
    # 사라지고 로봇이 아무 반응도 하지 않는다.
    #
    # [동작 변경] 만료 시 명령을 거부하지 않고 경고만 남기고 진행한다.
    # costmap clear 는 주행 전 best-effort 최적화일 뿐이고, 이것 때문에
    # 관제 명령을 버려서 로봇을 세우는 것이 더 나쁘다.
    COSTMAP_SERVICE_WAIT_TIMEOUT = 30.0

    def wait_for_services(self) -> bool:
        """두 Costmap 서비스를 상한을 두고 기다린다."""
        self.get_logger().info('Waiting for costmap clear services to become active...')

        deadline = self.get_clock().now().nanoseconds / 1e9 + self.COSTMAP_SERVICE_WAIT_TIMEOUT

        for client, name in ((self.client_local, '/local_costmap/clear_entirely_local_costmap'),
                             (self.client_global, '/global_costmap/clear_entirely_global_costmap')):
            while not client.wait_for_service(timeout_sec=1.0):
                if not rclpy.ok():
                    self.get_logger().error(f'Interrupted while waiting for {name}.')
                    return False
                if self.get_clock().now().nanoseconds / 1e9 > deadline:
                    self.get_logger().error(
                        f'Timed out ({self.COSTMAP_SERVICE_WAIT_TIMEOUT}s) waiting for '
                        f'{name}. Proceeding with a stale costmap.')
                    return False
                self.get_logger().info(f'Still waiting for {name}...')

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

    # [변경] 10 -> 16. NavigateCommand action 의 execute 콜백이 주행 goal 하나당
    # 스레드 하나를 주행 전체 기간(목표 점유 대기 최대 150초 + 실제 주행)동안
    # 점유한다. 기존 콜백 그룹들이 쓰던 여유를 잠식하지 않도록 늘린다.
    executor = MultiThreadedExecutor(num_threads=16)
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