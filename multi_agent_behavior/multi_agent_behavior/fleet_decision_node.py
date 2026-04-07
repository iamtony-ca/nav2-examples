#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from geometry_msgs.msg import Pose
from multi_agent_msgs.msg import PathAgentCollisionInfo
from multi_agent_msgs.msg import MultiAgentInfoArray, MultiAgentInfo, AgentStatus

from enum import IntEnum
from typing import Optional, Dict, Tuple, List

# ----------------------------------------------------------------------
# 1) Enums & Constants
# ----------------------------------------------------------------------
class MovingCommand(IntEnum):
    WAIT = 0             # 일반 대기
    WAIT_DETECT_AMR = 1  # 2초 대기 후 재탐색
    WAIT_OHTHER_AMR = 2  # 30초/450초 대기 후 재탐색
    WAIT_ABNORMAL = 3    # 30초/300초 대기 후 재탐색
    REROUTE = 4          # 즉시 재라우팅 요청

class MovingStopType(IntEnum):
    TYPE_NONE = 0
    TYPE_1 = 1   # Manual AMR
    TYPE_2 = 2   # Auto 동일 경로
    TYPE_3 = 3   # Auto 상대 Reroute & 경로 겹침
    TYPE_4 = 4   # (Reserved)
    TYPE_5 = 5   # 내 ID > 상대 (오래 대기)
    TYPE_6 = 6   # 내 ID < 상대 (짧게 대기)
    TYPE_7 = 7   # 상대 Reroute & 나 Reroute 안함
    TYPE_8 = 8   # 상대 Reroute 안함 & 나 Reroute 중
    TYPE_9 = 9   # 둘다 안함 & 내 ID > 상대
    TYPE_10 = 10 # 둘다 안함 & 내 ID < 상대
    TYPE_11 = 11 # 일반 장애물
    TYPE_12 = 12 # 전방 AMR 정지 상태

class RerouteStatus(IntEnum):
    NONE = 0
    PREPARE = 1
    EXECUTE = 2

SAME_PATH = 1
DIFFERENT_PATH = 0

# ----------------------------------------------------------------------
# 2) Helper Class for Wait Management
# ----------------------------------------------------------------------
class WaitManager:
    def __init__(self, clock):
        self.clock = clock
        self.start_time: Optional[Time] = None
        self.wait_duration_sec: float = 0.0
        self.is_waiting: bool = False
        self.triggered_type: MovingStopType = MovingStopType.TYPE_NONE

    def start_wait(self, duration_sec: float, stop_type: MovingStopType):
        # 이미 같은 타입으로 대기 중이면 리셋하지 않음 (타이머 유지)
        if not self.is_waiting or self.triggered_type != stop_type:
            self.start_time = self.clock.now()
            self.wait_duration_sec = duration_sec
            self.triggered_type = stop_type
            self.is_waiting = True

    def check_finished(self) -> bool:
        if not self.is_waiting or self.start_time is None:
            return False
        elapsed = (self.clock.now() - self.start_time).nanoseconds * 1e-9
        return elapsed >= self.wait_duration_sec

    def reset(self):
        self.is_waiting = False
        self.start_time = None
        self.triggered_type = MovingStopType.TYPE_NONE

# ----------------------------------------------------------------------
# 3) Main Node
# ----------------------------------------------------------------------
class FleetDecisionNode(Node):
    def __init__(self):
        super().__init__("fleet_decision_node")

        # ---- Parameters ----
        self.declare_parameter("my_machine_id", 1)
        self.declare_parameter("use_reroute", True)
        
        # Timeouts
        self.declare_parameter("wait_detect_amr_sec", 2.0)
        self.declare_parameter("wait_other_amr_long_sec", 450.0)
        self.declare_parameter("wait_other_amr_short_sec", 30.0)
        self.declare_parameter("wait_abnormal_long_sec", 300.0)
        self.declare_parameter("wait_abnormal_short_sec", 30.0)
        self.declare_parameter("wait_obstacle_sec", 15.0)
        
        self.declare_parameter("replan_ignore_sec_after_agent", 0.5)
        
        # [수정] 충돌 메시지 타임아웃 설정 (이 시간동안 메시지 없으면 장애물 해소로 간주)
        self.declare_parameter("collision_msg_timeout_sec", 3.0) 
        
        # [수정] Reroute 요청 쿨다운 시간 설정 (예: 2초 동안 재요청 금지)
        self.declare_parameter("reroute_cooldown_sec", 5.0)

        # Topics
        self.declare_parameter("topic_collision", "/path_agent_collision_info")
        self.declare_parameter("topic_agents", "/multi_agent_infos")
        self.declare_parameter("topic_replan_flag", "/replan_flag")

        # Output Topics
        self.declare_parameter("topic_decision_state", "/decision_state")
        self.declare_parameter("topic_request_replan", "/request_replan")
        # self.declare_parameter("topic_request_reroute", "/request_reroute")
        self.declare_parameter("service_reroute", "/request_reroute_srv")
        self.declare_parameter("topic_cmd_run", "/cmd/run")
        self.declare_parameter("topic_cmd_resume", "/controller_pause_flag")
        self.declare_parameter("topic_cmd_stop", "/controller_pause_flag")

        # Fetch Params
        self.my_id = self.get_parameter("my_machine_id").value
        self.use_reroute = self.get_parameter("use_reroute").value
        
        self.wait_detect_sec = self.get_parameter("wait_detect_amr_sec").value
        self.wait_other_long_sec = self.get_parameter("wait_other_amr_long_sec").value
        self.wait_other_short_sec = self.get_parameter("wait_other_amr_short_sec").value
        self.wait_abnormal_long_sec = self.get_parameter("wait_abnormal_long_sec").value
        self.wait_abnormal_short_sec = self.get_parameter("wait_abnormal_short_sec").value
        self.wait_obstacle_sec = self.get_parameter("wait_obstacle_sec").value
        
        self.replan_ignore_sec = self.get_parameter("replan_ignore_sec_after_agent").value
        self.collision_msg_timeout = self.get_parameter("collision_msg_timeout_sec").value
        
        # [수정] Reroute 쿨다운 값 가져오기
        self.reroute_cooldown_sec = self.get_parameter("reroute_cooldown_sec").value

# ---------------------------------------------------------
        # [수정 1] Replan Flag 수신 시 대기할 시간(N sec) 및 타이머 변수 추가
        # ---------------------------------------------------------
        self.declare_parameter("replan_flag_wait_sec", 2.0)
        self.replan_flag_wait_sec = self.get_parameter("replan_flag_wait_sec").value
        self._replan_flag_timer = None
        
        # Internal State
        self._is_reroute_status = RerouteStatus.NONE
        self._pre_moving_stop_type = MovingStopType.TYPE_NONE
        self._last_agent_event_time: Optional[Time] = None
        self._cached_agents: Dict[int, MultiAgentInfo] = {}
        
        # [수정] 마지막 충돌 메시지 수신 시각 저장용
        self._last_collision_msg_time: Optional[Time] = None 
        
        # [수정] 마지막 Reroute 요청 시각 저장용
        self._last_reroute_req_time: Optional[Time] = None

        self.wait_manager = WaitManager(self.get_clock())

        # Subscriptions
        self.create_subscription(MultiAgentInfoArray, 
            self.get_parameter("topic_agents").value, self.on_agents, 10)
        
        self.create_subscription(PathAgentCollisionInfo, 
            self.get_parameter("topic_collision").value, self.on_collision, 10)
        
        self.create_subscription(Bool, 
            self.get_parameter("topic_replan_flag").value, self.on_replan_flag, 10)

        # Publishers
        qos_req = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.pub_req_replan = self.create_publisher(Bool, 
            self.get_parameter("topic_request_replan").value, qos_req)
        
            # [수정] Reroute 전용 Publisher 생성
        # self.pub_req_reroute = self.create_publisher(Bool, 
        #     self.get_parameter("topic_request_reroute").value, 10)        
        # [NEW] Reroute Service Client
        self.cli_reroute = self.create_client(Trigger, 
            self.get_parameter("service_reroute").value)


        self.pub_state = self.create_publisher(String, 
            self.get_parameter("topic_decision_state").value, 10)
        
        self.pub_cmd_run = self.create_publisher(Bool, 
            self.get_parameter("topic_cmd_run").value, 10)

        # [수정] Resume Publisher 생성
        self.pub_cmd_resume = self.create_publisher(Bool, 
            self.get_parameter("topic_cmd_resume").value, qos_req)
        
        self.pub_cmd_stop = self.create_publisher(Bool, 
            self.get_parameter("topic_cmd_stop").value, qos_req)

        # [수정] Watchdog Timer 생성 (0.1초 간격으로 메시지 끊김 확인)
        self.create_timer(0.1, self.check_collision_timeout)

        # ------------------------------------------------------------------
        # Log All Parameters
        # ------------------------------------------------------------------
        self.get_logger().info("========== Fleet Decision Node Parameters ==========")
        self.get_logger().info(f" - my_machine_id           : {self.my_id}")
        self.get_logger().info(f" - use_reroute             : {self.use_reroute}")
        self.get_logger().info(f" - wait_detect_sec         : {self.wait_detect_sec}")
        self.get_logger().info(f" - wait_other_long_sec     : {self.wait_other_long_sec}")
        self.get_logger().info(f" - wait_other_short_sec    : {self.wait_other_short_sec}")
        self.get_logger().info(f" - wait_abnormal_long_sec  : {self.wait_abnormal_long_sec}")
        self.get_logger().info(f" - wait_abnormal_short_sec : {self.wait_abnormal_short_sec}")
        self.get_logger().info(f" - wait_obstacle_sec       : {self.wait_obstacle_sec}")
        self.get_logger().info(f" - replan_ignore_sec       : {self.replan_ignore_sec}")
        self.get_logger().info(f" - collision_msg_timeout   : {self.collision_msg_timeout}")
        self.get_logger().info(f" - reroute_cooldown_sec    : {self.reroute_cooldown_sec}")
        self.get_logger().info(f" - topic_collision         : {self.get_parameter('topic_collision').value}")
        self.get_logger().info(f" - topic_request_replan    : {self.get_parameter('topic_request_replan').value}")
        # self.get_logger().info(f" - topic_request_reroute   : {self.get_parameter('topic_request_reroute').value}")
        self.get_logger().info(f" - service_reroute         : {self.get_parameter('service_reroute').value}")        
        self.get_logger().info(f" - topic_cmd_resume        : {self.get_parameter('topic_cmd_resume').value}")
        self.get_logger().info(f" - topic_cmd_stop          : {self.get_parameter('topic_cmd_stop').value}")
        self.get_logger().info(f" - replan_flag_wait_sec          : {self.get_parameter('replan_flag_wait_sec').value}")

        self.get_logger().info("====================================================")

    # ------------------------------------------------------------------
    # [NEW] Watchdog Timer Callback
    # ------------------------------------------------------------------
    def check_collision_timeout(self):
        """
        충돌 메시지가 일정 시간 동안 수신되지 않으면 장애물이 사라진 것으로 간주하고
        RUN/RESUME 상태로 전환합니다.
        """
        # 아직 한 번도 충돌 메시지를 받은 적 없으면 패스 (혹은 기본 RUN 유지)
        if self._last_collision_msg_time is None:
            # 만약 초기 상태에서도 RUN 신호를 줘야 한다면 여기서 _handle_no_obstacle 호출 가능
            # 현재 로직상 on_collision이 호출되어야 상태가 변하므로, 
            # 초기에는 안전하게 아무것도 안 하거나, 안전하다고 가정하고 RUN을 쏠 수 있음.
            # 여기서는 '이전에 멈춘 적이 있다면' 풀어주는 용도로 사용.
            if self._pre_moving_stop_type != MovingStopType.TYPE_NONE:
                 self._handle_no_obstacle()
            return

        # 마지막 수신 시간으로부터 경과 시간 계산
        now = self.get_clock().now()
        dt = (now - self._last_collision_msg_time).nanoseconds * 1e-9

        # 타임아웃 발생 시 장애물 해소 처리
        if dt > self.collision_msg_timeout:
            # 이미 처리했으면(TYPE_NONE) 반복 호출 방지 (로그 스팸 방지용)
            # 단, RUN/RESUME 신호는 지속적으로 줘야 하므로 _handle_no_obstacle 내부는 실행
            self._handle_no_obstacle()

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def on_agents(self, msg: MultiAgentInfoArray):
        self._cached_agents = {a.machine_id: a for a in msg.agents}
        
        if self.my_id in self._cached_agents:
            me = self._cached_agents[self.my_id]
            if me.reroute:
                self._is_reroute_status = RerouteStatus.EXECUTE
            else:
                self._is_reroute_status = RerouteStatus.NONE

    # def on_replan_flag(self, msg: Bool):
    #     if not msg.data: return
    #     now = self.get_clock().now()
    #     if self._last_agent_event_time is not None:
    #         dt = (now - self._last_agent_event_time).nanoseconds * 1e-9
    #         if dt < self.replan_ignore_sec:
    #             return
    #     self.get_logger().warn("External replan_flag -> REPLAN")
    #     self._publish_replan()
    #     self._publish_state("REPLAN (Flag)")
    
# ------------------------------------------------------------------
    # [수정 2] on_replan_flag 콜백 변경 및 Timer 콜백 함수 추가
    # ------------------------------------------------------------------
    def on_replan_flag(self, msg: Bool):
        if not msg.data: return
        now = self.get_clock().now()
        if self._last_agent_event_time is not None:
            dt = (now - self._last_agent_event_time).nanoseconds * 1e-9
            if dt < self.replan_ignore_sec:
                return
        
        # 이미 대기 중인 타이머가 있다면 중복 실행 방지
        if self._replan_flag_timer is not None:
            self.get_logger().warn("External replan_flag ignored: Already waiting to replan.")
            return

        self.get_logger().warn(f"External replan_flag -> STOP and Wait {self.replan_flag_wait_sec}s")
        
        # 1. 즉시 STOP 퍼블리시
        if self.replan_flag_wait_sec != 0.0:
            self._publish_stop()
            self._publish_state(f"REPLAN_FLAG -> WAIT({self.replan_flag_wait_sec}s)")

            # 2. N초 후 REPLAN & RESUME을 실행할 1회용 비동기 타이머 생성
            self._replan_flag_timer = self.create_timer(self.replan_flag_wait_sec, self._replan_flag_timer_callback)
        elif self.replan_flag_wait_sec == 0.0:
                    # 3. REPLAN 퍼블리시
            self._publish_replan()
            self._publish_state("REPLAN_FLAG -> RESUME")

    
    def _replan_flag_timer_callback(self):
        """ N초 대기 후 Replan과 Resume을 실행하는 콜백 """
        
        # 타이머 파괴 (1회용 실행 보장 및 메모리 해제)
        if self._replan_flag_timer is not None:
            self._replan_flag_timer.cancel()
            self.destroy_timer(self._replan_flag_timer)
            self._replan_flag_timer = None

        self.get_logger().warn("External replan_flag -> REPLAN & RESUME Executed")
        
        # 3. REPLAN 퍼블리시
        self._publish_replan()
        
        # 4. RESUME 퍼블리시 (Pause 해제)
        self.pub_cmd_resume.publish(Bool(data=False))
        self._publish_state("REPLAN_FLAG -> RESUME")


    
    def on_collision(self, msg: PathAgentCollisionInfo):
        """ 메인 의사결정 루프 """
        
        # [수정] 메시지가 들어왔다는 것 자체가 충돌 감지를 의미하므로 시간 갱신
        self._last_collision_msg_time = self.get_clock().now()
        
        # 혹시라도 빈 메시지가 들어올 경우를 대비한 방어 코드 (여전히 유효)
        if not msg.x: 
            self._handle_no_obstacle()
            return

        self._last_agent_event_time = self.get_clock().now()

        # [리뷰 반영] machine_id가 비어있다 = 알려지지 않은 일반 장애물 = TYPE_11
        if not msg.machine_id:
            self._execute_command(MovingCommand.WAIT, MovingStopType.TYPE_11)
            return

        # 가장 가까운 충돌 객체 정보 추출
        target_id = int(msg.machine_id[0])
        collision_x = float(msg.x[0])
        collision_y = float(msg.y[0])

        # 내 자신이면 무시 (Safety)
        if target_id == self.my_id:
            self._handle_no_obstacle()
            return
        
        # 0번 ID도 보통 Invalid로 취급 -> TYPE_11로 Fallback하거나 RUN
        if target_id == 0:
            self._execute_command(MovingCommand.WAIT, MovingStopType.TYPE_11)
            return

        # [리뷰 반영] 충돌 좌표를 넘겨서 정밀 판정 수행
        command, stop_type = self._decide_obstacle_action(target_id, (collision_x, collision_y))
        
        self._execute_command(command, stop_type)

    # ------------------------------------------------------------------
    # Core Logic
    # ------------------------------------------------------------------
    def _decide_obstacle_action(self, target_id: int, collision_xy: Tuple[float, float]) -> Tuple[MovingCommand, MovingStopType]:
        
        n_check_complete = MovingCommand.WAIT
        moving_stop_type = MovingStopType.TYPE_NONE
        
        if self.use_reroute and self._is_reroute_status == RerouteStatus.PREPARE:
            self._is_reroute_status = RerouteStatus.EXECUTE

        agent = self._cached_agents.get(target_id)
        if agent is None:
            # 에이전트 정보 없으면 일반 장애물
            return MovingCommand.WAIT, MovingStopType.TYPE_11

        # 3. Decision Tree
        
        # [리뷰 반영] Manual Mode 판정 강화
        if self._check_vehicle_manual_mode(agent):
            n_check_complete = MovingCommand.WAIT_DETECT_AMR
            moving_stop_type = MovingStopType.TYPE_1
        else:
            # [리뷰 반영] 동일 경로 판정 (벡터 기반)
            if self._check_vehicle_path(agent) == SAME_PATH:
                # [리뷰 반영] 정지 상태 판정 강화
                if self._check_vehicle_status(agent): # Stopped
                    n_check_complete = MovingCommand.WAIT_DETECT_AMR
                    moving_stop_type = MovingStopType.TYPE_12
                else: # Moving
                    n_check_complete = MovingCommand.WAIT_ABNORMAL
                    moving_stop_type = MovingStopType.TYPE_2
            else:
                # 경로 다름
                if self.use_reroute:
                    if self._check_vehicle_rerouting(agent) == RerouteStatus.EXECUTE:
                        # [리뷰 반영] 경로 겹침 정밀 판정
                        if self._check_vehicle_rerouting_path(agent, collision_xy):
                            # TYPE_3: 상대 Reroute 경로가 내 충돌 위치와 겹침
                            n_check_complete = MovingCommand.REROUTE
                            moving_stop_type = MovingStopType.TYPE_3
                        else:
                            # 겹치지 않음 -> 우선순위 비교
                            if self._is_reroute_status != RerouteStatus.NONE:
                                if target_id < self.my_id:
                                    n_check_complete = MovingCommand.WAIT_OHTHER_AMR
                                    moving_stop_type = MovingStopType.TYPE_5
                                else:
                                    n_check_complete = MovingCommand.WAIT_DETECT_AMR
                                    moving_stop_type = MovingStopType.TYPE_6
                            else:
                                n_check_complete = MovingCommand.WAIT_DETECT_AMR
                                moving_stop_type = MovingStopType.TYPE_7
                    else:
                        # 상대 Reroute 아님
                        if self._is_reroute_status != RerouteStatus.NONE:
                             n_check_complete = MovingCommand.WAIT_ABNORMAL
                             moving_stop_type = MovingStopType.TYPE_8
                        else:
                            if target_id < self.my_id:
                                n_check_complete = MovingCommand.WAIT_OHTHER_AMR
                                moving_stop_type = MovingStopType.TYPE_9
                            else:
                                n_check_complete = MovingCommand.WAIT_DETECT_AMR
                                moving_stop_type = MovingStopType.TYPE_10
                else:
                    # Reroute 미사용
                    if target_id < self.my_id:
                        n_check_complete = MovingCommand.WAIT_OHTHER_AMR
                        moving_stop_type = MovingStopType.TYPE_5
                    else:
                        n_check_complete = MovingCommand.WAIT_DETECT_AMR
                        moving_stop_type = MovingStopType.TYPE_6
        
        return n_check_complete, moving_stop_type

    # ------------------------------------------------------------------
    # Helper Functions (Refined)
    # ------------------------------------------------------------------
    def _check_vehicle_manual_mode(self, agent: MultiAgentInfo) -> bool:
        # 1. Mode string check
        if agent.mode.lower() == "manual":
            return True
        # 2. Phase check (AgentStatus msg 참조)
        # STATUS_MANUAL_RUNNING(12), STATUS_MANUAL_COMPLETE(13)
        if agent.status.phase in [AgentStatus.STATUS_MANUAL_RUNNING, AgentStatus.STATUS_MANUAL_COMPLETE]:
            return True
        return False

    def _check_vehicle_path(self, agent: MultiAgentInfo) -> int:
        """ 
        [리뷰 반영] Yaw 비교 + Vector 비교 Hybrid
        """
        if self.my_id not in self._cached_agents:
            return DIFFERENT_PATH
        me = self._cached_agents[self.my_id]
        
        # 1. Truncated Path Vector 비교 (가장 정확)
        my_vec = self._get_path_vector(me.truncated_path.poses)
        other_vec = self._get_path_vector(agent.truncated_path.poses)
        
        if my_vec and other_vec:
            dot = my_vec[0]*other_vec[0] + my_vec[1]*other_vec[1]
            # 내적 > 0.707 (cos 45도) 이면 같은 방향
            if dot > 0.707:
                return SAME_PATH
            else:
                return DIFFERENT_PATH

        # 2. Fallback: Yaw 비교
        my_yaw = self._get_yaw(me.current_pose.pose)
        other_yaw = self._get_yaw(agent.current_pose.pose)
        diff = abs(math.degrees(self._ang_wrap(my_yaw - other_yaw)))
        
        if diff < 45.0:
            return SAME_PATH
        return DIFFERENT_PATH

    def _get_path_vector(self, poses) -> Optional[Tuple[float, float]]:
        """ 경로의 시작점과 끝점을 잇는 단위 벡터 계산 """
        if not poses or len(poses) < 2:
            return None
        start = poses[0].pose.position
        end = poses[-1].pose.position
        dx = end.x - start.x
        dy = end.y - start.y
        norm = math.hypot(dx, dy)
        if norm < 0.1: # 이동 거리가 너무 짧으면 벡터 신뢰 불가
            return None
        return (dx/norm, dy/norm)

    def _check_vehicle_status(self, agent: MultiAgentInfo) -> bool:
        """ True: Stopped, False: Moving """
        # 1. Phase Check
        moving_phases = [AgentStatus.STATUS_MOVING, AgentStatus.STATUS_PATH_SEARCHING]
        if agent.status.phase in moving_phases:
            # 2. Twist Check (Phase가 Moving이어도 실제 속도가 0이면 정지로 간주)
            lin_v = abs(agent.current_twist.linear.x)
            ang_v = abs(agent.current_twist.angular.z)
            if lin_v < 0.01 and ang_v < 0.01:
                return True # Stopped
            return False # Moving
        
        return True # Stopped (Idle, Error, etc.)

    def _check_vehicle_rerouting(self, agent: MultiAgentInfo) -> int:
        if agent.reroute:
            return RerouteStatus.EXECUTE
        return RerouteStatus.NONE

    def _check_vehicle_rerouting_path(self, agent: MultiAgentInfo, collision_xy: Tuple[float, float]) -> bool:
        """ 
        [리뷰 반영] 충돌 지점이 상대방의 Truncated Path 근처에 있는가? 
        """
        if not agent.reroute:
            return False
            
        poses = agent.truncated_path.poses
        if not poses:
            return False # 경로 정보 없으면 판단 불가 -> False (보수적) or True? C++은 보통 False

        cx, cy = collision_xy
        THRESHOLD = 0.5 # 0.5m 이내면 경로 상에 있다고 판단

        # 점과 경로(Polyline) 사이의 거리 계산
        min_dist = float('inf')
        for pose in poses:
            px = pose.pose.position.x
            py = pose.pose.position.y
            dist = math.hypot(cx - px, cy - py)
            if dist < min_dist:
                min_dist = dist
        
        return min_dist < THRESHOLD

    # ------------------------------------------------------------------
    # Execution
    # ------------------------------------------------------------------
    def _execute_command(self, command: MovingCommand, stop_type: MovingStopType):
        
        if stop_type != self._pre_moving_stop_type:
            self.get_logger().info(f"[Decision] Type: {stop_type.name}, Cmd: {command.name}")
            self._pre_moving_stop_type = stop_type
        
        state_str = f"{stop_type.name}"

        if command == MovingCommand.REROUTE:
            self.wait_manager.reset()
            
            # [수정] Reroute 요청 쿨다운 체크
            now = self.get_clock().now()
            should_request = True
            
            if self._last_reroute_req_time is not None:
                dt = (now - self._last_reroute_req_time).nanoseconds * 1e-9
                if dt < self.reroute_cooldown_sec:
                    should_request = False # 쿨타임 중이면 발행 생략
            
            if should_request:
                # 1. Reroute 요청
                # self._publish_reroute()
                self._call_reroute_service()
                self._last_reroute_req_time = now # 시간 갱신
                self.get_logger().warn("Request REROUTE sent.")
            else:
                # 쿨타임 중일 때는 로그 정도만 (선택 사항)
                # self.get_logger().debug("Reroute skipped due to cooldown.")
                pass

            # 2. Resume 요청 (무조건 발행 - BT Stop 해제용)
            # Reroute 토픽은 아껴도 Resume 토픽은 계속 보내줘야 로봇이 멈추지 않음
            self.pub_cmd_resume.publish(Bool(data=False))
            
            self._publish_state(state_str + " -> REROUTE & RESUME")
            return

        # 대기 시간 매핑
        wait_time = 0.0
        if command == MovingCommand.WAIT_DETECT_AMR:
            wait_time = self.wait_detect_sec
        elif command == MovingCommand.WAIT_OHTHER_AMR:
            wait_time = self.wait_other_long_sec if self.use_reroute else self.wait_other_short_sec
        elif command == MovingCommand.WAIT_ABNORMAL:
            wait_time = self.wait_abnormal_long_sec if self.use_reroute else self.wait_abnormal_short_sec
        elif command == MovingCommand.WAIT: # TYPE_11
             wait_time = self.wait_obstacle_sec

        # [리뷰 반영] TYPE_11 포함 모든 WAIT 커맨드에 대해 타이머 적용
        if not self.wait_manager.is_waiting:
            self.wait_manager.start_wait(wait_time, stop_type)
            self._publish_stop()
            self._publish_state(state_str + f" -> WAIT({wait_time}s)")
        else:
            if self.wait_manager.check_finished():
                self.get_logger().warn(f"Wait finished for {stop_type.name}. Requesting Replan.")
                self.wait_manager.reset()
                self._publish_replan()
                # BT의 Stop 노드를 풀기 위해 Resume 토픽도 같이 발행!
                self.pub_cmd_resume.publish(Bool(data=False))
                self._publish_state(state_str + " -> TIMEOUT REPLAN & RESUME")
            else:
                self._publish_stop()

    # ------------------------------------------------------------------
    # [수정] 장애물 없음 처리 (RUN vs RESUME 분기)
    # ------------------------------------------------------------------
    def _handle_no_obstacle(self):
        """ 장애물이 없는 상태 처리: RUN 또는 RESUME """
        
        # 1. 이전에 멈춰있었는가? (MovingStopType이 NONE이 아님)
        if self._pre_moving_stop_type != MovingStopType.TYPE_NONE:
            # [CASE: RESUME] 멈췄다가 출발하는 경우
            self.get_logger().info("[Decision] Clear -> RESUME")
            self.pub_cmd_resume.publish(Bool(data=False)) 
        else:
            self.pub_cmd_run.publish(Bool(data=True))    
        
        # 공통 상태 초기화
        self._pre_moving_stop_type = MovingStopType.TYPE_NONE
        self.wait_manager.reset()
        self._publish_state("RUN")

    # ------------------------------------------------------------------
    # Pub/Sub Utils
    # ------------------------------------------------------------------
    def _publish_stop(self): self.pub_cmd_stop.publish(Bool(data=True))
    def _publish_replan(self): self.pub_req_replan.publish(Bool(data=True))
    # def _publish_reroute(self): self.pub_req_reroute.publish(Bool(data=True))
    def _publish_state(self, txt: str): self.pub_state.publish(String(data=txt))

    # [NEW] Service Call Wrapper with Done Callback
    def _call_reroute_service(self):
        # 서비스가 준비되었는지 0.1초만 짧게 확인 (Blocking 방지)
        if not self.cli_reroute.service_is_ready():
            self.get_logger().error("Reroute service is not ready!")
            return
        
        req = Trigger.Request()
        # Call Async: 콜백 내에서 blocking 되는 것을 막기 위해 비동기 호출
        future = self.cli_reroute.call_async(req)
        future.add_done_callback(self._reroute_done_callback)

    # [NEW] Service Done Callback
    def _reroute_done_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Reroute Service Succeeded: {response.message}")
            else:
                self.get_logger().warn(f"Reroute Service Failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Reroute Service Call Exception: {e}")


    # ------------------------------------------------------------------
    # Math Utils
    # ------------------------------------------------------------------
    def _get_yaw(self, pose: Pose) -> float:
        qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        return math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))

    def _ang_wrap(self, a: float) -> float:
        while a > math.pi: a -= 2.0 * math.pi
        while a < -math.pi: a += 2.0 * math.pi
        return a

def main():
    rclpy.init()
    node = FleetDecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
