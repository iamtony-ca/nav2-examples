#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from functools import partial # 파일 최상단에 추가하세요
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from std_msgs.msg import Bool, String, UInt8
from geometry_msgs.msg import Pose
from robot_interfaces.msg import PathAgentCollisionInfo, PathStaticCollisionInfo
from robot_interfaces.msg import MultiAgentInfoArray, MultiAgentInfo, AgentStatus
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


from enum import IntEnum
from typing import Optional, Dict, Tuple, List
import time

# ----------------------------------------------------------------------
# 1) Enums & Constants
# ----------------------------------------------------------------------
class MovingCommand(IntEnum):
    WAIT = 0             # 일반 대기
    WAIT_DETECT_AMR = 1  # 2초 대기 후 재탐색
    WAIT_OHTHER_AMR = 2  # 30초/450초 대기 후 재탐색
    WAIT_ABNORMAL = 3    # 30초/300초 대기 후 재탐색
    REROUTE = 4          
    WAIT_SIMPLE_REPLAN = 5  # ID 큰 로봇: 대기 후 Replan & Resume
    WAIT_SIMPLE_RESUME = 6  # ID 작은 로봇: 대기 후 단순 Resume

class MovingStopType(IntEnum):
    TYPE_NONE = 0
    TYPE_1 = 1   
    TYPE_2 = 2   
    TYPE_3 = 3   
    TYPE_4 = 4   
    TYPE_5 = 5   
    TYPE_6 = 6   
    TYPE_7 = 7   
    TYPE_8 = 8   
    TYPE_9 = 9   
    TYPE_10 = 10 
    TYPE_11 = 11 
    TYPE_12 = 12 

class RerouteStatus(IntEnum):
    NONE = 0
    PREPARE = 1
    EXECUTE = 2

SAME_PATH = 1
DIFFERENT_PATH = 0

# ----------------------------------------------------------------------
# 3) Main Node
# ----------------------------------------------------------------------
class FleetDecisionNode(Node):
    def __init__(self):
        super().__init__("fleet_decision_node")

        # [FIX] 두 상태머신(check_collision_obstacle / check_collision_agent)이
        # 겹쳐 돌면 안 된다. 둘은 제어권 플래그 5개를 한쪽이 쓰고 다른 쪽이 읽는다:
        #   is_processing_agent_pause, is_processing_replan_pause,
        #   is_processing_goal_occupied_pause, is_processing_last_goal_occupied_pause,
        #   nav_stop_complete_
        # 쓰기-쓰기 충돌은 없지만 "플래그 확인 -> 자기 pause 진입" 사이에 상대가
        # 끼어들 수 있고(check-then-act), 그러면 둘이 각각 pause/resume 을 발행한다.
        #
        # ReentrantCallbackGroup + MultiThreadedExecutor(6) 에서는 실제로 겹친다.
        # 실측(2026-09-17, 90초 각 900틱): 겹침 249회 -> 0회.
        # 콜백 자체가 가벼워서(중앙 0.04ms / 0.73ms, 최대 6.8ms) 직렬화해도
        # 10Hz 주기는 그대로이고, 스레드 경합이 사라져 p95 는 오히려 줄었다
        # (3.41ms -> 0.93ms, 3.80ms -> 1.05ms).
        self.cb_group = MutuallyExclusiveCallbackGroup()

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
        self.declare_parameter("topic_replan_flag", "/path_static_collision_info")

        # Output Topics
        self.declare_parameter("topic_decision_state", "/decision_state")
        self.declare_parameter("topic_request_replan", "/request_replan")
        self.declare_parameter("topic_request_rmv_first_goals", "/remove_first_goals")
        self.declare_parameter("topic_request_rmv_passed_goals", "/remove_passed_goals")
        self.declare_parameter("topic_request_reroute", "/request_reroute")
        self.declare_parameter("topic_cmd_run", "/cmd/run")
        self.declare_parameter("topic_cmd_resume", "/controller_pause_flag")
        self.declare_parameter("topic_cmd_pause", "/controller_pause_flag")
        self.declare_parameter("topic_cmd_stop", "/stop_command")

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
        self.declare_parameter("replan_flag_wait_sec", 10.0)
        self.replan_flag_wait_sec = self.get_parameter("replan_flag_wait_sec").value
        self._replan_flag_timer = None
        
        # [추가] Resume 지연을 위한 타이머 변수
        self._resume_timer = None


        # [수정] Simple Mode 파라미터화 (하드코딩 방지)
        self.declare_parameter("simple_mode", False)
        self.declare_parameter("wait_simple_mode_sec", 5.0)
        self.simple_mode = self.get_parameter("simple_mode").value
        self.wait_simple_mode = self.get_parameter("wait_simple_mode_sec").value


        # [추가] Replan Flag가 False로 연속 유지되어야 하는 시간 (예: 2.0초)
        self.declare_parameter("replan_clear_timeout_sec", 2.0)
        self.replan_clear_timeout_sec = self.get_parameter("replan_clear_timeout_sec").value
        self.declare_parameter("goal_occupied_timeout_sec", 100.0)
        self.goal_occupied_timeout_sec = self.get_parameter("goal_occupied_timeout_sec").value        
        self.declare_parameter("agent_wait_before_resume", 3.0)
        self.agent_wait_before_resume = self.get_parameter("agent_wait_before_resume").value

        # [FIX] /nav_stop_complete 유실 대비 watchdog.
        # nav_stop_complete_ 가 False 인 동안 이 노드는 충돌 판정을 전부 멈춘다.
        # 해제 경로가 /nav_stop_complete 수신 하나뿐이라, 그 통지를 한 번이라도
        # 놓치면 다중로봇 조정이 영구히 죽는다. 유실 경로는 셋이다.
        #   (a) navigation_manager 가 cancel 을 걸었는데 goal 이 그 사이에
        #       SUCCEEDED / ABORTED 로 먼저 끝나는 경합
        #   (b) STOP 이 도착한 시점에 _goal_handle 이 이미 None 이라
        #       _nav_stop_callback 이 통지 없이 조기 return 하는 경우
        #   (c) /nav_stop_complete 가 VOLATILE 이라 구독 매칭 전에 발행되어
        #       메시지 자체가 사라지는 경우
        # (a) 는 navigation_manager 쪽에서 고쳤지만 (b)(c) 는 남으므로,
        # 원인과 무관하게 N초 뒤 강제 해제한다.
        self.declare_parameter("nav_stop_complete_timeout_sec", 10.0)
        self.nav_stop_complete_timeout = self.get_parameter("nav_stop_complete_timeout_sec").value
        self._nav_stop_wait_start: Optional[Time] = None

        # [FIX] 이웃 캐시 보존 시간.
        # winros_bridge 는 이웃 패킷 1개당 agent 1개짜리 배열을 발행하므로
        # (winros_bridge.cpp:1466-1615) 캐시를 통째로 교체하면 항상 1대만 남는다.
        # 병합으로 바꾸는 대신, 소식이 끊긴 이웃은 이 시간이 지나면 버린다.
        self.declare_parameter("agent_cache_ttl_sec", 5.0)
        self.agent_cache_ttl_sec = self.get_parameter("agent_cache_ttl_sec").value

        # [FIX] 스스로는 영영 움직일 수 없는 상대의 AgentStatus.phase 목록.
        #
        # _decide_obstacle_action 은 경로가 다를 때(TYPE_5~10) 상대의 상태를 전혀
        # 보지 않고 ID 비교만 했다. 그래서 고장나 멈춘 로봇 앞에서도
        # "내 ID 가 더 크니 양보" 로 우선순위 대기를 걸었다. 이 목록에 든 상태는
        # 내가 기다려도 비켜주지 않으므로 우선순위 비교를 건너뛰고
        # 일반 장애물(TYPE_11)로 격하해 wait_obstacle_sec 뒤 우회하게 한다.
        #
        #    0 INIT                   부팅 중
        #   14 ERROR                  고장. 사람이 와야 풀린다
        #   18 WAITING_FOR_ROS_STATUS ROS 스택 이상
        #   19 CHARGING / 20 CHARGE_DONE  충전기에 물려 있다
        #   21 UNKNOWN                상태 불명. 보수적으로 격하
        #
        # [주의] 이적재 상태(MARKING 5 / UNLOADING 6 / UNLOADED 7 / LOADING 8 /
        # LOADED 9)는 일부러 넣지 않는다. 현장 실측 50초 내외로 끝나는 '유한'
        # 정지라 우회할 이유가 없고, 좁은 통로에서 돌아가면 오히려 다른 구역에
        # 병목을 만든다. 기존 우선순위 분기로 두면 Early Exit
        # (agent_wait_before_resume, 3초)가 상대의 작업이 끝나는 즉시
        # 원래 경로로 출발시킨다 - 대기 시간들은 고정 정지가 아니라 상한이다.
        #
        # PAUSE(15) / WAITING_FOR_SAFETY(16) / WAITING_FOR_FLOWCONTROL(17) /
        # WAITING_FOR_OBS(2) / ARRIVED(4) 도 곧 다시 움직이므로 제외한다.
        # 이쪽까지 격하하면 서로 양보하느라 멈춘 두 로봇이 서로를 "정지했으니
        # 우회" 로 보고 동시에 출발할 수 있어 ID 우선순위의 tie-break 가 무너진다.
        # AUTORECOVERY(10) / RECOVERING(11) 은 후진·선회로 움직이는 중일 수 있어 뺐다.
        # MANUAL(12,13) 은 앞단에서 이미 TYPE_1 로 처리한다.
        #
        # phase 값은 Windows 관제가 정해 내려보내는 것이라(winros_bridge.cpp:1508)
        # 현장 매핑을 확인한 뒤 재빌드 없이 조정할 수 있도록 파라미터로 뺐다.
        # [FIX] 정적 장애물 진동(pause <-> resume 반복) 방어.
        #
        # 맵에 없는 정적 장애물에 경로가 아슬아슬하게 걸치면 차단 판정이 깜빡이고,
        # 그때마다 replan_pause SM 이 새로 진입하면서 _pause_start_time 을 0 으로
        # 되돌린다. 그래서 상황을 실제로 바꾸는 유일한 행동인 _publish_replan()
        # (replan_pause_timeout_sec=15초)에 **영영 도달하지 못하고** 무한히 왔다갔다 한다.
        #
        # 해법: 짧은 간격으로 재차단되면 '같은 상황' 으로 보고 경과 시간을 이어받는다.
        # 그러면 진동해도 누적 15초에 도달해 replan 이 나가고 상황이 해소된다.
        #
        # 좌표(hit_x/hit_y)를 쓰지 않는 이유:
        #   hit 은 '장애물 위치' 가 아니라 '경로 위 최초 충돌 지점' 이라 로봇이
        #   전진하거나 경로가 바뀌면 같은 장애물이라도 크게 움직인다. 반경으로
        #   묶으려면 튜닝이 까다롭고, 반대로 로봇이 장애물을 지나쳐 가는 중인데도
        #   같은 지점으로 오인할 수 있다.
        #   시간만 보면 이 문제가 사라진다 - 로봇이 실제로 지나쳤다면 재차단이
        #   일어나지 않거나 간격이 길어져 자동으로 리셋된다.
        #
        # 동적 장애물(사람이 가로지르는 등)과의 구분도 이 창이 해준다. 사람은
        # 보통 수 초 이상 비웠다가 다시 오므로 창을 넘고, 스침 진동은 clear 구간이
        # replan_clear_timeout_sec(2초)라 창 안에 들어온다.
        #
        # 0 으로 두면 누적이 꺼져 기존 동작 그대로가 된다.
        self.declare_parameter("static_rejoin_window_sec", 5.0)
        self.static_rejoin_window_sec = self.get_parameter("static_rejoin_window_sec").value

        # 같은 상황에서 replan 을 이 횟수만큼 시도했는데도 계속 막히면, 자력
        # 우회 실패로 보고 관제에 알린다(내부 정지 -> driving_abort -> 관제는
        # reroute 요청으로 읽는다). 0 이면 에스컬레이션을 하지 않는다.
        #
        # [세는 대상에 주의] '진동 횟수' 가 아니라 **replan 시도 횟수** 다.
        # 진동 횟수로 세면 빠른 진동에서 replan 을 한 번도 안 해보고 관제에
        # 보고해버리고(즉시 replan 배제 원칙 위반), 느린 진동에서는 replan 마다
        # 카운터가 지워져 영영 도달하지 못한다. 둘 다 틀린다.
        #
        # 보수적으로 크게 잡는다. replan 10회가 실패해야 관제로 넘어간다.
        self.declare_parameter("static_max_replan_retry", 10)
        self.static_max_replan_retry = int(self.get_parameter("static_max_replan_retry").value)

        self._static_last_release_t: Optional[Time] = None   # 마지막 해제 시각
        self._static_last_elapsed: float = 0.0               # 이어받을 경과
        self._static_replan_retry: int = 0                   # 이 상황에서의 replan 시도 횟수

        # [FIX B-7] agent 쪽에도 같은 장치를 둔다. 지금까지는 Phase 2 에서 replan 을 쏘고
        # Phase 3 에서 재개하며 상태를 전부 지웠기 때문에, 같은 상대에게 곧바로 다시 걸리면
        # 타이머가 0 부터 다시 돌고 몇 번째 시도인지 아무도 세지 않았다. 끝을 내는 것은
        # 조정 로직이 아니라 Nav2 의 goal abort 였다 (2026-09-17 실측, 롱런 3차에서
        # 좁은 통로에서 시간당 102회 반복).
        #
        # agent_rejoin_window_sec: 해제 뒤 이 시간 안에 **같은 상대**에게 다시 걸리면 같은
        #   상황으로 본다. Early Exit 뒤면 경과 시간을 이어받고, replan 뒤면 경과는 0 이지만
        #   재시도 횟수는 이어진다. 상대가 바뀌거나 창을 넘기면 다른 상황이라 초기화한다.
        # agent_max_replan_retry: 같은 상대에게 replan 을 이 횟수만큼 시도해도 계속 막히면
        #   자력 해결 불가로 보고 관제에 넘긴다(내부 정지 -> driving_abort). 0 이면 끔.
        #   [보수적 기본값] 현장 사이드 이펙트를 아직 모르므로 30 으로 두어 사실상 안 걸리게
        #   한다. 파라미터만 낮추면 활성화되므로 현장에서 재빌드 없이 튜닝한다.
        self.declare_parameter("agent_rejoin_window_sec", 5.0)
        self.agent_rejoin_window_sec = float(self.get_parameter("agent_rejoin_window_sec").value)
        self.declare_parameter("agent_max_replan_retry", 30)
        self.agent_max_replan_retry = int(self.get_parameter("agent_max_replan_retry").value)
        self._agent_last_release_t: Optional[Time] = None    # 마지막 agent 해제 시각
        self._agent_last_elapsed: float = 0.0                # 이어받을 경과
        self._agent_replan_retry: int = 0                    # 이 상대에 대한 replan 시도 횟수
        self._agent_last_target_id: Optional[int] = None     # 직전 상황의 상대

        self.declare_parameter("immobile_agent_phases", [0, 14, 18, 19, 20, 21])
        self._immobile_phases = set(
            int(v) for v in self.get_parameter("immobile_agent_phases").value)

        # Internal State
        self._is_reroute_status = RerouteStatus.NONE
        self._pre_moving_stop_type = MovingStopType.TYPE_NONE
        self._last_agent_event_time: Optional[Time] = None
        self._cached_agents: Dict[int, MultiAgentInfo] = {}
        # [FIX] machine_id -> 마지막 수신 시각. 캐시 만료 판정용.
        self._agent_seen_at: Dict[int, Time] = {}
        # [FIX] Phase 0 에서 잠근 대상. 대상이 바뀌면 재평가한다.
        self._locked_target_id: int = 0
        
        # [수정] 마지막 충돌 메시지 수신 시각 저장용
        self._last_collision_msg_time: Optional[Time] = None 
        
        # [수정] 마지막 Reroute 요청 시각 저장용
        self._last_reroute_req_time: Optional[Time] = None


        # Subscriptions
        self.create_subscription(MultiAgentInfoArray, 
            self.get_parameter("topic_agents").value, self.on_agents, 10, callback_group=self.cb_group)
        
        self.create_subscription(PathAgentCollisionInfo, 
            self.get_parameter("topic_collision").value, self.on_collision, 10, callback_group=self.cb_group)
        
        self.create_subscription(PathStaticCollisionInfo, 
            self.get_parameter("topic_replan_flag").value, self.on_replan_flag, 10, callback_group=self.cb_group)

        self.create_subscription(Bool,
                "/nav_stop_complete", self.stop_complete_callback, 10, callback_group=self.cb_group)

        self.create_subscription(String,
                "/robot_status", self.robot_status_callback, 10, callback_group=self.cb_group)

        # Publishers
        qos_req = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.pub_req_replan = self.create_publisher(Bool, 
            self.get_parameter("topic_request_replan").value, qos_req)

        self.pub_rmv_passed_goals = self.create_publisher(Bool, 
            self.get_parameter("topic_request_rmv_passed_goals").value, qos_req)

        self.pub_rmv_first_goals = self.create_publisher(Bool, 
            self.get_parameter("topic_request_rmv_first_goals").value, qos_req)

            # [수정] Reroute 전용 Publisher 생성
        self.pub_req_reroute = self.create_publisher(Bool, 
            self.get_parameter("topic_request_reroute").value, 10, callback_group=self.cb_group)
        
        self.pub_state = self.create_publisher(String, 
            self.get_parameter("topic_decision_state").value, 10)
        
        self.pub_cmd_run = self.create_publisher(Bool, 
            self.get_parameter("topic_cmd_run").value, 10)

        # [수정] Resume Publisher 생성
        self.pub_cmd_resume = self.create_publisher(Bool, 
            self.get_parameter("topic_cmd_resume").value, qos_req, callback_group=self.cb_group)
        
        self.pub_cmd_pause = self.create_publisher(Bool, 
            self.get_parameter("topic_cmd_pause").value, qos_req, callback_group=self.cb_group)

        self.pub_cmd_stop = self.create_publisher(UInt8, 
            self.get_parameter("topic_cmd_stop").value, 10, callback_group=self.cb_group)


        self.create_timer(0.1, self.check_collision_obstacle, callback_group=self.cb_group)

        self.create_timer(0.1, self.check_collision_agent, callback_group=self.cb_group)


        # [추가] 현재 일시정지/재계획 시퀀스가 진행 중인지 확인하는 플래그
        # [FIX] goal 점유 처리(10초 remove_passed / 15·20초 remove_first / 25초 replan)
        # 활성화 스위치. 예전에는 False 로 하드코딩되어 블록 전체가 사문화돼 있었다.
        #
        # 꺼져 있던 이유는 sim 실측으로 규명했다(2026-09-16): BT 의 PauseBranch 가
        # pause 중 경로 계산 실패를 만나면 ForceFailure_Planner 로 FAILURE 를 내고,
        # 그게 Repeat/Timeout 을 타고 올라가 UnpauseCleanupSequence 가
        # /controller_pause_flag=false 를 발행해 **BT 가 fleet_decision 의 pause 를
        # 몰래 풀어버렸다.** goal 이 막히면 경로 계산은 당연히 실패하므로 pause 가
        # 3초 만에 깨졌고, 그 뒤 보내는 제거 신호는 아무도 받지 못했다
        # (Special trigger received 0회, IntelligentRecovery 24회).
        # moduler32 가 그 자리를 ForceSuccess 로 감싸 고쳤고, 같은 시나리오에서
        # 제거 신호가 2회 소비되고 회복 0회로 정상 주행했다.
        #
        # 현장에서 문제가 생기면 재빌드 없이 끌 수 있도록 파라미터로 둔다.
        self.declare_parameter("enable_goal_occupied_handling", True)
        self.check_static_is_goal_occupied_ = self.get_parameter(
            "enable_goal_occupied_handling").value

        self.is_processing_replan_pause = False
        self.replan_flag_status = False
        self.delay_after_replan = False 
        self.replan_pause_timeout_sec = 15.0  # replan_flag가 True인 상태에서 대기할 최대 시간 (예: 15초)
        self.delay_after_replan_start_time: Optional[Time] = None
        self._pause_start_time: Optional[Time] = None
        self._replan_flag_false_start_time: Optional[Time] = None
        

        self.is_processing_goal_occupied_pause = False
        self.is_processing_last_goal_occupied_pause = False
        self.static_is_last_goal_occupied_ = False
        self.static_is_goal_occupied_ = False
        # self.goal_occupied_timeout_sec = 100.0  # Goal 점유 상태에서 대기할 최대 시간 (예: 100초)   
        self._goal_occupied_false_start_time: Optional[Time] = None
        self._last_goal_occupied_false_start_time: Optional[Time] = None
        

        self.delay_after_replan_start_time_goal_occupied = None
        self.is_processing_goal_occupied_pause = False
        self.delay_after_replan_goal_occupied = False
        self._goal_occupied_false_start_time = None


# [추가] 10초, 15초, 20초 토픽 1회 발행을 위한 Trigger 플래그
        self.published_goal_10s = False
        self.published_goal_15s = False
        self.published_goal_20s = False

        self.nav_stop_complete_ = True  # STOP 명령 발행 후 주행 재개 대기 상태 플래그 (초기값 True로 설정)

        self.current_robot_status = "IDLE"

        # ==========================================================
        # [add] Agent 충돌 예측 전용 상태 변수
        # ==========================================================
        self.agent_collision_status = False
        # [add] on_collision에서 받아온 최신 Raw Data 저장용
        self.latest_agent_target_id = 0
        self.latest_agent_collision_xy = (0.0, 0.0)

        self.is_processing_agent_pause = False
        self.delay_after_agent_action = False
        self.agent_pause_timeout_sec = 0.0  # N초 대기 (명령어마다 다름)
        self.current_agent_command = MovingCommand.WAIT
        self.current_agent_stop_type = MovingStopType.TYPE_NONE
        
        self.delay_after_agent_start_time: Optional[Time] = None
        self._agent_pause_start_time: Optional[Time] = None
        self._agent_clear_start_time: Optional[Time] = None
        # ==========================================================


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
        self.get_logger().info(f" - topic_request_rmv_passed_goals    : {self.get_parameter('topic_request_rmv_passed_goals').value}")
        self.get_logger().info(f" - topic_request_rmv_first_goals    : {self.get_parameter('topic_request_rmv_first_goals').value}")
        self.get_logger().info(f" - topic_request_reroute   : {self.get_parameter('topic_request_reroute').value}")
        self.get_logger().info(f" - topic_cmd_resume        : {self.get_parameter('topic_cmd_resume').value}")
        self.get_logger().info(f" - topic_cmd_pause          : {self.get_parameter('topic_cmd_pause').value}")
        self.get_logger().info(f" - topic_cmd_stop          : {self.get_parameter('topic_cmd_stop').value}")
        self.get_logger().info(f" - replan_flag_wait_sec          : {self.get_parameter('replan_flag_wait_sec').value}")
        self.get_logger().info(f" - simple_mode               : {self.get_parameter('simple_mode').value}")
        self.get_logger().info(f" - wait_simple_mode_sec      : {self.get_parameter('wait_simple_mode_sec').value}")
        self.get_logger().info(f" - replan_clear_timeout_sec : {self.get_parameter('replan_clear_timeout_sec').value}")
        self.get_logger().info(f" - goal_occupied_timeout_sec : {self.get_parameter('goal_occupied_timeout_sec').value}")
        self.get_logger().info(f" - agent_cache_ttl_sec     : {self.agent_cache_ttl_sec}")
        self.get_logger().info(f" - immobile_agent_phases   : {sorted(self._immobile_phases)}")
        self.get_logger().info(f" - goal_occupied_handling  : {self.check_static_is_goal_occupied_}")
        self.get_logger().info(f" - static_rejoin_window_sec: {self.static_rejoin_window_sec}")
        self.get_logger().info(f" - static_max_replan_retry : {self.static_max_replan_retry}")
        self.get_logger().info(f" - agent_rejoin_window_sec : {self.agent_rejoin_window_sec}")
        self.get_logger().info(f" - agent_max_replan_retry  : {self.agent_max_replan_retry}")
        self.get_logger().info("====================================================")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def on_agents(self, msg: MultiAgentInfoArray):
        # [FIX] 통째로 교체하지 않고 병합한다.
        #
        # winros_bridge 는 관제의 이웃 패킷 1개마다 agent 1개짜리 배열을 발행한다
        # (winros_bridge.cpp:1466 makeAndPublishMultiAgentMsg, :1614 push_back).
        # 예전처럼 dict 를 새로 만들면 캐시에 늘 1대만 남아서
        #   - _cached_agents.get(target_id) 가 거의 빗나가 _decide_obstacle_action
        #     이 agent None 폴백(TYPE_11 WAIT)으로 떨어지고 우선순위 트리를 건너뛴다
        #   - my_id 가 캐시에 없어 _check_vehicle_path 가 경로 비교에 도달하지 못한다
        #   - _is_reroute_status 가 초기값 NONE 에서 벗어나지 못한다
        # 로봇이 N대면 적중률이 대략 1/N 로 떨어진다.
        now = self.get_clock().now()
        for a in msg.agents:
            self._cached_agents[a.machine_id] = a
            self._agent_seen_at[a.machine_id] = now

        # [FIX] 병합만 하면 떠난 이웃이 영원히 남는다. 관제 링크가 끊겨도
        # 몇 분 전 자세/경로로 판단하게 되므로 오래된 항목은 버린다.
        stale = [mid for mid, t in self._agent_seen_at.items()
                 if (now - t).nanoseconds * 1e-9 > self.agent_cache_ttl_sec]
        for mid in stale:
            self._cached_agents.pop(mid, None)
            self._agent_seen_at.pop(mid, None)
            self.get_logger().warn(
                f"[on_agents] agent {mid} 소식 끊김 "
                f"({self.agent_cache_ttl_sec:.1f}s). 캐시에서 제거한다.")

        me = self._cached_agents.get(self.my_id)
        if me is not None:
            self._is_reroute_status = (RerouteStatus.EXECUTE if me.reroute
                                       else RerouteStatus.NONE)




    def stop_complete_callback(self, msg: Bool):
        self.nav_stop_complete_ = msg.data
        # [FIX] 통지를 정상 수신했으면 watchdog 을 해제한다.
        if msg.data:
            self._nav_stop_wait_start = None
        elif self._nav_stop_wait_start is None:
            self._nav_stop_wait_start = self.get_clock().now()
        self.get_logger().info(f"STOP sequence complete topic received: {msg.data}", throttle_duration_sec=2.0)

    def _check_nav_stop_watchdog(self, now: Time) -> None:
        """[FIX] /nav_stop_complete 를 못 받은 채 굳어 있으면 강제 해제한다.

        정상 동작에서는 STOP 후 수백 ms 안에 통지가 오므로 이 경로는 타지 않는다.
        타는 순간은 통지가 유실된 것이므로 error 로 남긴다.
        """
        if self.nav_stop_complete_ is not False:
            return
        if self._nav_stop_wait_start is None:
            # False 인데 시작 시각이 없으면(예: 외부에서 False 수신) 지금부터 센다.
            self._nav_stop_wait_start = now
            return
        dt = (now - self._nav_stop_wait_start).nanoseconds * 1e-9
        if dt >= self.nav_stop_complete_timeout:
            self.get_logger().error(
                f"[watchdog] /nav_stop_complete 를 {dt:.1f}s 동안 받지 못했다. "
                f"다중로봇 조정을 강제 재개한다. (통지 유실 의심)")
            self.nav_stop_complete_ = True
            self._nav_stop_wait_start = None


    def robot_status_callback(self, msg: String):
        """
        IDLE = "IDLE"
        READY = "READY"
        RECEIVED_GOAL = "RECEIVED_GOAL"
        PLANNING = "PLANNING"
        DRIVING = "DRIVING"
        PAUSED = "PAUSED"
        RECOVERY_FAILURE = "RECOVERY_FAILURE"
        RECOVERY_RUNNING = "RECOVERY_RUNNING"
        RECOVERY_SUCCESS = "RECOVERY_SUCCESS"
        SUCCEEDED = "SUCCEEDED"
        FAILED = "FAILED"
        CANCELED = "CANCELED"
        
        """

        self.current_robot_status = msg.data


# ------------------------------------------------------------------
    # [수정 2] on_replan_flag 콜백 변경 및 Timer 콜백 함수 추가
    # ------------------------------------------------------------------
    def on_replan_flag(self, msg: PathStaticCollisionInfo):
        """
        # PathStaticCollisionInfo.msg

        std_msgs/Header header
        bool replan_request
        bool is_goal_occupied          # Goal 점유 여부
        bool is_last_goal_occupied          # last Goal 점유 여부
        float64 hit_x                  # 충돌 지점 X
        float64 hit_y                  # 충돌 지점 Y
        geometry_msgs/Pose target_goal # 목표 지점(Goal) 좌표
        """
       
        if self.nav_stop_complete_ == False:
            self.replan_flag_status = False
            self.static_is_last_goal_occupied_ = False
            self.static_is_goal_occupied_ = False
            self.agent_collision_status = False            
            return # STOP 명령 발행 후 주행 재개 대기 중 (STOP 시퀀스 우선 처리)
       
        self.get_logger().info(f"[on_replan_flag] Received replan_flag: {msg.replan_request}", throttle_duration_sec=2.0)
        self.replan_flag_status = msg.replan_request
        self.static_is_last_goal_occupied_ = msg.is_last_goal_occupied
        self.static_is_goal_occupied_ = msg.is_goal_occupied




    def check_collision_obstacle(self):
        """
        충돌 메시지 타임아웃과 별개로, replan_flag가 True로 유지되는 경우 일정 시간 후에 자동으로 주행 재개하는 로직
        """
       
        now = self.get_clock().now()

        self._check_nav_stop_watchdog(now)  # [FIX] 통지 유실 시 강제 해제

        if self.nav_stop_complete_ == False:
            return # STOP 명령 발행 후 주행 재개 대기 중 (STOP 시퀀스 우선 처리)

        if self.is_processing_agent_pause is True:
            return # Agent 충돌 시 Replan Pause 시퀀스 우선 처리


        if self.current_robot_status in ['RECEIVED_GOAL', 'PLANNING', 'DRIVING', 'PAUSED', 'RECOVERY_FAILURE', 'RECOVERY_RUNNING', 'RECOVERY_SUCCESS']:
            ## is_last_goal_occupied
            if self.is_processing_last_goal_occupied_pause and self.static_is_last_goal_occupied_ is True and self.is_processing_replan_pause is False and self.is_processing_goal_occupied_pause is False:
                self._last_goal_occupied_false_start_time = None
                if self._pause_start_time is not None:
                    dt = (now - self._pause_start_time).nanoseconds * 1e-9
                    if dt < self.goal_occupied_timeout_sec : 
                        self.get_logger().info(f"[check_collision_obstacle] last Goal Occupied detected but pausing for {dt:.1f}s (within timeout threshold).", throttle_duration_sec=2.0)
                    elif dt >= self.goal_occupied_timeout_sec:   
                        self.get_logger().warn(f"[check_collision_obstacle] last Goal Occupied detected timeout for {dt:.1f}s. Initiating resume sequence.")
                        self.pub_cmd_stop.publish(UInt8(data=1))  # Stop 명령 발행 (예: 1 = 긴급 정지)
                        self.pub_cmd_stop.publish(UInt8(data=1))  # Stop 명령 발행 (예: 1 = 긴급 정지)
                        self._publish_state("STOP (Goal Occupied)")
                        self.nav_stop_complete_ = False # STOP 명령 발행 후 주행 재개 대기 상태로 전환
                        self._nav_stop_wait_start = now  # [FIX] watchdog 기동
                        self.static_is_last_goal_occupied_ = False
                        self._last_goal_occupied_false_start_time = None
                        self._pause_start_time = None
                        self.is_processing_last_goal_occupied_pause = False
                        return



            if self.is_processing_last_goal_occupied_pause and self.static_is_last_goal_occupied_ is False and self.is_processing_replan_pause is False and self.is_processing_goal_occupied_pause is False:
                if self._last_goal_occupied_false_start_time is None:
                    # 처음 False가 들어온 시간 기록
                    self._last_goal_occupied_false_start_time = now
                else:
                    elapsed = (now - self._last_goal_occupied_false_start_time).nanoseconds * 1e-9
                    # M초(replan_clear_timeout_sec) 이상 연속으로 False가 들어오면
                    if elapsed >= self.replan_clear_timeout_sec:
                        if self.is_processing_last_goal_occupied_pause:
                            self.get_logger().warn(f"[check_collision_obstacle] Path clear for {elapsed:.1f}s. Aborting Pause Sequence & Early Resume!")
                            # self._abort_sequence_and_resume()
                            
                            # self._pre_moving_stop_type = MovingStopType.TYPE_NONE
                            
                            # 3. 주행 재개 신호 즉시 발행
                            self.pub_cmd_resume.publish(Bool(data=False))
                            
                            self._publish_state("[check_collision_obstacle] RUN (Early Resume)")  
                        
                            # 이미 조기 종료를 처리했으므로 시간 초기화 (중복 실행 방지)
                            self.static_is_last_goal_occupied_ = False
                            self._last_goal_occupied_false_start_time = None
                            self._pause_start_time = None
                            self.is_processing_last_goal_occupied_pause = False


            if self.static_is_last_goal_occupied_ and not self.is_processing_last_goal_occupied_pause and self.is_processing_replan_pause is False and self.is_processing_goal_occupied_pause is False:
                self.get_logger().warn("[check_collision_obstacle] last Goal is occupied. Forcing Pause.")
                self.is_processing_last_goal_occupied_pause = True
                self._pause_start_time = now
                self._publish_pause()
                return

            if self.is_processing_last_goal_occupied_pause:
                return # 최상위 로직이 실행 중이면 아래 로직은 무시함



            if self.check_static_is_goal_occupied_ ==  True :

                # [FIX] goal 제거가 진행되어 남은 goal 이 마지막 하나가 되면
                # last_goal 분기(100초 -> STOP -> 관제 abort)가 맡아야 한다.
                # 예전에는 :548 의 진입 조건에 is_processing_goal_occupied_pause 가
                # False 여야 한다는 항이 있어 넘어가지 못했고, 25초 replan 사이클에
                # 갇혀 에스컬레이션에 도달하지 못했다(livelock).
                if self.is_processing_goal_occupied_pause and self.static_is_last_goal_occupied_:
                    self.get_logger().warn(
                        "[check_collision_obstacle] 남은 goal 이 마지막 하나가 되었다. "
                        "goal_occupied -> last_goal_occupied 로 인계한다.")
                    self.is_processing_goal_occupied_pause = False
                    self.delay_after_replan_goal_occupied = False
                    self.delay_after_replan_start_time_goal_occupied = None
                    self._goal_occupied_false_start_time = None
                    self.published_goal_10s = False
                    self.published_goal_15s = False
                    self.published_goal_20s = False
                    self._clear_goal_removal_latch()
                    # _pause_start_time 은 last_goal 분기가 새로 잡는다
                    return

                ###### is_goal_occupied
                ## replan이후 1.0 대기 후 resume
                if self.delay_after_replan_goal_occupied and self.delay_after_replan_start_time_goal_occupied is not None and self.is_processing_replan_pause is False and self.is_processing_last_goal_occupied_pause is False:
                    elapsed_delay = (now - self.delay_after_replan_start_time_goal_occupied).nanoseconds * 1e-9
                    if elapsed_delay >= 1.0:
                        self.pub_cmd_resume.publish(Bool(data=False))
                        self._publish_state("[check_collision_obstacle] RUN (Delay After Replan)")  
                        self._pause_start_time = None
                        self.delay_after_replan_start_time_goal_occupied = None
                        self.is_processing_goal_occupied_pause = False
                        self.delay_after_replan_goal_occupied = False
                        self._goal_occupied_false_start_time = None
                        self._clear_goal_removal_latch()   # [FIX] 래치 수명 제한
                        return
                    return

                ## replan 등 로직
                if self.is_processing_goal_occupied_pause and self.static_is_goal_occupied_ is True and self.is_processing_replan_pause is False and self.is_processing_last_goal_occupied_pause is False:
                    self._goal_occupied_false_start_time = None
                    if self._pause_start_time is not None:
                        dt = (now - self._pause_start_time).nanoseconds * 1e-9
                        self.get_logger().info(f"[check_collision_obstacle]  Goal Occupied detected but pausing for {dt:.1f}s (within timeout threshold).", throttle_duration_sec=2.0)


                        if dt >= 10.0 and self.published_goal_10s is False:
                            self._publish_remove_passed_goals()
                            self.published_goal_10s = True
                            self._goal_occupied_false_start_time = None
                        if dt >= 15.0 and self.published_goal_15s is False:
                            self._publish_remove_first_goals()
                            self.published_goal_15s = True
                            self._goal_occupied_false_start_time = None  
                        if dt >= 20.0 and self.published_goal_20s is False:
                            self._publish_remove_first_goals()
                            self.published_goal_20s = True
                            self._goal_occupied_false_start_time = None  

                        if dt >= 25.0 :   
                            self.get_logger().warn(f"[check_collision_obstacle]  Goal Occupied detected timeout for {dt:.1f}s. Initiating resume sequence.")
                            if self.delay_after_replan_goal_occupied == False:
                                self._publish_replan()
                                self.delay_after_replan_goal_occupied = True
                                if self.delay_after_replan_start_time_goal_occupied is None:
                                    self.delay_after_replan_start_time_goal_occupied = now
                                    return


                        


                    # early resume
                if self.is_processing_goal_occupied_pause and self.static_is_goal_occupied_ is False and self.is_processing_replan_pause is False and self.is_processing_last_goal_occupied_pause is False:
                    if self._goal_occupied_false_start_time is None:
                        # 처음 False가 들어온 시간 기록
                        self._goal_occupied_false_start_time = now
                    else:
                        elapsed = (now - self._goal_occupied_false_start_time).nanoseconds * 1e-9
                        # M초(replan_clear_timeout_sec) 이상 연속으로 False가 들어오면
                        if elapsed >= self.replan_clear_timeout_sec:
                            if self.is_processing_goal_occupied_pause:
                                self.get_logger().warn(f"[check_collision_obstacle] Path clear for {elapsed:.1f}s. Aborting Pause Sequence & Early Resume!")
                                # self._abort_sequence_and_resume()
                                
                                # self._pre_moving_stop_type = MovingStopType.TYPE_NONE
                                
                                # 3. 주행 재개 신호 즉시 발행
                                self.pub_cmd_resume.publish(Bool(data=False))
                                
                                self._publish_state("[check_collision_obstacle] RUN (Early Resume)")  
                            
                                # 이미 조기 종료를 처리했으므로 시간 초기화 (중복 실행 방지)
                                self.static_is_goal_occupied_ = False
                                self._goal_occupied_false_start_time = None
                                self._pause_start_time = None
                                self.is_processing_goal_occupied_pause = False
                                self.published_goal_10s = False
                                self.published_goal_15s = False
                                self.published_goal_20s = False
                                self._clear_goal_removal_latch()   # [FIX] 래치 수명 제한


                if self.static_is_goal_occupied_ and not self.is_processing_goal_occupied_pause and self.is_processing_replan_pause is False and self.is_processing_last_goal_occupied_pause is False:
                    self.get_logger().warn("[check_collision_obstacle] Goal is occupied. Forcing Pause.")
                    self.is_processing_goal_occupied_pause = True
                    self._pause_start_time = now

                    self.published_goal_10s = False
                    self.published_goal_15s = False
                    self.published_goal_20s = False

                    self._publish_pause()
                    return

                if self.is_processing_goal_occupied_pause:
                    return # Goal 처리가 진행 중이면 일반 장애물 무시


        else:
            # [신규 추가] 로봇이 IDLE, SUCCEEDED, CANCELED 등이 되면 모든 Pause 상태 강제 초기화
            if self.is_processing_replan_pause or self.is_processing_goal_occupied_pause or self.is_processing_last_goal_occupied_pause:
                self.get_logger().info("[check_collision_obstacle] Robot status inactive. Resetting all obstacle SMs.")
                
                # self.pub_cmd_resume.publish(Bool(data=False))
                # self._publish_state("RUN (Status Reset)")

                self.is_processing_replan_pause = False
                self.delay_after_replan = False
                self._pause_start_time = None
                self._replan_flag_false_start_time = None
                self.delay_after_replan_start_time = None
                
                self.is_processing_goal_occupied_pause = False
                self.delay_after_replan_goal_occupied = False
                self._goal_occupied_false_start_time = None
                self.delay_after_replan_start_time_goal_occupied = None
                self.published_goal_10s = False
                self.published_goal_15s = False
                self.published_goal_20s = False
                
                self.is_processing_last_goal_occupied_pause = False
                self._last_goal_occupied_false_start_time = None
                self._clear_goal_removal_latch()   # [FIX] 래치 수명 제한
                self._static_reset_osc()           # [FIX] 진동 추적도 접는다
                self._agent_reset_osc()            # [FIX B-7] agent 재시도 추적도 접는다
            return



###### obstacle collision
        if self.delay_after_replan and self.delay_after_replan_start_time is not None and self.is_processing_goal_occupied_pause is False and self.is_processing_last_goal_occupied_pause is False:
            elapsed_delay = (now - self.delay_after_replan_start_time).nanoseconds * 1e-9
            if elapsed_delay >= 1.5:
                self.pub_cmd_resume.publish(Bool(data=False))
                self._publish_state("[check_collision_obstacle] RUN (Delay After Replan)")  
                self._pause_start_time = None
                self.delay_after_replan_start_time = None
                self.is_processing_replan_pause = False
                self.delay_after_replan = False
                self._replan_flag_false_start_time = None
                # [FIX] replan 이 나갔으니 누적 경과는 0 부터 다시 센다.
                # 다만 재시도 횟수는 유지한다 - 새 경로가 또 막히면 그게
                # "replan 이 효과 없었다" 는 증거이고, 그걸 세야 한다.
                # 로봇이 실제로 통과하면 다음 진입의 간격이 창을 넘어
                # _static_pause_start() 에서 자동으로 리셋된다.
                self._static_release(now, after_replan=True)
                return
            return


        if self.is_processing_replan_pause and self.replan_flag_status is True and self.is_processing_goal_occupied_pause is False and self.is_processing_last_goal_occupied_pause is False:
            self._replan_flag_false_start_time = None
            if self._pause_start_time is not None:
                dt = (now - self._pause_start_time).nanoseconds * 1e-9
                if dt < self.replan_pause_timeout_sec: 
                    self.get_logger().info(f"[check_collision_obstacle] Obstacle detected but pausing for {dt:.1f}s (within timeout threshold).", throttle_duration_sec=2.0)
                elif dt >= self.replan_pause_timeout_sec:
                    self.get_logger().warn(f"[check_collision_obstacle] Obstacle detected timeout for {dt:.1f}s. Initiating resume sequence.")
                    
                    if self.delay_after_replan == False:
                        # [FIX] 이 상황에서 몇 번째 replan 인가.
                        self._static_replan_retry += 1
                        if (self.static_max_replan_retry > 0
                                and self._static_replan_retry > self.static_max_replan_retry):
                            self.get_logger().error(
                                f"[check_collision_obstacle] 같은 상황에서 replan "
                                f"{self._static_replan_retry - 1}회가 모두 실패했다. "
                                f"자력 우회 불가로 판단해 관제에 보고한다.")
                            self.pub_cmd_stop.publish(UInt8(data=1))
                            self.nav_stop_complete_ = False
                            self._nav_stop_wait_start = now      # watchdog 무장
                            self._static_reset_osc()
                            self.is_processing_replan_pause = False
                            self._pause_start_time = None
                            self._replan_flag_false_start_time = None
                            return
                        self.get_logger().warn(
                            f"[check_collision_obstacle] replan 시도 "
                            f"{self._static_replan_retry}/{self.static_max_replan_retry}")
                        self._publish_replan()
                        self.delay_after_replan = True
                        if self.delay_after_replan_start_time is None:
                            self.delay_after_replan_start_time = now
                            return

        # 다시 resume 되는 대기 시간.
        if self.is_processing_replan_pause and self.replan_flag_status is False and self.delay_after_replan == False and self.is_processing_goal_occupied_pause is False and self.is_processing_last_goal_occupied_pause is False:
            if self._replan_flag_false_start_time is None:
                # 처음 False가 들어온 시간 기록
                self._replan_flag_false_start_time = now
            else:
                elapsed = (now - self._replan_flag_false_start_time).nanoseconds * 1e-9
                # M초(replan_clear_timeout_sec) 이상 연속으로 False가 들어오면
                if elapsed >= self.replan_clear_timeout_sec:
                    if self.is_processing_replan_pause:
                        self.get_logger().warn(f"[check_collision_obstacle] Path clear for {elapsed:.1f}s. Aborting Pause Sequence & Early Resume!")
                        # self._abort_sequence_and_resume()
                        
                        # self._pre_moving_stop_type = MovingStopType.TYPE_NONE
                        
                        # 3. 주행 재개 신호 즉시 발행
                        self.pub_cmd_resume.publish(Bool(data=False))
                        
                        self._publish_state("[check_collision_obstacle] RUN (Early Resume)")  
                    
                        # 이미 조기 종료를 처리했으므로 시간 초기화 (중복 실행 방지)
                        self._static_release(now)       # [FIX] 진동 추적
                        self._replan_flag_false_start_time = None
                        self._pause_start_time = None
                        self.is_processing_replan_pause = False
                        self.delay_after_replan = False
                        self.delay_after_replan_start_time = None


                

        if self.replan_flag_status is True and not self.is_processing_replan_pause and self.delay_after_replan == False and self.is_processing_goal_occupied_pause is False and self.is_processing_last_goal_occupied_pause is False:
            self.is_processing_replan_pause = True
            # [FIX] 진동이면 경과 시간을 이어받는다. 예전에는 무조건 now 여서
            # 재진입마다 타이머가 0 으로 돌아가 replan 에 영영 도달하지 못했다.
            self._pause_start_time = self._static_pause_start(now)
            self._publish_pause()
            return




    def check_collision_agent(self):
        """ Agent 충돌 예측에 대한 상태 머신 (20Hz 주기 실행) """
        now = self.get_clock().now()

        self._check_nav_stop_watchdog(now)  # [FIX] 통지 유실 시 강제 해제

        if self.nav_stop_complete_ == False:
            self.get_logger().info("[check_collision_agent] Halted due to nav_stop_complete_ == False", throttle_duration_sec=2.0)            
            return # STOP 명령 발행 후 주행 재개 대기 중 (STOP 시퀀스 우선 처리)

        if self.is_processing_replan_pause is True or self.is_processing_goal_occupied_pause is True or self.is_processing_last_goal_occupied_pause is True:
            self.get_logger().info("[check_collision_agent]: Halted due to static obstacle collision", throttle_duration_sec=2.0)
            return # Replan Pause 시퀀스 진행 중이면 Agent 충돌 상태 머신은 일시 중지 (우선순위 보장)


        # [Phase 3] Action 후 지연 대기 (M초)
        if self.delay_after_agent_action and self.delay_after_agent_start_time is not None:
            self._agent_clear_start_time = None # Early Exit 카운트 리셋

            elapsed_delay = (now - self.delay_after_agent_start_time).nanoseconds * 1e-9
            
            # Reroute면 1.0초, 그 외는 1.5초 등 유동적 할당 가능
            wait_m = 1.0 if self.current_agent_command == MovingCommand.REROUTE else 1.5
            if self.current_agent_command == MovingCommand.WAIT_SIMPLE_RESUME:
                wait_m = 0.0 # Replan/Reroute 안 하는 경우 바로 Resume
                
            self.get_logger().info(f"[check_collision_agent][Phase 3] Waiting for action stabilization... ({elapsed_delay:.1f}s / {wait_m:.1f}s)", throttle_duration_sec=0.5)


            if elapsed_delay >= wait_m:
                self.get_logger().warn(f"[check_collision_agent][Phase 3] Stabilization complete. Resuming! (Action: {self.current_agent_command.name})")
                self.pub_cmd_resume.publish(Bool(data=False))
                self._publish_state(f"[check_collision_agent] RUN ({self.current_agent_command.name} Done)")  
                self._agent_release(now, after_replan=True)   # [FIX B-7] 재진입이 간격을 재게
                
                self._agent_pause_start_time = None
                self.delay_after_agent_start_time = None
                self.is_processing_agent_pause = False
                self.delay_after_agent_action = False
                self._agent_clear_start_time = None
            return

        # [FIX] 잠금 대상이 바뀌면 재평가한다 (B-2).
        #
        # Phase 0 는 current_agent_command / agent_pause_timeout_sec 를 고정하는데
        # 시퀀스가 끝날 때까지 다시 보지 않았다. 막던 로봇이 비켜나고 다른 로봇이
        # 들어와도 agent_collision_status 는 True 로 유지되므로 Early Exit 이
        # 성립하지 않고(아래 Phase 1 이 매 tick _agent_clear_start_time 을 지운다),
        # 사라진 로봇 기준의 대기 시간을 끝까지 센다.
        #   - 낮은 ID -> 높은 ID: 2초면 될 것을 150초 서 있는다 (정체)
        #   - 높은 ID -> 낮은 ID: 양보해야 할 상대 쪽으로 조기 출발한다 (프로토콜 위반)
        # 이미 action 을 쏜 뒤(delay_after_agent_action)라면 그 시퀀스는 끝까지 둔다.
        #
        # [주의] Phase 0 로 되돌리면 안 된다. _agent_pause_start_time 이 0 으로
        # 리셋되므로, 두 로봇이 번갈아 대상으로 잡히면 대기 시간이 영영 쌓이지
        # 않아 시퀀스가 끝나지 않는다(실측: 0.3초 교대에서 25초간 resume 없음,
        # pause 71회 발행). 그래서 **경과 시간은 그대로 두고** 명령과 목표
        # 대기시간만 제자리에서 갱신한다. 이미 기다린 시간은 실제로 기다린
        # 시간이므로 새 대상에게도 그대로 인정한다.
        if (self.is_processing_agent_pause
                and self.agent_collision_status is True
                and self.delay_after_agent_action is False
                and self.latest_agent_target_id != self._locked_target_id):
            new_cmd, new_type = self._decide_for_current_target()
            old_cmd = self.current_agent_command
            self.get_logger().warn(
                f"[check_collision_agent] 잠금 대상 변경 "
                f"{self._locked_target_id} -> {self.latest_agent_target_id}. "
                f"재평가: {old_cmd.name} -> {new_cmd.name} "
                f"({self.agent_pause_timeout_sec:.1f}s -> "
                f"{self._pause_timeout_for(new_cmd):.1f}s, 경과시간 유지)")
            self._locked_target_id = self.latest_agent_target_id
            self.current_agent_command = new_cmd
            self.current_agent_stop_type = new_type
            self.agent_pause_timeout_sec = self._pause_timeout_for(new_cmd)
            self._agent_clear_start_time = None
            # _agent_pause_start_time 은 건드리지 않는다 (경과시간 보존)
            # [FIX B-7] replan 재시도 횟수는 **상대별** 이다. 대기 중 상대가 바뀌면
            # 이 경로는 _agent_pause_start 를 거치지 않으므로 여기서 직접 0 으로
            # 되돌린다. 안 그러면 앞 상대에게 쓴 횟수가 새 상대에게 넘어가
            # 엉뚱한 상대에게 관제 보고를 하게 된다 (단위시험에서 잡음).
            if self._agent_replan_retry:
                self.get_logger().info(
                    f"[check_collision_agent] 상대 변경으로 agent 재시도 추적 초기화 "
                    f"(직전 {self._agent_replan_retry}회).")
                self._agent_replan_retry = 0
                self._agent_last_elapsed = 0.0
            self._publish_state(
                f"{new_type.name}: PAUSE {self.agent_pause_timeout_sec:.1f}s (재평가)")
            # pause 는 이미 걸려 있으므로 다시 발행하지 않는다

        # [Phase 1 & 2] Pause 진행 중
        if self.is_processing_agent_pause and self.agent_collision_status is True:
            
            self._agent_clear_start_time = None # Early Exit 카운트 리셋
            
            if self._agent_pause_start_time is not None:
                dt = (now - self._agent_pause_start_time).nanoseconds * 1e-9
                if dt < self.agent_pause_timeout_sec: 
                    self.get_logger().info(f"[check_collision_agent][Phase 1] Agent Pause: {dt:.1f}s / {self.agent_pause_timeout_sec:.1f}s (Cmd: {self.current_agent_command.name})", throttle_duration_sec=1.0)
                    # self.get_logger().info(f"Agent Pause: {dt:.1f}s / {self.agent_pause_timeout_sec}s", throttle_duration_sec=2.0)
                else:
                    self.get_logger().error(f"[check_collision_agent][Phase 2] Agent Timeout reached! ({dt:.1f}s). Initiating Action: {self.current_agent_command.name}")
                    # self.get_logger().warn(f"Agent Timeout {dt:.1f}s. Initiating Action.")
                    if self.delay_after_agent_action == False:
                        cmd = self.current_agent_command
                        if cmd == MovingCommand.WAIT_SIMPLE_RESUME:
                            # Replan 없이 즉시 출발
                            self.get_logger().warn("[check_collision_agent][Phase 2] WAIT_SIMPLE_RESUME and resume triggered. Releasing lock immediately.")
                            self.pub_cmd_resume.publish(Bool(data=False))
                            self._publish_state("[check_collision_agent] RUN (Simple Resume)")
                            # 상태 완전 초기화
                            self.is_processing_agent_pause = False
                            self._agent_pause_start_time = None
                            self._agent_clear_start_time = None
                            return # 여기서 끝냄

                        # Reroute 또는 Replan 실행
                        if cmd == MovingCommand.REROUTE:
                            self.get_logger().warn(f"[check_collision_agent] [Phase 2] Publishing command : {cmd}")
                            # self._publish_reroute()
                            self.pub_cmd_stop.publish(UInt8(data=1))
                        else:
                            # [FIX B-7] 같은 상대에게 몇 번째 replan 인가. 상한을 넘으면
                            # 정적 쪽과 같이 내부 정지로 관제에 넘긴다 (driving_abort).
                            self._agent_replan_retry += 1
                            if (self.agent_max_replan_retry > 0
                                    and self._agent_replan_retry > self.agent_max_replan_retry):
                                self.get_logger().error(
                                    f"[check_collision_agent] 상대 {self._locked_target_id} 에게 "
                                    f"agent replan 상한 도달 ({self._agent_replan_retry - 1}회 실패). "
                                    f"자력 해결 불가로 판단해 관제에 보고한다.")
                                self.pub_cmd_stop.publish(UInt8(data=1))
                                self.nav_stop_complete_ = False
                                self._nav_stop_wait_start = now      # watchdog 무장
                                self._publish_state(
                                    f"[check_collision_agent] ABORT (agent replan 상한 {self.agent_max_replan_retry})")
                                self._agent_reset_osc()
                                self.is_processing_agent_pause = False
                                self._agent_pause_start_time = None
                                self._agent_clear_start_time = None
                                return
                            self.get_logger().warn(
                                f"[check_collision_agent] agent replan 시도 "
                                f"{self._agent_replan_retry}/{self.agent_max_replan_retry} "
                                f"(상대 {self._locked_target_id})")
                            self.get_logger().warn(f"[check_collision_agent] [Phase 2] Publishing command : {cmd}")
                            self._publish_replan()
                            
                        self.delay_after_agent_action = True
                        if self.delay_after_agent_start_time is None:
                            self.delay_after_agent_start_time = now

                        return





        # [Phase 0 -> 조기 종료] 연속 False 판정 (Early Exit)
        if self.is_processing_agent_pause and self.agent_collision_status is False and self.delay_after_agent_action == False:
            if self._agent_clear_start_time is None:
                self.get_logger().info("[check_collision_agent] [Early Exit] Obstacle disappeared. Starting clear timer...")
                self._agent_clear_start_time = now
            else:
                elapsed = (now - self._agent_clear_start_time).nanoseconds * 1e-9
                self.get_logger().info(f"[check_collision_agent] [Early Exit] Clear timer: {elapsed:.1f}s / {self.agent_wait_before_resume:.1f}s", throttle_duration_sec=0.5)
                # self.agent_wait_before_resume(3.0초) 이상 비연속 충돌일 경우
                
                if elapsed >= self.agent_wait_before_resume: # 3.0초 사용 
                    self.get_logger().error(f"[check_collision_agent] [Early Exit] Agent path clear for {elapsed:.1f}s. Early Resume after waiting {self.agent_wait_before_resume}s!")
                    # self.get_logger().warn(f"Agent path clear for {elapsed:.1f}s. Early Resume after waiting {self.agent_wait_before_resume}s!")
                    self.pub_cmd_resume.publish(Bool(data=False))
                    self._publish_state("[check_collision_agent] RUN (Agent Early Resume)")  
                    self._agent_release(now, after_replan=False)  # [FIX B-7] 경과 이어받기용
                
                    self._agent_clear_start_time = None
                    self._agent_pause_start_time = None
                    self.is_processing_agent_pause = False
                    self.delay_after_agent_action = False
                    self.delay_after_agent_start_time = None

        # [Phase 0 -> 신규 진입] 새로운 장애물 발견 시
        if self.agent_collision_status is True and not self.is_processing_agent_pause and self.delay_after_agent_action == False:
            # 시퀀스 진입 직전에 최신 데이터를 바탕으로 의사결정을 수행하여 변수 고정 (Locking)
            self.get_logger().warn(f"[check_collision_agent] [Phase 0] New Agent Collision Detected! target_id: {self.latest_agent_target_id}, xy: {self.latest_agent_collision_xy}")
            # self.get_logger().warn(f"self.latest_agent_target_id : {self.latest_agent_target_id}, self.agent_collision_status : {self.agent_collision_status}, self.latest_agent_collision_xy: {self.latest_agent_collision_xy}")
            if self.latest_agent_target_id == 0:
                
                cmd = MovingCommand.WAIT
                stop_type = MovingStopType.TYPE_11
                # self.get_logger().warn(f"self.latest_agent_target_id == 0 , n_check_complete : {cmd}, moving_stop_type : {stop_type}")
                self.get_logger().warn(f"[check_collision_agent] [Phase 0] target_id is 0. Fallback to WAIT (TYPE_11).")

            else:
                cmd, stop_type = self._decide_obstacle_action(
                    self.latest_agent_target_id, 
                    self.latest_agent_collision_xy
                )
                self.get_logger().info(f"[check_collision_agent] [Phase 0] Decision Maker output -> Cmd: {cmd.name}, StopType: {stop_type.name}")

            # 결정된 명령을 전역 변수에 고정 (시퀀스가 끝날 때까지 바뀌지 않음)
            self.current_agent_command = cmd
            self.current_agent_stop_type = stop_type            
            
            
            # 시퀀스 잠금 시작
            self.is_processing_agent_pause = True
            self._locked_target_id = self.latest_agent_target_id  # [FIX] 재평가 기준
            # [FIX B-7] 같은 상대에게 짧은 간격으로 다시 걸린 것이면 경과·재시도를 이어받는다
            self._agent_pause_start_time = self._agent_pause_start(now, self._locked_target_id)
            
            # 대기 시간(N초) 매핑
            n_pause = self._pause_timeout_for(self.current_agent_command)
            self.agent_pause_timeout_sec = n_pause
            
            self.get_logger().error(f"[check_collision_agent] [Phase 0] Sequence Locked. Starting PAUSE for {n_pause}s.")
            self._publish_pause()
            self._publish_state(f"{self.current_agent_stop_type.name}: PAUSE {n_pause}s")




    def _pause_timeout_for(self, cmd: MovingCommand) -> float:
        """ 명령별 대기 시간(N초). Phase 0 진입과 대상 재평가가 함께 쓴다. """
        if cmd == MovingCommand.REROUTE: return 5.0
        if cmd == MovingCommand.WAIT_DETECT_AMR: return self.wait_detect_sec
        if cmd == MovingCommand.WAIT_OHTHER_AMR:
            return self.wait_other_long_sec if self.use_reroute else self.wait_other_short_sec
        if cmd == MovingCommand.WAIT_ABNORMAL:
            return self.wait_abnormal_long_sec if self.use_reroute else self.wait_abnormal_short_sec
        if cmd == MovingCommand.WAIT: return self.wait_obstacle_sec
        if cmd == MovingCommand.WAIT_SIMPLE_REPLAN: return 3.0
        if cmd == MovingCommand.WAIT_SIMPLE_RESUME: return 6.0
        return 0.0

    def _decide_for_current_target(self):
        """ 지금 잡혀 있는 대상으로 명령/타입을 결정한다. Phase 0 와 재평가가 공유. """
        if self.latest_agent_target_id == 0:
            return MovingCommand.WAIT, MovingStopType.TYPE_11
        return self._decide_obstacle_action(self.latest_agent_target_id,
                                            self.latest_agent_collision_xy)

    def on_collision(self, msg: PathAgentCollisionInfo):
        """ 센서처럼 주기적으로 들어오는 Agent 충돌 정보 업데이트 """
        self._last_collision_msg_time = self.get_clock().now()
        self._last_agent_event_time = self.get_clock().now()

        if self.nav_stop_complete_ == False:
            self.get_logger().info("[on_collision] Ignored: Waiting for nav_stop_complete_ to be True.", throttle_duration_sec=2.0)
            self.replan_flag_status = False
            self.static_is_last_goal_occupied_ = False
            self.agent_collision_status = False            
            return # STOP 명령 발행 후 주행 재개 대기 중 (STOP 시퀀스 우선 처리)


        # 1. "non_collision" 이거나 x 좌표가 비어있으면 장애물 없음(False)으로 처리
        is_clear = False
        if not msg.x:
            is_clear = True
            self.get_logger().info("[on_collision] Clear: msg.x is empty.", throttle_duration_sec=2.0)
        elif msg.note and "non_collision" in msg.note[0]:
            is_clear = True
            self.get_logger().info("[on_collision] Clear: 'non_collision' note received.", throttle_duration_sec=2.0)

        if is_clear:
            if self.agent_collision_status is True:
                self.get_logger().info("[on_collision] Agent collision status changed to FALSE (Path Clear).")
            self.agent_collision_status = False
            return

        # 2. 장애물이 있을 때 (True)
        if 0 in msg.machine_id: 
            self.get_logger().warn("[on_collision] Warning: '0' found in msg.machine_id! Treating as normal obstacle.", throttle_duration_sec=5.0)
        target_id = int(msg.machine_id[0]) if msg.machine_id else 0
        
        # 내 자신이면 무시
        if target_id == self.my_id:
            self.get_logger().info(f"[on_collision] Ignored: Target ID ({target_id}) is myself.", throttle_duration_sec=2.0)
            self.agent_collision_status = False
            return

        collision_x = float(msg.x[0]) if msg.x else 0.0
        collision_y = float(msg.y[0]) if msg.y else 0.0


        # [FIX] 비교를 대입보다 먼저 한다.
        # 예전에는 latest_agent_target_id 에 target_id 를 넣은 다음
        # latest_agent_target_id != target_id 를 검사해서 항상 False 였다.
        # 그래서 "타겟 ID가 바뀌었을 때" 경고가 한 번도 뜨지 않았고,
        # 대상 전환이 현장 로그에 아무 흔적을 남기지 않았다.
        target_changed = (self.latest_agent_target_id != target_id)

        # 의사결정은 여기서 하지 않고 Raw Data만 갱신
        self.latest_agent_target_id = target_id
        self.latest_agent_collision_xy = (collision_x, collision_y)

        # 상태가 False -> True로 바뀌는 순간이거나, 타겟 ID가 바뀌었을 때 로깅
        if not self.agent_collision_status or target_changed:
            self.get_logger().warn(f"[on_collision] New Agent Collision Data Cached -> Target ID: {target_id}, XY: ({collision_x:.2f}, {collision_y:.2f})")

        self.agent_collision_status = True



        

        # if target_id == 0:
        #     command = MovingCommand.WAIT
        #     stop_type = MovingStopType.TYPE_11
        # else:
        #     command, stop_type = self._decide_obstacle_action(target_id, (collision_x, collision_y))

        # # 상태 업데이트
        # self.agent_collision_status = True
        # self.current_agent_command = command
        # self.current_agent_stop_type = stop_type



    # ------------------------------------------------------------------
    # Core Logic
    # ------------------------------------------------------------------
    def _decide_obstacle_action(self, target_id: int, collision_xy: Tuple[float, float]) -> Tuple[MovingCommand, MovingStopType]:
        
        n_check_complete = MovingCommand.WAIT
        moving_stop_type = MovingStopType.TYPE_NONE

        # ---------------------------------------------------------
        # [수정] Simple Mode 로직: ID 비교에 따른 분기
        # ---------------------------------------------------------
        if self.simple_mode:
            if self.my_id > target_id: 
                # ID가 큰 녀석: 대기 후 Replan 해서 감
                return MovingCommand.WAIT_SIMPLE_REPLAN, MovingStopType.TYPE_4
            else:
                # ID가 작은 녀석: 대기만 하고 바로 Resume (Replan 안 함)
                return MovingCommand.WAIT_SIMPLE_RESUME, MovingStopType.TYPE_4
        # ---------------------------------------------------------


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
        elif self._check_vehicle_immobile(agent):
            # [FIX] 기다려도 비켜주지 않는 상대다. ID 우선순위를 건너뛰고
            # 일반 장애물로 격하해 우회한다.
            self.get_logger().warn(
                f"[decide_obstacle_action] agent {agent.machine_id} 가 스스로 움직일 수 "
                f"없는 상태(phase={agent.status.phase}). 일반 장애물로 격하한다.",
                throttle_duration_sec=2.0)
            n_check_complete = MovingCommand.WAIT
            moving_stop_type = MovingStopType.TYPE_11
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
        
        self.get_logger().warn(f"[decide_obstacle_action] result, n_check_complete : {n_check_complete}, moving_stop_type : {moving_stop_type}")
        
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

    def _check_vehicle_immobile(self, agent: MultiAgentInfo) -> bool:
        """ [FIX] 상대가 스스로는 영영 못 움직이는 상태인가. (immobile_agent_phases) """
        return int(agent.status.phase) in self._immobile_phases

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
    # Pub/Sub Utils
    # ------------------------------------------------------------------
    def _publish_pause(self): self.pub_cmd_pause.publish(Bool(data=True))
    def _publish_replan(self): self.pub_req_replan.publish(Bool(data=True))
    def _publish_reroute(self): self.pub_req_reroute.publish(Bool(data=True))
    def _publish_state(self, txt: str): self.pub_state.publish(String(data=txt))
    def _publish_remove_passed_goals(self): self.pub_rmv_passed_goals.publish(Bool(data=True))
    def _publish_remove_first_goals(self): self.pub_rmv_first_goals.publish(Bool(data=True))

    def _static_release(self, now: Time, after_replan: bool = False) -> None:
        """[FIX] 정적 pause 를 풀 때 호출. 다음 진입이 간격을 잴 수 있게 기록한다.

        after_replan=True 면 replan 이 나간 뒤의 해제다. replan 으로 경로가
        바뀌었으니 **누적 경과는 0 부터** 다시 센다. 다만 replan 재시도 횟수
        (_static_replan_retry)는 유지한다 - 그게 "이 상황에서 몇 번 시도했나" 다.
        """
        if after_replan:
            self._static_last_elapsed = 0.0
        elif self._pause_start_time is not None:
            self._static_last_elapsed = (now - self._pause_start_time).nanoseconds * 1e-9
        self._static_last_release_t = now

    def _static_pause_start(self, now: Time) -> Time:
        """[FIX] 정적 pause 진입 시각을 정한다.

        직전 해제로부터 static_rejoin_window_sec 이내면 '같은 상황에 다시 튕긴 것'
        으로 보고 경과 시간을 이어받는다. 그래야 진동해도 누적이 쌓여
        replan_pause_timeout_sec 에 도달하고 상황이 실제로 바뀐다.

        창보다 오래 비었으면 **다른 상황**이다. 경과와 replan 재시도 횟수를
        모두 0 으로 되돌린다. 로봇이 실제로 그 자리를 통과했다면 재차단이
        일어나지 않거나 간격이 길어지므로 여기서 자동으로 리셋된다.
        """
        if (self.static_rejoin_window_sec > 0.0
                and self._static_last_release_t is not None):
            gap = (now - self._static_last_release_t).nanoseconds * 1e-9
            if 0.0 <= gap <= self.static_rejoin_window_sec:
                if self._static_last_elapsed > 0.0:
                    self.get_logger().warn(
                        f"[check_collision_obstacle] 짧은 간격({gap:.1f}s) 재차단 - "
                        f"같은 상황으로 본다. 경과 {self._static_last_elapsed:.1f}s 이어받는다 "
                        f"(replan 재시도 {self._static_replan_retry}회).")
                    return now - Duration(seconds=self._static_last_elapsed)
                # replan 직후 재차단: 경과는 0 이지만 재시도 맥락은 이어진다
                return now

        # 창 밖 = 다른 상황. 전부 초기화한다.
        if self._static_replan_retry or self._static_last_elapsed > 0.0:
            self.get_logger().info(
                f"[check_collision_obstacle] 재차단 간격이 충분히 길다 - 다른 상황으로 본다. "
                f"진동 추적 초기화 (직전 replan 재시도 {self._static_replan_retry}회).")
        self._static_replan_retry = 0
        self._static_last_elapsed = 0.0
        return now

    def _static_reset_osc(self) -> None:
        """[FIX] 정적 진동 추적을 완전히 지운다. goal 이 끝났을 때 등."""
        self._static_last_release_t = None
        self._static_last_elapsed = 0.0
        self._static_replan_retry = 0

    # ---------------- [FIX B-7] agent 쪽 재진입/재시도 추적 (정적 쪽과 같은 형태) ----------------
    def _agent_release(self, now: Time, after_replan: bool = False) -> None:
        """agent pause 를 풀 때 호출. 다음 진입이 간격을 잴 수 있게 기록한다.
        after_replan=True 면 replan 뒤의 해제라 누적 경과는 0 부터, 재시도 횟수는 유지."""
        if after_replan:
            self._agent_last_elapsed = 0.0
        elif self._agent_pause_start_time is not None:
            self._agent_last_elapsed = (now - self._agent_pause_start_time).nanoseconds * 1e-9
        self._agent_last_release_t = now
        self._agent_last_target_id = self._locked_target_id

    def _agent_pause_start(self, now: Time, target_id: int) -> Time:
        """agent pause 진입 시각을 정한다.

        직전 해제로부터 agent_rejoin_window_sec 이내이고 **같은 상대**면 같은 상황으로
        보고 경과 시간(Early Exit 뒤)과 replan 재시도 횟수를 이어받는다. 상대가 다르거나
        창을 넘겼으면 다른 상황이라 둘 다 0 으로 되돌린다 — 그래서 서로 다른 상대·장소의
        시도가 누적되지 않는다.
        """
        if (self.agent_rejoin_window_sec > 0.0
                and self._agent_last_release_t is not None
                and self._agent_last_target_id == target_id):
            gap = (now - self._agent_last_release_t).nanoseconds * 1e-9
            if 0.0 <= gap <= self.agent_rejoin_window_sec:
                if self._agent_last_elapsed > 0.0:
                    self.get_logger().warn(
                        f"[check_collision_agent] 짧은 간격({gap:.1f}s) 같은 상대({target_id}) 재차단 - "
                        f"경과 {self._agent_last_elapsed:.1f}s 이어받는다 "
                        f"(agent replan 재시도 {self._agent_replan_retry}회).")
                    return now - Duration(seconds=self._agent_last_elapsed)
                # replan 직후 재차단: 경과는 0 이지만 재시도 맥락은 이어진다
                return now
        if self._agent_replan_retry or self._agent_last_elapsed > 0.0:
            self.get_logger().info(
                f"[check_collision_agent] 상대가 바뀌었거나 간격이 길다 - 다른 상황으로 본다. "
                f"agent 재시도 추적 초기화 (직전 {self._agent_replan_retry}회, "
                f"상대 {self._agent_last_target_id} -> {target_id}).")
        self._agent_replan_retry = 0
        self._agent_last_elapsed = 0.0
        return now

    def _agent_reset_osc(self) -> None:
        """agent 재시도 추적을 완전히 지운다. goal 이 끝났을 때, 관제 보고 직후."""
        self._agent_last_release_t = None
        self._agent_last_elapsed = 0.0
        self._agent_replan_retry = 0
        self._agent_last_target_id = None

    def _clear_goal_removal_latch(self):
        """[FIX] goal 제거 신호의 래치를 내린다.

        두 토픽 모두 TRANSIENT_LOCAL 로 True 만 발행되고 False 를 쏘는 곳이
        없었다. BT 가 PauseBranch 밖(회복 등)이면 소비되지 않고 래치에 남았다가
        나중에 무관한 pause 에서 실행되어 goal 이 조용히 사라진다.
        시퀀스가 끝날 때마다 명시적으로 내려 수명을 시퀀스 안으로 가둔다.
        """
        self.pub_rmv_passed_goals.publish(Bool(data=False))
        self.pub_rmv_first_goals.publish(Bool(data=False))



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
    # 4개의 스레드를 사용하는 MultiThreadedExecutor 생성
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
