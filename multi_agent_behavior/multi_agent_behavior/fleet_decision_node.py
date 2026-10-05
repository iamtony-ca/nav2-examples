#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import sys
from functools import partial # 파일 최상단에 추가하세요
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from std_msgs.msg import Bool, String, UInt8, UInt16
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, PoseStamped, PolygonStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from docking_monitoring_msgs.msg import DockingMonitoring   # [OP 09-28] 도킹 중이면 fleet 기동 금지
from robot_interfaces.msg import PathAgentCollisionInfo, PathStaticCollisionInfo
from robot_interfaces.msg import ModifierControl
# [V2] 자기 자세는 관제 레코드가 아니라 TF 에서 얻는다 (M-9)
import tf2_ros
from collections import Counter, deque
from robot_interfaces.msg import MultiAgentInfoArray, MultiAgentInfo, AgentStatus
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.action import ActionClient
from nav2_msgs.action import BackUp, Spin, DriveOnHeading


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

        # =====================================================================
        # [V2] 정책 v2 파라미터 (FLEET_DECISION_POLICY_PROPOSAL.md 8장).
        # v2_enable=false 면 아래 전부가 비활성이고 v1(현장 amhs) 과 같은 동작을 한다.
        # =====================================================================
        self.declare_parameter("v2_enable", False)   # [amhs 이관 S2 09-27] 기본 끔 = 현행(v1) 동작
        self.v2_enable = bool(self.get_parameter("v2_enable").value)
        # (1) 정지 대신 감속 대기. 0 이면 v1 처럼 정지.
        self.declare_parameter("slow_wait_speed_mps", 0.1)
        self.declare_parameter("slow_wait_angular_rps", 0.3)
        self.declare_parameter("slow_wait_min_dist_m", 0.9)   # 충돌점이 이보다 가까우면 정지
        self.declare_parameter("slow_wait_max_sec", 30.0)     # 우선 측이 감속 대기하는 상한 → 정지+replan
        self.declare_parameter("slow_clear_sec", 1.5)         # 감속 대기 중 해제 확인 시간
        # 같은 방향 주행 상대에게 속도 맞추기 (TYPE_2 의 자리)
        self.declare_parameter("speed_match_enable", True)
        self.declare_parameter("speed_match_ratio", 0.9)
        self.declare_parameter("speed_match_min_mps", 0.1)
        # 이웃 속도는 관제 twist 가 아니라 자세 이력으로 추정한다 (현장 bridge 는 twist 를 0 으로 채운다)
        self.declare_parameter("agent_speed_window_sec", 1.0)
        self.declare_parameter("agent_moving_mps", 0.05)
        # 정지 상대 분류별 대기 (3장). 기본은 보수적(늦게 회피)
        self.declare_parameter("stopped_yielding_wait_sec", 60.0)
        self.declare_parameter("stopped_working_wait_sec", 60.0)
        self.declare_parameter("yielding_phases", [15, 2, 1, 16, 17])
        self.declare_parameter("working_phases", [4, 5, 6, 7, 8, 9, 10, 11])
        # 상대(우선)가 PAUSE 이고 그 경로가 내 발자국을 지나면 = 나를 기다리는 것 → 내가 비켜준다 (PIBT 상속)
        self.declare_parameter("mutual_block_replan_sec", 10.0)
        self.declare_parameter("mutual_block_radius_m", 0.5)
        # [S8 v2 1차] 우선 로봇이 내 옆(near_m 안)에서 정지·회복만 반복하면(경로 HIT 는 없음, M-8)
        # 그것도 "나 때문에 막힘" 으로 본다 → mutual_block_replan_sec 뒤 내가 비켜준다.
        self.declare_parameter("mutual_block_near_m", 1.5)
        # 상대 몸체 중심이 이보다 가까우면 감속 대기 대신 정지 (충돌점 거리만 보면 옆구리로 붙는다)
        self.declare_parameter("slow_wait_min_body_m", 1.2)
        # [S8 v2 2차] 예측 감속(예약 영역 앞 대기의 국소 구현). 두 로봇이 같은 시각에 교차점에 닿게 되면
        # 우선순위가 낮은 쪽이 validator HIT 가 나기 **전에** 미리 감속한다. 2차에서 동시 도착 → 0.67 m 대면 → 152 s 교착.
        self.declare_parameter("predict_enable", True)
        self.declare_parameter("predict_horizon_sec", 8.0)
        self.declare_parameter("predict_gap_sec", 3.0)
        self.declare_parameter("predict_conflict_dist_m", 0.8)
        self.declare_parameter("predict_debug", True)
        # [S18 통로망] agent 대기 만료 시 replan 을 쏘면 고리형 통로망에서 동적 플래너가 다른 통로로 크게 돌아가는 경로를 내고
        # 정적 기준 경로와 1.6 m 이상 벗어나 403 → abort → 관제 DETOUR 가 반복된다. false 면 agent 대기 만료에 replan 대신
        # 대기를 이어간다(상대가 비켜주거나 내가 후진 양보). 정적 장애물 replan·goal 점유 처리는 그대로.
        self.declare_parameter("agent_replan_enable", True)
        # [S8 v2 3차] 대면 교착(몸체 < 0.9 m, 서로 replan 만 반복) 은 replan 으로 못 푼다. 낮은 쪽이 뒤로 물러난다.
        # 제안서 기본은 false 지만 sim 실험군에서는 켠다. cmd_vel_adjusted 로 -backoff_speed 를 backoff_m 만큼 낸다.
        self.declare_parameter("yield_backoff_enable", True)
        self.declare_parameter("yield_backoff_m", 1.0)
        self.declare_parameter("yield_backoff_speed_mps", 0.1)
        self.declare_parameter("yield_backoff_body_m", 1.0)     # 상대 몸체가 이보다 가깝고 앞쪽이면 후진
        self.declare_parameter("topic_cmd_vel_out", "/cmd_vel_adjusted")   # (구) 직접 후진용, 이제 미사용
        # [사용자 지시 09-19] 후진 양보는 Nav2 behavior_server 의 BackUp 액션으로 — 로컬 costmap 으로 **후방 충돌 검사**를 하며 물러난다.
        self.declare_parameter("backup_action_name", "backup")
        self.declare_parameter("backup_time_allowance_sec", 20.0)
        # [V2.1 09-20] 대기 순환(wait-for cycle) 검출: 내가 기다리는 상대 → 그 상대가 기다리는 상대 → … 가 나로 돌아오면
        # 순환 교착이다 (S18 66완주 판의 D1: r1→r2→r3→r1, 각자 남의 goal 위에 서서 100 s 뒤 STOP). 간선은 관제 cross_agent_id
        # (있으면) 또는 기하(정지 로봇의 전방 부채꼴 안 가장 가까운 로봇). 순환이 cycle_min_wait_sec 지속되면 우선순위가 가장
        # 낮은 구성원부터 cycle_stagger_sec 간격으로 BackUp(후방 충돌 검사) 으로 물러나 고리를 끊는다. 모두 같은 입력으로
        # 같은 순번을 계산하므로 통신 없이 결정적이다.
        self.declare_parameter("cycle_detect_enable", False)
        self.declare_parameter("cycle_max_len", 4)
        self.declare_parameter("cycle_front_deg", 70.0)
        self.declare_parameter("cycle_front_m", 2.5)
        self.declare_parameter("cycle_goal_search_m", 4.0)     # goal 점유 대기 중 '내가 기다리는 상대' 탐색 반경
        self.declare_parameter("cycle_min_wait_sec", 5.0)
        self.declare_parameter("cycle_stagger_sec", 8.0)
        self.declare_parameter("cycle_backoff_m", 1.2)
        self.declare_parameter("cycle_retry_sec", 60.0)        # 같은 순환에 대한 내 후진 재시도 간격
        # [V2.1 09-20] 밀어내기 양보(push yield, PIBT 의 priority inheritance): 우선 로봇이 "나 때문에 막혔다"(관제 cross_agent_id
        # = 나, 또는 정지한 채 나를 마주봄) 고 하는데 나는 push_min_sec 동안 무진전이면 — 내가 조정 대기 중이 아니어도(플래너
        # 실패·취소 반복 등) — **내 주행 이력을 따라** push_retreat_m 물러난다 (온 길은 벽이 없다는 걸 안다). 방향에 따라 BackUp
        # (바로 뒤) / Spin+DriveOnHeading (돌아서 감). 40분 롱런의 r1↔r4 11분 교착(Dubins 플래너라 우회 불가) 대책.
        self.declare_parameter("push_yield_enable", False)
        # [V2.11] 막힌 이웃 전용 양보 (관제 cross_agent_id 없이도 동작)
        self.declare_parameter("blocked_yield_enable", True)
        self.declare_parameter("blocked_yield_m", 3.5)
        self.declare_parameter("blocked_yield_sec", 20.0)
        self.declare_parameter("push_min_sec", 10.0)
        self.declare_parameter("push_retreat_m", 2.0)
        self.declare_parameter("push_cooldown_sec", 60.0)
        self.declare_parameter("retreat_speed_mps", 0.15)
        self.declare_parameter("spin_action_name", "spin")
        self.declare_parameter("drive_action_name", "drive_on_heading")
        # [사용자 09-20] 후진/후퇴 기동은 "후방 장애물을 명확히 감지하고 stop and go": behavior_server 가 장애물로 중단(abort)하면
        # 실패로 끝내지 않고 **정지한 채 기다렸다가 비면 남은 거리만 다시** 간다 (retry_sec 간격, max_sec 까지). 안전 최우선.
        self.declare_parameter("maneuver_stopgo_enable", True)
        self.declare_parameter("maneuver_stopgo_retry_sec", 2.0)
        self.declare_parameter("maneuver_stopgo_max_sec", 30.0)
        # [V2.22 09-23] 후퇴가 거부되는 이유를 셀 단위로 재 보니, 뒤쪽 ±60° 안 최근접 로봇은 1.26 m 이상이고
        # 정적 지도도 뒤 1.2 m 가 비어 있는데 BackUp 이 384/384 거부됐다(모두 COLLISION_AHEAD). 즉 막은 것은
        # 로봇도 벽도 아니고 **local costmap 에 남은 표식**이다. 재시도 직전에 local costmap 을 비워
        # 라이다가 다시 채우게 한다 — 진짜 장애물이면 한 스캔 만에 다시 찍히므로 충돌 검사는 그대로 유효하다.
        self.declare_parameter("retreat_clear_costmap", False)   # 기본 끔 — v222 config 에서만 켠다 (진행 중인 비교판 오염 방지)
        self.declare_parameter("retreat_clear_cooldown_sec", 4.0)
        # [V2.23 09-23] **후방 여유를 직접 재고** 막혀 있으면 후퇴를 아예 보내지 않는다.
        # 계측: 1.8 m 복도에서는 footprint 비용이 늘 99(inscribed) 이고, lethal 셀 하나가 더해지는 순간
        # 100 이 되어 BackUp 이 거부된다. 여유가 1 등급뿐이라 "일단 보내고 30 s 기다리기" 는 늘 헛수고다.
        # 판정은 nav2_behaviors 와 같은 기준(>= LETHAL) 을 OccupancyGrid 단위(100) 로 쓴다.
        self.declare_parameter("rear_check_enable", False)
        self.declare_parameter("rear_check_margin_m", 0.05)   # 요청 거리 + 여유만큼 앞서 본다
        self.declare_parameter("rear_check_max_age_sec", 3.0)
        # [V2.24 09-23] goal 점유 대기 중에도 **후방이 비어 있으면** 밀어내기를 허용한다.
        # 계측(T1 06:26): 얼어붙은 4대 중 3대(r1/r2/r4) 는 후방 0.30 m 까지 lethal 셀이 **한 표본도 없었다**.
        # 물리적으로 막힌 게 아니라 navigation_manager 의 goal 점유 대기에 걸려 아무 기동도 안 낸 것이다.
        # 기존에는 `is_processing_goal_occupied_pause` 면 push 를 통째로 건너뛰어(순환이 안 잡히면 아무 규칙도 없음)
        # 37분간 규칙 발화가 0 이었다.
        self.declare_parameter("push_during_goal_wait", False)
        self.declare_parameter("push_during_goal_wait_sec", 40.0)
        # [V2.25 09-23] **줄 해소의 교착.** S1 실측: 다섯 대가 33분간 "줄 해소 순서가 아니다" 를
        # **752회** 찍고 실제 기동은 4회였다. 게다가 로봇마다 순서를 다르게 계산했다
        # (`1>3>2>4>5` vs `5>3>1>2>4`) — `_line_order` 는 "모두가 같은 입력으로 같은 순서에 도달한다" 는
        # 전제인데, 이웃 정보가 로봇마다 달라 그 전제가 깨진다. 서로 상대 차례라고 믿으며 아무도 안 움직인다.
        # 고치는 방향은 **합의를 없애는 것**이다:
        #   (1) 후방이 막힌 로봇은 애초에 차례가 아니다 — 움직일 수 없는 로봇을 기다리면 전체가 멈춘다.
        #   (2) 줄이 line_escape_sec 이상 지속되면 **순서를 무시**하고, 후방이 빈 로봇은 그냥 움직인다.
        # 둘 다 각자 **자기 후방만** 보면 되므로 이웃 정보 불일치에 영향받지 않는다.
        self.declare_parameter("line_rear_first", False)
        self.declare_parameter("line_escape_sec", 45.0)
        self.declare_parameter("line_rear_probe_m", 0.4)
        # [V2.26 09-24] **교차로 안에서 멈추지 않는다.** E1 실측: 28분 정지 구간 내내
        # r2 와 r5 가 **교차로 사각형 안에 100 %** 서 있었고(표본 1786/1786), 그 결과 403 의 93 %가
        # 교차로 부근에서 났다. 교차로 하나가 막히면 사방이 함께 죽는다 — 22대에서 가장 위험한 형태다.
        # 관제는 "구간 안 로봇은 세우지 않는다" 를 지키지만, **로봇이 앞이 막혀 스스로 멈추는 것**은
        # 막지 못한다. 그래서 로봇 쪽에서 막는다: **출구가 비어 있을 때만 교차로에 들어간다.**
        # 판단 재료는 현장 신호 그대로 — 내 `area_id`(흐름제어가 채움) 와 이웃 위치다.
        self.declare_parameter("junction_exit_enable", False)
        self.declare_parameter("junction_exit_probe_m", 2.5)    # 내 경로에서 이만큼 앞을 출구로 본다
        self.declare_parameter("junction_exit_clear_m", 0.8)    # 출구에 이웃이 이보다 가까우면 진입 보류
        self.declare_parameter("junction_exit_max_hold_sec", 30.0)  # 무한 보류 방지
        # [V2.26b 09-24] **이미 교차로 안에서 멈췄으면 빠져나온다.** G2 실측: 두 대가 교차로 안에서
        # 출발해 시작 직후 엉켰고, V2.26 은 '구간 밖에서 접근하는 로봇' 만 막으므로 발화 0 이었다.
        # 예방(V2.26) 만으로는 이미 갇힌 경우를 못 푼다. 들어온 길은 대개 비어 있으므로 이력을 따라 물러난다.
        # 후진 안전 지침 그대로: 후방 실측(_rear_blocked) 이 막혔다고 하면 하지 않는다.
        self.declare_parameter("junction_escape_enable", False)
        self.declare_parameter("junction_escape_after_sec", 20.0)
        self.declare_parameter("junction_escape_m", 1.2)
        self.declare_parameter("junction_escape_cooldown_sec", 40.0)
        # [V2.1 yield-BT, 사용자 09-20] "BackUp 은 직선 후진만 된다. 현장 recovery 는 알고리즘 기반이라 곡선 후진 경로를 만든다."
        # → 양보 기동을 behavior_server 액션 대신 **BT 의 후퇴 경로 주행**으로 한다: 조정 계층은 후퇴 목표 자세만 정해 주고
        # (/yield_goal), BT 가 ReverseReedsShepp 플래너로 경로를 만들어 ReverseRPP 컨트롤러로 따라간다 (sim BT 복제본 moduler32_v2_yield.xml).
        # 후퇴 목표는 **이미 지나온 goal** 이 1순위 (사용자 제안) — 점유돼 있으면 그 앞의 goal, 그것도 없으면 내 주행 이력 위의 점.
        self.declare_parameter("yield_bt_enable", False)
        self.declare_parameter("yield_goal_topic", "/yield_goal")
        self.declare_parameter("yield_flag_topic", "/request_yield")
        self.declare_parameter("remaining_goals_topic", "/remaining_goals")
        self.declare_parameter("controller_selector_topic", "controller_selector")
        self.declare_parameter("yield_reverse_controller", "ReverseRPP")
        self.declare_parameter("yield_default_controller", "FollowPath")
        self.declare_parameter("yield_reach_tol_m", 0.4)
        self.declare_parameter("yield_timeout_sec", 45.0)
        self.declare_parameter("yield_passed_goal_max", 3)      # 몇 개까지 거슬러 올라갈까
        self.declare_parameter("yield_goal_occupied_m", 0.7)    # 이 안에 다른 로봇이 있으면 그 goal 은 점유
        self.declare_parameter("yield_goal_min_m", 0.8)         # 너무 가까운 지나온 goal 은 의미 없다
        self.declare_parameter("yield_no_move_sec", 15.0)       # 이 시간 동안 0.2 m 도 못 가면 후퇴 경로가 없는 것으로 본다
        self.declare_parameter("yield_try_sec", 8.0)            # 한 후보를 이만큼 시도해 보고 안 되면 **더 전 goal** 로 넘어간다
        # [09-20 09:10] 큐17 1판: 역방향 플래너가 먼 목표에서 자주 실패("no valid path" 81, "exceeded max iterations" 52).
        # → 후퇴 목표는 가까운 것만 쓰고(yield_goal_max_m), BT 후퇴가 실패하면 일정 시간 동안은 직선 BackUp 으로 되돌린다.
        self.declare_parameter("yield_goal_max_m", 2.5)
        # [V2.33 09-25 사용자] "필요한 만큼만" 후진. 이웃의 앞길(truncated_path)이 내 몸체와 겹치지 않게 되는 최소 거리를
        # 후진 방향으로 0.1 m 씩 찾아, 후퇴 목표를 모두 그 거리로 줄인다. 상한을 넘으면 상한만 가고 멈춘다 —
        # 아직 막혀 있으면 기존 규칙이 다시 판단해 한 번 더 짧게 물러난다(stop-and-go, 현장 BT recovery 와 같은 발상).
        self.declare_parameter("retreat_need_enable", False)
        self.declare_parameter("retreat_min_m", 0.3)
        self.declare_parameter("retreat_max_m", 1.0)
        self.declare_parameter("retreat_body_r_m", 0.32)          # 몸체 반경 (몸체 간격 0.1 m = 중심거리 0.73 m 에서)
        self.declare_parameter("retreat_clear_margin_m", 0.1)
        self.declare_parameter("retreat_path_look_m", 2.0)        # 이웃 앞길을 이만큼 본다 (짧으면 직선으로 늘림)
        # [V2.34 09-26] 후진 **공통 관문**. YM(몰린 링) 실측: 판당 후퇴 약 170회 중 내 몸체가 이웃 앞길과 겹친 것은 약 5 %
        # (junction-escape 3 %, unwedge 11 %, yield 3 %, standoff 6 %). 밀집 노선에서는 이런 후진이 처리량을 −23 % 깎았다(DC).
        # 모든 후진 입구(_backup_send·_retreat_along_history)에서 다음 셋 중 하나일 때만 허용한다.
        #   ① 주변 retreat_gate_free_m 안에 이웃이 없다 — 정적 끼임이라 후진해도 남을 방해하지 않는다(unwedge 본래 목적)
        #   ② 내 몸체가 이웃 앞길과 겹친다 — 누가 내 자리를 필요로 한다(V2.33 판정 재사용)
        #   ③ 내가 교차로 구역을 점유(occupancy)하고 있고 같은 구역을 기다리는 이웃이 있다 — 관제 흐름제어는 구역 단위라
        #      앞길 위가 아니어도 내가 구역을 막는다
        self.declare_parameter("retreat_gate_enable", False)
        self.declare_parameter("retreat_gate_free_m", 1.5)
        # [V2.34b 09-26] YG2 엉킴: 교차로 안에서 r3·r4 가 서로 막고 둘 다 멈췄는데 앞길 겹침이 안 잡혀 r3 의 후진이 보류됐다.
        # ④ 가까운 이웃과 **둘 다** retreat_gate_mutual_sec 이상 멈춰 있으면(상호 정지) 허용한다.
        # 또 ① '이웃 없음' 은 조정 대기(목표 점유·이웃·재계획 pause) 중이면 끼임이 아니라 기다리는 중이므로 허용하지 않는다(DG1 실측).
        self.declare_parameter("retreat_gate_mutual_sec", 20.0)
        # [V2.37 09-27] 곡선 후진 실패 분석(YC·DY·GY)에서 나온 두 가지.
        # (a) BT 가 돌지 않는 종료 상태(SUCCEEDED·CANCELED·FAILED)에서 BT 곡선 후진을 요청해 15 s 를 헛보냈다
        #     (DY 못 움직임 7/8, GY 7/17 이 SUCCEEDED). → 이 상태에서는 BT 대신 직접 기동을 쓴다.
        # (b) 관제가 세운 로봇(/nav_pause_flag=true, controller 가 제자리 유지)에게 후진을 요청했다. BT 곡선 후진은 controller 가 멈춰
        #     있어 못 움직이고, 직선 BackUp 폴백은 behavior_server 가 controller 를 거치지 않아 **관제가 세운 로봇을 움직일 수 있다**.
        #     → 관제 정지 중에는 어떤 후진도 하지 않는다. /nav_pause_flag 는 1회성 트리거(TRANSIENT_LOCAL)라 늦게 붙어도 현재 상태를 안다.
        self.declare_parameter("yield_bt_active_only", False)
        # [09-27 시험용, 기본 꺼짐] 충돌 시험이 fleet_decision 경로 그대로(관문·사전 검사·stop-and-go 포함) 후진을 내게 하는 입력.
        # /fleet_decision/test_retreat (String JSON): {"kind":"backup","dist":1.0,"speed":0.1} 또는 {"kind":"yield","dist":0.8}
        # (V2.39 후진 감시는 behavior_server 10 Hz 와 중복이라 뺐다 — 사용자 09-27. 사본: sim_runs/fleet_decision_node.py.with_v239_guard)
        self.declare_parameter("test_hooks_enable", False)
        self.declare_parameter("respect_nav_pause", False)
        self.declare_parameter("nav_pause_topic", "/nav_pause_flag")
        # [OP 09-28 현장 사고: 이적재 뒤 출발 시 unwedge 후진] 관제 명령 우선·자기 기동 허가 (verify/FLEET_V2_FIELD_ISSUE_0928.md)
        #   - 로봇은 관제 명령이 살아 있을 때만 움직인다. 관제 pause·stop·도킹·종료 상태에서는 fleet 이 절대 움직이지 않는다.
        #   - 허가가 사라지면 진행 중인 fleet 기동(BackUp/Drive/Spin/곡선 후진)을 즉시 거둔다.
        #   - 무진전 시계는 이번 주행 명령의 활성 시간만 센다(도킹·이적재·충전으로 서 있던 시간, 관제 pause 시간 제외).
        #   - 관제 pause 중에는 fleet 판단 루프를 모두 쉬고, 해제 때 대기 시계를 pause 길이만큼 밀어 이어서 센다(5 분+ pause 대비).
        self.declare_parameter("operator_priority_enable", True)
        self.declare_parameter("dock_monitoring_topic", "ros2_dock_monitoring_data")
        # [V2.35 09-26] 후퇴 도달 판정이 절대값 yield_reach_tol_m(0.4) 뿐이라, 0.4 m 보다 짧은 후퇴 목표는 **출발하자마자 '도달'** 로 끝났다
        # (W·Y 94 %, V2.33 이후 99 % 가 1.5 s 안에 '완료' — 실제로는 안 움직였다). >0 이면 허용 오차를
        # min(yield_reach_tol_m, max(0.05, yield_reach_frac × 목표 거리)) 로 줄인다. 0 = 기존 동작.
        self.declare_parameter("yield_reach_frac", 0.0)
        self.declare_parameter("yield_fallback_backup_sec", 90.0)
        # [09-20 09:20 정정] 큐17 1판 실패의 진짜 원인: ReverseReedsShepp(reverse_smac_planner) 는 **전진 primitive 를 아예 뺀**
        # 후진 전용 플래너다(node_hybrid.cpp: FORWARD/LEFT/RIGHT 주석 처리, REVERSE/REV_LEFT/REV_RIGHT 만 남김).
        # 그런데 이력 폴백이 '앞쪽(ahead)' 후보를 고르는 경우가 있어 후진만으로는 도달할 수 없었다 → "exceeded maximum iterations".
        # (실패 사례: 2.00 m 짜리도 실패했다. 거리 문제가 아니었다.) → BT 후퇴 목표는 **뒤쪽 부채꼴** 안에서만 고른다.
        self.declare_parameter("yield_rear_cone_deg", 75.0)
        # [09-20 09:45] 큐18 1판: 후퇴 목표가 **통로 밖(벽 안쪽)** 으로 잡혀 후진 전용 플래너가 반복 한도까지 헤맸다
        # (r1: 통로 H2(y −1.3~0.5) 안에서 '바로 뒤' 가 y −2.23 → 벽 너머). 지도로 목표와 경로 구간을 미리 검사한다.
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("yield_clearance_m", 0.35)    # 목표·구간 주변 이만큼은 비어 있어야 한다
        # [V2.2 09-20 12:30] **교차로를 막고 서지 않는다**(keep intersections clear). 5·6대 판의 붕괴는 대부분
        # "교차로 한가운데에서 정지 → 세 방향이 동시에 막힘" 에서 시작했다. 지도로 교차로(네 방향 중 3방향 이상이
        # junction_probe_m 만큼 열린 자리) 를 판정해, 그 안에서 정지해야 할 상황이면 **앞이 비어 있는 동안은 빠져나간 뒤**
        # 정지한다 (최대 junction_clear_max_sec). 통로 한가운데 정지는 종전대로.
        self.declare_parameter("junction_keep_clear_enable", False)
        self.declare_parameter("junction_probe_m", 1.2)
        self.declare_parameter("junction_open_dirs", 3)
        self.declare_parameter("junction_clear_ahead_m", 1.5)   # 이 거리만큼 내 경로가 비어 있어야 빠져나간다
        self.declare_parameter("junction_clear_max_sec", 6.0)
        # [V2.3 09-20 14:00] **자기 구출(self-unwedge)**. 긴 정체(1900~2100 s) 를 뜯어 보니 조정 대기가 아니라
        # **Nav2 회복 루프**였다: 로봇이 벽에 붙어 끼면 플래너가 시작점 점유/경로 없음을 내고 RECOVERY 만 반복,
        # 관제가 2.5분마다 다시 명령해도 start_timeout(주행 시작 못 함) 이 반복된다 (5대 판 r4: 33분, r5: 35분).
        # 이웃과 무관하므로 양보 규칙이 안 걸린다 → 이웃 없이도 **스스로 조금 물러나** 끼임을 푼다.
        self.declare_parameter("unwedge_enable", False)
        self.declare_parameter("unwedge_after_sec", 75.0)     # 이만큼 제자리면 (조정 대기 아님)
        # [09-30 사용자 결정 D11 (가)] abort·취소 뒤 같은 자리에서 받은 새 명령은 무진전 시계를 이어서 센다
        self.declare_parameter("stuck_carry_same_spot_m", 0.3)   # 직전 종료 위치에서 이만큼 안이면 '같은 자리'
        self.declare_parameter("stuck_carry_window_sec", 180.0)  # 종료 뒤 이 시간 안에 온 새 명령만
        self.declare_parameter("stuck_ready_reset_sec", 10.0)    # READY(목표 점유 대기)가 이만큼 넘으면 출발 때 새로 잡는다
        self.declare_parameter("unwedge_dist_m", 0.7)
        self.declare_parameter("unwedge_cooldown_sec", 60.0)
        self.declare_parameter("unwedge_max_tries", 6)
        # [V2.15] 한도에 닿아도 이만큼 쉬었다가 다시 시도한다 (영구 포기 금지)
        self.declare_parameter("unwedge_reset_sec", 120.0)
        # [A/B] false 면 V2.15 이전 동작(자리 기준 되돌림 없음 + 한도 도달 시 영구 포기)
        self.declare_parameter("unwedge_persist", True)
        # [V2.17] 공용 통로 줄 교착 해소. 지금 규칙은 전부 "나와 상대 한 대" 단위라
        # 한 통로에 여러 대가 일렬로 끼면 아무도 못 푼다. 게다가 다섯 대가 **동시에** 물러나려 해
        # 서로의 충돌 검사에 걸려 전부 ABORT 된다. → 줄을 인식하고 **한 번에 한 대씩** 순서대로 뺀다.
        self.declare_parameter("line_evac_enable", True)
        self.declare_parameter("line_min_n", 3)        # 이만큼 줄지어 서 있으면 줄로 본다
        self.declare_parameter("line_gap_m", 2.0)      # 이 안에 있으면 같은 줄로 잇는다
        self.declare_parameter("line_stuck_sec", 60.0) # 줄 구성원이 모두 이만큼 정지
        self.declare_parameter("line_turn_sec", 25.0)  # 한 대에게 주는 차례 시간
        # [V2.16] 기동 상태 전역 감시. `_backup_state` 가 idle 이 아니면 자기 구출 규칙 7곳이 전부 막힌다.
        # 기존 감시는 조정 대기 처리 안에만 있어, 밀어내기·자기 구출이 시작한 기동은 사각지대였다
        # (그 규칙들은 조정 대기 중이면 아예 실행하지 않는다). "waiting" 도 감시에서 빠져 있었다.
        self.declare_parameter("maneuver_watchdog_sec", 90.0)
        # [V2.7 09-20 22:20] **정면 대치**(2대) 해소. p3_a_v26 에서 r1-r4 가 0.70 m 간격으로 483 s 멈춰 있었다.
        # 순환 검출은 "대기 상태"에 들어간 3대 이상 고리를 잡지만, 회복 루프에 빠진 2대 마주보기는 놓친다.
        # 규칙: 둘 다 제자리 + 서로 마주봄 + 가까움 → **우선순위 낮은 쪽이 무조건 물러난다**(높은 쪽은 그대로).
        # [V2.10] 완주 직전 보호: 내 goal 이 이 거리 안이면 양보 기동 대상에서 뺀다 (완주가 먼저)
        self.declare_parameter("goal_guard_m", 0.35)
        # [V2.12] goal 코앞에서 오래 멈춰 있으면 조정 pause 를 풀어 **완주를 먼저 끝낸다**
        self.declare_parameter("goal_finish_m", 1.0)      # [V2.12] 완주 우선 재개는 양보 억제(0.35)보다 넓게 본다
        self.declare_parameter("goal_finish_sec", 25.0)
        self.declare_parameter("goal_finish_max_tries", 3)
        self.declare_parameter("goal_finish_cooldown_sec", 20.0)
        self.declare_parameter("standoff_enable", False)
        self.declare_parameter("standoff_after_sec", 25.0)    # 둘 다 이만큼 제자리
        self.declare_parameter("standoff_dist_m", 1.4)        # 중심 간 거리가 이 안
        self.declare_parameter("standoff_cone_deg", 70.0)     # 상대가 내 진행 방향 부채꼴 안
        self.declare_parameter("standoff_face_deg", 110.0)    # 두 헤딩 차이가 이보다 크면 마주 봄
        self.declare_parameter("standoff_retreat_m", 1.0)
        # [10-05 D20 J1] 후퇴 후보가 보고한 로봇에서 멀어져야 하는 최소 거리 = min(0.5, 이 비율 × 후퇴 거리).
        #   예전 고정 0.5 m 는 기본 후퇴 거리(1.0·2.0 m)에 맞춘 값이라, yaml 의 짧은 후퇴(standoff 0.3·push 0.5)에서는
        #   삼각부등식상 만족하는 후보가 없었다 (standoff 직선 후퇴 34/34 실패, push 양보 626/626 보류 — sim 전체 집계).
        self.declare_parameter("retreat_away_ratio", 0.5)
        self.declare_parameter("standoff_cooldown_sec", 25.0)
        # [V2.4 09-20 16:10 사용자 방향] "제어권을 최대한 fleet_decision 이 가져가자. BT 는 실행 엔진."
        # 1단계: **recovery 선택권**을 노드로. BT 가 올리는 `/bt_error_code` 를 노드가 보고 원인별 조치를 고른다.
        self.declare_parameter("bt_error_react_enable", False)
        self.declare_parameter("bt_error_topic", "/bt_error_code")
        self.declare_parameter("bt_error_burst_n", 4)
        self.declare_parameter("bt_error_window_sec", 12.0)
        self.declare_parameter("bt_error_stuck_sec", 12.0)
        self.declare_parameter("bt_error_cooldown_sec", 25.0)
        self.declare_parameter("recovery_planner_a", "RecoveryGridBased1")
        self.declare_parameter("recovery_planner_b", "RecoveryGridBased2")
        self.declare_parameter("recovery_planner_hold_sec", 20.0)
        self.declare_parameter("default_planner", "GridBased")
        # [V2.6 09-20 19:20] V2.5(BT recovery 삭제) 기각 뒤 설계 수정: **동작은 BT(현장 자산), 선택은 노드**.
        # 노드가 `/recovery_cmd_*` 하트비트로 지시하면 BT 가 그 동작만 실행한다 (moduler32_v2_exec2.xml).
        self.declare_parameter("recovery_cmd_enable", False)
        self.declare_parameter("recovery_cmd_hold_sec", 2.5)     # 지시를 유지(하트비트)하는 시간
        # 같은 경로 판정: 내 경로점 중 상대 튜브 안 비율
        self.declare_parameter("same_path_overlap_ratio", 0.5)
        self.declare_parameter("same_path_tube_m", 0.6)
        self.declare_parameter("same_path_heading_deg", 60.0)
        # 동적 우선순위 (aging)
        self.declare_parameter("dynamic_priority_enable", True)
        self.declare_parameter("aging_step_sec", 60.0)
        self.declare_parameter("aging_max_level", 2)
        # 무진전 보고
        self.declare_parameter("no_progress_report_sec", 300.0)
        self.declare_parameter("no_progress_dist_m", 0.3)
        # 정적 상한 추적: 같은 자리 재발은 같은 상황 (M-11)
        self.declare_parameter("static_same_spot_m", 0.3)
        # [09-28 현장 이상 2] 해제 지점에서 이만큼 벗어난 적이 있으면 '그 자리를 통과했다' — 같은 자리 재차단도 새 상황
        self.declare_parameter("static_same_spot_leave_m", 2.0)
        # 자기 정보 토픽 (M-9)
        self.declare_parameter("own_path_topic", "/plan_truncated_short")
        self.declare_parameter("own_twist_topic", "/cmd_vel")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("topic_velocity_control", "/velocity_modifier/control")
        gp = lambda n: self.get_parameter(n).value
        self.slow_wait_speed = float(gp("slow_wait_speed_mps"))
        self.slow_wait_angular = float(gp("slow_wait_angular_rps"))
        self.slow_wait_min_dist = float(gp("slow_wait_min_dist_m"))
        self.slow_wait_max_sec = float(gp("slow_wait_max_sec"))
        self.slow_clear_sec = float(gp("slow_clear_sec"))
        self.speed_match_enable = bool(gp("speed_match_enable"))
        self.speed_match_ratio = float(gp("speed_match_ratio"))
        self.speed_match_min = float(gp("speed_match_min_mps"))
        self.agent_speed_window = float(gp("agent_speed_window_sec"))
        self.agent_moving_mps = float(gp("agent_moving_mps"))
        self.stopped_yielding_wait = float(gp("stopped_yielding_wait_sec"))
        self.stopped_working_wait = float(gp("stopped_working_wait_sec"))
        self._yielding_phases = set(int(v) for v in gp("yielding_phases"))
        self._working_phases = set(int(v) for v in gp("working_phases"))
        self.mutual_block_replan_sec = float(gp("mutual_block_replan_sec"))
        self.mutual_block_radius = float(gp("mutual_block_radius_m"))
        self.mutual_block_near = float(gp("mutual_block_near_m"))
        self.slow_wait_min_body = float(gp("slow_wait_min_body_m"))
        self.predict_enable = bool(gp("predict_enable"))
        self.predict_horizon = float(gp("predict_horizon_sec"))
        self.predict_gap = float(gp("predict_gap_sec"))
        self.predict_conflict_dist = float(gp("predict_conflict_dist_m"))
        self._pre_slow_active: bool = False
        self._pre_slow_v: float = 0.0
        self._pre_slow_target: int = 0
        self.predict_debug = bool(gp("predict_debug"))
        self.agent_replan_enable = bool(gp("agent_replan_enable"))
        self.yield_backoff_enable = bool(gp("yield_backoff_enable"))
        self.yield_backoff_m = float(gp("yield_backoff_m"))
        self.yield_backoff_speed = float(gp("yield_backoff_speed_mps"))
        self.yield_backoff_body = float(gp("yield_backoff_body_m"))
        self._backoff_until: Optional[Time] = None
        self._backoff_done_for: Optional[int] = None
        self._predict_dbg_t: Optional[Time] = None
        self._v2_target_moving: bool = False      # 결정 시점에 상대가 주행 중이었나
        self._v2_mutual_applied: bool = False
        self.same_path_overlap_ratio = float(gp("same_path_overlap_ratio"))
        self.same_path_tube = float(gp("same_path_tube_m"))
        self.same_path_heading = math.radians(float(gp("same_path_heading_deg")))
        self.dynamic_priority_enable = bool(gp("dynamic_priority_enable"))
        self.aging_step_sec = float(gp("aging_step_sec"))
        self.aging_max_level = int(gp("aging_max_level"))
        self.no_progress_report_sec = float(gp("no_progress_report_sec"))
        self.no_progress_dist = float(gp("no_progress_dist_m"))
        self.static_same_spot = float(gp("static_same_spot_m"))
        self.static_same_spot_leave = float(gp("static_same_spot_leave_m"))
        self.global_frame = str(gp("global_frame"))
        self.base_frame = str(gp("base_frame"))

        # [V2] 상태
        self._own_path: Optional[Path] = None
        self._own_path_t: Optional[Time] = None
        self._own_twist = Twist()
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self, spin_thread=False)
        self._agent_hist: Dict[int, deque] = {}          # mid -> deque[(t_sec, x, y)]
        self._agent_still: Dict[int, Tuple[float, float, float, float]] = {}   # [V2.34b] mid -> (x, y, 멈춘 시각, 마지막 소식)
        self._agent_pause_since: Dict[int, Time] = {}    # mid -> PAUSE 가 관측되기 시작한 시각
        self._v2_mode: str = "pause"                      # 진행 중 시퀀스의 모드: pause | slow | match
        self._v2_pending_mode: str = "pause"
        self._v2_pending_timeout: Optional[float] = None
        self._v2_reason: str = ""
        self._speed_limited: bool = False
        self._last_speed_cmd: Optional[Tuple[int, float, float]] = None
        self._external_speed_ctrl: Optional[ModifierControl] = None
        self._static_last_hit_xy: Optional[Tuple[float, float]] = None
        self._static_hit_xy: Optional[Tuple[float, float]] = None
        # [09-28 현장 이상 2] 정적 해제 지점과, 그 뒤 거기서 static_same_spot_leave_m 이상 벗어난 적이 있는지
        self._static_release_xy: Optional[Tuple[float, float]] = None
        self._static_left_spot: bool = False
        self._np_anchor: Optional[Tuple[float, float]] = None
        self._np_anchor_t: Optional[Time] = None
        self._v2_prio_reeval_t: Optional[Time] = None
        self._v2_pause_since: Optional[Time] = None        # 내가 PAUSE 로 보이기 시작한 시각 (aging)
        self.declare_parameter("moving_target_pause_sec", 10.0)   # 가까운 주행 상대에게 정지로 기다리는 상한
        self.moving_target_pause_sec = float(self.get_parameter("moving_target_pause_sec").value)

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

        # [V2] 자기 경로·속도는 자기 토픽에서 (M-9). 관제가 자기 레코드를 주든 말든 무관.
        self.create_subscription(Path, self.get_parameter("own_path_topic").value,
                                 self._on_own_path, 10, callback_group=self.cb_group)
        self.create_subscription(Twist, self.get_parameter("own_twist_topic").value,
                                 self._on_own_twist, 10, callback_group=self.cb_group)
        qos_ctrl = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                              reliability=QoSReliabilityPolicy.RELIABLE,
                              durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(ModifierControl, self.get_parameter("topic_velocity_control").value,
                                 self._on_speed_ctrl, qos_ctrl, callback_group=self.cb_group)
        self.pub_speed_ctrl = self.create_publisher(
            ModifierControl, self.get_parameter("topic_velocity_control").value, qos_ctrl)
        self.pub_cmd_vel_out = self.create_publisher(Twist, self.get_parameter("topic_cmd_vel_out").value, 10)
        self._backup_cb_group = ReentrantCallbackGroup()
        self._backup_client = ActionClient(self, BackUp, self.get_parameter("backup_action_name").value,
                                           callback_group=self._backup_cb_group)
        self.backup_time_allowance = float(self.get_parameter("backup_time_allowance_sec").value)
        self.cycle_detect_enable = bool(self.get_parameter("cycle_detect_enable").value)
        self.cycle_max_len = int(self.get_parameter("cycle_max_len").value)
        self.cycle_front_deg = float(self.get_parameter("cycle_front_deg").value)
        self.cycle_front_m = float(self.get_parameter("cycle_front_m").value)
        self.cycle_goal_search_m = float(self.get_parameter("cycle_goal_search_m").value)
        self.cycle_min_wait_sec = float(self.get_parameter("cycle_min_wait_sec").value)
        self.cycle_stagger_sec = float(self.get_parameter("cycle_stagger_sec").value)
        self.cycle_backoff_m = float(self.get_parameter("cycle_backoff_m").value)
        self.cycle_retry_sec = float(self.get_parameter("cycle_retry_sec").value)
        self._cycle_sig: Optional[Tuple[int, ...]] = None
        self._cycle_since: Optional[Time] = None
        self._cycle_done: Dict[Tuple[int, ...], Time] = {}
        self._cycle_backoff_active = False          # 내가 순환 해소용 BackUp 을 보낸 상태
        self._cycle_backoff_static = False          # 그 BackUp 이 정적(goal 점유) 대기 중에 나간 것인가
        self._cycle_failed_sig: Optional[Tuple[int, ...]] = None   # 내 BackUp 이 실패한 순환 → 다음엔 이력 후퇴
        self.push_yield_enable = bool(self.get_parameter("push_yield_enable").value)
        self.blocked_yield_enable = bool(self.get_parameter("blocked_yield_enable").value)
        self.blocked_yield_m = float(self.get_parameter("blocked_yield_m").value)
        self.blocked_yield_sec = float(self.get_parameter("blocked_yield_sec").value)
        self.push_min_sec = float(self.get_parameter("push_min_sec").value)
        self.push_retreat_m = float(self.get_parameter("push_retreat_m").value)
        self.push_cooldown_sec = float(self.get_parameter("push_cooldown_sec").value)
        self.retreat_speed = float(self.get_parameter("retreat_speed_mps").value)
        self.stopgo_enable = bool(self.get_parameter("maneuver_stopgo_enable").value)
        self.stopgo_retry_sec = float(self.get_parameter("maneuver_stopgo_retry_sec").value)
        self.stopgo_max_sec = float(self.get_parameter("maneuver_stopgo_max_sec").value)
        self.retreat_clear_costmap = bool(self.get_parameter("retreat_clear_costmap").value)
        self.retreat_clear_cooldown = float(self.get_parameter("retreat_clear_cooldown_sec").value)
        self._retreat_clear_at: Optional[Time] = None
        self.rear_check_enable = bool(self.get_parameter("rear_check_enable").value)
        self.rear_check_margin = float(self.get_parameter("rear_check_margin_m").value)
        self.rear_check_max_age = float(self.get_parameter("rear_check_max_age_sec").value)
        self._lc_grid: Optional[OccupancyGrid] = None       # local costmap (odom 좌표계)
        self._lc_data: Optional[List[int]] = None
        self._lc_at: Optional[Time] = None
        self._lc_fp: Optional[List[Tuple[float, float]]] = None
        self._lc_odom: Optional[Tuple[float, float, float]] = None
        self._rear_block_at: Optional[float] = None
        self.push_during_goal_wait = bool(self.get_parameter("push_during_goal_wait").value)
        self.push_during_goal_wait_sec = float(self.get_parameter("push_during_goal_wait_sec").value)
        self.line_rear_first = bool(self.get_parameter("line_rear_first").value)
        self.line_escape_sec = float(self.get_parameter("line_escape_sec").value)
        self.line_rear_probe_m = float(self.get_parameter("line_rear_probe_m").value)
        self.junction_exit_enable = bool(self.get_parameter("junction_exit_enable").value)
        self.junction_exit_probe_m = float(self.get_parameter("junction_exit_probe_m").value)
        self.junction_exit_clear_m = float(self.get_parameter("junction_exit_clear_m").value)
        self.junction_exit_max_hold = float(self.get_parameter("junction_exit_max_hold_sec").value)
        self._jx_hold = False
        self._jx_since: Optional[Time] = None
        self.junction_escape_enable = bool(self.get_parameter("junction_escape_enable").value)
        self.junction_escape_after = float(self.get_parameter("junction_escape_after_sec").value)
        self.junction_escape_m = float(self.get_parameter("junction_escape_m").value)
        self.junction_escape_cooldown = float(self.get_parameter("junction_escape_cooldown_sec").value)
        self._jx_escape_last: Optional[Time] = None
        self._jx_escape_active = False
        if self.rear_check_enable:
            self.create_subscription(
                OccupancyGrid, "/local_costmap/costmap", self._on_local_costmap,
                QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                           reliability=QoSReliabilityPolicy.RELIABLE,
                           durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
                callback_group=self.cb_group)
            self.create_subscription(
                PolygonStamped, "/local_costmap/published_footprint", self._on_local_footprint,
                QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1), callback_group=self.cb_group)
            self.create_subscription(
                Odometry, "/odom", self._on_odom_for_rear,
                QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1), callback_group=self.cb_group)
        self._man_ctx: Optional[dict] = None        # 진행 중인 직진 기동 {kind, target, done, traveled, deadline, next, retries}
        self.yield_bt_enable = bool(self.get_parameter("yield_bt_enable").value)
        self.yield_reach_tol = float(self.get_parameter("yield_reach_tol_m").value)
        self.yield_timeout_sec = float(self.get_parameter("yield_timeout_sec").value)
        self.yield_passed_goal_max = int(self.get_parameter("yield_passed_goal_max").value)
        self.yield_goal_occupied_m = float(self.get_parameter("yield_goal_occupied_m").value)
        self.yield_goal_min_m = float(self.get_parameter("yield_goal_min_m").value)
        self.yield_no_move_sec = float(self.get_parameter("yield_no_move_sec").value)
        self.yield_try_sec = float(self.get_parameter("yield_try_sec").value)
        self._yield_cands: List[Tuple[float, float, float, str]] = []
        self._yield_idx = 0
        self.yield_goal_max_m = float(self.get_parameter("yield_goal_max_m").value)
        self.retreat_need_enable = bool(self.get_parameter("retreat_need_enable").value)
        self.retreat_min_m = float(self.get_parameter("retreat_min_m").value)
        self.retreat_max_m = float(self.get_parameter("retreat_max_m").value)
        self.retreat_body_r_m = float(self.get_parameter("retreat_body_r_m").value)
        self.retreat_clear_margin_m = float(self.get_parameter("retreat_clear_margin_m").value)
        self.retreat_path_look_m = float(self.get_parameter("retreat_path_look_m").value)
        self.retreat_gate_enable = bool(self.get_parameter("retreat_gate_enable").value)
        self.retreat_gate_free_m = float(self.get_parameter("retreat_gate_free_m").value)
        self.retreat_gate_mutual_sec = float(self.get_parameter("retreat_gate_mutual_sec").value)
        self.yield_bt_active_only = bool(self.get_parameter("yield_bt_active_only").value)
        self.respect_nav_pause = bool(self.get_parameter("respect_nav_pause").value)
        self._nav_paused = False
        self.operator_priority = bool(self.get_parameter("operator_priority_enable").value)
        self._nav_pause_since: Optional[Time] = None
        self._docking = False
        self._active_since: Optional[Time] = None     # 이번 관제 주행 명령의 활성 시각 (무진전 시계 상한)
        self._man_kind = "yield"                      # 진행 중인 fleet 기동 종류: "yield" | "self"
        self._maneuver_goal_handle = None             # Drive/Spin 단계 goal 핸들 (취소용)
        self._guard_revoked_logged = False
        self._nav_pause_denied = 0
        self.yield_reach_frac = float(self.get_parameter("yield_reach_frac").value)
        self._yield_tol = None
        self.yield_fallback_backup_sec = float(self.get_parameter("yield_fallback_backup_sec").value)
        self.yield_rear_cone_deg = float(self.get_parameter("yield_rear_cone_deg").value)
        self.yield_clearance_m = float(self.get_parameter("yield_clearance_m").value)
        self.junction_keep_clear = bool(self.get_parameter("junction_keep_clear_enable").value)
        self.junction_probe_m = float(self.get_parameter("junction_probe_m").value)
        self.junction_open_dirs = int(self.get_parameter("junction_open_dirs").value)
        self.junction_clear_ahead_m = float(self.get_parameter("junction_clear_ahead_m").value)
        self.junction_clear_max_sec = float(self.get_parameter("junction_clear_max_sec").value)
        self._junction_clear_since: Optional[Time] = None
        self._map_free_mask: Optional[bytearray] = None
        self._junc_cache = None
        self.unwedge_enable = bool(self.get_parameter("unwedge_enable").value)
        self.unwedge_after_sec = float(self.get_parameter("unwedge_after_sec").value)
        self.stuck_carry_same_spot = float(self.get_parameter("stuck_carry_same_spot_m").value)
        self.stuck_carry_window = float(self.get_parameter("stuck_carry_window_sec").value)
        self.stuck_ready_reset = float(self.get_parameter("stuck_ready_reset_sec").value)
        self._last_end = None          # [D11] 직전 활성 종료: {'status', 't', 'xy', 'active_since'}
        self._ready_since = None       # [D11] READY 에 들어온 시각
        self.unwedge_dist_m = float(self.get_parameter("unwedge_dist_m").value)
        self.unwedge_cooldown_sec = float(self.get_parameter("unwedge_cooldown_sec").value)
        self.unwedge_max_tries = int(self.get_parameter("unwedge_max_tries").value)
        self.unwedge_reset_sec = float(self.get_parameter("unwedge_reset_sec").value)
        self.unwedge_persist = bool(self.get_parameter("unwedge_persist").value)
        self.line_evac_enable = bool(self.get_parameter("line_evac_enable").value)
        self.line_min_n = int(self.get_parameter("line_min_n").value)
        self.line_gap_m = float(self.get_parameter("line_gap_m").value)
        self.line_stuck_sec = float(self.get_parameter("line_stuck_sec").value)
        self.line_turn_sec = float(self.get_parameter("line_turn_sec").value)
        self._line_sig: Optional[Tuple[int, ...]] = None
        self._line_since: Optional[Time] = None
        self.maneuver_watchdog_sec = float(self.get_parameter("maneuver_watchdog_sec").value)
        self._man_wd_state: Optional[str] = None
        self._man_wd_since: Optional[Time] = None
        self._unwedge_anchor: Optional[Tuple[float, float]] = None
        self._unwedge_exhausted_at: Optional[Time] = None
        self.goal_guard_m = float(self.get_parameter("goal_guard_m").value)
        self.goal_finish_m = float(self.get_parameter("goal_finish_m").value)
        self.goal_finish_sec = float(self.get_parameter("goal_finish_sec").value)
        self.goal_finish_max_tries = int(self.get_parameter("goal_finish_max_tries").value)
        self.goal_finish_cooldown_sec = float(self.get_parameter("goal_finish_cooldown_sec").value)
        self._goal_finish_last: Optional[Time] = None
        self._goal_finish_tries = 0
        self._goal_finish_anchor: Optional[Tuple[float, float]] = None
        self.standoff_enable = bool(self.get_parameter("standoff_enable").value)
        self.standoff_after_sec = float(self.get_parameter("standoff_after_sec").value)
        self.standoff_dist_m = float(self.get_parameter("standoff_dist_m").value)
        self.standoff_cone_deg = float(self.get_parameter("standoff_cone_deg").value)
        self.standoff_face_deg = float(self.get_parameter("standoff_face_deg").value)
        self.standoff_retreat_m = float(self.get_parameter("standoff_retreat_m").value)
        self.retreat_away_ratio = float(self.get_parameter("retreat_away_ratio").value)
        self._plan_retreat_why = ""
        self.standoff_cooldown_sec = float(self.get_parameter("standoff_cooldown_sec").value)
        self._standoff_last: Optional[Time] = None
        self._standoff_active = False
        self._standoff_sig: Optional[int] = None
        self._standoff_since: Optional[Time] = None
        self._unwedge_last: Optional[Time] = None
        self._unwedge_tries = 0
        self._unwedge_active = False
        self.bt_error_react = bool(self.get_parameter("bt_error_react_enable").value)
        self.bt_error_burst_n = int(self.get_parameter("bt_error_burst_n").value)
        self.bt_error_window = float(self.get_parameter("bt_error_window_sec").value)
        self.bt_error_stuck_sec = float(self.get_parameter("bt_error_stuck_sec").value)
        self.bt_error_cooldown = float(self.get_parameter("bt_error_cooldown_sec").value)
        self.recovery_planner_a = self.get_parameter("recovery_planner_a").value
        self.recovery_planner_b = self.get_parameter("recovery_planner_b").value
        self.recovery_planner_hold = float(self.get_parameter("recovery_planner_hold_sec").value)
        self.default_planner = self.get_parameter("default_planner").value
        self._bt_errs: deque = deque(maxlen=64)       # (t, code)
        self._recov_step = 0                           # 1 코스트맵 → 2 예비 플래너 A → 3 B → 4 자기 구출
        self._recov_last: Optional[Time] = None
        self._planner_override_until: Optional[Time] = None
        self.recovery_cmd_enable = bool(self.get_parameter("recovery_cmd_enable").value)
        self.recovery_cmd_hold = float(self.get_parameter("recovery_cmd_hold_sec").value)
        self._recov_cmd: Optional[str] = None
        self._recov_cmd_until: Optional[Time] = None
        self._map_msg: Optional[OccupancyGrid] = None
        self.create_subscription(
            OccupancyGrid, self.get_parameter("map_topic").value, self._on_map,
            QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=self.cb_group)
        self._yield_bt_fail_t: Optional[Time] = None
        self._yield_start_xy: Optional[Tuple[float, float]] = None
        self._passed_goals: deque = deque(maxlen=8)   # (x, y, yaw) 최근에 지나온 goal (최신이 뒤)
        self._prev_goals: List[Tuple[float, float, float]] = []
        self._yield_active = False
        self._yield_target: Optional[Tuple[float, float, float]] = None
        self._yield_start_t: Optional[Time] = None
        self._yield_src = ""
        self._own_hist: deque = deque(maxlen=600)   # (t, x, y) 내 자세 이력 (2 Hz, 5 분)
        self._agent_stop_since: Dict[int, Tuple[float, float, float]] = {}   # mid → (t, x, y) 정지 앵커
        self._push_done: Dict[int, Time] = {}
        self._push_paused = False
        self._maneuver: Optional[dict] = None       # 진행 중인 이력 후퇴 {mode, steps, ...}
        self._spin_client = ActionClient(self, Spin, self.get_parameter("spin_action_name").value,
                                         callback_group=self._backup_cb_group)
        self._drive_client = ActionClient(self, DriveOnHeading, self.get_parameter("drive_action_name").value,
                                          callback_group=self._backup_cb_group)
        self._backup_state: str = "idle"          # idle | sending | running | succeeded | failed
        self._backup_goal_handle = None
        self._backup_started_at: Optional[Time] = None

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
        _qos_latch = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                                reliability=QoSReliabilityPolicy.RELIABLE,
                                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        # [V2.36d 09-26 사용자] QoS 원칙: BT 가 구독하는 **1회성 트리거**는 TRANSIENT_LOCAL(유실 금지 + 초기화 설계),
        # **주기 하트비트·상태**는 VOLATILE 로 충분하다(다음 주기에 다시 온다, 잔여값 없음).
        #  - 후퇴 플래그·목표(/request_yield, /yield_goal)는 후퇴 중 2 Hz 하트비트 → VOLATILE. BT 쪽(YieldFlagFresh·
        #    GetYieldGoalAction)도 VOLATILE 로 구독하고 발행 시각으로 만료시킨다.
        #    (09-20 ~ 09-26 에는 여기가 VOLATILE, BT 쪽이 TRANSIENT_LOCAL 이라 DDS 호환이 안 돼 BT 가 한 번도 못 받았다.)
        #  - 회복 명령(/recovery_cmd_*)은 1회성 트리거 → TRANSIENT_LOCAL + 뜨자마자 false 로 초기화(_init_latched_flags).
        _qos_beat = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                               reliability=QoSReliabilityPolicy.RELIABLE,
                               durability=QoSDurabilityPolicy.VOLATILE)
        self.pub_yield_goal = self.create_publisher(
            PoseStamped, self.get_parameter("yield_goal_topic").value, _qos_beat)
        self.pub_yield_flag = self.create_publisher(
            Bool, self.get_parameter("yield_flag_topic").value, _qos_beat)
        self.pub_ctrl_sel = self.create_publisher(
            String, self.get_parameter("controller_selector_topic").value, _qos_latch)
        self.pub_planner_sel = self.create_publisher(String, "planner_selector", _qos_latch)
        self.pub_recov_cmd = {k: self.create_publisher(Bool, f"/recovery_cmd_{k}", _qos_latch)
                              for k in ("clear", "maneuver_a", "maneuver_b", "remove_goal")}
        if self.v2_enable:                            # v2 를 끄면 새 토픽에 아무것도 내지 않는다 (amhs 이관 S2)
            self._init_latched_flags()                # [V2.36d] 1회성 트리거(TRANSIENT_LOCAL)의 잔여값을 false 로 덮는다
        if bool(self.get_parameter("test_hooks_enable").value):
            self.create_subscription(String, "/fleet_decision/test_retreat", self._on_test_retreat, 10,
                                     callback_group=self.cb_group)
            self.get_logger().warn("[V2 test] 시험용 후진 입력 켜짐 (/fleet_decision/test_retreat) — 통합 판에서는 끌 것")
        # [V2.37b] 관제 정지 상태 — 1회성 트리거라 TRANSIENT_LOCAL 로 구독한다(늦게 떠도 현재 상태를 받는다)
        self.create_subscription(Bool, self.get_parameter("nav_pause_topic").value,
                                 self._on_nav_pause, _qos_latch, callback_group=self.cb_group)
        # [OP] 도킹 상태 (winros_bridge_to_dock 이 0.1 s 마다 발행)
        self.create_subscription(DockingMonitoring, self.get_parameter("dock_monitoring_topic").value,
                                 self._on_dock_monitoring, 10, callback_group=self.cb_group)
        self.create_subscription(UInt16, self.get_parameter("bt_error_topic").value,
                                 self._on_bt_error, 20, callback_group=self.cb_group)
        self._clear_global = self.create_client(Empty, "/global_costmap/clear_entirely_global_costmap",
                                                callback_group=self.cb_group)
        self._clear_local = self.create_client(Empty, "/local_costmap/clear_entirely_local_costmap",
                                               callback_group=self.cb_group)
        self.create_subscription(Path, self.get_parameter("remaining_goals_topic").value,
                                 self._on_remaining_goals, 10, callback_group=self.cb_group)
        self.pub_cmd_resume = self.create_publisher(Bool, 
            self.get_parameter("topic_cmd_resume").value, qos_req, callback_group=self.cb_group)
        
        self.pub_cmd_pause = self.create_publisher(Bool, 
            self.get_parameter("topic_cmd_pause").value, qos_req, callback_group=self.cb_group)

        self.pub_cmd_stop = self.create_publisher(UInt8, 
            self.get_parameter("topic_cmd_stop").value, 10, callback_group=self.cb_group)


        self.create_timer(0.1, self.check_collision_obstacle, callback_group=self.cb_group)

        self.create_timer(0.1, self.check_collision_agent, callback_group=self.cb_group)
        # [V2] 위치 무진전 감시 (1 Hz)
        self.create_timer(1.0, self._check_no_progress, callback_group=self.cb_group)
        # [V2] 교차 예측 감속 (2 Hz)
        self.create_timer(0.5, self._check_predicted_conflicts, callback_group=self.cb_group)
        self.create_timer(1.0, self._v2_cycle_tick, callback_group=self.cb_group)   # [V2.1] 대기 순환 검출
        self.create_timer(0.5, self._v2_own_hist_tick, callback_group=self.cb_group)  # [V2.1] 내 자세 이력
        self.create_timer(1.0, self._v2_push_tick, callback_group=self.cb_group)     # [V2.1] 밀어내기 양보
        self.create_timer(0.5, self._v2_maneuver_tick, callback_group=self.cb_group) # [V2.1] 기동 stop-and-go 재개
        self.create_timer(0.5, self._v2_yield_tick, callback_group=self.cb_group)    # [V2.1] BT 후퇴 주행 감시
        self.create_timer(1.0, self._v2_unwedge_tick, callback_group=self.cb_group)  # [V2.3] 자기 구출
        self.create_timer(1.0, self._v2_standoff_tick, callback_group=self.cb_group) # [V2.7] 정면 대치 해소
        self.create_timer(0.5, self._v2_junction_exit_tick, callback_group=self.cb_group)  # [V2.26] 교차로 진입 전 출구 확인
        self.create_timer(1.0, self._v2_maneuver_watchdog_tick, callback_group=self.cb_group)  # [V2.16] 기동 상태 굳음 감시
        self.create_timer(1.0, self._v2_goal_finish_tick, callback_group=self.cb_group) # [V2.12] 완주 우선
        self.create_timer(1.0, self._v2_bt_error_tick, callback_group=self.cb_group) # [V2.4] recovery 선택권
        self.create_timer(0.5, self._v2_yield_beat, callback_group=self.cb_group)    # [V2.1] 후퇴 요청 하트비트
        self.create_timer(0.5, self._v2_recov_cmd_beat, callback_group=self.cb_group)  # [V2.6] 회복 지시 하트비트
        self.create_timer(0.1, self._op_guard_tick, callback_group=self.cb_group)     # [OP] 허가 상실 시 fleet 기동 즉시 회수


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
        self.latest_agent_goal_occupied = False      # [V2] agent HIT 가 내 goal 점유인가 (validator 가 알려준다)
        self.latest_agent_last_goal_occupied = False
        self._v2_last_goal_report = False            # [V2] 마지막 goal 점유 대기 만료 시 replan 대신 STOP 보고
        self._v2_goal_occ_wait = False               # [V2] 지금 시퀀스가 'goal 점유 대기' 인가 (후진 양보 대상)

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
        self.get_logger().info(f" - [V2] v2_enable          : {self.v2_enable}")
        self.get_logger().info(f" - [V2] slow_wait          : {self.slow_wait_speed} m/s, min_dist {self.slow_wait_min_dist} m, max {self.slow_wait_max_sec} s")
        self.get_logger().info(f" - [V2] speed_match        : {self.speed_match_enable} x{self.speed_match_ratio} (min {self.speed_match_min})")
        self.get_logger().info(f" - [V2] stopped wait       : yielding {self.stopped_yielding_wait} s / working {self.stopped_working_wait} s")
        self.get_logger().info(f" - [V2] mutual_block       : {self.mutual_block_replan_sec} s / {self.mutual_block_radius} m")
        self.get_logger().info(f" - [V2] same_path          : ratio {self.same_path_overlap_ratio}, tube {self.same_path_tube} m")
        self.get_logger().info(f" - [V2] dynamic_priority   : {self.dynamic_priority_enable}, aging {self.aging_step_sec} s x{self.aging_max_level}")
        self.get_logger().info(f" - [V2] no_progress        : {self.no_progress_report_sec} s / {self.no_progress_dist} m")
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
            if self.v2_enable:
                self._v2_record_agent(a, now)

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

        # [V2.3 09-20 14:10] **새 명령이 오면 대기 상태를 반드시 푼다.**
        # 정체 분석(stall_report) 에서 "조정대기 상태로 굳음 — 결정 갱신 2023 s 없음" 이 두 판 연속 나왔다.
        # 원인: 목표 취소/중단(CANCELED·READY) 로 상태가 빠지면 check_collision_agent/obstacle 이 상태 게이트에서
        # 곧바로 return 해 대기 플래그(is_processing_*_pause, pause 발행) 가 **영영 남는다**. 관제가 다시 명령해도
        # 로봇은 pause 인 채라 start_timeout 만 반복한다 (관제 goal timeout 600 s 까지 통째로 낭비).
        prev = self.current_robot_status
        self.current_robot_status = msg.data
        # [OP] 종료·대기 상태(IDLE/SUCCEEDED/CANCELED/FAILED)에서 주행 계열로 들어오면 = 새 관제 명령.
        # 무진전 시계를 여기서 다시 시작한다 (도킹·이적재·충전 동안 서 있던 시간을 넘기지 않는다).
        # READY(목표 점유 대기, 최대 150 s) → RECEIVED_GOAL(실제 출발) 도 다시 시작한다 — 대기 시간을 무진전으로 넘기지 않는다.
        # [09-30 사용자 결정 D11 (가)] 관제가 abort 직후 곧바로 재명령하면(현장 Q4) 명령마다 시계가 0 이 되어
        #   standoff(25 s)·junction-escape(20 s)·unwedge(75 s) 가 끝내 발동하지 못했다 (sim L5f_dense4 H2V4 30 분,
        #   재명령 200여 회). 직전 종료가 CANCELED/FAILED 이고 같은 자리(0.3 m)·180 s 안·도킹 없이 온 새 명령이면
        #   이전 활성 시각을 이어 쓴다. SUCCEEDED 뒤·자리를 옮긴 뒤·도킹 뒤는 지금처럼 새로 잡는다 (09-28 결정 유지).
        #   READY→RECEIVED_GOAL 은 READY 가 10 s 넘게 이어졌을 때만(목표 점유로 정말 기다림) 새로 잡는다.
        _now_st = self.get_clock().now()
        if msg.data == 'READY' and prev != 'READY':
            self._ready_since = _now_st
        # [10-01 사용자 결정 D14 (가)] 종료 상태로 들어오는 순간 진행 중인 fleet 기동을 바로 거둔다 (관제 명령이 우선).
        #   관제 새 move 는 CANCELED(약 10 ms) → READY 로 지나가 0.1 s 주기 guard(_op_guard_tick)가 놓쳤고,
        #   READY 는 "허용" 이라 기동이 새 명령 뒤에도 이어졌다 (sim L5f_dense4 r5 0.2~0.4 m, L5g_dense2 r4 17.9 s).
        if self.operator_priority and msg.data in ('IDLE', 'SUCCEEDED', 'CANCELED', 'FAILED') and self._op_busy():
            self._op_revoke(f"상태 {msg.data}")
        if msg.data in ('IDLE', 'SUCCEEDED', 'CANCELED', 'FAILED') and prev in self._OP_ACTIVE:
            _me = self._own_pose()
            self._last_end = {'status': msg.data, 't': _now_st, 'xy': (_me[0], _me[1]) if _me is not None else None,
                              'active_since': self._active_since}
        if msg.data in self._OP_ACTIVE and (prev not in self._OP_ACTIVE or self._active_since is None):
            self._active_since = self._stuck_carry_or(_now_st)
        elif prev == 'READY' and msg.data == 'RECEIVED_GOAL':
            _ready = ((_now_st - self._ready_since).nanoseconds * 1e-9) if self._ready_since is not None else 1e9
            if _ready >= self.stuck_ready_reset:
                self._active_since = _now_st
        # [09-28 현장 이상 2] replan 재시도 횟수는 '한 관제 명령 안' 에서만 센다 (사용자 결정 F-a·F-b).
        # 새 명령(종료·대기 → 주행 계열, READY → RECEIVED_GOAL)과 goal 종료(IDLE/SUCCEEDED/CANCELED/FAILED) 때
        # 대기 상태 진행 여부와 상관없이 정적·agent 재시도 추적을 지운다.
        # 예전에는 대기가 진행 중일 때만 지워서, 정상 통과·완주한 명령의 횟수가 다음 바퀴로 넘어갔다.
        _new_cmd = (msg.data in self._OP_ACTIVE and prev != msg.data
                    and (prev not in self._OP_ACTIVE or (prev == 'READY' and msg.data == 'RECEIVED_GOAL')))
        _goal_end = (msg.data in ('IDLE', 'SUCCEEDED', 'CANCELED', 'FAILED') and prev != msg.data
                     and prev in self._OP_ACTIVE)
        if (_new_cmd or _goal_end) and (self._static_replan_retry or self._agent_replan_retry
                                        or self._static_last_release_t is not None
                                        or self._agent_last_release_t is not None):
            self.get_logger().info(
                f"[retry reset] {'새 명령' if _new_cmd else 'goal 종료'} ({prev} -> {msg.data}) — "
                f"replan 재시도 추적 초기화 (static {self._static_replan_retry}회, agent {self._agent_replan_retry}회)")
            self._static_reset_osc()
            self._agent_reset_osc()
        if self.v2_enable and msg.data in ('RECEIVED_GOAL', 'READY') and prev != msg.data:
            held = (self.is_processing_agent_pause or self.is_processing_replan_pause
                    or self.is_processing_goal_occupied_pause or self.is_processing_last_goal_occupied_pause)
            if held:
                now = self.get_clock().now()
                self.get_logger().warn(
                    f"[V2 reset] 새 상태 {msg.data} — 남아 있던 대기 상태를 푼다 "
                    f"(agent {self.is_processing_agent_pause}, static {self.is_processing_replan_pause}, "
                    f"goal {self.is_processing_goal_occupied_pause}/{self.is_processing_last_goal_occupied_pause})")
                self.is_processing_agent_pause = False
                self._agent_pause_start_time = None
                self._agent_clear_start_time = None
                self.is_processing_replan_pause = False
                self.is_processing_goal_occupied_pause = False
                self.is_processing_last_goal_occupied_pause = False
                self._pause_start_time = None
                self._goal_occupied_false_start_time = None
                self._last_goal_occupied_false_start_time = None
                self.static_is_goal_occupied_ = False
                self.static_is_last_goal_occupied_ = False
                self._v2_mode = "run"
                self._v2_pause_since = None
                if self._speed_limited:
                    self._v2_restore_speed("new goal")
                if self._yield_active:
                    self._yield_finish(False, "새 명령 수신")
                self.pub_cmd_resume.publish(Bool(data=False))
                self._publish_state("[V2 reset] RUN (new goal, stale wait cleared)")
                self._agent_reset_osc()
                self._static_reset_osc()
                self._np_anchor = None
                self._np_anchor_t = now


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
        if msg.replan_request:
            self._static_hit_xy = (float(msg.hit_x), float(msg.hit_y))   # [V2] M-11 같은 자리 판정용




    def check_collision_obstacle(self):
        """
        충돌 메시지 타임아웃과 별개로, replan_flag가 True로 유지되는 경우 일정 시간 후에 자동으로 주행 재개하는 로직
        """
       
        if self._op_frozen():
            return                                    # [OP] 관제 pause 중 — 판단 정지 (해제 때 시계를 이어서 센다)
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
                if self._junction_should_clear(now):
                    return                               # [V2.2] 교차로를 비우고 나서 멈춘다
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
                    if self._junction_should_clear(now):
                        return                           # [V2.2] 교차로를 비우고 나서 멈춘다
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
            if self._speed_limited:
                self._v2_restore_speed("status inactive")   # [V2]
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
                        # [09-29 D1-ⓑ] BT 가 recovery 안에 있으면 /request_replan 을 tick 하지 않아 이번 요청은 계산되지 않는다.
                        #   예전에는 그런 헛 요청도 세서 실제 계산 1~2회로 10/10 → 관제 보고에 닿았다 (L5m_grid r3, op_l2_D).
                        #   요청은 그대로 낸다 (래치 — recovery 에서 돌아오면 그때 계산). 끝내는 일은 BT recovery 한도·nav_stuck 이 맡는다.
                        _in_recovery = str(self.current_robot_status).startswith('RECOVERY_')
                        # [FIX] 이 상황에서 몇 번째 replan 인가.
                        if not _in_recovery:
                            self._static_replan_retry += 1
                        else:
                            self.get_logger().info(
                                f"[check_collision_obstacle] BT recovery 중({self.current_robot_status}) — replan 요청은 내되 "
                                f"시도 횟수에는 넣지 않는다 ({self._static_replan_retry}/{self.static_max_replan_retry})",
                                throttle_duration_sec=10.0)
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
                        if not _in_recovery:
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
            if self._junction_should_clear(now):
                return                                   # [V2.2] 교차로를 비우고 나서 멈춘다
            self.is_processing_replan_pause = True
            # [FIX] 진동이면 경과 시간을 이어받는다. 예전에는 무조건 now 여서
            # 재진입마다 타이머가 0 으로 돌아가 replan 에 영영 도달하지 못했다.
            self._pause_start_time = self._static_pause_start(now)
            self._publish_pause()
            return




    def check_collision_agent(self):
        """ Agent 충돌 예측에 대한 상태 머신 (20Hz 주기 실행) """
        if self._op_frozen():
            return                                    # [OP] 관제 pause 중 — 판단 정지 (해제 때 시계를 이어서 센다)
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
                if self._speed_limited:
                    self._v2_restore_speed("after action")    # [V2]
                self._v2_mode = "pause"
                self._v2_pause_since = None
                
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
            if self.v2_enable:
                self._v2_apply_mode_change(self._v2_pending_mode, now, "대상 변경")
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

            if self.v2_enable and self._v2_phase1_tick(now):
                return   # [V2] 감속 대기 중이거나 방금 상태를 바꿨다 — 아래 타임아웃 분기는 다음 틱에
            
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

                        # [V2] agent replan 금지 옵션: 대기를 wait_detect 만큼 이어가고 다시 본다 (상대가 움직이면 Early Exit 이 푼다)
                        if self.v2_enable and not self.agent_replan_enable and cmd != MovingCommand.REROUTE:
                            self.agent_pause_timeout_sec = dt + max(self.wait_detect_sec, 5.0)
                            self.get_logger().warn(
                                f"[V2] agent_replan_enable=false → replan 대신 대기 연장 ({self.agent_pause_timeout_sec:.0f}s, "
                                f"상대 {self._locked_target_id}, {self.current_agent_stop_type.name})", throttle_duration_sec=5.0)
                            self._publish_state(f"{self.current_agent_stop_type.name}: WAIT (no replan)")
                            return
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
                # [V2] 감속 대기(정지 안 함)면 짧은 확인으로 충분하다
                clear_need = (self.slow_clear_sec if (self.v2_enable and self._v2_mode != "pause")
                              else self.agent_wait_before_resume)
                self.get_logger().info(f"[check_collision_agent] [Early Exit] Clear timer: {elapsed:.1f}s / {clear_need:.1f}s", throttle_duration_sec=0.5)
                # self.agent_wait_before_resume(3.0초) 이상 비연속 충돌일 경우
                
                if elapsed >= clear_need:
                    self.get_logger().error(f"[check_collision_agent] [Early Exit] Agent path clear for {elapsed:.1f}s. Early Resume after waiting {self.agent_wait_before_resume}s!")
                    # self.get_logger().warn(f"Agent path clear for {elapsed:.1f}s. Early Resume after waiting {self.agent_wait_before_resume}s!")
                    if self._v2_mode == "pause":
                        self.pub_cmd_resume.publish(Bool(data=False))
                    if self._speed_limited:
                        self._v2_restore_speed("early exit")      # [V2]
                    if self.v2_enable:
                        self._publish_state(f"[check_collision_agent] RUN (Agent Early Resume, {self._v2_mode})")
                    else:
                        self._publish_state("[check_collision_agent] RUN (Agent Early Resume)")   # v1 문구 그대로 (amhs 이관 S2)
                    self._v2_mode = "pause"
                    self._v2_pause_since = None
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
            self._backoff_until = None                            # [V2]
            if self._yield_active:
                self._yield_finish(False, "조정 시퀀스 재잠금")     # 플래그·컨트롤러를 반드시 되돌린다
            # [V2.2] 교차로 한가운데서 잠그면 세 방향이 함께 막힌다 → 앞이 비어 있으면 빠져나간 뒤 잠근다
            if self._v2_pending_mode == "pause" and self._junction_should_clear(now):
                return
            if self._backup_state in ("sending", "running", "waiting"):
                self._backup_cancel()
            self._backup_state = "idle"; self._man_ctx = None
            # [FIX B-7] 같은 상대에게 짧은 간격으로 다시 걸린 것이면 경과·재시도를 이어받는다
            self._agent_pause_start_time = self._agent_pause_start(now, self._locked_target_id)
            
            # 대기 시간(N초) 매핑
            n_pause = self._pause_timeout_for(self.current_agent_command)
            self.agent_pause_timeout_sec = n_pause
            
            if self.v2_enable and self._v2_pending_mode != "pause":
                # [V2] 정지 대신 감속 대기 / 속도 맞추기. 정지·재개 왕복(실측 2~4 s) 을 아낀다.
                self._v2_mode = self._v2_pending_mode
                self._v2_apply_speed(now)
                self.get_logger().error(
                    f"[check_collision_agent] [Phase 0] Sequence Locked. {self._v2_mode.upper()} for {n_pause}s "
                    f"({self._v2_reason}).")
                self._publish_state(f"{self.current_agent_stop_type.name}: {self._v2_mode.upper()} {n_pause}s")
            else:
                self._v2_mode = "pause"
                self._v2_pause_since = now
                self.get_logger().error(f"[check_collision_agent] [Phase 0] Sequence Locked. Starting PAUSE for {n_pause}s. ({self._v2_reason})")
                self._publish_pause()
                self._publish_state(f"{self.current_agent_stop_type.name}: PAUSE {n_pause}s")




    def _pause_timeout_for(self, cmd: MovingCommand) -> float:
        """ 명령별 대기 시간(N초). Phase 0 진입과 대상 재평가가 함께 쓴다. """
        if self.v2_enable and self._v2_pending_timeout is not None:
            return float(self._v2_pending_timeout)     # [V2] 결정이 정한 대기 시간
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
        self.latest_agent_goal_occupied = bool(msg.is_goal_occupied or msg.is_last_goal_occupied)
        self.latest_agent_last_goal_occupied = bool(msg.is_last_goal_occupied)

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

        # [V2] 정책 v2. reroute 진행 중인 상대(TYPE_3/5~8) 는 v1 트리에 맡긴다.
        self._v2_pending_mode, self._v2_pending_timeout, self._v2_reason = "pause", None, ""
        if self.v2_enable and not self.simple_mode:
            agent_v2 = self._cached_agents.get(target_id)
            if agent_v2 is not None and not (self.use_reroute and agent_v2.reroute):
                return self._decide_v2(agent_v2, collision_xy)

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
        # [09-28 현장 이상 2] 해제 지점을 기록한다 — 여기서 static_same_spot_leave_m 벗어나면 통과한 것으로 본다
        me = self._own_pose()
        self._static_release_xy = (me[0], me[1]) if me is not None else None
        self._static_left_spot = False

    def _static_pause_start(self, now: Time) -> Time:
        """[FIX] 정적 pause 진입 시각을 정한다.

        직전 해제로부터 static_rejoin_window_sec 이내면 '같은 상황에 다시 튕긴 것'
        으로 보고 경과 시간을 이어받는다. 그래야 진동해도 누적이 쌓여
        replan_pause_timeout_sec 에 도달하고 상황이 실제로 바뀐다.

        창보다 오래 비었으면 **다른 상황**이다. 경과와 replan 재시도 횟수를
        모두 0 으로 되돌린다. 로봇이 실제로 그 자리를 통과했다면 재차단이
        일어나지 않거나 간격이 길어지므로 여기서 자동으로 리셋된다.
        """
        # [V2] M-11: 같은 자리(static_same_spot_m 안) 의 정적 HIT 재발은 사이에 agent 정지가
        # 끼어 간격이 길어져도 같은 상황으로 본다. 위치가 안 바뀌었으면 상황도 안 바뀐 것이다.
        same_spot = False
        if (self.v2_enable and self.static_same_spot > 0.0
                and self._static_hit_xy is not None and self._static_last_hit_xy is not None):
            same_spot = (math.hypot(self._static_hit_xy[0] - self._static_last_hit_xy[0],
                                    self._static_hit_xy[1] - self._static_last_hit_xy[1])
                         <= self.static_same_spot)
        self._static_last_hit_xy = self._static_hit_xy
        # [09-28 현장 이상 2] 해제 뒤 그 자리에서 벗어난 적이 있으면 간격·같은 자리와 무관하게 새 상황이다.
        # (예전: 같은 자리면 557 s 뒤에도, 한 바퀴 돌고 와서도 재시도 횟수를 이어받아 10회째에 관제 보고·cancel)
        if self._static_left_spot and self._static_last_release_t is not None:
            self.get_logger().info(
                f"[check_collision_obstacle] 직전 해제 뒤 그 자리를 벗어났었다 — 새 상황 "
                f"(직전 replan 재시도 {self._static_replan_retry}회 초기화).")
            self._static_replan_retry = 0
            self._static_last_elapsed = 0.0
            self._static_left_spot = False
            return now
        if (self.static_rejoin_window_sec > 0.0
                and self._static_last_release_t is not None):
            gap = (now - self._static_last_release_t).nanoseconds * 1e-9
            if same_spot and gap > self.static_rejoin_window_sec:
                self.get_logger().warn(
                    f"[check_collision_obstacle] [V2] 간격 {gap:.1f}s 지만 같은 자리 재차단 — 같은 상황으로 이어받는다 "
                    f"(replan 재시도 {self._static_replan_retry}회).")
                gap = 0.0
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
        self._static_last_hit_xy = None          # [09-28 현장 이상 2] 같은 자리 기준도 지운다
        self._static_release_xy = None
        self._static_left_spot = False

    # ---------------- [FIX B-7] agent 쪽 재진입/재시도 추적 (정적 쪽과 같은 형태) ----------------
    def _agent_release(self, now: Time, after_replan: bool = False) -> None:
        """agent pause 를 풀 때 호출. 다음 진입이 간격을 잴 수 있게 기록한다.
        after_replan=True 면 replan 뒤의 해제라 누적 경과는 0 부터, 재시도 횟수는 유지."""
        if after_replan:
            self._agent_last_elapsed = 0.0
        elif self._agent_pause_start_time is not None:
            self._agent_last_elapsed = (now - self._agent_pause_start_time).nanoseconds * 1e-9
        self._agent_last_release_t = now
        if self.v2_enable and self._locked_target_id and int(self._locked_target_id) not in self._cached_agents:
            self._locked_target_id = 0      # [V2.16] 캐시에서 사라진 상대를 계속 가리키지 않는다 (v2 에서만 — amhs 이관 S2)
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



    # ==================================================================
    # [V2] 정책 v2 — 자기 정보(M-9), 이웃 속도 추정, 정지 분류, 같은 경로 겹침 판정,
    #      감속 대기/속도 맞추기, 동적 우선순위, 상호 차단 해소, 무진전 보고
    # ==================================================================
    def _on_own_path(self, msg: Path):
        self._own_path = msg
        self._own_path_t = self.get_clock().now()

    def _on_own_twist(self, msg: Twist):
        self._own_twist = msg

    def _on_speed_ctrl(self, msg: ModifierControl):
        """velocity_modifier 명령을 엿본다. 내가 보낸 것이 아니면 '외부 기준값' 으로 기억했다가
        감속 대기가 끝날 때 그대로 되돌린다 (관제·다른 노드가 건 제한을 지우지 않기 위해)."""
        mine = self._last_speed_cmd
        if mine is not None and int(msg.command_type) == mine[0] \
                and abs(float(msg.linear_value) - mine[1]) < 1e-4 \
                and abs(float(msg.angular_value) - mine[2]) < 1e-4:
            return
        self._external_speed_ctrl = msg
        self.get_logger().info(
            f"[V2] 외부 속도 명령 기억: type {msg.command_type} lin {msg.linear_value:.2f} ang {msg.angular_value:.2f}")

    def _own_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self._tf_buffer.lookup_transform(self.global_frame, self.base_frame, Time())
        except Exception:                                    # noqa: BLE001
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (tf.transform.translation.x, tf.transform.translation.y, yaw)

    def _v2_record_agent(self, a: MultiAgentInfo, now: Time) -> None:
        mid = int(a.machine_id)
        t = now.nanoseconds * 1e-9
        h = self._agent_hist.setdefault(mid, deque(maxlen=64))
        p = a.current_pose.pose.position
        if math.isfinite(p.x) and math.isfinite(p.y):
            h.append((t, float(p.x), float(p.y)))
            # [V2.34b] 자세 이력은 3 s 만 남기므로, 오래 멈춘 시간은 따로 잰다 (마지막으로 0.3 m 넘게 움직인 시각)
            anc = self._agent_still.get(mid)
            if anc is None or math.hypot(float(p.x) - anc[0], float(p.y) - anc[1]) > 0.3:
                self._agent_still[mid] = (float(p.x), float(p.y), t, t)
            else:
                self._agent_still[mid] = (anc[0], anc[1], anc[2], t)
        while h and t - h[0][0] > 3.0:
            h.popleft()
        if int(a.status.phase) == AgentStatus.STATUS_PAUSE:
            self._agent_pause_since.setdefault(mid, now)
        else:
            self._agent_pause_since.pop(mid, None)
        # [V2.1] phase 와 무관한 '정지 시작' 추적: 앵커에서 0.3 m 안이면 유지, 벗어나면 다시 시작 (이력 창이 3 s 뿐이라 따로 둔다)
        anc = self._agent_stop_since.get(mid)
        if anc is None or math.hypot(p.x - anc[1], p.y - anc[2]) > 0.3:
            self._agent_stop_since[mid] = (t, float(p.x), float(p.y))

    def _agent_speed(self, mid: int, now: Time) -> Optional[float]:
        """자세 이력으로 추정한 속도 [m/s]. 표본이 부족하면 None."""
        h = self._agent_hist.get(int(mid))
        if not h or len(h) < 2:
            return None
        t_now = now.nanoseconds * 1e-9
        t1, x1, y1 = h[-1]
        if t_now - t1 > 2.0:
            return None                      # 소식이 오래됐다
        ref = None
        for (t0, x0, y0) in h:
            if t1 - t0 <= self.agent_speed_window + 0.6:
                ref = (t0, x0, y0)
                break
        if ref is None or t1 - ref[0] < 0.3:
            return None
        return math.hypot(x1 - ref[1], y1 - ref[2]) / (t1 - ref[0])

    def _agent_is_moving(self, agent: MultiAgentInfo, now: Time) -> bool:
        sp = self._agent_speed(agent.machine_id, now)
        if sp is not None:
            return sp > self.agent_moving_mps
        # 이력이 없으면 v1 판정(phase + twist) 을 뒤집어 쓴다
        return not self._check_vehicle_status(agent)

    def _agent_stop_class(self, agent: MultiAgentInfo, now: Time) -> str:
        if self._check_vehicle_immobile(agent):
            return "immobile"
        if self._agent_is_moving(agent, now):
            return "moving"
        ph = int(agent.status.phase)
        if ph in self._yielding_phases or ph == AgentStatus.STATUS_MOVING:
            return "yielding"            # MOVING 인데 속도 0 = 관제 phase 갱신 지연, 곧 움직일 정지로 본다
        if ph in self._working_phases:
            return "working"
        return "unknown"

    def _my_path_points(self) -> List[Tuple[float, float]]:
        pts: List[Tuple[float, float]] = []
        now = self.get_clock().now()
        if self._own_path is not None and self._own_path_t is not None \
                and (now - self._own_path_t).nanoseconds * 1e-9 < 3.0:
            pts = [(p.pose.position.x, p.pose.position.y) for p in self._own_path.poses]
        if len(pts) < 2:
            me = self._cached_agents.get(self.my_id)
            if me is not None:
                pts = [(p.pose.position.x, p.pose.position.y) for p in me.truncated_path.poses]
        return [(x, y) for (x, y) in pts if math.isfinite(x) and math.isfinite(y)]

    @staticmethod
    def _seg_dist_dir(px, py, ax, ay, bx, by):
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy
        if l2 < 1e-9:
            return math.hypot(px - ax, py - ay), None
        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / l2))
        cx, cy = ax + t * dx, ay + t * dy
        return math.hypot(px - cx, py - cy), math.atan2(dy, dx)

    def _same_path_v2(self, agent: MultiAgentInfo) -> int:
        """겹치는 경로 구간 비율 + 국소 진행 방향 일치로 같은 경로를 판정한다 (M-10).
        벡터 하나로 비교하던 v1 은 모서리에서 45° 를 넘어 호송을 교차로 분류했다."""
        mine = self._my_path_points()
        # [S11B v2 1차] 관제 경로는 10점(0.8 m) 뿐이라 4 m 짜리 내 경로와 겹치는 비율이 늘 0.2 이하 → 호송이 교차로 분류됐다.
        # 상대 경로를 진행 방향으로 최소 4 m 까지 연장한 뒤 비교한다.
        now = self.get_clock().now()
        sp = self._agent_speed(agent.machine_id, now) or 0.0
        other = self._agent_future_points(agent, max(sp, 4.0 / max(self.predict_horizon, 1.0)), now)
        if len(mine) < 2 or len(other) < 2:
            return self._check_vehicle_path(agent)          # 자료 부족 → v1 판정
        step = max(1, len(mine) // 30)
        n = hit = 0
        for i in range(0, len(mine) - 1, step):
            (x0, y0), (x1, y1) = mine[i], mine[min(i + step, len(mine) - 1)]
            if math.hypot(x1 - x0, y1 - y0) < 0.02:
                continue
            my_dir = math.atan2(y1 - y0, x1 - x0)
            best_d, best_dir = float("inf"), None
            for j in range(len(other) - 1):
                d, dr = self._seg_dist_dir(x0, y0, *other[j], *other[j + 1])
                if d < best_d:
                    best_d, best_dir = d, dr
            n += 1
            if best_d <= self.same_path_tube and best_dir is not None \
                    and abs(self._ang_wrap(my_dir - best_dir)) <= self.same_path_heading:
                hit += 1
        if n == 0:
            return self._check_vehicle_path(agent)
        ratio = hit / n
        self.get_logger().info(f"[V2] same_path ratio {ratio:.2f} (n={n}) vs agent {agent.machine_id}",
                               throttle_duration_sec=2.0)
        return SAME_PATH if ratio >= self.same_path_overlap_ratio else DIFFERENT_PATH

    def _target_blocked_by_me(self, agent: MultiAgentInfo) -> bool:
        """상대의 계획 경로가 내 발자국(반경 mutual_block_radius) 을 지나는가 = 상대가 나를 기다리는가."""
        me = self._own_pose()
        if me is None:
            return False
        r = self.mutual_block_radius + 0.315
        for (x, y) in self._agent_future_points(agent, 4.0 / max(self.predict_horizon, 1.0), self.get_clock().now()):
            if math.hypot(x - me[0], y - me[1]) <= r:
                return True
        return False

    def _target_dist(self, agent: MultiAgentInfo) -> float:
        me = self._own_pose()
        if me is None:
            return 0.0
        p = agent.current_pose.pose.position
        return math.hypot(p.x - me[0], p.y - me[1])

    def _target_in_front(self, agent: MultiAgentInfo) -> bool:
        me = self._own_pose()
        if me is None:
            return False
        p = agent.current_pose.pose.position
        ang = math.atan2(p.y - me[1], p.x - me[0]) - me[2]
        return abs(self._ang_wrap(ang)) < math.radians(70.0)

    def _target_reports_me(self, agent: MultiAgentInfo) -> bool:
        """[사용자 지시 09-19] 관제가 주는 cross_agent_id(= 관제의 detect_amr_id) 가 나면, 상대는 **나 때문에** 막힌 것 —
        2-사이클(서로 기다림)의 직접 증거. 0 이면 정보 없음(기하 추정으로 폴백)."""
        return int(agent.cross_agent_id) == int(self.my_id)

    def _target_blocked_by_me_ext(self, agent: MultiAgentInfo, now: Time, dt: float) -> bool:
        """관제가 상대의 감지 대상을 나로 알려 주거나, 경로가 내 발자국을 지나거나, 우선 로봇이 내 옆에서 dt 동안
        움직이지 못하고 있으면 (M-8)."""
        if self._target_reports_me(agent) and not self._agent_is_moving(agent, now):
            return True
        if self._target_blocked_by_me(agent):
            return True
        if dt < self.mutual_block_replan_sec:
            return False
        sp = self._agent_speed(agent.machine_id, now)
        return (sp is not None and sp < self.agent_moving_mps
                and self._target_dist(agent) <= self.mutual_block_near)

    def _hit_dist(self) -> float:
        me = self._own_pose()
        if me is None:
            return 0.0                    # 자세를 모르면 정지 쪽으로 (보수적)
        hx, hy = self.latest_agent_collision_xy
        return math.hypot(hx - me[0], hy - me[1])

    def _i_have_priority(self, agent: MultiAgentInfo, now: Time) -> bool:
        """동적 우선순위: aging 단계 (양쪽이 같은 입력으로 같은 답을 내야 하므로 관측 가능한
        PAUSE 지속시간만 쓴다) → machine_id."""
        if not self.dynamic_priority_enable:
            return self.my_id < agent.machine_id
        lvl_me = 0
        if self._v2_pause_since is not None:
            lvl_me = min(self.aging_max_level,
                         int((now - self._v2_pause_since).nanoseconds * 1e-9 / self.aging_step_sec))
        lvl_other = 0
        since = self._agent_pause_since.get(int(agent.machine_id))
        if since is not None:
            lvl_other = min(self.aging_max_level,
                            int((now - since).nanoseconds * 1e-9 / self.aging_step_sec))
        if lvl_me != lvl_other:
            return lvl_me > lvl_other
        return self.my_id < agent.machine_id

    def _decide_v2(self, agent: MultiAgentInfo, collision_xy: Tuple[float, float]) -> Tuple[MovingCommand, MovingStopType]:
        now = self.get_clock().now()
        mid = int(agent.machine_id)
        dist = self._hit_dist()
        bdist = self._target_dist(agent)
        far = (self.slow_wait_speed > 0.0 and dist > self.slow_wait_min_dist
               and bdist > self.slow_wait_min_body)
        slow_or_pause = "slow" if far else "pause"
        self._v2_mutual_applied = False

        def out(cmd, st, mode, timeout, reason):
            self._v2_pending_mode = mode
            self._v2_pending_timeout = float(timeout)
            self._v2_reason = reason
            self.get_logger().warn(
                f"[V2 decide] agent {mid} → {st.name}/{cmd.name} mode={mode} timeout={timeout:.0f}s "
                f"| {reason} | hit {dist:.2f} m, body {bdist:.2f} m")
            return cmd, st

        if self._check_vehicle_manual_mode(agent):
            return out(MovingCommand.WAIT_DETECT_AMR, MovingStopType.TYPE_1, "pause",
                       self.wait_detect_sec, "manual")
        cls = self._agent_stop_class(agent, now)
        if cls == "immobile":
            return out(MovingCommand.WAIT, MovingStopType.TYPE_11, "pause",
                       self.wait_obstacle_sec, f"immobile phase {agent.status.phase}")
        moving = (cls == "moving")
        self._v2_target_moving = moving
        sp = self._agent_speed(mid, now)
        sp_txt = f"{sp:.2f}" if sp is not None else "?"
        # [S11B v2] 정지한 상대가 **내 goal 위**에 있으면 돌아가 봐야 goal 은 그대로 막혀 있다 — 2 s replan 반복(v1) 대신
        # 정지 분류 시간만큼 기다린다 (그 사이 상대가 떠나면 조기 재개). 마지막 goal 도 같은 대기 → 만료 후 replan(현행 경로).
        self._v2_last_goal_report = False
        self._v2_goal_occ_wait = bool(not moving and self.latest_agent_goal_occupied)
        if not moving and self.latest_agent_goal_occupied:
            wait = (self.stopped_yielding_wait if cls == "yielding"
                    else self.stopped_working_wait if cls == "working" else self.wait_detect_sec)
            if self.latest_agent_last_goal_occupied:
                # 마지막 goal 점유는 현행 유지(사용자 결정): goal_occupied_timeout_sec 뒤 STOP → 관제 보고. replan 은 무의미.
                self._v2_last_goal_report = True
                return out(MovingCommand.WAIT_DETECT_AMR, MovingStopType.TYPE_12, slow_or_pause,
                           self.goal_occupied_timeout_sec,
                           f"my LAST goal occupied by stopped agent ({cls}) → wait {self.goal_occupied_timeout_sec:.0f}s then report")
            return out(MovingCommand.WAIT_DETECT_AMR, MovingStopType.TYPE_12, slow_or_pause, wait,
                       f"my goal occupied by stopped agent ({cls}, phase {agent.status.phase}) → wait, no replan")
        if self._same_path_v2(agent) == SAME_PATH:
            if moving:
                mode = "match" if (self.speed_match_enable and far) else slow_or_pause
                return out(MovingCommand.WAIT_ABNORMAL, MovingStopType.TYPE_2, mode,
                           self.wait_abnormal_long_sec if self.use_reroute else self.wait_abnormal_short_sec,
                           f"same path, moving {sp_txt} m/s → speed match")
            wait = (self.stopped_yielding_wait if cls == "yielding"
                    else self.stopped_working_wait if cls == "working" else self.wait_detect_sec)
            return out(MovingCommand.WAIT_DETECT_AMR, MovingStopType.TYPE_12, slow_or_pause, wait,
                       f"same path, stopped ({cls}, phase {agent.status.phase})")
        prio = self._i_have_priority(agent, now)
        if moving:
            if prio:
                return out(MovingCommand.WAIT_DETECT_AMR, MovingStopType.TYPE_10, slow_or_pause,
                           self.slow_wait_max_sec if far else self.moving_target_pause_sec,
                           f"crossing, target moving {sp_txt} m/s, I have priority → let it pass")
            return out(MovingCommand.WAIT_OHTHER_AMR, MovingStopType.TYPE_9, slow_or_pause,
                       self.wait_other_long_sec if self.use_reroute else self.wait_other_short_sec,
                       f"crossing, target moving {sp_txt} m/s, target has priority")
        if prio:
            return out(MovingCommand.WAIT_DETECT_AMR, MovingStopType.TYPE_10, slow_or_pause,
                       self.wait_detect_sec, f"blocker stopped ({cls}), I have priority → go around")
        if cls in ("yielding", "working") and self._target_reports_me(agent):
            return out(MovingCommand.WAIT_OHTHER_AMR, MovingStopType.TYPE_9, slow_or_pause,
                       self.mutual_block_replan_sec,
                       f"mutual block (관제 cross_agent_id={agent.cross_agent_id} = me): priority robot waits for me → I move (PIBT)")
        if cls == "yielding" and self._target_blocked_by_me(agent):
            return out(MovingCommand.WAIT_OHTHER_AMR, MovingStopType.TYPE_9, slow_or_pause,
                       self.mutual_block_replan_sec,
                       "mutual block: priority robot is paused on a path through me → I move (PIBT)")
        return out(MovingCommand.WAIT_OHTHER_AMR, MovingStopType.TYPE_9, slow_or_pause,
                   self.wait_other_long_sec if self.use_reroute else self.wait_other_short_sec,
                   f"target stopped ({cls}) and has priority")

    # ---- 속도 명령 ----
    def _v2_set_speed_limit(self, lin: float, ang: float) -> None:
        m = ModifierControl()
        m.command_type = ModifierControl.TYPE_SPEED_LIMIT
        m.linear_value = float(lin)
        m.angular_value = float(ang)
        self._last_speed_cmd = (int(m.command_type), float(m.linear_value), float(m.angular_value))
        self.pub_speed_ctrl.publish(m)
        self._speed_limited = True
        self.get_logger().warn(f"[V2] speed limit {lin:.2f} m/s / {ang:.2f} rad/s ({self._v2_mode})")

    def _v2_restore_speed(self, why: str) -> None:
        ext = self._external_speed_ctrl
        m = ModifierControl()
        if ext is not None:
            m.command_type = ext.command_type
            m.linear_value = ext.linear_value
            m.angular_value = ext.angular_value
        else:
            m.command_type = ModifierControl.TYPE_SPEED_SCALE
            m.linear_value = 1.0
            m.angular_value = 0.0
        self._last_speed_cmd = (int(m.command_type), float(m.linear_value), float(m.angular_value))
        self.pub_speed_ctrl.publish(m)
        self._speed_limited = False
        self._pre_slow_active = False
        self.get_logger().warn(f"[V2] speed restored ({why}): type {m.command_type} lin {m.linear_value:.2f}")

    def _v2_apply_speed(self, now: Time) -> None:
        if self._v2_mode == "match":
            sp = self._agent_speed(self._locked_target_id, now)
            v = max(self.speed_match_min, (sp or 0.0) * self.speed_match_ratio)
            self._v2_set_speed_limit(v, 1.0)
        elif self._v2_mode == "slow":
            self._v2_set_speed_limit(self.slow_wait_speed, self.slow_wait_angular)

    def _v2_apply_mode_change(self, new_mode: str, now: Time, why: str) -> None:
        if self._v2_mode == "pause":
            return                                   # 이미 정지 중이면 그대로 (보수적)
        if new_mode == "pause":
            self._v2_escalate_to_pause(now, why)
        elif new_mode != self._v2_mode:
            self._v2_mode = new_mode
            self._v2_apply_speed(now)

    def _v2_escalate_to_pause(self, now: Time, why: str) -> None:
        if self._junction_should_clear(now):
            return                          # [V2.2] 교차로를 비우고 나서 정지한다 (감속 상태 유지)
        self._junction_clear_since = None
        self.get_logger().warn(f"[V2] {self._v2_mode} → PAUSE ({why})")
        self._v2_mode = "pause"
        self._v2_pause_since = now
        if self._speed_limited:
            self._v2_restore_speed("pause")
        self._publish_pause()
        self._publish_state(f"{self.current_agent_stop_type.name}: PAUSE (escalated: {why})")

    def _v2_phase1_tick(self, now: Time) -> bool:
        """Phase 1 의 v2 부분. True 를 돌려주면 v1 의 타임아웃 분기를 이번 틱에는 건너뛴다."""
        if self._agent_pause_start_time is None:
            return False
        dt = (now - self._agent_pause_start_time).nanoseconds * 1e-9
        agent = self._cached_agents.get(self._locked_target_id)

        if agent is not None and dt >= 2.0:
            # (a) 우선인 내가 '주행 중' 상대를 기다렸는데 상대가 섰다(나에게 양보) → 상호 대기가 된다.
            #     정지 상대 규칙으로 다시 결정한다 (TYPE_10 2 s → replan 으로 돌아간다).
            if (self.current_agent_stop_type == MovingStopType.TYPE_10 and self._v2_target_moving
                    and self._agent_stop_class(agent, now) in ("yielding", "working")):
                cmd, st = self._decide_for_current_target()
                self.current_agent_command, self.current_agent_stop_type = cmd, st
                self.agent_pause_timeout_sec = dt + self._pause_timeout_for(cmd)
                self.get_logger().warn(
                    f"[V2] 주행 중이던 상대 {agent.machine_id} 가 정지 → 재결정 {st.name} "
                    f"mode={self._v2_pending_mode} timeout {self.agent_pause_timeout_sec:.0f}s")
                self._publish_state(f"{st.name}: {self._v2_mode.upper()} (target stopped, re-decided)")
                if self._v2_mode != "pause" and self._v2_pending_mode == "pause":
                    self._v2_escalate_to_pause(now, "target stopped")
                return True
            # (b) 양보 중(TYPE_9) 인데 우선 로봇이 나 때문에 못 움직인다 (경로가 내 발자국을 지나거나
            #     내 옆에서 정지·회복 반복, M-8) → mutual_block_replan_sec 뒤 내가 비켜준다.
            if (self.current_agent_stop_type == MovingStopType.TYPE_9 and not self._v2_mutual_applied
                    and self._target_blocked_by_me_ext(agent, now, dt)):
                new_to = min(self.agent_pause_timeout_sec, dt + max(2.0, self.mutual_block_replan_sec - dt))
                self._v2_mutual_applied = True
                src = "관제 cross_agent_id" if self._target_reports_me(agent) else "기하 추정"
                self.get_logger().warn(
                    f"[V2] mutual block ({src}): 우선 로봇 {agent.machine_id} 이 나 때문에 못 움직인다 "
                    f"(dt {dt:.0f}s, body {self._target_dist(agent):.2f} m) → 대기 한도 {self.agent_pause_timeout_sec:.0f}s → {new_to:.0f}s, 내가 replan 으로 비켜준다")
                self.agent_pause_timeout_sec = new_to
                self._publish_state("TYPE_9: mutual block → I move")

        if self._v2_mode == "pause":
            if self._v2_last_goal_report and dt >= self.agent_pause_timeout_sec:
                self.get_logger().error(
                    f"[V2] 마지막 goal 을 agent {self._locked_target_id} 가 {dt:.0f}s 동안 점유 → STOP, 관제에 보고 (현행 last-goal 규칙과 동일)")
                self._publish_state(f"STOP (last goal occupied by agent {self._locked_target_id}, {dt:.0f}s)")
                self.pub_cmd_stop.publish(UInt8(data=1))
                self.nav_stop_complete_ = False
                self._nav_stop_wait_start = now
                self._agent_reset_osc()
                self.is_processing_agent_pause = False
                self._agent_pause_start_time = None
                self._agent_clear_start_time = None
                self._v2_last_goal_report = False
                self._v2_pause_since = None
                return True
            # [후진 양보] BackUp 액션 진행 중 — 결과를 기다린다
            if self._backup_state in ("sending", "running", "waiting"):
                if self._backup_started_at is not None and self._backup_state != "waiting" and \
                        (now - self._backup_started_at).nanoseconds * 1e-9 > self.backup_time_allowance + 10.0:
                    self.get_logger().error("[V2 backoff] BackUp 응답 없음 → 취소하고 대기 계속")
                    self._backup_cancel(); self._backup_state = "failed"
                return True
            if self._backup_state == "succeeded":
                self._backup_state = "idle"
                self.get_logger().warn("[V2 backoff] BackUp 완료 → replan 하고 재개한다")
                self.agent_pause_timeout_sec = dt          # 즉시 Phase 2 (replan)
                return False
            if self._backup_state == "failed":
                self._backup_state = "idle"                # 후방이 막혔거나 거부됨 → 후진 없이 대기 계속 (상대별 1회)
                self._publish_state("TYPE_9: BACKOFF failed (rear blocked) → keep waiting")
                return True
            # [후진 양보] 낮은 쪽(TYPE_9) 이 우선 로봇과 대면(몸체 < body_m, 앞쪽) 이고 mutual block 이 잡혔으면 물러난다
            # [S17 v2] goal 점유 대기 중에도: 내 goal 위의 상대가 내 앞 가까이에서 못 움직이면(플래너 실패 등) 내가 길을
            #   막고 있는 것 → 후진해 비켜준다 (우선순위와 무관 — 상대가 떠나야 내 goal 이 빈다).
            goal_occ_stalled = (self._v2_goal_occ_wait and dt >= self.mutual_block_replan_sec and agent is not None
                                and (self._agent_speed(agent.machine_id, now) or 0.0) < self.agent_moving_mps
                                and self._target_dist(agent) <= self.mutual_block_near)
            if (self.yield_backoff_enable and agent is not None
                    and ((self.current_agent_stop_type == MovingStopType.TYPE_9 and self._v2_mutual_applied)
                         or goal_occ_stalled)
                    and self._backoff_done_for != self._locked_target_id
                    and self._target_dist(agent) <= max(self.yield_backoff_body, self.mutual_block_near if goal_occ_stalled else 0.0)
                    and self._target_in_front(agent)
                    and self._op_may('_v2_phase1_tick')):       # [OP] 불허면 공지·1회성 표시 없이 다음 분기로
                self._backoff_done_for = self._locked_target_id
                self.get_logger().warn(
                    f"[V2 backoff] 로봇 {agent.machine_id} 과 대면 (body {self._target_dist(agent):.2f} m) → "
                    f"BackUp 액션 {self.yield_backoff_m} m @ {self.yield_backoff_speed} m/s (후방 충돌 검사 포함)")
                self._publish_state(f"{self.current_agent_stop_type.name}: BACKOFF {self.yield_backoff_m} m (BackUp action)")
                self._backup_send(now)
                return True
            # 양보 중(TYPE_9) 인데 aging 으로 내가 우선이 되면 역할을 바꾼다 (5 s 마다 확인)
            if (agent is not None and self.current_agent_stop_type == MovingStopType.TYPE_9
                    and self.dynamic_priority_enable
                    and (self._v2_prio_reeval_t is None
                         or (now - self._v2_prio_reeval_t).nanoseconds * 1e-9 >= 5.0)):
                self._v2_prio_reeval_t = now
                if self._i_have_priority(agent, now):
                    self.get_logger().warn(
                        f"[V2] aging: 상대 {agent.machine_id} 보다 오래 기다렸다 → 우선권 획득, TYPE_9 → TYPE_10 (replan {self.wait_detect_sec:.0f}s 뒤)")
                    self.current_agent_command = MovingCommand.WAIT_DETECT_AMR
                    self.current_agent_stop_type = MovingStopType.TYPE_10
                    self.agent_pause_timeout_sec = dt + self.wait_detect_sec
                    self._publish_state(f"TYPE_10: PAUSE (aging priority)")
            return False

        # ---- 감속 대기 / 속도 맞추기 중 ----
        dist = self._hit_dist()
        bdist = self._target_dist(agent) if agent is not None else 0.0
        if dist <= self.slow_wait_min_dist or bdist <= self.slow_wait_min_body:
            self._v2_escalate_to_pause(now, f"hit {dist:.2f} m / body {bdist:.2f} m")
            if self.current_agent_stop_type == MovingStopType.TYPE_10 and self._v2_target_moving:
                self.agent_pause_timeout_sec = min(self.agent_pause_timeout_sec, dt + self.moving_target_pause_sec)
            return True
        if agent is not None and self._v2_mode == "match" and not self._agent_is_moving(agent, now):
            # 선두가 섰다 → 정지 분류로 다시 결정
            cmd, st = self._decide_for_current_target()
            self.current_agent_command, self.current_agent_stop_type = cmd, st
            self.agent_pause_timeout_sec = dt + self._pause_timeout_for(cmd)
            self.get_logger().warn(f"[V2] match 상대가 정지 → 재결정 {st.name} mode={self._v2_pending_mode}")
            if self._v2_pending_mode == "pause":
                self._v2_escalate_to_pause(now, "leader stopped")
            else:
                self._v2_mode = self._v2_pending_mode
                self._v2_apply_speed(now)
            return True
        if dt >= self.agent_pause_timeout_sec:
            # 감속 대기 상한 → 정지하고 1.5 s 뒤 v1 Phase 2 (replan) 가 실행되게 한다
            self._v2_escalate_to_pause(now, f"slow-wait timeout {dt:.0f}s")
            self.agent_pause_timeout_sec = dt + 1.5
            return True
        if self._v2_mode == "match" and (self._v2_prio_reeval_t is None
                                         or (now - self._v2_prio_reeval_t).nanoseconds * 1e-9 >= 1.0):
            self._v2_prio_reeval_t = now
            self._v2_apply_speed(now)          # 선두 속도 추종 갱신
        self.get_logger().info(
            f"[check_collision_agent][Phase 1][V2] {self._v2_mode}: {dt:.1f}s / {self.agent_pause_timeout_sec:.1f}s "
            f"(hit {dist:.2f} m, {self.current_agent_stop_type.name})", throttle_duration_sec=1.0)
        return True

    @staticmethod
    def _cum_dist(pts):
        out = [0.0]
        for k in range(1, len(pts)):
            out.append(out[-1] + math.hypot(pts[k][0] - pts[k - 1][0], pts[k][1] - pts[k - 1][1]))
        return out

    def _agent_future_points(self, a: MultiAgentInfo, sp: float, now: Time) -> List[Tuple[float, float]]:
        """상대의 앞으로 horizon 동안의 위치. 관제가 주는 경로는 짧다 (현장 프로토콜 궤적 10점 = 릴레이도 10점,
        4 m 경로의 앞 0.8 m 만) → 마지막 점에서 진행 방향으로 sp·horizon 까지 직선 연장한다. 경로가 없으면 자세에서."""
        pts = [(p.pose.position.x, p.pose.position.y) for p in a.truncated_path.poses]
        pts = [(x, y) for (x, y) in pts if math.isfinite(x) and math.isfinite(y)]
        px, py = a.current_pose.pose.position.x, a.current_pose.pose.position.y
        if not pts:
            pts = [(px, py)]
        # 진행 방향: 경로 마지막 구간 → 없으면 자세 이력 변위 → 없으면 yaw
        d = None
        if len(pts) >= 2 and math.hypot(pts[-1][0] - pts[-2][0], pts[-1][1] - pts[-2][1]) > 0.02:
            d = math.atan2(pts[-1][1] - pts[-2][1], pts[-1][0] - pts[-2][0])
        else:
            h = self._agent_hist.get(int(a.machine_id))
            if h and len(h) >= 2 and math.hypot(h[-1][1] - h[0][1], h[-1][2] - h[0][2]) > 0.05:
                d = math.atan2(h[-1][2] - h[0][2], h[-1][1] - h[0][1])
            else:
                d = self._get_yaw(a.current_pose.pose)
        need = sp * self.predict_horizon - self._cum_dist(pts)[-1]
        x, y = pts[-1]
        n = int(max(0.0, need) / 0.25)
        for k in range(1, n + 1):
            pts.append((x + 0.25 * k * math.cos(d), y + 0.25 * k * math.sin(d)))
        return pts

    def _check_predicted_conflicts(self) -> None:
        """[V2] 내 경로와 이웃 경로가 conflict_dist 안에서 만나고, 둘 다 horizon 안에 그 지점에 닿으며 도착 시각
        차이가 gap 보다 작으면 → 우선순위가 낮은 내가 미리 감속한다 (validator HIT 이전, 예약 영역 앞 대기)."""
        if self._op_frozen():
            return                                    # [OP] 관제 pause 중 — 판단 정지
        if not (self.v2_enable and self.predict_enable):
            return
        active = (self.is_processing_agent_pause or self.is_processing_replan_pause
                  or self.is_processing_goal_occupied_pause or self.is_processing_last_goal_occupied_pause
                  or self.nav_stop_complete_ is False
                  or self.current_robot_status not in ('DRIVING', 'PLANNING', 'RECEIVED_GOAL'))
        if active:
            if self._pre_slow_active and not self.is_processing_agent_pause:
                self._v2_restore_speed("pre-slow: SM active")
            self._pre_slow_active = False if self.is_processing_agent_pause else self._pre_slow_active
            return
        now = self.get_clock().now()
        mine = self._my_path_points()
        if len(mine) < 2:
            if self._pre_slow_active:
                self._v2_restore_speed("pre-slow: no path")
            return
        v_me = max(0.15, abs(float(self._own_twist.linear.x)))
        s_me = self._cum_dist(mine)
        best = None   # (v_cmd, mid, s_i, eta_me, eta_them)
        dbg: List[str] = []
        for mid, a in list(self._cached_agents.items()):
            if mid == self.my_id:
                continue
            sp = self._agent_speed(mid, now)
            if sp is None or sp <= self.agent_moving_mps:
                dbg.append(f"a{mid}:stopped"); continue    # 정지 상대는 SM 이 맡는다
            other = self._agent_future_points(a, sp, now)
            if len(other) < 2:
                dbg.append(f"a{mid}:nopath"); continue
            s_th = self._cum_dist(other)
            hit = None
            for i in range(len(mine)):
                if s_me[i] > v_me * self.predict_horizon:
                    break
                for j in range(len(other)):
                    if s_th[j] > sp * self.predict_horizon:
                        break
                    if math.hypot(mine[i][0] - other[j][0], mine[i][1] - other[j][1]) <= self.predict_conflict_dist:
                        hit = (i, j); break
                if hit:
                    break
            if hit is None:
                dm = min(math.hypot(mine[i][0] - other[j][0], mine[i][1] - other[j][1])
                         for i in range(0, len(mine), 3) for j in range(len(other)))
                dbg.append(f"a{mid}:nohit(min {dm:.1f}m, sp {sp:.2f}, pts {len(mine)}/{len(other)})"); continue
            i, j = hit
            eta_me, eta_th = s_me[i] / v_me, s_th[j] / sp
            if abs(eta_me - eta_th) >= self.predict_gap or s_me[i] < 0.3:
                dbg.append(f"a{mid}:eta {eta_me:.1f}/{eta_th:.1f} s@{s_me[i]:.1f}m"); continue
            if self._i_have_priority(a, now):
                dbg.append(f"a{mid}:i-have-priority"); continue     # 상대가 미리 감속할 차례
            v_cmd = max(self.slow_wait_speed, min(v_me, s_me[i] / (eta_th + self.predict_gap)))
            if best is None or v_cmd < best[0]:
                best = (v_cmd, mid, s_me[i], eta_me, eta_th)
        if best is None:
            if self.predict_debug and dbg and (self._predict_dbg_t is None
                                              or (now - self._predict_dbg_t).nanoseconds * 1e-9 >= 3.0):
                self._predict_dbg_t = now
                self.get_logger().info(f"[V2 predict dbg] v_me {v_me:.2f} path {len(mine)}pts " + " ".join(dbg))
            if self._pre_slow_active:
                self._v2_restore_speed("pre-slow: conflict cleared")
                self._publish_state("RUN (pre-slow cleared)")
            return
        v_cmd, mid, s_i, eta_me, eta_th = best
        if not self._pre_slow_active or abs(v_cmd - self._pre_slow_v) > 0.03 or mid != self._pre_slow_target:
            self._v2_mode = "slow"
            self._v2_set_speed_limit(v_cmd, self.slow_wait_angular + 0.5)
            self._v2_mode = "pause"
            self._pre_slow_active, self._pre_slow_v, self._pre_slow_target = True, v_cmd, mid
            self.get_logger().warn(
                f"[V2 predict] agent {mid} 와 {s_i:.1f} m 앞 교차 예상 (나 {eta_me:.1f}s / 상대 {eta_th:.1f}s) "
                f"→ 미리 {v_cmd:.2f} m/s 로 감속")
            self._publish_state(f"PRE-SLOW {v_cmd:.2f} m/s (agent {mid}, {s_i:.1f} m)")

    # ---- BackUp 액션 (후진 양보) ----
    def _man_begin(self, kind: str, target: float, now: Time) -> None:
        """직진 기동(backup/drive) 문맥 시작 — stop-and-go 재개용 (재시도 중이면 누적을 잇는다)."""
        if self._man_ctx is not None and self._man_ctx.get("kind") == kind and self._man_ctx.get("resuming"):
            self._man_ctx["resuming"] = False; self._man_ctx["traveled"] = 0.0
            return
        self._man_ctx = {"kind": kind, "target": float(target), "done": 0.0, "traveled": 0.0,
                         "deadline": now + Duration(seconds=self.stopgo_max_sec), "next": None, "retries": 0, "resuming": False}

    def _man_feedback(self, msg) -> None:
        if self._man_ctx is not None:
            self._man_ctx["traveled"] = float(getattr(msg.feedback, "distance_traveled", 0.0) or 0.0)

    def _man_aborted(self, now: Time, why: str) -> bool:
        """기동이 장애물 등으로 중단됨. stop-and-go 가 켜져 있고 시한 안이면 '정지 대기' 로 두고 True (실패 아님)."""
        c = self._man_ctx
        if not (self.stopgo_enable and c is not None and c["kind"] in ("backup", "drive")):
            return False
        c["done"] += c["traveled"]; c["traveled"] = 0.0
        remaining = c["target"] - c["done"]
        if remaining <= 0.1:
            return False                          # 사실상 다 갔다 → 호출자가 성공 처리하도록 False 를 주되 상태는 성공으로
        if now >= c["deadline"]:
            self.get_logger().error(f"[V2 stop-go] {c['kind']} 장애물 대기 {self.stopgo_max_sec:.0f}s 초과 (남은 {remaining:.2f} m) → 포기")
            return False
        c["retries"] += 1; c["next"] = now + Duration(seconds=self.stopgo_retry_sec); c["resuming"] = True
        self._backup_state = "waiting"
        self._retreat_clear_local(now)
        self.get_logger().warn(f"[V2 stop-go] {c['kind']} 중 장애물({why}) → 정지, {self.stopgo_retry_sec:.0f}s 뒤 비면 남은 {remaining:.2f} m 재개 (시도 {c['retries']})")
        self._publish_state(f"[V2 stop-go] {c['kind']} paused, {remaining:.2f} m left")
        return True

    # ---- [V2.23] 후방 여유 실측 (local costmap) ----
    def _on_local_costmap(self, msg: OccupancyGrid) -> None:
        self._lc_grid = msg
        self._lc_data = list(msg.data)
        self._lc_at = self.get_clock().now()

    def _on_local_footprint(self, msg: PolygonStamped) -> None:
        self._lc_fp = [(p.x, p.y) for p in msg.polygon.points]

    def _on_odom_for_rear(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._lc_odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    @staticmethod
    def _poly_contains(poly, x, y) -> bool:
        c = False; n = len(poly)
        for i in range(n):
            x1, y1 = poly[i]; x2, y2 = poly[(i + 1) % n]
            if (y1 > y) != (y2 > y):
                xin = (x2 - x1) * (y - y1) / (y2 - y1) + x1
                if x < xin:
                    c = not c
        return c

    def _footprint_max_cost(self, poly) -> Optional[int]:
        g = self._lc_grid; data = self._lc_data
        if g is None or data is None:
            return None
        res = g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        W, H = g.info.width, g.info.height
        xs = [p[0] for p in poly]; ys = [p[1] for p in poly]
        i0 = int((min(xs) - ox) / res); i1 = int((max(xs) - ox) / res) + 1
        j0 = int((min(ys) - oy) / res); j1 = int((max(ys) - oy) / res) + 1
        mx = -1
        for j in range(j0, j1 + 1):
            if not (0 <= j < H):
                return 100                      # 지도 밖 = nav2 의 NO_INFORMATION 과 같게 막힘으로 본다
            for i in range(i0, i1 + 1):
                if not (0 <= i < W):
                    return 100
                px = ox + (i + 0.5) * res; py = oy + (j + 0.5) * res
                if not self._poly_contains(poly, px, py):
                    continue
                v = data[j * W + i]
                if v < 0:
                    return 100                  # unknown = NO_INFORMATION(255) → nav2 도 충돌로 친다
                if v > mx:
                    mx = v
        return mx

    def _rear_blocked(self, dist: float, sign: float = -1.0) -> Optional[bool]:
        """sign<0 이면 뒤로, sign>0 이면 앞으로 dist m 가는 경로가 막혔는가.
        판단 못 하면 None (그 경우 호출자는 평소대로 보낸다).

        [V2.23a 09-23] Y2 에서 r2 가 `drive 중 장애물(status 6)` 로 9회 재시도했다 — 후진뿐 아니라
        **전진 기동(DriveOnHeading)** 도 같은 방식으로 거부된다. 그래서 방향을 인자로 받는다."""
        if not self.rear_check_enable:
            return None
        if self._lc_grid is None or self._lc_fp is None or self._lc_odom is None or self._lc_at is None:
            return None
        if (self.get_clock().now() - self._lc_at).nanoseconds * 1e-9 > self.rear_check_max_age:
            return None                          # 지도가 오래됐다 — 막혔다고 단정하지 않는다
        yaw = self._lc_odom[2]
        step = max(0.02, self._lc_grid.info.resolution)
        d = 0.0; target = float(dist) + self.rear_check_margin
        while d <= target + 1e-6:
            dx, dy = sign * d * math.cos(yaw), sign * d * math.sin(yaw)
            c = self._footprint_max_cost([(px + dx, py + dy) for px, py in self._lc_fp])
            if c is None:
                return None
            if c >= 100:
                self._rear_block_at = round(d, 2)
                return True
            d += step
        self._rear_block_at = None
        return False

    def _retreat_clear_local(self, now: Time) -> None:
        """[V2.22] 후퇴 재시도 직전에 local costmap 을 비운다 (남은 표식 때문에 영원히 거부되는 것을 막는다)."""
        if not self.retreat_clear_costmap:
            return
        if self._retreat_clear_at is not None and \
                (now - self._retreat_clear_at).nanoseconds * 1e-9 < self.retreat_clear_cooldown:
            return
        if not self._clear_local.service_is_ready():
            self.get_logger().warn("[V2 stop-go] local costmap clear 서비스가 아직 없다", throttle_duration_sec=30.0)
            return
        self._retreat_clear_at = now
        self._clear_local.call_async(Empty.Request())
        self.get_logger().warn("[V2 stop-go] 재시도 전에 local costmap 을 비운다 (남은 표식 제거, 실장애물은 즉시 재기록)")

    def _v2_maneuver_tick(self) -> None:
        if self._op_frozen():
            return                                    # [OP] 관제 pause 중 — 판단 정지
        c = self._man_ctx
        if self._backup_state != "waiting" or c is None or c.get("next") is None:
            return
        now = self.get_clock().now()
        if now < c["next"]:
            return
        remaining = c["target"] - c["done"]
        c["next"] = None
        if c["kind"] == "backup":
            self._backup_send(now, dist=remaining)
        else:
            self._drive_send(remaining)

    def _backup_send(self, now: Time, dist: Optional[float] = None) -> None:
        _caller = sys._getframe(1).f_code.co_name
        if not self._op_gate(_caller):                # [OP] 관제 명령 우선·상태별 허가
            self._backup_state = "failed"; self._man_ctx = None
            return
        if dist is None and _caller not in ("_maneuver_next", "_v2_maneuver_tick") \
                and not self._retreat_gate_check(_caller):
            self._backup_state = "failed"                 # [V2.34] 모든 규칙이 succeeded/failed 로 뒷정리한다
            return
        if self._yield_bt_ready(now) and not self._yield_active:
            a = self._cached_agents.get(self._locked_target_id)
            away = (a.current_pose.pose.position.x, a.current_pose.pose.position.y) if a is not None else None
            if not self._yield_start(now, away):
                self._backup_state = "failed"
            return
        self._backup_state = "sending"
        self._backup_started_at = now
        if not self._backup_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().error("[V2 backoff] behavior_server 의 backup 액션이 없다 → 후진 생략")
            self._backup_state = "failed"; self._man_ctx = None
            return
        d = float(self.yield_backoff_m if dist is None else dist)
        # [V2.23] 후방이 막혀 있으면 **보내지 않는다**. 보내 봐야 nav2 가 첫 검사에서 COLLISION_AHEAD 로 거부하고,
        # stop-and-go 가 30 s 를 헛되이 쓴다 (실측: 20회 시도 전부 이동 0 cm, /cmd_vel 발행 0건).
        rb = self._rear_blocked(d)
        if rb is True:
            self.get_logger().error(
                f"[V2 backoff] 후방 {self._rear_block_at} m 에 lethal 셀 — 후진 {d:.2f} m 를 보내지 않는다 "
                f"(보내도 거부된다). 다른 수단/다음 순번에 맡긴다")
            self._publish_state(f"[V2 backoff] rear blocked at {self._rear_block_at} m → skip")
            self._backup_state = "failed"; self._man_ctx = None
            return
        self._man_begin("backup", d, now)
        g = BackUp.Goal()
        g.target.x = d
        g.speed = float(abs(self.yield_backoff_speed))
        g.time_allowance = Duration(seconds=self.backup_time_allowance).to_msg()
        if hasattr(g, "disable_collision_checks"):
            g.disable_collision_checks = False          # 후방 충돌 검사는 항상 켠다 (안전 최우선)
        fut = self._backup_client.send_goal_async(g, feedback_callback=self._man_feedback)
        fut.add_done_callback(self._backup_goal_response)

    def _drive_send(self, dist: float) -> None:
        if not self._op_gate(sys._getframe(1).f_code.co_name):     # [OP]
            self._maneuver = None; self._backup_state = "failed"; self._man_ctx = None
            return
        # [V2.23a] 전진 기동도 막혀 있으면 보내지 않는다 (보내도 COLLISION_AHEAD 로 거부된다).
        fb = self._rear_blocked(float(dist), sign=1.0)
        if fb is True:
            self.get_logger().error(
                f"[V2 retreat] 전방 {self._rear_block_at} m 에 lethal 셀 — 전진 {dist:.2f} m 를 보내지 않는다")
            self._publish_state(f"[V2 retreat] front blocked at {self._rear_block_at} m → skip")
            self._maneuver = None; self._backup_state = "failed"; self._man_ctx = None
            return
        if not self._drive_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().error("[V2 retreat] drive_on_heading 액션 없음"); self._maneuver = None; self._backup_state = "failed"; self._man_ctx = None; return
        self._backup_state = "sending"
        self._man_begin("drive", float(dist), self.get_clock().now())
        g = DriveOnHeading.Goal(); g.target.x = float(dist); g.speed = float(self.retreat_speed)
        g.time_allowance = Duration(seconds=self.backup_time_allowance).to_msg()
        if hasattr(g, "disable_collision_checks"): g.disable_collision_checks = False
        fut = self._drive_client.send_goal_async(g, feedback_callback=self._man_feedback)
        fut.add_done_callback(self._maneuver_goal_response)

    def _backup_goal_response(self, fut) -> None:
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("[V2 backoff] BackUp 목표 거부됨")
            self._backup_state = "failed"
            return
        self._backup_goal_handle = gh
        self._backup_state = "running"
        gh.get_result_async().add_done_callback(self._backup_result)

    def _backup_result(self, fut) -> None:
        try:
            res = fut.result()
            status = res.status
        except Exception as exc:                                 # noqa: BLE001
            self.get_logger().error(f"[V2 backoff] BackUp 결과 오류: {exc}")
            self._backup_state = "failed"
            return
        # GoalStatus: 4 SUCCEEDED, 5 CANCELED, 6 ABORTED
        self._backup_goal_handle = None
        if status == 4:
            self.get_logger().warn("[V2 backoff] BackUp SUCCEEDED")
            self._backup_state = "succeeded"; self._man_ctx = None
            return
        err = getattr(res.result, "error_code", None)
        if self._man_aborted(self.get_clock().now(), f"status {status} err {err}"):
            return                                           # 정지 대기 → _v2_maneuver_tick 이 남은 거리를 재개
        c = self._man_ctx
        if c is not None and c["target"] - c["done"] <= 0.1:
            self.get_logger().warn("[V2 backoff] BackUp 남은 거리 0.1 m 미만 → 완료로 본다")
            self._backup_state = "succeeded"; self._man_ctx = None
            return
        self.get_logger().error(f"[V2 backoff] BackUp 실패 status {status} error {err} (후방 막힘 지속/시간 초과)")
        self._backup_state = "failed"; self._man_ctx = None

    def _backup_cancel(self) -> None:
        gh = self._backup_goal_handle
        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception:                                    # noqa: BLE001
                pass
        self._backup_goal_handle = None

    # ------------------------------------------------------------------
    # [V2.1] 대기 순환(wait-for cycle) 검출과 해소
    # ------------------------------------------------------------------
    def _agent_yaw(self, a: MultiAgentInfo) -> float:
        return self._get_yaw(a.current_pose.pose)

    def _front_nearest(self, x: float, y: float, yaw: float, exclude: int, now: Time,
                       max_m: float, deg: float) -> Optional[int]:
        """(x,y,yaw) 에서 전방 부채꼴(±deg, max_m) 안 가장 가까운 로봇 id (나 포함, exclude 제외)."""
        best = None; bd = 1e9
        cands = [(int(m), a.current_pose.pose.position.x, a.current_pose.pose.position.y)
                 for m, a in self._cached_agents.items() if int(m) != exclude and int(m) != int(self.my_id)]
        me = self._own_pose()
        if me is not None and exclude != int(self.my_id):
            cands.append((int(self.my_id), me[0], me[1]))
        for (m, px, py) in cands:
            d = math.hypot(px - x, py - y)
            if d > max_m or d <= 1e-6:
                continue
            ang = self._ang_wrap(math.atan2(py - y, px - x) - yaw)
            if abs(ang) <= math.radians(deg) and d < bd:
                best, bd = m, d
        return best

    def _cycle_my_wait_target(self, now: Time) -> Optional[int]:
        """내가 지금 누구를 기다리는가. agent 대기면 잠긴 상대, goal 점유(정적) 대기면 내 전방 반경 안 정지 로봇."""
        if self.is_processing_agent_pause and self._locked_target_id:
            return int(self._locked_target_id)
        if self.is_processing_last_goal_occupied_pause or self.is_processing_goal_occupied_pause:
            me = self._own_pose()
            if me is None:
                return None
            best = None; bd = 1e9
            for m, a in self._cached_agents.items():
                if int(m) == int(self.my_id) or self._agent_is_moving(a, now):
                    continue
                p = a.current_pose.pose.position
                d = math.hypot(p.x - me[0], p.y - me[1])
                if d > self.cycle_goal_search_m:
                    continue
                if abs(self._ang_wrap(math.atan2(p.y - me[1], p.x - me[0]) - me[2])) > math.radians(90.0):
                    continue
                if d < bd:
                    best, bd = int(m), d
            return best
        return None

    def _cycle_blocker_of(self, mid: int, now: Time) -> Optional[int]:
        """로봇 mid 가 기다리는 상대. 관제 cross_agent_id 가 있으면 그것, 없으면 기하(정지 + 전방 가장 가까운 로봇)."""
        if int(mid) == int(self.my_id):
            return self._cycle_my_wait_target(now)
        a = self._cached_agents.get(int(mid))
        if a is None:
            return None
        cls = self._agent_stop_class(a, now)
        if cls in ("moving", "immobile", "working"):
            return None                       # 주행 중/고장/작업 중은 누구를 기다리는 게 아니다
        rep = int(a.cross_agent_id)
        if rep > 0 and rep != int(mid):
            return rep
        p = a.current_pose.pose.position
        return self._front_nearest(p.x, p.y, self._agent_yaw(a), int(mid), now,
                                   self.cycle_front_m, self.cycle_front_deg)

    def _detect_cycle(self, now: Time) -> Optional[List[int]]:
        """나에서 출발하는 대기 사슬이 나로 돌아오면 그 사슬(나 포함, 순서대로) 을 돌려준다."""
        chain = [int(self.my_id)]
        cur = int(self.my_id)
        for _ in range(max(2, self.cycle_max_len)):
            nxt = self._cycle_blocker_of(cur, now)
            if nxt is None:
                return None
            if nxt == int(self.my_id):
                return chain if len(chain) >= 2 else None
            if nxt in chain:
                return None                   # 나를 빼고 도는 고리 — 내 일이 아니다
            chain.append(nxt)
            cur = nxt
        return None

    def _cycle_rank(self, chain: List[int]) -> int:
        """순환 구성원 중 내 후진 순번 (0 = 제일 먼저). 우선순위가 낮은(id 큰) 쪽부터."""
        order = sorted(chain, reverse=True)
        return order.index(int(self.my_id))

    def _v2_cycle_tick(self) -> None:
        if self._op_frozen():
            return                                    # [OP] 관제 pause 중 — 판단 정지
        if not (self.v2_enable and self.cycle_detect_enable):
            return
        now = self.get_clock().now()
        # 내가 보낸 순환 해소 BackUp 의 결과 처리 (정적 대기 쪽; agent 대기 쪽은 _v2_phase1_tick 이 succeeded 를 처리)
        if self._cycle_backoff_active and self._backup_state in ("succeeded", "failed"):
            ok = (self._backup_state == "succeeded")
            self._cycle_backoff_active = False
            if self._cycle_backoff_static:
                self._backup_state = "idle"
                if ok:
                    self.get_logger().warn("[V2 cycle] BackUp 완료 (goal 점유 대기) → 대기 해제·재개, 상대가 지나가면 다시 간다")
                    self.pub_cmd_resume.publish(Bool(data=False))
                    self._publish_state("[V2 cycle] RUN (after backoff)")
                    self.is_processing_last_goal_occupied_pause = False
                    self.is_processing_goal_occupied_pause = False
                    self.static_is_last_goal_occupied_ = False
                    self.static_is_goal_occupied_ = False
                    self._last_goal_occupied_false_start_time = None
                    self._goal_occupied_false_start_time = None
                    self._pause_start_time = None
                else:
                    self.get_logger().error("[V2 cycle] BackUp 실패 (후방 막힘) → 대기 계속, 다음 순번 로봇이 시도한다")
                    self._publish_state("[V2 cycle] BACKOFF failed → keep waiting")
            if not ok:
                self._cycle_failed_sig = self._cycle_sig
            return
        if self._backup_state != "idle" or self.nav_stop_complete_ is False:
            return
        waiting_agent = self.is_processing_agent_pause and self._v2_mode == "pause"
        waiting_static = self.is_processing_last_goal_occupied_pause or self.is_processing_goal_occupied_pause
        if not (waiting_agent or waiting_static):
            self._cycle_sig = None; self._cycle_since = None
            self._cycle_failed_sig = None      # [V2.16] 순환이 풀렸으면 실패 기억도 지운다 (안 지우면 영원히 남는다)
            return
        chain = self._detect_cycle(now)
        if chain is None:
            if self._cycle_sig is not None:
                self.get_logger().info(f"[V2 cycle] 순환 해소됨 {self._cycle_sig}")
            self._cycle_sig = None; self._cycle_since = None
            self._cycle_failed_sig = None      # [V2.16] 순환이 풀렸으면 실패 기억도 지운다 (안 지우면 영원히 남는다)
            return
        sig = tuple(sorted(chain))
        if sig != self._cycle_sig:
            self._cycle_sig = sig; self._cycle_since = now
            self.get_logger().warn(
                f"[V2 cycle] 대기 순환 검출 {'→'.join(str(m) for m in chain + [chain[0]])} "
                f"(내 후진 순번 {self._cycle_rank(chain)}, {'agent' if waiting_agent else 'goal점유'} 대기)")
            self._publish_state(f"[V2 cycle] detected {'-'.join(str(m) for m in chain)} rank {self._cycle_rank(chain)}")
            return
        dt = (now - self._cycle_since).nanoseconds * 1e-9
        rank = self._cycle_rank(chain)
        due = self.cycle_min_wait_sec + rank * self.cycle_stagger_sec
        last = self._cycle_done.get(sig)
        if last is not None and (now - last).nanoseconds * 1e-9 < self.cycle_retry_sec:
            return
        if dt < due:
            return
        if not self._op_may('_v2_cycle_tick'):          # [OP] 불허면 순환 해소 기록·공지 없이 다음 틱에 다시 본다
            return
        self._cycle_done[sig] = now
        self._cycle_backoff_active = True
        self._cycle_backoff_static = bool(waiting_static and not waiting_agent)
        if self._cycle_failed_sig == sig:
            # 2단계: 직선 후진이 막혔던 순환 → 내 이력을 따라 물러난다 (돌아서 갈 수 있다)
            self.get_logger().warn(
                f"[V2 cycle] 순환 {sig} 이 {dt:.0f}s 지속, 내 순번 {rank}, 직선 후진은 실패 → 이력 따라 {self.push_retreat_m} m 후퇴")
            self._publish_state(f"[V2 cycle] RETREAT {self.push_retreat_m} m (rank {rank})")
            tgt_a = self._cached_agents.get(self._cycle_my_wait_target(now) or -1)
            away = (tgt_a.current_pose.pose.position.x, tgt_a.current_pose.pose.position.y) if tgt_a is not None else None
            if not self._retreat_along_history(now, self.push_retreat_m, away):
                self._cycle_backoff_active = False
            return
        self.get_logger().warn(
            f"[V2 cycle] 순환 {sig} 이 {dt:.0f}s 지속, 내 순번 {rank} → BackUp {self.cycle_backoff_m} m 로 고리를 끊는다")
        self._publish_state(f"[V2 cycle] BACKOFF {self.cycle_backoff_m} m (rank {rank})")
        saved = self.yield_backoff_m
        self.yield_backoff_m = self.cycle_backoff_m
        try:
            self._backup_send(now)
        finally:
            self.yield_backoff_m = saved

    # ------------------------------------------------------------------
    # [V2.1] 내 자세 이력, 이력 따라 후퇴 (Spin + DriveOnHeading / BackUp), 밀어내기 양보
    # ------------------------------------------------------------------
    def _v2_own_hist_tick(self) -> None:
        me = self._own_pose()
        if me is None:
            return
        # [09-28 현장 이상 2] 정적 해제 지점에서 벗어난 적이 있는지 (한 번 벗어나면 다시 돌아와도 '통과한 것')
        if (self._static_release_xy is not None and not self._static_left_spot
                and self.static_same_spot_leave > 0.0
                and math.hypot(me[0] - self._static_release_xy[0],
                               me[1] - self._static_release_xy[1]) >= self.static_same_spot_leave):
            self._static_left_spot = True
            self.get_logger().info(
                f"[check_collision_obstacle] 정적 해제 지점에서 {self.static_same_spot_leave:.1f} m 이상 벗어났다 "
                f"— 다음 재차단은 새 상황으로 본다 (직전 replan 재시도 {self._static_replan_retry}회).")
        t = self.get_clock().now().nanoseconds * 1e-9
        if self._own_hist and math.hypot(me[0] - self._own_hist[-1][1], me[1] - self._own_hist[-1][2]) < 0.05:
            self._own_hist[-1] = (t, self._own_hist[-1][1], self._own_hist[-1][2])   # 제자리면 시각만 갱신 → 무진전 측정
            if len(self._own_hist) >= 2 and math.hypot(self._own_hist[-2][1] - me[0], self._own_hist[-2][2] - me[1]) < 0.05:
                return
        self._own_hist.append((t, me[0], me[1]))

    def _agent_stopped_sec(self, mid: int, now: Time, dist_m: float = 0.3) -> float:
        """상대 mid 가 dist_m 안에 머문 시간 [s] (자세 이력 3 s 창 + pause_since 로 보강)."""
        h = self._agent_hist.get(int(mid))
        if not h:
            return 0.0
        t_now = now.nanoseconds * 1e-9
        x1, y1 = h[-1][1], h[-1][2]
        t_leave = h[-1][0]
        for (t, x, y) in reversed(h):
            if math.hypot(x - x1, y - y1) > dist_m:
                break
            t_leave = t
        since = self._agent_pause_since.get(int(mid))
        if since is not None:
            t_leave = min(t_leave, since.nanoseconds * 1e-9)
        anc = self._agent_stop_since.get(int(mid))
        if anc is not None and math.hypot(anc[1] - x1, anc[2] - y1) <= dist_m:
            t_leave = min(t_leave, anc[0])
        return max(0.0, t_now - t_leave)

    def _stuck_carry_or(self, now: Time) -> Time:
        """[D11] 새 활성 시작 시각. 직전 종료가 CANCELED/FAILED·같은 자리·창 안이면 이전 활성 시각을 이어 쓴다."""
        e, self._last_end = self._last_end, None
        if not e or e['status'] not in ('CANCELED', 'FAILED') or e['active_since'] is None or e['xy'] is None:
            return now
        gap = (now - e['t']).nanoseconds * 1e-9
        me = self._own_pose()
        if me is None or gap > self.stuck_carry_window:
            return now
        d = math.hypot(me[0] - e['xy'][0], me[1] - e['xy'][1])
        if d > self.stuck_carry_same_spot:
            return now
        self.get_logger().info(
            f"[OP] 같은 자리 재명령 (직전 {e['status']}, {d:.2f} m, {gap:.0f} s 뒤) — 무진전 시계를 이어서 센다 "
            f"({(now - e['active_since']).nanoseconds * 1e-9:.0f} s)", throttle_duration_sec=10.0)
        return e['active_since']

    def _stuck_sec(self, now: Time, dist_m: float = 0.3) -> float:
        """dist_m 안에 머문 시간 [s] (내 이력 기준)."""
        me = self._own_pose()
        if me is None or not self._own_hist:
            return 0.0
        t_now = now.nanoseconds * 1e-9
        if self.operator_priority:
            # [OP] 이번 관제 명령의 활성 시간만 센다: 종료 상태면 0, 활성이면 활성 시각 이후만
            if self.current_robot_status not in self._OP_ACTIVE or self._active_since is None:
                return 0.0
            t_floor = self._active_since.nanoseconds * 1e-9
        else:
            t_floor = None
        t_leave = t_now
        for (t, x, y) in reversed(self._own_hist):
            if math.hypot(x - me[0], y - me[1]) > dist_m:
                break
            t_leave = t
        if t_floor is not None:
            t_leave = max(t_leave, t_floor)
        return max(0.0, t_now - t_leave)

    def _retreat_target(self, dist: float) -> Optional[Tuple[float, float]]:
        """내 이력을 거슬러 경로 거리 dist 만큼 뒤의 점 (내가 지나온 곳 = 벽 없음)."""
        me = self._own_pose()
        if me is None or len(self._own_hist) < 2:
            return None
        acc = 0.0
        px, py = me[0], me[1]
        for (t, x, y) in reversed(self._own_hist):
            d = math.hypot(x - px, y - py)
            if acc + d >= dist:
                r = (dist - acc) / d if d > 1e-6 else 0.0
                return (px + (x - px) * r, py + (y - py) * r)
            acc += d; px, py = x, y
        return (px, py) if acc >= 0.5 else None

    def _plan_retreat(self, dist: float, away_from: Optional[Tuple[float, float]] = None) -> Optional[dict]:
        """후퇴 계획. 후보 = 내 이력 뒤 점(온 길 = 벽 없음) / 바로 뒤(BackUp) / 바로 앞(DriveOnHeading).
        away_from(보고한 로봇 위치) 가 있으면 **그 로봇에서 min(0.5, retreat_away_ratio × dist) 이상 멀어지는** 후보만,
        가장 멀어지는 순으로 고른다 ([10-05 D20 J1] 예전 고정 0.5 m 는 dist < 0.5 이면 만족 불가였다)
        (큐15 ① 사례: 온 길이 고리라 이력 후퇴가 오히려 우선 로봇 쪽으로 갔다). 후보 실행 순서는 이력 → 뒤 → 앞.
        목표점 방향이 바로 뒤(±25°) 면 BackUp, 바로 앞(±25°) 이면 DriveOnHeading, 아니면 Spin 뒤 DriveOnHeading."""
        me = self._own_pose()
        if me is None:
            self._plan_retreat_why = "자세를 모름"
            return None
        cands = []
        hist = self._retreat_target(dist)
        if hist is not None and math.hypot(hist[0] - me[0], hist[1] - me[1]) >= 0.3:
            cands.append(("hist", hist))
        cands.append(("back", (me[0] - dist * math.cos(me[2]), me[1] - dist * math.sin(me[2]))))
        cands.append(("ahead", (me[0] + dist * math.cos(me[2]), me[1] + dist * math.sin(me[2]))))
        if away_from is not None:
            d0 = math.hypot(away_from[0] - me[0], away_from[1] - me[1])
            min_gain = max(1e-3, min(0.5, self.retreat_away_ratio * dist))
            scored = []
            best = -1e9
            for name, tgt in cands:
                gain = math.hypot(away_from[0] - tgt[0], away_from[1] - tgt[1]) - d0
                best = max(best, gain)
                if gain >= min_gain:
                    scored.append((gain, name, tgt))
            if not scored:
                self._plan_retreat_why = (f"상대에서 {min_gain:.2f} m 이상 멀어지는 후보 없음 "
                                          f"(후퇴 {dist:.2f} m, 최대 {best:.2f} m)")
                return None
            scored.sort(key=lambda z: (-(z[1] == "hist"), -z[0]))     # 이력 후보를 우선, 그 다음 멀어지는 정도
            cands = [(n, t) for _, n, t in scored]
        name, tgt = cands[0]
        d = math.hypot(tgt[0] - me[0], tgt[1] - me[1])
        rel = self._ang_wrap(math.atan2(tgt[1] - me[1], tgt[0] - me[0]) - me[2])
        if abs(rel) <= math.radians(25.0):
            return {"steps": [("drive", d)], "target": tgt, "cand": name}
        if abs(self._ang_wrap(rel + math.pi)) <= math.radians(25.0):
            return {"steps": [("backup", d)], "target": tgt, "cand": name}
        return {"steps": [("spin", rel), ("drive", d)], "target": tgt, "cand": name}

    def _retreat_along_history(self, now: Time, dist: float, away_from: Optional[Tuple[float, float]] = None) -> bool:
        if not self._op_gate(sys._getframe(1).f_code.co_name):     # [OP]
            return False
        if not self._retreat_gate_check(sys._getframe(1).f_code.co_name):
            return False                                  # [V2.34] 호출한 규칙이 '후퇴 못 걸었음' 경로로 정리한다
        if self._yield_bt_ready(now):
            return self._yield_start(now, away_from)
        if self.yield_bt_enable:
            self.get_logger().warn("[V2 yield] 직전 BT 후퇴가 실패한 자리 → 이번엔 직선 기동으로 물러난다")
        if self.retreat_need_enable:                      # [V2.33] 직선 기동도 필요한 만큼만 (바로 뒤 방향 기준)
            me = self._own_pose()
            if me is not None:
                dist = min(dist, self._retreat_need(me, -math.cos(me[2]), -math.sin(me[2]))[0])
        self._plan_retreat_why = ""
        plan = self._plan_retreat(dist, away_from)
        if plan is None:
            # [10-05 D20 J1] 예전 문구 "이력이 짧거나 자세를 몰라" 는 실제 원인(멀어짐 조건)을 가렸다
            self.get_logger().error(f"[V2 retreat] 후퇴 못 함: {self._plan_retreat_why or '후보 없음'}")
            return False
        self._maneuver = {"steps": list(plan["steps"]), "target": plan["target"], "started": now}
        self._backup_state = "sending"; self._backup_started_at = now
        self.get_logger().warn(
            f"[V2 retreat] {plan.get('cand', 'hist')} 후보 ({plan['target'][0]:.2f},{plan['target'][1]:.2f}) 로 후퇴: "
            + " → ".join(f"{k} {v:.2f}" for k, v in plan["steps"]))
        self._maneuver_next()
        return True

    def _maneuver_next(self) -> None:
        if not self._maneuver or not self._maneuver["steps"]:
            self._maneuver = None
            self._backup_state = "succeeded"
            self.get_logger().warn("[V2 retreat] 후퇴 완료")
            return
        kind, val = self._maneuver["steps"].pop(0)
        if kind == "backup":
            saved = self.yield_backoff_m; self.yield_backoff_m = float(val)
            try:
                self._backup_send(self.get_clock().now())     # 실패/성공은 _backup_result 가 _backup_state 로 알린다
            finally:
                self.yield_backoff_m = saved
            self._maneuver["steps"] = []                        # backup 은 단독 단계
            self._maneuver = None
            return
        if kind == "spin":
            if not self._op_gate('_maneuver_next'):                 # [OP]
                self._maneuver = None; self._backup_state = "failed"; return
            if not self._spin_client.wait_for_server(timeout_sec=0.5):
                self.get_logger().error("[V2 retreat] spin 액션 없음"); self._maneuver = None; self._backup_state = "failed"; return
            g = Spin.Goal(); g.target_yaw = float(val)
            g.time_allowance = Duration(seconds=self.backup_time_allowance).to_msg()
            if hasattr(g, "disable_collision_checks"): g.disable_collision_checks = False
            fut = self._spin_client.send_goal_async(g)
        else:
            self._drive_send(float(val))
            return
        fut.add_done_callback(self._maneuver_goal_response)

    def _maneuver_goal_response(self, fut) -> None:
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("[V2 retreat] 단계 목표 거부됨"); self._maneuver = None; self._backup_state = "failed"; return
        self._maneuver_goal_handle = gh               # [OP] 허가 상실 때 취소할 수 있게
        self._backup_state = "running"
        gh.get_result_async().add_done_callback(self._maneuver_result)

    def _maneuver_result(self, fut) -> None:
        self._maneuver_goal_handle = None
        try:
            status = fut.result().status
        except Exception as exc:                                 # noqa: BLE001
            self.get_logger().error(f"[V2 retreat] 단계 결과 오류: {exc}"); self._maneuver = None; self._backup_state = "failed"; return
        if status != 4:
            if self._man_aborted(self.get_clock().now(), f"status {status}"):
                return                                       # drive 단계 stop-and-go 대기 (spin 은 재개 없음)
            c = self._man_ctx
            if c is not None and c["kind"] == "drive" and c["target"] - c["done"] <= 0.1:
                self._man_ctx = None; self._maneuver_next(); return
            self.get_logger().error(f"[V2 retreat] 단계 실패 status {status} (막힘 지속/시간 초과)")
            self._maneuver = None; self._backup_state = "failed"; self._man_ctx = None; return
        self._man_ctx = None
        self._maneuver_next()

    def _i_am_nearest_to(self, x: float, y: float, exclude: int) -> bool:
        """[V2.11] 그 자리에 내가 제일 가까운가. 여러 대가 동시에 비키는 걸 막는 잠금 장치."""
        me = self._own_pose()
        if me is None:
            return False
        mine = math.hypot(me[0] - x, me[1] - y)
        for mid, a in self._cached_agents.items():
            if int(mid) in (int(self.my_id), int(exclude)):
                continue
            p = a.current_pose.pose.position
            if math.hypot(p.x - x, p.y - y) < mine:
                return False
        return True

    def _near_own_goal(self, radius: Optional[float] = None) -> bool:
        """[V2.10] 내 다음 goal 이 코앞인가. 이때 양보 기동을 걸면 완주 보고가 막혀 자리를 영영 안 비운다.
        radius 를 주면 그 반경으로 본다 ([V2.12] 완주 우선 재개는 더 넓게 본다)."""
        r = self.goal_guard_m if radius is None else radius
        if r <= 0.0 or not self._prev_goals:
            return False
        me = self._own_pose()
        if me is None:
            return False
        g = self._prev_goals[0]
        return math.hypot(g[0] - me[0], g[1] - me[1]) <= r

    def _v2_push_tick(self) -> None:
        """[V2.1] 밀어내기 양보: 우선 로봇이 나 때문에 막혔는데 나는 무진전 → 조정 대기 여부와 무관하게 이력 따라 물러난다."""
        if self._op_frozen():
            return                                    # [OP] 관제 pause 중 — 판단 정지
        if not (self.v2_enable and self.push_yield_enable):
            return
        now = self.get_clock().now()
        if self._push_paused and not self._yield_active and self._backup_state in ("succeeded", "failed", "idle"):
            ok = self._backup_state != "failed"
            self._push_paused = False
            if self._backup_state != "idle":
                self._backup_state = "idle"
            self.pub_cmd_resume.publish(Bool(data=False))
            self._publish_state(f"[V2 push] {'RUN (after retreat)' if ok else 'retreat failed → RUN'}")
            self.get_logger().warn(f"[V2 push] 후퇴 {'완료' if ok else '실패'} → 재개")
            return
        if self._backup_state != "idle" or self.nav_stop_complete_ is False:
            return
        goal_wait = self.is_processing_goal_occupied_pause or self.is_processing_last_goal_occupied_pause
        if self.is_processing_agent_pause or self.is_processing_replan_pause:
            return                                    # 조정 대기 중이면 그쪽 규칙(mutual block / cycle) 이 맡는다
        if goal_wait:
            # [V2.24] goal 점유 대기는 **순환이 잡혔을 때만** cycle 규칙에 맡긴다. 순환이 없으면 맡을 규칙이
            # 아무것도 없어서 그대로 얼어붙는다 — 그때는 후방이 비어 있는 로봇이 스스로 비켜 준다.
            if not self.push_during_goal_wait or self._cycle_sig is not None:
                return
            if self._stuck_sec(now) < self.push_during_goal_wait_sec:
                return
            if self._rear_blocked(self.push_retreat_m) is True:
                self.get_logger().info(
                    f"[V2 push] goal 점유 대기지만 후방 {self._rear_block_at} m 가 막혔다 — 내가 비킬 수 없다",
                    throttle_duration_sec=20.0)
                return
        # [V2.8] READY/IDLE 도 포함한다. 남의 goal 위에 **주행 없이 서 있는** 로봇이야말로 비켜 줘야 한다
        # (q1_base_fix: READY 로 선 r3 가 r1 의 goal 을 덮어 30분 연쇄 대기를 만들었다).
        if self.current_robot_status not in ('READY', 'IDLE', 'RECEIVED_GOAL', 'PLANNING', 'DRIVING', 'PAUSED',
                                             'RECOVERY_FAILURE', 'RECOVERY_RUNNING', 'RECOVERY_SUCCESS'):
            return
        stuck = self._stuck_sec(now)
        if stuck < self.push_min_sec:
            return
        turn = self._line_my_turn(now)
        if turn is False:
            self.get_logger().info("[V2 line] 줄 해소 순서가 아니다 — 이번엔 기동하지 않는다",
                                   throttle_duration_sec=20.0)
            return
        if self._near_own_goal():          # [V2.10] 완주가 코앞이면 비키지 말고 **먼저 끝낸다**
            self.get_logger().info("[V2 push] 내 goal 이 코앞이라 양보를 미룬다 — 완주하면 자리가 빈다",
                                   throttle_duration_sec=20.0)
            return
        me = self._own_pose()
        if me is None:
            return
        for m, a in self._cached_agents.items():
            mid = int(m)
            if mid == int(self.my_id) or self._i_have_priority(a, now) or self._agent_is_moving(a, now):
                continue
            last = self._push_done.get(mid)
            if last is not None and (now - last).nanoseconds * 1e-9 < self.push_cooldown_sec:
                continue
            # [03:30 수정] 관제 보고도 상대가 push_min_sec/2 이상 서 있을 때만 (막 선 로봇의 보고로 1~3 s 짜리 후퇴가 연발, 큐15 ③)
            reported = (int(a.cross_agent_id) == int(self.my_id)
                        and self._agent_stopped_sec(mid, now) >= self.push_min_sec / 2.0)
            geo = False
            # [02:40 수정] 기하 판정은 상대가 **진짜 대기 phase(PAUSE/WAIT_OBS)** 로 push_min_sec/2 이상 서 있을 때만.
            # 기동 직후 PATH_SEARCH(1)/MOVING 인데 속도 0 인 로봇을 "나를 기다린다" 로 오판해 스폰에서 후퇴하려 한 사례(큐15 ①).
            if not reported and int(a.status.phase) in (AgentStatus.STATUS_PAUSE, 2) \
                    and self._agent_stopped_sec(mid, now) >= self.push_min_sec / 2.0:
                p = a.current_pose.pose.position
                d = math.hypot(p.x - me[0], p.y - me[1])
                geo = (d <= self.cycle_front_m and abs(self._ang_wrap(
                    math.atan2(me[1] - p.y, me[0] - p.x) - self._agent_yaw(a))) <= math.radians(self.cycle_front_deg))
            blocked = False
            if not (reported or geo) and self.blocked_yield_enable \
                    and int(a.status.phase) in (AgentStatus.STATUS_PAUSE, 2) \
                    and self._agent_stopped_sec(mid, now) >= self.blocked_yield_sec:
                # [V2.11] 막힌 채 서 있는 이웃 + 내가 그 옆에서 제일 가깝다 → 관제 지목 없이도 내가 비킨다
                p = a.current_pose.pose.position
                if math.hypot(p.x - me[0], p.y - me[1]) <= self.blocked_yield_m \
                        and self._i_am_nearest_to(p.x, p.y, mid):
                    blocked = True
            if not (reported or geo or blocked):
                continue
            rp = a.current_pose.pose.position
            if self._plan_retreat(self.push_retreat_m, (rp.x, rp.y)) is None:
                self.get_logger().info(f"[V2 push] 로봇 {mid} 보고/대면이지만 그 로봇에서 멀어지는 후퇴 후보가 없다 → 보류", throttle_duration_sec=10.0)
                continue
            if not self._op_may('_v2_push_tick'):        # [OP] 불허면 pause·공지·쿨다운 없이 넘긴다
                return
            self._push_done[mid] = now
            src = ("관제 cross_agent_id" if reported else
                   "기하(정지한 채 나를 마주봄)" if geo else
                   "막힌 이웃 옆 최근접(V2.11, 관제 무관)")
            self.get_logger().warn(
                f"[V2 push] 우선 로봇 {mid} 이 나 때문에 막힘({src}), 나는 {stuck:.0f}s 무진전(status {self.current_robot_status}) "
                f"→ 내 이력 따라 {self.push_retreat_m} m 물러난다")
            self._publish_state(f"[V2 push] RETREAT {self.push_retreat_m} m for agent {mid}")
            if not self.yield_bt_enable:
                self._publish_pause()               # BT 후퇴 모드에서는 pause 하면 BT 가 주행을 못 한다
                self._push_paused = True
            else:
                self._push_paused = True
            if not self._retreat_along_history(now, self.push_retreat_m, (rp.x, rp.y)):
                self._backup_state = "failed"
            return

    # ------------------------------------------------------------------
    # [V2.1] BT 양보 후퇴 (사용자 09-20: 직선 BackUp 대신 recovery 방식의 곡선 후진)
    # ------------------------------------------------------------------
    def _on_remaining_goals(self, msg: Path) -> None:
        """BT 가 남은 goal 목록을 퍼블리시한다(/remaining_goals). 목록 앞에서 사라진 goal = 방금 지나온 goal."""
        cur = [(p.pose.position.x, p.pose.position.y, self._get_yaw(p.pose)) for p in msg.poses]
        if self._prev_goals and cur != self._prev_goals:
            still = {(round(x, 2), round(y, 2)) for (x, y, _) in cur}
            for g in self._prev_goals:
                if (round(g[0], 2), round(g[1], 2)) in still:
                    break                      # 목록 앞에서부터 사라진 것만 '지나온' 것으로 본다
                if not self._passed_goals or math.hypot(g[0] - self._passed_goals[-1][0],
                                                        g[1] - self._passed_goals[-1][1]) > 0.2:
                    self._passed_goals.append(g)
                    self.get_logger().info(f"[V2 yield] 지나온 goal 기록 ({g[0]:.2f},{g[1]:.2f}) — 총 {len(self._passed_goals)}개")
        self._prev_goals = cur

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map_msg = msg
        # [09-20 CPU] 매 호출마다 원본 data 를 훑으면 비싸다(실측 노드 CPU 21 %). 자유공간 여부를 1바이트 배열로 미리 만든다.
        self._map_free_mask = bytearray(1 if (0 <= v < 50) else 0 for v in msg.data)
        self._junc_cache = None
        self.get_logger().info(
            f"[V2 yield] 지도 수신 {msg.info.width}x{msg.info.height} @ {msg.info.resolution:.3f} m — 후퇴 목표 검사에 쓴다")

    def _map_free(self, x: float, y: float, clearance: Optional[float] = None) -> bool:
        """지도에서 (x,y) 둘레 clearance 가 자유 공간인가. 지도가 없으면 판단을 보류하고 True."""
        m = self._map_msg
        if m is None:
            return True
        mask = getattr(self, "_map_free_mask", None)
        res = m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        c = self.yield_clearance_m if clearance is None else clearance
        n = max(1, int(c / res))
        cx, cy = int((x - ox) / res), int((y - oy) / res)
        for dx in range(-n, n + 1):
            for dy in range(-n, n + 1):
                if dx * dx + dy * dy > n * n:
                    continue
                gx, gy = cx + dx, cy + dy
                if gx < 0 or gy < 0 or gx >= m.info.width or gy >= m.info.height:
                    return False
                idx = gy * m.info.width + gx
                if mask is not None:
                    if not mask[idx]:
                        return False
                else:
                    v = m.data[idx]
                    if v < 0 or v >= 50:
                        return False
        return True

    def _path_free(self, x0: float, y0: float, x1: float, y1: float) -> bool:
        """직선 구간을 0.25 m 간격으로 훑어 자유 공간인지 본다 (후진 경로가 벽을 통과하는 목표를 거른다)."""
        d = math.hypot(x1 - x0, y1 - y0)
        steps = max(1, int(d / 0.25))
        for i in range(steps + 1):
            t = i / steps
            if not self._map_free(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t):
                return False
        return True

    def _in_junction(self, x: float, y: float) -> bool:
        """지도상 (x,y) 가 교차로인가 — 네 방향 중 junction_open_dirs 개 이상이 probe 거리만큼 열려 있으면.
        [09-20 CPU] 같은 자리(0.25 m) 결과는 2 s 동안 재사용한다 — 10~20 Hz 판단 경로에서 매번 계산하면 비싸다."""
        if self._map_msg is None:
            return False
        now_s = self.get_clock().now().nanoseconds * 1e-9
        c = getattr(self, "_junc_cache", None)
        if c is not None and now_s - c[0] < 2.0 and math.hypot(x - c[1], y - c[2]) < 0.25:
            return c[3]
        open_dirs = 0
        for (dx, dy) in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ok = True
            steps = max(1, int(self.junction_probe_m / 0.3))
            for i in range(1, steps + 1):
                t = self.junction_probe_m * i / steps
                if not self._map_free(x + dx * t, y + dy * t, 0.3):
                    ok = False
                    break
            if ok:
                open_dirs += 1
        res_b = open_dirs >= self.junction_open_dirs
        self._junc_cache = (now_s, x, y, res_b)
        return res_b

    def _ahead_clear(self, dist: float) -> bool:
        """내 경로 앞 dist 구간이 지도상 비어 있고, 그 위에 다른 로봇도 없는가."""
        me = self._own_pose()
        pts = self._my_path_points()
        if me is None or len(pts) < 2:
            return False
        acc = 0.0
        px, py = me[0], me[1]
        for (x, y) in pts:
            d = math.hypot(x - px, y - py)
            if d < 1e-6:
                continue
            acc += d
            px, py = x, y
            if not self._map_free(x, y, 0.3):
                return False
            for m, a in self._cached_agents.items():
                if int(m) == int(self.my_id):
                    continue
                p = a.current_pose.pose.position
                if math.hypot(p.x - x, p.y - y) < 0.8:
                    return False
            if acc >= dist:
                return True
        return acc >= dist * 0.6          # 경로가 짧으면 그만큼만 확인

    def _junction_should_clear(self, now: Time) -> bool:
        """지금 정지하려는 자리가 교차로면, 앞이 비어 있는 동안은 빠져나간 뒤 정지한다."""
        if not (self.v2_enable and self.junction_keep_clear):
            return False
        me = self._own_pose()
        if me is None or not self._in_junction(me[0], me[1]):
            self._junction_clear_since = None
            return False
        if self._junction_clear_since is None:
            self._junction_clear_since = now
        elif (now - self._junction_clear_since).nanoseconds * 1e-9 > self.junction_clear_max_sec:
            self.get_logger().warn("[V2 junction] 교차로 탈출 시간 초과 → 그냥 정지한다")
            self._junction_clear_since = None
            return False
        if not self._ahead_clear(self.junction_clear_ahead_m):
            self._junction_clear_since = None
            return False                   # 앞도 막혔으면 별 수 없다
        self.get_logger().warn(
            f"[V2 junction] 교차로 한가운데다 ({me[0]:.2f},{me[1]:.2f}) → 앞 {self.junction_clear_ahead_m:.1f} m 가 비었으니 빠져나간 뒤 정지",
            throttle_duration_sec=2.0)
        self._publish_state("[V2 junction] clearing intersection before stop")
        return True

    def _yield_target_ok(self, me: Tuple[float, float, float], x: float, y: float) -> bool:
        return self._map_free(x, y) and self._path_free(me[0], me[1], x, y)

    def _goal_occupied_by_agent(self, x: float, y: float) -> bool:
        for m, a in self._cached_agents.items():
            if int(m) == int(self.my_id):
                continue
            p = a.current_pose.pose.position
            if math.hypot(p.x - x, p.y - y) <= self.yield_goal_occupied_m:
                return True
        return False

    def _retreat_need(self, me: Tuple[float, float, float], ux: float, uy: float) -> Tuple[float, int]:
        """[V2.33] (ux, uy) 방향으로 얼마나 물러나야 이웃 앞길에서 내 몸체가 빠지는가. (필요 거리, 막힌 이웃 수).
        지금 내 몸체와 겹치는 앞길만 본다 — 원래 안 막고 있던 로봇 때문에 더 물러나지 않는다.
        [V2.33b] 이웃 앞길(truncated_path)은 10점뿐이라 멈춘 로봇은 0.5 m 안팎이다(YN1 실측: 후퇴 93회 중 90회가 '겹침 0').
        짧으면 마지막 방향(없으면 로봇 방향)으로 retreat_path_look_m 까지 직선으로 늘려 본다.
        물러나도 겹침이 줄지 않으면(뒤따라오는 로봇 쪽으로 가는 셈) 최소 거리만 간다."""
        clear = 2.0 * self.retreat_body_r_m + self.retreat_clear_margin_m
        paths, bodies = [], []
        for m, a in self._cached_agents.items():
            if int(m) == int(self.my_id):
                continue
            c = a.current_pose.pose.position
            pts = [(c.x, c.y)]
            acc = 0.0
            for q in a.truncated_path.poses:
                x, y = q.pose.position.x, q.pose.position.y
                step = math.hypot(x - pts[-1][0], y - pts[-1][1])
                if step < 1e-3:
                    continue
                if acc + step > self.retreat_path_look_m:
                    break
                acc += step
                pts.append((x, y))
            if acc < self.retreat_path_look_m:
                if len(pts) >= 2 and math.hypot(pts[-1][0] - pts[-2][0], pts[-1][1] - pts[-2][1]) > 0.02:
                    dx, dy = pts[-1][0] - pts[-2][0], pts[-1][1] - pts[-2][1]
                    h = math.atan2(dy, dx)
                else:
                    o = a.current_pose.pose.orientation
                    h = math.atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z))
                ex, ey = pts[-1]
                k = 1
                while acc + 0.1 * k <= self.retreat_path_look_m + 1e-6:
                    pts.append((ex + 0.1 * k * math.cos(h), ey + 0.1 * k * math.sin(h)))
                    k += 1
            if min(math.hypot(x - me[0], y - me[1]) for x, y in pts) < clear:
                paths.append(pts)
                bodies.append((c.x, c.y))
        if not paths:
            return self.retreat_min_m, 0

        def worst(s: float) -> float:
            px, py = me[0] + ux * s, me[1] + uy * s
            return min(min(math.hypot(x - px, y - py) for x, y in pts) for pts in paths)

        s = self.retreat_min_m
        while s <= self.retreat_max_m + 1e-6:
            if worst(s) >= clear:
                return s, len(paths)
            s += 0.1
        # 끝내 못 빠지면(같은 차선 정면 대치 등) 겹친 이웃 **몸체**에서 멀어지는지만 본다.
        # 멀어지면 상한까지 벌려 준다(1.8 m 통로는 교행 가능 → 간격이 있어야 상대가 비켜 간다). 가까워지면(뒤따라오는 로봇) 최소만.
        def gap(s: float) -> float:
            px, py = me[0] + ux * s, me[1] + uy * s
            return min(math.hypot(bx - px, by - py) for bx, by in bodies)
        if gap(self.retreat_max_m) > gap(0.0) + 0.3:
            return self.retreat_max_m, len(paths)     # stop-and-go: 상한만 가고 멈춘 뒤 다시 판단
        return self.retreat_min_m, len(paths)

    def _on_test_retreat(self, msg: String) -> None:
        """[09-27 시험용] 규칙과 같은 입구로 후진을 낸다. backup: _backup_send(dist 지정 → 관문은 건너뛰고 사전 검사·stop-and-go 는 그대로),
        yield: 현재 자세 뒤 dist m 를 목표로 _yield_publish(곡선 후진, BT 가 돌고 있어야 한다)."""
        import json
        try:
            req = json.loads(msg.data)
        except Exception:                                # noqa: BLE001
            self.get_logger().error(f"[V2 test] 해석 실패: {msg.data}")
            return
        now = self.get_clock().now()
        kind = req.get("kind")
        dist = float(req.get("dist", 0.5))
        if not self._op_gate('_on_test_retreat'):     # [OP] 규칙과 같게 양보 계열 새 기동으로 판정 (곡선 후진도 여기서 막는다)
            return
        if kind == "backup":
            if "speed" in req:
                self.yield_backoff_speed = float(req["speed"])
            self._man_ctx = None
            self._backup_state = "idle"
            self.get_logger().warn(f"[V2 test] 직선 후진 {dist:.2f} m @ {self.yield_backoff_speed:.2f} m/s")
            self._backup_send(now, dist=dist)
        elif kind == "yield":
            me = self._own_pose()
            if me is None:
                return
            tx, ty = me[0] - dist * math.cos(me[2]), me[1] - dist * math.sin(me[2])
            self._backup_state = "idle"
            self.get_logger().warn(f"[V2 test] 곡선 후진 목표 ({tx:.2f},{ty:.2f}) {dist:.2f} m")
            self._yield_cands = [(tx, ty, me[2], "시험")]
            self._yield_idx = 0
            self._yield_publish(now, (tx, ty, me[2], "시험"))

    def _retreat_gate(self) -> Tuple[bool, str]:
        """[V2.34] 지금 후진해도 되는가. (허용 여부, 사유)."""
        me = self._own_pose()
        if me is None:
            return True, "자세 모름"
        near = []
        for m, a in self._cached_agents.items():
            if int(m) == int(self.my_id):
                continue
            c = a.current_pose.pose.position
            if math.hypot(c.x - me[0], c.y - me[1]) < self.retreat_gate_free_m:
                near.append(int(m))
        coord_wait = (self.is_processing_agent_pause or self.is_processing_replan_pause
                      or self.is_processing_goal_occupied_pause or self.is_processing_last_goal_occupied_pause)
        if not near:
            if coord_wait:
                return False, "주변에 이웃은 없지만 조정 대기 중(끼임 아님)"
            return True, f"주변 {self.retreat_gate_free_m:.1f} m 에 이웃 없음(정적 끼임)"
        _, nb = self._retreat_need(me, -math.cos(me[2]), -math.sin(me[2]))
        if nb >= 1:
            return True, f"이웃 {nb}대 앞길과 겹침"
        mine = self._cached_agents.get(int(self.my_id))
        area = int(getattr(mine, "area_id", 0) or 0) if mine is not None else 0
        if area != 0 and bool(getattr(mine, "occupancy", False)):
            waiting = [int(m) for m, a in self._cached_agents.items()
                       if int(m) != int(self.my_id) and int(getattr(a, "area_id", 0) or 0) == area
                       and not bool(getattr(a, "occupancy", False))]
            if waiting:
                return True, f"구역 {area} 점유, 대기 {waiting}"
        now = self.get_clock().now()
        if self.retreat_gate_mutual_sec > 0.0 and self._stuck_sec(now) >= self.retreat_gate_mutual_sec:
            both = [m for m in near if self._agent_stuck_sec(m, now) >= self.retreat_gate_mutual_sec]
            if both:
                return True, f"이웃 {both} 와 상호 정지({self.retreat_gate_mutual_sec:.0f}s 이상)"
        return False, f"이웃 {near} 가까이 있지만 아무도 막고 있지 않음"

    def _agent_stuck_sec(self, mid: int, now: Time, dist_m: float = 0.3) -> float:
        """[V2.34b] 이웃이 0.3 m 안에 머문 시간 [s]. 소식이 2 s 넘게 끊겼으면 0 (멈췄다고 단정하지 않는다)."""
        st = self._agent_still.get(int(mid))
        if st is None:
            return 0.0
        t_now = now.nanoseconds * 1e-9
        if t_now - st[3] > 2.0:
            return 0.0
        return max(0.0, t_now - st[2])

    # ================= [OP 09-28] 관제 명령 우선 · 자기 기동 허가 =================
    # 상태별 허가표 (사용자 09-28 결정):
    #   RECEIVED_GOAL / PLANNING / DRIVING / READY : 양보성·자기 구출성 기동 모두 허용
    #   PAUSED (fleet 자체 조정, 관제 pause 아님)  : 양보성만 (자기 구출성 금지)
    #   RECOVERY_RUNNING / SUCCESS / FAILURE        : 양보성만
    #   IDLE / SUCCEEDED / CANCELED / FAILED       : 금지
    #   관제 pause(/nav_pause_flag) · 도킹 중       : 상태와 무관하게 전부 금지
    #   자기 구출성 = unwedge · 교차로 탈출. 그 밖(곡선·직선 후진 양보, 밀어내기, 순환·줄·정면 대치 해소)은 양보성.
    _OP_ACTIVE = ('READY', 'RECEIVED_GOAL', 'PLANNING', 'DRIVING', 'PAUSED',
                  'RECOVERY_RUNNING', 'RECOVERY_SUCCESS', 'RECOVERY_FAILURE')
    _OP_SELF_CALLERS = ('_v2_unwedge_tick',)
    # [09-30 사용자 결정 D10 (가)] 교차로 비우기는 자기 구출이 아니라 이웃을 위한 양보로 본다 — PAUSED·RECOVERY_* 에서도 허용.
    #   (sim L5f_dense1: 교차로 안 RECOVERY 로봇의 junction-escape 가 67회 불허돼 5대 교착이 판 끝까지 갔다)
    _OP_JUNCTION_CALLERS = ('_junction_escape',)
    # [L3 준비] 시험 hook(_on_test_retreat)은 이어지는 단계가 아니라 새 양보 기동이다 — 직전 기동 종류를 물려받지 않게 뺐다
    _OP_CONTINUATIONS = ('_maneuver_next', '_v2_maneuver_tick', '_retreat_along_history', '_backup_send')
    # 관제 pause 해제 때 pause 길이만큼 뒤로 미는 대기 시계들 (얼린 것처럼 이어서 센다)
    _OP_FREEZE_ATTRS = (
        '_pause_start_time', '_agent_pause_start_time', '_agent_clear_start_time', 'delay_after_agent_start_time',
        'delay_after_replan_start_time', 'delay_after_replan_start_time_goal_occupied',
        '_goal_occupied_false_start_time', '_last_goal_occupied_false_start_time', '_replan_flag_false_start_time',
        '_v2_pause_since', '_nav_stop_wait_start', '_np_anchor_t', '_active_since',
        '_jx_since', '_junction_clear_since', '_line_since', '_cycle_since', '_standoff_since', '_standoff_last',
        '_unwedge_last', '_unwedge_exhausted_at', '_goal_finish_last', '_recov_last', '_recov_cmd_until',
        '_planner_override_until', '_yield_start_t', '_yield_bt_fail_t', '_backup_started_at', '_man_wd_since',
        '_static_last_release_t', '_agent_last_release_t', '_jx_escape_last', '_retreat_clear_at')

    def _op_frozen(self) -> bool:
        """관제 pause 중 — fleet 판단 루프를 쉰다 (STOP·replan·goal 제거·기동을 내지 않는다)."""
        return self.operator_priority and self._nav_paused

    def _op_kind(self, name: str) -> str:
        """기동 종류: 'self'(자기 구출) | 'junction'(교차로 비우기, D10) | 'yield'(양보)."""
        if name in self._OP_SELF_CALLERS:
            return 'self'
        if name in self._OP_JUNCTION_CALLERS:
            return 'junction'
        return 'yield'

    def _motion_allowed(self, kind: str) -> bool:
        """fleet 이 스스로 로봇을 움직여도 되는가. kind: 'yield'(양보성) | 'junction'(교차로 비우기) | 'self'(자기 구출성)."""
        if not self.operator_priority:
            return True
        if self._nav_paused or self._docking:
            return False
        st = self.current_robot_status
        if st in ('RECEIVED_GOAL', 'PLANNING', 'DRIVING', 'READY'):
            return True
        if st in ('PAUSED', 'RECOVERY_RUNNING', 'RECOVERY_SUCCESS', 'RECOVERY_FAILURE'):
            return kind in ('yield', 'junction')      # [D10] 교차로 비우기도 허용
        return False                                  # IDLE / SUCCEEDED / CANCELED / FAILED / 그 밖

    def _op_gate(self, caller: str, start: bool = True) -> bool:
        """기동 입구 공통 관문. 새 기동이면 종류를 기록하고, 이어지는 단계면 기록된 종류로 판정한다."""
        if not self.operator_priority:
            return True
        if caller in self._OP_CONTINUATIONS:
            kind = self._man_kind
        else:
            kind = self._op_kind(caller)
        if not self._motion_allowed(kind):
            self.get_logger().warn(
                f"[OP] fleet 기동 불허 — {caller} ({kind}) status {self.current_robot_status}, "
                f"관제 pause {self._nav_paused}, 도킹 {self._docking}", throttle_duration_sec=10.0)
            return False
        if start and caller not in self._OP_CONTINUATIONS:
            self._man_kind = kind
        return True

    def _op_may(self, rule: str) -> bool:
        """[OP 공지 전 확인 · 09-28 L2] 기동 규칙이 공지(decision_state)·pause·시도 횟수 같은 부수 효과를 내기 **전에**
        허가를 본다. 종류 판정은 전송 단계 관문(_op_gate)과 같다 (rule = 전송 함수를 부르는 규칙 이름).
        불허면 아무것도 바꾸지 않는다 — 공지·pause·쿨다운·시도 횟수·1회성 표시를 쓰지 않는다.
        전송 단계 관문은 그대로 둔다 (공지와 전송 사이에 상태가 바뀌는 경우의 마지막 방어선)."""
        if not self.operator_priority:
            return True
        kind = self._op_kind(rule)
        if self._motion_allowed(kind):
            return True
        self.get_logger().info(
            f"[OP] 기동 조건은 됐지만 불허 — {rule} ({kind}) status {self.current_robot_status}, "
            f"관제 pause {self._nav_paused}, 도킹 {self._docking} (공지·pause 생략)", throttle_duration_sec=10.0)
        return False

    def _op_revoke(self, why: str) -> None:
        """허가가 사라졌다 — 진행 중인 fleet 기동을 즉시 거둔다. stop-and-go 재개 예약도 버린다."""
        self._man_ctx = None                          # 먼저 지워야 결과 콜백이 stop-and-go 재개를 예약하지 않는다
        self._maneuver = None
        self._backup_cancel()
        gh = self._maneuver_goal_handle
        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception:                                    # noqa: BLE001
                pass
            self._maneuver_goal_handle = None
        if self._yield_active:
            self._yield_finish(False, f"허가 상실 — {why}")
        if self._backup_state in ("sending", "running", "waiting"):
            self._backup_state = "failed"             # 주인 규칙이 실패로 정리한다
        self.get_logger().error(f"[OP] fleet 기동 회수 — {why}")
        self._publish_state(f"[OP] maneuver revoked ({why})")

    def _op_busy(self) -> bool:
        """fleet 기동(BackUp·Drive·곡선 후진·stop-and-go 재개 예약)이 진행 중인가."""
        return (self._backup_state in ("sending", "running", "waiting") or self._yield_active
                or self._maneuver is not None or self._man_ctx is not None)

    def _op_guard_tick(self) -> None:
        if not self.operator_priority:
            return
        if not self._op_busy():
            self._guard_revoked_logged = False
            return
        if self._motion_allowed(self._man_kind):
            return
        why = ("관제 pause" if self._nav_paused else "도킹 중" if self._docking
               else f"상태 {self.current_robot_status}")
        self._op_revoke(why)

    def _on_dock_monitoring(self, msg: DockingMonitoring) -> None:
        d = bool(msg.ros_dock_docking)
        if d:
            self._last_end = None                     # [D11] 도킹을 거친 뒤의 명령은 무진전 시계를 새로 잡는다
        if d != self._docking:
            self.get_logger().info(f"[OP] 도킹 {'시작' if d else '끝'} (ros_dock_docking={d})")
        self._docking = d

    def _op_unfreeze(self, now: Time) -> None:
        """관제 pause 해제 — 대기 시계를 pause 길이만큼 밀어 '얼렸다 녹인 것' 처럼 이어서 센다."""
        if self._nav_pause_since is None:
            return
        dt = now - self._nav_pause_since
        # [09-29] 무진전 시계(_stuck_sec)는 내 위치 이력(_own_hist) 시각으로 잰다. 대기 시계와 같이 얼렸다 녹인다:
        #   pause 전 기록은 pause 길이만큼 뒤로 밀고, pause 중 기록은 해제 시각으로 모은다 (pause 구간을 한 순간으로 접는다).
        #   예전에는 _active_since 만 밀려서, pause 직전까지 달리던 로봇도 해제 직후 "활성 시각 이후 전부" 를 무진전으로 셌다
        #   (sim op_bt_V1 04:31:43: 60 s 관제 pause 해제 1.5 s 뒤 junction-escape "32s 무진전" 후진).
        _p0 = self._nav_pause_since.nanoseconds * 1e-9
        _d = dt.nanoseconds * 1e-9
        _tr = now.nanoseconds * 1e-9
        if self._own_hist:
            self._own_hist = deque([((t + _d) if t <= _p0 else _tr, x, y) for (t, x, y) in self._own_hist],
                                   maxlen=self._own_hist.maxlen)
        self._nav_pause_since = None
        shifted = 0
        for name in self._OP_FREEZE_ATTRS:
            v = getattr(self, name, None)
            if isinstance(v, Time):
                setattr(self, name, v + dt)
                shifted += 1
        _sec = dt.nanoseconds * 1e-9
        _line = f"[OP] 관제 pause 해제 — {_sec:.0f}s 동안 fleet 판단 정지, 대기 시계 {shifted}개를 이어서 센다"
        if _sec >= 5.0:                               # [09-28 미결 4b 초안] 교차로 흐름제어의 짧은 pause 는 DEBUG
            self.get_logger().warn(_line)
        else:
            self.get_logger().debug(_line)

    def _on_nav_pause(self, msg: Bool) -> None:
        """[V2.37b] navigation_manager 가 관제 pause/resume(또는 새 move) 때 1회 발행하는 controller 정지 플래그."""
        if bool(msg.data) != self._nav_paused:
            self.get_logger().info(f"[V2] 관제 정지 플래그 {'켜짐' if msg.data else '꺼짐'} (/nav_pause_flag)")
        was = self._nav_paused
        self._nav_paused = bool(msg.data)
        if self.operator_priority and self._nav_paused and not was:
            self._nav_pause_since = self.get_clock().now()
            # 관제 pause 는 절대 우선 — 진행 중인 fleet 기동을 즉시 거둔다.
            # [L1 확인] 거둘 기동이 없으면 아무것도 하지 않는다 (교차로 흐름제어의 짧은 pause 마다 ERROR 로그가 쌓였다)
            if self._op_busy():
                self._op_revoke("관제 pause")
        elif self.operator_priority and was and not self._nav_paused:
            self._op_unfreeze(self.get_clock().now())

    def _retreat_gate_check(self, caller: str) -> bool:
        """[V2.34] 관문 판정 + 기록. 규칙 이름은 호출한 함수 이름에서 딴다.
        [V2.37b] 관제 정지 중이면 관문 설정과 무관하게 보류한다(respect_nav_pause)."""
        if self.respect_nav_pause and self._nav_paused:
            rule = caller.replace("_v2_", "").replace("_tick", "").strip("_")
            self.get_logger().warn(f"[V2 gate] {rule} 후진 보류 — 관제 정지 중(/nav_pause_flag), 제자리 유지",
                                   throttle_duration_sec=5.0)
            self._nav_pause_denied += 1
            return False
        if not self.retreat_gate_enable:
            return True
        rule = caller.replace("_v2_", "").replace("_tick", "").strip("_")
        ok, why = self._retreat_gate()
        if ok:
            self.get_logger().info(f"[V2 gate] {rule} 후진 허용 — {why}")
        else:
            self.get_logger().warn(f"[V2 gate] {rule} 후진 보류 — {why}", throttle_duration_sec=5.0)
            self._gate_denied = getattr(self, "_gate_denied", 0) + 1
        return ok

    def _shorten_retreats(self, me: Tuple[float, float, float],
                          cands: List[Tuple[float, float, float, str]]) -> List[Tuple[float, float, float, str]]:
        """[V2.33] 후보 목표를 필요 거리로 줄인다. 방향은 그대로(지나온 goal·이력 쪽 = 왔던 차선), 거리만 줄인다."""
        out: List[Tuple[float, float, float, str]] = []
        for x, y, yaw, src in cands:
            d = math.hypot(x - me[0], y - me[1])
            if d < 1e-3:
                continue
            ux, uy = (x - me[0]) / d, (y - me[1]) / d
            need, nb = self._retreat_need(me, ux, uy)
            if d > need:
                tx, ty = me[0] + ux * need, me[1] + uy * need
                if not self._yield_target_ok(me, tx, ty):
                    continue
                x, y, src = tx, ty, f"{src} → 필요 {need:.1f} m(이웃 {nb})"
            if any(math.hypot(x - o[0], y - o[1]) < 0.1 for o in out):
                continue                                  # 줄이고 나면 같은 점이 되는 후보는 하나만
            out.append((x, y, yaw, src))
        return out

    def _pick_yield_goal(self, away_from: Optional[Tuple[float, float]]) -> Optional[Tuple[float, float, float, str]]:
        """후보 목록의 첫 번째 (호환용)."""
        c = self._yield_candidates(away_from)
        return c[0] if c else None

    def _yield_candidates(self, away_from: Optional[Tuple[float, float]]) -> List[Tuple[float, float, float, str]]:
        """[09-20 사용자] "직전에 지나온 goal 로 먼저, 실패하면 더 전 goal 로" — 후보를 **순서대로** 만든다.
        ① 지나온 goal(최신 → 과거, 최대 yield_passed_goal_max 개) ② 주행 이력 위의 점 ③ 바로 뒤 1.0/0.7/0.5 m.
        각 후보는 뒤쪽 부채꼴·지도(목표·경로 구간)·상대 점유·막은 로봇에서 멀어지는지를 통과해야 한다.
        방향(yaw) 은 모두 **현재 방향** — 후진 전용 플래너가 방향까지 맞추려다 실패하는 것을 막는다."""
        out: List[Tuple[float, float, float, str]] = []
        me = self._own_pose()
        if me is None:
            return out
        d0 = math.hypot(away_from[0] - me[0], away_from[1] - me[1]) if away_from else 0.0

        def behind(x: float, y: float) -> bool:
            """후진 전용 플래너가 갈 수 있는 방향인가 (내 뒤쪽 부채꼴)."""
            if not self.yield_bt_enable:
                return True
            return abs(self._ang_wrap(math.atan2(y - me[1], x - me[0]) - (me[2] + math.pi))) \
                <= math.radians(self.yield_rear_cone_deg)

        tried = 0
        for g in reversed(self._passed_goals):
            if tried >= self.yield_passed_goal_max:
                break
            tried += 1
            dg = math.hypot(g[0] - me[0], g[1] - me[1])
            if dg < self.yield_goal_min_m:
                continue                                  # 너무 가깝다 — 비켜 주는 효과가 없다
            if not behind(g[0], g[1]):
                continue                                  # 앞쪽 goal 은 후진으로 못 간다 (큐17 1판 실패 원인)
            if not self._yield_target_ok(me, g[0], g[1]):
                self.get_logger().info(f"[V2 yield] 지나온 goal ({g[0]:.2f},{g[1]:.2f}) 는 지도상 막혀 있다 → 다음 후보")
                continue
            if self._goal_occupied_by_agent(g[0], g[1]):
                self.get_logger().info(f"[V2 yield] 지나온 goal ({g[0]:.2f},{g[1]:.2f}) 는 다른 로봇이 점유 → 더 앞의 goal 로")
                continue
            if away_from is not None and math.hypot(away_from[0] - g[0], away_from[1] - g[1]) - d0 < 0.5:
                continue                                  # 막은 로봇에서 멀어지지 않는 방향
            if dg > self.yield_goal_max_m:
                # [09-20] 통로망 waypoint 간격은 5 m 가 넘어 역방향 플래너가 자주 실패한다(큐17 1판).
                # 방향(그 goal 쪽 = 왔던 차선) 은 그대로 두고 거리만 잘라 가까운 후퇴점으로 만든다.
                r = self.yield_goal_max_m / dg
                tx, ty = me[0] + (g[0] - me[0]) * r, me[1] + (g[1] - me[1]) * r
                if not self._yield_target_ok(me, tx, ty):
                    continue
                out.append((tx, ty, me[2], f"지나온 goal({tried}번째 앞, {self.yield_goal_max_m:.1f} m)"))
                continue
            out.append((g[0], g[1], me[2], f"지나온 goal({tried}번째 앞)"))
            continue
        plan = self._plan_retreat(self.push_retreat_m, away_from)
        if (plan is not None and behind(plan["target"][0], plan["target"][1])
                and self._yield_target_ok(me, plan["target"][0], plan["target"][1])):
            t = plan["target"]
            out.append((t[0], t[1], me[2], f"주행 이력({plan.get('cand', 'hist')})"))
        for frac in (1.0, 0.7, 0.5):
            bx = me[0] - self.push_retreat_m * frac * math.cos(me[2])
            by = me[1] - self.push_retreat_m * frac * math.sin(me[2])
            if not self._yield_target_ok(me, bx, by):
                continue                                  # 벽 너머로 물러나라고 하면 플래너가 헤맨다 (큐18 1판)
            if away_from is None or math.hypot(away_from[0] - bx, away_from[1] - by) - d0 >= 0.3:
                out.append((bx, by, me[2], f"바로 뒤({self.push_retreat_m * frac:.1f} m)"))
        if self.retreat_need_enable:
            out = self._shorten_retreats(me, out)
        return out

    def _yield_bt_ready(self, now: Time) -> bool:
        """BT 후퇴를 쓸 수 있는가. 직전 실패 뒤 yield_fallback_backup_sec 동안은 직선 BackUp 으로 되돌린다."""
        if not self.yield_bt_enable:
            return False
        if self.yield_bt_active_only and self.current_robot_status in ('SUCCEEDED', 'CANCELED', 'FAILED', 'ABORTED'):
            return False              # [V2.37a] 종료 상태 — BT 가 돌지 않는다 → 직접 기동
        if self.current_robot_status in ('READY', 'IDLE'):
            return False              # [V2.8] 주행이 없으면 BT 가 양보 경로를 따라갈 수 없다 → 직접 기동
        if self._yield_bt_fail_t is None:
            return True
        return (now - self._yield_bt_fail_t).nanoseconds * 1e-9 >= self.yield_fallback_backup_sec

    def _yield_start(self, now: Time, away_from: Optional[Tuple[float, float]] = None) -> bool:
        cands = self._yield_candidates(away_from)
        if not cands:
            self.get_logger().error("[V2 yield] 후퇴 목표를 못 정했다 (지나온 goal·이력 모두 없음)")
            return False
        self._yield_cands = cands
        self._yield_idx = 0
        self.get_logger().warn(
            f"[V2 yield] 후퇴 후보 {len(cands)}개: " + " / ".join(f"{i+1}) {c[3]}" for i, c in enumerate(cands)))
        return self._yield_publish(now, cands[0])

    def _yield_publish(self, now: Time, cand: Tuple[float, float, float, str]) -> bool:
        x, y, yaw, src = cand
        # [09-20 15:20] 후진 전용 플래너는 **목표 자세의 방향**까지 맞춰야 한다. 지나온 goal 의 yaw 를 그대로 쓰면
        # 내 현재 방향과 최대 180° 차이 나서(반대 방향으로 지났던 goal) 좁은 통로에서는 해가 없다 — 실측 실패의 큰 몫
        # ("no valid path found" A 221 / B 243). 위치만 쓰고 **방향은 내 현재 방향**으로 둔다(그대로 뒤로 물러나는 자세).
        ps = PoseStamped()
        ps.header.frame_id = self.global_frame
        ps.header.stamp = now.to_msg()
        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_yield_goal.publish(ps)
        self.pub_ctrl_sel.publish(String(data=self.get_parameter("yield_reverse_controller").value))
        self.pub_yield_flag.publish(Bool(data=True))
        self.pub_cmd_resume.publish(Bool(data=False))     # BT 가 후퇴 경로를 주행할 수 있게 pause 를 푼다
        self._yield_active = True
        self._yield_target = (float(x), float(y), float(yaw))
        _me = self._own_pose()
        if self.yield_reach_frac > 0.0 and _me is not None:      # [V2.35] 목표 거리에 비례한 도달 허용 오차
            _d0 = math.hypot(float(x) - _me[0], float(y) - _me[1])
            self._yield_tol = min(self.yield_reach_tol, max(0.05, self.yield_reach_frac * _d0))
        else:
            self._yield_tol = self.yield_reach_tol
        self._yield_start_t = now
        self._yield_src = src
        me0 = self._own_pose()
        self._yield_start_xy = (me0[0], me0[1]) if me0 else None
        self._backup_state = "running"                    # 기존 기동 상태기계를 그대로 쓴다
        self._man_ctx = None
        self.get_logger().warn(
            f"[V2 yield] 후퇴 목표 ({x:.2f},{y:.2f}) [{src}] → BT 가 {self.get_parameter('yield_reverse_controller').value} 로 곡선 후진한다")
        self._publish_state(f"[V2 yield] RETREAT to ({x:.2f},{y:.2f}) [{src}]")
        return True

    def _yield_finish(self, ok: bool, why: str) -> None:
        self.pub_yield_flag.publish(Bool(data=False))
        self.pub_ctrl_sel.publish(String(data=self.get_parameter("yield_default_controller").value))
        self._yield_active = False
        self._yield_target = None
        self._yield_start_t = None
        self._yield_start_xy = None
        self._yield_cands = []
        self._yield_idx = 0
        self._backup_state = "succeeded" if ok else "failed"
        if not ok and "재잠금" not in why:
            self._yield_bt_fail_t = self.get_clock().now()    # 당분간은 직선 BackUp 으로 (역방향 플래너가 안 되는 자리)
        if ok:                                            # [09-20 23:20] 한 줄에서 warn/error 를 번갈아 쓰면 rclpy 가 예외를 던져 노드가 죽는다
            self.get_logger().warn(f"[V2 yield] 후퇴 완료: {why}")
        else:
            self.get_logger().error(f"[V2 yield] 후퇴 실패: {why}")
        self._publish_state(f"[V2 yield] {'DONE' if ok else 'FAILED'} ({why})")

    def _init_latched_flags(self) -> None:
        """[V2.36d] TRANSIENT_LOCAL 로 남는 1회성 트리거(회복 명령)를 false 로 초기화한다.
        노드가 새로 뜰 때(재시작 포함) 부른다. 이 발행자의 마지막 값이 false 가 되므로 이후에 붙는 BT 도 false 를 받는다.
        후퇴 플래그는 VOLATILE 하트비트라 잔여값이 없어 초기화가 필요 없다."""
        for p in self.pub_recov_cmd.values():
            p.publish(Bool(data=False))
        self.get_logger().info("[V2] 회복 명령 트리거 초기화(false) — TRANSIENT_LOCAL 잔여값 제거")

    def _v2_yield_beat(self) -> None:
        """후퇴 중에만 플래그·목표를 2 Hz 로 다시 보낸다 (비래치이므로 하트비트가 끊기면 BT 는 곧바로 평소 경로로 돌아간다)."""
        if not (self.v2_enable and self.yield_bt_enable and self._yield_active and self._yield_target):
            return
        now = self.get_clock().now()
        ps = PoseStamped()
        ps.header.frame_id = self.global_frame
        ps.header.stamp = now.to_msg()
        ps.pose.position.x, ps.pose.position.y = float(self._yield_target[0]), float(self._yield_target[1])
        ps.pose.orientation.z = math.sin(self._yield_target[2] / 2.0)
        ps.pose.orientation.w = math.cos(self._yield_target[2] / 2.0)
        self.pub_yield_goal.publish(ps)
        self.pub_yield_flag.publish(Bool(data=True))

    def _v2_yield_tick(self) -> None:
        if self._op_frozen():
            return                                    # [OP] 관제 pause 중 — 판단 정지
        if not (self.v2_enable and self.yield_bt_enable and self._yield_active):
            return
        now = self.get_clock().now()
        me = self._own_pose()
        if me is not None and self._yield_target is not None:
            d = math.hypot(me[0] - self._yield_target[0], me[1] - self._yield_target[1])
            if d <= (self._yield_tol if self._yield_tol is not None else self.yield_reach_tol):
                self._yield_finish(True, f"{d:.2f} m 이내 도달")
                return
        if self._yield_start_t is None:
            return
        dt = (now - self._yield_start_t).nanoseconds * 1e-9
        stuck = (me is not None and self._yield_start_xy is not None
                 and math.hypot(me[0] - self._yield_start_xy[0], me[1] - self._yield_start_xy[1]) < 0.2)
        if stuck and dt >= self.yield_try_sec and self._yield_idx + 1 < len(self._yield_cands):
            # [사용자 09-20] 직전 goal 로 안 되면 **그보다 더 전 goal** 로 경로를 만들어 본다
            self._yield_idx += 1
            nxt = self._yield_cands[self._yield_idx]
            self.get_logger().warn(
                f"[V2 yield] {dt:.0f}s 동안 못 움직였다 → 다음 후보 {self._yield_idx + 1}/{len(self._yield_cands)}: {nxt[3]}")
            self._yield_publish(now, nxt)
            return
        if stuck and dt >= self.yield_no_move_sec:
            self._yield_finish(False, f"{dt:.0f}s 동안 0.2 m 도 못 움직였다 (후보 {self._yield_idx + 1}/{len(self._yield_cands)} 소진)")
            return
        if dt >= self.yield_timeout_sec:
            self._yield_finish(False, f"{self.yield_timeout_sec:.0f}s 시한 초과")

    def _on_bt_error(self, msg: UInt16) -> None:
        if int(msg.data) != 0:
            self._bt_errs.append((self.get_clock().now().nanoseconds * 1e-9, int(msg.data)))

    def _v2_recov_cmd_beat(self) -> None:
        """[V2.6] 노드가 고른 회복 동작을 BT 에 지시한다 (비래치 하트비트 — 끊기면 BT 는 곧 평소 흐름으로)."""
        if not (self.v2_enable and self.recovery_cmd_enable and self._recov_cmd):
            return
        now = self.get_clock().now()
        if self._recov_cmd_until is not None and now >= self._recov_cmd_until:
            self._recov_cmd = None
            self._recov_cmd_until = None
            return
        pub = self.pub_recov_cmd.get(self._recov_cmd)
        if pub is not None:
            pub.publish(Bool(data=True))

    def _v2_send_recov_cmd(self, cmd: str, now: Time) -> None:
        self._recov_cmd = cmd
        self._recov_cmd_until = now + Duration(seconds=self.recovery_cmd_hold)
        self.get_logger().warn(f"[V2 recovery] BT 에 회복 동작 지시: {cmd} ({self.recovery_cmd_hold:.1f}s)")
        self._publish_state(f"[V2 recovery] cmd {cmd}")

    def _v2_bt_error_tick(self) -> None:
        """[V2.4] BT 오류 코드로 **노드가** 회복 조치를 고른다 (BT RECOVERY CASE 선택권 이관 1단계).
        같은 창 안에 오류가 몰리고 로봇이 제자리면 ① 코스트맵 클리어 ② 예비 플래너 A ③ 예비 플래너 B ④ 자기 구출."""
        if self._op_frozen():
            return                                    # [OP] 관제 pause 중 — 판단 정지
        if not (self.v2_enable and self.bt_error_react):
            return
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        if self._planner_override_until is not None and now >= self._planner_override_until:
            self._planner_override_until = None
            self.pub_planner_sel.publish(String(data=self.default_planner))
            self.get_logger().warn(f"[V2 recovery] 플래너를 기본값({self.default_planner}) 으로 되돌린다")
        if self._recov_last is not None and (now - self._recov_last).nanoseconds * 1e-9 < self.bt_error_cooldown:
            return
        if self._backup_state != "idle" or self._yield_active or self._unwedge_active:
            return
        if self.is_processing_agent_pause or self.is_processing_goal_occupied_pause \
                or self.is_processing_last_goal_occupied_pause:
            return                                     # 조정 대기 중이면 그 규칙이 우선
        recent = [c for (ts, c) in self._bt_errs if t - ts <= self.bt_error_window]
        if len(recent) < self.bt_error_burst_n:
            return
        if self._stuck_sec(now) < self.bt_error_stuck_sec:
            self._recov_step = 0
            return
        code, cnt = Counter(recent).most_common(1)[0]
        self._recov_step = min(self._recov_step + 1, 4)
        self._recov_last = now
        if self._recov_step == 1:
            if self.recovery_cmd_enable:
                self._v2_send_recov_cmd("clear", now)
            for cli in (self._clear_local, self._clear_global):
                if cli.service_is_ready():
                    cli.call_async(Empty.Request())
            self.get_logger().warn(
                f"[V2 recovery] BT 오류 {code}×{cnt} + {self._stuck_sec(now):.0f}s 무진전 → 1단계: 코스트맵 클리어")
            self._publish_state("[V2 recovery] clear costmaps")
        elif self._recov_step in (2, 3):
            pl = self.recovery_planner_a if self._recov_step == 2 else self.recovery_planner_b
            if self.recovery_cmd_enable:
                # [V2.6] 현장 recovery 의 **대안 경로 기동**을 그대로 쓰게 지시한다 (플래너 전환보다 강건했다)
                self._v2_send_recov_cmd("maneuver_a" if self._recov_step == 2 else "maneuver_b", now)
            self.pub_planner_sel.publish(String(data=pl))
            self._planner_override_until = now + Duration(seconds=self.recovery_planner_hold)
            self.get_logger().warn(f"[V2 recovery] {self._recov_step}단계: 예비 플래너 {pl} ({self.recovery_planner_hold:.0f}s)")
            self._publish_state(f"[V2 recovery] planner → {pl}")
        else:
            self.get_logger().warn("[V2 recovery] 4단계: 자기 구출 기동으로 넘긴다")
            self._unwedge_last = None
            self._recov_step = 0

    def _v2_unwedge_tick(self) -> None:
        """[V2.3] 이웃과 무관한 끼임(회복 루프) 을 스스로 푼다: 조금 물러나 플래너가 시작점을 다시 잡게 한다."""
        if self._op_frozen():
            return                                    # [OP] 관제 pause 중 — 판단 정지
        if not (self.v2_enable and self.unwedge_enable):
            return
        now = self.get_clock().now()
        if self._unwedge_active:
            if self._backup_state in ("succeeded", "failed"):
                ok = self._backup_state == "succeeded"
                self._backup_state = "idle"
                self._unwedge_active = False
                self.pub_cmd_resume.publish(Bool(data=False))
                if ok:                                    # (심각도를 한 줄에서 바꾸면 rclpy 예외 → 노드 사망)
                    self.get_logger().warn(
                        f"[V2 unwedge] 자기 구출 성공 (시도 {self._unwedge_tries}/{self.unwedge_max_tries})")
                else:
                    self.get_logger().error(
                        f"[V2 unwedge] 자기 구출 실패 (시도 {self._unwedge_tries}/{self.unwedge_max_tries})")
                self._publish_state(f"[V2 unwedge] {'DONE' if ok else 'FAILED'}")
            return
        if self._backup_state != "idle" or self.nav_stop_complete_ is False or self._yield_active:
            return
        if self.is_processing_agent_pause or self.is_processing_replan_pause \
                or self.is_processing_goal_occupied_pause or self.is_processing_last_goal_occupied_pause:
            return                                    # 조정 대기 중이면 그쪽 규칙이 맡는다 (대기 상한까지 기다린다)
        if self.current_robot_status not in ('RECEIVED_GOAL', 'PLANNING', 'DRIVING',
                                             'RECOVERY_FAILURE', 'RECOVERY_RUNNING', 'RECOVERY_SUCCESS'):
            return
        if self._unwedge_last is not None and \
                (now - self._unwedge_last).nanoseconds * 1e-9 < self.unwedge_cooldown_sec:
            return
        if self._stuck_sec(now) < self.unwedge_after_sec:
            self._unwedge_tries = 0
            self._unwedge_exhausted_at = None
            return
        turn = self._line_my_turn(now)
        if turn is False:
            self.get_logger().info("[V2 line] 줄 해소 순서가 아니다 — 이번엔 기동하지 않는다",
                                   throttle_duration_sec=20.0)
            return
        me = self._own_pose()
        if me is None:
            return
        # [V2.15] (가) 자리가 바뀌었으면 새 상황이다 — 시도 횟수를 되돌린다
        if self.unwedge_persist and (self._unwedge_anchor is None or
                                     math.hypot(me[0] - self._unwedge_anchor[0],
                                                me[1] - self._unwedge_anchor[1]) > 0.3):
            self._unwedge_anchor = (me[0], me[1])
            self._unwedge_tries = 0
            self._unwedge_exhausted_at = None
        if self._unwedge_tries >= self.unwedge_max_tries:
            if not self.unwedge_persist:          # [A/B] 예전 동작: 영구 포기
                self.get_logger().error(
                    f"[V2 unwedge] {self.unwedge_max_tries}회 시도에도 못 벗어났다 — 관제 보고에 맡긴다",
                    throttle_duration_sec=30.0)
                return
            # [V2.15] (나) 영구 포기 대신 **쉬었다가 다시**
            if self._unwedge_exhausted_at is None:
                self._unwedge_exhausted_at = now
                self.get_logger().error(
                    f"[V2 unwedge] {self.unwedge_max_tries}회 시도 실패 → {self.unwedge_reset_sec:.0f}s 쉬었다가 다시 시도한다")
                return
            if (now - self._unwedge_exhausted_at).nanoseconds * 1e-9 < self.unwedge_reset_sec:
                return
            self._unwedge_tries = 0
            self._unwedge_exhausted_at = None
            self.get_logger().warn("[V2 unwedge] 쉬는 시간이 끝났다 → 자기 구출을 다시 시작한다")
        tgt = None
        for frac in (1.0, 0.7, 0.5):
            for sign, tag in ((-1.0, '뒤'), (1.0, '앞')):
                x = me[0] + sign * self.unwedge_dist_m * frac * math.cos(me[2])
                y = me[1] + sign * self.unwedge_dist_m * frac * math.sin(me[2])
                if self._map_free(x, y) and self._path_free(me[0], me[1], x, y):
                    tgt = (x, y, sign, self.unwedge_dist_m * frac, tag)
                    break
            if tgt:
                break
        if tgt is None:
            self.get_logger().error("[V2 unwedge] 앞뒤 모두 지도상 막혀 있다 — 구출 기동 없음", throttle_duration_sec=30.0)
            self._unwedge_last = now
            return
        if not self._op_may('_v2_unwedge_tick'):       # [OP] 불허면 시도 횟수·pause·공지를 쓰지 않는다
            return                                    # (예전: RECOVERY 중 pause 1 s 로 BT 복구를 붙잡고 시도 1/6 을 썼다)
        self._unwedge_tries += 1
        self._unwedge_last = now
        self._unwedge_active = True
        self.get_logger().warn(
            f"[V2 unwedge] {self._stuck_sec(now):.0f}s 무진전 + {self.current_robot_status} → {tgt[4]}로 {tgt[3]:.1f} m 빠져나간다 "
            f"(시도 {self._unwedge_tries}/{self.unwedge_max_tries})")
        self._publish_state(f"[V2 unwedge] escape {tgt[4]} {tgt[3]:.1f} m")
        self._publish_pause()
        if tgt[2] < 0:
            saved = self.yield_backoff_m
            self.yield_backoff_m = tgt[3]
            try:
                self._backup_send(now)
            finally:
                self.yield_backoff_m = saved
        else:
            self._drive_send(tgt[3])

    def _standoff_partner(self, now: Time) -> Optional[int]:
        """나와 정면으로 맞선 채 둘 다 멈춰 있는 상대. 없으면 None."""
        me = self._own_pose()
        if me is None:
            return None
        best = None; best_d = 1e9
        for mid, a in self._cached_agents.items():
            if int(mid) == int(self.my_id):
                continue
            if self._check_vehicle_immobile(a) or self._agent_is_moving(a, now):
                continue
            p = a.current_pose.pose.position
            d = math.hypot(p.x - me[0], p.y - me[1])
            if d > self.standoff_dist_m or d >= best_d:
                continue
            brg = math.atan2(p.y - me[1], p.x - me[0])
            if abs(self._ang_wrap(brg - me[2])) > math.radians(self.standoff_cone_deg):
                continue                                   # 내 앞이 아니다
            if abs(self._ang_wrap(self._agent_yaw(a) - me[2])) < math.radians(self.standoff_face_deg):
                continue                                   # 같은 방향 = 줄서기이지 대치가 아니다
            if self._agent_stopped_sec(int(mid), now) < self.standoff_after_sec:
                continue
            best = int(mid); best_d = d
        return best

    def _v2_goal_finish_tick(self) -> None:
        """[V2.12] goal 코앞에서 오래 서 있으면 조정 pause 를 풀어 완주를 마치게 한다.
        그 자리를 비워야 뒤가 풀린다 (q10: r1 이 자기 goal 앞에서 107 s 서 있는 동안 r4 가 막혔다)."""
        if self._op_frozen():
            return                                    # [OP] 관제 pause 중 — 판단 정지
        if not self.v2_enable or self.goal_finish_sec <= 0.0:
            return
        if not self._near_own_goal(self.goal_finish_m):
            self._goal_finish_tries = 0
            self._goal_finish_anchor = None
            return
        now = self.get_clock().now()
        if self._backup_state != "idle" or self._yield_active or self._unwedge_active or self._standoff_active:
            return                                     # 기동 중이면 그쪽이 끝나고 나서
        if self.current_robot_status in ('RECOVERY_RUNNING', 'RECOVERY_FAILURE'):
            return                                     # 회복 중에는 BT 가 주도한다
        me = self._own_pose()
        if me is None:
            return
        if self._goal_finish_anchor is None or \
                math.hypot(me[0] - self._goal_finish_anchor[0], me[1] - self._goal_finish_anchor[1]) > 0.15:
            self._goal_finish_anchor = (me[0], me[1])   # 움직였으면 시도 횟수를 되돌린다
            self._goal_finish_tries = 0
        if self._stuck_sec(now) < self.goal_finish_sec:
            return
        if self._goal_finish_last is not None and \
                (now - self._goal_finish_last).nanoseconds * 1e-9 < self.goal_finish_cooldown_sec:
            return
        if self._goal_finish_tries >= self.goal_finish_max_tries:
            self.get_logger().error(
                "[V2 goal-finish] goal 코앞인데 재개로도 못 끝냈다 — 회복/관제에 맡긴다", throttle_duration_sec=30.0)
            return
        self._goal_finish_tries += 1
        self._goal_finish_last = now
        self.get_logger().warn(
            f"[V2 goal-finish] goal 이 {self.goal_finish_m} m 안인데 {self._stuck_sec(now):.0f}s 제자리 → "
            f"조정 대기를 풀고 완주를 끝낸다 (시도 {self._goal_finish_tries}/{self.goal_finish_max_tries})")
        self._publish_state("[V2 goal-finish] RESUME to finish goal")
        self.pub_cmd_resume.publish(Bool(data=False))
        self.is_processing_agent_pause = False
        self.is_processing_goal_occupied_pause = False
        self.is_processing_last_goal_occupied_pause = False

    def _v2_maneuver_watchdog_tick(self) -> None:
        """[V2.16] 기동 상태가 굳으면 강제로 푼다.

        `_backup_state` 가 idle 이 아닌 동안에는 순환 해소·밀어내기·막힌 이웃 양보·정면 대치·자기 구출·완주 우선이
        **전부** 막힌다. behavior_server 응답이 한 번 유실되면 그 로봇은 남은 시간 내내 자기 구출을 잃는다.
        기존 감시(`_v2_phase1_tick`)는 조정 대기 중에만 돌고 "waiting" 은 제외한다 — 여기서 모든 상태를 본다.
        """
        if self._op_frozen():
            return                                    # [OP] 관제 pause 중 — 판단 정지
        if not self.v2_enable or self.maneuver_watchdog_sec <= 0.0:
            return
        now = self.get_clock().now()
        st = self._backup_state
        if st != self._man_wd_state:
            self._man_wd_state = st
            self._man_wd_since = now
            return
        if st == "idle" or self._man_wd_since is None:
            return
        held = (now - self._man_wd_since).nanoseconds * 1e-9
        if held < self.maneuver_watchdog_sec:
            return
        self.get_logger().error(
            f"[V2 watchdog] 기동 상태가 '{st}' 로 {held:.0f}s 굳었다 → 강제 해제 "
            f"(이 상태에서는 자기 구출 규칙이 전부 막힌다)")
        try:
            self._backup_cancel()
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().error(f"[V2 watchdog] 기동 취소 실패: {exc!r}")
        self._backup_state = "idle"
        self._man_ctx = None
        self._maneuver = None
        self._yield_active = False
        self._unwedge_active = False
        self._standoff_active = False
        self._push_paused = False
        self._man_wd_state = "idle"
        self._man_wd_since = now
        self.pub_cmd_resume.publish(Bool(data=False))
        self._publish_state("[V2 watchdog] maneuver state cleared")

    def _line_members(self, now: Time) -> List[Tuple[int, float, float]]:
        """[V2.17] 나를 포함해 **줄지어 멈춰 있는** 로봇들. 없으면 빈 목록.

        맵이 없어도 된다 — 이웃 자세만으로 `line_gap_m` 연쇄를 따라간다. 모두가 같은 입력으로
        같은 계산을 하므로 **메시지 없이 같은 결과**에 도달한다(관제 변경 불필요).
        """
        me = self._own_pose()
        if me is None or self._stuck_sec(now) < self.line_stuck_sec:
            return []
        pts = {int(self.my_id): (me[0], me[1])}
        for mid, a in self._cached_agents.items():
            mid = int(mid)
            if mid == int(self.my_id):
                continue
            if self._agent_is_moving(a, now) or self._check_vehicle_immobile(a):
                continue
            if self._agent_stopped_sec(mid, now) < self.line_stuck_sec:
                continue
            p = a.current_pose.pose.position
            if math.isfinite(p.x) and math.isfinite(p.y):
                pts[mid] = (p.x, p.y)
        # 나에서 출발해 line_gap_m 연쇄로 이어지는 덩어리만 남긴다
        seen = {int(self.my_id)}
        frontier = [int(self.my_id)]
        while frontier:
            cur = frontier.pop()
            for mid, q in pts.items():
                if mid in seen:
                    continue
                if math.hypot(q[0] - pts[cur][0], q[1] - pts[cur][1]) <= self.line_gap_m:
                    seen.add(mid); frontier.append(mid)
        if len(seen) < self.line_min_n:
            return []
        return sorted(((m, pts[m][0], pts[m][1]) for m in seen), key=lambda t: t[0])

    def _line_order(self, members: List[Tuple[int, float, float]]) -> List[int]:
        """줄의 축을 따라 **한쪽 끝부터** 나가는 순서. 축의 부호를 고정해 모두가 같은 순서를 얻는다."""
        xs = [m[1] for m in members]; ys = [m[2] for m in members]
        dx = max(xs) - min(xs); dy = max(ys) - min(ys)
        if dx >= dy:                                   # 가로 줄 → x 축, +x 쪽 끝부터
            key = lambda m: (-m[1], m[0])
        else:                                          # 세로 줄 → y 축, +y 쪽 끝부터
            key = lambda m: (-m[2], m[0])
        return [m[0] for m in sorted(members, key=key)]

    def _line_my_turn(self, now: Time) -> Optional[bool]:
        """줄이 잡혔으면 지금이 내 차례인지. 줄이 없으면 None."""
        if not (self.v2_enable and self.line_evac_enable):
            return None
        members = self._line_members(now)
        if not members:
            if self._line_sig is not None:
                self.get_logger().info(f"[V2 line] 줄 해소됨 {self._line_sig}")
            self._line_sig = None; self._line_since = None
            return None
        sig = tuple(sorted(m[0] for m in members))
        if sig != self._line_sig:
            self._line_sig = sig; self._line_since = now
            order = self._line_order(members)
            self.get_logger().warn(
                f"[V2 line] 통로에 {len(members)}대가 줄지어 멈췄다 {sig} → 순서대로 한 대씩 뺀다 "
                f"(순서 {'>'.join(str(x) for x in order)}, 내 차례 {order.index(int(self.my_id)) + 1}번째)")
            self._publish_state(f"[V2 line] queue {len(members)} rank {order.index(int(self.my_id))}")
        order = self._line_order(members)
        try:
            rank = order.index(int(self.my_id))
        except ValueError:
            return None
        dt = (now - self._line_since).nanoseconds * 1e-9 if self._line_since else 0.0
        # [V2.25] 후방 실측으로 합의 없이 순서를 정한다
        if self.line_rear_first:
            rb = self._rear_blocked(self.line_rear_probe_m)
            if rb is True:
                self.get_logger().info(
                    f"[V2 line] 후방 {self._rear_block_at} m 가 막혔다 — 내 차례로 치지 않는다 "
                    f"(움직일 수 없는 로봇을 기다리면 줄 전체가 멈춘다)", throttle_duration_sec=20.0)
                return False
            if rb is False and dt >= self.line_escape_sec:
                # [09-24] throttle — 조건이 서면 매 tick True 라 한 판에 3952줄 찍혔다 (동작은 정상)
                self.get_logger().warn(
                    f"[V2 line] 줄이 {dt:.0f}s 지속 — 순서를 무시하고 후방이 빈 내가 움직인다 "
                    f"(순번 합의가 깨져도 줄이 풀리게)", throttle_duration_sec=10.0)
                return True
        active = int(dt // max(1.0, self.line_turn_sec)) % len(order)
        return rank == active

    def _junction_escape(self, now: Time, area: int) -> None:
        """[V2.26b] 교차로 안에서 junction_escape_after_sec 이상 무진전이면 들어온 길로 빠져나온다."""
        if not self.junction_escape_enable:
            return
        # 내가 낸 기동의 결과는 내가 거둔다 — succeeded/failed 가 남으면 다른 규칙 7곳이 전부 막힌다(V2.16 교훈)
        if self._jx_escape_active and self._backup_state in ("succeeded", "failed"):
            self.get_logger().warn(f"[V2 junction-escape] 빠져나오기 {'완료' if self._backup_state == 'succeeded' else '실패'}")
            self._backup_state = "idle"
            self._jx_escape_active = False
            return
        if self._backup_state != "idle":
            return
        if self._stuck_sec(now) < self.junction_escape_after:
            return
        if self._jx_escape_last is not None and \
                (now - self._jx_escape_last).nanoseconds * 1e-9 < self.junction_escape_cooldown:
            return
        if self._rear_blocked(self.junction_escape_m) is True:
            self.get_logger().info(
                f"[V2 junction-escape] 구역 {area} 안에서 멈췄지만 후방 {self._rear_block_at} m 가 막혔다 — 못 뺀다",
                throttle_duration_sec=20.0)
            return
        if not self._op_may('_junction_escape'):       # [OP] 불허면 쿨다운·공지 없이 넘긴다
            return
        self._jx_escape_last = now
        self.get_logger().warn(
            f"[V2 junction-escape] 구역 {area} 안에서 {self._stuck_sec(now):.0f}s 무진전 "
            f"→ 들어온 길로 {self.junction_escape_m} m 빠져나온다 (교차로를 비워 사방을 살린다)")
        self._publish_state(f"[V2 junction-escape] RETREAT {self.junction_escape_m} m (area {area})")
        if self._retreat_along_history(now, self.junction_escape_m, None):
            self._jx_escape_active = True
        else:
            self._backup_state = "idle"          # 계획이 안 나오면 상태를 건드리지 않고 쿨다운 뒤 재시도

    def _v2_junction_exit_tick(self) -> None:
        """[V2.26] 출구가 막혔으면 교차로에 들어가지 않는다 (교차로 안 정지 방지)."""
        if self._op_frozen():
            return                                    # [OP] 관제 pause 중 — 판단 정지
        # [V2.26c 09-25] 탈출(V2.26b)만 켠 설정(v229)에서도 이 틱이 돌아야 한다.
        # 전에는 junction_exit_enable 이 꺼져 있으면 여기서 바로 돌아가서 _junction_escape 가 한 번도 불리지 않았다
        # (X1~X3: 발화 0회).
        if not (self.v2_enable and (self.junction_exit_enable or self.junction_escape_enable)):
            return
        now = self.get_clock().now()
        me = self._cached_agents.get(int(self.my_id))

        def release(why: str) -> None:
            if self._jx_hold:
                self._jx_hold = False
                self._jx_since = None
                self.pub_cmd_resume.publish(Bool(data=False))
                self._publish_state(f"[V2 junction-exit] RUN ({why})")
                self.get_logger().warn(f"[V2 junction-exit] 진입 보류 해제 — {why}")

        if me is None:
            return release("내 정보 없음")
        # 흐름제어가 채워 주는 신호: area_id != 0 이면 구역 접근/대기, occupancy 면 이미 안에 있다.
        area = int(getattr(me, "area_id", 0) or 0)
        inside = bool(getattr(me, "occupancy", False))
        if area != 0 and inside:
            self._junction_escape(now, area)
        if not self.junction_exit_enable:        # [V2.26c] 탈출만 켰으면 진입 보류(V2.26)는 하지 않는다
            return
        if area == 0 or inside:
            return release("구역 밖이거나 이미 점유 중")

        pts = [(p.pose.position.x, p.pose.position.y) for p in me.truncated_path.poses]
        if len(pts) < 2:
            return release("경로 없음")
        # 출구 지점 = 경로를 따라 probe_m 앞
        acc = 0.0
        exit_pt = pts[-1]
        for a, b in zip(pts, pts[1:]):
            acc += math.hypot(b[0] - a[0], b[1] - a[1])
            if acc >= self.junction_exit_probe_m:
                exit_pt = b
                break
        blocker = None
        for m, ag in self._cached_agents.items():
            if int(m) == int(self.my_id) or self._agent_is_moving(ag, now):
                continue
            q = ag.current_pose.pose.position
            if math.hypot(q.x - exit_pt[0], q.y - exit_pt[1]) <= self.junction_exit_clear_m:
                blocker = int(m)
                break
        if blocker is None:
            return release("출구가 비었다")
        if self._jx_hold and self._jx_since is not None \
                and (now - self._jx_since).nanoseconds * 1e-9 > self.junction_exit_max_hold:
            return release(f"보류 {self.junction_exit_max_hold:.0f}s 초과 — 더 기다리지 않는다")
        if not self._jx_hold:
            self._jx_hold = True
            self._jx_since = now
            self._publish_pause()
            self._publish_state(f"[V2 junction-exit] HOLD (구역 {area}, 출구에 로봇 {blocker})")
            self.get_logger().warn(
                f"[V2 junction-exit] 구역 {area} 출구({exit_pt[0]:.1f},{exit_pt[1]:.1f})에 로봇 {blocker} 정지 "
                f"→ 교차로에 들어가지 않고 앞에서 기다린다")

    def _v2_standoff_tick(self) -> None:
        """[V2.7] 2대 정면 대치: 둘 다 제자리 + 마주 봄 + 가까움 → 우선순위 낮은 쪽이 물러난다."""
        if self._op_frozen():
            return                                    # [OP] 관제 pause 중 — 판단 정지
        if not (self.v2_enable and self.standoff_enable):
            return
        now = self.get_clock().now()
        if self._standoff_active:
            if self._backup_state in ("succeeded", "failed"):
                ok = self._backup_state == "succeeded"
                self._backup_state = "idle"
                self._standoff_active = False
                self.pub_cmd_resume.publish(Bool(data=False))
                if ok:
                    self.get_logger().warn("[V2 standoff] 대치 해소 후퇴 성공")
                else:
                    self.get_logger().error("[V2 standoff] 대치 해소 후퇴 실패")
                self._publish_state(f"[V2 standoff] {'DONE' if ok else 'FAILED'}")
            return
        if self._backup_state != "idle" or self.nav_stop_complete_ is False or self._yield_active or self._unwedge_active:
            return
        if self._stuck_sec(now) < self.standoff_after_sec or self._near_own_goal():
            self._standoff_sig = None; self._standoff_since = None
            return                                     # [V2.10] 완주 직전이면 대치 후퇴도 미룬다
        turn = self._line_my_turn(now)
        if turn is False:
            self.get_logger().info("[V2 line] 줄 해소 순서가 아니다 — 이번엔 기동하지 않는다",
                                   throttle_duration_sec=20.0)
            return
        mid = self._standoff_partner(now)
        if mid is None:
            if self._standoff_sig is not None:
                self.get_logger().info(f"[V2 standoff] 대치 해소됨 (상대 {self._standoff_sig})")
            self._standoff_sig = None; self._standoff_since = None
            return
        if mid != self._standoff_sig:
            self._standoff_sig = mid; self._standoff_since = now
            a = self._cached_agents.get(mid)
            mine = self._i_have_priority(a, now) if a is not None else False
            self.get_logger().warn(
                f"[V2 standoff] 정면 대치 검출: 상대 {mid}, 둘 다 {self._stuck_sec(now):.0f}s 제자리 "
                f"({'내가 우선 → 상대가 물러난다' if mine else '내가 후순위 → 내가 물러난다'})")
            self._publish_state(f"[V2 standoff] detected {mid}")
            return
        a = self._cached_agents.get(mid)
        if a is None:
            return
        if self._i_have_priority(a, now):
            return                                        # 우선권자는 버틴다 (양쪽이 동시에 물러나면 또 막힌다)
        if self._standoff_last is not None and \
                (now - self._standoff_last).nanoseconds * 1e-9 < self.standoff_cooldown_sec:
            return
        p = a.current_pose.pose.position
        if not self._op_may('_v2_standoff_tick'):      # [OP] 불허면 pause·공지·쿨다운 없이 넘긴다
            return
        self._standoff_last = now
        self.get_logger().warn(
            f"[V2 standoff] 상대 {mid} 와 {self.standoff_after_sec:.0f}s 넘게 맞섰다 → "
            f"{self.standoff_retreat_m} m 물러난다")
        self._publish_state(f"[V2 standoff] RETREAT {self.standoff_retreat_m} m (vs {mid})")
        # [09-23 버그] 여기서 무조건 pause 를 걸고 BT 에게 후퇴 주행을 시켰다 — **컨트롤러가 멈춘 채라 움직일 수 없다.**
        # 실측: 후퇴 지시 138건 중 134건(97 %)에서 `/cmd_vel` 이 한 번도 안 나갔고, 지시 순간 `controller_server: pause_callback` 이 찍혔다.
        # 밀어내기 양보(_v2_push_tick) 에는 같은 조건이 이미 있었는데 여기에만 빠져 있었다.
        # BT 후퇴 모드가 아닐 때(직접 기동)는 behavior_server 가 몰기 때문에 pause 가 필요하다.
        if not self._yield_bt_ready(now):
            self._publish_pause()
        if self._retreat_along_history(now, self.standoff_retreat_m, (p.x, p.y)):
            self._standoff_active = True
        else:                                         # 후퇴를 못 걸었으면 내가 건 pause 를 되돌린다
            self.get_logger().error("[V2 standoff] 후퇴 기동을 못 걸었다 → 대기 해제, 자기 구출에 맡긴다")
            self.pub_cmd_resume.publish(Bool(data=False))

    def _check_no_progress(self) -> None:
        """[V2] 위치 무진전 보고: DRIVING 계열인데 no_progress_dist_m 안에 no_progress_report_sec 동안
        머물면 관제에 알린다 (STOP → driving_abort). 관제 goal_timeout(600 s) 보다 먼저."""
        if self._op_frozen():
            return                                    # [OP] 관제 pause 중 — 판단 정지
        if not self.v2_enable or self.no_progress_report_sec <= 0.0:
            return
        now = self.get_clock().now()
        if self.current_robot_status not in ('RECEIVED_GOAL', 'PLANNING', 'DRIVING', 'PAUSED',
                                             'RECOVERY_FAILURE', 'RECOVERY_RUNNING', 'RECOVERY_SUCCESS') \
                or self.nav_stop_complete_ is False:
            self._np_anchor = None
            return
        me = self._own_pose()
        if me is None:
            return
        if self._np_anchor is None or math.hypot(me[0] - self._np_anchor[0], me[1] - self._np_anchor[1]) > self.no_progress_dist:
            self._np_anchor = (me[0], me[1])
            self._np_anchor_t = now
            return
        dt = (now - self._np_anchor_t).nanoseconds * 1e-9
        if dt >= self.no_progress_report_sec:
            self.get_logger().error(
                f"[V2 no-progress] {dt:.0f}s 동안 {self.no_progress_dist} m 안 (status {self.current_robot_status}, "
                f"agent SM {self.is_processing_agent_pause}/{self.current_agent_stop_type.name}, "
                f"static SM {self.is_processing_replan_pause}). 관제에 보고한다 (STOP).")
            self._publish_state(f"ABORT (no progress {dt:.0f}s)")
            self.pub_cmd_stop.publish(UInt8(data=1))
            self.nav_stop_complete_ = False
            self._nav_stop_wait_start = now
            if self._speed_limited:
                self._v2_restore_speed("no progress abort")
            self._np_anchor = None
            self._static_reset_osc()
            self._agent_reset_osc()
            self.is_processing_agent_pause = False
            self._agent_pause_start_time = None
            self.is_processing_replan_pause = False
            self._pause_start_time = None

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
