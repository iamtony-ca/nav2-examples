#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from functools import partial # 파일 최상단에 추가하세요
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from std_msgs.msg import Bool, String, UInt8
from geometry_msgs.msg import Pose
from robot_interfaces.msg import PathAgentCollisionInfo, PathStaticCollisionInfo
from robot_interfaces.msg import MultiAgentInfoArray, MultiAgentInfo, AgentStatus
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


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

        self.cb_group = ReentrantCallbackGroup()

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

        # Internal State
        self._is_reroute_status = RerouteStatus.NONE
        self._pre_moving_stop_type = MovingStopType.TYPE_NONE
        self._last_agent_event_time: Optional[Time] = None
        self._cached_agents: Dict[int, MultiAgentInfo] = {}
        
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
        self.check_static_is_goal_occupied_ = False

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
        self.get_logger().info("====================================================")

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




    def stop_complete_callback(self, msg: Bool):
        self.nav_stop_complete_ = msg.data
        self.get_logger().info(f"STOP sequence complete topic received: {msg.data}", throttle_duration_sec=2.0)


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
                        self._replan_flag_false_start_time = None
                        self._pause_start_time = None
                        self.is_processing_replan_pause = False
                        self.delay_after_replan = False
                        self.delay_after_replan_start_time = None


                

        if self.replan_flag_status is True and not self.is_processing_replan_pause and self.delay_after_replan == False and self.is_processing_goal_occupied_pause is False and self.is_processing_last_goal_occupied_pause is False:
            self.is_processing_replan_pause = True
            self._pause_start_time = now
            self._publish_pause()
            return




    def check_collision_agent(self):
        """ Agent 충돌 예측에 대한 상태 머신 (20Hz 주기 실행) """
        now = self.get_clock().now()


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
                
                self._agent_pause_start_time = None
                self.delay_after_agent_start_time = None
                self.is_processing_agent_pause = False
                self.delay_after_agent_action = False
                self._agent_clear_start_time = None
            return

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
            self._agent_pause_start_time = now
            
            # 대기 시간(N초) 매핑
            n_pause = 0.0
            cmd = self.current_agent_command
            if cmd == MovingCommand.REROUTE: n_pause = 5.0 
            elif cmd == MovingCommand.WAIT_DETECT_AMR: n_pause = self.wait_detect_sec
            elif cmd == MovingCommand.WAIT_OHTHER_AMR: n_pause = self.wait_other_long_sec if self.use_reroute else self.wait_other_short_sec
            elif cmd == MovingCommand.WAIT_ABNORMAL: n_pause = self.wait_abnormal_long_sec if self.use_reroute else self.wait_abnormal_short_sec
            elif cmd == MovingCommand.WAIT: n_pause = self.wait_obstacle_sec
            elif cmd == MovingCommand.WAIT_SIMPLE_REPLAN: n_pause = 3.0            
            elif cmd == MovingCommand.WAIT_SIMPLE_RESUME: n_pause = 6.0
            
            self.agent_pause_timeout_sec = n_pause
            
            self.get_logger().error(f"[check_collision_agent] [Phase 0] Sequence Locked. Starting PAUSE for {n_pause}s.")
            self._publish_pause()
            self._publish_state(f"{self.current_agent_stop_type.name}: PAUSE {n_pause}s")




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


        # 의사결정은 여기서 하지 않고 Raw Data만 갱신
        self.latest_agent_target_id = target_id
        self.latest_agent_collision_xy = (collision_x, collision_y)

        # 상태가 False -> True로 바뀌는 순간이거나, 타겟 ID가 바뀌었을 때 로깅
        if not self.agent_collision_status or self.latest_agent_target_id != target_id:
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
