#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose
from multi_agent_msgs.msg import PathAgentCollisionInfo
from multi_agent_msgs.msg import MultiAgentInfoArray, MultiAgentInfo, AgentStatus

from enum import IntEnum
from typing import Optional, Dict, Tuple, List

# ... (Enum, WaitManager 클래스는 기존과 동일하므로 생략) ...
# (MovingCommand, MovingStopType, RerouteStatus, WaitManager 그대로 사용)

class MovingCommand(IntEnum):
    WAIT = 0
    WAIT_DETECT_AMR = 1
    WAIT_OHTHER_AMR = 2
    WAIT_ABNORMAL = 3
    REROUTE = 4

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

class WaitManager:
    def __init__(self, clock):
        self.clock = clock
        self.start_time: Optional[Time] = None
        self.wait_duration_sec: float = 0.0
        self.is_waiting: bool = False
        self.triggered_type: MovingStopType = MovingStopType.TYPE_NONE

    def start_wait(self, duration_sec: float, stop_type: MovingStopType):
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
        self.declare_parameter("collision_msg_timeout_sec", 3.0) 
        
        # [수정] Reroute 요청 쿨다운 시간 설정 (예: 2초 동안 재요청 금지)
        self.declare_parameter("reroute_cooldown_sec", 2.0)

        # Topics
        self.declare_parameter("topic_collision", "/path_agent_collision_info")
        self.declare_parameter("topic_agents", "/multi_agent_infos")
        self.declare_parameter("topic_replan_flag", "/replan_flag")

        # Output Topics
        self.declare_parameter("topic_decision_state", "/decision_state")
        self.declare_parameter("topic_request_replan", "/request_replan")
        self.declare_parameter("topic_request_reroute", "/request_reroute")
        self.declare_parameter("topic_cmd_run", "/cmd/run")
        self.declare_parameter("topic_cmd_resume", "/cmd/resume")
        self.declare_parameter("topic_cmd_stop", "/cmd/stop")

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

        # Internal State
        self._is_reroute_status = RerouteStatus.NONE
        self._pre_moving_stop_type = MovingStopType.TYPE_NONE
        self._last_agent_event_time: Optional[Time] = None
        self._cached_agents: Dict[int, MultiAgentInfo] = {}
        
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
        
        self.pub_req_reroute = self.create_publisher(Bool, 
            self.get_parameter("topic_request_reroute").value, qos_req) # [권장] qos_req 사용        

        self.pub_state = self.create_publisher(String, 
            self.get_parameter("topic_decision_state").value, 10)
        
        self.pub_cmd_run = self.create_publisher(Bool, 
            self.get_parameter("topic_cmd_run").value, 10)

        self.pub_cmd_resume = self.create_publisher(Bool, 
            self.get_parameter("topic_cmd_resume").value, 10)
        
        self.pub_cmd_stop = self.create_publisher(Bool, 
            self.get_parameter("topic_cmd_stop").value, 10)

        self.create_timer(0.1, self.check_collision_timeout)

        self.get_logger().info(f"FleetDecisionNode (Reroute Cooldown: {self.reroute_cooldown_sec}s). ID: {self.my_id}")

    # ... (check_collision_timeout, on_agents, on_replan_flag, on_collision 등 기존 로직 동일) ...
    # ------------------------------------------------------------------
    # Watchdog Timer Callback
    # ------------------------------------------------------------------
    def check_collision_timeout(self):
        if self._last_collision_msg_time is None:
            if self._pre_moving_stop_type != MovingStopType.TYPE_NONE:
                 self._handle_no_obstacle()
            return

        now = self.get_clock().now()
        dt = (now - self._last_collision_msg_time).nanoseconds * 1e-9

        if dt > self.collision_msg_timeout:
            self._handle_no_obstacle()

    def on_agents(self, msg: MultiAgentInfoArray):
        self._cached_agents = {a.machine_id: a for a in msg.agents}
        if self.my_id in self._cached_agents:
            me = self._cached_agents[self.my_id]
            if me.reroute: self._is_reroute_status = RerouteStatus.EXECUTE
            else: self._is_reroute_status = RerouteStatus.NONE

    def on_replan_flag(self, msg: Bool):
        if not msg.data: return
        now = self.get_clock().now()
        if self._last_agent_event_time is not None:
            dt = (now - self._last_agent_event_time).nanoseconds * 1e-9
            if dt < self.replan_ignore_sec: return
        self.get_logger().warn("External replan_flag -> REPLAN")
        self._publish_replan()
        self._publish_state("REPLAN (Flag)")

    def on_collision(self, msg: PathAgentCollisionInfo):
        self._last_collision_msg_time = self.get_clock().now()
        
        if not msg.x: 
            self._handle_no_obstacle()
            return

        self._last_agent_event_time = self.get_clock().now()

        if not msg.machine_id:
            self._execute_command(MovingCommand.WAIT, MovingStopType.TYPE_11)
            return

        target_id = int(msg.machine_id[0])
        collision_x = float(msg.x[0])
        collision_y = float(msg.y[0])

        if target_id == self.my_id:
            self._handle_no_obstacle()
            return
        
        if target_id == 0:
            self._execute_command(MovingCommand.WAIT, MovingStopType.TYPE_11)
            return

        command, stop_type = self._decide_obstacle_action(target_id, (collision_x, collision_y))
        self._execute_command(command, stop_type)

    # ... (Core Logic & Helper Functions 변경 없음) ...
    def _decide_obstacle_action(self, target_id: int, collision_xy: Tuple[float, float]) -> Tuple[MovingCommand, MovingStopType]:
        n_check_complete = MovingCommand.WAIT
        moving_stop_type = MovingStopType.TYPE_NONE
        if self.use_reroute and self._is_reroute_status == RerouteStatus.PREPARE:
            self._is_reroute_status = RerouteStatus.EXECUTE
        agent = self._cached_agents.get(target_id)
        if agent is None: return MovingCommand.WAIT, MovingStopType.TYPE_11

        if self._check_vehicle_manual_mode(agent):
            n_check_complete = MovingCommand.WAIT_DETECT_AMR; moving_stop_type = MovingStopType.TYPE_1
        else:
            if self._check_vehicle_path(agent) == SAME_PATH:
                if self._check_vehicle_status(agent):
                    n_check_complete = MovingCommand.WAIT_DETECT_AMR; moving_stop_type = MovingStopType.TYPE_12
                else:
                    n_check_complete = MovingCommand.WAIT_ABNORMAL; moving_stop_type = MovingStopType.TYPE_2
            else:
                if self.use_reroute:
                    if self._check_vehicle_rerouting(agent) == RerouteStatus.EXECUTE:
                        if self._check_vehicle_rerouting_path(agent, collision_xy):
                            n_check_complete = MovingCommand.REROUTE; moving_stop_type = MovingStopType.TYPE_3
                        else:
                            if self._is_reroute_status != RerouteStatus.NONE:
                                if target_id < self.my_id: n_check_complete = MovingCommand.WAIT_OHTHER_AMR; moving_stop_type = MovingStopType.TYPE_5
                                else: n_check_complete = MovingCommand.WAIT_DETECT_AMR; moving_stop_type = MovingStopType.TYPE_6
                            else:
                                n_check_complete = MovingCommand.WAIT_DETECT_AMR; moving_stop_type = MovingStopType.TYPE_7
                    else:
                        if self._is_reroute_status != RerouteStatus.NONE:
                             n_check_complete = MovingCommand.WAIT_ABNORMAL; moving_stop_type = MovingStopType.TYPE_8
                        else:
                            if target_id < self.my_id: n_check_complete = MovingCommand.WAIT_OHTHER_AMR; moving_stop_type = MovingStopType.TYPE_9
                            else: n_check_complete = MovingCommand.WAIT_DETECT_AMR; moving_stop_type = MovingStopType.TYPE_10
                else:
                    if target_id < self.my_id: n_check_complete = MovingCommand.WAIT_OHTHER_AMR; moving_stop_type = MovingStopType.TYPE_5
                    else: n_check_complete = MovingCommand.WAIT_DETECT_AMR; moving_stop_type = MovingStopType.TYPE_6
        return n_check_complete, moving_stop_type

    def _check_vehicle_manual_mode(self, agent: MultiAgentInfo) -> bool:
        if agent.mode.lower() == "manual": return True
        if agent.status.phase in [AgentStatus.STATUS_MANUAL_RUNNING, AgentStatus.STATUS_MANUAL_COMPLETE]: return True
        return False

    def _check_vehicle_path(self, agent: MultiAgentInfo) -> int:
        if self.my_id not in self._cached_agents: return DIFFERENT_PATH
        me = self._cached_agents[self.my_id]
        my_vec = self._get_path_vector(me.truncated_path.poses)
        other_vec = self._get_path_vector(agent.truncated_path.poses)
        if my_vec and other_vec:
            dot = my_vec[0]*other_vec[0] + my_vec[1]*other_vec[1]
            if dot > 0.707: return SAME_PATH
            else: return DIFFERENT_PATH
        my_yaw = self._get_yaw(me.current_pose.pose)
        other_yaw = self._get_yaw(agent.current_pose.pose)
        diff = abs(math.degrees(self._ang_wrap(my_yaw - other_yaw)))
        if diff < 45.0: return SAME_PATH
        return DIFFERENT_PATH

    def _get_path_vector(self, poses) -> Optional[Tuple[float, float]]:
        if not poses or len(poses) < 2: return None
        start = poses[0].pose.position; end = poses[-1].pose.position
        dx = end.x - start.x; dy = end.y - start.y
        norm = math.hypot(dx, dy)
        if norm < 0.1: return None
        return (dx/norm, dy/norm)

    def _check_vehicle_status(self, agent: MultiAgentInfo) -> bool:
        moving_phases = [AgentStatus.STATUS_MOVING, AgentStatus.STATUS_PATH_SEARCHING]
        if agent.status.phase in moving_phases:
            lin_v = abs(agent.current_twist.linear.x)
            ang_v = abs(agent.current_twist.angular.z)
            if lin_v < 0.01 and ang_v < 0.01: return True 
            return False 
        return True 

    def _check_vehicle_rerouting(self, agent: MultiAgentInfo) -> int:
        if agent.reroute: return RerouteStatus.EXECUTE
        return RerouteStatus.NONE

    def _check_vehicle_rerouting_path(self, agent: MultiAgentInfo, collision_xy: Tuple[float, float]) -> bool:
        if not agent.reroute: return False
        poses = agent.truncated_path.poses
        if not poses: return False 
        cx, cy = collision_xy
        THRESHOLD = 0.5 
        min_dist = float('inf')
        for pose in poses:
            px = pose.pose.position.x; py = pose.pose.position.y
            dist = math.hypot(cx - px, cy - py)
            if dist < min_dist: min_dist = dist
        return min_dist < THRESHOLD

    # ------------------------------------------------------------------
    # Execution (수정됨: Reroute 쿨다운 적용)
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
            should_publish = True
            
            if self._last_reroute_req_time is not None:
                dt = (now - self._last_reroute_req_time).nanoseconds * 1e-9
                if dt < self.reroute_cooldown_sec:
                    should_publish = False # 쿨타임 중이면 발행 생략
            
            if should_publish:
                # 1. Reroute 요청
                self._publish_reroute()
                self._last_reroute_req_time = now # 시간 갱신
                self.get_logger().warn("Request REROUTE sent.")
            else:
                # 쿨타임 중일 때는 로그 정도만 (선택 사항)
                # self.get_logger().debug("Reroute skipped due to cooldown.")
                pass

            # 2. Resume 요청 (무조건 발행 - BT Stop 해제용)
            # Reroute 토픽은 아껴도 Resume 토픽은 계속 보내줘야 로봇이 멈추지 않음
            self.pub_cmd_resume.publish(Bool(data=True))
            
            self._publish_state(state_str + " -> REROUTE & RESUME")
            return

        # ... (이하 WAIT 로직 기존 동일) ...
        wait_time = 0.0
        if command == MovingCommand.WAIT_DETECT_AMR: wait_time = self.wait_detect_sec
        elif command == MovingCommand.WAIT_OHTHER_AMR: wait_time = self.wait_other_long_sec if self.use_reroute else self.wait_other_short_sec
        elif command == MovingCommand.WAIT_ABNORMAL: wait_time = self.wait_abnormal_long_sec if self.use_reroute else self.wait_abnormal_short_sec
        elif command == MovingCommand.WAIT: wait_time = self.wait_obstacle_sec

        if not self.wait_manager.is_waiting:
            self.wait_manager.start_wait(wait_time, stop_type)
            self._publish_stop()
            self._publish_state(state_str + f" -> WAIT({wait_time}s)")
        else:
            if self.wait_manager.check_finished():
                self.get_logger().warn(f"Wait finished for {stop_type.name}. Requesting Replan.")
                self.wait_manager.reset()
                self._publish_replan()
                self.pub_cmd_resume.publish(Bool(data=True))
                self._publish_state(state_str + " -> TIMEOUT REPLAN & RESUME")
            else:
                self._publish_stop()

    def _handle_no_obstacle(self):
        if self._pre_moving_stop_type != MovingStopType.TYPE_NONE:
            self.get_logger().info("[Decision] Clear -> RESUME")
            self.pub_cmd_resume.publish(Bool(data=True)) 
        else:
            self.pub_cmd_run.publish(Bool(data=True))    
        
        self._pre_moving_stop_type = MovingStopType.TYPE_NONE
        self.wait_manager.reset()
        self._publish_state("RUN")

    # ------------------------------------------------------------------
    # Pub/Sub Utils
    # ------------------------------------------------------------------
    def _publish_stop(self): self.pub_cmd_stop.publish(Bool(data=True))
    def _publish_replan(self): self.pub_req_replan.publish(Bool(data=True))
    def _publish_reroute(self): self.pub_req_reroute.publish(Bool(data=True))
    def _publish_state(self, txt: str): self.pub_state.publish(String(data=txt))

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