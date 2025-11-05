#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Path

from multi_agent_msgs.msg import PathAgentCollisionInfo
from multi_agent_msgs.msg import MultiAgentInfoArray, MultiAgentInfo, AgentStatus


# ---------------------------
# Small math / utils
# ---------------------------

def ang_wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_of(pose: Pose) -> float:
    qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    # yaw from quaternion
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def dist2(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return math.hypot(dx, dy)


def dot2(ax: float, ay: float, bx: float, by: float) -> float:
    return ax * bx + ay * by


def unit(vx: float, vy: float) -> Tuple[float, float]:
    n = math.hypot(vx, vy)
    if n < 1e-6:
        return 0.0, 0.0
    return vx / n, vy / n


# ---------------------------
# Score helpers
# ---------------------------

def heading_bonus(rel_heading_deg: float) -> float:
    """head-on > crossing > same-lane (positive means more severe)."""
    a = abs(rel_heading_deg)
    if a <= 25.0:
        return 0.1   # same-lane 뒤따름
    if a <= 110.0:
        return 0.4   # 교차
    return 1.0       # 거의 정면


def mode_bonus(mode: str) -> float:
    """manual이면 내가 더 양보(상대 priority 높임)."""
    return 1.0 if mode.strip().lower() == "manual" else 0.0


def id_bonus(my_id: int, other_id: int) -> float:
    """my_id > other_id면 내가 양보쪽으로 가중."""
    return 1.0 if my_id > other_id else -0.2


def right_of_way_score(agent: MultiAgentInfo, my_id: int) -> float:
    s = 0.0
    # corridor/area/occupancy가 있다면 양보받을 이유 +1
    if agent.occupancy:
        s += 1.0
    # 작은 machine_id가 기본 우선
    s += (0.5 if agent.machine_id < my_id else -0.2)
    return s


# ---------------------------
# Main Decision Node
# ---------------------------

@dataclass
class Candidate:
    machine_id: int
    type_id: str
    px: float
    py: float
    T_eff: float
    severity: float
    yprio: float
    score: float
    note: str


class FleetDecisionNode(Node):
    def __init__(self):
        super().__init__("fleet_decision_node")

        # ---- Parameters (defaults reflect prior design) ----
        # Frames & my identity
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("my_machine_id", 10)

        # TTC combine
        self.declare_parameter("w1_ttc", 1.0)    # weight direct TTC
        self.declare_parameter("w2_alt", 0.8)    # weight surrogate TTC
        self.declare_parameter("T_min", 0.5)
        self.declare_parameter("d_min", 0.2)

        # Severity weights
        self.declare_parameter("a1_invT", 1.2)
        self.declare_parameter("a2_invd", 0.2)
        self.declare_parameter("a3_heading", 0.5)
        self.declare_parameter("a4_vclosing", 0.3)

        # Yield weights
        self.declare_parameter("b1_mode", 0.7)
        self.declare_parameter("b2_rowgap", 0.9)
        self.declare_parameter("b3_reroute", 0.8)
        self.declare_parameter("b4_pathsearch", 0.4)
        self.declare_parameter("b5_occupancy", 0.5)
        self.declare_parameter("b6_id", 0.2)
        self.declare_parameter("kappa", 0.6)

        # headings
        self.declare_parameter("same_lane_deg", 25.0)
        self.declare_parameter("cross_lane_deg", 110.0)

        # thresholds / timers
        self.declare_parameter("T_slow", 6.0)
        self.declare_parameter("T_yield", 2.5)
        self.declare_parameter("yield_priority_thresh", 0.8)

        self.declare_parameter("hold_sec", 1.5)
        self.declare_parameter("behavior_min", 0.7)
        self.declare_parameter("release_hys", 0.5)
        self.declare_parameter("deadlock_window", 8.0)

        # speed caps
        self.declare_parameter("v_nom", 0.7)
        self.declare_parameter("v_slow", 0.30)
        self.declare_parameter("v_yield", 0.08)
        self.declare_parameter("w_slow", 0.7)

        # topic names
        self.declare_parameter("topic_collision", "/path_agent_collision_info")
        self.declare_parameter("topic_agents", "/multi_agent_infos")
        self.declare_parameter("topic_replan_flag", "/replan_flag")

        self.declare_parameter("topic_decision_state", "/decision_state")
        self.declare_parameter("topic_decision_speed", "/decision_speed_limit")
        self.declare_parameter("topic_request_replan", "/request_replan")
        self.declare_parameter("topic_request_reroute", "/request_reroute")
        self.declare_parameter("topic_debug", "/decision_debug")


        # __init__ 내 파라미터
        self.declare_parameter("release_idle_sec", 1.5)
        self.declare_parameter("T_yield_enter", 2.5)
        self.declare_parameter("T_yield_exit", 3.5)
        self.declare_parameter("Y_enter", 0.8)
        self.declare_parameter("Y_exit", 0.5)
        self.declare_parameter("clean_tick_sec", 0.2)
        self.declare_parameter("K_clean", 5)




        # ---- get params ----
        self.release_idle_sec = self.get_parameter("release_idle_sec").value
        self.T_yield_enter = self.get_parameter("T_yield_enter").value
        self.T_yield_exit  = self.get_parameter("T_yield_exit").value
        self.Y_enter = self.get_parameter("Y_enter").value
        self.Y_exit  = self.get_parameter("Y_exit").value
        self.clean_tick_sec = self.get_parameter("clean_tick_sec").value
        self.K_clean = int(self.get_parameter("K_clean").value)



        self.global_frame = self.get_parameter("global_frame").get_parameter_value().string_value
        self.my_id = self.get_parameter("my_machine_id").get_parameter_value().integer_value

        self.w1_ttc = self.get_parameter("w1_ttc").get_parameter_value().double_value
        self.w2_alt = self.get_parameter("w2_alt").get_parameter_value().double_value
        self.T_min = self.get_parameter("T_min").get_parameter_value().double_value
        self.d_min = self.get_parameter("d_min").get_parameter_value().double_value

        self.a1 = self.get_parameter("a1_invT").get_parameter_value().double_value
        self.a2 = self.get_parameter("a2_invd").get_parameter_value().double_value
        self.a3 = self.get_parameter("a3_heading").get_parameter_value().double_value
        self.a4 = self.get_parameter("a4_vclosing").get_parameter_value().double_value

        self.b1 = self.get_parameter("b1_mode").get_parameter_value().double_value
        self.b2 = self.get_parameter("b2_rowgap").get_parameter_value().double_value
        self.b3 = self.get_parameter("b3_reroute").get_parameter_value().double_value
        self.b4 = self.get_parameter("b4_pathsearch").get_parameter_value().double_value
        self.b5 = self.get_parameter("b5_occupancy").get_parameter_value().double_value
        self.b6 = self.get_parameter("b6_id").get_parameter_value().double_value
        self.kappa = self.get_parameter("kappa").get_parameter_value().double_value

        self.same_lane_deg = self.get_parameter("same_lane_deg").get_parameter_value().double_value
        self.cross_lane_deg = self.get_parameter("cross_lane_deg").get_parameter_value().double_value

        self.T_slow = self.get_parameter("T_slow").get_parameter_value().double_value
        self.T_yield = self.get_parameter("T_yield").get_parameter_value().double_value
        self.Y_th = self.get_parameter("yield_priority_thresh").get_parameter_value().double_value

        self.hold_sec = self.get_parameter("hold_sec").get_parameter_value().double_value
        self.behavior_min = self.get_parameter("behavior_min").get_parameter_value().double_value
        self.release_hys = self.get_parameter("release_hys").get_parameter_value().double_value
        self.deadlock_window = self.get_parameter("deadlock_window").get_parameter_value().double_value

        self.v_nom = self.get_parameter("v_nom").get_parameter_value().double_value
        self.v_slow = self.get_parameter("v_slow").get_parameter_value().double_value
        self.v_yield = self.get_parameter("v_yield").get_parameter_value().double_value
        self.w_slow = self.get_parameter("w_slow").get_parameter_value().double_value

        self.topic_collision = self.get_parameter("topic_collision").get_parameter_value().string_value
        self.topic_agents = self.get_parameter("topic_agents").get_parameter_value().string_value
        self.topic_replan_flag = self.get_parameter("topic_replan_flag").get_parameter_value().string_value
        self.topic_decision_state = self.get_parameter("topic_decision_state").get_parameter_value().string_value
        self.topic_decision_speed = self.get_parameter("topic_decision_speed").get_parameter_value().string_value
        self.topic_request_replan = self.get_parameter("topic_request_replan").get_parameter_value().string_value
        self.topic_request_reroute = self.get_parameter("topic_request_reroute").get_parameter_value().string_value
        self.topic_debug = self.get_parameter("topic_debug").get_parameter_value().string_value


        # 상태 부가 변수
        self.last_collision_stamp = None
        self.clean_count = 0

        # ---- state machine ----
        self.state = "RUN"
        self.last_state_change = self.get_clock().now()
        self.deadlock_start: Optional[rclpy.time.Time] = None
        self.agent_hold_until: Optional[rclpy.time.Time] = None

        # Agent DB (최신 스냅샷)
        self.last_agents: Optional[MultiAgentInfoArray] = None

        # ---- pubs/subs ----
        self.sub_collision = self.create_subscription(
            PathAgentCollisionInfo, self.topic_collision, self.on_collision, 10
        )
        self.sub_agents = self.create_subscription(
            MultiAgentInfoArray, self.topic_agents, self.on_agents, 10
        )
        self.sub_replan_flag = self.create_subscription(
            Bool, self.topic_replan_flag, self.on_replan_flag, 10
        )

        self.pub_state = self.create_publisher(String, self.topic_decision_state, 10)
        self.pub_speed = self.create_publisher(Twist, self.topic_decision_speed, 10)
        self.pub_req_replan = self.create_publisher(Bool, self.topic_request_replan, 10)
        self.pub_req_reroute = self.create_publisher(Bool, self.topic_request_reroute, 10)
        self.pub_debug = self.create_publisher(String, self.topic_debug, 10)

        # periodic sanity tick (speed cap publish & hysteresis release)
        self.timer = self.create_timer(0.2, self.on_timer)

        self.get_logger().info(f"fleet_decision_node ready. my_id={self.my_id}")

    # --------- Callbacks ---------

    def on_agents(self, msg: MultiAgentInfoArray):
        self.last_agents = msg

    def on_replan_flag(self, msg: Bool):
        # Non-agent flag가 들어왔을 때: agent-hold면 무시
        now = self.get_clock().now()
        if self.agent_hold_until and now < self.agent_hold_until:
            self.get_logger().info("replan_flag ignored due to agent-hold window")
            return

        if msg.data:
            # 안전을 해치지 않도록 최소 지속시간 후 상태 전환
            if self.state in ("RUN", "SLOWDOWN"):
                self.request_replan("external replan_flag")
            else:
                # 이미 YIELD/REPLAN/REROUTE/STOP 중이면 무시하거나 로그만
                self.get_logger().info(f"replan_flag received but state={self.state}; ignored")


    def on_collision(self, msg):
        now = self.get_clock().now()
        self.last_collision_stamp = now
        self.agent_hold_until = now + Duration(seconds=self.hold_sec)

        cands = self.build_candidates(msg)
        if not cands:
            self.release_if_safe("no candidates from collision msg")
            return

        primary = max(cands, key=lambda c: c.score)

        # --- hysteresis 판단 변경 ---
        # 진입/이탈 임계 분리
        if self.state in ("YIELD", "SLOWDOWN"):
            # 이탈 조건
            if primary.T_eff > self.T_yield_exit and primary.yprio < self.Y_exit:
                decision = "RUN"
            else:
                # 기존 로직 유지
                decision = self.decide_with_primary(primary)
        else:
            # 진입 조건
            if primary.T_eff < self.T_yield_enter or primary.yprio >= self.Y_enter:
                decision = "YIELD"
            elif primary.T_eff < self.T_slow:
                decision = "SLOWDOWN"
            else:
                decision = "RUN"

        self.apply_decision(decision, reason=f"primary(mid={primary.machine_id}) score={primary.score:.2f}")


    # def on_collision(self, msg: PathAgentCollisionInfo):
    #     now = self.get_clock().now()
    #     # Agent-hold 갱신: hold_sec 동안은 외부 replan_flag 무시
    #     self.agent_hold_until = now + Duration(seconds=self.hold_sec)

    #     cands = self.build_candidates(msg)
    #     if not cands:
    #         # 충돌 후보가 없으면 완화 시도
    #         self.release_if_safe("no candidates from collision msg")
    #         return

    #     # 우선 대상(최대 score) 선택
    #     primary = max(cands, key=lambda c: c.score)
    #     decision = self.decide_with_primary(primary)

    #     # 상태 및 속도/요청 반영
    #     self.apply_decision(decision, reason=f"primary(mid={primary.machine_id}) score={primary.score:.2f}")

    #     # 디버그 출력
    #     dbg = f"Primary(mid={primary.machine_id},type={primary.type_id}) " \
    #           f"T_eff={primary.T_eff:.2f} sev={primary.severity:.2f} yprio={primary.yprio:.2f} " \
    #           f"score={primary.score:.2f} note={primary.note}"
    #     self.pub_debug.publish(String(data=dbg))

    # --------- Core building blocks ---------

    def build_candidates(self, msg: PathAgentCollisionInfo) -> List[Candidate]:
        if self.last_agents is None or len(msg.x) == 0:
            return []

        # 에이전트 인덱싱: machine_id -> info
        agents_by_id = {a.machine_id: a for a in self.last_agents.agents}

        out: List[Candidate] = []
        for i in range(len(msg.x)):
            px, py = msg.x[i], msg.y[i]
            ttc = msg.ttc_first[i] if i < len(msg.ttc_first) else -1.0
            note = msg.note[i] if i < len(msg.note) else ""

            # 충돌 원인 후보: 메시지엔 source id가 없으므로 "근처/덮치는" 에이전트를 추정
            agent, rel_heading_deg, v_closing = self.match_agent_at_point(px, py, agents_by_id)
            if agent is None or agent.machine_id == self.my_id:
                continue

            # combine TTC (direct + surrogate)
            T_eff = self.combine_ttc(ttc, px, py, agent)

            # severity
            me_pose = self.find_me_pose(agents_by_id)
            d = dist2((me_pose.position.x, me_pose.position.y), (px, py)) if me_pose else 10.0
            sev = self.a1 * (1.0 / max(T_eff, self.T_min)) + \
                  self.a2 * (1.0 / max(d, self.d_min)) + \
                  self.a3 * heading_bonus(rel_heading_deg) + \
                  self.a4 * v_closing

            # yield priority
            row_me = right_of_way_score(agents_by_id.get(self.my_id, agent), self.my_id)
            row_ot = right_of_way_score(agent, self.my_id)
            yprio = self.b1 * mode_bonus(agent.mode) + \
                    self.b2 * (row_ot - row_me) + \
                    self.b3 * (1.0 if agent.reroute else 0.0) + \
                    self.b4 * (1.0 if agent.status.phase == AgentStatus.STATUS_PATH_SEARCHING else 0.0) + \
                    self.b5 * (1.0 if agent.occupancy else 0.0) + \
                    self.b6 * id_bonus(self.my_id, agent.machine_id)

            score = sev * (1.0 + self.kappa * yprio)
            out.append(Candidate(
                machine_id=int(agent.machine_id),
                type_id=agent.type_id,
                px=px, py=py,
                T_eff=T_eff, severity=sev, yprio=yprio, score=score, note=note
            ))

        return out

    def find_me_pose(self, agents_by_id) -> Optional[Pose]:
        me = agents_by_id.get(self.my_id, None)
        if me:
            return me.current_pose.pose
        return None

    def match_agent_at_point(self, px: float, py: float, agents_by_id) -> Tuple[Optional[MultiAgentInfo], float, float]:
        """가장 가까운(또는 footprint 근접) agent를 선택하고 상대Heading/closing speed 근사."""
        best = None
        best_d = 1e9
        rel_heading_deg = 90.0
        v_closing = 0.0

        me = agents_by_id.get(self.my_id, None)
        my_pose = me.current_pose.pose if me else None

        for a in agents_by_id.values():
            if a.machine_id == self.my_id:
                continue
            ax, ay = a.current_pose.pose.position.x, a.current_pose.pose.position.y
            d = dist2((ax, ay), (px, py))
            if d < best_d:
                best = a
                best_d = d

        if best is None:
            return None, 90.0, 0.0

        # 상대 heading과 내 heading 차이
        a_yaw = yaw_of(best.current_pose.pose)
        if my_pose:
            my_yaw = yaw_of(my_pose)
            rel_heading_deg = abs(math.degrees(ang_wrap(my_yaw - a_yaw)))
        else:
            rel_heading_deg = 90.0

        # closing speed surrogate
        ax, ay = best.current_pose.pose.position.x, best.current_pose.pose.position.y
        ux, uy = unit(px - ax, py - ay)  # agent->collision point
        v_other = best.current_twist.linear.x * dot2(math.cos(a_yaw), math.sin(a_yaw), ux, uy)

        v_me = 0.0
        if my_pose:
            my_yaw = yaw_of(my_pose)
            mx, my = my_pose.position.x, my_pose.position.y
            umx, umy = unit(px - mx, py - my)
            # 내 속도 정보는 없을 수도 있으니 conservative 0.0
            v_me = 0.0  # 필요하면 외부에서 /odom or status 확장

        v_closing = max(0.0, v_other + v_me)
        return best, rel_heading_deg, v_closing

    def combine_ttc(self, ttc_direct: float, px: float, py: float, agent: MultiAgentInfo) -> float:
        # direct TTC 신뢰도
        C_ttc = 1.0 if ttc_direct > 0.0 else 0.0

        # surrogate TTC
        ax, ay = agent.current_pose.pose.position.x, agent.current_pose.pose.position.y
        dx, dy = px - ax, py - ay
        d = math.hypot(dx, dy)
        a_yaw = yaw_of(agent.current_pose.pose)
        ux, uy = unit(dx, dy)
        v_other = agent.current_twist.linear.x * dot2(math.cos(a_yaw), math.sin(a_yaw), ux, uy)
        v_closing = max(0.0, v_other)  # 내 속도 미상 가정

        if v_closing < 0.05:
            T_alt = float("inf")
            C_alt = 0.2  # 신뢰 낮음
        else:
            T_alt = d / v_closing
            C_alt = 0.8  # 양호

        num = 0.0
        den = 0.0
        if C_ttc > 0.0:
            num += self.w1_ttc * C_ttc * ttc_direct
            den += self.w1_ttc * C_ttc
        if C_alt > 0.0 and math.isfinite(T_alt):
            num += self.w2_alt * C_alt * T_alt
            den += self.w2_alt * C_alt

        if den <= 1e-6:
            return float("inf")
        return max(num / den, 0.0)

    def decide_with_primary(self, c: Candidate) -> str:
        """RUN/SLOWDOWN/YIELD/REPLAN/REROUTE/STOP 중 선택."""
        # STOP 조건(초근접 + T_eff 매우 짧음)은 필요 시 추가
        if c.T_eff < self.T_yield or c.yprio >= self.Y_th:
            return "YIELD"
        if c.T_eff < self.T_slow:
            return "SLOWDOWN"
        return "RUN"

    def apply_decision(self, decision: str, reason: str):
        now = self.get_clock().now()

        # 최소 지속시간(behavior_min) 유지
        elapsed = (now - self.last_state_change).nanoseconds / 1e9
        can_change = (elapsed >= self.behavior_min) or (decision == self.state)

        if not can_change:
            # 상태 유지, 속도만 refresh
            self.publish_speed_cap(self.state)
            return

        # 상태 전이
        if decision != self.state:
            self.state = decision
            self.last_state_change = now
            self.get_logger().info(f"[STATE] -> {self.state} ({reason})")
            self.pub_state.publish(String(data=self.state))

            # deadlock timer 관리
            if self.state in ("YIELD",):
                if self.deadlock_start is None:
                    self.deadlock_start = now
            else:
                self.deadlock_start = None

        # 속도 발행
        self.publish_speed_cap(self.state)

        # deadlock window 초과 시 Replan/Reroute escalation
        if self.state == "YIELD" and self.deadlock_start is not None:
            dsec = (now - self.deadlock_start).nanoseconds / 1e9
            if dsec >= self.deadlock_window:
                # 우선 Replan 요청
                self.request_replan("deadlock window exceeded")
                # 다음주기에도 계속 deadlock이면 Reroute로 escalate
                # (간단 구현: Replan 요청 후, 다시 deadlock_window 지나면 Reroute)
                self.deadlock_start = now  # 타임스탬프 리셋

        # RUN으로 올라올 때 약한 히스테리시스: release_hys 동안은 replan_flag 무시
        if self.state == "RUN":
            self.agent_hold_until = now + Duration(seconds=self.release_hys)

    def publish_speed_cap(self, state: str):
        tw = Twist()
        if state == "RUN":
            tw.linear.x = self.v_nom
            tw.angular.z = 2.0  # 충분히 크게
        elif state == "SLOWDOWN":
            tw.linear.x = self.v_slow
            tw.angular.z = self.w_slow
        elif state == "YIELD":
            tw.linear.x = self.v_yield
            tw.angular.z = self.w_slow
        elif state in ("REPLAN", "REROUTE"):
            tw.linear.x = 0.0
            tw.angular.z = 0.0
        elif state == "STOP":
            tw.linear.x = 0.0
            tw.angular.z = 0.0
        else:
            tw.linear.x = self.v_nom
            tw.angular.z = 2.0
        self.pub_speed.publish(tw)

    def request_replan(self, why: str):
        if self.state not in ("REPLAN", "REROUTE"):
            self.state = "REPLAN"
            self.last_state_change = self.get_clock().now()
            self.pub_state.publish(String(data=self.state))
            self.get_logger().warn(f"Request REPLAN: {why}")
        self.pub_req_replan.publish(Bool(data=True))
        self.publish_speed_cap(self.state)

    def request_reroute(self, why: str):
        if self.state != "REROUTE":
            self.state = "REROUTE"
            self.last_state_change = self.get_clock().now()
            self.pub_state.publish(String(data=self.state))
            self.get_logger().warn(f"Request REROUTE: {why}")
        self.pub_req_reroute.publish(Bool(data=True))
        self.publish_speed_cap(self.state)

    def release_if_safe(self, why: str):
        # 충돌 감지 없음 → RUN 복귀(히스테리시스 반영)
        now = self.get_clock().now()
        elapsed = (now - self.last_state_change).nanoseconds / 1e9
        if self.state in ("YIELD", "SLOWDOWN") and elapsed >= self.behavior_min:
            self.state = "RUN"
            self.last_state_change = now
            self.pub_state.publish(String(data=self.state))
            self.get_logger().info(f"Release to RUN: {why}")
        self.publish_speed_cap(self.state)

    # # 주기적 점검: 상태 유지, 히스테리시스, deadlock escalate
    # def on_timer(self):
    #     self.publish_speed_cap(self.state)

    #     # deadlock escalate to REROUTE (두 번째 윈도우도 초과)
    #     if self.state == "REPLAN" and self.deadlock_start is not None:
    #         now = self.get_clock().now()
    #         dsec = (now - self.deadlock_start).nanoseconds / 1e9
    #         if dsec >= self.deadlock_window:
    #             self.request_reroute("deadlock persists after replan")
    def on_timer(self):
        self.publish_speed_cap(self.state)

        # Idle-time release + K-consecutive clean
        if self.state in ("YIELD", "SLOWDOWN"):
            now = self.get_clock().now()
            no_collision = (self.last_collision_stamp is None) or \
                        ((now - self.last_collision_stamp).nanoseconds * 1e-9 > self.release_idle_sec)

            if no_collision:
                self.clean_count += 1
            else:
                self.clean_count = 0

            if no_collision and self.clean_count * self.clean_tick_sec >= self.release_idle_sec:
                self.release_if_safe("idle no collision")
                self.clean_count = 0

        # deadlock escalate (기존 로직 유지)
        if self.state == "REPLAN" and self.deadlock_start is not None:
            now = self.get_clock().now()
            dsec = (now - self.deadlock_start).nanoseconds / 1e9
            if dsec >= self.deadlock_window:
                self.request_reroute("deadlock persists after replan")





def main():
    rclpy.init()
    node = FleetDecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()