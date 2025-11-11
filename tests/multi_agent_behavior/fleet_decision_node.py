#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple, Dict

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose
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
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)

def dist2(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(ax - bx, ay - by)

def dot2(ax: float, ay: float, bx: float, by: float) -> float:
    return ax * bx + ay * by

def unit(vx: float, vy: float) -> Tuple[float, float]:
    n = math.hypot(vx, vy)
    if n < 1e-6:
        return (0.0, 0.0)
    return (vx / n, vy / n)

def heading_bonus(rel_heading_deg: float) -> float:
    # head-on > crossing > same-lane (positive means more severe)
    a = abs(rel_heading_deg)
    if a <= 25.0:   # same lane
        return 0.1
    if a <= 110.0:  # crossing
        return 0.4
    return 1.0      # almost head-on

def mode_bonus(mode: str) -> float:
    return 1.0 if (mode or "").strip().lower() == "manual" else 0.0

def id_bonus(my_id: int, other_id: int) -> float:
    return 1.0 if my_id > other_id else -0.2

def right_of_way_score(agent: MultiAgentInfo, my_id: int) -> float:
    s = 0.0
    if agent.occupancy:
        s += 1.0
    s += (0.5 if agent.machine_id < my_id else -0.2)
    return s


# ---------------------------
# Candidate model
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


# ---------------------------
# Event-driven decision node (with resume & timeouts)
# ---------------------------

class FleetDecisionNode(Node):
    def __init__(self):
        super().__init__("fleet_decision_node_ev")

        # ---- Parameters ----
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("my_machine_id", 1)

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

        # thresholds for enter
        self.declare_parameter("T_slow", 6.0)
        self.declare_parameter("T_yield", 2.5)
        self.declare_parameter("yield_priority_thresh", 0.8)

        # resume hysteresis (exit thresholds & K-连续)
        self.declare_parameter("T_resume_slow", 6.5)
        self.declare_parameter("T_resume_yield", 3.5)
        self.declare_parameter("T_resume_stop", 5.0)
        self.declare_parameter("Y_exit", 0.5)
        self.declare_parameter("K_slow_clean", 2)
        self.declare_parameter("K_yield_clean", 3)
        self.declare_parameter("K_stop_clean", 3)

        # idle-based resume
        self.declare_parameter("resume_idle_sec", 1.5)  # no collision for this window -> resume (SLOWDOWN/YIELD only)

        # timeouts (hard cap)
        self.declare_parameter("resume_timeout_slow", 6.0)
        self.declare_parameter("resume_timeout_yield", 10.0)
        self.declare_parameter("resume_timeout_stop", 15.0)

        # debounce / ignore windows
        self.declare_parameter("agent_event_silence_sec", 1.0)         # 동일 agent 재처리 금지 시간
        self.declare_parameter("replan_ignore_sec_after_agent", 0.5)    # agent 이벤트 직후 외부 replan 무시
        self.declare_parameter("run_pulse_silence_sec", 0.5)            # /cmd/run 펄스 디바운스

        # topics (inputs)
        self.declare_parameter("topic_collision", "/path_agent_collision_info")
        self.declare_parameter("topic_agents", "/multi_agent_infos")
        self.declare_parameter("topic_replan_flag", "/replan_flag")

        # topics (outputs)
        self.declare_parameter("topic_decision_state", "/decision_state")
        self.declare_parameter("topic_request_replan", "/request_replan")
        self.declare_parameter("topic_request_reroute", "/request_reroute")

        # state command topics (pulse true)
        self.declare_parameter("topic_cmd_run", "/cmd/run")
        self.declare_parameter("topic_cmd_slowdown", "/cmd/slowdown")
        self.declare_parameter("topic_cmd_yield", "/cmd/yield")
        self.declare_parameter("topic_cmd_stop", "/cmd/stop")

        # ---- get params ----
        self.global_frame = self.get_parameter("global_frame").value
        self.my_id = int(self.get_parameter("my_machine_id").value)

        self.w1_ttc = float(self.get_parameter("w1_ttc").value)
        self.w2_alt = float(self.get_parameter("w2_alt").value)
        self.T_min = float(self.get_parameter("T_min").value)
        self.d_min = float(self.get_parameter("d_min").value)

        self.a1 = float(self.get_parameter("a1_invT").value)
        self.a2 = float(self.get_parameter("a2_invd").value)
        self.a3 = float(self.get_parameter("a3_heading").value)
        self.a4 = float(self.get_parameter("a4_vclosing").value)

        self.b1 = float(self.get_parameter("b1_mode").value)
        self.b2 = float(self.get_parameter("b2_rowgap").value)
        self.b3 = float(self.get_parameter("b3_reroute").value)
        self.b4 = float(self.get_parameter("b4_pathsearch").value)
        self.b5 = float(self.get_parameter("b5_occupancy").value)
        self.b6 = float(self.get_parameter("b6_id").value)
        self.kappa = float(self.get_parameter("kappa").value)

        self.T_slow = float(self.get_parameter("T_slow").value)
        self.T_yield = float(self.get_parameter("T_yield").value)
        self.Y_th = float(self.get_parameter("yield_priority_thresh").value)

        self.T_resume_slow = float(self.get_parameter("T_resume_slow").value)
        self.T_resume_yield = float(self.get_parameter("T_resume_yield").value)
        self.T_resume_stop  = float(self.get_parameter("T_resume_stop").value)
        self.Y_exit = float(self.get_parameter("Y_exit").value)

        self.K_slow_clean  = int(self.get_parameter("K_slow_clean").value)
        self.K_yield_clean = int(self.get_parameter("K_yield_clean").value)
        self.K_stop_clean  = int(self.get_parameter("K_stop_clean").value)

        self.resume_idle_sec = float(self.get_parameter("resume_idle_sec").value)

        self.resume_timeout_slow  = float(self.get_parameter("resume_timeout_slow").value)
        self.resume_timeout_yield = float(self.get_parameter("resume_timeout_yield").value)
        self.resume_timeout_stop  = float(self.get_parameter("resume_timeout_stop").value)

        self.agent_event_silence_sec = float(self.get_parameter("agent_event_silence_sec").value)
        self.replan_ignore_sec_after_agent = float(self.get_parameter("replan_ignore_sec_after_agent").value)
        self.run_pulse_silence_sec = float(self.get_parameter("run_pulse_silence_sec").value)

        self.topic_collision = self.get_parameter("topic_collision").value
        self.topic_agents = self.get_parameter("topic_agents").value
        self.topic_replan_flag = self.get_parameter("topic_replan_flag").value

        self.topic_decision_state = self.get_parameter("topic_decision_state").value
        self.topic_request_replan = self.get_parameter("topic_request_replan").value
        self.topic_request_reroute = self.get_parameter("topic_request_reroute").value

        self.topic_cmd_run = self.get_parameter("topic_cmd_run").value
        self.topic_cmd_slowdown = self.get_parameter("topic_cmd_slowdown").value
        self.topic_cmd_yield = self.get_parameter("topic_cmd_yield").value
        self.topic_cmd_stop = self.get_parameter("topic_cmd_stop").value

        # ---- runtime caches ----
        self.last_agents: Optional[MultiAgentInfoArray] = None
        self.last_agent_event_time: Dict[Tuple[int, str], Time] = {}  # (machine_id, type_id) -> last processed time
        self.last_agent_event_any: Optional[Time] = None              # 마지막 agent 충돌 이벤트 시간

        # state & timers
        self.state: str = "RUN"
        self.last_state_change: Time = self.get_clock().now()
        self.last_run_cmd_time: Optional[Time] = None

        # resume clean streaks
        self.slow_clean = 0
        self.yield_clean = 0
        self.stop_clean = 0

        # ---- pubs/subs ----
        self.sub_agents = self.create_subscription(MultiAgentInfoArray, self.topic_agents, self.on_agents, 10)
        self.sub_collision = self.create_subscription(PathAgentCollisionInfo, self.topic_collision, self.on_collision, 20)
        self.sub_replan_flag = self.create_subscription(Bool, self.topic_replan_flag, self.on_replan_flag, 10)

        self.pub_state = self.create_publisher(String, self.topic_decision_state, 10)
        self.pub_req_replan = self.create_publisher(Bool, self.topic_request_replan, 10)
        self.pub_req_reroute = self.create_publisher(Bool, self.topic_request_reroute, 10)

        self.pub_cmd_run = self.create_publisher(Bool, self.topic_cmd_run, 10)
        self.pub_cmd_slow = self.create_publisher(Bool, self.topic_cmd_slowdown, 10)
        self.pub_cmd_yield = self.create_publisher(Bool, self.topic_cmd_yield, 10)
        self.pub_cmd_stop = self.create_publisher(Bool, self.topic_cmd_stop, 10)

        self.pub_debug = self.create_publisher(String, "/decision_debug", 10)

        # lightweight timer only for timeouts & idle-resume (no heavy logic)
        self.timer = self.create_timer(0.2, self.on_timer)

        self.get_logger().info(f"[event] fleet_decision_node ready. my_id={self.my_id}")

        # 최초 상태(기본 RUN) 공지 + 1회 명령 펄스
        self._set_state_and_emit("RUN", reason="node start")

    # ---------- Subscribers ----------

    def on_agents(self, msg: MultiAgentInfoArray):
        self.last_agents = msg

    def on_replan_flag(self, msg: Bool):
        if not msg.data:
            return
        now = self.get_clock().now()
        # 직전 에이전트 이벤트 직후에는 외부 replan_flag를 무시 (옵션)
        if self.last_agent_event_any is not None:
            dt = (now - self.last_agent_event_any).nanoseconds * 1e-9
            if dt < self.replan_ignore_sec_after_agent:
                self.get_logger().info(f"replan_flag ignored ({dt:.2f}s after agent event)")
                return

        # 이벤트 방식: REPLAN을 1회성 요청
        self.get_logger().warn("external replan_flag -> request REPLAN")
        self._set_state_and_emit("REPLAN", reason="external replan_flag")
        # self.pub_req_replan.publish(Bool(data=True))

    def on_collision(self, msg: PathAgentCollisionInfo):
        now = self.get_clock().now()
        self.last_agent_event_any = now

        cands_all = self.build_candidates(msg)
        if not cands_all:
            # 충돌 후보가 없으면: 상태가 SLOWDOWN/YIELD/STOP이면 clean 카운팅
            self._maybe_resume_on_clean(None)
            return

        # per-agent 디바운스
        cands: List[Candidate] = []
        for c in cands_all:
            key = (c.machine_id, c.type_id)
            t_last = self.last_agent_event_time.get(key, None)
            if t_last is not None:
                dt = (now - t_last).nanoseconds * 1e-9
                if dt < self.agent_event_silence_sec:
                    continue
            cands.append(c)

        if not cands:
            # 모두 디바운스에 걸려도 clean 판정에는 사용 (충돌이 사라진 효과와 유사)
            self._maybe_resume_on_clean(None)
            return

        primary = max(cands, key=lambda c: c.score)
        self.last_agent_event_time[(primary.machine_id, primary.type_id)] = now

        # 현재 state에 따라 진입/유지/복귀 판정
        decision = self.decide_with_primary(primary, primary_is_reroute=self._agent_is_reroute(primary.machine_id))

        # resume 히스테리시스 (현재가 SLOWDOWN/YIELD/STOP이고 안전해졌는지 검사)
        if self._maybe_resume_on_clean(primary):
            return

        # 그 외: 이벤트 상태로 전환
        self._set_state_and_emit(decision,
                                 reason=f"agent(mid={primary.machine_id}) score={primary.score:.2f} T={primary.T_eff:.2f} Y={primary.yprio:.2f}")

    # ---------- Core building blocks ----------

    def _agents_by_id(self):
        if self.last_agents is None:
            return {}
        return {a.machine_id: a for a in self.last_agents.agents}

    def _agent_is_reroute(self, machine_id: int) -> bool:
        agents = self._agents_by_id()
        a = agents.get(machine_id, None)
        return bool(a and a.reroute)


    def build_candidates(self, msg: PathAgentCollisionInfo) -> List[Candidate]:
        if self.last_agents is None or len(msg.x) == 0:
            return []

        agents_by_id = self._agents_by_id()
        me_pose = agents_by_id.get(self.my_id, None).current_pose.pose if self.my_id in agents_by_id else None

        out: List[Candidate] = []
        N = len(msg.x)
        for i in range(N):
            px, py = msg.x[i], msg.y[i]
            ttc = msg.ttc_first[i] if i < len(msg.ttc_first) else -1.0
            note = msg.note[i] if i < len(msg.note) else ""

            # --- 1) ID 기반 매칭 시도 ---
            agent = None
            rel_heading_deg = 90.0
            v_closing = 0.0

            mid = msg.machine_id[i] if i < len(msg.machine_id) else 0
            if mid:
                a = agents_by_id.get(int(mid), None)
                if a is not None:
                    agent = a
                    # 상대 heading / v_closing 계산 (ID 매칭 기준)
                    a_yaw = yaw_of(a.current_pose.pose)
                    if me_pose is not None:
                        rel_heading_deg = abs(math.degrees(ang_wrap(yaw_of(me_pose) - a_yaw)))
                    ux, uy = unit(px - a.current_pose.pose.position.x, py - a.current_pose.pose.position.y)
                    v_other = a.current_twist.linear.x * dot2(math.cos(a_yaw), math.sin(a_yaw), ux, uy)
                    v_closing = max(0.0, v_other)

                    # (선택) 일관성 체크: 너무 멀면 폴백
                    if dist2(a.current_pose.pose.position.x, a.current_pose.pose.position.y, px, py) > 10.0:    # need to check
                        agent = None  # 다시 폴백
                        rel_heading_deg, v_closing = 90.0, 0.0

            # --- 2) 폴백: 근접 매칭 ---
            if agent is None:
                agent, rel_heading_deg, v_closing = self.match_agent_at_point(px, py, agents_by_id)

            if agent is None or agent.machine_id == self.my_id:
                continue

            # 나머지는 종전 로직 그대로
            T_eff = self.combine_ttc(ttc, px, py, agent)
            d_me = dist2(me_pose.position.x, me_pose.position.y, px, py) if me_pose else 10.0

            sev = self.a1 * (1.0 / max(T_eff, self.T_min)) + \
                self.a2 * (1.0 / max(d_me, self.d_min)) + \
                self.a3 * heading_bonus(rel_heading_deg) + \
                self.a4 * v_closing

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


    def match_agent_at_point(self, px: float, py: float, agents_by_id) -> Tuple[Optional[MultiAgentInfo], float, float]:
        best = None
        best_d = 1e9
        for a in agents_by_id.values():
            if a.machine_id == self.my_id:
                continue
            d = dist2(a.current_pose.pose.position.x, a.current_pose.pose.position.y, px, py)
            if d < best_d:
                best = a; best_d = d
        if best is None:
            return None, 90.0, 0.0

        a_yaw = yaw_of(best.current_pose.pose)

        me = agents_by_id.get(self.my_id, None)
        rel_heading_deg = 90.0
        if me is not None:
            my_yaw = yaw_of(me.current_pose.pose)
            rel_heading_deg = abs(math.degrees(ang_wrap(my_yaw - a_yaw)))

        ux, uy = unit(px - best.current_pose.pose.position.x, py - best.current_pose.pose.position.y)
        v_other = best.current_twist.linear.x * dot2(math.cos(a_yaw), math.sin(a_yaw), ux, uy)
        v_closing = max(0.0, v_other)  # 내 속도 미상 가정
        return best, rel_heading_deg, v_closing

    def combine_ttc(self, ttc_direct: float, px: float, py: float, agent: MultiAgentInfo) -> float:
        C_ttc = 1.0 if ttc_direct > 0.0 else 0.0

        ax, ay = agent.current_pose.pose.position.x, agent.current_pose.pose.position.y
        dx, dy = px - ax, py - ay
        d = math.hypot(dx, dy)
        a_yaw = yaw_of(agent.current_pose.pose)
        ux, uy = unit(dx, dy)
        v_other = agent.current_twist.linear.x * dot2(math.cos(a_yaw), math.sin(a_yaw), ux, uy)
        v_closing = max(0.0, v_other)

        if v_closing < 0.05:
            T_alt = float("inf"); C_alt = 0.2
        else:
            T_alt = d / v_closing; C_alt = 0.8

        num = 0.0; den = 0.0
        if C_ttc > 0.0:
            num += self.w1_ttc * C_ttc * max(ttc_direct, 0.0)
            den += self.w1_ttc * C_ttc
        if C_alt > 0.0 and math.isfinite(T_alt):
            num += self.w2_alt * C_alt * T_alt
            den += self.w2_alt * C_alt
        if den <= 1e-6:
            return float("inf")
        return max(num / den, 0.0)

    def decide_with_primary(self, c: Candidate, primary_is_reroute: bool) -> str:
        # 간편 규칙:
        # 1) 매우 위험(T_eff<T_yield or yprio>=Y_th) → YIELD
        # 2) 중간(T_eff<T_slow) → SLOWDOWN
        # 3) 상대가 reroute 상태고 yprio가 매우 크면 → REROUTE(내가 크게 양보)
        # 4) 그 외 → RUN
        if c.T_eff < self.T_yield or c.yprio >= self.Y_th:
            return "YIELD"
        if c.T_eff < self.T_slow:
            return "SLOWDOWN"
        if primary_is_reroute and c.yprio >= (self.Y_th - 0.2):
            return "REROUTE"
        return "RUN"

    # ---------- Resume & publishing ----------

    def _emit_state_pulse(self, state: str):
        # publish state
        self.pub_state.publish(String(data=state))
        # publish one-shot command
        if state == "RUN":
            now = self.get_clock().now()
            if self.last_run_cmd_time is not None:
                dt = (now - self.last_run_cmd_time).nanoseconds * 1e-9
                if dt < self.run_pulse_silence_sec:
                    # debounce: skip frequent run pulses
                    return
            self.last_run_cmd_time = now
            self.pub_cmd_run.publish(Bool(data=True))
        elif state == "SLOWDOWN":
            self.pub_cmd_slow.publish(Bool(data=True))
        elif state == "YIELD":
            self.pub_cmd_yield.publish(Bool(data=True))
        elif state == "STOP":
            self.pub_cmd_stop.publish(Bool(data=True))
        elif state == "REPLAN":
            self.pub_req_replan.publish(Bool(data=True))
        elif state == "REROUTE":
            self.pub_req_reroute.publish(Bool(data=True))

    def _set_state_and_emit(self, state: str, reason: str = ""):
        self.state = state
        self.last_state_change = self.get_clock().now()
        if reason:
            self.get_logger().info(f"[STATE] -> {state} ({reason})")
        else:
            self.get_logger().info(f"[STATE] -> {state}")
        self._emit_state_pulse(state)

    def _maybe_resume_on_clean(self, primary: Optional[Candidate]) -> bool:
        """
        현재 상태가 SLOWDOWN/YIELD/STOP일 때,
        - collision 후보 없음(=primary None) 또는
        - 후보가 있으나 충분히 안전(T_resume_* 이상 & yprio < Y_exit)
        이면 K-연속 충족 시 RUN 펄스. 충족하면 True 반환.
        """
        if self.state not in ("SLOWDOWN", "YIELD", "STOP"):
            # 현재 RUN류면 복귀 검사 불필요
            self.slow_clean = self.yield_clean = self.stop_clean = 0
            return False

        safe = False
        if primary is None:
            # 후보 자체가 없다 → 안전
            safe = True
        else:
            if self.state == "SLOWDOWN":
                safe = (primary.T_eff >= self.T_resume_slow) and (primary.yprio < self.Y_exit)
            elif self.state == "YIELD":
                safe = (primary.T_eff >= self.T_resume_yield) and (primary.yprio < self.Y_exit)
            elif self.state == "STOP":
                safe = (primary.T_eff >= self.T_resume_stop) and (primary.yprio < self.Y_exit)

        if not safe:
            # 안전 기준 미충족 → 해당 상태의 clean 카운터 리셋
            if self.state == "SLOWDOWN": self.slow_clean = 0
            if self.state == "YIELD":    self.yield_clean = 0
            if self.state == "STOP":     self.stop_clean = 0
            return False

        # 안전 기준 충족 → clean 카운팅
        if self.state == "SLOWDOWN":
            self.slow_clean += 1
            if self.slow_clean >= self.K_slow_clean:
                self._set_state_and_emit("RUN", reason="resume clean(SLOWDOWN)")
                self.slow_clean = 0
                self.yield_clean = 0
                self.stop_clean = 0
                return True
        elif self.state == "YIELD":
            self.yield_clean += 1
            if self.yield_clean >= self.K_yield_clean:
                self._set_state_and_emit("RUN", reason="resume clean(YIELD)")
                self.slow_clean = 0
                self.yield_clean = 0
                self.stop_clean = 0
                return True
        elif self.state == "STOP":
            self.stop_clean += 1
            if self.stop_clean >= self.K_stop_clean:
                self._set_state_and_emit("RUN", reason="resume clean(STOP)")
                self.slow_clean = 0
                self.yield_clean = 0
                self.stop_clean = 0
                return True

        return False

    # ---------- Timeout / Idle resume timer ----------

    def on_timer(self):
        now = self.get_clock().now()

        # Idle resume: 최근 충돌 이벤트가 일정 시간 없으면(SLOWDOWN/YIELD일 때만) RUN
        if self.state in ("SLOWDOWN", "YIELD"):
            if self.last_agent_event_any is not None:
                dt_idle = (now - self.last_agent_event_any).nanoseconds * 1e-9
                if dt_idle >= self.resume_idle_sec:
                    self._set_state_and_emit("RUN", reason=f"idle resume ({dt_idle:.2f}s no collisions)")
                    return

        # Hard timeout resume for SLOWDOWN/YIELD/STOP
        dt_state = (now - self.last_state_change).nanoseconds * 1e-9
        if self.state == "SLOWDOWN" and dt_state >= self.resume_timeout_slow:
            self._set_state_and_emit("RUN", reason=f"SLOWDOWN timeout {dt_state:.2f}s")
            return
        if self.state == "YIELD" and dt_state >= self.resume_timeout_yield:
            self._set_state_and_emit("RUN", reason=f"YIELD timeout {dt_state:.2f}s")
            return
        if self.state == "STOP" and dt_state >= self.resume_timeout_stop:
            self._set_state_and_emit("RUN", reason=f"STOP timeout {dt_state:.2f}s")
            return


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
