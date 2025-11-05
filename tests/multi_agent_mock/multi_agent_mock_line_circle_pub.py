#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point32
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import Path

from multi_agent_msgs.msg import MultiAgentInfoArray, MultiAgentInfo, AgentStatus

FRAME_MAP = "map"         # 월드 좌표
FOOTPRINT_FRAME = "base_link"  # 로봇 로컬 좌표
PUB_TOPIC = "/multi_agent_infos"
HZ = 10.0
MAX_POSES = 20

def rect_footprint(w: float, l: float) -> PolygonStamped:
    """
    로봇 로컬 프레임(FOOTPRINT_FRAME) 기준 직사각형 footprint (w:폭, l:길이)
    점 순서는 REP-103 기준 CCW (x:전방, y:좌측)
    """
    fp = PolygonStamped()
    fp.header.frame_id = FOOTPRINT_FRAME
    hw, hl = w * 0.5, l * 0.5
    # CCW: (앞-좌) -> (뒤-좌) -> (뒤-우) -> (앞-우)
    fp.polygon.points = [
        Point32(x= hl, y= hw, z=0.0),   # front-left
        Point32(x=-hl, y= hw, z=0.0),   # rear-left
        Point32(x=-hl, y=-hw, z=0.0),   # rear-right
        Point32(x= hl, y=-hw, z=0.0),   # front-right
    ]
    return fp

def yaw_wrap(yaw: float) -> float:
    # wrap to (-pi, pi]
    return (yaw + math.pi) % (2.0 * math.pi) - math.pi

def pose_xyth(x: float, y: float, th: float) -> Pose:
    p = Pose()
    p.position.x = x
    p.position.y = y
    th = yaw_wrap(th)
    c = math.cos(th * 0.5)
    s = math.sin(th * 0.5)
    p.orientation.z = s
    p.orientation.w = c
    return p

def make_path_with_stamps(start_stamp, poses: List[Pose], dt: float) -> Path:
    path = Path()
    path.header.frame_id = FRAME_MAP
    path.header.stamp = start_stamp
    base = start_stamp.sec + start_stamp.nanosec * 1e-9
    for i, po in enumerate(poses):
        ts = base + i * dt
        sec = int(ts)
        nsec = int((ts - sec) * 1e9)
        ps = PoseStamped()
        ps.header.frame_id = FRAME_MAP
        ps.header.stamp.sec = sec
        ps.header.stamp.nanosec = nsec
        ps.pose = po
        path.poses.append(ps)
    return path


class MultiAgentMockLineCirclePub(Node):
    def __init__(self):
        super().__init__("multi_agent_mock_pub")
        self.pub = self.create_publisher(MultiAgentInfoArray, PUB_TOPIC, 10)
        self.t = 0.0
        self.dt = 1.0 / HZ
        self.timer = self.create_timer(self.dt, self.on_timer)

        # 공통 footprint (로컬 프레임)
        self.fp = rect_footprint(0.50, 0.70)

        # IDs
        self.agentA_id = 101  # 정지
        self.agentB_id = 202  # 직선 왕복
        self.agentC_id = 303  # 반원 왕복
        self.type_id = "amr"

        # --- B: 직선 왕복 파라미터 ---
        self.A_pt = (-2.0, 0.0)
        self.B_pt = (0.0, 0.0)
        self.v = 0.25  # m/s
        ax, ay = self.A_pt
        bx, by = self.B_pt
        dx, dy = (bx - ax), (by - ay)
        self.L = math.hypot(dx, dy) if (dx or dy) else 1e-6
        self.ux, self.uy = (dx / self.L, dy / self.L)

        # --- C: 반원(π rad) 왕복 파라미터 ---
        self.cx, self.cy = (1.0, 1.0)
        self.R_c = 1.0
        self.theta0 = -math.pi / 2.0   # 시작각
        self.v_c = 0.20                # m/s
        self.arc_len = math.pi * self.R_c  # 반원 호길이

        self.get_logger().info(f"Publishing {PUB_TOPIC} at {HZ:.1f} Hz")

    @staticmethod
    def _wrap_bounce(s: float, L: float) -> Tuple[float, int]:
        """
        왕복(삼각파) 변환: s 누적거리 → (구간내 s', 진행방향)
        direction: +1 정방향, -1 역방향
        """
        if L <= 0.0:
            return 0.0, 1
        period = 2.0 * L
        m = s % period
        if m <= L:
            return m, +1
        else:
            return (2.0 * L - m), -1

    # ----- B: 직선 왕복 -----
    def _pose_on_line(self, t: float) -> Pose:
        s = max(0.0, self.v * t)
        s_prime, direction = self._wrap_bounce(s, self.L)
        x = self.A_pt[0] + self.ux * s_prime
        y = self.A_pt[1] + self.uy * s_prime
        yaw = math.atan2(self.uy, self.ux) if direction >= 0 else math.atan2(-self.uy, -self.ux)
        return pose_xyth(x, y, yaw)

    # ----- C: 반원 왕복 -----
    def _pose_on_semicircle(self, t: float) -> Pose:
        """
        [theta0, theta0+π] 구간을 v_c로 왕복.
        정방향: theta0 → theta0+π, 역방향: theta0+π → theta0
        """
        s = max(0.0, self.v_c * t)
        s_prime, direction = self._wrap_bounce(s, self.arc_len)  # [0, πR]
        dtheta = s_prime / self.R_c  # [0, π]

        # 방향에 따라 theta 증가/감소
        theta = (self.theta0 + dtheta) if direction >= 0 else (self.theta0 + math.pi - dtheta)

        # 위치
        x = self.cx + self.R_c * math.cos(theta)
        y = self.cy + self.R_c * math.sin(theta)

        # 접선 헤딩
        yaw = yaw_wrap(theta + direction * (math.pi / 2.0))
        return pose_xyth(x, y, yaw)

    def on_timer(self):
        now = self.get_clock().now().to_msg()

        # --- Agent A: 정지 ---
        aA = MultiAgentInfo()
        aA.machine_id = self.agentA_id
        aA.type_id = self.type_id
        aA.mode = "auto"
        aA.pos_std_m = 0.03
        aA.footprint = self.fp   # 로컬 프레임(base_link)
        aA.status = AgentStatus()
        aA.status.phase = AgentStatus.STATUS_WAITING

        A_pose = pose_xyth(1.0, 0.0, 0.0)
        aA.current_pose = PoseStamped()
        aA.current_pose.header.stamp = now
        aA.current_pose.header.frame_id = FRAME_MAP
        aA.current_pose.pose = A_pose
        aA.truncated_path = make_path_with_stamps(now, [A_pose] * MAX_POSES, self.dt)

        # --- Agent B: 직선 왕복 ---
        aB = MultiAgentInfo()
        aB.machine_id = self.agentB_id
        aB.type_id = self.type_id
        aB.mode = "auto"
        aB.pos_std_m = 0.05
        aB.footprint = self.fp   # 로컬 프레임(base_link)
        aB.status = AgentStatus()
        aB.status.phase = AgentStatus.STATUS_MOVING

        path_poses_b = [self._pose_on_line(self.t + k * self.dt) for k in range(MAX_POSES)]
        aB.truncated_path = make_path_with_stamps(now, path_poses_b, self.dt)

        aB.current_pose = PoseStamped()
        aB.current_pose.header.stamp = now
        aB.current_pose.header.frame_id = FRAME_MAP
        aB.current_pose.pose = path_poses_b[0]

        # --- Agent C: 반원 왕복 ---
        aC = MultiAgentInfo()
        aC.machine_id = self.agentC_id
        aC.type_id = self.type_id
        aC.mode = "auto"
        aC.pos_std_m = 0.05
        aC.footprint = self.fp   # 로컬 프레임(base_link)
        aC.status = AgentStatus()
        aC.status.phase = AgentStatus.STATUS_MOVING

        path_poses_c = [self._pose_on_semicircle(self.t + k * self.dt) for k in range(MAX_POSES)]
        aC.truncated_path = make_path_with_stamps(now, path_poses_c, self.dt)

        aC.current_pose = PoseStamped()
        aC.current_pose.header.stamp = now
        aC.current_pose.header.frame_id = FRAME_MAP
        aC.current_pose.pose = path_poses_c[0]

        # --- Array ---
        arr = MultiAgentInfoArray()
        arr.header.stamp = now
        arr.header.frame_id = FRAME_MAP
        arr.agents = [aA, aB, aC]

        self.pub.publish(arr)
        self.t += self.dt


def main():
    rclpy.init()
    node = MultiAgentMockLineCirclePub()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
