#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point32
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import Path

# --- multi_agent_msgs (사용자 패키지) ---
from multi_agent_msgs.msg import MultiAgentInfoArray, MultiAgentInfo, AgentStatus

FRAME = "map"
PUB_TOPIC = "/multi_agent_infos"
HZ = 10.0
MAX_POSES = 20

def rect_footprint(w: float, l: float) -> PolygonStamped:
    """centered rectangle footprint (width w, length l) in robot local frame"""
    fp = PolygonStamped()
    fp.header.frame_id = FRAME
    hw, hl = w * 0.5, l * 0.5
    pts = [
        Point32(x= hl, y= hw, z=0.0),
        Point32(x= hl, y=-hw, z=0.0),
        Point32(x=-hl, y=-hw, z=0.0),
        Point32(x=-hl, y= hw, z=0.0),
    ]
    fp.polygon.points = pts
    return fp

def pose_xyth(x: float, y: float, th: float) -> Pose:
    p = Pose()
    p.position.x = x
    p.position.y = y
    c = math.cos(th * 0.5)
    s = math.sin(th * 0.5)
    p.orientation.z = s
    p.orientation.w = c
    return p

def make_path(poses: List[Pose]) -> Path:
    path = Path()
    path.header.frame_id = FRAME
    for po in poses:
        ps = PoseStamped()
        ps.header.frame_id = FRAME
        ps.pose = po
        path.poses.append(ps)
    return path


class MultiAgentMockPub(Node):
    def __init__(self):
        super().__init__("multi_agent_mock_pub")
        self.pub = self.create_publisher(MultiAgentInfoArray, PUB_TOPIC, 10)
        self.t = 0.0
        self.dt = 1.0 / HZ
        self.timer = self.create_timer(self.dt, self.on_timer)

        # 공통 footprint (폭 0.5 m, 길이 0.7 m)
        self.fp = rect_footprint(0.50, 0.70)

        # 고정 에이전트 파라미터
        self.agentA_id = 101  # 정지
        self.agentB_id = 202  # 이동
        self.type_id = "amr"

        # --- Agent B 직선 왕복 파라미터 ---
        # A와 B를 잇는 구간을 일정 속도로 왕복
        self.A_pt = (-2.0, 0.0)
        self.B_pt = (0.0, 0.0)
        self.v = 0.25  # m/s

        # 사전 계산
        ax, ay = self.A_pt
        bx, by = self.B_pt
        dx, dy = (bx - ax), (by - ay)
        self.L = math.hypot(dx, dy) if (dx or dy) else 1e-6
        self.ux, self.uy = (dx / self.L, dy / self.L)  # 단위 벡터

        self.get_logger().info(f"Publishing {PUB_TOPIC} at {HZ:.1f} Hz")

    @staticmethod
    def _wrap_bounce(s: float, L: float) -> Tuple[float, int]:
        """
        왕복(트라이앵글 웨이브) 진행거리로 변환.
        s: 누적 이동거리, L: 구간 길이
        반환: (구간 내 위치거리 s', 진행방향(+1: A->B, -1: B->A))
        """
        if L <= 0.0:
            return 0.0, 1
        period = 2.0 * L
        m = s % period
        if m <= L:
            return m, +1
        else:
            return (2.0 * L - m), -1

    def _pose_on_line(self, t: float) -> Pose:
        """
        시각 t에서의 agent_b 포즈 (위치 + 헤딩, 헤딩은 진행방향 기준)
        """
        # 현재 위치
        s = max(0.0, self.v * t)  # 누적 이동거리
        s_prime, direction = self._wrap_bounce(s, self.L)
        x = self.A_pt[0] + self.ux * s_prime
        y = self.A_pt[1] + self.uy * s_prime

        # 진행방향 헤딩
        if direction >= 0:
            yaw = math.atan2(self.uy, self.ux)  # A->B
        else:
            yaw = math.atan2(-self.uy, -self.ux)  # B->A

        return pose_xyth(x, y, yaw)

    def on_timer(self):
        now = self.get_clock().now().to_msg()

        # --- Agent A: 정지 (대기) ---
        aA = MultiAgentInfo()
        aA.machine_id = self.agentA_id
        aA.type_id = self.type_id
        aA.mode = "auto"
        aA.pos_std_m = 0.03
        aA.footprint = self.fp

        aA.status = AgentStatus()
        aA.status.phase = AgentStatus.STATUS_WAITING

        A_theta = 0.0
        A_pose = pose_xyth(1.0, 0.0, A_theta)
        aA.current_pose = PoseStamped()
        aA.current_pose.header.stamp = now
        aA.current_pose.header.frame_id = FRAME
        aA.current_pose.pose = A_pose

        aA.truncated_path = make_path([A_pose] * MAX_POSES)

        # --- Agent B: 이동 (A<->B 직선 왕복) ---
        aB = MultiAgentInfo()
        aB.machine_id = self.agentB_id
        aB.type_id = self.type_id
        aB.mode = "auto"
        aB.pos_std_m = 0.05
        aB.footprint = self.fp

        aB.status = AgentStatus()
        aB.status.phase = AgentStatus.STATUS_MOVING

        # 미래 경로 샘플
        path_poses = [self._pose_on_line(self.t + k * self.dt) for k in range(MAX_POSES)]
        aB.truncated_path = make_path(path_poses)

        aB.current_pose = PoseStamped()
        aB.current_pose.header.stamp = now
        aB.current_pose.header.frame_id = FRAME
        aB.current_pose.pose = path_poses[0]

        # --- Array 채우기 ---
        arr = MultiAgentInfoArray()
        arr.header.stamp = now
        arr.header.frame_id = FRAME
        arr.agents = [aA, aB]

        self.pub.publish(arr)
        self.t += self.dt


def main():
    rclpy.init()
    node = MultiAgentMockPub()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
