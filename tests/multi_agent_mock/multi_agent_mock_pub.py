#!/usr/bin/env python3
import math
from typing import List

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
    fp.header.frame_id = FRAME  # frame은 레이어에서 검증 안하지만 맞춰둡니다
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
    # z=0, quaternion from yaw
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

        self.get_logger().info(f"Publishing {PUB_TOPIC} at {HZ:.1f} Hz")

    def on_timer(self):
        now = self.get_clock().now().to_msg()

        # --- Agent A: 정지 (대기) ---
        aA = MultiAgentInfo()
        aA.machine_id = self.agentA_id
        aA.type_id = self.type_id
        aA.mode = "auto"
        aA.pos_std_m = 0.03  # 위치 표준편차 예시
        aA.footprint = self.fp

        aA.status = AgentStatus()
        aA.status.phase = AgentStatus.STATUS_WAITING

        # 고정 위치 (x=1.0, y=0.0)
        A_theta = 0.0
        A_pose = pose_xyth(1.0, 0.0, A_theta)
        aA.current_pose = PoseStamped()
        aA.current_pose.header.stamp = now
        aA.current_pose.header.frame_id = FRAME
        aA.current_pose.pose = A_pose

        aA.truncated_path = make_path([A_pose] * MAX_POSES)  # 정지: 동일 포즈 반복

        # --- Agent B: 이동 (원형 경로) ---
        aB = MultiAgentInfo()
        aB.machine_id = self.agentB_id
        aB.type_id = self.type_id
        aB.mode = "auto"
        aB.pos_std_m = 0.05
        aB.footprint = self.fp

        aB.status = AgentStatus()
        aB.status.phase = AgentStatus.STATUS_MOVING

        # 원 중심 (x=-1.0, y=0.0), 반경 1.0 m, 각속도 0.25 rad/s
        R = 1.0
        w = 0.25
        cx, cy = -1.0, 0.0

        # 경로 포즈들 (미래 진행방향으로 MAX_POSES 샘플)
        path_poses = []
        for k in range(MAX_POSES):
            th = w * (self.t + k * self.dt)
            x = cx + R * math.cos(th)
            y = cy + R * math.sin(th)
            yaw = th + math.pi / 2.0  # 진행방향을 접선방향으로 설정
            path_poses.append(pose_xyth(x, y, yaw))

        aB.truncated_path = make_path(path_poses)
        aB.current_pose = PoseStamped()
        aB.current_pose.header.stamp = now
        aB.current_pose.header.frame_id = FRAME
        aB.current_pose.pose = path_poses[0]  # 현재는 첫 포즈로

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