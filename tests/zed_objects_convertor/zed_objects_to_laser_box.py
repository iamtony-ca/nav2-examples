import math
import array  # 추가: C-style 배열을 만들기 위한 파이썬 표준 라이브러리
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from zed_interfaces.msg import ObjectsStamped


class ZedBoxToLaserNode(Node):
    """
    ZED 3D Bounding Box를 2D LaserScan 평면으로 정밀 투영하는 노드
    """

    def __init__(self):
        super().__init__('zed_box_to_laser_node')

        self.declare_parameter('objects_topic', '/zed/zed_node/obj_det/objects')
        self.declare_parameter('scan_topic', '/zed_object_scan')
        
        # Z-axis Filtering 
        self.declare_parameter('min_z_height', 0.1)  
        self.declare_parameter('max_z_height', 2.0)  

        # LaserScan Specs
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', math.radians(0.5))
        self.declare_parameter('range_min', 0.2)
        self.declare_parameter('range_max', 20.0)

        # 프로젝션 시 선분을 따라 점을 찍을 간격 (예: 2cm)
        self.declare_parameter('projection_resolution', 0.02)

        self.objects_topic = self.get_parameter('objects_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.min_z = self.get_parameter('min_z_height').value
        self.max_z = self.get_parameter('max_z_height').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.proj_res = self.get_parameter('projection_resolution').value

        self.num_rays = int((self.angle_max - self.angle_min) / self.angle_increment)

        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.obj_sub = self.create_subscription(
            ObjectsStamped, self.objects_topic, self.objects_callback, 10
        )

    def draw_segment(self, scan_ranges: List[float], p1: Tuple[float, float], p2: Tuple[float, float]):
        """두 점(p1, p2) 사이를 선분으로 잇고 LaserScan 배열을 업데이트합니다."""
        x1, y1 = p1
        x2, y2 = p2
        dist = math.hypot(x2 - x1, y2 - y1)
        
        if dist == 0:
            return

        # 선분을 resolution 간격으로 쪼개서 샘플링
        steps = int(dist / self.proj_res) + 1
        
        for i in range(steps + 1):
            t = i / steps
            px = x1 + t * (x2 - x1)
            py = y1 + t * (y2 - y1)
            
            r = math.hypot(px, py)
            if r < self.range_min or r > self.range_max:
                continue
                
            theta = math.atan2(py, px)
            idx = int((theta - self.angle_min) / self.angle_increment)
            
            if 0 <= idx < self.num_rays:
                # 더 가까운 표면일 경우에만 업데이트 (가림 현상 자동 해결)
                if r < scan_ranges[idx]:
                    scan_ranges[idx] = r

    def objects_callback(self, msg: ObjectsStamped) -> None:
        scan_msg = LaserScan()
        scan_msg.header = msg.header
        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max

        # 초기화 (에러 방지를 위해 range_max보다 약간 큰 값 사용)
        out_of_range_val = float(self.range_max + 1.0)
        # scan_ranges = [out_of_range_val] * self.num_rays
        scan_ranges = [math.inf] * self.num_rays

        for obj in msg.objects:
            # if obj.tracking_state == 0:
            #     continue

            # --- [수정 포인트 1: 정상 추적(OK) 상태만 허용] ---
            # zed_interfaces 기준 tracking_state Enum:
            # 0: OFF, 1: OK, 2: SEARCHING, 3: TERMINATE
            # SEARCHING(2) 상태인 유령 박스를 강제 차단합니다.
            if obj.tracking_state != 1:
                continue

            # --- [수정 포인트 2: Ghost Box 강제 거리 차단] ---
            # 객체 중심(position)의 2D 평면(XY) 거리를 계산합니다.
            # ZED의 3D 객체 인지 한계인 2.8m 밖으로 중심이 벗어나면 무조건 무시합니다.
            dist_to_center = math.hypot(obj.position[0], obj.position[1])
            if dist_to_center > 2.8:
                continue
            
            # 1. 3D 꼭짓점 추출 (ZED SDK 버전에 따른 필드명 호환)
            corners_2d = []
            z_values = []
            
            for corner in obj.bounding_box_3d.corners:
                # zed_interfaces 최신 버전은 'kp' 배열 사용, 구버전은 x, y, z 사용
                if hasattr(corner, 'kp'):
                    cx, cy, cz = corner.kp[0], corner.kp[1], corner.kp[2]
                else:
                    cx, cy, cz = corner.x, corner.y, corner.z
                
                corners_2d.append((cx, cy))
                z_values.append(cz)

            # 2. 정밀한 Z축 필터링 (Bounding Box 전체의 높이를 검사)
            obj_min_z = min(z_values)
            obj_max_z = max(z_values)

            # 물체가 로봇 설정 높이 범위를 완전히 벗어나면 무시
            if obj_min_z > self.max_z or obj_max_z < self.min_z:
                continue

            # 3. 2D Bounding Box 윤곽선 렌더링
            # 8개의 점으로 만들 수 있는 모든 선분을 다 그려버립니다.
            # 내부 대각선이 그려지더라도, LaserScan은 '최단 거리'만 취하므로
            # 자연스럽게 로봇과 마주 보는 외부 표면만 남게 됩니다. (Hidden Surface Removal 효과)
            num_corners = len(corners_2d)
            for i in range(num_corners):
                for j in range(i + 1, num_corners):
                    self.draw_segment(scan_ranges, corners_2d[i], corners_2d[j])

        # 타입 에러 방지용 강제 캐스팅
        # scan_msg.ranges = [float(r) for r in scan_ranges]
        # 2. 타입 에러 및 inf 바운딩 에러 우회
        # 파이썬 리스트를 C-style의 float32('f') 배열로 묶어서 대입합니다.
        # 이렇게 하면 ROS 2가 개별 값 검사를 건너뛰고 C++로 바로 넘깁니다.
        scan_msg.ranges = array.array('f', scan_ranges)
        
        self.scan_pub.publish(scan_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ZedBoxToLaserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
