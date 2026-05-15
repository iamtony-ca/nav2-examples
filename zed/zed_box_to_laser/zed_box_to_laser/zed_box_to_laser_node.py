import math
import array
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from zed_msgs.msg import ObjectsStamped


class ZedBoxToLaserNode(Node):
    """
    ZED 3D Bounding Box를 2D LaserScan 평면으로 정밀 투영하는 노드
    (근접 객체 Truncation 보정 기능 포함)
    """

    def __init__(self):
        super().__init__('zed_box_to_laser_node')

        self.declare_parameter('objects_topic', '/zed_multi/zed_front/obj_det/objects')
        self.declare_parameter('scan_topic', '/scan_zed_objects_front')
        
        # Z-axis Filtering 
        self.declare_parameter('min_z_height', 0.1)  
        self.declare_parameter('max_z_height', 2.0)  

        # LaserScan Specs
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', math.radians(0.5))
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 5.0)

        # 프로젝션 시 선분을 따라 점을 찍을 간격 (예: 2cm)
        self.declare_parameter('projection_resolution', 0.02)

        # --- [추가 파라미터: 근접 객체 보정용] ---
        # 사람이 너무 가까워져서 Bounding Box가 다리만 인식되어 작아지는 현상 방지
        self.declare_parameter('close_distance_threshold', 1.5)  # 1.5m 이내면 가깝다고 판단
        self.declare_parameter('fixed_footprint_size', 0.5)      # 0.5m x 0.5m의 고정 크기 사용

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
        
        self.close_dist_thresh = self.get_parameter('close_distance_threshold').value
        self.fixed_size = self.get_parameter('fixed_footprint_size').value

        self.num_rays = int((self.angle_max - self.angle_min) / self.angle_increment)

        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.obj_sub = self.create_subscription(
            ObjectsStamped, self.objects_topic, self.objects_callback, 10
        )

        params_info = (
            f"\n=== ZED Box to Laser Node Parameters ===\n"
            f" - objects_topic:         {self.objects_topic}\n"
            f" - scan_topic:            {self.scan_topic}\n"
            f" - min_z_height:          {self.min_z} m\n"
            f" - max_z_height:          {self.max_z} m\n"
            f" - angle_min:             {self.angle_min:.4f} rad\n"
            f" - angle_max:             {self.angle_max:.4f} rad\n"
            f" - angle_increment:       {self.angle_increment:.6f} rad\n"
            f" - range_min:             {self.range_min} m\n"
            f" - range_max:             {self.range_max} m\n"
            f" - projection_resolution: {self.proj_res} m\n"
            f" - close_dist_thresh:     {self.close_dist_thresh} m\n"
            f" - fixed_footprint_size:  {self.fixed_size} m\n"
            f"========================================"
        )
        self.get_logger().info(params_info)


    def draw_segment(self, scan_ranges: List[float], p1: Tuple[float, float], p2: Tuple[float, float]):
        """두 점(p1, p2) 사이를 선분으로 잇고 LaserScan 배열을 업데이트합니다."""
        x1, y1 = p1
        x2, y2 = p2
        dist = math.hypot(x2 - x1, y2 - y1)
        
        if dist == 0:
            return

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

        scan_ranges = [math.inf] * self.num_rays

        for obj in msg.objects:
            if obj.tracking_state == 0:
                continue

            # --- [수정 포인트 1: 정상 추적(OK) 상태만 허용] ---
            # zed_interfaces 기준 tracking_state Enum:
            # 0: OFF, 1: OK, 2: SEARCHING, 3: TERMINATE
            # SEARCHING(2) 상태인 유령 박스를 강제 차단합니다.
            # if obj.tracking_state != 1:
            #     continue

            # --- [수정 포인트 2: Ghost Box 강제 거리 차단] ---
            # 객체 중심(position)의 2D 평면(XY) 거리를 계산합니다.
            # ZED의 3D 객체 인지 한계인 2.8m 밖으로 중심이 벗어나면 무조건 무시합니다
            dist_to_center = math.hypot(obj.position[0], obj.position[1])
            if dist_to_center > 2.8:
                continue

            # 3. 추가: 속도(Velocity) 기반 필터링
            # 사람이 멀어지는 속도가 너무 빠르면(예: 1.5m/s 이상) 
            # 경계면에서 잔상이 남을 확률이 높으므로 선제적으로 차단할 수 있습니다.
            vx = obj.velocity[0]
            vy = obj.velocity[1]
            speed = math.hypot(vx, vy)
            
            # 멀어지는 방향의 속도가 큰데 이미 경계선(2.5m 이상) 근처라면 유령일 확률 높음
            if dist_to_center > 2.5 and speed > 1.2:
                continue



            # 1. 원래의 3D 꼭짓점 추출 및 정밀한 Z축 높이 검사
            original_corners_2d = []
            z_values = []
            
            for corner in obj.bounding_box_3d.corners:
                if hasattr(corner, 'kp'):
                    cx, cy, cz = corner.kp[0], corner.kp[1], corner.kp[2]
                else:
                    cx, cy, cz = corner.x, corner.y, corner.z
                
                original_corners_2d.append((cx, cy))
                z_values.append(cz)

            obj_min_z = min(z_values)
            obj_max_z = max(z_values)

            # 다리만 보이더라도 높이가 로봇의 충돌 범위를 벗어나면 무시
            if obj_min_z > self.max_z or obj_max_z < self.min_z:
                continue




# --- [수정 포인트: 실제 박스의 2D 점유 크기(가로/세로 최댓값) 계산] ---
            # 투영된 2D 꼭짓점들의 X, Y 최소/최대 좌표를 통해 실제 박스의 폭과 깊이를 구합니다.
            min_x = min(c[0] for c in original_corners_2d)
            max_x = max(c[0] for c in original_corners_2d)
            min_y = min(c[1] for c in original_corners_2d)
            max_y = max(c[1] for c in original_corners_2d)
            
            actual_size_x = max_x - min_x
            actual_size_y = max_y - min_y
            actual_max_size = max(actual_size_x, actual_size_y)

            # --- [수정 포인트: 거리 및 실제 크기를 모두 고려한 조건식] ---
            corners_2d = []
            # 조건 1: 거리가 설정 임계값(1.5m)보다 가깝다.
            # 조건 2: 그리고 실제 박스 크기가 설정한 고정 크기(0.5m)보다 작다.
            if dist_to_center < self.close_dist_thresh and actual_max_size < self.fixed_size:
                # 2. 너무 가까워서 다리만 잡히는 등 박스가 비정상적으로 작아진 경우: 강제 최소 크기 보장
                half_size = self.fixed_size / 2.0
                center_x, center_y = obj.position[0], obj.position[1]
                
                corners_2d = [
                    (center_x + half_size, center_y + half_size),
                    (center_x - half_size, center_y + half_size),
                    (center_x - half_size, center_y - half_size),
                    (center_x + half_size, center_y - half_size)
                ]
            else:
                # 3. 객체가 멀리 있거나, 가깝더라도 이미 충분히 큰 박스(예: 큰 짐)라면 원래 박스 유지
                corners_2d = original_corners_2d


            # 4. 2D Bounding Box 윤곽선 렌더링
            # . 2D Bounding Box 윤곽선 렌더링
            # 8개의 점으로 만들 수 있는 모든 선분을 다 그려버립니다.
            # 내부 대각선이 그려지더라도, LaserScan은 '최단 거리'만 취하므로
            # 자연스럽게 로봇과 마주 보는 외부 표면만 남게 됩니다. (Hidden Surface Removal 효과)

            num_corners = len(corners_2d)
            for i in range(num_corners):
                for j in range(i + 1, num_corners):
                    self.draw_segment(scan_ranges, corners_2d[i], corners_2d[j])

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