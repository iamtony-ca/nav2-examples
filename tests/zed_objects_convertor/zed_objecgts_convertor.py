import math
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
# ZED ROS 2 패키지 구조에 맞게 import (zed_interfaces 사용)
from zed_interfaces.msg import ObjectsStamped


class ZedObjectToLaserNode(Node):
    """
    ZED 3D Object Detections를 2D LaserScan으로 변환하는 노드.
    Nav2 Costmap 주입을 목적으로 설계됨.
    """

    def __init__(self):
        super().__init__('zed_object_to_laser_node')

        # --- Parameters ---
        self.declare_parameter('objects_topic', '/zed/zed_node/obj_det/objects')
        self.declare_parameter('scan_topic', '/zed_object_scan')
        self.declare_parameter('base_frame_id', 'zed_camera_link')
        
        # Z-axis Filtering (로봇의 충돌 높이 기준)
        self.declare_parameter('min_z_height', 0.1)  # 바닥 노이즈 제외
        self.declare_parameter('max_z_height', 2.0)  # 로봇 키보다 높은 객체 제외

        # LaserScan Specs
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', math.radians(0.5))  # 0.5도 분해능
        self.declare_parameter('range_min', 0.2)
        self.declare_parameter('range_max', 20.0)

        # Retrieve parameters
        self.objects_topic = self.get_parameter('objects_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.min_z = self.get_parameter('min_z_height').value
        self.max_z = self.get_parameter('max_z_height').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value

        self.num_rays = int((self.angle_max - self.angle_min) / self.angle_increment)

        # --- Publishers & Subscribers ---
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.obj_sub = self.create_subscription(
            ObjectsStamped,
            self.objects_topic,
            self.objects_callback,
            10
        )

        self.get_logger().info('ZED Object to LaserScan Node has been started.')

    def objects_callback(self, msg: ObjectsStamped) -> None:
        scan_msg = LaserScan()
        scan_msg.header = msg.header
        # 필요시 base_link 등 원하는 frame으로 덮어씌울 수 있습니다.
        # scan_msg.header.frame_id = self.base_frame_id 
        
        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max

        # 초기 거리는 무한대로 설정
        scan_ranges: List[float] = [float('inf')] * self.num_rays

        for obj in msg.objects:
            # Tracking state가 유효하지 않으면 무시 (0: OFF, 2: SEARCHING)
            if obj.tracking_state == 0:
                continue

            x, y, z = obj.position

            # 1. Z축 높이 필터링
            if not (self.min_z <= z <= self.max_z):
                continue

            # 2. 객체의 2D 점유 반경(Radius) 추정
            # dimensions_3d는 일반적으로 [가로, 세로, 높이]를 나타냅니다.
            # 보수적인 충돌 회피를 위해 XY 평면에서 가장 긴 변을 기준으로 반경을 잡습니다.
            dim_x, dim_y, _ = obj.dimensions_3d
            radius = max(dim_x, dim_y) / 2.0

            # 카메라(원점)로부터 객체 중심까지의 거리 계산
            dist = math.hypot(x, y)

            # 유효 거리(Range) 확인
            if dist < self.range_min or dist > self.range_max:
                continue

            # 3. 객체가 차지하는 각도 범위(Angular Spread) 계산
            angle_center = math.atan2(y, x)

            # 객체가 너무 가까워서 반지름이 거리보다 크면 전체 범위를 덮음
            if dist > radius:
                angle_spread = math.asin(radius / dist)
            else:
                angle_spread = math.pi

            angle_start = angle_center - angle_spread
            angle_end = angle_center + angle_spread

            # 4. LaserScan 배열 채우기 (Ray casting 근사)
            start_idx = int((angle_start - self.angle_min) / self.angle_increment)
            end_idx = int((angle_end - self.angle_min) / self.angle_increment)

            # 표면까지의 거리 (중심점 거리 - 객체 반경)
            surface_dist = max(self.range_min, dist - radius)

            # 인덱스가 범위를 벗어나는 경우(Wrap-around)를 처리하여 배열 갱신
            for i in range(start_idx, end_idx + 1):
                idx = i % self.num_rays
                if surface_dist < scan_ranges[idx]:
                    scan_ranges[idx] = surface_dist

        scan_msg.ranges = scan_ranges
        self.scan_pub.publish(scan_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ZedObjectToLaserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
