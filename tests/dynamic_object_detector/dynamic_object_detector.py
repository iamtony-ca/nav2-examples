import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
import sensor_msgs_py.point_cloud2 as pc2

import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np
import math


class DynamicObstacleDetector(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_detector')

        # ==========================================
        # 파라미터 선언 및 초기화
        # ==========================================
        self.declare_parameter('use_sim_time', False)

        self.roi_min_dist = self.declare_parameter('roi.min_dist', 0.9).value
        self.roi_max_dist = self.declare_parameter('roi.max_dist', 1.1).value
        self.roi_max_angle = self.declare_parameter('roi.max_angle', 0.2).value

        self.occupancy_thresh = self.declare_parameter('map.occupancy_threshold', 50).value

        self.global_frame = self.declare_parameter('frames.global', 'map').value
        self.base_frame = self.declare_parameter('frames.base', 'base_link').value

        map_topic = self.declare_parameter('topics.map', '/map').value
        scan_topic = self.declare_parameter('topics.scan', '/scan').value
        scan_cam_topic = self.declare_parameter('topics.scan_cam', '/scan_cam').value
        alert_topic = self.declare_parameter('topics.alert', '/dynamic_obstacle_alert').value

        # ==========================================
        # 노드 초기화
        # ==========================================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Map 데이터 캐싱용 변수
        self.static_map_msg = None
        self.map_data_np = None  # numpy array로 변환된 맵 데이터

        # Transient Local QoS 설정 (정적 맵 수신용)
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, map_topic, self.map_callback, map_qos)

        self.scan_sub = self.create_subscription(
            PointCloud2, scan_topic, self.point_cloud_callback, rclpy.qos.qos_profile_sensor_data)

        self.scan_cam_sub = self.create_subscription(
            LaserScan, scan_cam_topic, self.laser_scan_callback, rclpy.qos.qos_profile_sensor_data)

        self.alert_pub = self.create_publisher(Bool, alert_topic, 10)

    def map_callback(self, msg: OccupancyGrid):
        self.static_map_msg = msg
        # 빠른 검색을 위해 맵 데이터를 미리 numpy 배열로 캐싱
        self.map_data_np = np.array(msg.data, dtype=np.int8)
        self.get_logger().info('Static map received.')

    def laser_scan_callback(self, scan_msg: LaserScan):
        if self.static_map_msg is None:
            return

        try:
            trans = self.tf_buffer.lookup_transform(
                self.global_frame,
                scan_msg.header.frame_id,
                scan_msg.header.stamp,
                rclpy.duration.Duration(seconds=0.1)
            )
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'TF Error (LaserScan): {e}', throttle_duration_sec=1.0)
            return

        # 1. LaserScan 데이터를 Numpy 배열로 변환
        ranges = np.array(scan_msg.ranges)
        angles = scan_msg.angle_min + np.arange(len(ranges)) * scan_msg.angle_increment

        # 2. 유효한 값 및 ROI 필터링 (Vectorized)
        valid_mask = np.isfinite(ranges)
        roi_mask = (ranges > self.roi_min_dist) & \
                   (ranges < self.roi_max_dist) & \
                   (np.abs(angles) < self.roi_max_angle)
        
        target_mask = valid_mask & roi_mask
        
        if not np.any(target_mask):
            return

        # 3. 국소 좌표 계산
        target_ranges = ranges[target_mask]
        target_angles = angles[target_mask]
        local_x = target_ranges * np.cos(target_angles)
        local_y = target_ranges * np.sin(target_angles)

        # 4. TF 변환 (base_link -> map)
        yaw = self._get_yaw_from_quaternion(trans.transform.rotation)
        tx = trans.transform.translation.x
        ty = trans.transform.translation.y
        
        cos_y, sin_y = np.cos(yaw), np.sin(yaw)
        map_x = local_x * cos_y - local_y * sin_y + tx
        map_y = local_x * sin_y + local_y * cos_y + ty

        # 5. 맵 점유 상태 확인
        if self._check_unmapped_obstacles_vectorized(map_x, map_y):
            self.publish_alert()

    def point_cloud_callback(self, cloud_msg: PointCloud2):
        if self.static_map_msg is None:
            return

        try:
            # ROI(전방 1m)를 검사하기 위해 map -> base_link 변환 획득
            trans = self.tf_buffer.lookup_transform(
                self.base_frame,
                cloud_msg.header.frame_id,
                cloud_msg.header.stamp,
                rclpy.duration.Duration(seconds=0.1)
            )
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'TF Error (PointCloud): {e}', throttle_duration_sec=1.0)
            return

        # 1. PointCloud2 데이터를 Numpy 배열(Nx2)로 고속 추출 (map 프레임 기준)
        pts_map = pc2.read_points_numpy(cloud_msg, field_names=('x', 'y'), skip_nans=True)
        if pts_map.size == 0:
            return

        map_x_raw = pts_map[:, 0]
        map_y_raw = pts_map[:, 1]

        # 2. 역변환 연산 (map -> base_link) Vectorized
        yaw = self._get_yaw_from_quaternion(trans.transform.rotation)
        tx = trans.transform.translation.x
        ty = trans.transform.translation.y

        dx = map_x_raw - tx
        dy = map_y_raw - ty
        cos_y, sin_y = np.cos(yaw), np.sin(yaw)
        
        base_x = dx * cos_y + dy * sin_y
        base_y = -dx * sin_y + dy * cos_y

        # 3. base_link 기준 ROI 검사
        ranges = np.hypot(base_x, base_y)
        angles = np.arctan2(base_y, base_x)

        roi_mask = (ranges > self.roi_min_dist) & \
                   (ranges < self.roi_max_dist) & \
                   (np.abs(angles) < self.roi_max_angle)

        if not np.any(roi_mask):
            return

        # 4. 필터링된 포인트들의 원래 map 좌표만 추출
        target_map_x = map_x_raw[roi_mask]
        target_map_y = map_y_raw[roi_mask]

        # 5. 맵 점유 상태 확인
        if self._check_unmapped_obstacles_vectorized(target_map_x, target_map_y):
            self.publish_alert()

    def _check_unmapped_obstacles_vectorized(self, x_array, y_array) -> bool:
        """Numpy를 이용해 다수의 좌표를 한 번에 맵 배열에서 조회합니다."""
        res = self.static_map_msg.info.resolution
        orig_x = self.static_map_msg.info.origin.position.x
        orig_y = self.static_map_msg.info.origin.position.y
        width = self.static_map_msg.info.width
        height = self.static_map_msg.info.height

        # 1D 인덱스 계산
        map_x = ((x_array - orig_x) / res).astype(np.int32)
        map_y = ((y_array - orig_y) / res).astype(np.int32)

        # 맵 범위를 벗어나는 포인트 필터링
        valid_mask = (map_x >= 0) & (map_x < width) & (map_y >= 0) & (map_y < height)
        map_x = map_x[valid_mask]
        map_y = map_y[valid_mask]

        if map_x.size == 0:
            return False

        indices = map_y * width + map_x
        occupancies = self.map_data_np[indices]

        # 조건: -1 이상, 임계값 미만인 셀이 하나라도 존재하면 True 반환
        return bool(np.any((occupancies >= -1) & (occupancies < self.occupancy_thresh)))

    def publish_alert(self):
        msg = Bool()
        msg.data = True
        self.alert_pub.publish(msg)
        self.get_logger().info('Dynamic obstacle detected in front!', throttle_duration_sec=0.5)

    @staticmethod
    def _get_yaw_from_quaternion(q):
        """Quaternion 메시지에서 Yaw 값을 추출합니다."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
