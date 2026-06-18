# laser_deskew_node.py
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def quat_rotate(q, v):
    """쿼터니언 q(x,y,z,w)로 벡터 v를 회전. q는 (4,) 또는 (N,4), v는 (N,3)."""
    q = np.asarray(q, dtype=np.float64)
    v = np.asarray(v, dtype=np.float64)
    qv = q[..., :3]
    qw = q[..., 3:4]
    t = 2.0 * np.cross(qv, v)
    return v + qw * t + np.cross(qv, t)


def slerp(q0, q1, t):
    """q0 -> q1 사이를 비율 배열 t(N,)로 보간. 반환 (N,4)."""
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:          # 최단 경로 보장
        q1 = -q1
        dot = -dot
    t = np.asarray(t, dtype=np.float64)
    if dot > 0.9995:       # 거의 평행하면 선형 보간 + 정규화
        res = q0[None, :] + t[:, None] * (q1 - q0)[None, :]
        return res / np.linalg.norm(res, axis=1, keepdims=True)
    theta0 = math.acos(max(-1.0, min(1.0, dot)))
    sin0 = math.sin(theta0)
    s0 = np.sin((1.0 - t) * theta0) / sin0
    s1 = np.sin(t * theta0) / sin0
    return s0[:, None] * q0[None, :] + s1[:, None] * q1[None, :]


def transform_to_pq(ts):
    """TransformStamped -> (translation(3,), quaternion xyzw(4,))."""
    tr = ts.transform.translation
    ro = ts.transform.rotation
    return (np.array([tr.x, tr.y, tr.z]),
            np.array([ro.x, ro.y, ro.z, ro.w]))


class LaserDeskewNode(Node):
    def __init__(self):
        super().__init__('laser_deskew_node')

        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.input_topic = self.declare_parameter('input_topic', 'scan').value
        self.output_topic = self.declare_parameter(
            'output_topic', 'scan_deskewed').value
        self.stamp_at_end = self.declare_parameter('stamp_at_end', False).value
        self.min_range = self.declare_parameter('min_range', 0.0).value
        self.max_range = self.declare_parameter('max_range', 0.0).value  # <=0 → range_max
        self.stride = self.declare_parameter('stride', 1).value          # 1 = 비활성

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(
            PointCloud2, self.output_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(
            LaserScan, self.input_topic, self.on_scan, qos_profile_sensor_data)

        self._fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12,
                       datatype=PointField.FLOAT32, count=1),
        ]

    def on_scan(self, scan):
        laser_frame = scan.header.frame_id
        n = len(scan.ranges)
        if n < 2:
            return

        # 1) 빔별 절대 시간 (LaserScan 규약상 stamp = 첫 빔 시각)
        scan_dur = (scan.scan_time if scan.scan_time > 0.0
                    else (scan.time_increment * (n - 1)
                          if scan.time_increment > 0.0 else 0.0))
        t_first = Time.from_msg(scan.header.stamp)
        if self.stamp_at_end:
            t_first = t_first - Duration(seconds=scan_dur)

        dt = (scan.time_increment if scan.time_increment > 0.0
              else (scan_dur / (n - 1) if scan_dur > 0.0 else 0.0))
        if dt <= 0.0:
            self.get_logger().warn(
                'time_increment/scan_time 가 0 → 빔별 de-skew 불가. 드라이버 설정 확인.',
                throttle_duration_sec=2.0)
            return

        t0 = t_first
        t1 = t_first + Duration(seconds=(n - 1) * dt)
        t_ref = t1  # 기준 = 스윕 끝 (출력 stamp)

        # 2) 양 끝점 TF 한 번씩만 조회 (odom <- laser)
        if not self.tf_buffer.can_transform(
                self.odom_frame, laser_frame, t1, Duration(seconds=0.0)):
            self.get_logger().warn(
                f'TF({self.odom_frame}<-{laser_frame})@t_end 미준비 → drop',
                throttle_duration_sec=1.0)
            return
        try:
            p0, q0 = transform_to_pq(
                self.tf_buffer.lookup_transform(self.odom_frame, laser_frame, t0))
            p1, q1 = transform_to_pq(
                self.tf_buffer.lookup_transform(self.odom_frame, laser_frame, t1))
        except TransformException as exc:
            self.get_logger().warn(f'TF 예외: {exc}', throttle_duration_sec=1.0)
            return

        # 3) 거리 밴드 + 옵션 stride 로 유효 빔 추출 (alpha 는 원래 인덱스 기준)
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        angles = scan.angle_min + np.arange(n) * scan.angle_increment
        alphas = np.arange(n, dtype=np.float64) / (n - 1)

        eff_min = max(scan.range_min, self.min_range)
        eff_max = (scan.range_max if self.max_range <= 0.0
                   else min(scan.range_max, self.max_range))

        mask = (np.isfinite(ranges)
                & (ranges >= eff_min)
                & (ranges <= eff_max))
        if self.stride > 1:                 # 각도 방향 다운샘플
            keep = np.zeros(n, dtype=bool)
            keep[::self.stride] = True
            mask &= keep

        r = ranges[mask]
        a = angles[mask]
        alpha = alphas[mask]
        if r.size == 0:
            return

        if len(scan.intensities) == n:
            inten = np.asarray(scan.intensities, dtype=np.float32)[mask]
        else:
            inten = np.zeros(r.size, dtype=np.float32)

        # 4) 빔별 보간: T_i = (slerp(q0,q1), lerp(p0,p1)) — odom <- laser(t_i)
        p_laser = np.column_stack(
            [r * np.cos(a), r * np.sin(a), np.zeros_like(r)])
        q_i = slerp(q0, q1, alpha)
        trans_i = (1.0 - alpha)[:, None] * p0 + alpha[:, None] * p1
        p_odom = quat_rotate(q_i, p_laser) + trans_i

        # 5) T_ref^-1 적용: p_out = R1^T (p_odom - p1) = quat_rotate(conj(q1), ...)
        q1_conj = np.array([-q1[0], -q1[1], -q1[2], q1[3]])
        p_out = quat_rotate(q1_conj, p_odom - p1)

        # 6) PointCloud2 발행 (frame_id = 센서 프레임 유지!)
        header = Header()
        header.stamp = t_ref.to_msg()
        header.frame_id = laser_frame
        pts = np.column_stack([p_out.astype(np.float32), inten])
        cloud = point_cloud2.create_cloud(header, self._fields, pts)
        self.pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = LaserDeskewNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
