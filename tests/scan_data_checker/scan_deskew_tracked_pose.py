# scan_tracked_pose_corrector.py
import bisect
import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def quat_rotate(q, v):
    """q(x,y,z,w)로 v 회전. q:(4,) 또는 (N,4), v:(N,3)."""
    q = np.asarray(q, dtype=np.float64)
    v = np.asarray(v, dtype=np.float64)
    qv = q[..., :3]
    qw = q[..., 3:4]
    t = 2.0 * np.cross(qv, v)
    return v + qw * t + np.cross(qv, t)


def slerp_pair(q0, q1, a):
    """단일 비율 a로 q0->q1 보간 (a: float)."""
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        out = q0 + a * (q1 - q0)
        return out / np.linalg.norm(out)
    th = math.acos(max(-1.0, min(1.0, dot)))
    s = math.sin(th)
    return (math.sin((1.0 - a) * th) / s) * q0 + (math.sin(a * th) / s) * q1


class ScanTrackedPoseCorrector(Node):
    def __init__(self):
        super().__init__('scan_tracked_pose_corrector')

        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.input_topic = self.declare_parameter('input_topic', 'f_scan').value
        self.output_topic = self.declare_parameter(
            'output_topic', 'f_scan_corrected').value
        self.tracked_pose_topic = self.declare_parameter(
            'tracked_pose_topic', '/tracked_pose').value
        self.stamp_at_end = self.declare_parameter('stamp_at_end', True).value
        self.pose_wait = self.declare_parameter('pose_wait_sec', 0.2).value
        self.min_range = self.declare_parameter('min_range', 0.0).value
        self.max_range = self.declare_parameter('max_range', 0.0).value
        self.stride = self.declare_parameter('stride', 1).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # tracked_pose 시계열: (t_sec, pos(3,), quat(4,))
        self.pose_buf = deque(maxlen=400)
        self.pending = deque(maxlen=50)  # 보류 스캔

        self.pose_sub = self.create_subscription(
            PoseStamped, self.tracked_pose_topic, self.on_pose,
            qos_profile_sensor_data)
        self.scan_sub = self.create_subscription(
            LaserScan, self.input_topic, self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(
            PointCloud2, self.output_topic, qos_profile_sensor_data)

        self._fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12,
                       datatype=PointField.FLOAT32, count=1),
        ]
        # 주기적으로 보류 스캔 처리 시도
        self.create_timer(0.01, self.flush_pending)

    # ---- tracked_pose 수집 ----
    def on_pose(self, msg):
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        p = np.array([msg.pose.position.x, msg.pose.position.y,
                      msg.pose.position.z])
        q = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                      msg.pose.orientation.z, msg.pose.orientation.w])
        self.pose_buf.append((t, p, q))

    def lookup_pose(self, t_query):
        """tracked_pose 버퍼에서 t_query(map->base_link)를 양쪽 보간. 없으면 None."""
        buf = self.pose_buf
        if len(buf) < 2:
            return None
        times = [b[0] for b in buf]
        if t_query < times[0] or t_query > times[-1]:
            return None  # 아직 감싸지 못함 (미래) 또는 너무 오래됨
        i = bisect.bisect_left(times, t_query)
        if i == 0:
            i = 1
        t_a, p_a, q_a = buf[i - 1]
        t_b, p_b, q_b = buf[i]
        if t_b <= t_a:
            return p_b, q_b
        a = (t_query - t_a) / (t_b - t_a)
        return (1.0 - a) * p_a + a * p_b, slerp_pair(q_a, q_b, a)

    # ---- 스캔 수신: de-skew(base_link) 후 보류 ----
    def on_scan(self, scan):
        laser_frame = scan.header.frame_id
        n = len(scan.ranges)
        if n < 2:
            return

        dt = (scan.time_increment if scan.time_increment > 0.0
              else (scan.scan_time / (n - 1) if scan.scan_time > 0.0 else 0.0))
        if dt <= 0.0:
            return
        sweep = (n - 1) * dt
        t_stamp = Time.from_msg(scan.header.stamp)
        t_first = (t_stamp - Duration(seconds=sweep)) if self.stamp_at_end else t_stamp
        t0 = t_first
        t1 = t_first + Duration(seconds=sweep)
        t_ref = t1

        # de-skew 를 base_link 기준으로 (스윕 내부 왜곡만 제거)
        try:
            T0 = self.tf_buffer.lookup_transform('base_link', laser_frame, t0)
            T1 = self.tf_buffer.lookup_transform('base_link', laser_frame, t1)
        except TransformException as exc:
            self.get_logger().warn(f'TF(base_link<-{laser_frame}): {exc}',
                                   throttle_duration_sec=1.0)
            return
        p0 = np.array([T0.transform.translation.x, T0.transform.translation.y,
                       T0.transform.translation.z])
        q0 = np.array([T0.transform.rotation.x, T0.transform.rotation.y,
                       T0.transform.rotation.z, T0.transform.rotation.w])
        p1 = np.array([T1.transform.translation.x, T1.transform.translation.y,
                       T1.transform.translation.z])
        q1 = np.array([T1.transform.rotation.x, T1.transform.rotation.y,
                       T1.transform.rotation.z, T1.transform.rotation.w])

        ranges = np.asarray(scan.ranges, dtype=np.float64)
        angles = scan.angle_min + np.arange(n) * scan.angle_increment
        alphas = np.arange(n, dtype=np.float64) / (n - 1)
        eff_min = max(scan.range_min, self.min_range)
        eff_max = (scan.range_max if self.max_range <= 0.0
                   else min(scan.range_max, self.max_range))
        mask = (np.isfinite(ranges) & (ranges >= eff_min) & (ranges <= eff_max))
        if self.stride > 1:
            keep = np.zeros(n, dtype=bool)
            keep[::self.stride] = True
            mask &= keep
        r = ranges[mask]
        a = angles[mask]
        alpha = alphas[mask]
        if r.size == 0:
            return
        inten = (np.asarray(scan.intensities, dtype=np.float32)[mask]
                 if len(scan.intensities) == n else np.zeros(r.size, np.float32))

        p_laser = np.column_stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r)])
        # base_link(t_ref) 기준으로 각 빔 재투영
        qb = np.array([slerp_pair(q0, q1, av) for av in alpha])
        tb = (1.0 - alpha)[:, None] * p0 + alpha[:, None] * p1
        p_bl_i = quat_rotate(qb, p_laser) + tb
        q1c = np.array([-q1[0], -q1[1], -q1[2], q1[3]])
        p_base = quat_rotate(q1c, p_bl_i - p1)  # base_link(t_ref) 좌표

        self.pending.append((t_ref.nanoseconds * 1e-9, p_base, inten))

    # ---- 보류 스캔을 tracked_pose 도착 시 map 으로 배치 ----

    def flush_pending(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        keep = deque(maxlen=self.pending.maxlen)
        while self.pending:
            t_ref, p_base, inten = self.pending.popleft()
            pose = self.lookup_pose(t_ref)          # 1순위: 보정 pose 버퍼
            if pose is None:
                if now - t_ref < self.pose_wait:
                    keep.append((t_ref, p_base, inten))  # 아직 도착 가능 → 대기
                    continue
                # 2순위: 누락 확정 → 예측 TF 로 fallback (연속성 확보)
                pose = self.lookup_tf_pose(t_ref)
                if pose is None:
                    continue
            p_map_base, q_map_base = pose
            p_map = quat_rotate(q_map_base, p_base) + p_map_base
            self.publish_cloud(p_map, inten, t_ref)
        self.pending = keep

    def lookup_tf_pose(self, t_ref):
        """누락 구간 fallback: map->base_link 예측 TF."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, 'base_link',
                Time(seconds=t_ref), Duration(seconds=0.02))
        except TransformException:
            return None
        p = np.array([tf.transform.translation.x, tf.transform.translation.y,
                      tf.transform.translation.z])
        q = np.array([tf.transform.rotation.x, tf.transform.rotation.y,
                      tf.transform.rotation.z, tf.transform.rotation.w])
        return p, q

    

    def publish_cloud(self, pts_xyz, inten, t_ref_sec):
        header = Header()
        header.stamp = Time(seconds=t_ref_sec).to_msg()
        header.frame_id = self.map_frame
        pts = np.column_stack([pts_xyz.astype(np.float32), inten])
        self.pub.publish(point_cloud2.create_cloud(header, self._fields, pts))


def main(args=None):
    rclpy.init(args=args)
    node = ScanTrackedPoseCorrector()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
