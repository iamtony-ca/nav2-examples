#!/usr/bin/env python3
"""Log the robot tilt once per second from /imu and from TF, side by side.

Usage (on the robot, after sourcing ROS 2):
  python3 imu_tilt_monitor.py
  python3 imu_tilt_monitor.py --ros-args -p topic:=/imu -p period:=1.0 \
      -p map_frame:=map -p base_frame:=base_link

Each period prints two lines:
  Angles are printed in degrees and radians.
  [IMU] ori  roll/pitch from the IMU orientation (quaternion)
        acc  roll/pitch from the accelerometer (gravity direction); only
             valid when the robot is not accelerating or turning
        tilt total angle between the IMU z axis and vertical (from acc)
        n    IMU messages received in the last period
  [TF ] roll/pitch of base_link in map (the same frames winros_bridge uses
        for the roll/pitch it sends to Windows in message 7777), and the
        age of that transform
"""
import math

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformException, TransformListener


def quat_to_rpy(x, y, z, w):
    """Return roll, pitch, yaw in radians."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def acc_to_roll_pitch(ax, ay, az):
    """Return roll, pitch, total tilt (radians) and |a| from gravity."""
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    g = math.sqrt(ax * ax + ay * ay + az * az)
    tilt = math.acos(max(-1.0, min(1.0, az / g))) if g > 1e-6 else float('nan')
    return roll, pitch, tilt, g


def ang(name, rad):
    """Format an angle in degrees and radians."""
    return f'{name} {math.degrees(rad):+6.2f} deg ({rad:+.4f} rad)'


class TiltMonitor(Node):
    def __init__(self):
        super().__init__('imu_tilt_monitor')
        self.declare_parameter('topic', '/imu')
        self.declare_parameter('period', 1.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        topic = self.get_parameter('topic').value
        period = float(self.get_parameter('period').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.last_imu = None
        self.count = 0
        # Sensor-data QoS (best effort) also receives from a reliable publisher.
        self.create_subscription(Imu, topic, self.on_imu, qos_profile_sensor_data)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_timer(period, self.on_timer)
        self.get_logger().info(
            f'IMU topic {topic}, TF {self.map_frame} -> {self.base_frame}, '
            f'logging every {period:.1f} s')

    def on_imu(self, msg):
        self.last_imu = msg
        self.count += 1

    def imu_line(self):
        n, self.count = self.count, 0
        msg = self.last_imu
        if msg is None or n == 0:
            return '[IMU] no message in the last period'

        q = msg.orientation
        if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
            ori = 'ori n/a'
        else:
            r, p, _ = quat_to_rpy(q.x, q.y, q.z, q.w)
            ori = f"ori {ang('roll', r)} {ang('pitch', p)}"

        a = msg.linear_acceleration
        ar, ap, tilt, g = acc_to_roll_pitch(a.x, a.y, a.z)
        return (f"[IMU] {ori} | acc {ang('roll', ar)} {ang('pitch', ap)} "
                f"{ang('tilt', tilt)} | |a| {g:5.2f} m/s^2 | n {n}")

    def tf_line(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException as ex:
            return f'[TF ] {self.map_frame} -> {self.base_frame} not available: {ex}'

        q = t.transform.rotation
        r, p, y = quat_to_rpy(q.x, q.y, q.z, q.w)
        stamp = Time.from_msg(t.header.stamp)
        if stamp.nanoseconds == 0:
            age = 'static'
        else:
            age = f'age {(self.get_clock().now() - stamp).nanoseconds / 1e9:5.2f} s'
        return (f"[TF ] {ang('roll', r)} {ang('pitch', p)} "
                f"| yaw {math.degrees(y):+7.2f} deg | {age}")

    def on_timer(self):
        self.get_logger().info(self.imu_line())
        self.get_logger().info(self.tf_line())


def main():
    rclpy.init()
    node = TiltMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
