#!/usr/bin/env python3
"""Synthetic nav2-like traffic for recorder tests (load, export and nightly tests).

Sizes and rates of a busy robot: /tf 500 Hz (+ map->odom 10 Hz from a second publisher), /odom
100 Hz, /scan 7200 beams 30 Hz, 4 x 720-beam scans (/scan_front ...) 15 Hz, local costmap_raw 500x500
2 Hz (rolling window over a world-fixed obstacle field), global costmap diff stream (latched
costmap_raw + costmap_raw_updates 5 Hz + get_costmap), /map, /plan 1 Hz, /local_plan 20 Hz, cmd_vel
chain 20 Hz, /robot_state, /bt_error_code, navigate_through_poses action status. A scripted 60 s
cycle produces R1 (stop 12 s), R2 (speed limited 3 s), R3 (recovery 5 s) and R4 (bt_error_code +
ABORTED) once per cycle.

  python3 fake_nav_pub.py [--seconds N] [--no-scan-load]
"""
import argparse
import array
import math
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav2_msgs.msg import Costmap, CostmapUpdate
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, UInt16
from tf2_msgs.msg import TFMessage

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
RES = 0.02
GW, GH, GRES = 400, 250, 0.05   # synthetic global map


def world_cost(gx, gy):
    h = (gx * 73856093) ^ (gy * 19349663)
    return np.where(h % 97 == 0, 254, np.where(h % 89 == 0, 128, 0)).astype(np.uint8)


def stamp(node):
    return node.get_clock().now().to_msg()


class Fake(Node):
    def __init__(self, scan_load=True):
        super().__init__('fake_nav_pub')
        self.t0 = time.monotonic()
        self.rng = np.random.default_rng(0)
        self.pub = {}
        P = self.create_publisher
        self.p_tf = P(TFMessage, '/tf', 100)
        self.p_odom = P(Odometry, '/odom', 10)
        self.p_scan = P(LaserScan, '/scan', QoSProfile(
            depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.p_scans = [P(LaserScan, f'/scan_{n}', 5)
                        for n in ('front', 'rear', 'right', 'left')]
        self.p_local = P(Costmap, '/local_costmap/costmap_raw', 1)
        self.p_gfull = P(Costmap, '/global_costmap/costmap_raw', LATCHED)
        self.p_gupd = P(CostmapUpdate, '/global_costmap/costmap_raw_updates', 10)
        self.p_map = P(OccupancyGrid, '/map', LATCHED)
        self.p_plan = P(Path, '/plan', 1)
        self.p_lplan = P(Path, '/local_plan', 1)
        self.p_sm = P(Twist, '/cmd_vel_smoothed', 1)
        self.p_adj = P(Twist, '/cmd_vel', 1)
        self.p_status = P(String, '/robot_state', LATCHED)
        self.p_err = P(UInt16, '/bt_error_code', 10)
        self.p_act = P(GoalStatusArray, '/navigate_through_poses/_action/status', LATCHED)
        self.p_static = P(TFMessage, '/tf_static', LATCHED)
        self.create_service(GetCostmap, '/global_costmap/get_costmap', self.on_get)

        # second node-like publisher for map->odom (separate writer on /tf)
        self.helper = rclpy.create_node('fake_nav_pub_loc')
        self.p_tf2 = self.helper.create_publisher(TFMessage, '/tf', 100)
        self.p_static2 = self.helper.create_publisher(TFMessage, '/tf_static', LATCHED)

        self.global_grid = np.zeros((GH, GW), np.uint8)
        gy, gx = np.mgrid[0:GH, 0:GW]
        self.global_grid[:] = world_cost(gx, gy)
        self.global_stamp = stamp(self)
        self.publish_static()
        self.publish_map()
        self.status = None
        self.goal_uuid = bytes(self.rng.integers(0, 255, 16, dtype=np.uint8))
        self.scan_ranges = self.rng.uniform(0.3, 30.0, 7200).astype(np.float32)
        self.small_ranges = [self.rng.uniform(0.3, 10.0, 720).astype(np.float32) for _ in range(4)]
        self.x = self.y = self.yaw = 0.0
        self.v = 0.0

        T = self.create_timer
        T(1 / 500, self.on_tf)
        self.helper_timer = self.helper.create_timer(1 / 10, self.on_map_odom)
        T(1 / 100, self.on_odom)
        if scan_load:
            T(1 / 30, self.on_scan)
            T(1 / 15, self.on_small_scans)
        T(1 / 2, self.on_local_costmap)
        T(1 / 5, self.on_global_update)
        T(1.0, self.on_plan)
        T(1 / 20, self.on_20hz)
        T(0.5, self.on_script)

    # ------------------------------------------------------------------ scripted scenario
    def phase(self):
        c = (time.monotonic() - self.t0) % 60.0
        if c < 5:
            return 'IDLE'
        if 15 <= c < 27:
            return 'STOPPED'        # status DRIVING but speed 0 -> R1
        if 30 <= c < 33:
            return 'LIMITED'        # adjusted < 50% smoothed -> R2
        if 40 <= c < 45:
            return 'RECOVERY'       # -> R3 (recovery_states)
        if 50 <= c < 51:
            return 'ERROR'          # bt_error_code, ABORTED -> R4
        return 'DRIVING'

    def on_script(self):
        ph = self.phase()
        status = {'IDLE': 'IDLE', 'RECOVERY': 'RECOVERY'}.get(ph, 'DRIVING')
        if status != self.status:
            self.status = status
            self.p_status.publish(String(data=status))
        e = UInt16(data=1 if ph == 'ERROR' else 0)
        self.p_err.publish(e)
        arr = GoalStatusArray()
        gs = GoalStatus()
        gs.goal_info.goal_id.uuid = list(self.goal_uuid)
        gs.goal_info.stamp = stamp(self)
        gs.status = GoalStatus.STATUS_ABORTED if ph == 'ERROR' else (
            GoalStatus.STATUS_EXECUTING if status != 'IDLE' else GoalStatus.STATUS_SUCCEEDED)
        arr.status_list = [gs]
        self.p_act.publish(arr)
        if ph == 'IDLE' and gs.status == GoalStatus.STATUS_SUCCEEDED:
            self.goal_uuid = bytes(self.rng.integers(0, 255, 16, dtype=np.uint8))

    # ------------------------------------------------------------------ publishers
    def publish_static(self):
        def st(parent, child, x):
            t = TransformStamped()
            t.header.stamp = stamp(self)
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x = x
            t.transform.rotation.w = 1.0
            return t
        self.p_static.publish(TFMessage(transforms=[st('base_link', 'laser_front', 0.4),
                                                    st('base_link', 'laser_rear', -0.4)]))
        self.p_static2.publish(TFMessage(transforms=[st('base_link', 'camera_front', 0.5)]))

    def publish_map(self):
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.header.stamp = stamp(self)
        m.info.resolution = GRES
        m.info.width, m.info.height = GW, GH
        m.info.origin.orientation.w = 1.0
        m.data = array.array('b', np.where(self.global_grid == 254, 100, 0).astype(np.int8).tobytes())
        self.p_map.publish(m)
        full = self.global_msg()
        self.p_gfull.publish(full)

    def global_msg(self):
        c = Costmap()
        c.header.frame_id = 'map'
        c.header.stamp = self.global_stamp
        c.metadata.resolution = GRES
        c.metadata.size_x, c.metadata.size_y = GW, GH
        c.metadata.origin.orientation.w = 1.0
        c.data = array.array('B', self.global_grid.tobytes())
        return c

    def on_get(self, req, resp):
        resp.map = self.global_msg()
        return resp

    def on_global_update(self):
        x = int(self.rng.integers(0, GW - 20))
        y = int(self.rng.integers(0, GH - 20))
        patch = self.rng.choice(np.array([0, 254], np.uint8), size=(10, 12))
        self.global_grid[y:y + 10, x:x + 12] = patch
        self.global_stamp = stamp(self)
        u = CostmapUpdate()
        u.header.frame_id = 'map'
        u.header.stamp = self.global_stamp
        u.x, u.y, u.size_x, u.size_y = x, y, 12, 10
        u.data = array.array('B', patch.tobytes())
        self.p_gupd.publish(u)

    def on_tf(self):
        t = TransformStamped()
        t.header.stamp = stamp(self)
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.yaw / 2)
        t.transform.rotation.w = math.cos(self.yaw / 2)
        self.p_tf.publish(TFMessage(transforms=[t]))

    def on_map_odom(self):
        t = TransformStamped()
        t.header.stamp = self.helper.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = 5.0
        t.transform.translation.y = 4.0
        t.transform.rotation.w = 1.0
        self.p_tf2.publish(TFMessage(transforms=[t]))

    def on_odom(self):
        ph = self.phase()
        self.v = 0.0 if ph in ('IDLE', 'STOPPED', 'RECOVERY') else 0.5
        w = 0.1
        dt = 0.01
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += w * dt if self.v else 0.0
        o = Odometry()
        o.header.stamp = stamp(self)
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.twist.twist.linear.x = self.v
        self.p_odom.publish(o)

    def scan_msg(self, ranges, frame):
        s = LaserScan()
        s.header.stamp = stamp(self)
        s.header.frame_id = frame
        s.angle_min = -math.pi
        s.angle_increment = 2 * math.pi / len(ranges)
        s.angle_max = s.angle_min + s.angle_increment * (len(ranges) - 1)
        s.range_min, s.range_max = 0.05, 40.0
        s.scan_time = 1 / 30
        s.ranges = array.array('f', ranges.astype(np.float32).tobytes())
        return s

    def on_scan(self):
        self.scan_ranges += self.rng.normal(0, 0.005, self.scan_ranges.size).astype(np.float32)
        np.clip(self.scan_ranges, 0.06, 39.0, out=self.scan_ranges)
        r = self.scan_ranges.copy()
        r[::500] = np.inf
        self.p_scan.publish(self.scan_msg(r, 'laser_front'))

    def on_small_scans(self):
        for p, r in zip(self.p_scans, self.small_ranges):
            r += self.rng.normal(0, 0.005, r.size).astype(np.float32)
            np.clip(r, 0.06, 9.9, out=r)
            p.publish(self.scan_msg(r, 'base_link'))

    def on_local_costmap(self):
        mx, my = 5.0 + self.x, 4.0 + self.y          # robot in map
        cx = int(round((mx - 5.0) / RES))
        cy = int(round((my - 5.0) / RES))
        gy, gx = np.mgrid[0:500, 0:500]
        data = world_cost(gx + cx, gy + cy)
        data[240:260, 240:260] = 253
        c = Costmap()
        c.header.frame_id = 'odom'
        c.header.stamp = stamp(self)
        c.metadata.resolution = RES
        c.metadata.size_x = c.metadata.size_y = 500
        c.metadata.origin.position.x = cx * RES
        c.metadata.origin.position.y = cy * RES
        c.metadata.origin.orientation.w = 1.0
        c.data = array.array('B', data.astype(np.uint8).tobytes())
        self.p_local.publish(c)

    def path(self, n, step, frame):
        p = Path()
        p.header.frame_id = frame
        p.header.stamp = stamp(self)
        bx, by = 5.0 + self.x, 4.0 + self.y
        for i in range(n):
            ps = PoseStamped()
            ps.header = p.header
            ps.pose.position.x = bx + i * step * math.cos(self.yaw)
            ps.pose.position.y = by + i * step * math.sin(self.yaw)
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        return p

    def on_plan(self):
        self.p_plan.publish(self.path(400, 0.05, 'map'))

    def on_20hz(self):
        self.p_lplan.publish(self.path(60, 0.05, 'map'))
        sm = Twist()
        sm.linear.x = self.v
        adj = Twist()
        adj.linear.x = self.v * (0.3 if self.phase() == 'LIMITED' else 1.0)
        self.p_sm.publish(sm)
        self.p_adj.publish(adj)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=0)
    ap.add_argument('--no-scan-load', action='store_true')
    a = ap.parse_args()
    rclpy.init()
    n = Fake(scan_load=not a.no_scan_load)
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(n)
    ex.add_node(n.helper)
    t0 = time.monotonic()
    try:
        while rclpy.ok() and (not a.seconds or time.monotonic() - t0 < a.seconds):
            ex.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
