import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Path, Odometry
import tf2_ros
import math
import numpy as np
import tf_transformations

def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

class StrictLateralDockingNode(Node):
    def __init__(self):
        super().__init__('strict_lateral_docking_node')

        # --- Parameters ---
        self.path_length_threshold = 2.0
        self.cte_enable_threshold = 0.025
        self.cte_disable_threshold = 0.010
        self.final_xy_tolerance = 0.01      
        self.final_yaw_tolerance = 0.05     
        self.xy_stable_duration = 1.0       
        self.max_linear_diff = 0.05         
        self.max_angular_diff = 0.5         
        self.min_creep_speed = 0.02

        # --- State ---
        self.is_correcting = False
        self.xy_stable_start_time = None
        self.xy_completed = False

        self.latest_cmd_vel = Twist()
        self.latest_cmd_time = self.get_clock().now()
        self.pruned_path = None
        
        # --- TF ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # --- Subscribers (기본 그룹 사용 = Single Thread) ---
        self.create_subscription(Path, '/plan_pruned', self.pruned_path_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_smoothed', self.cmd_callback, 10)
        self.create_subscription(Odometry, '/odom', lambda msg: None, 10)
        
        # --- Publisher ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_input_monitor', 10)
        
        # --- Timer ---
        self.create_timer(0.05, self.control_loop)

    def pruned_path_callback(self, msg): 
        self.pruned_path = msg

    def cmd_callback(self, msg): 
        self.latest_cmd_vel = msg
        self.latest_cmd_time = self.get_clock().now()

    # ... (get_path_length, get_dist_to_global_goal, transform 등 헬퍼 함수는 그대로 유지) ...
    def get_path_length(self):
        if not self.pruned_path or len(self.pruned_path.poses) < 2: return 0.0
        coords = np.array([(p.pose.position.x, p.pose.position.y) for p in self.pruned_path.poses])
        return np.sum(np.linalg.norm(coords[1:] - coords[:-1], axis=1))

    def get_dist_to_global_goal(self, robot_pose):
        if not self.pruned_path or len(self.pruned_path.poses) == 0: return float('inf')
        goal_pt = self.pruned_path.poses[-1].pose.position
        return math.hypot(goal_pt.x - robot_pose.position.x, goal_pt.y - robot_pose.position.y)

    def transform_global_to_local(self, global_pt, robot_pose):
        dx = global_pt[0] - robot_pose.position.x
        dy = global_pt[1] - robot_pose.position.y
        import tf_transformations
        _, _, robot_yaw = tf_transformations.euler_from_quaternion(
            [robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w])
        local_x = dx * math.cos(robot_yaw) + dy * math.sin(robot_yaw)
        local_y = -dx * math.sin(robot_yaw) + dy * math.cos(robot_yaw)
        return local_x, local_y, robot_yaw

    def get_lookahead_point(self, robot_pose, lookahead_dist=0.4):
        if not self.pruned_path or len(self.pruned_path.poses) < 2: return None
        path_arr = np.array([(p.pose.position.x, p.pose.position.y) for p in self.pruned_path.poses])
        robot_xy = np.array([robot_pose.position.x, robot_pose.position.y])
        dists = np.linalg.norm(path_arr - robot_xy, axis=1)
        min_idx = np.argmin(dists)
        curr_dist = 0.0
        target_pt = path_arr[min_idx]
        for i in range(min_idx, len(path_arr) - 1):
            p1 = path_arr[i]; p2 = path_arr[i+1]
            seg_len = np.linalg.norm(p2 - p1)
            if curr_dist + seg_len >= lookahead_dist:
                ratio = (lookahead_dist - curr_dist) / seg_len
                return p1 + (p2 - p1) * ratio
            curr_dist += seg_len
            target_pt = p2
        return target_pt

    def control_loop(self):
        # Safety Check
        if (self.get_clock().now() - self.latest_cmd_time).nanoseconds > 0.5 * 1e9:
            self.cmd_pub.publish(Twist()); return

        final_cmd = Twist()
        final_cmd.linear = self.latest_cmd_vel.linear
        final_cmd.angular = self.latest_cmd_vel.angular

        # [중요] Single Thread에서는 타임아웃이 길면 전체가 멈춤. 
        # 반드시 0.0 또는 매우 짧게 설정하여 Non-blocking 효과를 냄.
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), 
                timeout=Duration(seconds=0.0) # 0.0초 추천 (즉시 리턴)
            )
            robot_pose = Pose()
            robot_pose.position.x = trans.transform.translation.x
            robot_pose.position.y = trans.transform.translation.y
            robot_pose.position.z = trans.transform.translation.z
            robot_pose.orientation = trans.transform.rotation
        except Exception:
            # TF 못 구하면 이번 턴은 넘김 (Blocking 하지 않음)
            self.cmd_pub.publish(final_cmd) 
            return

        if self.pruned_path is None: return

        # ... (이하 로직은 기존과 완전히 동일) ...
        # (Stage 2 및 Stage 1 로직 복사해서 사용하시면 됩니다)
        path_len = self.get_path_length()
        dist_to_goal = self.get_dist_to_global_goal(robot_pose)

        # [Stage 2] Final Docking
        if dist_to_goal < 0.05:
            goal_pose_global = self.pruned_path.poses[-1].pose
            _, _, goal_yaw = tf_transformations.euler_from_quaternion(
                [goal_pose_global.orientation.x, goal_pose_global.orientation.y, goal_pose_global.orientation.z, goal_pose_global.orientation.w])
            _, _, current_yaw = tf_transformations.euler_from_quaternion(
                [robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w])
            yaw_error = normalize_angle(goal_yaw - current_yaw)
            
            xy_satisfied = dist_to_goal < self.final_xy_tolerance
            yaw_satisfied = abs(yaw_error) < self.final_yaw_tolerance

            if xy_satisfied and yaw_satisfied:
                final_cmd.linear.x = 0.0; final_cmd.angular.z = 0.0
                self.cmd_pub.publish(final_cmd); self.is_correcting = True; return

            if xy_satisfied:
                if self.xy_stable_start_time is None: self.xy_stable_start_time = self.get_clock().now()
                if (self.get_clock().now() - self.xy_stable_start_time).nanoseconds / 1e9 > self.xy_stable_duration:
                    self.xy_completed = True
            else:
                self.xy_stable_start_time = None; self.xy_completed = False

            if self.xy_completed:
                self.is_correcting = False; self.cmd_pub.publish(final_cmd); return

            goal_pt_global = [goal_pose_global.position.x, goal_pose_global.position.y]
            local_x, local_y, _ = self.transform_global_to_local(goal_pt_global, robot_pose)
            
            kp_dist = 1.5; calc_vx = kp_dist * local_x 
            speed_limit = 0.1 
            if abs(calc_vx) > speed_limit: calc_vx = math.copysign(speed_limit, calc_vx)
            if abs(calc_vx) < self.min_creep_speed: calc_vx = math.copysign(self.min_creep_speed, calc_vx)

            target_yaw_local = math.atan2(local_y, local_x)
            if local_x < 0: steering_error = normalize_angle(target_yaw_local - math.pi)
            else: steering_error = target_yaw_local

            calc_w = 2.5 * steering_error
            calc_w = max(min(calc_w, 0.8), -0.8)

            lin_diff = max(min(calc_vx - final_cmd.linear.x, self.max_linear_diff), -self.max_linear_diff)
            final_cmd.linear.x = final_cmd.linear.x + lin_diff
            ang_diff = max(min(calc_w - final_cmd.angular.z, self.max_angular_diff), -self.max_angular_diff)
            final_cmd.angular.z = final_cmd.angular.z + ang_diff
            self.is_correcting = True

        # [Stage 1] Approach
        elif path_len < self.path_length_threshold:
            lookahead_pt = self.get_lookahead_point(robot_pose, lookahead_dist=0.4)
            if lookahead_pt is not None:
                local_x, local_y, _ = self.transform_global_to_local(lookahead_pt, robot_pose)
                current_cte = abs(local_y)
                if not self.is_correcting:
                    if current_cte > self.cte_enable_threshold: self.is_correcting = True
                else:
                    if current_cte < self.cte_disable_threshold: self.is_correcting = False

                if self.is_correcting:
                    target_v = final_cmd.linear.x
                    if abs(target_v) < self.min_creep_speed:
                         if abs(target_v) < 0.001: target_v = self.min_creep_speed 
                         else: target_v = math.copysign(self.min_creep_speed, target_v)

                    dist_sq = local_x**2 + local_y**2
                    curvature = 2.0 * local_y / dist_sq if dist_sq > 0 else 0
                    final_cmd.linear.x = target_v
                    final_cmd.angular.z = max(min(target_v * curvature, 0.8), -0.8)
        else:
            self.is_correcting = False

        self.cmd_pub.publish(final_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = StrictLateralDockingNode()
    # [복귀] 기본 Single Threaded Executor 사용 (가장 효율적)
    rclpy.spin(node) 
    node.destroy_node()
    rclpy.shutdown()
