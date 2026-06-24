#!/usr/bin/env python3

import math
import random
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
import tf2_ros


class RandomNavTester(Node):
    def __init__(self):
        super().__init__('random_nav_tester')

        # Parameters (최대 허용 오프셋 거리 및 각도)
        self.declare_parameter('max_dx', 1.5)
        self.declare_parameter('max_dy', 1.5)
        self.declare_parameter('max_dyaw', 1.57) # 약 90도
        
        self.max_dx = self.get_parameter('max_dx').value
        self.max_dy = self.get_parameter('max_dy').value
        self.max_dyaw = self.get_parameter('max_dyaw').value

        # State variables
        self.iterations_left = 0
        self.is_navigating = False

        # TF2 Setup for getting current pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Callback group for async operations
        self.cb_group = ReentrantCallbackGroup()

        # Trigger Subscriber
        self.trigger_sub = self.create_subscription(
            Int32,
            'trigger_random_nav',
            self.trigger_callback,
            10,
            callback_group=self.cb_group
        )

        # Action Client
        self.action_client = ActionClient(
            self, 
            NavigateThroughPoses, 
            'navigate_through_poses',
            callback_group=self.cb_group
        )

        self.get_logger().info("Random Nav Tester initialized. Waiting for trigger...")

    def trigger_callback(self, msg: Int32):
        if self.is_navigating:
            self.get_logger().warn("Already navigating! Ignoring new trigger.")
            return
        
        if msg.data <= 0:
            self.get_logger().warn("Trigger value must be greater than 0.")
            return

        self.iterations_left = msg.data
        self.is_navigating = True
        self.get_logger().info(f"Trigger received. Starting {self.iterations_left} random iterations.")
        self.execute_next_goal()

    def execute_next_goal(self):
        if self.iterations_left <= 0:
            self.get_logger().info("All random navigation iterations completed.")
            self.is_navigating = False
            return

        # 1. Get current pose
        try:
            now = rclpy.time.Time()
            # map frame 기준 base_link의 현재 위치 확인
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_link', now, timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except tf2_ros.TransformException as ex:
            self.get_logger().error(f"Could not transform map to base_link: {ex}")
            self.is_navigating = False
            return

        curr_x = trans.transform.translation.x
        curr_y = trans.transform.translation.y
        curr_q = trans.transform.rotation

        # 현재 Yaw 계산
        curr_yaw = math.atan2(
            2.0 * (curr_q.w * curr_q.z + curr_q.x * curr_q.y),
            1.0 - 2.0 * (curr_q.y * curr_q.y + curr_q.z * curr_q.z)
        )

        # 2. Generate random offsets
        dx = random.uniform(-self.max_dx, self.max_dx)
        dy = random.uniform(-self.max_dy, self.max_dy)
        dyaw = random.uniform(-self.max_dyaw, self.max_dyaw)

        target_yaw = curr_yaw + dyaw

        # 3. Create Goal Pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = curr_x + dx
        goal_pose.pose.position.y = curr_y + dy
        goal_pose.pose.position.z = 0.0
        
        # Target Yaw -> Quaternion 변환
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = math.sin(target_yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(target_yaw / 2.0)

        # 4. Send Goal
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = [goal_pose]

        self.get_logger().info(
            f"Iteration [{self.iterations_left}]: Sending goal -> "
            f"x: {goal_pose.pose.position.x:.2f}, y: {goal_pose.pose.position.y:.2f}, "
            f"yaw offset: {dyaw:.2f} rad"
        )

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateThroughPoses action server not available!")
            self.is_navigating = False
            return

        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by server.")
            self.is_navigating = False
            return

        self.get_logger().info("Goal accepted by server, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        # Action Status 4 = SUCCEEDED
        if status == 4:
            self.get_logger().info("Goal reached successfully!")
        else:
            self.get_logger().warn(f"Goal failed or canceled! Status code: {status}")

        # 성공/실패 여부와 관계없이 다음 이터레이션 진행 (원할 경우 실패 시 중단하도록 수정 가능)
        self.iterations_left -= 1
        self.execute_next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = RandomNavTester()
    
    # Action Client와 Subscriber가 비동기로 동작할 수 있도록 MultiThreadedExecutor 사용
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.\n")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
