#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Path # (추가) Path 메시지 임포트

class PathValidator(Node):
    def __init__(self):
        super().__init__('path_validator_node')
        self._action_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        
        self.subscription = self.create_subscription(
            PoseArray,
            '/validate_path_request',
            self.listener_callback,
            10)
        
        # (추가) 계산된 경로를 발행할 퍼블리셔 생성
        self.path_publisher = self.create_publisher(Path, '/validated_path', 10)
        
        self.current_goal_msg = None

        self.get_logger().info("Path Validator Node is running.")
        self.get_logger().info("Waiting for /compute_path_to_pose action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server is ready. Waiting for path requests on /validate_path_request...")

    # ... listener_callback, validate_path, goal_response_callback 함수는 이전과 동일 ...
    def listener_callback(self, msg: PoseArray):
        if len(msg.poses) < 2:
            self.get_logger().warn("Received PoseArray must contain at least 2 poses (start and goal).")
            return
        start_pose = PoseStamped(header=msg.header, pose=msg.poses[0])
        goal_pose = PoseStamped(header=msg.header, pose=msg.poses[1])
        self.get_logger().info("Received path request.")
        self.validate_path(start_pose, goal_pose)

    def validate_path(self, start_pose, goal_pose):
        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start_pose
        goal_msg.goal = goal_pose
        goal_msg.use_start = True
        goal_msg.planner_id = "GridBased"
        self.current_goal_msg = goal_msg
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        if not self.current_goal_msg:
            self.get_logger().error("Could not find the goal message for this result.")
            return

        result = future.result().result
        start_pose = self.current_goal_msg.start.pose
        goal_pose = self.current_goal_msg.goal.pose
        start_str = f"({start_pose.position.x:.2f}, {start_pose.position.y:.2f})"
        goal_str = f"({goal_pose.position.x:.2f}, {goal_pose.position.y:.2f})"

        if result.path and len(result.path.poses) > 0:
            self.get_logger().info(f" SUCCESS: Path from {start_str} to {goal_str} is VALID.")
            # (추가) 성공한 경로를 /validated_path 토픽으로 발행
            self.path_publisher.publish(result.path)
        else:
            self.get_logger().warn(f" FAILURE: Path from {start_str} to {goal_str} is INVALID.")

# ... main 함수는 이전과 동일 ...
def main(args=None):
    rclpy.init(args=args)
    path_validator_node = PathValidator()
    rclpy.spin(path_validator_node)
    path_validator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()