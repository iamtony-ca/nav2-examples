#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from robot_interfaces.msg import NavigationCommand

class NavigationCommandPublisher(Node):
    def __init__(self):
        super().__init__('navigation_command_publisher')
        
        # Publisher 생성 (QoS depth = 10)
        self.publisher_ = self.create_publisher(
            NavigationCommand,
            'move_command',  # Subscriber가 구독 중인 토픽 이름에 맞게 수정하세요
            10
        )

    def publish_command(self):
        msg = NavigationCommand()
        msg.goal_cnt = 2
        msg.cmd_seq_num = 1

        # 첫 번째 목표 Pose (예: x=1.0, y=0.0)
        pose1 = Pose()
        pose1.position = Point(x=1.0, y=0.0, z=0.0)
        pose1.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # 두 번째 목표 Pose (예: x=2.0, y=1.0, 90도 회전)
        pose2 = Pose()
        pose2.position = Point(x=3.0, y=1.0, z=0.0)
        pose2.orientation = Quaternion(x=0.0, y=0.0, z=0.707, w=0.707)

        # 메시지 배열 데이터 채우기
        msg.goal_poses = [pose1, pose2]
        msg.from_node_id = [100, 101]
        msg.to_node_id = [101, 102]

        # Subscriber가 연결될 때까지 대기 (One-shot 퍼블리시 데이터 유실 방지)
        self.get_logger().info('Waiting for subscribers...')
        while self.publisher_.get_subscription_count() == 0:
            if not rclpy.ok():
                return
            rclpy.spin_once(self, timeout_sec=0.1)

        # Subscriber가 확인되면 메시지 발행
        self.publisher_.publish(msg)
        self.get_logger().info('Successfully published NavigationCommand with 2 goals.')

def main(args=None):
    rclpy.init(args=args)
    node = NavigationCommandPublisher()

    try:
        node.publish_command()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()