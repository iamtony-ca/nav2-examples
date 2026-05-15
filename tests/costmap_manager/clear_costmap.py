#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from nav2_msgs.srv import ClearEntireCostmap


class RobustCostmapClearer(Node):
    def __init__(self):
        super().__init__('robust_costmap_clearer_node')
        
        # bt.xml에 명시된 서비스 이름과 정확히 동일하게 클라이언트 생성
        self.client_local = self.create_client(
            ClearEntireCostmap, 
            '/local_costmap/clear_entirely_local_costmap'
        )
        self.client_global = self.create_client(
            ClearEntireCostmap, 
            '/global_costmap/clear_entirely_global_costmap'
        )

    def wait_for_services(self) -> bool:
        """두 Costmap 서비스가 모두 활성화될 때까지 무한 대기합니다."""
        self.get_logger().info('Waiting for costmap clear services to become active...')
        
        # local_costmap 대기
        while not self.client_local.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for local_costmap service.')
                return False
            self.get_logger().info('Still waiting for /local_costmap/clear_entirely_local_costmap...')
            
        # global_costmap 대기
        while not self.client_global.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for global_costmap service.')
                return False
            self.get_logger().info('Still waiting for /global_costmap/clear_entirely_global_costmap...')
            
        self.get_logger().info('Both costmap services are now ready!')
        return True

    def clear_both_costmaps(self):
        """두 Costmap을 모두 초기화합니다."""
        if not self.wait_for_services():
            return

        req = ClearEntireCostmap.Request()

        # Local Costmap 초기화 요청
        future_local = self.client_local.call_async(req)
        rclpy.spin_until_future_complete(self, future_local)
        if future_local.result() is not None:
            self.get_logger().info('Successfully cleared Local Costmap.')
        else:
            self.get_logger().error('Failed to clear Local Costmap.')

        # Global Costmap 초기화 요청
        future_global = self.client_global.call_async(req)
        rclpy.spin_until_future_complete(self, future_global)
        if future_global.result() is not None:
            self.get_logger().info('Successfully cleared Global Costmap.')
        else:
            self.get_logger().error('Failed to clear Global Costmap.')


def main(args=None):
    rclpy.init(args=args)
    clearer = RobustCostmapClearer()

    try:
        clearer.clear_both_costmaps()
    except KeyboardInterrupt:
        clearer.get_logger().info('Interrupted by user.')
    finally:
        clearer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()