#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import Costmap
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math

class CostExplorerNode(Node):
    """
    Costmap 데이터를 구독하여 특정 world 좌표나 영역의 cost 값을 로깅하는 노드.
    /global_costmap/costmap_raw (nav2_msgs/msg/Costmap) 토픽을 사용합니다.
    [수정됨] width/height 대신 size_x/size_y 사용
    """

    def __init__(self):
        super().__init__('cost_explorer_node')
        self.costmap = None

        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.subscription = self.create_subscription(
            Costmap,
            '/global_costmap/costmap_raw',
            self.costmap_callback,
            qos_profile
        )
        self.get_logger().info("CostExplorerNode 시작. '/global_costmap/costmap_raw' 대기 중...")

    def costmap_callback(self, msg):
        """Costmap 메시지를 수신하면 노드의 멤버 변수에 저장합니다."""
        if self.costmap is None:
            # [수정] msg.metadata.width -> msg.metadata.size_x
            # [수정] msg.metadata.height -> msg.metadata.size_y
            self.get_logger().info(f"Costmap 수신 완료! (크기: {msg.metadata.size_x}x{msg.metadata.size_y})")
        self.costmap = msg

    def world_to_grid(self, wx, wy):
        """World 좌표(m)를 Grid 좌표(셀 인덱스)로 변환합니다."""
        if not self.costmap:
            return None

        info = self.costmap.metadata
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        resolution = info.resolution

        gx = int((wx - origin_x) / resolution)
        gy = int((wy - origin_y) / resolution)

        # [수정] info.width -> info.size_x
        # [수정] info.height -> info.size_y
        if 0 <= gx < info.size_x and 0 <= gy < info.size_y:
            return (gx, gy)
        else:
            return None

    def get_grid_cost(self, gx, gy):
        """Grid 좌표의 cost 값을 1D 데이터 배열에서 가져옵니다."""
        if not self.costmap:
            return -1

        info = self.costmap.metadata
        # [수정] info.width -> info.size_x
        index = gy * info.size_x + gx

        if 0 <= index < len(self.costmap.data):
            return self.costmap.data[index]
        else:
            return -1

    def log_cost_at_point(self, world_x, world_y):
        """[요청 1] 단일 world 좌표의 cost 값을 로깅합니다."""
        if not self.costmap:
            self.get_logger().warn("Costmap이 아직 수신되지 않았습니다.")
            return

        self.get_logger().info(f"--- [단일 지점] World ({world_x:.2f}, {world_y:.2f}) 조회 ---")
        grid_pos = self.world_to_grid(world_x, world_y)

        if grid_pos:
            gx, gy = grid_pos
            cost = self.get_grid_cost(gx, gy)
            self.get_logger().info(f"  -> Grid (col={gx}, row={gy}), Cost = {cost}")
        else:
            self.get_logger().warn("  -> 위치가 Costmap 범위 밖입니다.")

    def log_costs_in_region(self, world_x1, world_y1, world_x2, world_y2):
        """
        [요청 2] 두 world 좌표로 정의된 사각 영역의 모든 cost 값을 로깅합니다.
        """
        if not self.costmap:
            self.get_logger().warn("Costmap이 아직 수신되지 않았습니다.")
            return

        self.get_logger().info(
            f"--- [영역 조회] World [({world_x1:.2f}, {world_y1:.2f}) "
            f"to ({world_x2:.2f}, {world_y2:.2f})] ---"
        )

        grid_p1 = self.world_to_grid(world_x1, world_y1)
        grid_p2 = self.world_to_grid(world_x2, world_y2)

        if not grid_p1 or not grid_p2:
            self.get_logger().warn("  -> 영역의 일부 또는 전체가 Costmap 범위 밖입니다.")
            return

        gx1, gy1 = grid_p1
        gx2, gy2 = grid_p2

        start_x = min(gx1, gx2)
        end_x = max(gx1, gx2)
        start_y = min(gy1, gy2)
        end_y = max(gy1, gy2)

        self.get_logger().info(f"  -> Grid (col=[{start_x}...{end_x}], row=[{start_y}...{end_y}])")

        for y in range(start_y, end_y + 1):
            row_costs = []
            for x in range(start_x, end_x + 1):
                cost = self.get_grid_cost(x, y)
                row_costs.append(str(cost))
            
            self.get_logger().info(f"  Row {y:03d}: [{' '.join(row_costs)}]")


def main(args=None):
    rclpy.init(args=args)
    node = CostExplorerNode()

    timeout_sec = 10.0
    start_time = node.get_clock().now()
    while rclpy.ok() and node.costmap is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        if (node.get_clock().now() - start_time).nanoseconds * 1e-9 > timeout_sec:
            node.get_logger().error("타임아웃: Costmap을 수신하지 못했습니다. Nav2가 실행 중인지, 토픽 이름이 맞는지 확인하세요.")
            node.destroy_node()
            rclpy.shutdown()
            return

    if node.costmap:
        # --- 테스트할 좌표를 여기에 입력 ---

        # 1. 단일 지점 조회 테스트
        node.log_cost_at_point(1.0, 1.0)
        node.log_cost_at_point(5.0, 5.0)

        # 2. 영역 조회 테스트 (예: (1.5, 2.0) 부터 (2.0, 2.5) 까지의 영역)
        node.log_costs_in_region(1.5, 2.0, 2.0, 2.5)
        
        # 3. 맵 밖의 영역 테스트
        node.log_costs_in_region(1000.0, 1000.0, 1001.0, 1001.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()