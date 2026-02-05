#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import Costmap
from geometry_msgs.msg import PointStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class CostExplorerNode(Node):
    def __init__(self):
        super().__init__('cost_explorer_node')
        self.costmap = None
        # 사전 정의 테스트 실행 여부를 추적하는 플래그
        self.predefined_tests_done = False

        # 1. Costmap 구독 (Transient Local 설정 유지)
        costmap_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.costmap_sub = self.create_subscription(
            Costmap,
            '/global_costmap/costmap_raw',
            self.costmap_callback,
            costmap_qos
        )

        # 2. RViz Clicked Point 구독
        self.clicked_point_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10
        )

        self.get_logger().info("CostExplorerNode 가동. 코스트맵 데이터를 기다리는 중...")

    def costmap_callback(self, msg):
        """코스트맵 메시지 수신 시 호출"""
        self.costmap = msg
        
        # 첫 번째 메시지를 받으면 사전 정의된 테스트를 자동으로 실행
        if not self.predefined_tests_done:
            self.run_predefined_tests()
            self.predefined_tests_done = True

    def clicked_point_callback(self, msg):
        """RViz에서 지점을 클릭했을 때 호출 (실시간 인터랙션)"""
        if self.costmap is None:
            return
        self.log_cost_at_point(msg.point.x, msg.point.y)

    def run_predefined_tests(self):
        """[신규] 사전에 입력된 좌표 및 영역에 대한 테스트 로깅 메서드"""
        self.get_logger().info("="*50)
        self.get_logger().info("🚀 사전 정의된 위치 및 영역 조회를 시작합니다.")
        
        # 1. 단일 지점 조회 테스트
        self.log_cost_at_point(1.0, 1.0)
        self.log_cost_at_point(5.0, 5.0)

        # 2. 영역 조회 테스트 (예: (1.5, 2.0) 부터 (2.0, 2.5) 까지의 영역)
        self.log_costs_in_region(1.5, 2.0, 2.0, 2.5)
        
        # 3. 맵 밖의 영역 테스트
        self.log_costs_in_region(1000.0, 1000.0, 1001.0, 1001.0)
        
        self.get_logger().info("✅ 사전 정의 테스트 완료. 이제 RViz에서 'Publish Point'를 사용해 보세요.")
        self.get_logger().info("="*50)

    def world_to_grid(self, wx, wy):
        """World 좌표(m) -> Grid 좌표(cell index) 변환"""
        info = self.costmap.metadata
        gx = int((wx - info.origin.position.x) / info.resolution)
        gy = int((wy - info.origin.position.y) / info.resolution)

        if 0 <= gx < info.size_x and 0 <= gy < info.size_y:
            return (gx, gy)
        return None

    def log_cost_at_point(self, wx, wy):
        """단일 지점의 cost 로깅"""
        grid_pos = self.world_to_grid(wx, wy)
        if grid_pos:
            gx, gy = grid_pos
            cost = self.costmap.data[gy * self.costmap.metadata.size_x + gx]
            self.get_logger().info(f"📍 [Point] World:({wx:.2f}, {wy:.2f}) -> Grid:[{gx}, {gy}] | Cost: {cost}")
        else:
            self.get_logger().warn(f"⚠️ [Point] Out of bounds: ({wx:.2f}, {wy:.2f})")

    def log_costs_in_region(self, wx1, wy1, wx2, wy2):
        """특정 사각 영역의 모든 cost 로깅"""
        self.get_logger().info(f"🟦 [Region] ({wx1:.2f}, {wy1:.2f}) ~ ({wx2:.2f}, {wy2:.2f})")
        
        gp1 = self.world_to_grid(wx1, wy1)
        gp2 = self.world_to_grid(wx2, wy2)

        if not gp1 or not gp2:
            self.get_logger().warn("   -> 영역의 일부가 코스트맵 범위를 벗어났습니다.")
            return

        sx, ex = min(gp1[0], gp2[0]), max(gp1[0], gp2[0])
        sy, ey = min(gp1[1], gp2[1]), max(gp1[1], gp2[1])

        for y in range(sy, ey + 1):
            row = [str(self.costmap.data[y * self.costmap.metadata.size_x + x]) for x in range(sx, ex + 1)]
            self.get_logger().info(f"   Row {y:03d}: [{' '.join(row)}]")

def main(args=None):
    rclpy.init(args=args)
    node = CostExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()