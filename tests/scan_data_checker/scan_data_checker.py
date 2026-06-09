import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class LidarRegionMonitor(Node):
    """지정된 map 좌표(x, y) 주변에 PointCloud2 데이터(장애물)가 존재하는지 모니터링하는 노드"""

    def __init__(self) -> None:
        super().__init__('lidar_region_monitor')
        
        # 파라미터 선언 (런타임에 모니터링할 좌표와 토픽 변경 가능)
        self.declare_parameter('topic_name', '/scan_matched_points2')  # 실제 사용하는 토픽명으로 변경
        self.declare_parameter('target_x', 3.8854)                # 확인할 map 기준 x 좌표
        self.declare_parameter('target_y', -6.7145)                # 확인할 map 기준 y 좌표
        self.declare_parameter('tolerance', 0.3)               # 검색 반경 (m)
        
        # 파라미터 값 가져오기
        topic_name = self.get_parameter('topic_name').value
        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.tolerance = self.get_parameter('tolerance').value
        
        # Subscriber 생성
        self.subscription = self.create_subscription(
            PointCloud2,
            topic_name,
            self.lidar_callback,
            10
        )
        
        self.get_logger().info(
            f"LiDAR 모니터링 시작. 타겟 좌표: ({self.target_x}, {self.target_y}), 검색 반경: {self.tolerance}m"
        )

    def lidar_callback(self, msg: PointCloud2) -> None:
        # 데이터 자체가 비어있는지 1차 확인
        if msg.width == 0 or msg.height == 0:
            self.get_logger().warn("PointCloud2 메시지가 비어 있습니다. (width/height = 0)")
            return

        # sensor_msgs_py를 사용하여 byte 배열에서 x, y 값 추출 (NaN 무시)
        points_iter = pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)
        
        # [수정된 부분] Generator 데이터를 명시적인 2차원 리스트 [[x, y], [x, y], ...] 형태로 강제 언패킹
        points_list = [[p[0], p[1]] for p in points_iter]

        if not points_list:
            self.get_logger().warn("유효한(NaN이 아닌) PointCloud 데이터가 없습니다.")
            return

        # Numpy float32 타입의 2차원 배열로 확실하게 변환 (shape: N x 2)
        points = np.array(points_list, dtype=np.float32)

        # 연산 속도를 위해 Bounding Box(사각형 영역) 방식으로 필터링 적용
        min_x = self.target_x - self.tolerance
        max_x = self.target_x + self.tolerance
        min_y = self.target_y - self.tolerance
        max_y = self.target_y + self.tolerance

        # 2차원 배열(N, 2)이므로 이제 에러 없이 인덱싱이 작동합니다.
        # points[:, 0]은 x좌표, points[:, 1]은 y좌표
        in_region = points[
            (points[:, 0] >= min_x) & (points[:, 0] <= max_x) &
            (points[:, 1] >= min_y) & (points[:, 1] <= max_y)
        ]

        # 지정된 영역 안에 데이터가 없다면 장애물이 없는 것으로 간주하고 Log 출력
        if len(in_region) == 0:
            self.get_logger().warn(
                f"[장애물 미감지] (x: {self.target_x}, y: {self.target_y}) 반경 {self.tolerance}m 내에 데이터가 없습니다."
            )
        else:
            # 데이터가 정상적으로 들어오는 경우 debug 레벨로 출력
            self.get_logger().info(
                f"[장애물 감지] 영역 내 Point 개수: {len(in_region)}"
            )

def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarRegionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
