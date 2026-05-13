import rclpy
from rclpy.node import Node
from nav2_msgs.msg import BehaviorTreeLog, BehaviorTreeStatusChange
import itertools


class BTLogPublisher(Node):
    def __init__(self):
        super().__init__('bt_log_publisher_node')
        
        # Publisher 생성 (토픽명: /behavior_tree_log)
        self.publisher_ = self.create_publisher(
            BehaviorTreeLog, 
            'behavior_tree_log', 
            10
        )
        
        # 1초 주기로 퍼블리시하는 타이머 생성
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # BehaviorTree.CPP 상태 사이클 (테스트용)
        self.statuses = ['IDLE', 'RUNNING', 'SUCCESS', 'FAILURE']
        self.status_cycle = itertools.cycle(self.statuses)
        self.current_status = next(self.status_cycle)

    def timer_callback(self):
        # 1. 최상위 Log 메시지 생성
        log_msg = BehaviorTreeLog()
        log_msg.timestamp = self.get_clock().now().to_msg()
        
        # 2. 개별 노드의 상태 변화 이벤트 생성
        event = BehaviorTreeStatusChange()
        event.timestamp = log_msg.timestamp
        event.node_name = "NavigationManagerReady"
        
        # 이전 상태 기록
        event.previous_status = self.current_status
        
        # 다음 상태로 업데이트 후 현재 상태 기록
        self.current_status = next(self.status_cycle)
        event.current_status = self.current_status
        
        # 3. Log 메시지의 event_log 배열에 추가
        log_msg.event_log.append(event)
        
        # 퍼블리시 및 로그 출력
        self.publisher_.publish(log_msg)
        self.get_logger().info(
            f'Published: [{event.node_name}] '
            f'{event.previous_status} -> {event.current_status}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = BTLogPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
