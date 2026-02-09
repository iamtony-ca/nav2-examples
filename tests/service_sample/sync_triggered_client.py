# sync_triggered_client.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
from example_interfaces.srv import AddTwoInts

class SyncTriggeredClient(Node):

    def __init__(self):
        super().__init__('sync_triggered_client')

        # 중요 1: ReentrantCallbackGroup 생성
        # 이 그룹에 속한 콜백들은 멀티스레드 환경에서 병렬 실행이 허용됩니다.
        self.callback_group = ReentrantCallbackGroup()

        # 1. Trigger Topic Subscriber (콜백 그룹 지정)
        self.subscription = self.create_subscription(
            Bool,
            'trigger_service',
            self.trigger_callback,
            10,
            callback_group=self.callback_group) # 필수 설정
        
        # 2. Service Client (콜백 그룹 지정)
        self.cli = self.create_client(
            AddTwoInts, 
            'add_two_ints', 
            callback_group=self.callback_group) # 필수 설정
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.get_logger().info('Sync Client Ready (Multi-threaded). Waiting for /trigger_service...')

    def trigger_callback(self, msg):
        """
        이 콜백은 별도의 스레드에서 실행됩니다.
        따라서 여기서 block이 걸려도, 다른 스레드가 서비스 응답을 받을 수 있습니다.
        """
        if msg.data:
            self.get_logger().info('Trigger received! Sending Sync Request (Blocking)...')
            
            req = AddTwoInts.Request()
            req.a = 50
            req.b = 50
            
            # 동기 호출 (Block)
            # MultiThreadedExecutor 덕분에 여기서 멈춰도 데드락이 걸리지 않습니다.
            response = self.cli.call(req)
            
            self.get_logger().info(f'[Result] {req.a} + {req.b} = {response.sum}')

def main(args=None):
    rclpy.init(args=args)
    node = SyncTriggeredClient()
    
    # 중요 2: MultiThreadedExecutor 사용
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()