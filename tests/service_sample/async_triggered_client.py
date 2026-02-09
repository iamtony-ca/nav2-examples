# async_triggered_client.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from example_interfaces.srv import AddTwoInts
from functools import partial

class AsyncTriggeredClient(Node):

    def __init__(self):
        super().__init__('async_triggered_client')

        # 1. Trigger Topic Subscriber
        self.subscription = self.create_subscription(
            Bool,
            'trigger_service',
            self.trigger_callback,
            10)
        
        # 2. Service Client
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        
        # 서비스 서버 대기
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
            
        self.get_logger().info('Async Client Ready. Waiting for /trigger_service topic...')

    def trigger_callback(self, msg):
        """
        토픽이 들어오면 실행되는 콜백.
        여기서 멈추지(block) 않고 비동기 요청을 보냅니다.
        """
        if msg.data:
            self.get_logger().info('Trigger received! Sending Async Request...')
            
            req = AddTwoInts.Request()
            req.a = 10
            req.b = 20
            
            # 비동기 호출
            future = self.cli.call_async(req)
            
            # 중요: 결과가 도착했을 때 실행될 콜백을 등록합니다.
            # 이렇게 하면 메인 루프가 멈추지 않습니다.
            future.add_done_callback(partial(self.response_callback, a=req.a, b=req.b))

    def response_callback(self, future, a, b):
        """
        서비스 응답이 도착하면 호출되는 콜백
        """
        try:
            response = future.result()
            self.get_logger().info(f'[Result] {a} + {b} = {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = AsyncTriggeredClient()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()