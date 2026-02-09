#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
from example_interfaces.srv import AddTwoInts
import time

class WaitInCallbackNode(Node):

    def __init__(self):
        super().__init__('wait_in_callback_node')

        # [중요 1] ReentrantCallbackGroup 사용
        # 이 그룹에 속한 콜백들은 서로 다른 스레드에서 동시에 실행될 수 있습니다.
        # 즉, 내가 여기서 멈춰(Wait) 있어도, 다른 스레드가 내 서비스 응답을 받아줄 수 있습니다.
        self.cb_group = ReentrantCallbackGroup()

        # Trigger Subscriber
        self.sub = self.create_subscription(
            Bool, 'trigger', self.trigger_callback, 10, 
            callback_group=self.cb_group)

        # Service Client
        self.cli = self.create_client(
            AddTwoInts, 'add_two_ints', 
            callback_group=self.cb_group)
        
        # (테스트용) 다른 작업이 잘 도는지 확인하기 위한 Timer
        self.timer = self.create_timer(
            0.5, self.timer_callback, 
            callback_group=self.cb_group)

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def timer_callback(self):
        """
        Trigger 콜백이 멈춰있는 동안에도 이 타이머는 계속 동작해야 함.
        """
        self.get_logger().info('Beep... (Other callbacks are alive)', throttle_duration_sec=2.0)



    def trigger_callback(self, msg):
        self.get_logger().info('[Trigger] Request sent. Waiting for response inside callback...')

        req = AddTwoInts.Request()
        req.a = 10
        req.b = 20

        # 1. Async 호출
        future = self.cli.call_async(req)

        # 2. 결과 기다리기 (수동 타임아웃 구현)
        timeout_sec = 5.0
        start_time = time.time()

        # future가 완료될 때까지 루프를 돕니다.
        while not future.done():
            # 타임아웃 체크
            if time.time() - start_time > timeout_sec:
                self.get_logger().error('[Trigger] Service timed out!')
                # 필요 시 future 취소 (선택 사항)
                future.cancel()
                return

            # 중요: 0.01초 정도 쉬어주어야 CPU를 독점하지 않고,
            # MultiThreadedExecutor의 다른 스레드가 서비스 응답 처리를 할 수 있습니다.
            time.sleep(0.01)

        # 3. 루프를 빠져나왔다면 결과가 도착했다는 뜻입니다.
        try:
            response = future.result() # 이제 인자 없이 호출해도 안전합니다.
            self.get_logger().info(f'[Trigger] Response received! Sum: {response.sum}')
            self.get_logger().info('[Trigger] Continuing next logic...')
            
        except Exception as e:
            self.get_logger().error(f'[Trigger] Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = WaitInCallbackNode()

    # [중요 2] MultiThreadedExecutor 필수
    # 기본(Single) Executor를 쓰면 future.result()에서 영원히 멈춥니다 (Deadlock).
    # 최소 2개 이상의 스레드가 있어야 "기다리는 놈"과 "받아오는 놈"이 공존할 수 있습니다.
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