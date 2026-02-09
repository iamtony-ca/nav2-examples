#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
from example_interfaces.srv import AddTwoInts
from functools import partial

class AsyncTimeoutClient(Node):

    def __init__(self):
        super().__init__('async_timeout_client')
        self.cb_group = ReentrantCallbackGroup()
        self.sub = self.create_subscription(Bool, 'trigger_service', self.trigger_callback, 10, callback_group=self.cb_group)
        self.cli = self.create_client(AddTwoInts, 'add_two_ints', callback_group=self.cb_group)

    def trigger_callback(self, msg):
        self.get_logger().info('[Trigger] Sending Async Request...')
        req = AddTwoInts.Request()
        req.a, req.b = 10, 20
        
        future = self.cli.call_async(req)

        # [1] 타이머 생성 (아직 start는 아님, 생성되자마자 돔)
        # partial을 사용해 future를 넘겨줍니다.
        timer = self.create_timer(
            2.0, # 2초 타임아웃
            lambda: self.on_timeout(future, timer),
            callback_group=self.cb_group
        )

        # [2] Done Callback에 timer 정보도 같이 넘깁니다.
        future.add_done_callback(
            partial(self.on_response, timer=timer)
        )

    def on_response(self, future, timer):
        """응답이 오면 실행"""
        # 가장 먼저 타이머를 죽입니다 (취소)
        if not timer.is_canceled():
            timer.cancel()
            self.destroy_timer(timer)

        if future.cancelled():
            # 타임아웃에 의해 취소된 경우
            return

        try:
            result = future.result()
            self.get_logger().info(f'[Success] {result.sum}')
        except Exception as e:
            self.get_logger().error(f'[Error] {e}')

    def on_timeout(self, future, timer):
        """타이머가 울리면 실행"""
        self.get_logger().error('[Timeout] Request timed out!')
        
        # Future 취소
        future.cancel()
        
        # 타이머 자기 자신도 취소 (반복 실행 방지)
        timer.cancel()
        self.destroy_timer(timer)

def main(args=None):
    rclpy.init(args=args)
    node = AsyncTimeoutClient()
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