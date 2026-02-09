#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
from example_interfaces.srv import AddTwoInts
from functools import partial

class AsyncTimeoutClientSafe(Node):

    def __init__(self):
        super().__init__('async_timeout_client_safe')
        self.cb_group = ReentrantCallbackGroup()
        
        self.sub = self.create_subscription(
            Bool, 'trigger_service', self.trigger_callback, 10, 
            callback_group=self.cb_group)
        
        self.cli = self.create_client(
            AddTwoInts, 'add_two_ints', 
            callback_group=self.cb_group)

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def trigger_callback(self, msg):
        self.get_logger().info('[Trigger] Sending Async Request...')
        
        req = AddTwoInts.Request()
        req.a = 10
        req.b = 20
        
        # 1. 요청 전송
        future = self.cli.call_async(req)
        
        # 2. 타이머 생성 (변수에 담아둠)
        # 주의: lambda 안에서 timer 변수를 쓰려면, timer 생성 후 바인딩하거나
        # 별도의 클래스 메서드로 관리해야 합니다. 
        # 여기서는 가장 안전한 방법인 "Mutable Container(List)" 트릭을 씁니다.
        timer_holder = []
        
        def timeout_handler():
            # 타임아웃 발생 시 실행되는 내부 함수
            self.get_logger().error('[Timeout] Request timed out!')
            
            # (1) Future를 취소합니다.
            # 이것이 on_response를 유발할 수도 있지만, on_response에서 방어할 것입니다.
            future.cancel()
            
            # (2) 타이머를 멈춥니다.
            if timer_holder:
                timer = timer_holder[0]
                timer.cancel()
                # 안전을 위해 destroy는 생략하거나 try-except로 감쌉니다.
                # self.destroy_timer(timer) 

        # 3. 실제 타이머 생성
        timer = self.create_timer(
            2.0, # 2초 타임아웃
            timeout_handler,
            callback_group=self.cb_group
        )
        timer_holder.append(timer) # 리스트에 담아서 handler가 접근 가능하게 함

        # 4. 결과 콜백 등록
        future.add_done_callback(
            partial(self.on_response, timer=timer)
        )

    def on_response(self, future, timer):
        """
        서비스 응답이 왔을 때 (성공이든, 취소든)
        """
        # [핵심 방어 로직 1]
        # 만약 타임아웃 핸들러에 의해 future가 취소된 상태라면,
        # 이미 타임아웃 쪽에서 정리를 시작했으므로 여기서는 아무것도 하지 말고 "빠져나가야" 합니다.
        if future.cancelled():
            # self.get_logger().warn('[Result] Future was cancelled. Ignoring response logic.')
            return

        # [핵심 방어 로직 2]
        # 정상 응답이 왔으므로, 타임아웃 타이머를 끕니다.
        # destroy_timer 대신 cancel()을 쓰는 것이 훨씬 안전합니다.
        try:
            timer.cancel()
        except Exception:
            pass # 이미 취소되었거나 삭제되었으면 무시

        # 결과 처리
        try:
            response = future.result()
            self.get_logger().info(f'[Success] {response.sum}')
        except Exception as e:
            self.get_logger().error(f'[Result] Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = AsyncTimeoutClientSafe()
    
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