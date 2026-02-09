#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
from example_interfaces.srv import AddTwoInts
from functools import partial

class AsyncTimeoutSafeTest(Node):

    def __init__(self):
        super().__init__('async_timeout_safe_test')

        # [중요] ReentrantCallbackGroup
        # 이 그룹을 사용해야 Timer와 Service Client가 서로 다른 스레드에서 병렬로 실행됩니다.
        self.cb_group = ReentrantCallbackGroup()

        # 1. Trigger Subscriber (테스트 시작용)
        self.sub = self.create_subscription(
            Bool, 'trigger_service', self.trigger_callback, 10, 
            callback_group=self.cb_group)
        
        # 2. Service Client
        self.cli = self.create_client(
            AddTwoInts, 'add_two_ints', 
            callback_group=self.cb_group)

        # 3. [검증용] Non-blocking 확인 Timer
        # 서비스 요청을 보내고 응답을 기다리는 와중에도, 이 타이머는 절대 멈추면 안 됩니다.
        self.check_timer = self.create_timer(
            0.5, self.timer_callback, 
            callback_group=self.cb_group)

        # 서비스 서버 대기
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.get_logger().info('Node Ready. Publish /trigger_service to test.')

    def timer_callback(self):
        """
        [검증용] 주기적으로 실행되어 노드가 멈추지(Block) 않았음을 증명합니다.
        """
        self.get_logger().info('Beep... (I am alive & Non-blocking)', throttle_duration_sec=1.0)

    def trigger_callback(self, msg):
        """
        Trigger 토픽이 들어오면 실행됩니다.
        여기서 절대 time.sleep()이나 while 루프로 기다리지 않습니다.
        """
        self.get_logger().info('\n[Trigger] Sending Async Request...')
        
        req = AddTwoInts.Request()
        req.a = 10
        req.b = 20
        
        # 1. 비동기 요청 전송 (즉시 future 반환)
        future = self.cli.call_async(req)
        
        # 2. [타임아웃 로직]
        # 타임아웃 발생 시 실행될 내부 함수 정의
        # 리스트(mutable)를 이용해 timer 변수를 캡처할 준비를 합니다.
        timer_holder = []

        def on_timeout():
            self.get_logger().error('[Timeout] 2.0 seconds passed! Cancelling request...')
            
            # (1) Future 취소 요청 -> 이것이 on_response를 유발할 수 있음
            future.cancel()
            
            # (2) 타임아웃 타이머 정지 (반복 실행 방지)
            if timer_holder:
                timer_holder[0].cancel() 
                # 주의: destroy_timer는 여기서 호출하면 위험할 수 있으므로 cancel만 합니다.

        # 3. 타임아웃 타이머 생성 (2초 설정)
        timeout_timer = self.create_timer(
            2.0, 
            on_timeout, 
            callback_group=self.cb_group
        )
        timer_holder.append(timeout_timer) # 핸들 저장

        # 4. 결과 처리 콜백 등록 (Non-blocking의 핵심)
        # partial을 이용해 위에서 만든 timeout_timer를 전달합니다.
        future.add_done_callback(
            partial(self.on_response, timer=timeout_timer)
        )
        
        self.get_logger().info('[Trigger] Function finished. Returning to main loop...\n')

    def on_response(self, future, timer):
        """
        서비스 응답이 왔거나, 취소되었을 때 실행되는 콜백
        """
        # [방어 로직 1] 타임아웃 타이머 정리
        # 응답이 왔으니 타임아웃 알람은 필요 없습니다. 끕니다.
        try:
            timer.cancel()
            # self.destroy_timer(timer) # 안전을 위해 생략 (GC에 맡김)
        except Exception:
            pass

        # [방어 로직 2] 취소 여부 확인
        # 타임아웃 핸들러에 의해 취소된 요청이라면, 결과를 처리하지 않고 종료합니다.
        if future.cancelled():
            self.get_logger().warn('[Result] Request was cancelled. Ignoring result logic.')
            return

        # [정상 결과 처리]
        try:
            response = future.result()
            self.get_logger().info(f'[Success] Response received: {response.sum}')
        except Exception as e:
            self.get_logger().error(f'[Result] Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = AsyncTimeoutSafeTest()
    
    # [필수] MultiThreadedExecutor
    # 이것이 있어야 Timer, Trigger, Service Response가 동시에 처리됩니다.
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