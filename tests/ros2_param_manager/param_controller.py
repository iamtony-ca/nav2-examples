#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter_client import AsyncParameterClient
from rclpy.parameter import parameter_value_to_python

# Trigger 서비스와 파라미터 메시지 import
from std_srvs.srv import Trigger
from rcl_interfaces.msg import ParameterDescriptor

class ParamEventNode(Node):
    def __init__(self):
        super().__init__('param_event_node')
        
        self.target_node_name = 'param_server_node'
        
        # [Expert Point] ReentrantCallbackGroup 사용
        # 이 그룹에 속한 콜백들은 병렬로 실행될 수 있습니다. 
        # 즉, trigger_callback 내부에서 다른 서비스(param)를 기다려도 멈추지 않습니다.
        self.cb_group = ReentrantCallbackGroup()

        # 1. Parameter Client 생성 (callback_group 설정 필수)
        self.cli = AsyncParameterClient(
            self, 
            self.target_node_name, 
            callback_group=self.cb_group
        )

        # 2. Trigger Service Server 생성 (이벤트를 받는 부분)
        self.srv = self.create_service(
            Trigger, 
            'trigger_param_update', 
            self.trigger_callback, 
            callback_group=self.cb_group
        )

        self.get_logger().info(f'Event Node Ready. Call /trigger_param_update service to start.')

    async def trigger_callback(self, request, response):
        """
        외부에서 서비스 호출이 오면 실행되는 콜백 (async)
        """
        self.get_logger().info("!!! Trigger Received !!! Starting Sequence...")

        # 서비스 연결 확인
        if not self.cli.wait_for_services(timeout_sec=2.0):
            response.success = False
            response.message = "Target node services not available"
            return response

        try:
            # Step 1: 초기값 조회
            self.get_logger().info('[1] Checking initial value...')
            val1 = await self.get_target_param('target_speed')

            # Step 2: 값 변경
            self.get_logger().info('[2] Setting new value to 7.7...')
            set_success = await self.set_target_param('target_speed', 7.7)
            
            if not set_success:
                raise Exception("Failed to set parameter")

            # Step 3: 변경된 값 조회
            self.get_logger().info('[3] Verifying new value...')
            val2 = await self.get_target_param('target_speed')

            # 결과 리턴
            response.success = True
            response.message = f"Sequence Complete. {val1} -> {val2}"
            self.get_logger().info("!!! Sequence Finished !!!")

        except Exception as e:
            self.get_logger().error(f"Error during sequence: {e}")
            response.success = False
            response.message = str(e)

        return response

    async def get_target_param(self, param_name):
        future = self.cli.get_parameters([param_name])
        response = await future  # 여기서 await해도 Reentrant라 멈추지 않음
        
        if response.values:
            value = parameter_value_to_python(response.values[0])
            self.get_logger().info(f"   -> Got '{param_name}': {value}")
            return value
        return None

    async def set_target_param(self, param_name, value):
        param = Parameter(param_name, Parameter.Type.DOUBLE, value)
        future = self.cli.set_parameters([param])
        response = await future
        
        # 하나라도 실패하면 False 리턴
        for result in response.results:
            if not result.successful:
                self.get_logger().error(f"   -> Set Failed: {result.reason}")
                return False
        
        self.get_logger().info(f"   -> Set Success")
        return True

def main(args=None):
    rclpy.init(args=args)
    node = ParamEventNode()

    # [Expert Point] MultiThreadedExecutor 사용
    # 여러 콜백(Service Request, Service Response 등)을 병렬로 처리하기 위해 필요
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        # 노드가 죽지 않고 계속 실행됨
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()