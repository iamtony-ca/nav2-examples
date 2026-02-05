#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# [수정 전] 틀린 위치
# from rclpy.parameter import ParameterDescriptor 

# [수정 후] 올바른 위치 (rcl_interfaces 패키지의 msg 모듈)
from rcl_interfaces.msg import ParameterDescriptor

class ParamServer(Node):
    def __init__(self):
        super().__init__('param_server_node')
        
        # 'target_speed'라는 파라미터 선언 (기본값 1.0)
        self.declare_parameter(
            'target_speed', 
            1.0, 
            ParameterDescriptor(description='Robot target speed')
        )
        
        # 파라미터 변경 시 콜백 등록 (옵션)
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.get_logger().info('Param Server is ready. Waiting for requests...')

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'target_speed':
                self.get_logger().info(f"Parameter '{param.name}' changed to {param.value}")
        
        # 변경 승인
        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = ParamServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()