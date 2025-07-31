from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_monitoring',
            executable='status_manager_node',
            name='status_manager',
            output='screen',
            parameters=[
                # 여기에 필요하면 파라미터 YAML 파일 경로를 추가할 수 있음
                # Launch configuration에서 지정된 param file을 사용하고 싶다면 여기에 추가하세요.
                # Launch configuration에 따라: {'use_sim_time': True}, 'params.yaml', etc.
            ]
        )
    ])
