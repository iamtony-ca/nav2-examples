import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'zed_monitoring'

    # Configuration File Path
    config_file_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'zed_monitoring.yaml'
    )

    # Node Definition
    zed_monitoring_node = Node(
        package=pkg_name,
        executable='zed_monitoring_node',
        name='zed_monitoring_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            config_file_path
        ],
        # 노드가 예기치 않게 종료되더라도 자동으로 재시작되도록 respawn 설정
        # (장애물 회피 목적의 노드이므로 재시작 대기 시간을 2초로 짧게 설정)
        respawn=True,
        respawn_delay=5.0
    )

    return LaunchDescription([
        zed_monitoring_node
    ])