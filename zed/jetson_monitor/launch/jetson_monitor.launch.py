import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'jetson_monitor'

    # Configuration File Path
    config_file_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'jetson_monitor.yaml'
    )

    # Node Definition
    jetson_monitor_node = Node(
        package=pkg_name,
        executable='jetson_monitor_node',
        name='jetson_monitor_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            config_file_path
        ],
        # 모니터링 노드는 죽으면 안 되므로 respawn 설정
        respawn=True,
        respawn_delay=10.0
    )

    return LaunchDescription([
        jetson_monitor_node
    ])