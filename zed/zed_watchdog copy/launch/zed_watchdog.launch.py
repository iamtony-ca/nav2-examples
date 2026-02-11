import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'zed_watchdog'

    # Configuration File Path
    config_file_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'zed_watchdog.yaml'
    )

    # Node Definition
    zed_watchdog_node = Node(
        package=pkg_name,
        executable='zed_watchdog_node',
        name='zed_watchdog',
        output='screen',
        emulate_tty=True,
        parameters=[
            config_file_path
        ],
        # Watchdog 자체가 죽으면 안 되므로 respawn 설정
        respawn=True,
        respawn_delay=10.0
    )

    return LaunchDescription([
        zed_watchdog_node
    ])