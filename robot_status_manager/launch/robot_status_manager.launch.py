import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'robot_status_manager'

    # Configuration File Path
    config_file_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'robot_status_manager.yaml'
    )

    # Node Definition
    robot_status_manager_node = Node(
        package=pkg_name,
        executable='robot_status_manager_node',
        name='robot_status_manager',
        output='screen',
        emulate_tty=True,
        parameters=[
            config_file_path
        ],
        respawn=True,
        respawn_delay=10.0
    )

    return LaunchDescription([
        robot_status_manager_node
    ])