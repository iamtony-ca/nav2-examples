import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'nav_safety_manager'
    
    # 1. Configuration File Path
    config_file_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'nav_safety_manager.yaml'
    )

    # 2. Node Definition
    nav_safety_manager_node = Node(
        package=pkg_name,
        executable='nav_safety_manager_node',
        name='nav_safety_manager_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            config_file_path  # Load YAML params
        ],
        respawn=True,
        respawn_delay=10.0
    )

    return LaunchDescription([
        nav_safety_manager_node
    ])