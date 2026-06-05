import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'navigation_manager'
    
    # 1. Configuration File Path
    config_file_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'roi_range_manager.yaml'
    )

    # 2. Node Definition
    roi_range_manager_node = Node(
        package=pkg_name,
        executable='roi_range_manager_node',
        name='roi_range_manager_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            config_file_path  # Load YAML params
        ],
        respawn=True,
        respawn_delay=10.0
    )

    return LaunchDescription([
        roi_range_manager_node
    ])