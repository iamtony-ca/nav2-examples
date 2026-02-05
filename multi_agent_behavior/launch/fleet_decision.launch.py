import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'multi_agent_behavior'
    
    # 1. Configuration File Path
    config_file_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'fleet_decision.yaml'
    )

    # 2. Launch Arguments (옵션으로 ID 변경 가능)
    # 예: ros2 launch multi_agent_behavior fleet_decision.launch.py my_id:=2
    my_id_arg = DeclareLaunchArgument(
        'my_id',
        default_value='1',
        description='Machine ID of this robot'
    )

    # 3. Node Definition
    fleet_decision_node = Node(
        package=pkg_name,
        executable='fleet_decision_node',
        name='fleet_decision_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            config_file_path,       # Load YAML first
            {'my_machine_id': LaunchConfiguration('my_id')} # Override ID if arg provided
        ],
        respawn=True,
        respawn_delay=10
    )

    return LaunchDescription([
        my_id_arg,
        fleet_decision_node
    ])