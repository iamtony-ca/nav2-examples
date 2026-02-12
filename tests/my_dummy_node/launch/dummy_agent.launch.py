import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 패키지 경로 및 파라미터 파일 경로 찾기
    package_name = 'my_dummy_node'
    pkg_share = get_package_share_directory(package_name)
    default_config_path = os.path.join(pkg_share, 'config', 'params.yaml')

    # 2. Launch Argument 선언 (CLI에서 config 파일 경로 변경 가능하도록)
    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config_path,
        description='Path to the config.yaml file'
    )

    # 3. 노드 설정
    dummy_node = Node(
        package=package_name,
        executable='dummy_publisher_node',
        name='dummy_agent_publisher_node', # params.yaml의 최상위 키와 일치해야 함
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('config')]
    )

    return LaunchDescription([
        config_arg,
        dummy_node
    ])