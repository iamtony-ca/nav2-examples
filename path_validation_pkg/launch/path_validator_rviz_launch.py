import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # ... 이전 package_name, pkg_share 정의 ...
    package_name = 'path_validation_pkg'
    pkg_share = get_package_share_directory(package_name)

    # ... 이전 declare_map_arg, declare_params_file_arg 정의 ...
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_share, 'maps', 'warehouse.yaml'),
        description='Full path to map file to load'
    )
    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'params', 'nav2_minimal_params.yaml'),
        description='Full path to the ROS2 parameters file to use'
    )

    # (추가) RViz 설정 파일 경로 변수
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'validator.rviz')
    
    # ... 이전 map_server_node, planner_server_node, static_tf_pub_node, lifecycle_manager_node 정의 ...
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'yaml_filename': LaunchConfiguration('map')}
        ]
    )
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': False}
        ]
    )
    static_tf_pub_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'base_link']
    )
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_validator',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': ['map_server', 'planner_server']}
        ]
    )

    # (추가) RViz2 노드 정의
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] # -d 옵션으로 설정 파일 로드
    )

    ld = LaunchDescription()
    ld.add_action(declare_map_arg)
    ld.add_action(declare_params_file_arg)
    ld.add_action(map_server_node)
    ld.add_action(planner_server_node)
    ld.add_action(static_tf_pub_node)
    ld.add_action(lifecycle_manager_node)
    ld.add_action(rviz_node) # (추가) RViz 노드를 런치에 추가
    return ld