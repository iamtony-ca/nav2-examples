import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    lifecycle_nodes = ['filter_mask_server', 'costmap_filter_info_server']

    # Arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file') # nav2_params.yaml을 그대로 받음
    mask_yaml_file = LaunchConfiguration('mask')     # median_mask.yaml 경로를 받음
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)

    # 파라미터 재작성 (마스크 파일 경로 덮어쓰기 위함)
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': mask_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    # 노드 정의 (Standalone)
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='filter_mask_server',
                namespace=namespace,
                output='screen',
                emulate_tty=True,
                parameters=[configured_params]),
            Node(
                package='nav2_map_server',
                executable='costmap_filter_info_server',
                name='costmap_filter_info_server',
                namespace=namespace,
                output='screen',
                emulate_tty=True,
                parameters=[configured_params]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_costmap_filters',
                namespace=namespace,
                output='screen',
                emulate_tty=True,
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}])
        ]
    )

    # 노드 정의 (Composition)
    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::MapServer',
                        name='filter_mask_server',
                        parameters=[configured_params]),
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::CostmapFilterInfoServer',
                        name='costmap_filter_info_server',
                        parameters=[configured_params]),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_costmap_filters',
                        parameters=[{'use_sim_time': use_sim_time},
                                    {'autostart': autostart},
                                    {'node_names': lifecycle_nodes}]),
                ]
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('params_file'), # 필수 입력
        DeclareLaunchArgument('mask'),        # 필수 입력
        DeclareLaunchArgument('use_composition', default_value='True'),
        DeclareLaunchArgument('container_name', default_value='nav2_container'),
        load_nodes,
        load_composable_nodes
    ])