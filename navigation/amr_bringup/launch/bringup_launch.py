# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushROSNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('amr_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    use_localization = LaunchConfiguration('use_localization')

    # [추가] Launch Configuration 변수
    use_filters = LaunchConfiguration('use_filters')
    filter_params_file = LaunchConfiguration('filter_params_file')
    filter_mask_file = LaunchConfiguration('filter_mask_file')



    # Path to the maps directory
    # maps_dir = os.path.join(bringup_dir, 'maps')
    maps_dir = '/home/amr/ammr_ws/src/assets/maps'
    # Create a new variable that resolves the map path
    resolved_map_yaml_file = map_yaml_file
    resolved_map_yaml_file = PythonExpression([
        '"', map_yaml_file, '" if "', map_yaml_file, '".startswith("/") else "',
        maps_dir, '/', map_yaml_file, '"'
    ])


    # 경로 자동 완성 로직 (Smart Path Logic)
    # 1. 입력값이 '' (빈 값)이거나 '/'로 시작하면 -> 그대로 사용 (절대경로 or 미사용)
    # 2. 그 외(파일명만 입력) -> maps_dir + '/' + 파일명
    final_filter_mask_path = PythonExpression([
        "'", filter_mask_file, "' if '", filter_mask_file, "'.startswith('/') or '", filter_mask_file, "' == '' ",
        "else '", maps_dir, "/' + '", filter_mask_file, "'"
    ])


    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # (orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Only it applys when `use_namespace` is True.
    # '<robot_namespace>' keyword shall be replaced by 'namespace' launch argument
    # in config file 'nav2_multirobot_params.yaml' as a default & example.
    # User defined config file should contain '<robot_namespace>' keyword for the replacements.
    params_file = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': ('/', namespace)},
        condition=IfCondition(use_namespace),
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)






    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack',
    )

    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='False', description='Whether run a SLAM'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', default_value='', description='Full path to map yaml file to load'
    )

    declare_use_localization_cmd = DeclareLaunchArgument(
        'use_localization', default_value='True',
        description='Whether to enable amr_bringup/localization_launch.py '
                    '(map_server). Set False when localization is provided '
                    'externally (e.g. sim: nav2_bringup AMCL+map_server).'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        # default_value = '/home/......./src/software/navigation/amr_navigation2/amr_bringup/params/nav2_params.yaml',
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Whether to use composed bringup',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )


    # [추가] 필터 관련 Launch Argument 선언
    declare_use_filters_cmd = DeclareLaunchArgument(
        'use_filters', default_value='True',
        description='Whether to enable costmap filters'
    )

    declare_filter_params_file_cmd = DeclareLaunchArgument(
        'filter_params_file',
        # 기본값은 본인의 median_params.yaml 경로로 설정
        default_value=os.path.join(bringup_dir, 'params', 'keepout_params.yaml'),
        description='Full path to the Costmap Filter params file'
    )

    declare_filter_mask_file_cmd = DeclareLaunchArgument(
        'filter_mask_file',
        # 기본값은 본인의 median_mask.yaml 경로로 설정
        # default_value=os.path.join(bringup_dir, 'maps', 'v2_260114_map_keepout.yaml'),
        default_value=os.path.join(bringup_dir, 'maps', '251223_map_keepout.yaml'),
        description='Full path to the Costmap Filter mask file'
    )




    # Specify the actions
    bringup_cmd_group = GroupAction(
        [
            PushROSNamespace(condition=IfCondition(use_namespace), namespace=namespace),
            Node(
                condition=IfCondition(use_composition),
                name='nav2_container',
                package='rclcpp_components',
                executable='component_container_isolated',
                parameters=[configured_params, {'autostart': autostart}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
                output='screen',
            ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         os.path.join(launch_dir, 'slam_launch.py')
            #     ),
            #     condition=IfCondition(PythonExpression([slam, ' and ', use_localization])),
            #     launch_arguments={
            #         'namespace': namespace,
            #         'use_sim_time': use_sim_time,
            #         'autostart': autostart,
            #         'use_respawn': use_respawn,
            #         'params_file': params_file,
            #     }.items(),
            # ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'localization_launch.py')
                ),
                condition=IfCondition(use_localization),
                launch_arguments={
                    'namespace': namespace,
                    # 'map': map_yaml_file,
                    'map': resolved_map_yaml_file,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': params_file,
                    'use_composition': use_composition,
                    'use_respawn': use_respawn,
                    'container_name': 'nav2_container',
                }.items(),
            ),


            # # [추가] Costmap Filters 실행
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'costmap_filter.launch.py')
                ),
                condition=IfCondition(use_filters),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    
                    # [핵심 변경] 필터 전용 파일 대신, 메인 params_file을 그대로 전달
                    'params_file': params_file, 
                    
                    # 'mask': filter_mask_file,
                    'mask': final_filter_mask_path,
                    'use_composition': use_composition,
                    'container_name': 'nav2_container',
                }.items(),
            ),


            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'navigation_launch.py')
                ),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': params_file,
                    'use_composition': use_composition,
                    'use_respawn': use_respawn,
                    'container_name': 'nav2_container',
                }.items(),
            ),




        ]
    )
    # print(3333)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    # ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_localization_cmd)

    ld.add_action(declare_use_filters_cmd)
    ld.add_action(declare_filter_mask_file_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld
