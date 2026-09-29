from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('replan_monitor')
    default_params = os.path.join(pkg_share, 'config', 'path_validator.params.yaml')
    # default_params = '/........../src/software/replan_monitor/config/path_validator.params.yaml'

    # Launch args
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Full path to the YAML file with node parameters'
    )

    # Topic remap launch args (optional)
    costmap_topic_arg = DeclareLaunchArgument(
        'costmap_topic', default_value='/global_costmap/costmap_raw',
        description='Costmap topic (nav2_msgs/Costmap)'
    )
    pruned_path_topic_arg = DeclareLaunchArgument(
        'pruned_path_topic', default_value='/plan_pruned',
        description='Pruned path topic (nav_msgs/Path)'
    )
    robot_status_topic_arg = DeclareLaunchArgument(
        'robot_status_topic', default_value='/robot_status',
        description='Robot status topic (std_msgs/String)'
    )
    replan_flag_topic_arg = DeclareLaunchArgument(
        'replan_flag_topic', default_value='/replan_flag',
        description='Replan flag topic (std_msgs/Bool)'
    )

    node = Node(
        package='replan_monitor',
        executable='path_validator_node',
        name='path_validator_node',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            # (from, to)
            ('/global_costmap/costmap_raw', LaunchConfiguration('costmap_topic')),
            ('/plan_pruned', LaunchConfiguration('pruned_path_topic')),
            ('/robot_status', LaunchConfiguration('robot_status_topic')),
            ('/replan_flag', LaunchConfiguration('replan_flag_topic')),
        ],
        # [FIX PV-2] 기동 직후 SIGSEGV 로 죽은 사례(sim, 15회 중 1회). 이 노드가 없으면
        # navigation_manager 가 cond_static/cond_agent 를 못 받아 로봇이 영영 출발하지 못한다.
        # fleet_decision.launch.py 와 같은 방식으로 되살린다.
        respawn=True,
        respawn_delay=5.0,
    )

    return LaunchDescription([
        use_sim_time,
        params_file,
        costmap_topic_arg,
        pruned_path_topic_arg,
        robot_status_topic_arg,
        replan_flag_topic_arg,
        node
    ])