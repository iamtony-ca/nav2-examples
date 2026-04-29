from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    # ── 런치 인자 선언 ───────────────────────────────────────────
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            # default_value='robot1',
            default_value='',
            description='Robot namespace (TF, topics)'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use Gazebo / sim time'),

        # ── SmartRecoveryServer ─────────────────────────────────
        Node(
            package='smart_recovery',
            executable='smart_recovery_server',
            namespace=LaunchConfiguration('namespace'),
            name='smart_recovery_server',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            remappings=[
                # action, service는 네임스페이스 자동 적용.
                ('/recovery_path', 'recovery_path'),           # <ns>/recovery_path
                ('/recovery_done', 'recovery_done')
            ]
        ),

        # ── SmartRecoveryPlanner ───────────────────────────────
        Node(
            package='smart_recovery',
            executable='smart_recovery_planner',
            namespace=LaunchConfiguration('namespace'),
            name='smart_recovery_planner',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                # 필요 시 추가 planner 파라미터
                # 'planner_search_radius': 1.2
            }],
            remappings=[
                ('/local_costmap/costmap', 'local_costmap/costmap')
            ]
        ),

        # ── MPPI CmdVel Generator ──────────────────────────────
        Node(
            package='smart_recovery',
            executable='mppi_cmdvel_generator',
            namespace=LaunchConfiguration('namespace'),
            name='mppi_cmdvel_generator',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                # 'linear_velocity': 0.25,
                # 'goal_tolerance': 0.12
            }],
            remappings=[
                ('/cmd_vel', 'cmd_vel'),                       # 결과 속도 → <ns>/cmd_vel
                ('/local_costmap/costmap', 'local_costmap/costmap')
            ]
        ),
    ])
