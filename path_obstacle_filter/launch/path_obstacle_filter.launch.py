import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 파라미터 설정을 위한 YAML 파일 경로 (선택 사항)
    # params_file = os.path.join(
    #     get_package_share_directory('path_obstacle_filter'),
    #     'config',
    #     'params.yaml'
    # )

    # path_obstacle_filter 노드 실행 설정
    path_obstacle_filter_node = Node(
        package='path_obstacle_filter',
        executable='path_obstacle_filter_node',
        name='path_obstacle_filter',
        output='screen',
        emulate_tty=True,
        parameters=[
            # 여기에 직접 파라미터 값을 설정할 수 있습니다.
            {'global_frame': 'map'},
            {'robot_base_frame': 'base_link'},
            {'distance_threshold': 0.1},
            {'lookahead_dist': 4.5},
            # YAML 파일을 사용하려면 아래 라인의 주석을 해제하세요.
            # params_file
        ]
    )

    return LaunchDescription([
        path_obstacle_filter_node
    ])