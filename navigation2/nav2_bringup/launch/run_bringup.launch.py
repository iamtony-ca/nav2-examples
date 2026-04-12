import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup') # nav2_bringup 패키지 경로 가져오기
    # [수정됨] 새 패키지 경로 가져오기
    fleet_config_dir = get_package_share_directory('ammr_common_config')
    
    # [수정됨] 새 패키지에서 common_ammr.yaml 읽기
    with open(os.path.join(fleet_config_dir, 'config', 'common_ammr.yaml'), 'r') as f:
        common_data = yaml.safe_load(f)['common_settings']

    fleet_rewrites = {
        'self_machine_id': common_data['machine_id'],
        'self_type_id': common_data['type_id'],
        'robot_ids': common_data['robot_ids']
    }
    for bot_id in common_data['robot_ids']:
        if bot_id in common_data:
            fleet_rewrites[bot_id] = common_data[bot_id]

    configured_params = RewrittenYaml(
        source_file=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        param_rewrites=fleet_rewrites,
        convert_types=True
    )

    run_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={'params_file': configured_params}.items()
    )

    return LaunchDescription([run_cmd])