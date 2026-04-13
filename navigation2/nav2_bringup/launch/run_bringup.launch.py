import os
import yaml
import json  # [추가]

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    bringup_dir = get_package_share_directory('amr_bringup')
    fleet_config_dir = get_package_share_directory('amr_fleet_config')
    
    with open(os.path.join(fleet_config_dir, 'config', 'common_fleet.yaml'), 'r') as f:
        common_data = yaml.safe_load(f)['common_settings']

    # [핵심 수정] 문자열 캐스팅
    fleet_rewrites = {
        'self_machine_id': str(common_data['machine_id']),
        'self_type_id': str(common_data['type_id']),
        'robot_ids': json.dumps(common_data['robot_ids'])
    }
    for bot_id in common_data['robot_ids']:
        if bot_id in common_data:
            fleet_rewrites[bot_id] = json.dumps(common_data[bot_id])

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
