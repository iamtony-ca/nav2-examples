import os
import yaml
import json  # [추가] 리스트와 딕셔너리를 문자열로 변환하기 위해 필요

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    replan_dir = get_package_share_directory('replan_monitor')
    fleet_config_dir = get_package_share_directory('amr_fleet_config')
    
    with open(os.path.join(fleet_config_dir, 'config', 'common_fleet.yaml'), 'r') as f:
        common_data = yaml.safe_load(f)['common_settings']

    # ---------------------------------------------------------
    # [핵심 수정] 모든 Value를 강제로 문자열(String)로 감싸줍니다.
    # convert_types=True 덕분에 임시 YAML 생성 시 다시 원래 타입으로 캐스팅됩니다.
    # ---------------------------------------------------------
    fleet_rewrites = {
        'self_machine_id': str(common_data['machine_id']), # int -> str
        'robot_ids': json.dumps(common_data['robot_ids'])  # list -> str
    }
    for bot_id in common_data['robot_ids']:
        if bot_id in common_data:
            fleet_rewrites[bot_id] = json.dumps(common_data[bot_id]) # dict -> str

    configured_params = RewrittenYaml(
        source_file=os.path.join(replan_dir, 'config', 'path_validator.params.yaml'),
        param_rewrites=fleet_rewrites,
        convert_types=True
    )

    run_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(replan_dir, 'launch', 'path_validator.launch.py')
        ),
        launch_arguments={'params_file': configured_params}.items()
    )

    return LaunchDescription([run_cmd])
