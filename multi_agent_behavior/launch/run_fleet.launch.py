import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    fleet_dir = get_package_share_directory('multi_agent_behavior')
    # [수정됨] 새 패키지 경로
    fleet_config_dir = get_package_share_directory('ammr_common_config')
    
    with open(os.path.join(fleet_config_dir, 'config', 'common_ammr.yaml'), 'r') as f:
        machine_id = yaml.safe_load(f)['common_settings']['machine_id']

    run_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fleet_dir, 'launch', 'fleet_decision.launch.py')
        ),
        launch_arguments={'my_id': str(machine_id)}.items()
    )

    return LaunchDescription([run_cmd])