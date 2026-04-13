import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    fleet_dir = get_package_share_directory('multi_agent_behavior')
    fleet_config_dir = get_package_share_directory('amr_fleet_config')
    
    # 1. 공통 YAML 읽기
    with open(os.path.join(fleet_config_dir, 'config', 'common_fleet.yaml'), 'r') as f:
        machine_id = yaml.safe_load(f)['common_settings']['machine_id']

    # ---------------------------------------------------------
    # [핵심 수정] 파일이 'launch' 폴더 안에 있는지 먼저 확인하고, 
    # 없으면 패키지 share 루트에서 바로 찾도록 유연하게 분기 처리합니다.
    # ---------------------------------------------------------
    target_launch_file = os.path.join(fleet_dir, 'launch', 'fleet_decision.launch.py')
    if not os.path.exists(target_launch_file):
        # 파이썬 setup.py 구조 등으로 인해 launch 폴더 없이 설치된 경우
        target_launch_file = os.path.join(fleet_dir, 'fleet_decision.launch.py')

    run_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(target_launch_file),
        launch_arguments={'my_id': str(machine_id)}.items()
    )

    return LaunchDescription([run_cmd])
