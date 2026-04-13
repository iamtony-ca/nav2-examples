import os
import yaml
import tempfile
import copy  # [추가] 1. 깊은 복사를 위한 copy 모듈 임포트

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


# [수정] 2. 함수 내부에 copy.deepcopy() 적용
def replace_keys_recursively(data, rewrites):
    if isinstance(data, dict):
        for k, v in data.items():
            if k in rewrites:
                # 메모리 참조를 끊어 똑같은 데이터를 넣어도 Alias(*, &)가 생기지 않도록 강제함
                data[k] = copy.deepcopy(rewrites[k])
            elif isinstance(v, dict):
                replace_keys_recursively(v, rewrites)


def generate_launch_description():
    bringup_dir = get_package_share_directory('amr_bringup')
    fleet_config_dir = get_package_share_directory('amr_fleet_config')
    
    # 1. 공통 YAML 읽기
    with open(os.path.join(fleet_config_dir, 'config', 'common_fleet.yaml'), 'r') as f:
        common_data = yaml.safe_load(f)['common_settings']

    # 2. 덮어쓸 파라미터 (self_type_id 포함)
    fleet_rewrites = {
        'self_machine_id': common_data['machine_id'],
        'self_type_id': common_data['type_id'],
        'robot_ids': common_data['robot_ids']
    }
    for bot_id in common_data['robot_ids']:
        if bot_id in common_data:
            fleet_rewrites[bot_id] = common_data[bot_id]

    # 3. 원본 nav2_params.yaml 읽기
    original_yaml_path = os.path.join(bringup_dir, 'params', 'nav2_params.yaml')
    with open(original_yaml_path, 'r') as f:
        yaml_data = yaml.safe_load(f)

    # 4. 메모리 상에서 병합 (global_costmap, local_costmap 내부의 agent_layer까지 모두 탐색 후 치환됨)
    replace_keys_recursively(yaml_data, fleet_rewrites)

    # 5. OS 임시 폴더에 쓰기
    temp_yaml_file = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
    yaml.dump(yaml_data, temp_yaml_file)
    temp_yaml_file.close()

    # 6. 임시 파일 경로를 인자로 넣어 원본 Bringup 실행
    run_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={'params_file': temp_yaml_file.name}.items()
    )

    return LaunchDescription([run_cmd])
