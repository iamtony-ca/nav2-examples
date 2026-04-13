import os
import yaml
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# [핵심 함수] 원본 YAML 구조를 순회하며 키가 일치하면 값을 덮어쓰는 재귀 함수
def replace_keys_recursively(data, rewrites):
    if isinstance(data, dict):
        for k, v in data.items():
            if k in rewrites:
                data[k] = rewrites[k]  # 값 덮어쓰기 (타입 완벽 보존)
            elif isinstance(v, dict):
                replace_keys_recursively(v, rewrites)

def generate_launch_description():
    replan_dir = get_package_share_directory('replan_monitor')
    fleet_config_dir = get_package_share_directory('amr_fleet_config')
    
    # 1. 공통 YAML 읽기
    with open(os.path.join(fleet_config_dir, 'config', 'common_fleet.yaml'), 'r') as f:
        common_data = yaml.safe_load(f)['common_settings']

    # 2. 덮어쓸 파라미터 딕셔너리 (문자열 변환 없이 파이썬 타입 그대로 유지!)
    fleet_rewrites = {
        'self_machine_id': common_data['machine_id'],
        'robot_ids': common_data['robot_ids']
    }
    for bot_id in common_data['robot_ids']:
        if bot_id in common_data:
            fleet_rewrites[bot_id] = common_data[bot_id]

    # 3. 원본 path_validator.params.yaml 읽기
    original_yaml_path = os.path.join(replan_dir, 'config', 'path_validator.params.yaml')
    with open(original_yaml_path, 'r') as f:
        yaml_data = yaml.safe_load(f)

    # 4. 메모리 상에서 파라미터 덮어쓰기 병합
    replace_keys_recursively(yaml_data, fleet_rewrites)

    # 5. OS 임시 폴더에 안전하게 병합된 YAML 파일 쓰기
    temp_yaml_file = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
    yaml.dump(yaml_data, temp_yaml_file)
    temp_yaml_file.close() # 디스크에 쓰기 완료

    # 6. 임시 파일 경로를 인자로 넣어 원본 Launch 실행
    run_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(replan_dir, 'launch', 'path_validator.launch.py')
        ),
        launch_arguments={'params_file': temp_yaml_file.name}.items()
    )

    return LaunchDescription([run_cmd])
