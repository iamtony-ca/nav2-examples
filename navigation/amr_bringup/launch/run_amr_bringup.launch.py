import copy
import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def replace_keys_recursively(data, rewrites):
    if isinstance(data, dict):
        for k, v in data.items():
            # [안전장치 1] rewrites[k]가 비어있지 않은지 한 번 더 체크
            if k in rewrites and rewrites[k] is not None:
                data[k] = copy.deepcopy(rewrites[k])
            elif isinstance(v, dict):
                replace_keys_recursively(v, rewrites)


def _overlay_and_include(context, *args, **kwargs):
    """Runs at launch time so LaunchConfiguration values are resolved."""
    bringup_dir = get_package_share_directory('amr_bringup')
    fleet_config_dir = get_package_share_directory('ammr_common_config')

    # `params_file_source` 를 실제 문자열로 해석 (launch 시점)
    params_file_source = LaunchConfiguration('params_file_source').perform(context)

    # 1. 공통 YAML 읽기
    with open(os.path.join(fleet_config_dir, 'config', 'common_ammr.yaml'), 'r') as f:
        common_data = yaml.safe_load(f)['common_settings']

    # 2. 덮어쓸 파라미터
    fleet_rewrites = {
        'self_machine_id': common_data['my_machine_id'],
        'self_type_id': common_data['my_type_id'],
        'robot_ids': common_data['robot_ids']
    }
    for bot_id in common_data.get('robot_ids', []):
        if bot_id in common_data:
            fleet_rewrites[bot_id] = common_data[bot_id]

    # 3. 소스 params YAML 읽기 (launch arg 로 지정, 기본 = nav2_params.yaml)
    with open(params_file_source, 'r') as f:
        yaml_data = yaml.safe_load(f)

    # 원본 파일이 비어있어 None으로 읽히는지 체크
    if yaml_data is None:
        raise ValueError(f"에러: {params_file_source} 파일이 비어있거나 잘못되었습니다.")

    # 4. 메모리 상에서 병합
    replace_keys_recursively(yaml_data, fleet_rewrites)

    # 5. OS 임시 폴더에 쓰기
    temp_yaml_file = tempfile.NamedTemporaryFile(
        mode='w', delete=False, suffix='.yaml')
    # [안전장치 2] default_flow_style=False 를 반드시 추가!
    # 이렇게 해야 ROS 2 파서가 오작동하지 않는 완전한 줄바꿈(Block) 형태의 YAML로 강제 생성됩니다.
    yaml.dump(yaml_data, temp_yaml_file, default_flow_style=False)
    temp_yaml_file.close()

    # 6. 임시 파일 경로를 인자로 넣어 원본 Bringup 실행
    run_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'bringup_launch.py')
        ),
        # [안전장치 3] .items() 대신 명시적인 List of Tuples 형태로 전달
        launch_arguments=[('params_file', temp_yaml_file.name)]
    )
    return [run_cmd]


def generate_launch_description():
    bringup_dir = get_package_share_directory('amr_bringup')

    declare_params_file_source_cmd = DeclareLaunchArgument(
        'params_file_source',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Source params YAML to overlay common_ammr.yaml onto. '
                    'Defaults to amr_bringup/params/nav2_params.yaml '
                    '(real-robot setup). Override for sim or other configs.'
    )

    return LaunchDescription([
        declare_params_file_source_cmd,
        OpaqueFunction(function=_overlay_and_include),
    ])
