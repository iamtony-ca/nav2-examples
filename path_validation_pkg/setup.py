import os
from glob import glob
from setuptools import setup

package_name = 'path_validation_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch, params, maps 폴더 및 내부 파일들을 설치 경로에 복사
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')), # rviz 폴더 설치 추가


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@todo.todo',
    description='A minimal package to validate Nav2 global path existence.',
    license='Apache-2.0',
    tests_require=['pytest'],
    # 실행 가능한 파이썬 스크립트를 정의
    entry_points={
        'console_scripts': [
            'path_validator = path_validation_pkg.path_validator_node:main',
        ],
    },
)