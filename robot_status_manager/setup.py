import os
from glob import glob
from setuptools import find_packages
from setuptools import setup

package_name = 'robot_status_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='User@todo.todo',
    description='Robot Status Manager processing BT logs and Action status',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 패키지명.파일명:함수명
            'robot_status_manager_node = robot_status_manager.robot_status_manager_node:main',
        ],
    },
)