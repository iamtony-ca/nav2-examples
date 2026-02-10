import os
from glob import glob
from setuptools import find_packages
from setuptools import setup

package_name = 'zed_watchdog'

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
    description='Watchdog node for ZED Multi-Camera system',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 패키지명.파일명:함수명
            'zed_watchdog_node = zed_watchdog.zed_watchdog_node:main',
        ],
    },
)