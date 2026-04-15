import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'zed_box_to_laser'

setup(
    name=package_name,
    version='0.1.0',
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
    maintainer='Robotics Developer',
    maintainer_email='dev@todo.todo',
    description='Converts ZED 3D Object Detections to 2D LaserScan for Nav2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zed_box_to_laser_node = zed_box_to_laser.zed_box_to_laser_node:main'
        ],
    },
)