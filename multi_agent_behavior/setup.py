import os
from glob import glob
from setuptools import find_packages
from setuptools import setup

package_name = 'multi_agent_behavior'
submodules = "multi_agent_behavior/submodules"

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        # Config files (YAML)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='User@todo.todo',
    description='Fleet decision logic for multi-agent navigation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'multi_agent_behavior_controller = multi_agent_behavior.multi_agent_behavior_controller:main',
            # 'multi_agent_behavior_driver = multi_agent_behavior.multi_agent_behavior_driver:main',
            # node_executable_name = package_name.python_file:main_func
            'fleet_decision_node = multi_agent_behavior.fleet_decision_node:main',            
        ],
    },
)
