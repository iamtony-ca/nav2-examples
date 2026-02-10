import os
from glob import glob
from setuptools import find_packages
from setuptools import setup

package_name = 'nav_safety_manager'
submodules = "nav_safety_manager/submodules"

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Config files (YAML)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='User@todo.todo',
    description='Navigation Safety Manager handling PLC and Collision states',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'nav_safety_manager_controller = nav_safety_manager.nav_safety_manager_controller:main',
            # 'nav_safety_manager_driver = nav_safety_manager.nav_safety_manager_driver:main',
            # node_executable_name = package_name.python_file:main_func
            'nav_safety_manager_node = nav_safety_manager.nav_safety_manager_node:main',            
        ],
    },
)




# import os
# from glob import glob
# from setuptools import find_packages
# from setuptools import setup

# package_name = 'nav_safety_manager'

# setup(
#     name=package_name,
#     version='0.0.1',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         # Launch files
#         (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
#         # Config files (YAML)
#         (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),        
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='User',
#     maintainer_email='User@todo.todo',
#     description='Navigation Safety Manager handling PLC and Collision states',
#     license='TODO: License declaration',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#             # node_executable_name = package_name.python_file:main_func
#             'nav_safety_manager_node = nav_safety_manager.nav_safety_manager_node:main',            
#         ],
#     },
# )