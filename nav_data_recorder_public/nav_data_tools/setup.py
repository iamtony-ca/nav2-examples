from setuptools import setup

package_name = 'nav_data_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nav_data_recorder maintainers',
    maintainer_email='maintainer@example.com',
    description='nav_data_recorder tools',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_data_export = nav_data_tools.export:main',
            'nav_data_session = nav_data_tools.session:main',
            'nav_data_prune = nav_data_tools.prune:main',
            'nav_data_nightly = nav_data_tools.nightly:main',
            'nav_data_profile = nav_data_tools.profile:main',
            'nav_data_maintenance = nav_data_tools.maintenance:main',
        ],
    },
)
