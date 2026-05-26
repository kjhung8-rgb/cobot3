from glob import glob
from setuptools import find_packages, setup

package_name = 'cobot_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', ['config/explore.yaml']),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='Spot exploration perception utilities for camera coverage and arrival rotation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rotate_on_arrival = cobot_perception.rotate_on_arrival:main',
            'camera_coverage_tracker = cobot_perception.camera_coverage_tracker:main',
            'camera_coverage_sweep = cobot_perception.camera_coverage_sweep:main',
            'coverage_path_planner = cobot_perception.coverage_path_planner:main',
            'stop_watchdog = cobot_perception.stop_watchdog:main',
            'monitoring_gui = cobot_perception.monitoring_gui:main',
            'mission_manager = cobot_perception.mission_manager:main',
            'spot_obstacle_publisher = cobot_perception.spot_obstacle_publisher:main',
        ],
    },
)
