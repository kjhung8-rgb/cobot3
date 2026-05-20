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
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/models', glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='YOLOv8 survivor detector for Spot front camera.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'survivor_detector = cobot_perception.survivor_detector:main',
            'survivor_pose_mock = cobot_perception.survivor_pose_mock:main',
            'survivor_pose_to_marker = cobot_perception.survivor_pose_to_marker:main',
            'rotate_on_arrival = cobot_perception.rotate_on_arrival:main',
            'camera_coverage_tracker = cobot_perception.camera_coverage_tracker:main',
        ],
    },
)
