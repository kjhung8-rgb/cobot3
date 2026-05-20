# Mock survivor world pose (no YOLO): PoseStamped + RViz X marker + terminal logs.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("cobot_perception")
    params = os.path.join(pkg, "config", "survivor_pose_mock.yaml")

    return LaunchDescription(
        [
            Node(
                package="cobot_perception",
                executable="survivor_pose_mock",
                name="survivor_pose_mock",
                output="screen",
                parameters=[params],
            ),
        ]
    )
