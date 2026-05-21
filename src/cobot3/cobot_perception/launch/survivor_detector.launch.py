# Compatibility launch for quick YOLO testing.
#
# The YOLO detector/marker nodes live in the separate `yolo` package now.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    yolo_launch = os.path.join(
        get_package_share_directory("yolo"),
        "launch",
        "yolo_pipeline.launch.py",
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(yolo_launch)),
        ]
    )
