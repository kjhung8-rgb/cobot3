# Survivor detector only (no SLAM/Nav2/explore). For quick YOLO testing.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


PERCEPTION_VENV_PYTHON = "/home/rokey/dev_ws/venv/perception/bin/python"


def generate_launch_description():
    pkg = get_package_share_directory("cobot_perception")
    detector_params = os.path.join(pkg, "config", "survivor_detector.yaml")
    yolo_model = os.path.join(pkg, "models", "yolov8n.pt")

    return LaunchDescription(
        [
            Node(
                package="cobot_perception",
                executable="survivor_detector",
                name="survivor_detector",
                output="screen",
                prefix=PERCEPTION_VENV_PYTHON,
                parameters=[detector_params, {"model_path": yolo_model}],
            ),
        ]
    )
