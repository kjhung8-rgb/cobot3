# Cobot3 full stack — single-terminal launcher.
#
# Wraps the four launches that must run together for the spot→carter
# survivor-detection pipeline so you can `ros2 launch cobot_perception
# cobot3_full.launch.py` and Ctrl+C just once.
#
# Order matters (TimerAction-staged):
#   T+0s : spot_explore (SLAM, exploration, default-ns nav2)
#          yolo_pipeline (YOLO detector + survivor marker)
#   T+8s : carter_localize (AMCL — needs spot SLAM's /map to be ready)
#   T+15s: carter_navigate (nav2 + mission_manager — needs AMCL's
#          map→carter_0/odom TF before global_costmap configure)
#
# Adjust the TimerAction periods if a machine boots SLAM slowly.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _include(pkg: str, launch_file: str) -> IncludeLaunchDescription:
    path = os.path.join(get_package_share_directory(pkg), "launch", launch_file)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path))


def generate_launch_description():
    spot_explore = _include("cobot_perception", "spot_explore.launch.py")
    yolo_pipeline = _include("yolo", "yolo_pipeline.launch.py")
    carter_localize = _include("cobot_perception", "carter_localize.launch.py")
    carter_navigate = _include("cobot_perception", "carter_navigate.launch.py")

    return LaunchDescription(
        [
            # Tier 0 — spot SLAM/exploration + YOLO detector.
            spot_explore,
            yolo_pipeline,
            # Tier 1 — carter AMCL (needs /map from spot SLAM).
            TimerAction(period=8.0, actions=[carter_localize]),
            # Tier 2 — carter Nav2 + MissionManager (needs map->carter_0/odom TF).
            TimerAction(period=15.0, actions=[carter_navigate]),
        ]
    )
