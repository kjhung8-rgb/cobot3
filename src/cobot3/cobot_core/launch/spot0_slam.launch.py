import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("cobot_core")
    slam_params = os.path.join(pkg_share, "config", "spot0_slam_toolbox.yaml")

    scan_sanitizer = Node(
        package="cobot_core",
        executable="scan_sanitizer",
        name="spot0_scan_sanitizer",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"input_topic": "/spot_0/scan"},
            {"output_topic": "/spot_0/scan_slam"},
            {"min_range": 0.05},
        ],
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        scan_sanitizer,
        slam_toolbox,
    ])
