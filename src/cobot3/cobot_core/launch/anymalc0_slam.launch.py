import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("cobot_core")
    slam_params = os.path.join(pkg_share, "config", "anymalc0_slam_toolbox.yaml")

    slam_toolbox_share = get_package_share_directory("slam_toolbox")
    slam_launch = os.path.join(slam_toolbox_share, "launch", "online_async_launch.py")

    scan_sanitizer = Node(
        package="cobot_core",
        executable="scan_sanitizer",
        name="anymalc_scan_sanitizer",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"input_scan_topic": "/anymal_0/scan"},
            {"output_scan_topic": "/anymalc_0/scan_slam"},
            {"frame_id": "anymal_0/lidar_link"},
        ],
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            "slam_params_file": slam_params,
            "use_sim_time": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            scan_sanitizer,
            # Give sanitizer a moment so /anymalc_0/scan_slam exists first.
            TimerAction(period=1.0, actions=[slam]),
        ]
    )
