# SLAM mapping for Spot in Isaac Sim.
# Prerequisite: cobot3.spot extension publishing /spot_0/odom, /spot_0/scan and TF.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("cobot3_navigation")
    slam_params = os.path.join(pkg_dir, "params", "spot_slam_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description=(
                    "Use /clock when true. The cobot3 Spot Isaac extension "
                    "publishes sensor and odom stamps with system time."
                ),
            ),
            Node(
                package="cobot_core",
                executable="scan_sanitizer",
                name="spot_slam_scan_sanitizer",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"input_scan_topic": "/spot_0/scan"},
                    {"output_scan_topic": "/spot_0/scan_slam"},
                    {"frame_id": "spot_0/lidar_link"},
                ],
            ),
            TimerAction(
                period=1.0,
                actions=[
                    Node(
                        package="slam_toolbox",
                        executable="async_slam_toolbox_node",
                        name="slam_toolbox",
                        output="screen",
                        parameters=[slam_params, {"use_sim_time": use_sim_time}],
                    ),
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", os.path.join(pkg_dir, "rviz2", "spot_slam.rviz")],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
