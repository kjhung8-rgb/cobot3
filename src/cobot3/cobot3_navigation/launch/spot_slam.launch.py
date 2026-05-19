# SLAM mapping for Spot in Isaac Sim (full_warehouse).
# Prerequisite: cobot3.spot extension publishing /spot_0/odometry and /spot_0/lidar/scan

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("cobot3_navigation")
    slam_params = os.path.join(pkg_dir, "params", "spot_slam_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Isaac Sim /clock",
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_params, {"use_sim_time": use_sim_time}],
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
