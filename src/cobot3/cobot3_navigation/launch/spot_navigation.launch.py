# SPDX-License-Identifier: Apache-2.0
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_map_yaml():
    share = get_package_share_directory("cobot3_navigation")
    local = os.path.join(share, "maps", "carter_warehouse_navigation.yaml")
    if os.path.isfile(local):
        return local
    carter_share = get_package_share_directory("carter_navigation")
    return os.path.join(carter_share, "maps", "carter_warehouse_navigation.yaml")


def generate_launch_description():
    pkg_cobot3_nav = get_package_share_directory("cobot3_navigation")
    default_map = _resolve_map_yaml()
    default_params = os.path.join(pkg_cobot3_nav, "params", "spot_navigation_params.yaml")
    rviz_cfg = os.path.join(pkg_cobot3_nav, "rviz2", "spot_navigation.rviz")

    nav2_bringup_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")

    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
                description="Full path to map YAML (Carter warehouse, same as Nav2 example).",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Nav2 parameters for Spot + Isaac topics.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use /clock from Isaac Sim when true.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, "bringup_launch.py")
                ),
                launch_arguments={
                    "slam": "False",
                    "map": map_yaml,
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                    "autostart": "true",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, "rviz_launch.py")
                ),
                launch_arguments={
                    "namespace": "",
                    "use_namespace": "false",
                    "rviz_config": rviz_cfg,
                }.items(),
            ),
            Node(
                package="cobot3_navigation",
                executable="cmd_vel_relay.py",
                name="cmd_vel_relay",
                output="screen",
            ),
            Node(
                package="cobot3_navigation",
                executable="map_odom_bootstrap.py",
                name="map_odom_bootstrap",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="cobot3_navigation",
                executable="publish_isaac_spawn_pose.py",
                name="spot_initial_pose",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
