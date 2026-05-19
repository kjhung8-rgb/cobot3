# SPDX-License-Identifier: Apache-2.0
"""Launch Nav2 for ANYmal C in the Carter warehouse.

Usage:
  ros2 launch cobot3_navigation anymal_navigation.launch.py
  ros2 launch cobot3_navigation anymal_navigation.launch.py use_sim_time:=true
"""
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
    # Fallback to carter_navigation package if available
    try:
        carter_share = get_package_share_directory("carter_navigation")
        return os.path.join(carter_share, "maps", "carter_warehouse_navigation.yaml")
    except Exception:
        return local


def generate_launch_description():
    pkg_cobot3_nav = get_package_share_directory("cobot3_navigation")
    default_map = _resolve_map_yaml()
    default_params = os.path.join(pkg_cobot3_nav, "params", "anymal_navigation_params.yaml")
    rviz_cfg = os.path.join(pkg_cobot3_nav, "rviz2", "anymal_navigation.rviz")

    nav2_bringup_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")

    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
                description="Full path to occupancy map YAML (Carter warehouse).",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Nav2 parameters for ANYmal C + Isaac topics.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use /clock from Isaac Sim when true.",
            ),
            # Nav2 full stack (map_server, AMCL, planner, controller, costmaps, …)
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
            # RViz2 with Nav2 plugin
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
            # Bridge: Nav2 /cmd_vel -> /anymal_0/cmd_vel consumed by Isaac OmniGraph
            Node(
                package="cobot3_navigation",
                executable="anymal_cmd_vel_relay.py",
                name="anymal_cmd_vel_relay",
                output="screen",
            ),
            # Bootstrap map->odom TF until AMCL takes over
            Node(
                package="cobot3_navigation",
                executable="map_odom_bootstrap.py",
                name="map_odom_bootstrap",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            # Publish /initialpose from ~/.cobot3/anymal_spawn_pose.json (or origin)
            Node(
                package="cobot3_navigation",
                executable="publish_anymal_spawn_pose.py",
                name="anymal_initial_pose",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
