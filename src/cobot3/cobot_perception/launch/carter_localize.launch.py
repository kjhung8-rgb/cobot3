# Stage 2 — Carter localization only (no nav2 navigation stack).
#
# Brings up:
#   1. scan_sanitizer (global ns): /carter_0/scan -> /carter_0/scan_nav
#   2. AMCL under /carter_0 ns, subscribing /map (from spot SLAM) and
#      /carter_0/scan_nav. Publishes map -> carter_0/odom TF.
#   3. lifecycle_manager_localization to bring AMCL to active.
#
# Carter and Spot share the same ROS_DOMAIN_ID (default 141). Namespace
# isolation via /carter_0 keeps spot's slam_toolbox lifecycle_manager and
# carter's AMCL lifecycle_manager from colliding.
#
# Prerequisites:
#   - cobot3.spot extension: Load Scene + Play + Setup ROS2 + Setup LiDAR/SLAM
#     (spot SLAM publishes /map)
#   - cobot3.spot Carter buttons: C1/C2/C3 done (so /carter_0/cmd_vel, odom,
#     scan + TFs exist)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


CARTER_NS = "carter_0"


def generate_launch_description():
    pkg_nav = get_package_share_directory("cobot3_navigation")
    amcl_params = os.path.join(pkg_nav, "params", "carter_amcl_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock. Isaac publishes system-time stamps.",
            ),
            # scan_sanitizer stays in default ns; subscribes /carter_0/scan,
            # republishes /carter_0/scan_nav. AMCL uses the sanitized scan.
            Node(
                package="cobot_core",
                executable="scan_sanitizer",
                name="carter_amcl_scan_sanitizer",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"input_scan_topic": "/carter_0/scan"},
                    {"output_scan_topic": "/carter_0/scan_nav"},
                    {"frame_id": "carter_0/laser"},
                ],
            ),
            GroupAction(
                actions=[
                    PushRosNamespace(CARTER_NS),
                    Node(
                        package="nav2_amcl",
                        executable="amcl",
                        name="amcl",
                        output="screen",
                        parameters=[amcl_params, {"use_sim_time": use_sim_time}],
                    ),
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_localization",
                        output="screen",
                        parameters=[
                            {"use_sim_time": use_sim_time},
                            {"autostart": True},
                            {"node_names": ["amcl"]},
                        ],
                    ),
                ]
            ),
        ]
    )
