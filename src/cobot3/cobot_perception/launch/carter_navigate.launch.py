# Stage 3 — Carter NAV2 stack (same-domain multi-robot).
#
# Spot and Carter share the same ROS_DOMAIN_ID (default 141). Carter's NAV2
# lives entirely under /carter_0 so it coexists with spot's stack without
# node-name or topic clashes.
#
# Why this pattern (RewrittenYaml + explicit Node namespace):
#   - ROS2 yaml root keys are matched against fully-qualified node names.
#     A relative key like `controller_server:` only matches default-ns
#     `/controller_server`, NOT `/carter_0/controller_server`. Verified by
#     observing controller_server fall back to DWB defaults when given
#     unprefixed yaml — its `FollowPath` MPPI section never reached the node.
#   - RewrittenYaml(root_key='carter_0') auto-prefixes every root key in
#     the yaml so namespaced nodes find their section.
#   - We avoid nav2_bringup/navigation_launch.py because its `namespace=`
#     arg only feeds RewrittenYaml's root_key and never applies to the
#     spawned Node objects (humble bug). Each Node here passes
#     `namespace=CARTER_NS` explicitly to keep the two in sync.
#   - PushRosNamespace inside GroupAction would also work for top-level
#     nodes, but it doesn't cross IncludeLaunchDescription boundaries and
#     the explicit form is less surprising.
#   - /tf and /tf_static are NOT remapped — a single global tf tree is
#     shared (spot SLAM owns map->spot_0/odom, carter AMCL owns
#     map->carter_0/odom).
#
# Goals: /carter_0/goal_pose (Nav2 default, namespaced). mission_manager
# (also launched here) bridges spot's /detected_survivor_pose to that topic
# with a planar-distance dedup. Manual goals via RViz Nav2 Goal or topic
# pub still work in parallel.
#
# Prerequisites (all on the same ROS_DOMAIN_ID):
#   - cobot3.spot Load Scene + Play + Setup ROS2/Camera/LiDAR-SLAM
#   - cobot3.spot Carter C1/C2/C3 buttons (cmd_vel + odom/TF + scan/TF)
#   - carter_localize.launch.py running (provides map -> carter_0/odom TF)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


CARTER_NS = "carter_0"

LIFECYCLE_NODES = [
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
]


def generate_launch_description():
    pkg_nav = get_package_share_directory("cobot3_navigation")
    nav2_params = os.path.join(pkg_nav, "params", "carter_nav2_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    # Auto-prefix every yaml root key with /carter_0 so namespaced nodes
    # find their section. convert_types lets RewrittenYaml drop the
    # substitution-vs-typed value mismatch warning.
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params,
            root_key=CARTER_NS,
            param_rewrites={
                "use_sim_time": use_sim_time,
                "autostart": autostart,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    common_node_kwargs = dict(
        namespace=CARTER_NS,
        output="screen",
        parameters=[configured_params],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock. Isaac publishes system-time stamps.",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Auto-bring lifecycle nodes to active.",
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                remappings=[("cmd_vel", "cmd_vel_nav")],
                **common_node_kwargs,
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                **common_node_kwargs,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                **common_node_kwargs,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                **common_node_kwargs,
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                **common_node_kwargs,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                **common_node_kwargs,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                remappings=[
                    ("cmd_vel", "cmd_vel_nav"),
                    ("cmd_vel_smoothed", "cmd_vel"),
                ],
                **common_node_kwargs,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                namespace=CARTER_NS,
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": LIFECYCLE_NODES},
                    # Larger map + first-time StaticLayer resize blows past the
                    # default 5s service timeout; lifecycle_manager then aborts
                    # bringup before behavior_server / bt_navigator even start.
                    {"node_timeout": 30.0},
                    # Disable bond watchdog after bringup so a slow tick on any
                    # node doesn't kill the whole stack.
                    {"bond_timeout": 0.0},
                ],
            ),
            # MissionManager lives in the default ns to subscribe to spot's
            # /detected_survivor_pose, then sends each pose to carter's
            # NavigateToPose action — one goal at a time. New detections
            # received mid-navigation are held and dispatched only after
            # carter reaches the current goal.
            Node(
                package="cobot_perception",
                executable="mission_manager",
                name="mission_manager",
                output="screen",
                parameters=[
                    {"input_topic": "/detected_survivor_pose"},
                    {"nav_action": "/carter_0/navigate_to_pose"},
                    {"dedup_distance_m": 0.5},
                ],
            ),
        ]
    )
