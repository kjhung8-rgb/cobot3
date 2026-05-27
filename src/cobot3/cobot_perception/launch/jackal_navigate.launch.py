# Stage 3 — Jackal NAV2 stack.
#
# Spot and Jackal share the same ROS_DOMAIN_ID (default 141). Jackal's NAV2
# lives entirely under /jackal_0 so it coexists with Spot's stack without
# node-name or topic clashes.
#
# Jackal uses Spot's SLAM /map for path planning. No LiDAR or AMCL required —
# localization comes from the static map→jackal_0/odom TF set by
# jackal_localize.launch.py. Spot's position is tracked in jackal's costmap via
# SpotObstaclePublisher (dual PointCloud2: clearing at 2m, marking at 0.5m),
# giving a ~1m exclusion zone around Spot (after costmap inflation).
#
# Same RewrittenYaml + explicit Node namespace pattern as carter_navigate.
#
# Prerequisites (all on the same ROS_DOMAIN_ID):
#   - cobot3.spot Isaac extension: Load Scene (Jackal) + Play + Setup Spot ROS
#   - cobot3.spot Jackal button: J. Setup Jackal ROS (CmdVel + Odom/TF)
#   - jackal_localize.launch.py running (provides static map→jackal_0/odom TF)
#   - Spot SLAM running (provides /map + spot_0/base_link TF for obstacle publisher)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


JACKAL_NS = "jackal_0"

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
    nav2_params = os.path.join(pkg_nav, "params", "jackal_nav2_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params,
            root_key=JACKAL_NS,
            param_rewrites={
                "use_sim_time": use_sim_time,
                "autostart": autostart,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    common_node_kwargs = dict(
        namespace=JACKAL_NS,
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
            # Jackal scan sanitizer: spot 본체 위치 (spot_0/base_link)의 ray를
            # inf로 치환. 그래야 jackal scan-based obstacle_layer가 spot을
            # obstacle로 안 봄 → 상호 마스킹 (spot이 jackal 안 보는 것의 대칭).
            Node(
                package="cobot_core",
                executable="scan_sanitizer",
                name="jackal_nav_scan_sanitizer",
                output="screen",
                parameters=[
                    {"input_scan_topic": "/jackal_0/scan"},
                    {"output_scan_topic": "/jackal_0/scan_nav"},
                    {"frame_id": "jackal_0/laser"},
                    {"mask_frames": ["spot_0/base_link"]},
                    # 0.6 → 0.3: spot body 일부만 mask. 외곽 LiDAR 반사는
                    # 통과 → jackal costmap에 spot 외곽선 찍혀 회피 가능.
                    {"mask_radius_m": 0.3},
                    {"mask_max_range_m": 20.0},
                ],
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
                # behavior_server의 spin/backup/drive_on_heading/wait recovery
                # 액션이 모두 cmd_vel을 publish. velocity_smoother까지 합쳐 5
                # publisher가 동시 spam → Isaac OmniGraph가 silent stall →
                # jackal cmd_vel 못 받음. recovery는 cmd_vel_recovery로 빼서
                # Isaac 신호선에서 분리.
                remappings=[("cmd_vel", "cmd_vel_recovery")],
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
                    # nav2 cmd → /jackal_0/cmd_vel_nav_smoothed로. 그 뒤
                    # cmd_vel_relay가 mode에 따라 /jackal_0/cmd_vel로 게이트.
                    ("cmd_vel_smoothed", "cmd_vel_nav_smoothed"),
                ],
                **common_node_kwargs,
            ),
            # cmd_vel_relay: nav vs teleop을 /jackal_0/control_mode 토픽으로 분기.
            # manual 모드일 때만 /jackal_0/teleop_cmd_vel을 /jackal_0/cmd_vel로
            # 보냄. autonomous에선 nav2 smoothed 그대로 통과.
            Node(
                package="cobot3_navigation",
                executable="cmd_vel_relay.py",
                name="jackal_cmd_vel_relay",
                output="screen",
                parameters=[
                    {"enable_person_dampening": False},
                    {"control_mode_topic": "/jackal_0/control_mode"},
                    {"teleop_cmd_vel_topic": "/jackal_0/teleop_cmd_vel"},
                    {"nav_cmd_vel_topic": "/jackal_0/cmd_vel_nav_smoothed"},
                    {"output_cmd_vel_topic": "/jackal_0/cmd_vel"},
                    {"default_control_mode": "autonomous"},
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                namespace=JACKAL_NS,
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": LIFECYCLE_NODES},
                    {"node_timeout": 30.0},
                    {"bond_timeout": 0.0},
                ],
            ),
            Node(
                package="cobot_perception",
                executable="mission_manager",
                name="mission_manager",
                output="screen",
                parameters=[
                    {"input_topic": "/detected_survivor_pose"},
                    # RViz 2D Nav Goal 등 사용자 클릭 좌표 입력 토픽.
                    {"manual_goal_topic": "/jackal_0/manual_goal"},
                    # Bool 토픽 (true → home 복귀 트리거).
                    {"return_home_topic": "/jackal_0/return_home"},
                    {"resume_topic": "/jackal_0/mission_resume"},
                    {"rescue_action": "/jackal_0/navigate_to_pose"},
                    {"dedup_distance_m": 0.5},
                    # 0 = retry 없음. ABORT 즉시 mission_complete → 다음 pending
                    # 큐로 advance. retry는 lethal 영역에선 효과 없어서 비활성.
                    {"rescue_max_retries": 0},
                    {"rescue_retry_step_m": 2.0},
                    {"robot_base_frame": "jackal_0/base_link"},
                    {"map_frame": "map"},
                    # mission 끝나면 jackal이 시작 위치로 자동 복귀.
                    {"home_return_enabled": True},
                    {"home_arrival_radius_m": 0.5},
                ],
            ),
        ]
    )
