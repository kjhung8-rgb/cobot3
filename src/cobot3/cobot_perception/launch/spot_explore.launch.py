# Full Spot exploration + survivor detection stack.
#
# Brings up:
#   1. scan_sanitizer (x2: one for SLAM, one for Nav2 costmap)
#   2. slam_toolbox (online mapping; publishes /map and map->odom TF)
#   3. Nav2 navigation_launch (no map_server/AMCL, SLAM provides map)
#   4. cmd_vel_relay (Nav2 /cmd_vel -> /spot_0/cmd_vel)
#   5. coverage_path_planner (camera-coverage waypoint exploration)
#   6. yolo_detector (YOLOv8 RGB-D localization on front camera)
#   7. survivor_pose_to_marker (PoseStamped -> RViz X; 테스트: ros2 topic pub --once ...)
#   8. RViz with combined view
#
# Prerequisite: cobot3.spot extension publishing /spot_0/{odom,scan,*_cam/*}.

import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_speed_config(pkg_nav):
    speed_path = os.path.join(pkg_nav, "config", "spot_speed.yaml")
    with open(speed_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return data.get("spot_speed", {})


def _nav2_params_with_speed(nav2_params, speed):
    with open(nav2_params, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    default_linear = float(speed["default_linear_mps"])
    default_angular = float(speed["default_angular_radps"])
    max_linear = float(speed["max_linear_mps"])
    max_angular = float(speed["max_angular_radps"])
    max_lateral = float(speed["max_lateral_mps"])
    max_reverse = float(speed["max_reverse_mps"])

    velocity_smoother = data["velocity_smoother"]["ros__parameters"]
    velocity_smoother["max_velocity"] = [max_linear, max_lateral, max_angular]
    velocity_smoother["min_velocity"] = [-max_reverse, -max_lateral, -max_angular]

    follow_path = data["controller_server"]["ros__parameters"]["FollowPath"]
    follow_path["vx_max"] = default_linear
    follow_path["wz_max"] = default_angular

    tmp = tempfile.NamedTemporaryFile(
        mode="w",
        prefix="spot_navigation_params_",
        suffix=".yaml",
        delete=False,
        encoding="utf-8",
    )
    with tmp:
        yaml.safe_dump(data, tmp, sort_keys=False)
    return tmp.name


def generate_launch_description():
    pkg_perception = get_package_share_directory("cobot_perception")
    pkg_yolo = get_package_share_directory("yolo")
    pkg_nav = get_package_share_directory("cobot3_navigation")
    nav2_bringup_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")

    slam_params = os.path.join(pkg_nav, "params", "spot_slam_params.yaml")
    nav2_params = os.path.join(pkg_nav, "params", "spot_navigation_params.yaml")
    speed = _load_speed_config(pkg_nav)
    nav2_params = _nav2_params_with_speed(nav2_params, speed)
    detector_params = os.path.join(pkg_yolo, "config", "yolo_detector.yaml")
    pose_to_marker_params = os.path.join(pkg_yolo, "config", "survivor_pose_to_marker.yaml")
    rviz_cfg = os.path.join(pkg_perception, "rviz", "spot_explore.rviz")
    yolo_model = os.path.join(pkg_yolo, "models", "yolov8s.pt")

    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock. Isaac extension publishes system time stamps.",
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
            Node(
                package="cobot_core",
                executable="scan_sanitizer",
                name="spot_nav_scan_sanitizer",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"input_scan_topic": "/spot_0/scan"},
                    {"output_scan_topic": "/spot_0/scan_nav"},
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
                    # Camera coverage tracker: must come up before Nav2 so
                    # /map_explorable exists when global_costmap.static_layer
                    # tries to subscribe.
                    Node(
                        package="cobot_perception",
                        executable="camera_coverage_tracker",
                        name="camera_coverage_tracker",
                        output="screen",
                        parameters=[
                            {"use_sim_time": use_sim_time},
                            {
                                "camera_frames": [
                                    "spot_0/front_cam_link",
                                    "spot_0/left_cam_link",
                                    "spot_0/right_cam_link",
                                ],
                            },
                            {
                                "camera_info_topics": [
                                    "/spot_0/front_cam/camera_info",
                                    "/spot_0/left_cam/camera_info",
                                    "/spot_0/right_cam/camera_info",
                                ],
                            },
                            # 4 m: matches waypoint spacing in CPP, faster
                            # coverage growth, fewer waypoints to visit.
                            {"max_range_m": 4.0},
                        ],
                    ),
                ],
            ),
            TimerAction(
                period=3.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(nav2_bringup_dir, "navigation_launch.py")
                        ),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            "params_file": nav2_params,
                            "autostart": "true",
                            "use_composition": "False",
                        }.items(),
                    ),
                ],
            ),
            Node(
                package="cobot3_navigation",
                executable="cmd_vel_relay.py",
                name="cmd_vel_relay",
                output="screen",
                parameters=[
                    {"enable_person_dampening": True},
                    {"slowdown_required_topic": "/spot_0/yolo/slowdown_required"},
                    {"dampening_factor": float(speed["yolo_speed_factor"])},
                    {"dampening_hold_sec": float(speed["yolo_hold_sec"])},
                ],
            ),
            # ── Pure CPP architecture ──
            # explore_lite and camera_coverage_sweep are both gone. A single
            # coverage_path_planner generates a grid of waypoints on
            # SLAM-free space (spaced = camera range) and visits each one,
            # spinning at each so the camera covers all directions.
            # No frontier algorithm → no instant-success / blacklist / etc.
            Node(
                package="yolo",
                executable="yolo_detector",
                name="yolo_detector",
                output="screen",
                additional_env={
                    "MPLCONFIGDIR": "/tmp/matplotlib",
                    "YOLO_CONFIG_DIR": "/tmp/Ultralytics",
                },
                parameters=[
                    detector_params,
                    {"model_path": yolo_model},
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="yolo",
                executable="survivor_pose_to_marker",
                name="survivor_pose_to_marker",
                output="screen",
                parameters=[pose_to_marker_params, {"use_sim_time": use_sim_time}],
            ),
            TimerAction(
                period=10.0,
                actions=[
                    Node(
                        package="cobot_perception",
                        executable="coverage_path_planner",
                        name="coverage_path_planner",
                        output="screen",
                        parameters=[
                            {"use_sim_time": use_sim_time},
                            {"waypoint_spacing_m": 4.0},
                            {"do_spin_at_waypoint": True},
                            {"spin_duration_sec": 4.0},
                            {"spin_speed_rad_s": float(speed["default_angular_radps"])},
                            {"skip_already_seen": True},
                            {"use_start_pose_as_home": True},
                            {"auto_return_enabled": True},
                            {"auto_return_coverage_threshold": 0.95},
                            {"auto_return_hold_sec": 5.0},
                            # Zone partitioning: 15m × 15m. With 4m
                            # waypoint spacing each zone has ~12 waypoints
                            # → meaningful "stay and finish current area
                            # before moving on" behaviour.
                            {"zone_size_m": 15.0},
                        ],
                    ),
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
