# Full Spot exploration + survivor detection stack.
#
# Brings up:
#   1. scan_sanitizer (x2: one for SLAM, one for Nav2 costmap)
#   2. slam_toolbox (online mapping; publishes /map and map->odom TF)
#   3. Nav2 navigation_launch (no map_server/AMCL, SLAM provides map)
#   4. cmd_vel_relay (Nav2 /cmd_vel -> /spot_0/cmd_vel)
#   5. explore_lite (frontier-based autonomous exploration)
#   6. yolo_detector (YOLOv8 RGB-D localization on front camera)
#   7. survivor_pose_to_marker (PoseStamped -> RViz X; 테스트: ros2 topic pub --once ...)
#   8. RViz with combined view
#
# Prerequisite: cobot3.spot extension publishing /spot_0/{odom,scan,front_cam/*}.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_perception = get_package_share_directory("cobot_perception")
    pkg_yolo = get_package_share_directory("yolo")
    pkg_nav = get_package_share_directory("cobot3_navigation")
    nav2_bringup_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")

    slam_params = os.path.join(pkg_nav, "params", "spot_slam_params.yaml")
    nav2_params = os.path.join(pkg_nav, "params", "spot_navigation_params.yaml")
    detector_params = os.path.join(pkg_yolo, "config", "yolo_detector.yaml")
    explore_params = os.path.join(pkg_perception, "config", "explore.yaml")
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
                        parameters=[{"use_sim_time": use_sim_time}],
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
                    {"dampening_factor": 0.3},
                    {"dampening_hold_sec": 2.0},
                ],
            ),
            TimerAction(
                period=8.0,
                actions=[
                    Node(
                        package="explore_lite",
                        executable="explore",
                        name="explore",
                        output="screen",
                        parameters=[explore_params, {"use_sim_time": use_sim_time}],
                    ),
                ],
            ),
            Node(
                package="yolo",
                executable="yolo_detector",
                name="yolo_detector",
                output="screen",
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
            # Spin 360 after every explore goal so the narrow front camera FOV
            # gets a chance to see what 360-deg LiDAR already mapped through.
            TimerAction(
                period=10.0,
                actions=[
                    Node(
                        package="cobot_perception",
                        executable="rotate_on_arrival",
                        name="rotate_on_arrival",
                        output="screen",
                        parameters=[{"use_sim_time": use_sim_time}],
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
