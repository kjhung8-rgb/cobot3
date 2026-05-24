import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ultralytics is installed in the venv perception interpreter, not in
# Isaac Sim's bundled python or system ROS python. yolo_detector must be
# launched via this interpreter or `import ultralytics` fails immediately.
PERCEPTION_VENV_PYTHON = os.environ.get(
    "PERCEPTION_VENV_PYTHON",
    "/home/rokey/dev_ws/venv/perception/bin/python",
)


def generate_launch_description():
    pkg_yolo = get_package_share_directory("yolo")
    detector_params = os.path.join(pkg_yolo, "config", "yolo_detector.yaml")
    marker_params = os.path.join(pkg_yolo, "config", "survivor_pose_to_marker.yaml")
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
                package="yolo",
                executable="yolo_detector",
                name="yolo_detector",
                output="screen",
                prefix=PERCEPTION_VENV_PYTHON,
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
                parameters=[marker_params, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
