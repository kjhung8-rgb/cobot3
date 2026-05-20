from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")
    robot_type = LaunchConfiguration("robot_type")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_name",
            default_value="anymalc_0",
            description="Robot namespace name. Example: spot_0, spot_1, jetbot",
        ),
        DeclareLaunchArgument(
            "robot_type",
            default_value="anymalc",
            description="BaseAction type. auto uses namespace prefix. Example: spot, jetbot",
        ),

        Node(
            package="cobot_core",
            executable="command_router",
            name="command_router",
            output="screen",
            parameters=[
                {"default_robot": robot_name},
            ],
        ),

        Node(
            package="cobot_core",
            executable="action_node",
            name="action_node",
            namespace=robot_name,
            output="screen",
            parameters=[
                {"robot_type": robot_type},
            ],
        ),
    ])
