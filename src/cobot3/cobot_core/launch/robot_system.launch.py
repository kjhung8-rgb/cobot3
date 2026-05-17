from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_name",
            default_value="jetbot",
            description="Robot namespace name. Example: jetbot, carter, turtlebot",
        ),

        Node(
            package="cobot_core",
            executable="command_router",
            name="command_router",
            output="screen",
        ),

        Node(
            package="cobot_core",
            executable="action_node",
            name="action_node",
            namespace=robot_name,
            output="screen",
        ),
    ])
