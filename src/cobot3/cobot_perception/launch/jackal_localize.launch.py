# Stage 2 — Jackal localization: static map→odom TF (no LiDAR, no AMCL).
#
# Jackal uses Spot's SLAM map for path planning. Localization is a fixed static
# TF: map → jackal_0/odom placed at jackal's spawn offset from spot's SLAM
# origin: (2.5, 0.0) m = jackal_spawn(24,29) − spot_spawn(21.5,29) in Isaac world.
#
# Jackal's onboard odometry (ComputeOdometry in Isaac) tracks motion relative to
# its spawn, so jackal_0/odom → jackal_0/base_link drifts are the only error
# source. For short warehouse runs this is acceptable without loop closure.
#
# Prerequisites:
#   - cobot3.spot Isaac extension: Load Scene (Jackal) + Play + Setup Spot ROS
#   - cobot3.spot Jackal button: J. Setup Jackal ROS (CmdVel + Odom/TF)

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_jackal_odom_tf",
                # args: x y z yaw pitch roll parent child
                arguments=["2.5", "0", "0", "0", "0", "0", "map", "jackal_0/odom"],
                output="screen",
            ),
        ]
    )
