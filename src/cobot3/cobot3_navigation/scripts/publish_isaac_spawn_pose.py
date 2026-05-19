#!/usr/bin/env python3
"""Publish /initialpose from pose written by cobot3.spot Load Scene (Carter chassis in USD)."""
import json
import math
import os

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

POSE_FILE = os.path.expanduser("~/.cobot3/spot_spawn_pose.json")


class PublishIsaacSpawnPose(Node):
    def __init__(self):
        super().__init__("spot_initial_pose")
        self._pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self._done = False
        self.create_timer(1.0, self._try_publish)

    def _try_publish(self):
        if self._done:
            return
        if not os.path.isfile(POSE_FILE):
            return
        with open(POSE_FILE, encoding="utf-8") as handle:
            data = json.load(handle)
        yaw = float(data["yaw"])
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(data["x"])
        msg.pose.pose.position.y = float(data["y"])
        msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891909122467
        self._pub.publish(msg)
        self._done = True
        self.get_logger().info(
            f"Published /initialpose from Isaac spawn file: "
            f"x={data['x']:.2f} y={data['y']:.2f} yaw={yaw:.2f}"
        )


def main():
    rclpy.init()
    node = PublishIsaacSpawnPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
