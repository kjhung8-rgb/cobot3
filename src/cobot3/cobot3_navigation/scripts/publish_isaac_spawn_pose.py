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
        self._publish_count = 0
        self._max_publish_count = 10
        self._warned_no_subscriber = False
        self.create_timer(0.5, self._try_publish)

    def _try_publish(self):
        if self._done:
            return
        if self._pub.get_subscription_count() == 0:
            if not self._warned_no_subscriber:
                self.get_logger().info("Waiting for /initialpose subscriber")
                self._warned_no_subscriber = True
            return

        if not os.path.isfile(POSE_FILE):
            # Fallback for maps saved from this launch flow: slam_toolbox starts
            # the map frame at the robot's initial pose.
            self._publish_at(0.0, 0.0, 0.0)
            return
        with open(POSE_FILE, encoding="utf-8") as handle:
            data = json.load(handle)
        self._publish_at(float(data["x"]), float(data["y"]), float(data["yaw"]))

    def _publish_at(self, x: float, y: float, yaw: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891909122467
        self._pub.publish(msg)
        self._publish_count += 1
        if self._publish_count == 1:
            self.get_logger().info(
                f"Publishing /initialpose: x={x:.2f} y={y:.2f} yaw={yaw:.2f}"
            )
        if self._publish_count >= self._max_publish_count:
            self._done = True
            self.get_logger().info("Finished initial pose burst")


def main():
    rclpy.init()
    node = PublishIsaacSpawnPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
