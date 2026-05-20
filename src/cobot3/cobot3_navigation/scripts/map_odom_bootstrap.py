#!/usr/bin/env python3
"""Publish identity map->odom until AMCL runs, then exit so AMCL owns that transform."""
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class MapOdomBootstrap(Node):
    def __init__(self):
        super().__init__("map_odom_bootstrap")
        self._broadcaster = TransformBroadcaster(self)
        self._done = False
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._on_amcl_pose,
            10,
        )
        self.create_timer(0.1, self._publish_bootstrap)
        self.get_logger().info("Publishing map->odom until /amcl_pose is received")

    def _on_amcl_pose(self, _msg: PoseWithCovarianceStamped):
        if self._done:
            return
        self._done = True
        self.get_logger().info("AMCL pose received; releasing map->odom to AMCL")
        self.destroy_node()
        rclpy.shutdown()

    def _publish_bootstrap(self):
        if self._done:
            return
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"
        transform.transform.rotation.w = 1.0
        self._broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = MapOdomBootstrap()
    rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
