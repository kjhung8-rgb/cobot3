#!/usr/bin/env python3
"""Relay Nav2 /cmd_vel to Isaac Sim ANYmal C bridge topic."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class AnymalCmdVelRelay(Node):
    def __init__(self):
        super().__init__("anymal_cmd_vel_relay")
        self._pub = self.create_publisher(Twist, "/anymal_0/cmd_vel", 10)
        self._sub = self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.get_logger().info("Relaying /cmd_vel -> /anymal_0/cmd_vel")

    def _on_cmd(self, msg: Twist):
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = AnymalCmdVelRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
