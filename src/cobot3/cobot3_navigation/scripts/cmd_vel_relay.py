#!/usr/bin/env python3
"""Relay Nav2 /cmd_vel to Isaac Sim Spot bridge topic.

When ``enable_person_dampening`` is true the relay also listens to
``person_detected_topic`` (Bool) and dampens linear/angular velocity for
``dampening_hold_sec`` after each True message. This makes the robot
linger when YOLO spots a survivor so the camera has time to dwell.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__("cmd_vel_relay")

        self.declare_parameter("enable_person_dampening", False)
        self.declare_parameter("person_detected_topic", "/spot_0/yolo/person_detected")
        self.declare_parameter("dampening_factor", 0.5)
        self.declare_parameter("dampening_hold_sec", 2.0)

        self._enable_dampen = self.get_parameter("enable_person_dampening").value
        det_topic = self.get_parameter("person_detected_topic").value
        self._factor = float(self.get_parameter("dampening_factor").value)
        self._hold_sec = float(self.get_parameter("dampening_hold_sec").value)

        self._pub = self.create_publisher(Twist, "/spot_0/cmd_vel", 10)
        self._sub = self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)

        self._last_detect_ns: int = 0
        if self._enable_dampen:
            self._det_sub = self.create_subscription(Bool, det_topic, self._on_detect, 10)
            self.get_logger().info(
                f"Relaying /cmd_vel -> /spot_0/cmd_vel "
                f"(dampening x{self._factor:.2f} for {self._hold_sec:.1f}s after detection on {det_topic})"
            )
        else:
            self.get_logger().info("Relaying /cmd_vel -> /spot_0/cmd_vel")

    def _on_detect(self, msg: Bool):
        if msg.data:
            self._last_detect_ns = self.get_clock().now().nanoseconds

    def _on_cmd(self, msg: Twist):
        out = msg
        if self._enable_dampen and self._last_detect_ns:
            elapsed = (self.get_clock().now().nanoseconds - self._last_detect_ns) / 1e9
            if elapsed < self._hold_sec:
                out = Twist()
                out.linear.x = msg.linear.x * self._factor
                out.linear.y = msg.linear.y * self._factor
                out.linear.z = msg.linear.z * self._factor
                out.angular.x = msg.angular.x * self._factor
                out.angular.y = msg.angular.y * self._factor
                out.angular.z = msg.angular.z * self._factor
        self._pub.publish(out)


def main():
    rclpy.init()
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
