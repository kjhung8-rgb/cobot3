#!/usr/bin/env python3
"""Relay gated velocity commands to Isaac Sim Spot bridge topic.

When ``enable_person_dampening`` is true the relay also listens to
``person_detected_topic`` (Bool) and dampens linear/angular velocity for
``dampening_hold_sec`` after each True message. This makes the robot
linger when YOLO spots a survivor so the camera has time to dwell.

Control modes:
    autonomous: relay Nav2 /cmd_vel to /spot_0/cmd_vel
    manual: relay GUI /teleop_cmd_vel to /spot_0/cmd_vel
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__("cmd_vel_relay")

        self.declare_parameter("enable_person_dampening", False)
        self.declare_parameter("person_detected_topic", "/spot_0/yolo/person_detected")
        self.declare_parameter("dampening_factor", 0.5)
        self.declare_parameter("dampening_hold_sec", 2.0)
        self.declare_parameter("control_mode_topic", "/control_mode")
        self.declare_parameter("teleop_cmd_vel_topic", "/teleop_cmd_vel")
        self.declare_parameter("default_control_mode", "autonomous")

        self._enable_dampen = self.get_parameter("enable_person_dampening").value
        det_topic = self.get_parameter("person_detected_topic").value
        self._factor = float(self.get_parameter("dampening_factor").value)
        self._hold_sec = float(self.get_parameter("dampening_hold_sec").value)
        self._mode = str(self.get_parameter("default_control_mode").value).strip().lower()
        if self._mode not in ("autonomous", "manual"):
            self.get_logger().warn(
                f"Invalid default_control_mode={self._mode!r}; using autonomous"
            )
            self._mode = "autonomous"

        self._pub = self.create_publisher(Twist, "/spot_0/cmd_vel", 10)
        self._nav_sub = self.create_subscription(Twist, "/cmd_vel", self._on_nav_cmd, 10)
        self._teleop_sub = self.create_subscription(
            Twist,
            self.get_parameter("teleop_cmd_vel_topic").value,
            self._on_teleop_cmd,
            10,
        )
        self._mode_sub = self.create_subscription(
            String,
            self.get_parameter("control_mode_topic").value,
            self._on_mode,
            10,
        )

        self._last_detect_ns: int = 0
        if self._enable_dampen:
            self._det_sub = self.create_subscription(Bool, det_topic, self._on_detect, 10)
            self.get_logger().info(
                f"Relaying gated cmd_vel -> /spot_0/cmd_vel "
                f"(dampening x{self._factor:.2f} for {self._hold_sec:.1f}s after detection on {det_topic})"
            )
        else:
            self.get_logger().info("Relaying gated cmd_vel -> /spot_0/cmd_vel")
        self.get_logger().info(f"Initial control mode: {self._mode}")

    def _on_detect(self, msg: Bool):
        if msg.data:
            self._last_detect_ns = self.get_clock().now().nanoseconds

    def _on_mode(self, msg: String):
        mode = msg.data.strip().lower()
        if mode not in ("autonomous", "manual"):
            self.get_logger().warn(f"Ignoring invalid control mode: {msg.data!r}")
            return
        if mode == self._mode:
            return
        self._mode = mode
        self._publish_stop()
        self.get_logger().info(f"Control mode switched to {self._mode}")

    def _on_nav_cmd(self, msg: Twist):
        if self._mode != "autonomous":
            return
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

    def _on_teleop_cmd(self, msg: Twist):
        if self._mode != "manual":
            return
        self._pub.publish(msg)

    def _publish_stop(self):
        self._pub.publish(Twist())


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
