#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanSanitizer(Node):
    def __init__(self):
        super().__init__("scan_sanitizer")

        self.declare_parameter("input_scan_topic", "/spot_0/scan")
        self.declare_parameter("output_scan_topic", "/spot_0/scan_slam")
        self.declare_parameter("frame_id", "spot_0/lidar_link")
        self.declare_parameter("min_range_epsilon", 0.01)

        input_topic = self.get_parameter("input_scan_topic").value
        output_topic = self.get_parameter("output_scan_topic").value

        self.pub = self.create_publisher(LaserScan, output_topic, 10)
        self.sub = self.create_subscription(
            LaserScan,
            input_topic,
            self.cb,
            qos_profile_sensor_data,
        )

        self.get_logger().info(f"ScanSanitizer: {input_topic} -> {output_topic}")

    def cb(self, msg: LaserScan):
        out = LaserScan()
        out.header = msg.header
        out.header.frame_id = self.get_parameter("frame_id").value

        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max

        ranges = list(msg.ranges)
        n = len(ranges)

        if n > 1:
            out.angle_increment = (out.angle_max - out.angle_min) / float(n - 1)
        else:
            out.angle_increment = msg.angle_increment

        out.scan_time = msg.scan_time
        if n > 0 and math.isfinite(msg.scan_time) and msg.scan_time > 0.0:
            out.time_increment = msg.scan_time / float(n)
        elif math.isfinite(msg.time_increment):
            out.time_increment = msg.time_increment
        else:
            out.time_increment = 0.0
        out.range_min = msg.range_min
        out.range_max = msg.range_max

        eps = float(self.get_parameter("min_range_epsilon").value)

        clean = []
        for r in ranges:
            if r is None or math.isnan(r) or r <= out.range_min + eps:
                clean.append(float("inf"))
            elif r > out.range_max:
                clean.append(float("inf"))
            else:
                clean.append(float(r))

        out.ranges = clean

        if msg.intensities and len(msg.intensities) == n:
            out.intensities = list(msg.intensities)
        else:
            out.intensities = []

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanSanitizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
