#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException
from tf2_ros.transform_listener import TransformListener


class ScanSanitizer(Node):
    def __init__(self):
        super().__init__("scan_sanitizer")

        self.declare_parameter("input_scan_topic", "/spot_0/scan")
        self.declare_parameter("output_scan_topic", "/spot_0/scan_slam")
        self.declare_parameter("frame_id", "spot_0/lidar_link")
        self.declare_parameter("min_range_epsilon", 0.01)
        self.declare_parameter("mask_frames", [""])
        self.declare_parameter("mask_radius_m", 0.45)
        self.declare_parameter("mask_max_range_m", 6.0)

        input_topic = self.get_parameter("input_scan_topic").value
        output_topic = self.get_parameter("output_scan_topic").value
        raw_mask_frames = self.get_parameter("mask_frames").value
        self._mask_frames = [frame for frame in (raw_mask_frames or []) if frame]
        self._mask_radius = float(self.get_parameter("mask_radius_m").value)
        self._mask_max_range = float(self.get_parameter("mask_max_range_m").value)

        if self._mask_frames:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self.get_logger().info(
                f"ScanSanitizer mask: {self._mask_frames} "
                f"(r={self._mask_radius:.2f}m, max={self._mask_max_range:.1f}m)"
            )

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

        if self._mask_frames:
            self._apply_mask(out, clean)

        out.ranges = clean
        if msg.intensities and len(msg.intensities) == n:
            out.intensities = list(msg.intensities)
        else:
            out.intensities = []

        self.pub.publish(out)

    def _apply_mask(self, out: LaserScan, clean: list[float]) -> None:
        scan_frame = out.header.frame_id
        latest = rclpy.time.Time()
        n = len(clean)

        for target in self._mask_frames:
            try:
                tf = self._tf_buffer.lookup_transform(scan_frame, target, latest)
            except (LookupException, ConnectivityException, ExtrapolationException):
                continue

            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            distance = math.hypot(tx, ty)
            if distance < 1e-3 or distance > self._mask_max_range:
                continue

            angle_to_target = math.atan2(ty, tx)
            if self._mask_radius >= distance:
                half_width = math.pi
            else:
                half_width = math.asin(self._mask_radius / distance) + math.radians(5.0)

            keep_below = max(0.0, distance - self._mask_radius - 0.1)
            for idx in range(n):
                angle = out.angle_min + idx * out.angle_increment
                diff = (angle - angle_to_target + math.pi) % (2.0 * math.pi) - math.pi
                if abs(diff) > half_width:
                    continue
                if clean[idx] < keep_below:
                    continue
                clean[idx] = float("inf")


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
