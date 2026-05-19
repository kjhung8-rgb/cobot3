import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanSanitizer(Node):
    """
    Republish Isaac Sim RTX LaserScan with angle metadata made consistent.

    Input : /spot_0/scan
    Output: /spot_0/scan_slam
    """

    def __init__(self):
        super().__init__("spot0_scan_sanitizer")

        self.input_topic = self.declare_parameter(
            "input_topic", "/spot_0/scan"
        ).get_parameter_value().string_value

        self.output_topic = self.declare_parameter(
            "output_topic", "/spot_0/scan_slam"
        ).get_parameter_value().string_value

        self.min_range = self.declare_parameter(
            "min_range", 0.05
        ).get_parameter_value().double_value

        self.pub = self.create_publisher(LaserScan, self.output_topic, 10)
        self.sub = self.create_subscription(
            LaserScan,
            self.input_topic,
            self._callback,
            10,
        )

        self._warned = False
        self.get_logger().info(
            f"✅ scan_sanitizer: {self.input_topic} -> {self.output_topic}"
        )

    def _callback(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return

        out = LaserScan()
        out.header = msg.header

        out.angle_min = float(msg.angle_min)
        out.angle_max = float(msg.angle_max)

        if n > 1:
            span = out.angle_max - out.angle_min

            if not math.isfinite(span) or abs(span) < 1e-9:
                out.angle_min = -math.pi
                out.angle_max = math.pi
                span = out.angle_max - out.angle_min

            out.angle_increment = float(span / (n - 1))
        else:
            out.angle_increment = float(msg.angle_increment)

        out.time_increment = float(msg.time_increment)
        out.scan_time = float(msg.scan_time)

        if n > 0 and out.scan_time > 0.0:
            out.time_increment = float(out.scan_time / n)

        out.range_min = max(float(msg.range_min), float(self.min_range))
        out.range_max = float(msg.range_max)

        ranges = []
        for r in msg.ranges:
            rf = float(r)
            if math.isnan(rf):
                ranges.append(float("inf"))
            elif rf < out.range_min:
                ranges.append(float("inf"))
            else:
                ranges.append(rf)

        out.ranges = ranges
        out.intensities = list(msg.intensities)

        if not self._warned:
            expected = round((out.angle_max - out.angle_min) / out.angle_increment) + 1 if out.angle_increment else -1
            self.get_logger().info(
                f"first scan fixed: frame={out.header.frame_id}, ranges={n}, expected={expected}, "
                f"angle_min={out.angle_min:.4f}, angle_max={out.angle_max:.4f}, inc={out.angle_increment:.6f}"
            )
            self._warned = True

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanSanitizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
