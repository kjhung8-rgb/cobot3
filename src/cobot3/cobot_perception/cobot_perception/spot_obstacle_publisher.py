"""Publishes Spot's position as dual PointCloud2 streams for jackal's costmap.

Two topics:
  /spot_marking_cloud  — 12 points at mark_radius around spot (marking=True, clearing=False)
  /spot_clearing_cloud — 16 points at clear_radius around spot (clearing=True, marking=False)

The obstacle_layer in jackal's costmap uses both:
  - clearing source raycasts from spot_0/base_link outward to clear_radius, erasing old marks
  - marking source marks the current spot position at mark_radius (+ inflation gives ~1m zone)
"""

from __future__ import annotations

import math
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import tf2_ros


class SpotObstaclePublisher(Node):
    def __init__(self):
        super().__init__("spot_obstacle_publisher")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("mark_radius_m", 0.5)
        self.declare_parameter("clear_radius_m", 2.0)
        self.declare_parameter("mark_topic", "/spot_marking_cloud")
        self.declare_parameter("clear_topic", "/spot_clearing_cloud")
        self.declare_parameter("spot_frame", "spot_0/base_link")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        mark_topic = self.get_parameter("mark_topic").value
        clear_topic = self.get_parameter("clear_topic").value
        rate = self.get_parameter("publish_rate_hz").value

        self.mark_pub = self.create_publisher(PointCloud2, mark_topic, 10)
        self.clear_pub = self.create_publisher(PointCloud2, clear_topic, 10)
        self.timer = self.create_timer(1.0 / rate, self._publish)

    def _publish(self):
        spot_frame = self.get_parameter("spot_frame").value
        mark_r = float(self.get_parameter("mark_radius_m").value)
        clear_r = float(self.get_parameter("clear_radius_m").value)

        try:
            self.tf_buffer.lookup_transform("map", spot_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().debug(f"TF not ready: {e}")
            return

        stamp = self.get_clock().now().to_msg()

        mark_pts = self._circle_points(mark_r, 12)
        self.mark_pub.publish(self._make_cloud(spot_frame, stamp, mark_pts))

        clear_pts = self._circle_points(clear_r, 16)
        self.clear_pub.publish(self._make_cloud(spot_frame, stamp, clear_pts))

    def _circle_points(self, radius: float, n: int) -> list[tuple[float, float, float]]:
        return [
            (radius * math.cos(2 * math.pi * i / n), radius * math.sin(2 * math.pi * i / n), 0.3)
            for i in range(n)
        ]

    def _make_cloud(self, frame_id: str, stamp, points: list) -> PointCloud2:
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        data = bytearray()
        for x, y, z in points:
            data.extend(struct.pack("fff", x, y, z))

        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = frame_id
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * len(points)
        cloud.data = bytes(data)
        cloud.is_dense = True
        return cloud


def main(args=None):
    rclpy.init(args=args)
    node = SpotObstaclePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
