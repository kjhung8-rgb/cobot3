"""Publish a fake survivor world pose for integration testing (no YOLO / no camera).

Use this until the real pipeline publishes the same PoseStamped topic from Spot.
Carter (Nav2) can later subscribe to the identical topic name configured here.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


def _quat_from_yaw(yaw_rad: float) -> Quaternion:
    half = yaw_rad * 0.5
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


class SurvivorPoseMock(Node):
    def __init__(self):
        super().__init__("survivor_pose_mock")

        self.declare_parameter("pose_topic", "/detected_survivor_pose")
        self.declare_parameter("marker_topic", "/survivor_goal_marker")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("x", 2.0)
        self.declare_parameter("y", 1.0)
        self.declare_parameter("z", 0.0)
        self.declare_parameter("yaw_deg", 0.0)
        self.declare_parameter("marker_half_size", 0.35)
        self.declare_parameter("publish_hz", 1.0)
        self.declare_parameter("log_each_publish", True)

        self._pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value
        self._marker_topic = self.get_parameter("marker_topic").get_parameter_value().string_value
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self._x = self.get_parameter("x").get_parameter_value().double_value
        self._y = self.get_parameter("y").get_parameter_value().double_value
        self._z = self.get_parameter("z").get_parameter_value().double_value
        yaw_deg = self.get_parameter("yaw_deg").get_parameter_value().double_value
        self._yaw_rad = math.radians(yaw_deg)
        self._half = self.get_parameter("marker_half_size").get_parameter_value().double_value
        hz = self.get_parameter("publish_hz").get_parameter_value().double_value
        self._log_each = self.get_parameter("log_each_publish").get_parameter_value().bool_value

        self._pub_pose = self.create_publisher(PoseStamped, self._pose_topic, 10)
        self._pub_marker = self.create_publisher(Marker, self._marker_topic, 10)

        period = 1.0 / hz if hz > 0.0 else 0.0
        if period > 0.0:
            self._timer = self.create_timer(period, self._publish)
        else:
            self._timer = None

        self._publish()
        self.get_logger().info(
            f"Mock survivor pose: topic={self._pose_topic} frame={self._frame_id} "
            f"x={self._x:.3f} y={self._y:.3f} z={self._z:.3f} yaw_deg={yaw_deg:.1f} "
            f"marker={self._marker_topic} @ {hz} Hz"
        )

    def _build_pose(self, stamp) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.pose.position.x = self._x
        msg.pose.position.y = self._y
        msg.pose.position.z = self._z
        msg.pose.orientation = _quat_from_yaw(self._yaw_rad)
        return msg

    def _build_x_marker(self, stamp) -> Marker:
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = self._frame_id
        m.ns = "survivor"
        m.id = 0
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0

        hx, hy, hz = self._half, self._half, 0.05
        cx, cy, cz = self._x, self._y, self._z
        # Two strokes forming an X on the horizontal plane (slightly above ground).
        def pt(px: float, py: float, pz: float) -> Point:
            p = Point()
            p.x, p.y, p.z = px, py, pz
            return p

        z = cz + hz
        m.points = [
            pt(cx - hx, cy - hy, z),
            pt(cx + hx, cy + hy, z),
            pt(cx - hx, cy + hy, z),
            pt(cx + hx, cy - hy, z),
        ]
        m.scale.x = 0.04
        c = ColorRGBA()
        c.r, c.g, c.b, c.a = 1.0, 0.2, 0.1, 1.0
        m.color = c
        m.lifetime.sec = 0
        m.lifetime.nanosec = 0
        return m

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        pose = self._build_pose(stamp)
        self._pub_pose.publish(pose)
        self._pub_marker.publish(self._build_x_marker(stamp))
        if self._log_each:
            p = pose.pose.position
            o = pose.pose.orientation
            self.get_logger().info(
                "[survivor mock] PoseStamped "
                f"frame={pose.header.frame_id} "
                f"xyz=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) "
                f"quat_xyzw=({o.x:.3f}, {o.y:.3f}, {o.z:.3f}, {o.w:.3f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = SurvivorPoseMock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
