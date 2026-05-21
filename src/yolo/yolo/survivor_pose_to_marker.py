"""Subscribe to survivor PoseStamped; publish RViz Marker (red X + center sphere).

Use until survivor_detector publishes the same PoseStamped from YOLO+localization.
Temporary test: ros2 topic pub --once /detected_survivor_pose geometry_msgs/msg/PoseStamped '...'
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


def _x_marker_msg(
    pose: PoseStamped, half: float, z_lift: float, line_width: float, marker_id: int
) -> Marker:
    cx = pose.pose.position.x
    cy = pose.pose.position.y
    cz = pose.pose.position.z

    m = Marker()
    m.header = pose.header
    m.ns = "survivor"
    m.id = marker_id
    m.type = Marker.LINE_LIST
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0

    hx = half
    hy = half
    z = cz + z_lift

    def pt(px: float, py: float, pz: float) -> Point:
        p = Point()
        p.x, p.y, p.z = px, py, pz
        return p

    m.points = [
        pt(cx - hx, cy - hy, z),
        pt(cx + hx, cy + hy, z),
        pt(cx - hx, cy + hy, z),
        pt(cx + hx, cy - hy, z),
    ]
    m.scale.x = line_width
    col = ColorRGBA()
    col.r, col.g, col.b, col.a = 1.0, 0.15, 0.1, 1.0
    m.color = col
    return m


def _sphere_marker_msg(
    pose: PoseStamped, diameter: float, z_lift: float, marker_id: int
) -> Marker:
    m = Marker()
    m.header = pose.header
    m.ns = "survivor"
    m.id = marker_id
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.pose.position.x = pose.pose.position.x
    m.pose.position.y = pose.pose.position.y
    m.pose.position.z = pose.pose.position.z + z_lift
    m.pose.orientation.w = 1.0
    m.scale.x = diameter
    m.scale.y = diameter
    m.scale.z = diameter
    col = ColorRGBA()
    col.r, col.g, col.b, col.a = 1.0, 0.45, 0.05, 0.85
    m.color = col
    return m


class SurvivorPoseToMarker(Node):
    """Bridge: PoseStamped (YOLO 등) -> visualization_msgs/Marker for RViz."""

    def __init__(self):
        super().__init__("survivor_pose_to_marker")

        self.declare_parameter("pose_subscription_topic", "/detected_survivor_pose")
        self.declare_parameter("marker_publication_topic", "/survivor_goal_marker")
        self.declare_parameter("marker_half_size", 0.75)
        self.declare_parameter("marker_z_lift", 0.15)
        self.declare_parameter("line_width", 0.12)
        self.declare_parameter("sphere_diameter", 0.4)
        self.declare_parameter("accumulate_markers", True)
        self.declare_parameter("log_each_pose", True)

        pose_topic = (
            self.get_parameter("pose_subscription_topic").get_parameter_value().string_value
        )
        marker_topic = (
            self.get_parameter("marker_publication_topic").get_parameter_value().string_value
        )
        self._half = self.get_parameter("marker_half_size").get_parameter_value().double_value
        self._z_lift = self.get_parameter("marker_z_lift").get_parameter_value().double_value
        self._line_width = self.get_parameter("line_width").get_parameter_value().double_value
        self._sphere_diameter = (
            self.get_parameter("sphere_diameter").get_parameter_value().double_value
        )
        self._accumulate = (
            self.get_parameter("accumulate_markers").get_parameter_value().bool_value
        )
        self._log_each = self.get_parameter("log_each_pose").get_parameter_value().bool_value
        self._next_pose_index = 0

        # Match `ros2 topic pub` defaults (Reliable).
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pub = self.create_publisher(Marker, marker_topic, qos)
        self._sub = self.create_subscription(PoseStamped, pose_topic, self._cb, qos)

        self.get_logger().info(
            f"survivor_pose_to_marker: sub {pose_topic} -> Marker pub {marker_topic} "
            f"(accumulate={self._accumulate})"
        )

    def _marker_ids(self) -> tuple[int, int]:
        if self._accumulate:
            base = self._next_pose_index * 2
            self._next_pose_index += 1
            return base, base + 1
        return 0, 1

    def _cb(self, msg: PoseStamped):
        x_id, sphere_id = self._marker_ids()
        self._pub.publish(_x_marker_msg(msg, self._half, self._z_lift, self._line_width, x_id))
        self._pub.publish(
            _sphere_marker_msg(msg, self._sphere_diameter, self._z_lift, sphere_id)
        )

        if self._log_each:
            p = msg.pose.position
            o = msg.pose.orientation
            self.get_logger().info(
                f"[survivor pose #{self._next_pose_index if self._accumulate else 1}] "
                f"marker_ids=({x_id}, {sphere_id}) frame={msg.header.frame_id} "
                f"xyz=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) "
                f"quat_xyzw=({o.x:.3f}, {o.y:.3f}, {o.z:.3f}, {o.w:.3f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = SurvivorPoseToMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
