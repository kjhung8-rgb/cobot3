"""Subscribe to survivor PoseStamped; publish RViz Marker (red X + center sphere).

Use until survivor_detector publishes the same PoseStamped from YOLO+localization.
Temporary test: ros2 topic pub --once /detected_survivor_pose geometry_msgs/msg/PoseStamped '...'
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, Int32
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


def _label_marker_msg(
    pose: PoseStamped, label: str, z_lift: float, text_height: float, marker_id: int
) -> Marker:
    m = Marker()
    m.header = pose.header
    m.ns = "survivor"
    m.id = marker_id
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.pose.position.x = pose.pose.position.x
    m.pose.position.y = pose.pose.position.y
    m.pose.position.z = pose.pose.position.z + z_lift
    m.pose.orientation.w = 1.0
    m.scale.z = text_height
    col = ColorRGBA()
    col.r, col.g, col.b, col.a = 1.0, 1.0, 1.0, 1.0
    m.color = col
    m.text = label
    return m


def _delete_marker_msg(frame_id: str, marker_id: int) -> Marker:
    m = Marker()
    m.header.frame_id = frame_id
    m.ns = "survivor"
    m.id = marker_id
    m.action = Marker.DELETE
    return m


def _delete_all_marker_msg(frame_id: str) -> Marker:
    m = Marker()
    m.header.frame_id = frame_id
    m.ns = "survivor"
    m.action = Marker.DELETEALL
    return m


class SurvivorPoseToMarker(Node):
    """Bridge: PoseStamped (YOLO 등) -> visualization_msgs/Marker for RViz."""

    def __init__(self):
        super().__init__("survivor_pose_to_marker")

        self.declare_parameter("pose_subscription_topic", "/detected_survivor_pose")
        self.declare_parameter("marker_publication_topic", "/survivor_goal_marker")
        self.declare_parameter("survivor_delete_topic", "/survivor_delete_id")
        self.declare_parameter("delete_marker_frame", "map")
        self.declare_parameter("marker_half_size", 0.75)
        self.declare_parameter("marker_z_lift", 0.15)
        self.declare_parameter("label_z_lift", 0.8)
        self.declare_parameter("label_text_height", 0.45)
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
        delete_topic = (
            self.get_parameter("survivor_delete_topic").get_parameter_value().string_value
        )
        self._delete_frame = (
            self.get_parameter("delete_marker_frame").get_parameter_value().string_value
        )
        self._half = self.get_parameter("marker_half_size").get_parameter_value().double_value
        self._z_lift = self.get_parameter("marker_z_lift").get_parameter_value().double_value
        self._label_z_lift = (
            self.get_parameter("label_z_lift").get_parameter_value().double_value
        )
        self._label_text_height = (
            self.get_parameter("label_text_height").get_parameter_value().double_value
        )
        self._line_width = self.get_parameter("line_width").get_parameter_value().double_value
        self._sphere_diameter = (
            self.get_parameter("sphere_diameter").get_parameter_value().double_value
        )
        self._accumulate = (
            self.get_parameter("accumulate_markers").get_parameter_value().bool_value
        )
        self._log_each = self.get_parameter("log_each_pose").get_parameter_value().bool_value
        self._next_pose_index = 0
        self._last_marker_frame = self._delete_frame

        # Match `ros2 topic pub` defaults (Reliable).
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pub = self.create_publisher(Marker, marker_topic, qos)
        self._sub = self.create_subscription(PoseStamped, pose_topic, self._cb, qos)
        self._delete_sub = self.create_subscription(
            Int32, delete_topic, self._on_delete_survivor, qos
        )

        self.get_logger().info(
            f"survivor_pose_to_marker: sub {pose_topic} -> Marker pub {marker_topic} "
            f"delete_sub {delete_topic} (accumulate={self._accumulate})"
        )

    def _next_marker_ids(self) -> tuple[int, int, int, int]:
        if self._accumulate:
            survivor_id = self._next_pose_index + 1
            base = self._marker_base_id(survivor_id)
            self._next_pose_index += 1
            return survivor_id, base, base + 1, base + 2
        return 1, 0, 1, 2

    @staticmethod
    def _marker_base_id(survivor_id: int) -> int:
        return (survivor_id - 1) * 3

    def _cb(self, msg: PoseStamped):
        self._last_marker_frame = msg.header.frame_id or self._delete_frame
        survivor_id, x_id, sphere_id, label_id = self._next_marker_ids()
        self._pub.publish(
            _x_marker_msg(msg, self._half, self._z_lift, self._line_width, x_id)
        )
        self._pub.publish(
            _sphere_marker_msg(msg, self._sphere_diameter, self._z_lift, sphere_id)
        )
        self._pub.publish(
            _label_marker_msg(
                msg,
                f"#{survivor_id}",
                self._label_z_lift,
                self._label_text_height,
                label_id,
            )
        )

        if self._log_each:
            p = msg.pose.position
            o = msg.pose.orientation
            self.get_logger().info(
                f"[survivor marker #{survivor_id}] "
                f"marker_ids=({x_id}, {sphere_id}, {label_id}) "
                f"frame={msg.header.frame_id} "
                f"xyz=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) "
                f"quat_xyzw=({o.x:.3f}, {o.y:.3f}, {o.z:.3f}, {o.w:.3f})"
            )

    def _on_delete_survivor(self, msg: Int32):
        survivor_id = int(msg.data)
        frame_id = self._last_marker_frame or self._delete_frame

        if survivor_id == 0:
            delete_msg = _delete_all_marker_msg(frame_id)
            delete_msg.header.stamp = self.get_clock().now().to_msg()
            self._pub.publish(delete_msg)
            self._next_pose_index = 0
            self.get_logger().info("deleted all survivor markers")
            return

        if survivor_id < 0:
            self.get_logger().warn(
                f"ignoring invalid survivor delete id {survivor_id}; use 0 for all"
            )
            return

        if self._accumulate:
            x_id = self._marker_base_id(survivor_id)
            sphere_id = x_id + 1
            label_id = x_id + 2
        else:
            x_id, sphere_id, label_id = 0, 1, 2

        stamp = self.get_clock().now().to_msg()
        x_delete = _delete_marker_msg(frame_id, x_id)
        sphere_delete = _delete_marker_msg(frame_id, sphere_id)
        label_delete = _delete_marker_msg(frame_id, label_id)
        x_delete.header.stamp = stamp
        sphere_delete.header.stamp = stamp
        label_delete.header.stamp = stamp
        self._pub.publish(x_delete)
        self._pub.publish(sphere_delete)
        self._pub.publish(label_delete)
        self.get_logger().info(
            f"deleted survivor marker #{survivor_id} "
            f"marker_ids=({x_id}, {sphere_id}, {label_id})"
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
