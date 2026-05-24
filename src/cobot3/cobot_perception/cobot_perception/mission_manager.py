"""Forward spot's survivor-pose detections into carter's Nav2 goal action.

Spot YOLO publishes PoseStamped on /detected_survivor_pose (default ns,
map frame). This node sends each pose to carter's NavigateToPose action,
but only one goal at a time:

  - While carter is still navigating to a goal, new detections are
    NOT dispatched. The most-recent detection is held in `_pending_pose`.
  - When the active goal completes (succeeded / aborted / canceled),
    the pending pose (if any) is dispatched as the next goal.
  - Detections within `dedup_distance_m` of the last dispatched goal or
    the current pending are dropped (noise from repeated detections of
    the same person).

This avoids carter flip-flopping mid-route when a second survivor is
detected before the first goal is reached.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class MissionManager(Node):
    def __init__(self):
        super().__init__("mission_manager")

        self.declare_parameter("input_topic", "/detected_survivor_pose")
        self.declare_parameter("nav_action", "/carter_0/navigate_to_pose")
        self.declare_parameter("dedup_distance_m", 0.5)

        in_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        nav_action = self.get_parameter("nav_action").get_parameter_value().string_value
        self._dedup_d = self.get_parameter("dedup_distance_m").get_parameter_value().double_value

        # Currently-dispatched goal (None when carter is idle).
        self._active_pose: PoseStamped | None = None
        # Goal handle for the dispatched NavigateToPose action; kept so we
        # can ignore result callbacks from already-superseded goals.
        self._active_goal_handle = None
        # Newest detection received while a goal is active. Dispatched
        # after the active goal completes.
        self._pending_pose: PoseStamped | None = None
        # Most recently dispatched pose, used for cross-dispatch dedup.
        self._last_dispatched: PoseStamped | None = None

        self._sub = self.create_subscription(PoseStamped, in_topic, self._on_pose, 10)
        self._nav_client = ActionClient(self, NavigateToPose, nav_action)

        self.get_logger().info(
            f"MissionManager: {in_topic} -> action {nav_action} (dedup {self._dedup_d:.2f} m)"
        )

    # ── helpers ──────────────────────────────────────────────────────
    def _planar_distance(self, a: PoseStamped, b: PoseStamped) -> float:
        return math.hypot(
            a.pose.position.x - b.pose.position.x,
            a.pose.position.y - b.pose.position.y,
        )

    def _is_duplicate(self, msg: PoseStamped) -> bool:
        for ref in (self._last_dispatched, self._pending_pose, self._active_pose):
            if ref is not None and self._planar_distance(msg, ref) < self._dedup_d:
                return True
        return False

    # ── detection inflow ─────────────────────────────────────────────
    def _on_pose(self, msg: PoseStamped) -> None:
        if self._is_duplicate(msg):
            return

        if self._active_pose is not None:
            # Carter is busy. Remember the newest non-duplicate detection
            # and dispatch it after the active goal finishes.
            self._pending_pose = msg
            self.get_logger().info(
                f"Pending goal queued ({msg.pose.position.x:.2f}, "
                f"{msg.pose.position.y:.2f}) — carter still moving"
            )
            return

        self._dispatch(msg)

    # ── action dispatch ──────────────────────────────────────────────
    def _dispatch(self, msg: PoseStamped) -> None:
        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("NavigateToPose server not ready; dropping goal")
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = msg.header.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose = msg.pose

        self._active_pose = goal.pose
        self._last_dispatched = goal.pose

        self.get_logger().info(
            f"Dispatch goal -> ({goal.pose.pose.position.x:.2f}, "
            f"{goal.pose.pose.position.y:.2f}) in '{goal.pose.header.frame_id}'"
        )

        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by NavigateToPose server")
            self._active_pose = None
            self._active_goal_handle = None
            self._maybe_dispatch_pending()
            return

        self._active_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        status = future.result().status
        self.get_logger().info(f"Goal finished with status={status}")
        self._active_pose = None
        self._active_goal_handle = None
        self._maybe_dispatch_pending()

    def _maybe_dispatch_pending(self) -> None:
        if self._pending_pose is None:
            return
        pending, self._pending_pose = self._pending_pose, None
        self._dispatch(pending)


def main():
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
