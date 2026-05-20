"""Spin 360 after each explore_lite goal so the narrow camera FOV gets a
chance to see what the 360-deg LiDAR already covered.

Flow:
  1. Watch /navigate_to_pose/_action/status for SUCCEEDED transitions.
  2. On a new SUCCEEDED goal, pause explore (publish /explore/resume false)
     so it does not immediately fire the next goal.
  3. Send a Spin action (nav2_msgs/action/Spin) for 2*pi radians.
  4. When Spin finishes, resume explore (/explore/resume true).
"""

from __future__ import annotations

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from action_msgs.msg import GoalStatus, GoalStatusArray
from nav2_msgs.action import Spin
from std_msgs.msg import Bool


class RotateOnArrival(Node):
    def __init__(self):
        super().__init__('rotate_on_arrival')

        self.declare_parameter('spin_radians', 2.0 * math.pi)
        self.declare_parameter('spin_time_allowance_sec', 12.0)
        self.declare_parameter('status_topic', '/navigate_to_pose/_action/status')
        self.declare_parameter('spin_action_name', 'spin')
        self.declare_parameter('explore_resume_topic', '/explore/resume')

        self._spin_rad = self.get_parameter('spin_radians').get_parameter_value().double_value
        self._spin_timeout = self.get_parameter('spin_time_allowance_sec').get_parameter_value().double_value
        status_topic = self.get_parameter('status_topic').get_parameter_value().string_value
        spin_action = self.get_parameter('spin_action_name').get_parameter_value().string_value
        resume_topic = self.get_parameter('explore_resume_topic').get_parameter_value().string_value

        # Action status is published with the action server's default QoS
        # (reliable, transient_local, keep_last). Match it so we don't miss.
        action_status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._spin_client = ActionClient(self, Spin, spin_action)
        self._status_sub = self.create_subscription(
            GoalStatusArray, status_topic, self._on_status, action_status_qos)
        self._explore_resume_pub = self.create_publisher(Bool, resume_topic, 10)

        self._spinning = False
        self._seen_succeeded: set[bytes] = set()
        self.get_logger().info(
            f'rotate_on_arrival ready (status={status_topic} spin={spin_action} '
            f'angle={self._spin_rad:.2f} rad)'
        )

    def _on_status(self, msg: GoalStatusArray):
        if self._spinning:
            return
        for status in msg.status_list:
            if status.status != GoalStatus.STATUS_SUCCEEDED:
                continue
            goal_id = bytes(status.goal_info.goal_id.uuid)
            if goal_id in self._seen_succeeded:
                continue
            self._seen_succeeded.add(goal_id)
            # Cap memory; only the most recent 256 UUIDs matter for dedup.
            if len(self._seen_succeeded) > 256:
                self._seen_succeeded = set(list(self._seen_succeeded)[-128:])
            self._trigger_spin()
            return

    def _trigger_spin(self):
        if self._spinning:
            return
        self._spinning = True

        self.get_logger().info('Nav2 goal reached → pausing explore and spinning 360')
        self._publish_resume(False)

        if not self._spin_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Spin action server unavailable; aborting spin')
            self._cleanup()
            return

        goal = Spin.Goal()
        goal.target_yaw = float(self._spin_rad)
        goal.time_allowance = Duration(seconds=self._spin_timeout).to_msg()
        send_future = self._spin_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_spin_accepted)

    def _on_spin_accepted(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Spin send_goal failed: {exc}')
            self._cleanup()
            return
        if not handle.accepted:
            self.get_logger().warn('Spin goal rejected')
            self._cleanup()
            return
        handle.get_result_async().add_done_callback(self._on_spin_done)

    def _on_spin_done(self, _future):
        self._cleanup()

    def _cleanup(self):
        self._publish_resume(True)
        self._spinning = False
        self.get_logger().info('Spin done → explore resumed')

    def _publish_resume(self, value: bool):
        msg = Bool()
        msg.data = value
        self._explore_resume_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RotateOnArrival()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
