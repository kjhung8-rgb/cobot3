"""Stop watchdog — diagnostic snapshot whenever the robot stops moving.

Run alongside the main explore launch. Subscribes to /cmd_vel; when no
non-trivial cmd arrives for ``idle_threshold_sec``, captures and prints
the state of explore, last few Nav2 goal results, frontier count, robot
position and camera-coverage trend. Logs to console *and* to a file so
you can leave the test running and review afterwards.

Run:
    ros2 run cobot_perception stop_watchdog \
        --ros-args -p idle_threshold_sec:=5.0 -p log_file:=/tmp/spot_stops.log
"""

from __future__ import annotations

import collections
import time
from typing import Deque

import numpy as np
import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import MarkerArray


_GOAL_STATUS = {
    0: '?', 1: 'ACCEPTED', 2: 'EXEC',
    3: 'CANCELING', 4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED',
}

_EXPLORE_STATUS = {
    0: 'IDLE', 1: 'STARTED', 2: 'IN_PROGRESS',
    3: 'PAUSED', 4: 'COMPLETE', 5: 'RETURNING_TO_ORIGIN',
}


class StopWatchdog(Node):
    def __init__(self):
        super().__init__('stop_watchdog')

        self.declare_parameter('idle_threshold_sec', 5.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('log_file', '/tmp/spot_stops.log')
        self.declare_parameter('robot_frame', 'spot_0/base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('heartbeat_sec', 30.0)

        self._idle_threshold = float(self.get_parameter('idle_threshold_sec').value)
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        self._log_file = self.get_parameter('log_file').value
        self._robot_frame = self.get_parameter('robot_frame').value
        self._map_frame = self.get_parameter('map_frame').value
        self._heartbeat = float(self.get_parameter('heartbeat_sec').value)

        # State
        self._last_cmd_time = time.time()
        self._launch_time = time.time()
        self._is_stopped = False
        self._stopped_at = 0.0
        self._goal_results: Deque[int] = collections.deque(maxlen=20)
        self._seen_goal_ids: set = set()
        self._latest_explore_status = 0
        self._latest_frontier_count = 0
        self._latest_seen_cells = 0
        self._prev_seen_cells = 0
        self._prev_seen_time = time.time()
        self._last_heartbeat = time.time()

        # /cmd_vel
        self.create_subscription(Twist, cmd_topic, self._on_cmd_vel, 10)

        # Nav action status (transient_local)
        action_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self._on_nav_status, action_qos,
        )

        # explore status (custom msg may or may not exist; subscribe generically)
        try:
            from explore_lite_msgs.msg import ExploreStatus
            self.create_subscription(
                ExploreStatus, '/explore/status', self._on_explore_status, action_qos,
            )
        except Exception:
            self.get_logger().warn('explore_lite_msgs not importable; explore status unavailable')

        # frontiers (MarkerArray)
        self.create_subscription(
            MarkerArray, '/explore/frontiers', self._on_frontiers, 10,
        )

        # camera coverage
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            OccupancyGrid, '/camera_coverage', self._on_camera_coverage, latched_qos,
        )

        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_timer(1.0, self._tick)

        banner = (
            f'stop_watchdog ready (idle>{self._idle_threshold}s '
            f'→ snapshot; log={self._log_file})'
        )
        self.get_logger().info(banner)
        self._log_line(banner)

    # ------------------------------------------------------------------ #

    def _on_cmd_vel(self, msg: Twist):
        active = (abs(msg.linear.x) > 0.01
                  or abs(msg.linear.y) > 0.01
                  or abs(msg.angular.z) > 0.02)
        if not active:
            return
        now = time.time()
        self._last_cmd_time = now
        if self._is_stopped:
            stopped_for = now - self._stopped_at
            self._log_line(f'[RESUMED] after {stopped_for:.1f}s of stillness')
            self._is_stopped = False

    def _on_nav_status(self, msg: GoalStatusArray):
        for status in msg.status_list:
            gid = bytes(status.goal_info.goal_id.uuid)
            if gid in self._seen_goal_ids:
                continue
            if status.status in (GoalStatus.STATUS_SUCCEEDED,
                                  GoalStatus.STATUS_CANCELED,
                                  GoalStatus.STATUS_ABORTED):
                self._seen_goal_ids.add(gid)
                self._goal_results.append(status.status)
                if len(self._seen_goal_ids) > 200:
                    self._seen_goal_ids = set(list(self._seen_goal_ids)[-100:])

    def _on_explore_status(self, msg):
        self._latest_explore_status = msg.status

    def _on_frontiers(self, msg: MarkerArray):
        # Each frontier is published as one marker; DELETE markers excluded.
        cnt = sum(1 for m in msg.markers if m.action != 2)
        self._latest_frontier_count = cnt

    def _on_camera_coverage(self, msg: OccupancyGrid):
        arr = np.array(msg.data, dtype=np.int8)
        self._latest_seen_cells = int(np.count_nonzero(arr == 0))

    # ------------------------------------------------------------------ #

    def _tick(self):
        now = time.time()
        idle_for = now - self._last_cmd_time
        if not self._is_stopped and idle_for > self._idle_threshold:
            self._is_stopped = True
            self._stopped_at = self._last_cmd_time
            self._snapshot('STOP DETECTED')
        # heartbeat: even when alive, periodically log progress
        if now - self._last_heartbeat >= self._heartbeat:
            self._last_heartbeat = now
            self._snapshot('heartbeat')

    def _snapshot(self, label: str):
        results = list(self._goal_results)
        last5 = ', '.join(_GOAL_STATUS.get(s, '?') for s in results[-5:]) or '(none)'
        explore_str = _EXPLORE_STATUS.get(self._latest_explore_status,
                                          str(self._latest_explore_status))

        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._robot_frame, rclpy.time.Time(),
            )
            pos_str = f'({tf.transform.translation.x:+.2f}, {tf.transform.translation.y:+.2f})'
        except Exception:
            pos_str = '(TF n/a)'

        dt_seen = time.time() - self._prev_seen_time
        delta = self._latest_seen_cells - self._prev_seen_cells
        if dt_seen <= 0:
            trend = '?'
        elif delta > 100:
            trend = f'GROWING (+{delta} in {dt_seen:.0f}s)'
        elif abs(delta) < 20:
            trend = f'STAGNANT (Δ{delta:+d} in {dt_seen:.0f}s)'
        else:
            trend = f'SLOW (Δ{delta:+d} in {dt_seen:.0f}s)'

        elapsed = time.time() - self._launch_time

        msg = (
            f'\n[{label}] T+{elapsed:.0f}s  robot {pos_str}\n'
            f'   explore_status   : {explore_str}\n'
            f'   last 5 nav goals : {last5}\n'
            f'   frontier markers : {self._latest_frontier_count}\n'
            f'   camera seen cells: {self._latest_seen_cells}  [{trend}]'
        )

        # Heuristic guess
        guesses = []
        if explore_str == 'PAUSED':
            guesses.append('① rotate_on_arrival deadlock')
        if explore_str == 'COMPLETE':
            guesses.append('② explore declared empty frontier and stopped')
        succeeded = sum(1 for s in results[-5:] if s == GoalStatus.STATUS_SUCCEEDED)
        aborted = sum(1 for s in results[-5:] if s == GoalStatus.STATUS_ABORTED)
        canceled = sum(1 for s in results[-5:] if s == GoalStatus.STATUS_CANCELED)
        if aborted >= 2:
            guesses.append(f'③ progress_checker timeout / unreachable ({aborted}/5 ABORTED)')
        if canceled >= 2:
            guesses.append(f'rotate cancelling newly-sent goals ({canceled}/5 CANCELED)')
        if self._latest_frontier_count == 0:
            guesses.append('frontier search empty')
        if 'STAGNANT' in trend:
            guesses.append('⑦ camera tracker stuck OR robot not moving')
        if label == 'STOP DETECTED' and guesses:
            msg += '\n   suspected        : ' + '; '.join(guesses)

        self._log_line(msg)

        self._prev_seen_cells = self._latest_seen_cells
        self._prev_seen_time = time.time()

    # ------------------------------------------------------------------ #

    def _log_line(self, text: str):
        ts = time.strftime('%H:%M:%S')
        line = f'[{ts}] {text}'
        self.get_logger().warn(line)
        try:
            with open(self._log_file, 'a') as fh:
                fh.write(line + '\n')
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = StopWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
