"""Phase 2: visit SLAM-known-free cells the camera never saw.

Runs alongside explore_lite. Stays idle until /explore/status reports
EXPLORATION_COMPLETE (Phase 1 done), then enters its own loop:

    while there are camera-unseen + slam-free cell clusters:
        pick the nearest unvisited cluster
        send NavigateToPose goal to the cluster's free centroid
        on arrival: 360-deg spin so the camera sees the area
        mark cluster visited

Goals go through the same Nav2 stack as explore_lite, but only one is
active at a time (this node waits for completion before sending the next).
"""

from __future__ import annotations

import math
import time
from typing import Optional, Set, Tuple

import numpy as np
import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class CameraCoverageSweep(Node):
    def __init__(self):
        super().__init__('camera_coverage_sweep')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('coverage_topic', '/camera_coverage')
        self.declare_parameter('explore_status_topic', '/explore/status')
        self.declare_parameter('robot_frame', 'spot_0/base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.declare_parameter('start_after_explore_complete', True)
        self.declare_parameter('startup_delay_sec', 5.0)
        self.declare_parameter('tick_period_sec', 3.0)
        self.declare_parameter('min_cluster_cells', 50)
        self.declare_parameter('visited_dedup_radius_m', 1.5)

        self.declare_parameter('do_spin_at_goal', True)
        self.declare_parameter('spin_duration_sec', 6.0)
        self.declare_parameter('spin_speed_rad_s', 1.0)

        self._map_frame = self.get_parameter('map_frame').value
        self._robot_frame = self.get_parameter('robot_frame').value
        self._wait_for_complete = bool(self.get_parameter('start_after_explore_complete').value)
        startup_delay = float(self.get_parameter('startup_delay_sec').value)
        self._min_cluster = int(self.get_parameter('min_cluster_cells').value)
        self._dedup_radius = float(self.get_parameter('visited_dedup_radius_m').value)
        self._do_spin = bool(self.get_parameter('do_spin_at_goal').value)
        self._spin_dur = float(self.get_parameter('spin_duration_sec').value)
        self._spin_speed = float(self.get_parameter('spin_speed_rad_s').value)

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._map: Optional[OccupancyGrid] = None
        self._coverage: Optional[OccupancyGrid] = None
        self._explore_complete = not self._wait_for_complete  # if not waiting, start immediately

        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value,
            self._on_map, latched_qos,
        )
        self.create_subscription(
            OccupancyGrid, self.get_parameter('coverage_topic').value,
            self._on_coverage, latched_qos,
        )

        # explore_status — best effort, gracefully handle msg missing
        self._has_explore_status_sub = False
        try:
            from explore_lite_msgs.msg import ExploreStatus
            self.create_subscription(
                ExploreStatus, self.get_parameter('explore_status_topic').value,
                self._on_explore_status, latched_qos,
            )
            self._has_explore_status_sub = True
        except Exception as exc:
            self.get_logger().warn(
                f'explore_lite_msgs unavailable ({exc}); will start sweep after startup_delay'
            )

        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10,
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Sweep state
        self._active_goal_handle = None
        self._busy = False  # True while a goal is in flight or while spinning
        self._spinning = False
        self._spin_start_ns = 0
        self._visited: Set[Tuple[float, float]] = set()  # rounded (x,y) of visited cluster centers
        self._sweep_started = False
        self._sweep_done = False
        self._launch_time = time.time()

        self.create_timer(0.1, self._spin_tick)
        self.create_timer(
            float(self.get_parameter('tick_period_sec').value),
            self._main_tick,
        )

        self.get_logger().info(
            f'camera_coverage_sweep ready (wait_for_explore_complete={self._wait_for_complete}, '
            f'startup_delay={startup_delay}s, min_cluster={self._min_cluster} cells, '
            f'spin={self._do_spin})'
        )

    # ------------------------------------------------------------------ #

    def _on_map(self, msg: OccupancyGrid):
        self._map = msg

    def _on_coverage(self, msg: OccupancyGrid):
        self._coverage = msg

    def _on_explore_status(self, msg):
        # ExploreStatus.EXPLORATION_COMPLETE == 4 in explore_lite_msgs
        if hasattr(msg, 'status') and msg.status == 4:
            if not self._explore_complete:
                self.get_logger().info(
                    'Phase 1 (explore_lite) reported COMPLETE → enabling sweep'
                )
            self._explore_complete = True

    # ------------------------------------------------------------------ #

    def _main_tick(self):
        if self._sweep_done or self._busy:
            return

        # Gate startup
        if not self._sweep_started:
            if not self._explore_complete:
                return
            elapsed = time.time() - self._launch_time
            startup_delay = float(self.get_parameter('startup_delay_sec').value)
            if elapsed < startup_delay:
                return
            self._sweep_started = True
            self.get_logger().info('Sweep phase START')

        if self._map is None or self._coverage is None:
            return
        if not self._maps_aligned():
            self.get_logger().warn('/map and /camera_coverage geometry mismatch — skipping tick')
            return

        target = self._find_next_target()
        if target is None:
            self.get_logger().info(
                f'No more unseen clusters (min {self._min_cluster} cells). '
                f'Sweep DONE after visiting {len(self._visited)} clusters.'
            )
            self._sweep_done = True
            return

        self._send_goal(target)

    def _maps_aligned(self) -> bool:
        m, c = self._map.info, self._coverage.info
        return (m.width == c.width and m.height == c.height
                and abs(m.resolution - c.resolution) < 1e-6
                and abs(m.origin.position.x - c.origin.position.x) < 1e-6
                and abs(m.origin.position.y - c.origin.position.y) < 1e-6)

    def _find_next_target(self) -> Optional[Tuple[float, float, int]]:
        info = self._map.info
        H, W = info.height, info.width
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y

        map_arr = np.array(self._map.data, dtype=np.int8).reshape((H, W))
        cov_arr = np.array(self._coverage.data, dtype=np.int8).reshape((H, W))
        unseen = (map_arr == 0) & (cov_arr == -1)
        if not unseen.any():
            return None

        try:
            from scipy.ndimage import label
        except Exception:
            self.get_logger().error('scipy.ndimage.label required; install python3-scipy')
            return None

        labeled, n_clusters = label(unseen)
        if n_clusters == 0:
            return None

        # Robot pose
        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame, self._robot_frame, rclpy.time.Time(),
            )
            rx = t.transform.translation.x
            ry = t.transform.translation.y
        except Exception:
            rx, ry = 0.0, 0.0

        best = None
        best_dist = float('inf')
        for i in range(1, n_clusters + 1):
            ys, xs = np.where(labeled == i)
            if len(ys) < self._min_cluster:
                continue
            cx_cell = float(xs.mean())
            cy_cell = float(ys.mean())
            wx = ox + (cx_cell + 0.5) * res
            wy = oy + (cy_cell + 0.5) * res

            # Visited dedup
            if any(math.hypot(wx - vx, wy - vy) < self._dedup_radius
                   for (vx, vy) in self._visited):
                continue

            # Use the cluster cell closest to the centroid (and free in /map)
            # to guarantee the goal is on a navigable cell.
            cell_xs = xs.astype(np.float32)
            cell_ys = ys.astype(np.float32)
            d2 = (cell_xs - cx_cell) ** 2 + (cell_ys - cy_cell) ** 2
            idx = int(np.argmin(d2))
            goal_gx = int(xs[idx])
            goal_gy = int(ys[idx])
            goal_wx = ox + (goal_gx + 0.5) * res
            goal_wy = oy + (goal_gy + 0.5) * res

            dist = math.hypot(goal_wx - rx, goal_wy - ry)
            if dist < best_dist:
                best_dist = dist
                best = (goal_wx, goal_wy, len(ys))

        return best

    # ------------------------------------------------------------------ #

    def _send_goal(self, target):
        wx, wy, n_cells = target
        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Nav2 NavigateToPose server unavailable')
            return

        self.get_logger().info(
            f'→ goal cluster center ({wx:+.2f}, {wy:+.2f}) size={n_cells} cells'
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self._map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = wx
        goal_msg.pose.pose.position.y = wy
        goal_msg.pose.pose.orientation.w = 1.0

        # Memorise visited center (rounded) so a small repositioning doesn't
        # re-target the same cluster.
        self._visited.add((round(wx, 1), round(wy, 1)))

        self._busy = True
        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'send_goal failed: {exc}')
            self._busy = False
            return
        if not handle.accepted:
            self.get_logger().warn('Goal rejected')
            self._busy = False
            return
        self._active_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        try:
            result = future.result()
            status = result.status
        except Exception as exc:
            self.get_logger().error(f'Goal result failed: {exc}')
            self._busy = False
            return

        names = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
                 GoalStatus.STATUS_ABORTED:   'ABORTED',
                 GoalStatus.STATUS_CANCELED:  'CANCELED'}
        self.get_logger().info(f'goal result: {names.get(status, status)}')

        self._active_goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED and self._do_spin:
            # Trigger spin; _spin_tick will publish cmd_vel for spin_duration,
            # then clear _busy.
            self._spinning = True
            self._spin_start_ns = self.get_clock().now().nanoseconds
        else:
            self._busy = False  # ready for next pick

    # ------------------------------------------------------------------ #

    def _spin_tick(self):
        if not self._spinning:
            return
        elapsed = (self.get_clock().now().nanoseconds - self._spin_start_ns) / 1e9
        if elapsed < self._spin_dur:
            t = Twist()
            t.angular.z = self._spin_speed
            self._cmd_pub.publish(t)
            return
        # Done
        stop = Twist()
        for _ in range(3):
            self._cmd_pub.publish(stop)
        self._spinning = False
        self._busy = False


def main(args=None):
    rclpy.init(args=args)
    node = CameraCoverageSweep()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
