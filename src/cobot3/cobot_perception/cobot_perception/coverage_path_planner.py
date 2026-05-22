"""Coverage Path Planning (boustrophedon-style) explorer.

Drives the robot to a grid of waypoints spaced ``waypoint_spacing_m``
apart (default = camera range, so the camera covers every cell as the
robot passes through). Replaces explore_lite + camera_coverage_sweep
entirely.

Waypoint generation:
    1. Read /global_costmap/costmap (so inflation is already accounted for).
    2. Sample on a grid of ``spacing_m`` × ``spacing_m``.
    3. Keep only cells with cost == 0 (FREE_SPACE — drivable, not inflated).
    4. Persist visited flag across replans so we don't re-target.

Order:
    Greedy nearest-unvisited from robot's current position. Predictable
    coverage without boustrophedon row math; also handles the
    "robot spawned far from any wall" case naturally.

Replan:
    When the costmap grows or its bounds shift, regenerate waypoint list
    but PRESERVE visited flags by position match (rounded coords).

On unreachable waypoints:
    Mark visited even on ABORTED — avoids infinite retry, just skips.
"""

from __future__ import annotations

import math
import time
from typing import Dict, List, Optional, Tuple

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
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


WaypointKey = Tuple[int, int]   # rounded grid index, used as dict key


class CoveragePathPlanner(Node):
    def __init__(self):
        super().__init__('coverage_path_planner')

        # ── Parameters ──
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('coverage_topic', '/camera_coverage')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('robot_frame', 'spot_0/base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.declare_parameter('waypoint_spacing_m', 2.0)
        self.declare_parameter('tick_period_sec', 1.5)
        self.declare_parameter('replan_period_sec', 8.0)
        self.declare_parameter('startup_delay_sec', 8.0)

        # ── Zone partitioning ──
        # Divide the map into ``zone_size_m`` × ``zone_size_m`` squares and
        # complete every waypoint in one zone before moving to the next.
        # Set to <= 0 to disable zoning (use plain nearest-unvisited).
        self.declare_parameter('zone_size_m', 5.0)

        # Skip a candidate waypoint if its cell (or a small disk around it)
        # is already camera-seen — no need to revisit.
        self.declare_parameter('skip_already_seen', True)
        self.declare_parameter('seen_skip_radius_m', 0.5)

        self.declare_parameter('do_spin_at_waypoint', True)
        self.declare_parameter('spin_duration_sec', 4.0)
        self.declare_parameter('spin_speed_rad_s', 1.0)

        self._wp_spacing = float(self.get_parameter('waypoint_spacing_m').value)
        self._replan_period = float(self.get_parameter('replan_period_sec').value)
        self._map_frame = self.get_parameter('map_frame').value
        self._robot_frame = self.get_parameter('robot_frame').value
        self._do_spin = bool(self.get_parameter('do_spin_at_waypoint').value)
        self._spin_dur = float(self.get_parameter('spin_duration_sec').value)
        self._spin_speed = float(self.get_parameter('spin_speed_rad_s').value)
        self._skip_seen = bool(self.get_parameter('skip_already_seen').value)
        self._seen_skip_radius = float(self.get_parameter('seen_skip_radius_m').value)
        self._zone_size = float(self.get_parameter('zone_size_m').value)
        self._zone_enabled = self._zone_size > 0.0
        self._current_zone: Optional[Tuple[int, int]] = None
        startup_delay = float(self.get_parameter('startup_delay_sec').value)

        # ── State ──
        self._costmap: Optional[OccupancyGrid] = None
        self._coverage: Optional[OccupancyGrid] = None
        # waypoint dict: (grid_x, grid_y) -> {world_xy, visited}
        self._waypoints: Dict[WaypointKey, Dict] = {}
        self._busy = False
        self._spinning = False
        self._spin_start_ns = 0
        self._current_wp_key: Optional[WaypointKey] = None
        self._launch_time = time.time()
        self._last_replan_ts = 0.0

        # ── ROS ──
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            OccupancyGrid, self.get_parameter('costmap_topic').value,
            self._on_costmap, latched_qos,
        )
        self.create_subscription(
            OccupancyGrid, self.get_parameter('coverage_topic').value,
            self._on_coverage, latched_qos,
        )
        self._cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10,
        )
        self._zones_pub = self.create_publisher(
            MarkerArray, '/coverage_zones', latched_qos,
        )
        self._wps_pub = self.create_publisher(
            MarkerArray, '/coverage_waypoints', latched_qos,
        )
        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_timer(float(self.get_parameter('tick_period_sec').value), self._main_tick)
        self.create_timer(0.1, self._spin_tick)

        self.get_logger().info(
            f'coverage_path_planner ready (spacing={self._wp_spacing}m, '
            f'startup_delay={startup_delay}s, spin={self._do_spin})'
        )

    # ------------------------------------------------------------------ #

    def _on_costmap(self, msg: OccupancyGrid):
        self._costmap = msg

    def _on_coverage(self, msg: OccupancyGrid):
        self._coverage = msg

    def _get_robot_pose(self) -> Optional[Tuple[float, float]]:
        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame, self._robot_frame, rclpy.time.Time(),
            )
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

    # ------------------------------------------------------------------ #

    def _replan(self):
        """Regenerate waypoint grid from current costmap, preserve visited."""
        if self._costmap is None:
            return

        info = self._costmap.info
        H, W = info.height, info.width
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y

        arr = np.array(self._costmap.data, dtype=np.int8).reshape((H, W))
        spacing_cells = max(1, int(round(self._wp_spacing / res)))

        # Optional camera-seen check — skip waypoint if its surrounding disk
        # in /camera_coverage is already seen.
        cov_arr = None
        cov_aligned = False
        seen_r_cells = 0
        if self._skip_seen and self._coverage is not None:
            ci = self._coverage.info
            if (ci.width == W and ci.height == H
                    and abs(ci.resolution - res) < 1e-6
                    and abs(ci.origin.position.x - ox) < 1e-6
                    and abs(ci.origin.position.y - oy) < 1e-6):
                cov_arr = np.array(self._coverage.data, dtype=np.int8).reshape((H, W))
                cov_aligned = True
                seen_r_cells = max(1, int(round(self._seen_skip_radius / res)))

        new_wps: Dict[WaypointKey, Dict] = {}
        skipped_seen = 0
        # For each ``spacing_cells × spacing_cells`` window of the grid, find
        # the FREE costmap cell closest to the window centre and put one
        # waypoint there. Plain grid sampling missed almost every cell
        # because at this stage FREE is < 1 % of the costmap.
        half = spacing_cells // 2
        for win_gy in range(half, H, spacing_cells):
            for win_gx in range(half, W, spacing_cells):
                y0 = max(0, win_gy - half)
                y1 = min(H, win_gy + half + 1)
                x0 = max(0, win_gx - half)
                x1 = min(W, win_gx + half + 1)
                sub = arr[y0:y1, x0:x1]
                free_local_ys, free_local_xs = np.where(sub == 0)
                if len(free_local_ys) == 0:
                    continue
                free_ys = free_local_ys + y0
                free_xs = free_local_xs + x0
                # Pick FREE cell closest to window centre
                d2 = (free_ys - win_gy) ** 2 + (free_xs - win_gx) ** 2
                idx = int(np.argmin(d2))
                gy = int(free_ys[idx])
                gx = int(free_xs[idx])
                # Skip if waypoint disk is already mostly camera-seen
                if cov_aligned:
                    sy0 = max(0, gy - seen_r_cells)
                    sy1 = min(H, gy + seen_r_cells + 1)
                    sx0 = max(0, gx - seen_r_cells)
                    sx1 = min(W, gx + seen_r_cells + 1)
                    cov_sub = cov_arr[sy0:sy1, sx0:sx1]
                    if cov_sub.size > 0:
                        seen_ratio = (cov_sub == 0).sum() / cov_sub.size
                        if seen_ratio >= 0.8:
                            skipped_seen += 1
                            continue
                key: WaypointKey = (gx, gy)
                wx = ox + (gx + 0.5) * res
                wy = oy + (gy + 0.5) * res
                existing = self._waypoints.get(key)
                visited = bool(existing and existing.get('visited'))
                zone = (int(math.floor(wx / self._zone_size)),
                        int(math.floor(wy / self._zone_size))) if self._zone_enabled else None
                new_wps[key] = {'x': wx, 'y': wy, 'visited': visited, 'zone': zone}

        added = len(new_wps) - len(self._waypoints)
        removed = len(self._waypoints) - len([k for k in self._waypoints if k in new_wps])
        self._waypoints = new_wps
        visited_count = sum(1 for w in new_wps.values() if w['visited'])
        self.get_logger().info(
            f'replan: {len(new_wps)} waypoints (+{added} new, -{removed} dropped, '
            f'{skipped_seen} skipped already-seen), {visited_count} visited'
        )
        self._publish_zone_viz()
        self._publish_waypoint_viz()

    def _pick_next_waypoint(self) -> Optional[WaypointKey]:
        """If zoning is on: finish current zone before moving on. Otherwise
        plain greedy nearest unvisited."""
        pose = self._get_robot_pose()
        if pose is None:
            return None
        rx, ry = pose

        if not self._zone_enabled:
            best_key = None
            best_dist = float('inf')
            for key, wp in self._waypoints.items():
                if wp['visited']:
                    continue
                d = math.hypot(wp['x'] - rx, wp['y'] - ry)
                if d < best_dist:
                    best_dist = d
                    best_key = key
            return best_key

        # Pick / advance current zone
        if self._current_zone is None:
            self._current_zone = (int(math.floor(rx / self._zone_size)),
                                  int(math.floor(ry / self._zone_size)))
            self.get_logger().info(f'starting zone {self._current_zone}')

        # Try to pick nearest unvisited within current zone
        best_key, best_dist = None, float('inf')
        for key, wp in self._waypoints.items():
            if wp['visited'] or wp['zone'] != self._current_zone:
                continue
            d = math.hypot(wp['x'] - rx, wp['y'] - ry)
            if d < best_dist:
                best_dist = d
                best_key = key
        if best_key is not None:
            return best_key

        # Current zone is done — advance to nearest unvisited zone
        unvisited_zones = {wp['zone'] for wp in self._waypoints.values()
                           if not wp['visited'] and wp['zone'] is not None}
        if not unvisited_zones:
            return None

        def zone_center(z):
            return ((z[0] + 0.5) * self._zone_size,
                    (z[1] + 0.5) * self._zone_size)

        new_zone = min(unvisited_zones,
                       key=lambda z: math.hypot(zone_center(z)[0] - rx,
                                                zone_center(z)[1] - ry))
        self.get_logger().info(
            f'zone {self._current_zone} done → entering zone {new_zone}'
        )
        self._current_zone = new_zone

        # Now find waypoint in the new zone
        for key, wp in self._waypoints.items():
            if wp['visited'] or wp['zone'] != self._current_zone:
                continue
            d = math.hypot(wp['x'] - rx, wp['y'] - ry)
            if d < best_dist:
                best_dist = d
                best_key = key
        return best_key

    # ------------------------------------------------------------------ #

    def _main_tick(self):
        # Startup delay so Nav2 has time to come up
        if time.time() - self._launch_time < float(self.get_parameter('startup_delay_sec').value):
            return
        if self._costmap is None:
            return

        # Periodic replan (or first-time replan)
        if (not self._waypoints
                or time.time() - self._last_replan_ts > self._replan_period):
            self._replan()
            self._last_replan_ts = time.time()

        if self._busy:
            return

        next_key = self._pick_next_waypoint()
        if next_key is None:
            unvisited = sum(1 for w in self._waypoints.values() if not w['visited'])
            if unvisited == 0 and self._waypoints:
                self.get_logger().info(
                    f'ALL {len(self._waypoints)} waypoints visited — coverage complete.'
                )
            return

        wp = self._waypoints[next_key]
        self._send_goal(next_key, wp['x'], wp['y'])

    # ------------------------------------------------------------------ #

    def _send_goal(self, key: WaypointKey, wx: float, wy: float):
        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Nav2 server unavailable')
            return

        n_visited = sum(1 for w in self._waypoints.values() if w['visited'])
        n_total = len(self._waypoints)
        self.get_logger().info(
            f'→ wp [{n_visited+1}/{n_total}] ({wx:+.2f}, {wy:+.2f})'
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self._map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = wx
        goal_msg.pose.pose.position.y = wy
        # Face the waypoint so the robot walks forward (camera looking
        # forward) instead of backing up. Without this Nav2/MPPI under
        # "Omni" motion_model often picks reverse motion as shorter when
        # the waypoint is behind the robot.
        pose = self._get_robot_pose()
        if pose is not None:
            rx, ry = pose
            yaw = math.atan2(wy - ry, wx - rx)
            goal_msg.pose.pose.orientation.x = 0.0
            goal_msg.pose.pose.orientation.y = 0.0
            goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            goal_msg.pose.pose.orientation.w = 1.0

        self._busy = True
        self._current_wp_key = key
        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_goal_response)
        # Refresh viz so current zone/waypoint highlight is up-to-date
        self._publish_zone_viz()
        self._publish_waypoint_viz()

    def _on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'send_goal failed: {exc}')
            self._mark_visited_and_release()
            return
        if not handle.accepted:
            self.get_logger().warn('Goal rejected')
            self._mark_visited_and_release()
            return
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        try:
            res = future.result()
            status = res.status
        except Exception as exc:
            self.get_logger().error(f'goal result failed: {exc}')
            self._mark_visited_and_release()
            return

        name = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
                GoalStatus.STATUS_ABORTED: 'ABORTED',
                GoalStatus.STATUS_CANCELED: 'CANCELED'}.get(status, str(status))
        self.get_logger().info(f'   result: {name}')

        # Either way, mark visited so we don't pick it forever. SUCCESS:
        # great. ABORTED: unreachable, skip. CANCELED: shouldn't happen
        # here but treat same.
        if status == GoalStatus.STATUS_SUCCEEDED and self._do_spin:
            # Trigger spin, _spin_tick will release _busy when done.
            self._mark_visited()
            self._spinning = True
            self._spin_start_ns = self.get_clock().now().nanoseconds
        else:
            self._mark_visited_and_release()

    def _mark_visited(self):
        if self._current_wp_key is not None and self._current_wp_key in self._waypoints:
            self._waypoints[self._current_wp_key]['visited'] = True

    def _mark_visited_and_release(self):
        self._mark_visited()
        self._current_wp_key = None
        self._busy = False

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
        stop = Twist()
        for _ in range(3):
            self._cmd_pub.publish(stop)
        self._spinning = False
        self._current_wp_key = None
        self._busy = False

    # ------------------------------------------------------------------ #
    # Visualization

    def _publish_delete_all_markers(self, publisher, namespace: str):
        ma = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self._map_frame
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.ns = namespace
        clear.id = -1
        clear.pose.orientation.w = 1.0
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        publisher.publish(ma)

    def _publish_zone_viz(self):
        """One LINE_LIST + TEXT marker per zone — colored by state."""
        if not self._zone_enabled or not self._waypoints:
            self._publish_delete_all_markers(self._zones_pub, 'coverage_zones')
            return

        # Aggregate per-zone: (visited_count, total_count)
        per_zone: Dict[Tuple[int, int], Tuple[int, int]] = {}
        for wp in self._waypoints.values():
            z = wp['zone']
            if z is None:
                continue
            v, t = per_zone.get(z, (0, 0))
            per_zone[z] = (v + (1 if wp['visited'] else 0), t + 1)

        self._publish_delete_all_markers(self._zones_pub, 'coverage_zones')
        ma = MarkerArray()

        mid = 0
        for zone, (v, total) in per_zone.items():
            x0 = zone[0] * self._zone_size
            y0 = zone[1] * self._zone_size
            x1 = x0 + self._zone_size
            y1 = y0 + self._zone_size

            # Color by state
            is_current = (zone == self._current_zone)
            done = (v == total)
            if is_current:
                c = ColorRGBA(r=0.2, g=1.0, b=0.2, a=0.8)  # green
            elif done:
                c = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.4)  # gray
            else:
                c = ColorRGBA(r=0.3, g=0.6, b=1.0, a=0.5)  # light blue

            # Zone outline (LINE_STRIP)
            outline = Marker()
            outline.header.frame_id = self._map_frame
            outline.header.stamp = self.get_clock().now().to_msg()
            outline.ns = 'coverage_zones'
            outline.id = mid
            mid += 1
            outline.type = Marker.LINE_STRIP
            outline.action = Marker.ADD
            outline.pose.orientation.w = 1.0
            outline.scale.x = 0.1  # line thickness
            outline.color = c
            outline.points = [
                _pt(x0, y0), _pt(x1, y0), _pt(x1, y1), _pt(x0, y1), _pt(x0, y0),
            ]
            ma.markers.append(outline)

            # Zone label (TEXT_VIEW_FACING)
            text = Marker()
            text.header.frame_id = self._map_frame
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = 'coverage_zones'
            text.id = mid
            mid += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = (x0 + x1) / 2.0
            text.pose.position.y = (y0 + y1) / 2.0
            text.pose.position.z = 0.5
            text.pose.orientation.w = 1.0
            text.scale.z = 0.8
            text.color = c
            text.text = f'{zone}\n{v}/{total}'
            ma.markers.append(text)

        self._zones_pub.publish(ma)

    def _publish_waypoint_viz(self):
        """Sphere per waypoint — colored by visited / current."""
        if not self._waypoints:
            self._publish_delete_all_markers(self._wps_pub, 'coverage_waypoints')
            return

        self._publish_delete_all_markers(self._wps_pub, 'coverage_waypoints')
        ma = MarkerArray()

        for mid, (key, wp) in enumerate(self._waypoints.items()):
            m = Marker()
            m.header.frame_id = self._map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'coverage_waypoints'
            m.id = mid + 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wp['x']
            m.pose.position.y = wp['y']
            m.pose.position.z = 0.2
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.25
            if key == self._current_wp_key:
                m.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # yellow
                m.scale.x = m.scale.y = m.scale.z = 0.45
            elif wp['visited']:
                m.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.6)  # gray
            else:
                m.color = ColorRGBA(r=0.9, g=0.3, b=0.8, a=0.9)  # magenta
            ma.markers.append(m)

        self._wps_pub.publish(ma)


def _pt(x, y, z=0.05):
    from geometry_msgs.msg import Point
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = float(z)
    return p


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
