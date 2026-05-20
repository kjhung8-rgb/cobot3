"""Track which map cells the front camera has actually seen.

Background: explore_lite drives the robot toward LiDAR-unknown cells. But
the LiDAR has 360-deg / 25 m reach, so it maps rooms through doorways from
the corridor and the robot never enters. The camera (~70-deg FOV) misses
everyone inside those rooms.

This node maintains a *second* grid aligned with SLAM /map:
  -1 = camera has not seen this cell
   0 = camera has seen this cell

Then it republishes the SLAM map masked by this grid as /map_explorable:
  obstacles (>50 in /map)              -> 100  (preserved)
  free (0 in /map) AND camera-seen     -> 0    (free)
  everything else                      -> -1   (unknown)

If global_costmap.static_layer.map_topic is pointed at /map_explorable,
the cells the camera has never looked at become frontiers for explore_lite,
forcing the robot to physically visit every region.
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class CameraCoverageTracker(Node):
    def __init__(self):
        super().__init__('camera_coverage_tracker')

        self.declare_parameter('camera_frame', 'spot_0/front_cam_link')
        self.declare_parameter('fallback_frame', 'spot_0/base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('camera_info_topic', '/spot_0/front_cam/camera_info')
        self.declare_parameter('horizontal_fov_deg', 70.0)
        self.declare_parameter('max_range_m', 6.0)
        self.declare_parameter('n_rays', 80)
        self.declare_parameter('update_rate_hz', 5.0)

        self._cam_frame = self.get_parameter('camera_frame').value
        self._fallback_frame = self.get_parameter('fallback_frame').value
        self._map_frame = self.get_parameter('map_frame').value
        self._h_fov = math.radians(self.get_parameter('horizontal_fov_deg').value)
        self._max_range = float(self.get_parameter('max_range_m').value)
        self._n_rays = int(self.get_parameter('n_rays').value)
        rate_hz = float(self.get_parameter('update_rate_hz').value)

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value,
            self._on_map, latched_qos,
        )
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self._on_caminfo, 10,
        )

        self._cov_pub = self.create_publisher(OccupancyGrid, '/camera_coverage', latched_qos)
        self._exp_pub = self.create_publisher(OccupancyGrid, '/map_explorable', latched_qos)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._map: Optional[OccupancyGrid] = None
        self._map_data: Optional[np.ndarray] = None     # int8 (H, W)
        self._coverage: Optional[np.ndarray] = None     # int8 (H, W) — -1 unseen, 0 seen

        self._tick_count = 0
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f'camera_coverage_tracker ready (fov={math.degrees(self._h_fov):.0f}°, '
            f'range={self._max_range}m, rays={self._n_rays}, rate={rate_hz}Hz)'
        )

    # ------------------------------------------------------------------ #
    # Subscriptions

    def _on_caminfo(self, msg: CameraInfo):
        # k[0] is fx; horizontal FOV = 2 * atan(width / (2*fx))
        if msg.width > 0 and len(msg.k) >= 1 and msg.k[0] > 0:
            new_fov = 2.0 * math.atan2(msg.width / 2.0, msg.k[0])
            if abs(new_fov - self._h_fov) > 0.01:
                self.get_logger().info(
                    f'h_fov updated from camera_info: {math.degrees(new_fov):.1f}°'
                )
                self._h_fov = new_fov

    def _on_map(self, msg: OccupancyGrid):
        new_data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )

        if self._map is None or self._coverage is None:
            self._coverage = np.full(new_data.shape, -1, dtype=np.int8)
        else:
            old_info = self._map.info
            new_info = msg.info
            same_geometry = (
                old_info.width == new_info.width
                and old_info.height == new_info.height
                and abs(old_info.resolution - new_info.resolution) < 1e-6
                and abs(old_info.origin.position.x - new_info.origin.position.x) < 1e-6
                and abs(old_info.origin.position.y - new_info.origin.position.y) < 1e-6
            )
            if not same_geometry:
                self._coverage = self._resize_coverage(old_info, new_info, self._coverage)

        self._map = msg
        self._map_data = new_data
        # Publish /map_explorable immediately so Nav2 sees a valid topic.
        self._publish()

    def _resize_coverage(self, old_info, new_info, old_cov: np.ndarray) -> np.ndarray:
        """Copy old coverage cells into a fresh grid sized to the new map."""
        new_cov = np.full((new_info.height, new_info.width), -1, dtype=np.int8)
        res = new_info.resolution
        dx_cells = int(round((old_info.origin.position.x - new_info.origin.position.x) / res))
        dy_cells = int(round((old_info.origin.position.y - new_info.origin.position.y) / res))
        y0_dst = max(0, dy_cells)
        x0_dst = max(0, dx_cells)
        y0_src = max(0, -dy_cells)
        x0_src = max(0, -dx_cells)
        h = min(old_info.height - y0_src, new_info.height - y0_dst)
        w = min(old_info.width - x0_src, new_info.width - x0_dst)
        if h > 0 and w > 0:
            new_cov[y0_dst:y0_dst + h, x0_dst:x0_dst + w] = (
                old_cov[y0_src:y0_src + h, x0_src:x0_src + w]
            )
        return new_cov

    # ------------------------------------------------------------------ #
    # Ray-casting tick

    def _tick(self):
        if self._map is None or self._map_data is None or self._coverage is None:
            return
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._cam_frame, rclpy.time.Time()
            )
        except Exception:
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._map_frame, self._fallback_frame, rclpy.time.Time()
                )
            except Exception:
                return

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = yaw_from_quat(tf.transform.rotation)

        info = self._map.info
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        H, W = self._map_data.shape

        n_steps = max(2, int(self._max_range / res))
        angle_start = yaw - self._h_fov / 2.0
        angle_step = self._h_fov / max(1, self._n_rays - 1)

        for i in range(self._n_rays):
            angle = angle_start + i * angle_step
            dx = math.cos(angle) * res
            dy = math.sin(angle) * res
            cx, cy = x, y
            for _ in range(n_steps):
                cx += dx
                cy += dy
                gx = int((cx - ox) / res)
                gy = int((cy - oy) / res)
                if gx < 0 or gx >= W or gy < 0 or gy >= H:
                    break
                v = self._map_data[gy, gx]
                if v > 50:  # SLAM-known obstacle blocks the ray
                    break
                self._coverage[gy, gx] = 0

        self._tick_count += 1
        if self._tick_count % 50 == 0:
            seen = int(np.count_nonzero(self._coverage == 0))
            self.get_logger().info(f'camera seen cells: {seen}')

        self._publish()

    # ------------------------------------------------------------------ #
    # Publish

    def _publish(self):
        if self._map is None or self._map_data is None or self._coverage is None:
            return

        cov_msg = OccupancyGrid()
        cov_msg.header.frame_id = self._map_frame
        cov_msg.header.stamp = self.get_clock().now().to_msg()
        cov_msg.info = self._map.info
        cov_msg.data = self._coverage.flatten().tolist()
        self._cov_pub.publish(cov_msg)

        # /map_explorable:
        #   obstacle  -> 100 (always preserved so Nav2 sees walls)
        #   free + camera-seen -> 0
        #   otherwise -> -1 (unknown → becomes a frontier for explore_lite)
        obstacle = self._map_data > 50
        free_seen = (self._map_data == 0) & (self._coverage == 0)
        exp = np.full_like(self._map_data, -1)
        exp[free_seen] = 0
        exp[obstacle] = 100

        exp_msg = OccupancyGrid()
        exp_msg.header.frame_id = self._map_frame
        exp_msg.header.stamp = self.get_clock().now().to_msg()
        exp_msg.info = self._map.info
        exp_msg.data = exp.flatten().tolist()
        self._exp_pub.publish(exp_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraCoverageTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
