"""PyQt5 monitoring dashboard for Spot survivor-search.

Layout
======

    +-------------------+-------------------+-------------------+
    |      왼쪽 CAM      |      중앙 CAM      |     오른쪽 CAM      |
    +-------------------+-------------------+-------------------+
    |  2D 상단 뷰 지도                                             |
    |  (SLAM 지도 + 카메라 시야 범위 + 로봇 + 생존자)                  |
    +-----------------------------------------------------------+
    |  제어 패널 (모드 전환, 수동 조작, 복귀)                          |
    +-----------------------------------------------------------+
    |  상태 패널                 생존자 목록 + 캡처 이미지            |
    |  - 경과 시간            |  - 탐지된 생존자 수                  |
    |  - 현재 구역            |  - 생존자 위치 목록                  |
    |  - 웨이포인트 진행률      |  - 생존자 삭제 버튼                  |
    |  - 카메라 커버리지 %      |  - 최신 생존자 이미지                |
    +------------------------+--------------------------------+
이미지 파일
-----------
    최신 생존자 캡처 이미지는 survivor_*.jpg/jpeg/png 파일에서 불러옵니다.
    검색 순서:
        1. $COBOT_SURVIVOR_CAPTURE_DIR
        2. ./src/yolo/dectected_person
        3. ~/dev_ws/cobot3/src/yolo/dectected_person
        4. /home/rokey/dev_ws/cobot3/src/yolo/dectected_person
"""

from __future__ import annotations

import os
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32, String
from visualization_msgs.msg import MarkerArray

from PyQt5 import QtCore, QtGui, QtWidgets


DEFAULT_VIDEO_MAX_WIDTH = 640
DEFAULT_VIDEO_MAX_HEIGHT = 360


def _prefer_pyqt_platform_plugins():
    """Undo cv2's Qt plugin path override before QApplication starts."""
    plugin_path = os.environ.get('QT_QPA_PLATFORM_PLUGIN_PATH', '')
    if 'site-packages/cv2/qt/plugins' in plugin_path:
        os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = QtCore.QLibraryInfo.location(
            QtCore.QLibraryInfo.PluginsPath
        )

    font_path = os.environ.get('QT_QPA_FONTDIR', '')
    if 'site-packages/cv2/qt/fonts' in font_path:
        os.environ.pop('QT_QPA_FONTDIR', None)


# ─────────────────────────────────────────────────────────────────── #
# ROS bridge node — runs in its own thread, exposes the latest data
# ─────────────────────────────────────────────────────────────────── #


@dataclass
class Survivor:
    survivor_id: int
    x: float
    y: float
    t_recv: float  # seconds since launch start


class MonitorRosNode(Node):
    """Subscribes to all the topics we display; thread-safe getters."""

    def __init__(self, launch_t0: float):
        super().__init__('monitoring_gui_ros')

        self._lock = threading.Lock()
        self._bridge = CvBridge()
        self._t0 = launch_t0
        self._video_max_size = self._read_video_max_size()

        # State
        self._images: dict[str, Optional[np.ndarray]] = {
            'left': None,
            'center': None,
            'right': None,
        }
        self._map: Optional[OccupancyGrid] = None
        self._costmap: Optional[OccupancyGrid] = None
        self._coverage: Optional[OccupancyGrid] = None
        self._robot_xy: Optional[tuple] = None
        self._zones: Optional[MarkerArray] = None
        self._waypoints: Optional[MarkerArray] = None
        self._survivors: List[Survivor] = []
        self._next_survivor_id = 1
        self._control_mode = 'autonomous'

        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        image_topics = {
            'left': '/spot_0/yolo/left_annotated_image',
            'center': '/spot_0/yolo/annotated_image',
            'right': '/spot_0/yolo/right_annotated_image',
        }
        self._image_subs = []
        for camera_name, topic in image_topics.items():
            self._image_subs.append(
                self.create_subscription(
                    Image,
                    topic,
                    lambda msg, name=camera_name: self._on_image(name, msg),
                    sensor,
                )
            )
        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self._on_costmap, latched)
        self.create_subscription(OccupancyGrid, '/camera_coverage',
                                 self._on_coverage, latched)
        self.create_subscription(MarkerArray, '/coverage_zones',
                                 self._on_zones, latched)
        self.create_subscription(MarkerArray, '/coverage_waypoints',
                                 self._on_waypoints, latched)
        self.create_subscription(PoseStamped, '/detected_survivor_pose',
                                 self._on_survivor, 10)
        self._survivor_delete_pub = self.create_publisher(
            Int32, '/survivor_delete_id', 10
        )
        self._control_mode_pub = self.create_publisher(
            String, '/control_mode', 10
        )
        self._explore_resume_pub = self.create_publisher(
            Bool, '/explore/resume', 10
        )
        self._return_home_pub = self.create_publisher(
            Bool, '/coverage_planner/return_home', 10
        )
        self._teleop_pub = self.create_publisher(
            Twist, '/teleop_cmd_vel', 10
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self.create_timer(0.2, self._tick_tf)

    # ── callbacks ──

    def _on_image(self, camera_name: str, msg: Image):
        try:
            arr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception:
            return
        arr = self._downsample_image(arr)
        with self._lock:
            self._images[camera_name] = arr

    @staticmethod
    def _read_video_max_size() -> tuple[int, int]:
        width = DEFAULT_VIDEO_MAX_WIDTH
        height = DEFAULT_VIDEO_MAX_HEIGHT
        try:
            width = int(os.environ.get('COBOT_GUI_VIDEO_MAX_WIDTH', width))
            height = int(os.environ.get('COBOT_GUI_VIDEO_MAX_HEIGHT', height))
        except ValueError:
            width = DEFAULT_VIDEO_MAX_WIDTH
            height = DEFAULT_VIDEO_MAX_HEIGHT
        return max(1, width), max(1, height)

    def _downsample_image(self, arr: np.ndarray) -> np.ndarray:
        max_w, max_h = self._video_max_size
        h, w = arr.shape[:2]
        scale = min(max_w / max(w, 1), max_h / max(h, 1), 1.0)
        if scale >= 1.0:
            return arr

        new_w = max(1, int(w * scale))
        new_h = max(1, int(h * scale))
        y_idx = np.linspace(0, h - 1, new_h).astype(np.intp)
        x_idx = np.linspace(0, w - 1, new_w).astype(np.intp)
        return np.ascontiguousarray(arr[y_idx][:, x_idx])

    def _on_map(self, msg: OccupancyGrid):
        with self._lock:
            self._map = msg

    def _on_costmap(self, msg: OccupancyGrid):
        with self._lock:
            self._costmap = msg

    def _on_coverage(self, msg: OccupancyGrid):
        with self._lock:
            self._coverage = msg

    def _on_zones(self, msg: MarkerArray):
        with self._lock:
            self._zones = msg

    def _on_waypoints(self, msg: MarkerArray):
        with self._lock:
            self._waypoints = msg

    def _on_survivor(self, msg: PoseStamped):
        with self._lock:
            # Dedup: if a survivor within 0.5 m already known, skip
            for existing in self._survivors:
                if ((existing.x - msg.pose.position.x) ** 2
                        + (existing.y - msg.pose.position.y) ** 2 < 0.25):
                    return
            s = Survivor(
                survivor_id=self._next_survivor_id,
                x=msg.pose.position.x,
                y=msg.pose.position.y,
                t_recv=time.time() - self._t0,
            )
            self._next_survivor_id += 1
            self._survivors.append(s)

    def _tick_tf(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'spot_0/base_link', rclpy.time.Time()
            )
            with self._lock:
                self._robot_xy = (
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                )
        except Exception:
            pass

    # ── thread-safe snapshot ──

    def snapshot(self):
        with self._lock:
            return {
                'images': {
                    name: None if image is None else image.copy()
                    for name, image in self._images.items()
                },
                'map': self._map,
                'costmap': self._costmap,
                'coverage': self._coverage,
                'robot_xy': self._robot_xy,
                'zones': self._zones,
                'waypoints': self._waypoints,
                'survivors': list(self._survivors),
                'control_mode': self._control_mode,
            }

    def delete_survivor(self, survivor_id: int):
        msg = Int32()
        msg.data = int(survivor_id)
        self._survivor_delete_pub.publish(msg)

        with self._lock:
            if survivor_id == 0:
                self._survivors.clear()
                self._next_survivor_id = 1
            else:
                self._survivors = [
                    s for s in self._survivors if s.survivor_id != survivor_id
                ]

    def set_control_mode(self, mode: str):
        mode = mode.strip().lower()
        if mode not in ('autonomous', 'manual'):
            return

        if mode == 'manual':
            self.publish_teleop_stop()
            self._publish_explore_resume(False)
            self._publish_control_mode('manual')
        else:
            self.publish_teleop_stop()
            self._publish_control_mode('autonomous')
            self._publish_explore_resume(True)

        with self._lock:
            self._control_mode = mode

    def publish_teleop(self, linear_x: float = 0.0, angular_z: float = 0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self._teleop_pub.publish(msg)

    def publish_teleop_stop(self):
        for _ in range(3):
            self.publish_teleop(0.0, 0.0)

    def request_return_home(self):
        msg = Bool()
        msg.data = True
        self._return_home_pub.publish(msg)

    def _publish_control_mode(self, mode: str):
        msg = String()
        msg.data = mode
        self._control_mode_pub.publish(msg)

    def _publish_explore_resume(self, resume: bool):
        msg = Bool()
        msg.data = bool(resume)
        self._explore_resume_pub.publish(msg)


def ros_thread_main(node: MonitorRosNode):
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()


# ─────────────────────────────────────────────────────────────────── #
# Qt widgets
# ─────────────────────────────────────────────────────────────────── #


class ImagePanel(QtWidgets.QLabel):
    def __init__(self, waiting_text='No Image'):
        super().__init__()
        self._waiting_text = waiting_text
        self.setMinimumSize(320, 180)
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setStyleSheet(
            'background-color: #1f2937; color: #9ca3af;'
            'border: 1px solid #374151;'
        )
        self.setText(f'{self._waiting_text}\n(Setup Camera in Isaac Sim?)')

    def update_image(self, arr: Optional[np.ndarray]):
        if arr is None:
            return
        h, w = arr.shape[:2]
        bytes_per_line = arr.strides[0]
        qimg = QtGui.QImage(arr.data, w, h, bytes_per_line,
                            QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap.fromImage(qimg).scaled(
            self.size(),
            QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.SmoothTransformation,
        )
        self.setPixmap(pix)


class MapPanel(QtWidgets.QWidget):
    """2D top-down view: SLAM map + camera coverage + robot + survivors."""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(480, 360)
        self.setStyleSheet('background-color: #1f2937;')
        self.setCursor(QtCore.Qt.OpenHandCursor)
        self._map: Optional[OccupancyGrid] = None
        self._costmap: Optional[OccupancyGrid] = None
        self._cov: Optional[OccupancyGrid] = None
        self._robot_xy: Optional[tuple] = None
        self._zones: Optional[MarkerArray] = None
        self._waypoints: Optional[MarkerArray] = None
        self._survivors: List[Survivor] = []
        self._rotation_deg = 0
        self._zoom = 1.0
        self._pan_px = QtCore.QPointF(0.0, 0.0)
        self._last_drag_pos: Optional[QtCore.QPoint] = None
        self._layers = {
            'map': True,
            'costmap': True,
            'coverage': True,
            'zones': True,
            'waypoints': True,
            'robot': True,
            'survivors': True,
        }
        self._image_cache: dict[str, tuple[tuple, QtGui.QImage]] = {}

    def set_layer_visible(self, layer: str, visible: bool):
        if layer not in self._layers:
            return
        self._layers[layer] = bool(visible)
        self.update()

    def set_rotation_degrees(self, degrees: int):
        self._rotation_deg = int(degrees)
        self.update()

    def rotate_by(self, degrees: int):
        self.set_rotation_degrees(self._rotation_deg + int(degrees))

    def reset_rotation(self):
        self.set_rotation_degrees(0)

    def set_zoom(self, zoom: float):
        self._zoom = min(max(float(zoom), 0.25), 8.0)
        self.update()

    def zoom_by(self, factor: float):
        self.set_zoom(self._zoom * float(factor))

    def reset_view(self):
        self._zoom = 1.0
        self._pan_px = QtCore.QPointF(0.0, 0.0)
        self.update()

    def update_state(self, snap: dict):
        self._map = snap['map']
        self._costmap = snap['costmap']
        self._cov = snap['coverage']
        self._robot_xy = snap['robot_xy']
        self._zones = snap['zones']
        self._waypoints = snap['waypoints']
        self._survivors = snap['survivors']
        self.update()

    def paintEvent(self, event):  # noqa: N802 (Qt API)
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing, True)
        p.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, True)
        p.fillRect(self.rect(), QtGui.QColor('#1f2937'))
        ref_grid = self._reference_grid()
        if ref_grid is None:
            p.setPen(QtGui.QColor('#9ca3af'))
            p.drawText(self.rect(), QtCore.Qt.AlignCenter,
                       'Waiting for /map …')
            return

        transform = self._view_transform(ref_grid)
        p.setTransform(transform)
        self._draw_grid(p, ref_grid)

        if self._layers['map'] and self._map is not None:
            self._draw_grid_image(
                p, self._map, self._cached_grid_image('map', self._map, self._map_image)
            )

        if self._layers['coverage'] and self._cov is not None:
            self._draw_grid_image(
                p, self._cov,
                self._cached_grid_image('coverage', self._cov, self._coverage_image),
            )

        if self._layers['costmap'] and self._costmap is not None:
            self._draw_grid_image(
                p, self._costmap,
                self._cached_grid_image('costmap', self._costmap, self._costmap_image),
            )

        if self._layers['zones'] and self._zones is not None:
            self._draw_zones(p, self._zones)

        if self._layers['waypoints'] and self._waypoints is not None:
            self._draw_waypoints(p, self._waypoints)

        if self._layers['robot'] and self._robot_xy is not None:
            self._draw_robot(p, self._robot_xy)

        if self._layers['survivors']:
            self._draw_survivors(p)

        p.resetTransform()
        self._draw_rotation_label(p)

    def _reference_grid(self) -> Optional[OccupancyGrid]:
        return self._map or self._costmap or self._cov

    def _view_transform(self, grid: OccupancyGrid) -> QtGui.QTransform:
        info = grid.info
        H, W = info.height, info.width
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        world_w = W * res
        world_h = H * res
        center_x = ox + world_w / 2.0
        center_y = oy + world_h / 2.0
        radians = np.deg2rad(self._rotation_deg)
        bbox_w = abs(world_w * np.cos(radians)) + abs(world_h * np.sin(radians))
        bbox_h = abs(world_w * np.sin(radians)) + abs(world_h * np.cos(radians))
        margin = 0.06
        widget_w = self.width() * (1 - 2 * margin)
        widget_h = self.height() * (1 - 2 * margin)
        scale = min(widget_w / max(bbox_w, 1e-6), widget_h / max(bbox_h, 1e-6))

        transform = QtGui.QTransform()
        transform.translate(
            self.width() / 2.0 + self._pan_px.x(),
            self.height() / 2.0 + self._pan_px.y(),
        )
        transform.rotate(-self._rotation_deg)
        transform.scale(scale * self._zoom, -scale * self._zoom)
        transform.translate(-center_x, -center_y)
        return transform

    def wheelEvent(self, event):  # noqa: N802 (Qt API)
        delta = event.angleDelta().y()
        if delta == 0:
            return
        self.zoom_by(1.15 if delta > 0 else 1.0 / 1.15)
        event.accept()

    def mousePressEvent(self, event):  # noqa: N802 (Qt API)
        if event.button() == QtCore.Qt.LeftButton:
            self._last_drag_pos = event.pos()
            self.setCursor(QtCore.Qt.ClosedHandCursor)
            event.accept()

    def mouseMoveEvent(self, event):  # noqa: N802 (Qt API)
        if self._last_drag_pos is None:
            return
        delta = event.pos() - self._last_drag_pos
        self._pan_px += QtCore.QPointF(delta.x(), delta.y())
        self._last_drag_pos = event.pos()
        self.update()
        event.accept()

    def mouseReleaseEvent(self, event):  # noqa: N802 (Qt API)
        if event.button() == QtCore.Qt.LeftButton:
            self._last_drag_pos = None
            self.setCursor(QtCore.Qt.OpenHandCursor)
            event.accept()

    def mouseDoubleClickEvent(self, event):  # noqa: N802 (Qt API)
        if event.button() == QtCore.Qt.LeftButton:
            self.reset_view()
            event.accept()

    @staticmethod
    def _grid_rect(grid: OccupancyGrid) -> QtCore.QRectF:
        info = grid.info
        return QtCore.QRectF(
            info.origin.position.x,
            info.origin.position.y,
            info.width * info.resolution,
            info.height * info.resolution,
        )

    def _draw_grid_image(self, painter: QtGui.QPainter, grid: OccupancyGrid,
                         image: QtGui.QImage):
        if image.isNull():
            return
        painter.drawImage(self._grid_rect(grid), image)

    def _cached_grid_image(self, name: str, grid: OccupancyGrid, factory):
        key = self._grid_cache_key(grid)
        cached = self._image_cache.get(name)
        if cached is not None and cached[0] == key:
            return cached[1]
        image = factory(grid)
        self._image_cache[name] = (key, image)
        return image

    @staticmethod
    def _grid_cache_key(grid: OccupancyGrid) -> tuple:
        info = grid.info
        stamp = grid.header.stamp
        return (
            stamp.sec,
            stamp.nanosec,
            info.width,
            info.height,
            info.resolution,
            info.origin.position.x,
            info.origin.position.y,
            len(grid.data),
        )

    @staticmethod
    def _map_image(grid: OccupancyGrid) -> QtGui.QImage:
        H, W = grid.info.height, grid.info.width
        arr = np.asarray(grid.data, dtype=np.int8).reshape((H, W))
        a = np.full((H, W), 0xff, dtype=np.uint32)
        occupied = arr >= 65
        r = np.where(arr == -1, 0x37, np.where(occupied, 0x11, 0xd1)).astype(np.uint32)
        g = np.where(arr == -1, 0x41, np.where(occupied, 0x18, 0xd5)).astype(np.uint32)
        b = np.where(arr == -1, 0x51, np.where(occupied, 0x27, 0xdb)).astype(np.uint32)
        packed = (a << 24) | (r << 16) | (g << 8) | b
        return QtGui.QImage(
            packed.astype(np.uint32).tobytes(), W, H, W * 4,
            QtGui.QImage.Format_ARGB32
        ).copy()

    @staticmethod
    def _coverage_image(grid: OccupancyGrid) -> QtGui.QImage:
        H, W = grid.info.height, grid.info.width
        arr = np.asarray(grid.data, dtype=np.int8).reshape((H, W))
        rgba = np.zeros((H, W, 4), dtype=np.uint8)
        seen = arr == 0
        rgba[seen] = [80, 220, 130, 80]
        return QtGui.QImage(
            rgba.tobytes(), W, H, W * 4, QtGui.QImage.Format_RGBA8888
        ).copy()

    @staticmethod
    def _costmap_image(grid: OccupancyGrid) -> QtGui.QImage:
        H, W = grid.info.height, grid.info.width
        arr = np.asarray(grid.data, dtype=np.int16).reshape((H, W))
        cost = np.clip(arr, 0, 100).astype(np.float32) / 100.0
        active = arr > 0
        rgba = np.zeros((H, W, 4), dtype=np.uint8)
        rgba[..., 0] = 255
        rgba[..., 1] = np.clip(180 * (1.0 - cost), 20, 180).astype(np.uint8)
        rgba[..., 2] = np.clip(40 * (1.0 - cost), 0, 40).astype(np.uint8)
        rgba[..., 3] = np.where(active, (80 + 120 * cost).astype(np.uint8), 0)
        return QtGui.QImage(
            rgba.tobytes(), W, H, W * 4, QtGui.QImage.Format_RGBA8888
        ).copy()

    def _draw_grid(self, painter: QtGui.QPainter, grid: OccupancyGrid):
        rect = self._grid_rect(grid)
        pen = QtGui.QPen(QtGui.QColor(60, 120, 220, 150))
        pen.setCosmetic(True)
        pen.setWidth(1)
        painter.setPen(pen)
        step = 10.0
        x = np.floor(rect.left() / step) * step
        while x <= rect.right():
            painter.drawLine(QtCore.QPointF(x, rect.top()), QtCore.QPointF(x, rect.bottom()))
            x += step
        y = np.floor(rect.top() / step) * step
        while y <= rect.bottom():
            painter.drawLine(QtCore.QPointF(rect.left(), y), QtCore.QPointF(rect.right(), y))
            y += step

    @staticmethod
    def _marker_color(marker, fallback: QtGui.QColor) -> QtGui.QColor:
        color = marker.color
        if color.a <= 0.0:
            return fallback
        return QtGui.QColor.fromRgbF(
            min(max(color.r, 0.0), 1.0),
            min(max(color.g, 0.0), 1.0),
            min(max(color.b, 0.0), 1.0),
            min(max(color.a, 0.0), 1.0),
        )

    def _draw_zones(self, painter: QtGui.QPainter, markers: MarkerArray):
        for marker in markers.markers:
            if marker.action != 0 or marker.ns != 'coverage_zones':
                continue
            if marker.type == 4 and len(marker.points) >= 2:  # LINE_STRIP
                pen = QtGui.QPen(self._marker_color(marker, QtGui.QColor('#60a5fa')))
                pen.setCosmetic(True)
                pen.setWidth(2)
                painter.setPen(pen)
                points = [QtCore.QPointF(pt.x, pt.y) for pt in marker.points]
                for start, end in zip(points, points[1:]):
                    painter.drawLine(start, end)

    def _draw_waypoints(self, painter: QtGui.QPainter, markers: MarkerArray):
        for marker in markers.markers:
            if marker.action != 0 or marker.ns != 'coverage_waypoints':
                continue
            if marker.type != 2:  # SPHERE
                continue
            color = self._marker_color(marker, QtGui.QColor('#d946ef'))
            pen = QtGui.QPen(color)
            pen.setCosmetic(True)
            pen.setWidth(1)
            painter.setPen(pen)
            painter.setBrush(color)
            radius = max(float(marker.scale.x) / 2.0, 0.15)
            pos = marker.pose.position
            painter.drawEllipse(QtCore.QPointF(pos.x, pos.y), radius, radius)

    def _draw_robot(self, painter: QtGui.QPainter, robot_xy: tuple):
        rx, ry = robot_xy
        pen = QtGui.QPen(QtGui.QColor('#fde047'))
        pen.setCosmetic(True)
        pen.setWidth(2)
        painter.setPen(pen)
        painter.setBrush(QtGui.QColor('#facc15'))
        painter.drawEllipse(QtCore.QPointF(rx, ry), 0.35, 0.35)

    def _draw_survivors(self, painter: QtGui.QPainter):
        world_to_screen = painter.transform()
        painter.save()
        painter.resetTransform()
        for survivor in self._survivors:
            center = world_to_screen.map(QtCore.QPointF(survivor.x, survivor.y))
            size = 24.0
            rect = QtCore.QRectF(
                center.x() - size / 2.0,
                center.y() - size / 2.0,
                size,
                size,
            )
            outline = QtGui.QPen(QtGui.QColor('#991b1b'))
            outline.setWidth(2)
            painter.setPen(outline)
            painter.setBrush(QtGui.QColor(255, 255, 255, 240))
            painter.drawRoundedRect(rect, 3, 3)

            cross_pen = QtGui.QPen(QtGui.QColor('#dc2626'))
            cross_pen.setWidth(5)
            painter.setPen(cross_pen)
            half = size * 0.28
            painter.drawLine(
                QtCore.QPointF(center.x() - half, center.y()),
                QtCore.QPointF(center.x() + half, center.y()),
            )
            painter.drawLine(
                QtCore.QPointF(center.x(), center.y() - half),
                QtCore.QPointF(center.x(), center.y() + half),
            )

            label = f'#{survivor.survivor_id}'
            font = QtGui.QFont('Sans', 9, QtGui.QFont.Bold)
            painter.setFont(font)
            metrics = QtGui.QFontMetrics(font)
            label_rect = metrics.boundingRect(label).adjusted(-5, -3, 5, 3)
            label_rect.moveTopLeft(
                QtCore.QPoint(int(center.x() + size / 2.0 + 5), int(center.y() - size / 2.0))
            )
            painter.setPen(QtGui.QPen(QtGui.QColor('#991b1b')))
            painter.setBrush(QtGui.QColor(255, 255, 255, 235))
            painter.drawRoundedRect(QtCore.QRectF(label_rect), 3, 3)
            painter.drawText(label_rect, QtCore.Qt.AlignCenter, label)
        painter.restore()

    def _draw_rotation_label(self, painter: QtGui.QPainter):
        painter.setPen(QtGui.QColor('#9ca3af'))
        painter.drawText(
            self.rect().adjusted(10, 10, -10, -10),
            QtCore.Qt.AlignTop | QtCore.Qt.AlignRight,
            f'rotation {self._rotation_deg:+d}°  zoom {self._zoom:.2f}x',
        )


class LatestCapturePanel(QtWidgets.QGroupBox):
    def __init__(self):
        super().__init__('최근 생존자 사진')
        self.setMinimumWidth(300)
        self.setMaximumWidth(420)
        self._latest_key = None
        self._pixmap: Optional[QtGui.QPixmap] = None
        self._capture_dirs = self._default_capture_dirs()

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        self.image_lbl = QtWidgets.QLabel('No survivor image')
        self.image_lbl.setMinimumSize(280, 210)
        self.image_lbl.setAlignment(QtCore.Qt.AlignCenter)
        self.image_lbl.setStyleSheet(
            'background-color: #1f2937; color: #9ca3af;'
            'border: 1px solid #374151;'
        )
        self.file_lbl = QtWidgets.QLabel('—')
        self.file_lbl.setStyleSheet('color: #9ca3af;')
        self.file_lbl.setWordWrap(True)

        layout.addWidget(self.image_lbl, 1)
        layout.addWidget(self.file_lbl, 0)

    @staticmethod
    def _default_capture_dirs() -> list[Path]:
        dirs = []
        env_dir = os.environ.get('COBOT_SURVIVOR_CAPTURE_DIR')
        if env_dir:
            dirs.append(Path(env_dir).expanduser())
        dirs.extend([
            Path.cwd() / 'src/yolo/dectected_person',
            Path.home() / 'dev_ws/cobot3/src/yolo/dectected_person',
            Path('/home/rokey/dev_ws/cobot3/src/yolo/dectected_person'),
        ])

        unique = []
        seen = set()
        for path in dirs:
            resolved = path.resolve(strict=False)
            if resolved in seen:
                continue
            seen.add(resolved)
            unique.append(path)
        return unique

    def update_latest(self):
        latest = self._find_latest_capture()
        if latest is None:
            self._pixmap = None
            self._latest_key = None
            self.image_lbl.setPixmap(QtGui.QPixmap())
            self.image_lbl.setText('No survivor image')
            self.file_lbl.setText('—')
            return

        try:
            stat = latest.stat()
        except OSError:
            return

        key = (str(latest), stat.st_mtime_ns, stat.st_size)
        if key == self._latest_key:
            return

        pixmap = QtGui.QPixmap(str(latest))
        if pixmap.isNull():
            self.file_lbl.setText(f'이미지 로드 실패: {latest.name}')
            return

        self._latest_key = key
        self._pixmap = pixmap
        self._apply_pixmap()
        self.file_lbl.setText(latest.name)

    def resizeEvent(self, event):  # noqa: N802 (Qt API)
        super().resizeEvent(event)
        self._apply_pixmap()

    def _apply_pixmap(self):
        if self._pixmap is None or self._pixmap.isNull():
            return
        scaled = self._pixmap.scaled(
            self.image_lbl.size(),
            QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.SmoothTransformation,
        )
        self.image_lbl.setPixmap(scaled)

    def _find_latest_capture(self) -> Optional[Path]:
        candidates = []
        for directory in self._capture_dirs:
            if not directory.is_dir():
                continue
            candidates.extend(directory.glob('survivor_*.jpg'))
            candidates.extend(directory.glob('survivor_*.jpeg'))
            candidates.extend(directory.glob('survivor_*.png'))
        if not candidates:
            return None
        return max(candidates, key=lambda path: path.stat().st_mtime_ns)


class StatusPanel(QtWidgets.QWidget):
    delete_survivor_requested = QtCore.pyqtSignal(int)

    def __init__(self):
        super().__init__()
        font_label = QtGui.QFont('Sans', 9)
        font_value = QtGui.QFont('Sans', 14, QtGui.QFont.Bold)

        self.elapsed_lbl = QtWidgets.QLabel('00:00')
        self.elapsed_lbl.setFont(font_value)
        self.zone_lbl = QtWidgets.QLabel('—')
        self.zone_lbl.setFont(font_value)
        self.progress_lbl = QtWidgets.QLabel('0 / 0 waypoints')
        self.progress_lbl.setFont(font_value)
        self.coverage_lbl = QtWidgets.QLabel('0%')
        self.coverage_lbl.setFont(font_value)
        self.survivor_count_lbl = QtWidgets.QLabel('0')
        self.survivor_count_lbl.setFont(QtGui.QFont('Sans', 22, QtGui.QFont.Bold))
        self.survivor_count_lbl.setStyleSheet('color: #ef4444;')

        root = QtWidgets.QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(10)

        status_box = QtWidgets.QGroupBox('상태')
        status_box.setMaximumWidth(330)
        status_layout = QtWidgets.QVBoxLayout(status_box)
        status_layout.setContentsMargins(12, 12, 12, 12)
        status_layout.setSpacing(8)

        def add_metric(label_text, value_widget):
            row = QtWidgets.QWidget()
            row_layout = QtWidgets.QVBoxLayout(row)
            row_layout.setContentsMargins(0, 0, 0, 0)
            row_layout.setSpacing(2)
            lbl = QtWidgets.QLabel(label_text)
            lbl.setFont(font_label)
            lbl.setStyleSheet('color: #9ca3af;')
            value_widget.setMinimumHeight(28)
            row_layout.addWidget(lbl)
            row_layout.addWidget(value_widget)
            status_layout.addWidget(row)

        add_metric('경과 시간', self.elapsed_lbl)
        add_metric('현재 zone', self.zone_lbl)
        add_metric('Waypoint 진행', self.progress_lbl)
        add_metric('전체맵 탐색률', self.coverage_lbl)
        status_layout.addStretch()

        # Survivor section
        surv_box = QtWidgets.QGroupBox('발견된 생존자')
        surv_layout = QtWidgets.QVBoxLayout(surv_box)
        surv_layout.setContentsMargins(12, 12, 12, 12)
        surv_layout.setSpacing(8)
        count_row = QtWidgets.QHBoxLayout()
        count_row.addWidget(QtWidgets.QLabel('총 발견:'))
        count_row.addWidget(self.survivor_count_lbl)
        count_row.addStretch()
        surv_layout.addLayout(count_row)
        self.surv_list = QtWidgets.QListWidget()
        self.surv_list.setMinimumHeight(210)
        self.surv_list.setUniformItemSizes(True)
        self.surv_list.setVerticalScrollMode(QtWidgets.QAbstractItemView.ScrollPerPixel)
        surv_layout.addWidget(self.surv_list)
        self._survivor_rows_key = ()
        self.capture_panel = LatestCapturePanel()

        root.addWidget(status_box, 0)
        root.addWidget(surv_box, 2)
        root.addWidget(self.capture_panel, 1)

    def _make_survivor_row(self, survivor: Survivor) -> QtWidgets.QWidget:
        row = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(row)
        layout.setContentsMargins(8, 2, 6, 2)
        layout.setSpacing(8)

        mm = int(survivor.t_recv // 60)
        ss = int(survivor.t_recv % 60)
        label = QtWidgets.QLabel(
            f'#{survivor.survivor_id}  '
            f'({survivor.x:+.2f}, {survivor.y:+.2f})   T+{mm:02d}:{ss:02d}'
        )
        label.setFont(QtGui.QFont('Sans', 11))
        delete_btn = QtWidgets.QToolButton()
        delete_btn.setText('X')
        delete_btn.setFixedSize(28, 28)
        delete_btn.setToolTip(f'생존자 #{survivor.survivor_id} 삭제')
        delete_btn.setStyleSheet(
            'QToolButton { color: #fecaca; background-color: #7f1d1d; '
            'border: 1px solid #ef4444; border-radius: 4px; font-weight: bold; }'
            'QToolButton:hover { background-color: #991b1b; }'
        )
        delete_btn.clicked.connect(
            lambda checked=False, sid=survivor.survivor_id:
            self.delete_survivor_requested.emit(sid)
        )

        layout.addWidget(label, 1)
        layout.addWidget(delete_btn, 0)
        return row

    def update_state(self, snap: dict, t_elapsed: float):
        self.capture_panel.update_latest()

        # elapsed
        mm = int(t_elapsed // 60)
        ss = int(t_elapsed % 60)
        self.elapsed_lbl.setText(f'{mm:02d}:{ss:02d}')

        # Waypoint progress from /coverage_waypoints markers
        wps = snap['waypoints']
        if wps is not None:
            visited = 0
            total = 0
            for m in wps.markers:
                if m.action == 0 and m.ns == 'coverage_waypoints':
                    total += 1
                    # gray marker == visited (gray = (0.5, 0.5, 0.5))
                    if (abs(m.color.r - 0.5) < 0.05
                            and abs(m.color.g - 0.5) < 0.05
                            and abs(m.color.b - 0.5) < 0.05):
                        visited += 1
            pct = (100 * visited / total) if total else 0
            self.progress_lbl.setText(f'{visited} / {total}  ({pct:.0f}%)')
        else:
            self.progress_lbl.setText('—')

        # Zone: find the green-colored zone (current)
        zones = snap['zones']
        if zones is not None:
            cur = None
            for m in zones.markers:
                if (m.action == 0 and m.ns == 'coverage_zones'
                        and m.type == 9  # TEXT_VIEW_FACING
                        and abs(m.color.g - 1.0) < 0.05
                        and m.color.r < 0.5):
                    cur = m.text.split('\n')[0]  # first line: "(x, y)"
                    break
            self.zone_lbl.setText(cur if cur else '—')
        else:
            self.zone_lbl.setText('—')

        # Camera coverage %
        cov = snap['coverage']
        m = snap['map']
        if cov is not None and m is not None:
            cov_arr = np.asarray(cov.data, dtype=np.int8)
            map_arr = np.asarray(m.data, dtype=np.int8)
            if cov_arr.size == map_arr.size:
                free_cells = int(np.count_nonzero(map_arr == 0))
                seen_free = int(np.count_nonzero(
                    (map_arr == 0) & (cov_arr == 0)))
                pct = (100 * seen_free / free_cells) if free_cells else 0
                self.coverage_lbl.setText(f'{pct:.0f}%')

        # Survivors
        survivors = snap['survivors']
        self.survivor_count_lbl.setText(str(len(survivors)))
        rows_key = tuple(
            (s.survivor_id, round(s.x, 3), round(s.y, 3), int(s.t_recv))
            for s in survivors
        )
        if rows_key == self._survivor_rows_key:
            return
        self._survivor_rows_key = rows_key
        self.surv_list.clear()
        for s in survivors:
            item = QtWidgets.QListWidgetItem()
            item.setData(QtCore.Qt.UserRole, s.survivor_id)
            item.setSizeHint(QtCore.QSize(0, 36))
            self.surv_list.addItem(item)
            self.surv_list.setItemWidget(item, self._make_survivor_row(s))


# ─────────────────────────────────────────────────────────────────── #
# Main window
# ─────────────────────────────────────────────────────────────────── #


class MainWindow(QtWidgets.QMainWindow):
    TELEOP_DEFAULT_LINEAR_SPEED = 0.5
    TELEOP_DEFAULT_ANGULAR_SPEED = 1.0
    TELEOP_MIN_SPEED = 0.1
    TELEOP_MAX_LINEAR_SPEED = 2.0
    TELEOP_MAX_ANGULAR_SPEED = 2.5
    TELEOP_SPEED_STEP = 0.1

    def __init__(self, ros_node: MonitorRosNode, launch_t0: float):
        super().__init__()
        self.setWindowTitle('Spot 생존자 탐색 모니터')
        self.resize(1440, 1000)
        self.setStyleSheet('''
            QMainWindow, QWidget { background-color: #111827; color: #e5e7eb; }
            QGroupBox { border: 1px solid #374151; margin-top: 12px;
                        font-weight: bold; padding: 8px; }
            QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }
            QListWidget { background-color: #1f2937; border: 1px solid #374151; }
            QListWidget::item { border-bottom: 1px solid #374151; }
        ''')

        self._ros = ros_node
        self._t0 = launch_t0
        self._control_mode = 'autonomous'
        self._pressed_keys: set[int] = set()
        self._teleop_linear_speed = self.TELEOP_DEFAULT_LINEAR_SPEED
        self._teleop_angular_speed = self.TELEOP_DEFAULT_ANGULAR_SPEED
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        app = QtWidgets.QApplication.instance()
        if app is not None:
            app.installEventFilter(self)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # Top row: left / center / right YOLO camera images
        camera_row = QtWidgets.QHBoxLayout()
        camera_row.setSpacing(8)
        self.image_panels: dict[str, ImagePanel] = {}
        for key, title in (
                ('left', '왼쪽 cam'),
                ('center', '중앙 cam'),
                ('right', '오른쪽 cam')):
            image_box = QtWidgets.QGroupBox(title)
            image_layout = QtWidgets.QVBoxLayout(image_box)
            image_layout.setContentsMargins(8, 10, 8, 8)
            panel = ImagePanel(title)
            self.image_panels[key] = panel
            image_layout.addWidget(panel)
            camera_row.addWidget(image_box, 1)
        layout.addLayout(camera_row, 2)

        map_box = QtWidgets.QGroupBox('맵 + 로봇 + 생존자')
        map_layout = QtWidgets.QVBoxLayout(map_box)
        self.map_panel = MapPanel()
        map_controls = QtWidgets.QHBoxLayout()
        map_controls.setSpacing(8)

        for label, layer in (
                ('SLAM', 'map'),
                ('Cost', 'costmap'),
                ('Coverage', 'coverage'),
                ('Zones', 'zones'),
                ('Waypoints', 'waypoints'),
                ('Robot', 'robot'),
                ('Survivor', 'survivors')):
            cb = QtWidgets.QCheckBox(label)
            cb.setChecked(True)
            cb.toggled.connect(
                lambda checked, layer_name=layer:
                self.map_panel.set_layer_visible(layer_name, checked)
            )
            map_controls.addWidget(cb)

        map_controls.addStretch()
        left_btn = QtWidgets.QToolButton()
        left_btn.setText('⟲')
        left_btn.setToolTip('맵 왼쪽으로 15도 회전')
        right_btn = QtWidgets.QToolButton()
        right_btn.setText('⟳')
        right_btn.setToolTip('맵 오른쪽으로 15도 회전')
        reset_btn = QtWidgets.QToolButton()
        reset_btn.setText('0')
        reset_btn.setToolTip('맵 회전 초기화')
        zoom_out_btn = QtWidgets.QToolButton()
        zoom_out_btn.setText('-')
        zoom_out_btn.setToolTip('맵 축소')
        zoom_in_btn = QtWidgets.QToolButton()
        zoom_in_btn.setText('+')
        zoom_in_btn.setToolTip('맵 확대')
        fit_btn = QtWidgets.QToolButton()
        fit_btn.setText('Fit')
        fit_btn.setToolTip('맵 이동/확대 초기화')
        self.map_rotation_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.map_rotation_slider.setRange(-180, 180)
        self.map_rotation_slider.setValue(0)
        self.map_rotation_slider.setFixedWidth(160)
        left_btn.clicked.connect(lambda: self._adjust_map_rotation(-15))
        right_btn.clicked.connect(lambda: self._adjust_map_rotation(15))
        reset_btn.clicked.connect(lambda: self.map_rotation_slider.setValue(0))
        zoom_out_btn.clicked.connect(lambda: self.map_panel.zoom_by(1.0 / 1.2))
        zoom_in_btn.clicked.connect(lambda: self.map_panel.zoom_by(1.2))
        fit_btn.clicked.connect(self.map_panel.reset_view)
        self.map_rotation_slider.valueChanged.connect(
            self.map_panel.set_rotation_degrees
        )
        map_controls.addWidget(left_btn)
        map_controls.addWidget(self.map_rotation_slider)
        map_controls.addWidget(right_btn)
        map_controls.addWidget(reset_btn)
        map_controls.addWidget(zoom_out_btn)
        map_controls.addWidget(zoom_in_btn)
        map_controls.addWidget(fit_btn)

        map_layout.addLayout(map_controls)
        map_layout.addWidget(self.map_panel)
        layout.addWidget(map_box, 4)

        mode_box = QtWidgets.QGroupBox('주행 모드')
        mode_layout = QtWidgets.QHBoxLayout(mode_box)
        mode_layout.setContentsMargins(12, 10, 12, 10)
        mode_layout.setSpacing(10)
        self.auto_btn = QtWidgets.QPushButton('자율탐사')
        self.manual_btn = QtWidgets.QPushButton('수동조작')
        self.return_home_btn = QtWidgets.QPushButton('강제 복귀')
        for btn in (self.auto_btn, self.manual_btn, self.return_home_btn):
            btn.setCheckable(True)
            btn.setMinimumHeight(34)
        self.return_home_btn.setCheckable(False)
        self.return_home_btn.setToolTip('탐색을 중단하고 시작 위치로 복귀')
        self.auto_btn.setChecked(True)
        self.mode_status_lbl = QtWidgets.QLabel('자율탐사 모드')
        self.mode_status_lbl.setStyleSheet('color: #9ca3af;')
        self.teleop_speed_lbl = QtWidgets.QLabel(self._teleop_speed_text())
        self.teleop_speed_lbl.setStyleSheet('color: #9ca3af;')
        self.auto_btn.clicked.connect(self._set_autonomous_mode)
        self.manual_btn.clicked.connect(self._set_manual_mode)
        self.return_home_btn.clicked.connect(self._request_return_home)
        mode_layout.addWidget(self.auto_btn)
        mode_layout.addWidget(self.manual_btn)
        mode_layout.addWidget(self.return_home_btn)
        mode_layout.addSpacing(12)
        mode_layout.addWidget(self.mode_status_lbl)
        mode_layout.addStretch()
        mode_layout.addWidget(self.teleop_speed_lbl)
        layout.addWidget(mode_box, 0)

        # Bottom: left status metrics + center survivor list
        self.status_panel = StatusPanel()
        layout.addWidget(self.status_panel, 2)
        self.status_panel.delete_survivor_requested.connect(self._delete_survivor)

        # Periodic update
        timer = QtCore.QTimer(self)
        timer.timeout.connect(self._tick)
        timer.start(200)  # 5 Hz

        self._teleop_timer = QtCore.QTimer(self)
        self._teleop_timer.timeout.connect(self._publish_current_teleop)
        self._teleop_timer.start(100)  # 10 Hz

    def _tick(self):
        snap = self._ros.snapshot()
        images = snap['images']
        for name, panel in self.image_panels.items():
            panel.update_image(images.get(name))
        self.map_panel.update_state(snap)
        self.status_panel.update_state(snap, time.time() - self._t0)

    def _delete_survivor(self, survivor_id: int):
        self._ros.delete_survivor(survivor_id)

    def _adjust_map_rotation(self, delta: int):
        value = self.map_rotation_slider.value() + int(delta)
        if value > 180:
            value -= 360
        elif value < -180:
            value += 360
        self.map_rotation_slider.setValue(value)

    def eventFilter(self, obj, event):  # noqa: N802 (Qt API)
        if self._control_mode != 'manual':
            return super().eventFilter(obj, event)

        event_type = event.type()
        if event_type not in (QtCore.QEvent.KeyPress, QtCore.QEvent.KeyRelease):
            return super().eventFilter(obj, event)

        key = event.key()
        if not self._is_teleop_key(key):
            return super().eventFilter(obj, event)

        if event_type == QtCore.QEvent.KeyPress:
            if key == QtCore.Qt.Key_Q and not event.isAutoRepeat():
                self._adjust_teleop_speed(+self.TELEOP_SPEED_STEP)
            elif key == QtCore.Qt.Key_Z and not event.isAutoRepeat():
                self._adjust_teleop_speed(-self.TELEOP_SPEED_STEP)
            elif key == QtCore.Qt.Key_Space:
                self._pressed_keys.clear()
                self._ros.publish_teleop_stop()
            elif self._is_motion_key(key):
                self._pressed_keys.add(key)
                self._publish_current_teleop()
            return True

        if event_type == QtCore.QEvent.KeyRelease:
            if event.isAutoRepeat():
                return True
            if self._is_motion_key(key):
                self._pressed_keys.discard(key)
                self._publish_current_teleop()
            return True

        return super().eventFilter(obj, event)

    def closeEvent(self, event):  # noqa: N802 (Qt API)
        self._pressed_keys.clear()
        self._ros.publish_teleop_stop()
        super().closeEvent(event)

    def _set_autonomous_mode(self, checked=False):
        self._control_mode = 'autonomous'
        self._pressed_keys.clear()
        self._ros.set_control_mode('autonomous')
        self._reset_teleop_speed()
        self.auto_btn.setChecked(True)
        self.manual_btn.setChecked(False)
        self.mode_status_lbl.setText('자율탐사 모드')
        self.setFocus()

    def _set_manual_mode(self, checked=False):
        self._control_mode = 'manual'
        self._pressed_keys.clear()
        self._ros.set_control_mode('manual')
        self.auto_btn.setChecked(False)
        self.manual_btn.setChecked(True)
        self.mode_status_lbl.setText('수동조작 모드')
        self.setFocus()

    def _request_return_home(self, checked=False):
        self._control_mode = 'autonomous'
        self._pressed_keys.clear()
        self._ros.set_control_mode('autonomous')
        self._ros.request_return_home()
        self._reset_teleop_speed()
        self.auto_btn.setChecked(True)
        self.manual_btn.setChecked(False)
        self.mode_status_lbl.setText('원점 복귀 요청')
        self.setFocus()

    def _teleop_speed_text(self) -> str:
        return (
            f'WASD/방향키  Q/Z 속도 +/-  선속도 {self._teleop_linear_speed:.1f} m/s  '
            f'각속도 {self._teleop_angular_speed:.1f} rad/s'
        )

    @staticmethod
    def _is_motion_key(key: int) -> bool:
        return key in (
            QtCore.Qt.Key_W,
            QtCore.Qt.Key_S,
            QtCore.Qt.Key_A,
            QtCore.Qt.Key_D,
            QtCore.Qt.Key_Up,
            QtCore.Qt.Key_Down,
            QtCore.Qt.Key_Left,
            QtCore.Qt.Key_Right,
        )

    def _is_teleop_key(self, key: int) -> bool:
        return self._is_motion_key(key) or key in (
            QtCore.Qt.Key_Q,
            QtCore.Qt.Key_Z,
            QtCore.Qt.Key_Space,
        )

    def _adjust_teleop_speed(self, delta: float):
        self._teleop_linear_speed = min(
            max(self._teleop_linear_speed + delta, self.TELEOP_MIN_SPEED),
            self.TELEOP_MAX_LINEAR_SPEED,
        )
        self._teleop_angular_speed = min(
            max(self._teleop_angular_speed + delta, self.TELEOP_MIN_SPEED),
            self.TELEOP_MAX_ANGULAR_SPEED,
        )
        self.teleop_speed_lbl.setText(self._teleop_speed_text())
        self._publish_current_teleop()

    def _reset_teleop_speed(self):
        self._teleop_linear_speed = self.TELEOP_DEFAULT_LINEAR_SPEED
        self._teleop_angular_speed = self.TELEOP_DEFAULT_ANGULAR_SPEED
        self.teleop_speed_lbl.setText(self._teleop_speed_text())

    def _publish_current_teleop(self):
        if self._control_mode != 'manual':
            return

        linear = 0.0
        angular = 0.0
        if QtCore.Qt.Key_W in self._pressed_keys or QtCore.Qt.Key_Up in self._pressed_keys:
            linear += self._teleop_linear_speed
        if QtCore.Qt.Key_S in self._pressed_keys or QtCore.Qt.Key_Down in self._pressed_keys:
            linear -= self._teleop_linear_speed
        if QtCore.Qt.Key_A in self._pressed_keys or QtCore.Qt.Key_Left in self._pressed_keys:
            angular += self._teleop_angular_speed
        if QtCore.Qt.Key_D in self._pressed_keys or QtCore.Qt.Key_Right in self._pressed_keys:
            angular -= self._teleop_angular_speed

        self._ros.publish_teleop(linear, angular)


def main(args=None):
    _prefer_pyqt_platform_plugins()
    rclpy.init(args=args)
    t0 = time.time()
    ros_node = MonitorRosNode(t0)

    ros_thread = threading.Thread(
        target=ros_thread_main, args=(ros_node,), daemon=True
    )
    ros_thread.start()

    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow(ros_node, t0)
    win.show()
    try:
        app.exec_()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
