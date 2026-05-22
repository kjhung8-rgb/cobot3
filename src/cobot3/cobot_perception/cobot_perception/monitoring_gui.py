"""PyQt5 monitoring dashboard for Spot survivor-search.

Layout
======

    +---------------------------+----------------------------+
    |  YOLO annotated camera    |  2D top-down map view      |
    |  (Image)                  |  (SLAM map + camera        |
    |                           |   coverage + robot + goal) |
    +---------------------------+----------------------------+
    |               Progress / status panel                  |
    |  Current zone, waypoint progress, camera coverage %,   |
    |  detected survivor count, list of survivor poses.      |
    +--------------------------------------------------------+

Topics consumed
---------------
    /spot_0/yolo/annotated_image  sensor_msgs/Image
    /map                          nav_msgs/OccupancyGrid (SLAM)
    /camera_coverage              nav_msgs/OccupancyGrid (tracker)
    /coverage_zones               visualization_msgs/MarkerArray
    /coverage_waypoints           visualization_msgs/MarkerArray
    /detected_survivor_pose       geometry_msgs/PoseStamped (one shot per
                                  survivor; we keep a list)
    TF: map -> spot_0/base_link
"""

from __future__ import annotations

import sys
import threading
import time
from dataclasses import dataclass
from typing import List, Optional

import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray

from PyQt5 import QtCore, QtGui, QtWidgets


# ─────────────────────────────────────────────────────────────────── #
# ROS bridge node — runs in its own thread, exposes the latest data
# ─────────────────────────────────────────────────────────────────── #


@dataclass
class Survivor:
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

        # State
        self._image: Optional[np.ndarray] = None
        self._map: Optional[OccupancyGrid] = None
        self._coverage: Optional[OccupancyGrid] = None
        self._robot_xy: Optional[tuple] = None
        self._zones: Optional[MarkerArray] = None
        self._waypoints: Optional[MarkerArray] = None
        self._survivors: List[Survivor] = []

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

        self.create_subscription(Image, '/spot_0/yolo/annotated_image',
                                 self._on_image, sensor)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
        self.create_subscription(OccupancyGrid, '/camera_coverage',
                                 self._on_coverage, latched)
        self.create_subscription(MarkerArray, '/coverage_zones',
                                 self._on_zones, latched)
        self.create_subscription(MarkerArray, '/coverage_waypoints',
                                 self._on_waypoints, latched)
        self.create_subscription(PoseStamped, '/detected_survivor_pose',
                                 self._on_survivor, 10)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self.create_timer(0.2, self._tick_tf)

    # ── callbacks ──

    def _on_image(self, msg: Image):
        try:
            arr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception:
            return
        with self._lock:
            self._image = arr

    def _on_map(self, msg: OccupancyGrid):
        with self._lock:
            self._map = msg

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
        s = Survivor(
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            t_recv=time.time() - self._t0,
        )
        with self._lock:
            # Dedup: if a survivor within 0.5 m already known, skip
            for existing in self._survivors:
                if (existing.x - s.x) ** 2 + (existing.y - s.y) ** 2 < 0.25:
                    return
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
                'image': None if self._image is None else self._image.copy(),
                'map': self._map,
                'coverage': self._coverage,
                'robot_xy': self._robot_xy,
                'zones': self._zones,
                'waypoints': self._waypoints,
                'survivors': list(self._survivors),
            }


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
    def __init__(self):
        super().__init__()
        self.setMinimumSize(480, 360)
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setStyleSheet(
            'background-color: #1f2937; color: #9ca3af;'
            'border: 1px solid #374151;'
        )
        self.setText('No Image\n(Setup Camera in Isaac Sim?)')

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
        self._map: Optional[OccupancyGrid] = None
        self._cov: Optional[OccupancyGrid] = None
        self._robot_xy: Optional[tuple] = None
        self._survivors: List[Survivor] = []

    def update_state(self, snap: dict):
        self._map = snap['map']
        self._cov = snap['coverage']
        self._robot_xy = snap['robot_xy']
        self._survivors = snap['survivors']
        self.update()

    def paintEvent(self, event):  # noqa: N802 (Qt API)
        p = QtGui.QPainter(self)
        p.fillRect(self.rect(), QtGui.QColor('#1f2937'))
        if self._map is None:
            p.setPen(QtGui.QColor('#9ca3af'))
            p.drawText(self.rect(), QtCore.Qt.AlignCenter,
                       'Waiting for /map …')
            return

        # Compute world-to-widget mapping. Show map bbox with margin.
        info = self._map.info
        H, W = info.height, info.width
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        world_w = W * res
        world_h = H * res
        margin = 0.04
        widget_w = self.width() * (1 - 2 * margin)
        widget_h = self.height() * (1 - 2 * margin)
        scale = min(widget_w / world_w, widget_h / world_h)
        offset_x = margin * self.width() + (widget_w - world_w * scale) / 2.0
        offset_y = margin * self.height() + (widget_h - world_h * scale) / 2.0

        def w2px(x, y):
            """world (x, y) → widget pixel (px, py). +y maps to UP."""
            px = offset_x + (x - ox) * scale
            py = offset_y + (world_h - (y - oy)) * scale
            return px, py

        # Draw SLAM map (lightweight: just obstacles as dark pixels)
        m_arr = np.array(self._map.data, dtype=np.int8).reshape((H, W))
        img = QtGui.QImage(W, H, QtGui.QImage.Format_RGB32)
        for v, color in ((-1, 0xff_374151),   # gray (unknown)
                         (0,  0xff_d1d5db),   # light gray (free)
                         (100, 0xff_111827)):  # near black (obstacle)
            pass
        # Vectorize: fill pixel buffer via numpy
        rgb = np.zeros((H, W, 4), dtype=np.uint8)
        rgb[m_arr == -1] = [0x37, 0x41, 0x51, 0xff]   # B G R A — but we encode RGB32
        rgb[m_arr == 0] = [0xd1, 0xd5, 0xdb, 0xff]
        rgb[m_arr == 100] = [0x11, 0x18, 0x27, 0xff]
        # Format_RGB32 expects 0xAARRGGBB packed; build via numpy uint32
        a = np.full((H, W), 0xff, dtype=np.uint32)
        r = np.where(m_arr == -1, 0x37, np.where(m_arr == 100, 0x11, 0xd1)).astype(np.uint32)
        g = np.where(m_arr == -1, 0x41, np.where(m_arr == 100, 0x18, 0xd5)).astype(np.uint32)
        b = np.where(m_arr == -1, 0x51, np.where(m_arr == 100, 0x27, 0xdb)).astype(np.uint32)
        packed = (a << 24) | (r << 16) | (g << 8) | b
        # Map needs to be flipped vertically (image origin top-left, world origin bottom-left)
        packed = np.flipud(packed.astype(np.uint32))
        img = QtGui.QImage(packed.tobytes(), W, H,
                           W * 4, QtGui.QImage.Format_ARGB32)
        # Draw at world bounds
        tl_px, tl_py = w2px(ox, oy + world_h)  # top-left in world (= upper-y)
        target = QtCore.QRectF(tl_px, tl_py, world_w * scale, world_h * scale)
        p.drawImage(target, img)

        # Camera coverage as semi-transparent green overlay
        if self._cov is not None and (
                self._cov.info.width == W and self._cov.info.height == H):
            cov_arr = np.array(self._cov.data, dtype=np.int8).reshape((H, W))
            cov_seen = cov_arr == 0
            if cov_seen.any():
                ovr = np.zeros((H, W, 4), dtype=np.uint8)
                ovr[cov_seen] = [120, 0, 60, 90]  # R, G, B, A — pinkish green tint
                # Actually use a nicer green
                ovr[cov_seen] = [80, 220, 130, 80]
                ovr_img = QtGui.QImage(np.flipud(ovr).tobytes(), W, H,
                                       W * 4, QtGui.QImage.Format_RGBA8888)
                p.drawImage(target, ovr_img)

        # Robot dot
        if self._robot_xy is not None:
            rx, ry = self._robot_xy
            px, py = w2px(rx, ry)
            p.setPen(QtGui.QPen(QtGui.QColor('#fde047'), 2))
            p.setBrush(QtGui.QColor('#facc15'))
            p.drawEllipse(QtCore.QPointF(px, py), 8, 8)

        # Survivors as red X
        p.setPen(QtGui.QPen(QtGui.QColor('#ef4444'), 3))
        for s in self._survivors:
            px, py = w2px(s.x, s.y)
            d = 8
            p.drawLine(int(px - d), int(py - d), int(px + d), int(py + d))
            p.drawLine(int(px - d), int(py + d), int(px + d), int(py - d))


class StatusPanel(QtWidgets.QWidget):
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

        grid = QtWidgets.QGridLayout(self)
        grid.setContentsMargins(12, 12, 12, 12)

        def add(row, col, label_text, value_widget):
            lbl = QtWidgets.QLabel(label_text)
            lbl.setFont(font_label)
            lbl.setStyleSheet('color: #9ca3af;')
            grid.addWidget(lbl, row, col)
            grid.addWidget(value_widget, row + 1, col)

        add(0, 0, '경과 시간', self.elapsed_lbl)
        add(0, 1, '현재 zone', self.zone_lbl)
        add(0, 2, 'Waypoint 진행', self.progress_lbl)
        add(0, 3, '카메라 커버리지', self.coverage_lbl)

        # Survivor section
        surv_box = QtWidgets.QGroupBox('발견된 생존자')
        surv_layout = QtWidgets.QVBoxLayout(surv_box)
        count_row = QtWidgets.QHBoxLayout()
        count_row.addWidget(QtWidgets.QLabel('총 발견:'))
        count_row.addWidget(self.survivor_count_lbl)
        count_row.addStretch()
        surv_layout.addLayout(count_row)
        self.surv_list = QtWidgets.QListWidget()
        self.surv_list.setMaximumHeight(120)
        surv_layout.addWidget(self.surv_list)
        grid.addWidget(surv_box, 2, 0, 1, 4)

    def update_state(self, snap: dict, t_elapsed: float):
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
            cov_arr = np.frombuffer(bytes(cov.data), dtype=np.int8)
            map_arr = np.frombuffer(bytes(m.data), dtype=np.int8)
            if cov_arr.size == map_arr.size:
                free_cells = int(np.count_nonzero(map_arr == 0))
                seen_free = int(np.count_nonzero(
                    (map_arr == 0) & (cov_arr == 0)))
                pct = (100 * seen_free / free_cells) if free_cells else 0
                self.coverage_lbl.setText(f'{pct:.0f}%')

        # Survivors
        survivors = snap['survivors']
        self.survivor_count_lbl.setText(str(len(survivors)))
        existing = self.surv_list.count()
        for i, s in enumerate(survivors):
            if i < existing:
                continue
            mm = int(s.t_recv // 60)
            ss = int(s.t_recv % 60)
            self.surv_list.addItem(
                f'#{i+1}  ({s.x:+.2f}, {s.y:+.2f})   T+{mm:02d}:{ss:02d}'
            )


# ─────────────────────────────────────────────────────────────────── #
# Main window
# ─────────────────────────────────────────────────────────────────── #


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, ros_node: MonitorRosNode, launch_t0: float):
        super().__init__()
        self.setWindowTitle('Spot 생존자 탐색 모니터')
        self.resize(1280, 760)
        self.setStyleSheet('''
            QMainWindow, QWidget { background-color: #111827; color: #e5e7eb; }
            QGroupBox { border: 1px solid #374151; margin-top: 12px;
                        font-weight: bold; padding: 8px; }
            QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }
            QListWidget { background-color: #1f2937; border: 1px solid #374151; }
        ''')

        self._ros = ros_node
        self._t0 = launch_t0

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(8, 8, 8, 8)

        # Top row: image + map
        top_row = QtWidgets.QHBoxLayout()
        image_box = QtWidgets.QGroupBox('YOLO 카메라')
        image_layout = QtWidgets.QVBoxLayout(image_box)
        self.image_panel = ImagePanel()
        image_layout.addWidget(self.image_panel)

        map_box = QtWidgets.QGroupBox('맵 + 로봇 + 생존자')
        map_layout = QtWidgets.QVBoxLayout(map_box)
        self.map_panel = MapPanel()
        map_layout.addWidget(self.map_panel)

        top_row.addWidget(image_box, 1)
        top_row.addWidget(map_box, 1)
        layout.addLayout(top_row, 3)

        # Bottom: status
        status_box = QtWidgets.QGroupBox('상태')
        status_box_layout = QtWidgets.QVBoxLayout(status_box)
        self.status_panel = StatusPanel()
        status_box_layout.addWidget(self.status_panel)
        layout.addWidget(status_box, 1)

        # Periodic update
        timer = QtCore.QTimer(self)
        timer.timeout.connect(self._tick)
        timer.start(200)  # 5 Hz

    def _tick(self):
        snap = self._ros.snapshot()
        self.image_panel.update_image(snap['image'])
        self.map_panel.update_state(snap)
        self.status_panel.update_state(snap, time.time() - self._t0)


def main(args=None):
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
