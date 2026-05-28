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
        3. <project root>/src/yolo/dectected_person
"""

from __future__ import annotations

import ctypes
import json
import math
import os
import re
import shutil
import subprocess
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
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, Int32, String
from visualization_msgs.msg import MarkerArray

from PyQt5 import QtCore, QtGui, QtWidgets


DEFAULT_VIDEO_MAX_WIDTH = 640
DEFAULT_VIDEO_MAX_HEIGHT = 360
ANNOTATED_IMAGE_STALE_SEC = 1.0
MAP_UPDATE_PERIOD_SEC = 0.5
STATUS_UPDATE_PERIOD_SEC = 1.0


def _prefer_pyqt_platform_plugins():
    """Undo cv2's Qt plugin path override before QApplication starts."""
    if os.environ.get('DISPLAY') and not os.environ.get('QT_QPA_PLATFORM'):
        os.environ['QT_QPA_PLATFORM'] = 'xcb'

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
        # /map 첫 수신 시각. None이면 아직 SLAM 시작 전.
        self._map_first_t: Optional[float] = None
        self._video_max_size = self._read_video_max_size()

        # State
        self._images: dict[str, Optional[np.ndarray]] = {
            'left': None,
            'center': None,
            'right': None,
        }
        self._last_annotated_image_time: dict[str, float] = {
            'left': 0.0,
            'center': 0.0,
            'right': 0.0,
        }
        self._map: Optional[OccupancyGrid] = None
        self._costmap: Optional[OccupancyGrid] = None
        self._coverage: Optional[OccupancyGrid] = None
        self._scan_points: list[tuple[float, float]] = []
        self._plan_points: list[tuple[float, float]] = []
        self._robot_xy: Optional[tuple] = None
        self._jackal_xy: Optional[tuple] = None
        self._zones: Optional[MarkerArray] = None
        self._waypoints: Optional[MarkerArray] = None
        self._survivors: List[Survivor] = []
        self._robot_velocity: Optional[tuple[float, float, float]] = None
        self._next_survivor_id = 1
        self._control_mode = 'autonomous'
        # 키보드 teleop이 어느 로봇을 조작할지. 'spot' 또는 'jackal'.
        self._teleop_target = 'spot'
        # GUI 재시작 시 생존자 좌표 + map_first_t 복원 위해 disk persistence.
        self._survivors_path = Path.home() / '.ros' / 'cobot3_survivors.json'
        self._load_survivors()

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

        annotated_image_topics = {
            'left': '/spot_0/yolo/left_annotated_image',
            'center': '/spot_0/yolo/annotated_image',
            'right': '/spot_0/yolo/right_annotated_image',
        }
        raw_image_topics = {
            'left': '/spot_0/left_cam/color_image',
            'center': '/spot_0/front_cam/color_image',
            'right': '/spot_0/right_cam/color_image',
        }
        self._image_subs = []
        for camera_name, topic in annotated_image_topics.items():
            self._image_subs.append(
                self.create_subscription(
                    Image,
                    topic,
                    lambda msg, name=camera_name: self._on_image(
                        name, msg, annotated=True
                    ),
                    sensor,
                )
            )
        for camera_name, topic in raw_image_topics.items():
            self._image_subs.append(
                self.create_subscription(
                    Image,
                    topic,
                    lambda msg, name=camera_name: self._on_image(
                        name, msg, annotated=False
                    ),
                    sensor,
                )
            )
        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self._on_costmap, latched)
        self.create_subscription(OccupancyGrid, '/camera_coverage',
                                 self._on_coverage, latched)
        self.create_subscription(LaserScan, '/spot_0/scan', self._on_scan, sensor)
        self.create_subscription(NavPath, '/plan', self._on_plan, 10)
        self.create_subscription(MarkerArray, '/coverage_zones',
                                 self._on_zones, latched)
        self.create_subscription(MarkerArray, '/coverage_waypoints',
                                 self._on_waypoints, latched)
        self.create_subscription(PoseStamped, '/detected_survivor_pose',
                                 self._on_survivor, 10)
        self.create_subscription(Odometry, '/spot_0/odom', self._on_odom, 10)
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
        # mission_manager에 jackal RESCUE 좌표 (manual click).
        self._jackal_manual_goal_pub = self.create_publisher(
            PoseStamped, '/jackal_0/manual_goal', 10
        )
        # mission_manager에 jackal 홈 복귀 트리거.
        self._jackal_return_home_pub = self.create_publisher(
            Bool, '/jackal_0/return_home', 10
        )
        # mission_manager에 home 중단 + 큐 재개 트리거.
        self._jackal_resume_pub = self.create_publisher(
            Bool, '/jackal_0/mission_resume', 10
        )
        # jackal keyboard teleop — jackal cmd_vel_relay가 manual 모드일 때
        # /jackal_0/cmd_vel로 forward.
        self._jackal_teleop_pub = self.create_publisher(
            Twist, '/jackal_0/teleop_cmd_vel', 10
        )
        self._jackal_control_mode_pub = self.create_publisher(
            String, '/jackal_0/control_mode', 10
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self.create_timer(0.2, self._tick_tf)

    # ── callbacks ──

    def _on_image(self, camera_name: str, msg: Image, annotated: bool):
        now = time.monotonic()
        if not annotated:
            with self._lock:
                annotated_age = (
                    now - self._last_annotated_image_time.get(camera_name, 0.0)
                )
            if annotated_age < ANNOTATED_IMAGE_STALE_SEC:
                return

        try:
            arr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception:
            return
        arr = self._downsample_image(arr)
        now = time.monotonic()
        with self._lock:
            if annotated:
                self._last_annotated_image_time[camera_name] = now
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
            if self._map_first_t is None:
                self._map_first_t = time.time()
                self._save_survivors_locked()

    def _on_costmap(self, msg: OccupancyGrid):
        with self._lock:
            self._costmap = msg

    def _on_coverage(self, msg: OccupancyGrid):
        with self._lock:
            self._coverage = msg

    def _on_scan(self, msg: LaserScan):
        points = self._scan_to_map_points(msg)
        with self._lock:
            self._scan_points = points

    def _on_plan(self, msg: NavPath):
        points = [(pose.pose.position.x, pose.pose.position.y)
                  for pose in msg.poses]
        with self._lock:
            self._plan_points = points

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
            self._save_survivors_locked()

    def _load_survivors(self):
        """GUI 재시작 시 disk에서 survivor 목록 + 세션 메타 복원."""
        try:
            if not self._survivors_path.exists():
                return
            with open(self._survivors_path) as f:
                data = json.load(f)
            with self._lock:
                self._survivors = [
                    Survivor(
                        survivor_id=int(d['survivor_id']),
                        x=float(d['x']),
                        y=float(d['y']),
                        t_recv=float(d.get('t_recv', 0.0)),
                    )
                    for d in data.get('survivors', [])
                ]
                if self._survivors:
                    self._next_survivor_id = max(
                        s.survivor_id for s in self._survivors
                    ) + 1
                saved_t = data.get('meta', {}).get('map_first_t')
                if saved_t is not None:
                    self._map_first_t = float(saved_t)
        except Exception as exc:
            print(f'[monitoring_gui] survivor load 실패: {exc}', file=sys.stderr)

    def _save_survivors_locked(self):
        """주의: caller가 self._lock 잡고 있어야 함."""
        try:
            self._survivors_path.parent.mkdir(parents=True, exist_ok=True)
            data = {
                'meta': {'map_first_t': self._map_first_t},
                'survivors': [
                    {'survivor_id': s.survivor_id, 'x': s.x, 'y': s.y, 't_recv': s.t_recv}
                    for s in self._survivors
                ],
            }
            tmp = self._survivors_path.with_suffix('.json.tmp')
            with open(tmp, 'w') as f:
                json.dump(data, f, indent=2)
            os.replace(tmp, self._survivors_path)
        except Exception as exc:
            print(f'[monitoring_gui] survivor save 실패: {exc}', file=sys.stderr)

    def _on_odom(self, msg: Odometry):
        linear = msg.twist.twist.linear.x
        angular = msg.twist.twist.angular.z
        with self._lock:
            self._robot_velocity = (linear, angular, time.time())

    def _on_odom(self, msg: Odometry):
        linear = msg.twist.twist.linear.x
        angular = msg.twist.twist.angular.z
        with self._lock:
            self._robot_velocity = (linear, angular, time.time())

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
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'jackal_0/base_link', rclpy.time.Time()
            )
            with self._lock:
                self._jackal_xy = (
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                )
        except Exception:
            pass

    def _scan_to_map_points(self, msg: LaserScan) -> list[tuple[float, float]]:
        frame_id = msg.header.frame_id or 'spot_0/lidar_link'
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', frame_id, rclpy.time.Time()
            )
        except Exception:
            return []

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        points: list[tuple[float, float]] = []
        angle = msg.angle_min
        # Cap work per scan; display density stays high enough for the GUI.
        step = max(1, len(msg.ranges) // 1800)
        for i, rng in enumerate(msg.ranges):
            if i % step:
                angle += msg.angle_increment
                continue
            if math.isfinite(rng) and msg.range_min <= rng <= msg.range_max:
                lx = rng * math.cos(angle)
                ly = rng * math.sin(angle)
                points.append((
                    t.x + cos_yaw * lx - sin_yaw * ly,
                    t.y + sin_yaw * lx + cos_yaw * ly,
                ))
            angle += msg.angle_increment
        return points

    def _scan_to_map_points(self, msg: LaserScan) -> list[tuple[float, float]]:
        frame_id = msg.header.frame_id or 'spot_0/lidar_link'
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', frame_id, rclpy.time.Time()
            )
        except Exception:
            return []

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        points: list[tuple[float, float]] = []
        angle = msg.angle_min
        # Cap work per scan; display density stays high enough for the GUI.
        step = max(1, len(msg.ranges) // 1800)
        for i, rng in enumerate(msg.ranges):
            if i % step:
                angle += msg.angle_increment
                continue
            if math.isfinite(rng) and msg.range_min <= rng <= msg.range_max:
                lx = rng * math.cos(angle)
                ly = rng * math.sin(angle)
                points.append((
                    t.x + cos_yaw * lx - sin_yaw * ly,
                    t.y + sin_yaw * lx + cos_yaw * ly,
                ))
            angle += msg.angle_increment
        return points

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
                'scan_points': list(self._scan_points),
                'plan_points': list(self._plan_points),
                'robot_xy': self._robot_xy,
                'jackal_xy': self._jackal_xy,
                'zones': self._zones,
                'waypoints': self._waypoints,
                'survivors': list(self._survivors),
                'robot_velocity': self._robot_velocity,
                'control_mode': self._control_mode,
                'map_first_t': self._map_first_t,
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
            self._save_survivors_locked()

    def set_control_mode(self, mode: str):
        mode = mode.strip().lower()
        if mode not in ('autonomous', 'manual'):
            return
        with self._lock:
            self._control_mode = mode
        self._apply_control_mode()

    def set_teleop_target(self, target: str):
        target = target.strip().lower()
        if target not in ('spot', 'jackal'):
            return
        with self._lock:
            self._teleop_target = target
        self._apply_control_mode()

    def _apply_control_mode(self):
        with self._lock:
            mode = self._control_mode
            target = self._teleop_target
        # 모드/타겟 전환 시 안전상 양쪽 다 stop.
        self.publish_teleop_stop()
        if mode == 'autonomous':
            self._publish_control_mode('autonomous')
            self._publish_jackal_control_mode('autonomous')
            self._publish_explore_resume(True)
            return
        # manual mode
        if target == 'spot':
            self._publish_control_mode('manual')
            self._publish_jackal_control_mode('autonomous')
            self._publish_explore_resume(False)
        else:  # jackal
            # spot은 계속 자율탐사, jackal만 키보드 제어.
            self._publish_control_mode('autonomous')
            self._publish_jackal_control_mode('manual')
            self._publish_explore_resume(True)

    def publish_teleop(self, linear_x: float = 0.0, angular_z: float = 0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        with self._lock:
            target = self._teleop_target
        if target == 'jackal':
            self._jackal_teleop_pub.publish(msg)
        else:
            self._teleop_pub.publish(msg)

    def publish_teleop_stop(self):
        # 안전 — 양쪽 다 0 publish (타겟 전환 직후 잔여 동작 차단).
        stop = Twist()
        for _ in range(3):
            self._teleop_pub.publish(stop)
            self._jackal_teleop_pub.publish(stop)

    def request_return_home(self):
        msg = Bool()
        msg.data = True
        self._return_home_pub.publish(msg)

    def publish_jackal_manual_goal(self, x: float, y: float, frame_id: str = 'map'):
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.w = 1.0
        self._jackal_manual_goal_pub.publish(msg)

    def request_jackal_home(self):
        msg = Bool()
        msg.data = True
        self._jackal_return_home_pub.publish(msg)

    def request_jackal_resume(self):
        msg = Bool()
        msg.data = True
        self._jackal_resume_pub.publish(msg)

    def _publish_control_mode(self, mode: str):
        msg = String()
        msg.data = mode
        self._control_mode_pub.publish(msg)

    def _publish_jackal_control_mode(self, mode: str):
        msg = String()
        msg.data = mode
        self._jackal_control_mode_pub.publish(msg)

    def _publish_explore_resume(self, resume: bool):
        msg = Bool()
        msg.data = bool(resume)
        self._explore_resume_pub.publish(msg)


def ros_thread_main(node: MonitorRosNode):
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except ExternalShutdownException:
        pass
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
        # 파노라마 stitching 위해 border 제거 + margin 0. 옆 panel과 픽셀
        # 단위로 붙음. 크기는 SizePolicy로 부모 layout이 균등 분배.
        self.setStyleSheet('background-color: #1f2937; color: #9ca3af; border: none;')
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                           QtWidgets.QSizePolicy.Expanding)
        self.setScaledContents(False)
        self.setText(f'{self._waiting_text}\n(Setup Camera in Isaac Sim?)')

    def update_image(self, arr: Optional[np.ndarray]):
        if arr is None:
            return
        h, w = arr.shape[:2]
        bytes_per_line = arr.strides[0]
        qimg = QtGui.QImage(arr.data, w, h, bytes_per_line,
                            QtGui.QImage.Format_RGB888)
        # IgnoreAspectRatio: 카메라 16:9 (640x360) → panel 가로/세로 전체
        # 채움. 인접 panel과 동일 높이라 자연스럽게 이어붙는 파노라마.
        pix = QtGui.QPixmap.fromImage(qimg).scaled(
            self.size(),
            QtCore.Qt.IgnoreAspectRatio,
            QtCore.Qt.SmoothTransformation,
        )
        self.setPixmap(pix)


class EmbeddedRvizPanel(QtWidgets.QWidget):
    """Host an RViz2 render window inside the dashboard."""

    embed_failed = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setAttribute(QtCore.Qt.WA_NativeWindow, True)
        self.setMinimumSize(480, 360)
        self.setStyleSheet('background-color: #303030;')
        self._process: Optional[subprocess.Popen] = None
        self._rviz_window_id: Optional[int] = None
        self._x11_display = None
        self._x11 = None
        self._start_attempted = False
        self._poll_count = 0

        self._layout = QtWidgets.QVBoxLayout(self)
        self._layout.setContentsMargins(0, 0, 0, 0)
        self._layout.setSpacing(0)

        self._status_lbl = QtWidgets.QLabel('Starting embedded RViz...')
        self._status_lbl.setAlignment(QtCore.Qt.AlignCenter)
        self._status_lbl.setStyleSheet('color: #cbd5e1; background-color: #303030;')
        self._layout.addWidget(self._status_lbl, 1)

        self._poll_timer = QtCore.QTimer(self)
        self._poll_timer.timeout.connect(self._try_embed_window)

    def showEvent(self, event):  # noqa: N802 (Qt API)
        super().showEvent(event)
        if not self._start_attempted:
            self._start_attempted = True
            QtCore.QTimer.singleShot(0, self._start_rviz)

    def _start_rviz(self):
        rviz_bin = shutil.which('rviz2')
        if rviz_bin is None:
            self._fail('rviz2 executable not found')
            return

        rviz_cfg = self._rviz_config_path()
        if rviz_cfg is None:
            self._fail('spot_explore.rviz config not found')
            return

        env = os.environ.copy()
        if not env.get('QT_QPA_PLATFORM'):
            env['QT_QPA_PLATFORM'] = 'xcb'

        cmd = [
            rviz_bin,
            '-d',
            str(rviz_cfg),
            '--ros-args',
            '-r',
            '__node:=monitoring_gui_embedded_rviz',
        ]
        try:
            self._process = subprocess.Popen(
                cmd,
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except OSError as exc:
            self._fail(f'failed to start rviz2: {exc}')
            return

        self._poll_count = 0
        self._poll_timer.start(250)

    @staticmethod
    def _rviz_config_path() -> Optional[Path]:
        env_path = os.environ.get('COBOT_GUI_RVIZ_CONFIG')
        if env_path:
            path = Path(env_path).expanduser()
            if path.is_file():
                return path

        try:
            from ament_index_python.packages import get_package_share_directory

            share_dir = Path(get_package_share_directory('cobot_perception'))
            path = share_dir / 'rviz' / 'spot_explore.rviz'
            if path.is_file():
                return path
        except Exception:
            pass

        source_path = Path(__file__).resolve().parents[1] / 'rviz' / 'spot_explore.rviz'
        if source_path.is_file():
            return source_path
        return None

    def _try_embed_window(self):
        if self._process is None:
            self._fail('rviz process missing')
            return
        if self._process.poll() is not None:
            self._fail('rviz2 exited before its window was embedded')
            return

        window_id = self._find_window_id(self._process.pid)
        if window_id is None:
            self._poll_count += 1
            if self._poll_count > 60:
                self._fail('could not find rviz2 X11 window')
            return

        self._poll_timer.stop()
        try:
            self._reparent_x11_window(window_id)
        except RuntimeError as exc:
            self._fail(str(exc))
            return

        self._layout.removeWidget(self._status_lbl)
        self._status_lbl.hide()
        self._status_lbl.deleteLater()
        self._rviz_window_id = window_id
        self._resize_embedded_window()

    def _find_window_id(self, pid: int) -> Optional[int]:
        window_id = self._find_window_id_with_wmctrl(pid)
        if window_id is not None:
            return window_id
        window_id = self._find_window_id_with_xdotool(pid)
        if window_id is not None:
            return window_id
        return self._find_window_id_with_xprop(pid)

    @staticmethod
    def _find_window_id_with_wmctrl(pid: int) -> Optional[int]:
        if shutil.which('wmctrl') is None:
            return None
        try:
            out = subprocess.check_output(
                ['wmctrl', '-lp'],
                text=True,
                timeout=1.0,
                stderr=subprocess.DEVNULL,
            )
        except (OSError, subprocess.SubprocessError):
            return None

        for line in out.splitlines():
            parts = line.split(None, 4)
            if len(parts) < 3:
                continue
            try:
                if int(parts[2]) == pid:
                    return int(parts[0], 16)
            except ValueError:
                continue
        return None

    @staticmethod
    def _find_window_id_with_xdotool(pid: int) -> Optional[int]:
        if shutil.which('xdotool') is None:
            return None
        try:
            out = subprocess.check_output(
                ['xdotool', 'search', '--pid', str(pid)],
                text=True,
                timeout=1.0,
                stderr=subprocess.DEVNULL,
            )
        except (OSError, subprocess.SubprocessError):
            return None

        for line in reversed(out.splitlines()):
            try:
                return int(line.strip(), 0)
            except ValueError:
                continue
        return None

    @staticmethod
    def _find_window_id_with_xprop(pid: int) -> Optional[int]:
        if shutil.which('xwininfo') is None or shutil.which('xprop') is None:
            return None
        try:
            out = subprocess.check_output(
                ['xwininfo', '-root', '-children'],
                text=True,
                timeout=1.0,
                stderr=subprocess.DEVNULL,
            )
        except (OSError, subprocess.SubprocessError):
            return None

        for match in re.finditer(r'\b(0x[0-9a-fA-F]+)\b', out):
            window_hex = match.group(1)
            try:
                prop = subprocess.check_output(
                    ['xprop', '-id', window_hex, '_NET_WM_PID'],
                    text=True,
                    timeout=0.2,
                    stderr=subprocess.DEVNULL,
                )
            except (OSError, subprocess.SubprocessError):
                continue
            if f'= {pid}' in prop:
                return int(window_hex, 16)
        return None

    def _reparent_x11_window(self, window_id: int):
        if self._x11 is None:
            try:
                self._x11 = ctypes.cdll.LoadLibrary('libX11.so.6')
            except OSError as exc:
                raise RuntimeError(f'failed to load libX11: {exc}') from exc

            self._x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
            self._x11.XOpenDisplay.restype = ctypes.c_void_p
            self._x11.XReparentWindow.argtypes = [
                ctypes.c_void_p,
                ctypes.c_ulong,
                ctypes.c_ulong,
                ctypes.c_int,
                ctypes.c_int,
            ]
            self._x11.XMapRaised.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
            self._x11.XMoveResizeWindow.argtypes = [
                ctypes.c_void_p,
                ctypes.c_ulong,
                ctypes.c_int,
                ctypes.c_int,
                ctypes.c_uint,
                ctypes.c_uint,
            ]
            self._x11.XFlush.argtypes = [ctypes.c_void_p]
            self._x11.XCloseDisplay.argtypes = [ctypes.c_void_p]

        if self._x11_display is None:
            self._x11_display = self._x11.XOpenDisplay(None)
        if not self._x11_display:
            raise RuntimeError('failed to open X11 display')

        parent_id = int(self.winId())
        result = self._x11.XReparentWindow(
            self._x11_display,
            ctypes.c_ulong(window_id),
            ctypes.c_ulong(parent_id),
            0,
            0,
        )
        if result == 0:
            raise RuntimeError('XReparentWindow failed')
        self._x11.XMapRaised(self._x11_display, ctypes.c_ulong(window_id))
        self._x11.XFlush(self._x11_display)

    def _resize_embedded_window(self):
        if self._x11 is None or self._x11_display is None or self._rviz_window_id is None:
            return
        width = max(1, self.width())
        height = max(1, self.height())
        self._x11.XMoveResizeWindow(
            self._x11_display,
            ctypes.c_ulong(self._rviz_window_id),
            0,
            0,
            ctypes.c_uint(width),
            ctypes.c_uint(height),
        )
        self._x11.XFlush(self._x11_display)

    def resizeEvent(self, event):  # noqa: N802 (Qt API)
        super().resizeEvent(event)
        self._resize_embedded_window()

    def _fail(self, reason: str):
        self._poll_timer.stop()
        self._status_lbl.setText(f'Embedded RViz unavailable\n{reason}')
        self.stop()
        self.embed_failed.emit(reason)

    def stop(self):
        if self._process is None:
            return
        if self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=1.5)
            except subprocess.TimeoutExpired:
                self._process.kill()
        self._process = None
        self._rviz_window_id = None
        if self._x11 is not None and self._x11_display is not None:
            self._x11.XCloseDisplay(self._x11_display)
        self._x11_display = None

    def closeEvent(self, event):  # noqa: N802 (Qt API)
        self.stop()
        super().closeEvent(event)


class MapPanel(QtWidgets.QWidget):
    """2D top-down view: SLAM map + camera coverage + robot + survivors."""

    # 우클릭 → jackal manual goal (map frame, meters).
    goalClicked = QtCore.pyqtSignal(float, float)

    def __init__(self):
        super().__init__()
        self.setMinimumSize(480, 360)
        self.setStyleSheet('background-color: #1f2937;')
        self.setCursor(QtCore.Qt.OpenHandCursor)
        self._map: Optional[OccupancyGrid] = None
        self._costmap: Optional[OccupancyGrid] = None
        self._cov: Optional[OccupancyGrid] = None
        self._scan_points: list[tuple[float, float]] = []
        self._plan_points: list[tuple[float, float]] = []
        self._robot_xy: Optional[tuple] = None
        self._jackal_xy: Optional[tuple] = None
        self._zones: Optional[MarkerArray] = None
        self._waypoints: Optional[MarkerArray] = None
        self._survivors: List[Survivor] = []
        self._robot_velocity: Optional[tuple[float, float, float]] = None
        self._rotation_deg = 0
        self._zoom = 1.0
        self._pan_px = QtCore.QPointF(0.0, 0.0)
        self._last_drag_pos: Optional[QtCore.QPoint] = None
        # 사용자 우클릭으로 찍은 jackal manual goal 좌표 (map frame, m).
        # None이면 표시 안 함. 새 클릭 시 덮어씀, Jackal 복귀 시 clear.
        self._jackal_goal_pin: Optional[tuple[float, float]] = None
        self._layers = {
            'map': True,
            'costmap': True,
            'coverage': True,
            'scan': True,
            'plan': True,
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

    def set_jackal_goal_pin(self, x: float, y: float):
        self._jackal_goal_pin = (float(x), float(y))
        self.update()

    def clear_jackal_goal_pin(self):
        if self._jackal_goal_pin is not None:
            self._jackal_goal_pin = None
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
        self._scan_points = snap['scan_points']
        self._plan_points = snap['plan_points']
        self._robot_xy = snap['robot_xy']
        self._jackal_xy = snap.get('jackal_xy')
        self._zones = snap['zones']
        self._waypoints = snap['waypoints']
        self._survivors = snap['survivors']
        self._robot_velocity = snap['robot_velocity']
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

        if self._layers['scan']:
            self._draw_scan(p)

        if self._layers['plan']:
            self._draw_plan(p)

        if self._layers['zones'] and self._zones is not None:
            self._draw_zones(p, self._zones)

        if self._layers['waypoints'] and self._waypoints is not None:
            self._draw_waypoints(p, self._waypoints)

        if self._layers['robot']:
            if self._robot_xy is not None:
                self._draw_robot(p, self._robot_xy, color='#facc15', outline='#fde047', label='S')
            if self._jackal_xy is not None:
                self._draw_robot(p, self._jackal_xy, color='#60a5fa', outline='#bfdbfe', label='J')

        if self._layers['survivors']:
            self._draw_survivors(p)

        if self._jackal_goal_pin is not None:
            self._draw_jackal_pin(p, self._jackal_goal_pin)

        p.resetTransform()
        self._draw_rotation_label(p)
        self._draw_velocity_label(p)

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

    def _pixel_to_world(self, pixel: QtCore.QPoint) -> Optional[tuple[float, float]]:
        ref_grid = self._reference_grid()
        if ref_grid is None:
            return None
        transform = self._view_transform(ref_grid)
        inv, ok = transform.inverted()
        if not ok:
            return None
        world_pt = inv.map(QtCore.QPointF(pixel))
        return float(world_pt.x()), float(world_pt.y())

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
            return
        if event.button() == QtCore.Qt.RightButton:
            world = self._pixel_to_world(event.pos())
            if world is not None:
                self.goalClicked.emit(world[0], world[1])
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
        a = np.where(arr == -1, 70, 180).astype(np.uint32)
        occupied = arr >= 65
        r = np.where(arr == -1, 0x80, np.where(occupied, 0x20, 0xee)).astype(np.uint32)
        g = np.where(arr == -1, 0x8a, np.where(occupied, 0x20, 0xee)).astype(np.uint32)
        b = np.where(arr == -1, 0x90, np.where(occupied, 0x20, 0xee)).astype(np.uint32)
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
        rgba[seen] = [110, 205, 255, 150]
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
        rgba[..., 0] = np.where(cost > 0.75, 255, 210).astype(np.uint8)
        rgba[..., 1] = np.clip(150 * (1.0 - cost), 40, 150).astype(np.uint8)
        rgba[..., 2] = np.clip(170 * (1.0 - cost), 70, 170).astype(np.uint8)
        rgba[..., 3] = np.where(active, (60 + 80 * cost).astype(np.uint8), 0)
        return QtGui.QImage(
            rgba.tobytes(), W, H, W * 4, QtGui.QImage.Format_RGBA8888
        ).copy()

    def _draw_grid(self, painter: QtGui.QPainter, grid: OccupancyGrid):
        rect = self._grid_rect(grid)
        pen = QtGui.QPen(QtGui.QColor(160, 160, 164, 120))
        pen.setCosmetic(True)
        pen.setWidth(1)
        painter.setPen(pen)
        step = 1.0
        x = np.floor(rect.left() / step) * step
        while x <= rect.right():
            painter.drawLine(QtCore.QPointF(x, rect.top()), QtCore.QPointF(x, rect.bottom()))
            x += step
        y = np.floor(rect.top() / step) * step
        while y <= rect.bottom():
            painter.drawLine(QtCore.QPointF(rect.left(), y), QtCore.QPointF(rect.right(), y))
            y += step

    def _draw_scan(self, painter: QtGui.QPainter):
        if not self._scan_points:
            return
        pen = QtGui.QPen(QtGui.QColor(255, 255, 255, 235))
        pen.setCosmetic(True)
        pen.setWidth(3)
        painter.setPen(pen)
        for x, y in self._scan_points:
            painter.drawPoint(QtCore.QPointF(x, y))

    def _draw_plan(self, painter: QtGui.QPainter):
        if len(self._plan_points) < 2:
            return
        pen = QtGui.QPen(QtGui.QColor(25, 255, 0, 240))
        pen.setCosmetic(True)
        pen.setWidth(2)
        painter.setPen(pen)
        points = [QtCore.QPointF(x, y) for x, y in self._plan_points]
        for start, end in zip(points, points[1:]):
            painter.drawLine(start, end)

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
                color = self._marker_color(marker, QtGui.QColor('#60a5fa'))
                is_current_zone = marker.color.g > 0.9 and marker.color.r < 0.5
                pen = QtGui.QPen(color)
                pen.setCosmetic(True)
                pen.setWidth(8 if is_current_zone else 2)
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

    def _draw_robot(self, painter: QtGui.QPainter, robot_xy: tuple,
                    color: str = '#facc15', outline: str = '#fde047',
                    label: str = ''):
        rx, ry = robot_xy
        pen = QtGui.QPen(QtGui.QColor(outline))
        pen.setCosmetic(True)
        pen.setWidth(2)
        painter.setPen(pen)
        painter.setBrush(QtGui.QColor(color))
        painter.drawEllipse(QtCore.QPointF(rx, ry), 0.35, 0.35)
        if label:
            # 라벨은 screen space로 그려 — world rotation/scale 영향 안 받게.
            world_to_screen = painter.transform()
            screen_pt = world_to_screen.map(QtCore.QPointF(rx, ry))
            painter.save()
            painter.resetTransform()
            painter.setPen(QtGui.QColor('#1f2937'))
            painter.setFont(QtGui.QFont('Sans', 9, QtGui.QFont.Bold))
            painter.drawText(
                QtCore.QRectF(screen_pt.x() - 10, screen_pt.y() - 10, 20, 20),
                QtCore.Qt.AlignCenter,
                label,
            )
            painter.restore()

    def _draw_jackal_pin(self, painter: QtGui.QPainter, world_xy: tuple[float, float]):
        """우클릭 좌표 핀 — 머리 원 + 줄기 형태 (지도용 marker pin)."""
        world_to_screen = painter.transform()
        painter.save()
        painter.resetTransform()
        center = world_to_screen.map(QtCore.QPointF(world_xy[0], world_xy[1]))
        # 줄기 (위쪽에서 좌표점으로 떨어지는 선)
        head_r = 8.0
        stem_h = 18.0
        head_center = QtCore.QPointF(center.x(), center.y() - stem_h)
        # 그림자
        shadow_pen = QtGui.QPen(QtGui.QColor(0, 0, 0, 120))
        shadow_pen.setWidth(4)
        painter.setPen(shadow_pen)
        painter.drawLine(head_center, center)
        # 본체 줄기
        stem_pen = QtGui.QPen(QtGui.QColor('#fbbf24'))
        stem_pen.setWidth(3)
        painter.setPen(stem_pen)
        painter.drawLine(head_center, center)
        # 점 (좌표점)
        painter.setBrush(QtGui.QColor('#fbbf24'))
        painter.setPen(QtGui.QPen(QtGui.QColor('#78350f'), 2))
        painter.drawEllipse(center, 3.0, 3.0)
        # 머리 원
        painter.setBrush(QtGui.QColor('#fbbf24'))
        painter.setPen(QtGui.QPen(QtGui.QColor('#78350f'), 2))
        painter.drawEllipse(head_center, head_r, head_r)
        # 'J' 라벨
        painter.setPen(QtGui.QColor('#1f2937'))
        painter.setFont(QtGui.QFont('Sans', 9, QtGui.QFont.Bold))
        painter.drawText(
            QtCore.QRectF(head_center.x() - head_r, head_center.y() - head_r,
                          head_r * 2, head_r * 2),
            QtCore.Qt.AlignCenter,
            'J',
        )
        painter.restore()

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

    def _draw_velocity_label(self, painter: QtGui.QPainter):
        if self._robot_velocity is None:
            text = '선속도 -- m/s   각속도 -- rad/s'
        else:
            linear, angular, t_recv = self._robot_velocity
            age = time.time() - t_recv
            if age > 1.5:
                text = '선속도 -- m/s   각속도 -- rad/s'
            else:
                text = f'선속도 {linear:+.2f} m/s   각속도 {angular:+.2f} rad/s'

        font = QtGui.QFont('Sans', 10, QtGui.QFont.Bold)
        painter.setFont(font)
        metrics = QtGui.QFontMetrics(font)
        text_rect = metrics.boundingRect(text).adjusted(-8, -5, 8, 5)
        text_rect.moveBottomLeft(QtCore.QPoint(12, self.height() - 12))

        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(17, 24, 39, 210))
        painter.drawRoundedRect(QtCore.QRectF(text_rect), 4, 4)
        painter.setPen(QtGui.QColor('#e5e7eb'))
        painter.drawText(text_rect, QtCore.Qt.AlignCenter, text)


class LatestCapturePanel(QtWidgets.QGroupBox):
    def __init__(self):
        super().__init__('최근 생존자 사진')
        # maxWidth 제거 — 부모 layout에 맞춰 자연스럽게 확장. minWidth만 유지.
        self.setMinimumWidth(300)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                           QtWidgets.QSizePolicy.Expanding)
        self._latest_key = None
        self._pixmap: Optional[QtGui.QPixmap] = None
        self._capture_dirs = self._default_capture_dirs()

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        self.image_lbl = QtWidgets.QLabel('No survivor image')
        self.image_lbl.setMinimumSize(280, 210)
        self.image_lbl.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                                     QtWidgets.QSizePolicy.Expanding)
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
        project_root = Path(__file__).resolve().parents[5]
        dirs.extend([
            Path.cwd() / 'src/yolo/dectected_person',
            project_root / 'src/yolo/dectected_person',
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

        # StatusPanel은 더 이상 내부 레이아웃 가지지 않음.
        # 3개 박스(status_box, surv_box, capture_panel)를 attribute로 노출 →
        # MainWindow가 자유롭게 배치. 자기 자신은 hidden parent role만 함.
        self.status_box = QtWidgets.QGroupBox('상태')
        self.status_box.setMaximumWidth(330)
        status_layout = QtWidgets.QVBoxLayout(self.status_box)
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
        self.surv_box = QtWidgets.QGroupBox('발견된 생존자')
        surv_layout = QtWidgets.QVBoxLayout(self.surv_box)
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
        # MainWindow가 status_box / surv_box / capture_panel을 own layout에 추가.
        # StatusPanel 자체는 빈 QWidget으로 남아 — signal source + 데이터 holder.

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

    def update_state(self, snap: dict, t_elapsed: Optional[float]):
        self.capture_panel.update_latest()

        # elapsed (SLAM map 시작 이후)
        if t_elapsed is None:
            self.elapsed_lbl.setText('--:--')
        else:
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
        self.resize(1600, 1300)
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

        # Top row: 카메라 3개를 파노라마처럼 연결. spacing=0 + 박스 테두리/패딩
        # 제거 → 옆 panel과 시각적으로 이어지게.
        camera_row = QtWidgets.QHBoxLayout()
        camera_row.setSpacing(0)
        camera_row.setContentsMargins(0, 0, 0, 0)
        self.image_panels: dict[str, ImagePanel] = {}
        for key, title in (
                ('left', '왼쪽 cam'),
                ('center', '중앙 cam'),
                ('right', '오른쪽 cam')):
            panel = ImagePanel(title)
            panel.setStyleSheet('border: none;')
            self.image_panels[key] = panel
            camera_row.addWidget(panel, 1)
        layout.addLayout(camera_row, 2)

        map_box = QtWidgets.QGroupBox('맵 + 로봇 + 생존자')
        map_layout = QtWidgets.QVBoxLayout(map_box)
        self._map_layout = map_layout
        self.map_panel = MapPanel()
        self._rviz_panel: Optional[EmbeddedRvizPanel] = None
        self._map_widget: QtWidgets.QWidget = self.map_panel
        if self._use_embedded_rviz():
            self._rviz_panel = EmbeddedRvizPanel()
            self._rviz_panel.embed_failed.connect(self._fallback_to_qt_map)
            self._map_widget = self._rviz_panel

        map_controls = QtWidgets.QHBoxLayout()
        map_controls.setSpacing(8)

        for label, layer in (
                ('SLAM', 'map'),
                ('Cost', 'costmap'),
                ('Coverage', 'coverage'),
                ('Scan', 'scan'),
                ('Plan', 'plan'),
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
        map_layout.addWidget(self._map_widget)
        # map_box는 아래 bottom_row에서 status_panel과 가로 배치.

        mode_box = QtWidgets.QGroupBox('주행 모드')
        mode_layout = QtWidgets.QHBoxLayout(mode_box)
        mode_layout.setContentsMargins(12, 10, 12, 10)
        mode_layout.setSpacing(10)
        self.auto_btn = QtWidgets.QPushButton('자율탐사')
        self.manual_btn = QtWidgets.QPushButton('Spot 수동조작')
        self.jackal_manual_btn = QtWidgets.QPushButton('Jackal 수동조작')
        self.jackal_auto_btn = QtWidgets.QPushButton('Jackal 자동')
        self.return_home_btn = QtWidgets.QPushButton('Spot 복귀')
        self.jackal_home_btn = QtWidgets.QPushButton('Jackal 복귀')
        mode_group = QtWidgets.QButtonGroup(self)
        mode_group.setExclusive(True)
        for btn in (self.auto_btn, self.manual_btn, self.jackal_manual_btn):
            btn.setCheckable(True)
            btn.setMinimumHeight(34)
            mode_group.addButton(btn)
        for btn in (self.jackal_auto_btn, self.return_home_btn, self.jackal_home_btn):
            btn.setCheckable(False)
            btn.setMinimumHeight(34)
        self.return_home_btn.setToolTip('Spot 탐색 중단 후 시작 위치로 복귀')
        self.jackal_home_btn.setToolTip('Jackal을 시작 위치로 복귀')
        self.manual_btn.setToolTip('키보드로 Spot 직접 조작 (Jackal은 자율 유지)')
        self.jackal_manual_btn.setToolTip('키보드로 Jackal 직접 조작 (Spot은 자율탐사 유지)')
        self.jackal_auto_btn.setToolTip('Jackal만 자동 모드 복귀 (Spot 상태 유지)')
        self.auto_btn.setChecked(True)
        self.mode_status_lbl = QtWidgets.QLabel('자율탐사 모드')
        self.mode_status_lbl.setStyleSheet('color: #9ca3af;')
        self.teleop_speed_lbl = QtWidgets.QLabel(self._teleop_speed_text())
        self.teleop_speed_lbl.setStyleSheet('color: #9ca3af;')
        self.auto_btn.clicked.connect(self._set_autonomous_mode)
        self.manual_btn.clicked.connect(self._set_spot_manual_mode)
        self.jackal_manual_btn.clicked.connect(self._set_jackal_manual_mode)
        self.jackal_auto_btn.clicked.connect(self._set_jackal_auto_mode)
        self.return_home_btn.clicked.connect(self._request_return_home)
        self.jackal_home_btn.clicked.connect(self._request_jackal_home)
        # 우클릭 → jackal manual goal dispatch.
        self.map_panel.goalClicked.connect(self._on_map_goal_clicked)
        mode_layout.addWidget(self.auto_btn)
        mode_layout.addWidget(self.manual_btn)
        mode_layout.addWidget(self.jackal_manual_btn)
        mode_layout.addWidget(self.jackal_auto_btn)
        mode_layout.addWidget(self.return_home_btn)
        mode_layout.addWidget(self.jackal_home_btn)
        mode_layout.addSpacing(12)
        mode_layout.addWidget(self.mode_status_lbl)
        mode_layout.addStretch()
        mode_layout.addWidget(self.teleop_speed_lbl)
        layout.addWidget(mode_box, 0)

        # Bottom: 상태(좌) | 맵(중앙) | 생존자목록+사진(우, 세로 스택)
        self.status_panel = StatusPanel()
        self.status_panel.delete_survivor_requested.connect(self._delete_survivor)
        bottom_row = QtWidgets.QHBoxLayout()
        bottom_row.setSpacing(8)
        bottom_row.addWidget(self.status_panel.status_box, 0)
        bottom_row.addWidget(map_box, 3)
        right_col = QtWidgets.QVBoxLayout()
        right_col.setSpacing(8)
        right_col.addWidget(self.status_panel.surv_box, 2)
        right_col.addWidget(self.status_panel.capture_panel, 1)
        bottom_row.addLayout(right_col, 2)
        layout.addLayout(bottom_row, 6)

        # Periodic update
        self._last_map_update = 0.0
        self._last_status_update = 0.0
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

        now = time.monotonic()
        if (self._map_widget is self.map_panel
                and now - self._last_map_update >= MAP_UPDATE_PERIOD_SEC):
            self.map_panel.update_state(snap)
            self._last_map_update = now
        if now - self._last_status_update >= STATUS_UPDATE_PERIOD_SEC:
            # SLAM /map 첫 메시지 받기 전엔 elapsed = None → '--:--' 표시.
            map_first_t = snap.get('map_first_t')
            t_elapsed = (time.time() - map_first_t) if map_first_t else None
            self.status_panel.update_state(snap, t_elapsed)
            self._last_status_update = now

    @staticmethod
    def _use_embedded_rviz() -> bool:
        backend = os.environ.get('COBOT_GUI_MAP_BACKEND', 'qt').strip().lower()
        return backend in ('rviz', 'embedded_rviz', 'embedded-rviz')

    def _fallback_to_qt_map(self, reason: str):
        if self._map_widget is self.map_panel:
            return
        old_widget = self._map_widget
        index = self._map_layout.indexOf(old_widget)
        if index < 0:
            index = self._map_layout.count()
        self._map_layout.removeWidget(old_widget)
        old_widget.setParent(None)
        old_widget.deleteLater()
        self._map_layout.insertWidget(index, self.map_panel)
        self._map_widget = self.map_panel
        self._rviz_panel = None
        self.statusBar().showMessage(
            f'RViz 임베드 실패: {reason}. 기존 Qt 맵으로 전환했습니다.',
            8000,
        )

    @staticmethod
    def _use_embedded_rviz() -> bool:
        backend = os.environ.get('COBOT_GUI_MAP_BACKEND', 'qt').strip().lower()
        return backend in ('rviz', 'embedded_rviz', 'embedded-rviz')

    def _fallback_to_qt_map(self, reason: str):
        if self._map_widget is self.map_panel:
            return
        old_widget = self._map_widget
        index = self._map_layout.indexOf(old_widget)
        if index < 0:
            index = self._map_layout.count()
        self._map_layout.removeWidget(old_widget)
        old_widget.setParent(None)
        old_widget.deleteLater()
        self._map_layout.insertWidget(index, self.map_panel)
        self._map_widget = self.map_panel
        self._rviz_panel = None
        self.statusBar().showMessage(
            f'RViz 임베드 실패: {reason}. 기존 Qt 맵으로 전환했습니다.',
            8000,
        )

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
        if self._rviz_panel is not None:
            self._rviz_panel.stop()
        super().closeEvent(event)

    def _set_autonomous_mode(self, checked=False):
        self._control_mode = 'autonomous'
        self._pressed_keys.clear()
        self._ros.set_control_mode('autonomous')
        self._reset_teleop_speed()
        self.auto_btn.setChecked(True)
        self.manual_btn.setChecked(False)
        self.jackal_manual_btn.setChecked(False)
        self.mode_status_lbl.setText('자율탐사 모드')
        self.setFocus()

    def _set_spot_manual_mode(self, checked=False):
        self._enter_manual_mode('spot')

    def _set_jackal_manual_mode(self, checked=False):
        self._enter_manual_mode('jackal')

    def _set_jackal_auto_mode(self, checked=False):
        """Jackal 자동/재개 통합 버튼:

        1. 수동조작 모드면 → autonomous로 전환 (`/jackal_0/control_mode`)
        2. 홈 복귀 중이면 → 중단하고 큐 dispatch (`/jackal_0/mission_resume`)
        둘 다 no-op이면 안전. Spot 상태는 건드리지 않음.
        """
        self._pressed_keys.clear()
        self._ros.publish_teleop_stop()

        if self.manual_btn.isChecked():
            # Spot은 수동조작 유지 — jackal만 autonomous로.
            self._ros._publish_jackal_control_mode('autonomous')
        else:
            # 전체 autonomous로 복귀 — ros_node 상태 일관성 위해 set_control_mode 호출.
            self._control_mode = 'autonomous'
            self.jackal_manual_btn.setChecked(False)
            self.auto_btn.setChecked(True)
            self._ros.set_teleop_target('spot')
            self._ros.set_control_mode('autonomous')

        # home 가는 중이었으면 중단 + 큐 재개 (idle/active면 mission_manager가 무시).
        self._ros.request_jackal_resume()

        self.mode_status_lbl.setText('Jackal 자동/재개')
        self.setFocus()

    def _enter_manual_mode(self, target: str):
        self._control_mode = 'manual'
        self._pressed_keys.clear()
        # 타겟을 먼저 set → 그 다음 manual 모드 적용 (잘못된 로봇에 잠시
        # 모드 신호 가지 않도록).
        self._ros.set_teleop_target(target)
        self._ros.set_control_mode('manual')
        if target == 'spot':
            self.manual_btn.setChecked(True)
        else:
            self.jackal_manual_btn.setChecked(True)
        self.auto_btn.setChecked(False)
        who = 'Spot' if target == 'spot' else 'Jackal'
        self.mode_status_lbl.setText(f'{who} 수동조작 모드')
        self.setFocus()

    def _request_return_home(self, checked=False):
        self._control_mode = 'autonomous'
        self._pressed_keys.clear()
        self._ros.set_control_mode('autonomous')
        self._ros.request_return_home()
        self._reset_teleop_speed()
        self.auto_btn.setChecked(True)
        self.manual_btn.setChecked(False)
        self.jackal_manual_btn.setChecked(False)
        self.mode_status_lbl.setText('Spot 원점 복귀 요청')
        self.setFocus()

    def _request_jackal_home(self, checked=False):
        self._ros.request_jackal_home()
        # home 복귀 시작 → manual goal 핀 의미 없어짐, clear.
        self.map_panel.clear_jackal_goal_pin()
        self.mode_status_lbl.setText('Jackal 원점 복귀 요청')
        self.setFocus()

    def _on_map_goal_clicked(self, x: float, y: float):
        """우클릭 좌표 → mission_manager에 jackal manual goal로 dispatch + 맵에 핀."""
        self._ros.publish_jackal_manual_goal(x, y)
        self.map_panel.set_jackal_goal_pin(x, y)
        self.mode_status_lbl.setText(f'Jackal 목표 ({x:+.2f}, {y:+.2f}) 전송')
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
