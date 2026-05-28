"""
Cobot3 Spot Extension
=====================

Thin UI entrypoint. Heavy logic is split into:
  - scene.py            : SpotFireRescue world/policy runtime
  - ros_graphs.py       : cmd_vel, camera, LiDAR/SLAM graph setup
  - teleop_launcher.py  : terminal launcher for spot_teleop.py
  - constants.py        : /spot_0 topic/frame contract

Optional secondary robots (Carter, Jackal) are isolated in cobot3/carter/ and
cobot3/jackal/ subpackages. Each contributes its own UI block via the three
``build_*`` helpers in its panel.py. To drop one direction, delete the
subpackage folder and remove the two-line panel imports + the three
``build_*`` calls below.
"""

from __future__ import annotations

import asyncio

import omni.ext
import omni.timeline
import omni.ui as ui

from .constants import (
    CAMERA_SPECS,
    CMD_VEL_TOPIC,
    ODOM_TOPIC,
    SCAN_TOPIC,
)
from .ros_graphs import setup_camera_graph, setup_cmd_vel_graph, setup_slam_sensors
from .teleop_launcher import TeleopLauncher
from ..utils import get_ros_domain_id

# ── Optional secondary-robot panels ────────────────────────────────────────
# Sibling subpackages under cobot3/ — each is fully self-contained. To drop
# one, delete its folder and remove the matching import + the three
# build_*_load_button/panel/topic_labels calls below.
from ..carter.panel import (
    build_carter_load_button,
    build_carter_panel,
    build_carter_topic_labels,
)
from ..jackal.panel import (
    build_jackal_load_button,
    build_jackal_panel,
    build_jackal_topic_labels,
)


class Cobot3SpotExtension(omni.ext.IExt):
    """Isaac Sim extension UI for the Cobot3 Spot fire-rescue project."""

    def on_startup(self, ext_id):
        print("[cobot3.spot] Extension 시작")
        self._sample = None
        self._teleop = TeleopLauncher()

        self._window = ui.Window("Cobot3 Spot - Fire Rescue", width=520, height=620)
        with self._window.frame:
            with ui.VStack(spacing=6):
                ui.Label("🔥 Cobot3 Spot - Fire Rescue", style={"font_size": 16})
                ui.Label(f"ROS_DOMAIN_ID: {get_ros_domain_id()}", style={"font_size": 11})
                ui.Spacer(height=4)

                ui.Label("[ Isaac Sim 설정 ]", style={"font_size": 12})
                build_carter_load_button(self)
                build_jackal_load_button(self)
                ui.Button(
                    "2. Setup Spot ROS (CmdVel + Camera + LiDAR/SLAM)",
                    clicked_fn=self._setup_spot_all,
                )
                ui.Spacer(height=4)

                build_carter_panel(self)
                build_jackal_panel(self)

                ui.Label("[ 제어 ]", style={"font_size": 12})
                ui.Button("Start Teleop Terminal", clicked_fn=self._start_teleop)
                ui.Button("Stop Teleop Terminal", clicked_fn=self._stop_teleop)
                ui.Button("Reset", clicked_fn=self._reset)
                ui.Button("Stop Timeline", clicked_fn=self._stop)
                ui.Spacer(height=-0.4)

                ui.Label("[ 주요 토픽 ]", style={"font_size": 12})
                ui.Label(CMD_VEL_TOPIC, style={"font_size": 11})
                ui.Label(SCAN_TOPIC, style={"font_size": 11})
                ui.Label(ODOM_TOPIC, style={"font_size": 11})
                for spec in CAMERA_SPECS:
                    ui.Label(f"{spec['label']} RGB: {spec['color_topic']}", style={"font_size": 11})
                    ui.Label(f"{spec['label']} DEPTH: {spec['depth_topic']}", style={"font_size": 11})
                    ui.Label(f"{spec['label']} INFO: {spec['camera_info_topic']}", style={"font_size": 11})
                build_carter_topic_labels()
                build_jackal_topic_labels()

        print("[cobot3.spot] UI 준비 완료")

    def on_shutdown(self):
        print("[cobot3.spot] Extension 종료")
        if hasattr(self, "_teleop") and self._teleop:
            self._teleop.stop()
        self._window = None

    def _setup_spot_all(self):
        """One click: spot cmd_vel + camera + LiDAR/SLAM OmniGraphs."""
        setup_cmd_vel_graph(self._sample)
        setup_camera_graph(self._sample)
        setup_slam_sensors(self._sample)
        print("[cobot3.spot] ✅ Spot ROS graphs all set up")

    def _start_teleop(self):
        self._teleop.start()

    def _stop_teleop(self):
        self._teleop.stop()

    def _reset(self):
        if self._sample:
            self._sample.reset_command()
            asyncio.ensure_future(self._sample.reset_async())

    def _stop(self):
        if self._sample:
            self._sample.reset_command()
        omni.timeline.get_timeline_interface().stop()
