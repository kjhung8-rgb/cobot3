"""
Cobot3 Spot Extension
=====================

Thin UI entrypoint. Heavy logic is split into:
  - scene.py            : SpotFireRescue world/policy runtime
  - ros_graphs.py       : cmd_vel, camera, LiDAR/SLAM graph setup
  - teleop_launcher.py  : terminal launcher for spot_teleop.py
  - constants.py        : /spot_0 topic/frame contract
"""

from __future__ import annotations

import asyncio

import numpy as np
import omni.ext
import omni.timeline
import omni.ui as ui

from .constants import (
    CAMERA_INFO_TOPIC,
    CMD_VEL_TOPIC,
    COLOR_IMAGE_TOPIC,
    DEPTH_IMAGE_TOPIC,
    ODOM_TOPIC,
    SCAN_TOPIC,
)
from .ros_graphs import setup_camera_graph, setup_cmd_vel_graph, setup_slam_sensors
from .scene import SpotFireRescue
from .teleop_launcher import TeleopLauncher
from .utils import get_ros_domain_id


class Cobot3SpotExtension(omni.ext.IExt):
    """Isaac Sim extension UI for the Cobot3 Spot fire-rescue project."""

    def on_startup(self, ext_id):
        print("[cobot3.spot] Extension 시작")
        self._sample = None
        self._teleop = TeleopLauncher()

        self._window = ui.Window("Cobot3 Spot - Fire Rescue", width=440, height=500)
        with self._window.frame:
            with ui.VStack(spacing=6):
                ui.Label("🔥 Cobot3 Spot - Fire Rescue", style={"font_size": 16})
                ui.Label(f"ROS_DOMAIN_ID: {get_ros_domain_id()}", style={"font_size": 11})
                ui.Spacer(height=4)

                ui.Label("[ Isaac Sim 설정 ]", style={"font_size": 12})
                ui.Button("1. Load Scene", clicked_fn=self._load_scene)
                ui.Button("2. Setup ROS2 CmdVel", clicked_fn=self._setup_ros2)
                ui.Button("3. Setup Camera", clicked_fn=self._setup_camera)
                ui.Button("4. Setup LiDAR/SLAM", clicked_fn=self._setup_slam_sensors)
                ui.Spacer(height=4)

                ui.Label("[ 제어 ]", style={"font_size": 12})
                ui.Button("Start Teleop Terminal", clicked_fn=self._start_teleop)
                ui.Button("Stop Teleop Terminal", clicked_fn=self._stop_teleop)
                ui.Button("Reset", clicked_fn=self._reset)
                ui.Button("Stop Timeline", clicked_fn=self._stop)
                ui.Spacer(height=4)

                ui.Label("[ 주요 토픽 ]", style={"font_size": 12})
                ui.Label(CMD_VEL_TOPIC, style={"font_size": 11})
                ui.Label(SCAN_TOPIC, style={"font_size": 11})
                ui.Label(ODOM_TOPIC, style={"font_size": 11})
                ui.Label(COLOR_IMAGE_TOPIC, style={"font_size": 11})
                ui.Label(DEPTH_IMAGE_TOPIC, style={"font_size": 11})
                ui.Label(CAMERA_INFO_TOPIC, style={"font_size": 11})

        print("[cobot3.spot] UI 준비 완료")

    def on_shutdown(self):
        print("[cobot3.spot] Extension 종료")
        if hasattr(self, "_teleop") and self._teleop:
            self._teleop.stop()
        self._window = None

    def _load_scene(self):
        self._sample = SpotFireRescue()
        asyncio.ensure_future(self._sample.load_world_async())
        print("[cobot3.spot] Scene 로딩 중... (잠시 기다려주세요)")

    def _setup_ros2(self):
        setup_cmd_vel_graph(self._sample)

    def _setup_camera(self):
        setup_camera_graph(self._sample)

    def _setup_slam_sensors(self):
        setup_slam_sensors(self._sample)

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
