"""UI panel + handlers for the optional Carter secondary robot.

extension.py wires this in with three calls inside its ``omni.ui`` frame:

    from .carter.panel import (
        build_carter_load_button,
        build_carter_panel,
        build_carter_topic_labels,
    )
    ...
    build_carter_load_button(self)   # in the main Load Scene area
    build_carter_panel(self)         # ROS2 graph button block
    build_carter_topic_labels()      # in the topic display section

Drop the spot/carter/ folder and remove those three calls + the import to
delete Carter support entirely.
"""

from __future__ import annotations

import asyncio

import omni.ui as ui

from ..spot.scene import SpotFireRescue
from .constants import (
    CARTER_CMD_VEL_TOPIC,
    CARTER_ODOM_TOPIC,
    CARTER_SCAN_TOPIC,
)
from .ros_graphs import (
    setup_carter_cmd_vel_graph,
    setup_carter_odom_tf_graph,
    setup_carter_scan_tf_graph,
)
from .scene import add_carter_to_world


def build_carter_load_button(ext):
    """Place the 'Load Scene (Carter)' button. This is the default secondary
    pairing — call this first in the main Load Scene button group."""
    ui.Button("1. Load Scene (Carter)", clicked_fn=lambda: _load_scene_carter(ext))


def _setup_carter_all():
    """One click: cmd_vel + odom/TF + scan/TF OmniGraphs for Carter."""
    setup_carter_cmd_vel_graph()
    setup_carter_odom_tf_graph()
    setup_carter_scan_tf_graph()
    print("[cobot3.spot/carter] ✅ Carter ROS graphs all set up")


def build_carter_panel(ext):  # noqa: ARG001 — keep signature symmetric with build_carter_load_button
    """Place the Carter ROS2 graph setup button (single combined trigger)."""
    ui.Label("[ Carter ROS2 그래프 ]", style={"font_size": 12})
    ui.Button(
        "C. Setup Carter ROS (CmdVel + Odom/TF + LiDAR/Scan)",
        clicked_fn=_setup_carter_all,
    )
    ui.Spacer(height=4)


def build_carter_topic_labels():
    """Place /carter_0/* topic labels in the topic display block."""
    ui.Spacer(height=2)
    ui.Label(CARTER_CMD_VEL_TOPIC, style={"font_size": 11})
    ui.Label(CARTER_ODOM_TOPIC, style={"font_size": 11})
    ui.Label(CARTER_SCAN_TOPIC, style={"font_size": 11})


def _load_scene_carter(ext):
    ext._sample = SpotFireRescue()
    # Register Carter co-spawn as the secondary-robot callback. The main
    # scene reads this in setup_scene() and invokes it after spawning Spot.
    ext._sample._secondary_spawn = add_carter_to_world
    asyncio.ensure_future(ext._sample.load_world_async())
    print("[cobot3.spot] Scene 로딩 중... (Carter 모드, 잠시 기다려주세요)")
