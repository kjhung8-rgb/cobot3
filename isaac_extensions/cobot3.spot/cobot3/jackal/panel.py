"""UI panel + handlers for the optional Jackal secondary robot.

extension.py wires this in with three calls inside its ``omni.ui`` frame:

    from .jackal.panel import (
        build_jackal_load_button,
        build_jackal_panel,
        build_jackal_topic_labels,
    )
    ...
    build_jackal_load_button(self)   # next to Carter's Load Scene button
    build_jackal_panel(self)         # after Carter ROS2 graph buttons
    build_jackal_topic_labels()      # in the topic display section

Drop the spot/jackal/ folder and remove those three calls + the import to
delete Jackal support entirely.
"""

from __future__ import annotations

import asyncio

import omni.ui as ui

from ..spot.scene import SpotFireRescue
from .constants import (
    JACKAL_CMD_VEL_TOPIC,
    JACKAL_ODOM_TOPIC,
    JACKAL_SCAN_TOPIC,
)
from .ros_graphs import (
    disable_jackal_lidar_viz,
    setup_jackal_cmd_vel_graph,
    setup_jackal_odom_tf_graph,
)
from .scene import add_jackal_to_world


def build_jackal_load_button(ext):
    """Place the 'Load Scene (Jackal)' button. Call this immediately after
    extension.py's main 'Load Scene (Carter)' button so both sit together."""
    ui.Button("1-J. Load Scene (Jackal)", clicked_fn=lambda: _load_scene_jackal(ext))


def _setup_jackal_all():
    """One click: cmd_vel + odom/TF OmniGraphs + LiDAR viewport viz off.

    No LiDAR scan/TF graph — jackal uses spot's SLAM map for nav and spot's
    position (via spot_obstacle_publisher) as a dynamic obstacle. We still
    turn off the lidar's viewport ray rendering since the USD enables it
    by default.
    """
    setup_jackal_cmd_vel_graph()
    setup_jackal_odom_tf_graph()
    disable_jackal_lidar_viz()
    print("[cobot3.spot/jackal] ✅ Jackal ROS graphs all set up")


def build_jackal_panel(ext):  # noqa: ARG001 — keep signature symmetric with build_jackal_load_button
    """Place the Jackal ROS2 graph setup button (single combined trigger).
    Call inside the main extension UI frame, after the Carter ROS2 section."""
    ui.Label("[ Jackal ROS2 그래프 ]", style={"font_size": 12})
    ui.Button(
        "J. Setup Jackal ROS (CmdVel + Odom/TF)",
        clicked_fn=_setup_jackal_all,
    )
    ui.Spacer(height=4)


def build_jackal_topic_labels():
    """Place /jackal_0/* topic labels. Call inside the topic display block."""
    ui.Spacer(height=2)
    ui.Label(JACKAL_CMD_VEL_TOPIC, style={"font_size": 11})
    ui.Label(JACKAL_ODOM_TOPIC, style={"font_size": 11})
    ui.Label(JACKAL_SCAN_TOPIC, style={"font_size": 11})


def _load_scene_jackal(ext):
    ext._sample = SpotFireRescue()
    # Register the Jackal spawn helper as the secondary-robot callback. The
    # main scene reads this in setup_scene() and calls it in place of the
    # default _add_carter().
    ext._sample._secondary_spawn = add_jackal_to_world
    asyncio.ensure_future(ext._sample.load_world_async())
    print("[cobot3.spot] Scene 로딩 중... (Jackal 모드, 잠시 기다려주세요)")
