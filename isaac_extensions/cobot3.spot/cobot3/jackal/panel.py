"""UI panel + handlers for the optional Jackal secondary robot.

extension.py wires this in with three calls inside its ``omni.ui`` frame:

    from .jackal.panel import (
        build_jackal_load_button,
        build_jackal_panel,
        build_jackal_topic_labels,
    )
    ...
    build_jackal_load_button(self)
    build_jackal_panel(self)
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
    setup_jackal_scan_tf_graph,
)
from .scene import add_jackal_to_world


def build_jackal_load_button(ext):
    """Place the 'Load Scene (Jackal)' button."""
    ui.Button("1-J. Load Scene (Jackal)", clicked_fn=lambda: _load_scene_jackal(ext))


def _setup_jackal_all():
    """One click: cmd_vel + odom/TF + scan/TF OmniGraphs + viewport viz off.

    Scan graph는 jackal local_costmap obstacle_layer가 /jackal_0/scan을
    사용하기 위해 활성화. spot이 마스킹한 /map static_layer 외에 jackal
    자기 lidar로 실시간 obstacle 감지해 충돌 회피.
    """
    setup_jackal_cmd_vel_graph()
    setup_jackal_odom_tf_graph()
    setup_jackal_scan_tf_graph()
    disable_jackal_lidar_viz()
    print("[cobot3.spot/jackal] ✅ Jackal ROS graphs all set up (cmd_vel + odom + scan)")


def build_jackal_panel(ext):  # noqa: ARG001 — keep signature symmetric with build_jackal_load_button
    """Place the Jackal ROS2 graph setup button."""
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
    # main scene reads this in setup_scene().
    ext._sample._secondary_spawn = add_jackal_to_world
    asyncio.ensure_future(ext._sample.load_world_async())
    print("[cobot3.spot] Scene 로딩 중... (Jackal 모드, 잠시 기다려주세요)")
