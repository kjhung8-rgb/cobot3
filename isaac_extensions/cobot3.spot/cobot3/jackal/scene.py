"""Scene helper for spawning Clearpath Jackal as the secondary robot.

Mirrors the Carter spawn pattern in spot/scene.py:_add_carter but kept as a
free function so spot/scene.py only needs one lazy import + one branch to
wire it in. The panel module sets ``sample._secondary_spawn`` to this
function before ``load_world_async()`` runs.
"""

from __future__ import annotations

import math

import carb
import numpy as np

from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path

from .constants import (
    JACKAL_PRIM_PATH,
    JACKAL_SPAWN_POSITION,
    JACKAL_SPAWN_YAW_DEG,
    JACKAL_USD_NUCLEUS_PATH,
)


def _yaw_to_quat_wxyz(yaw_deg):
    half_yaw = math.radians(yaw_deg) * 0.5
    return np.array([math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)], dtype=np.float64)


def add_jackal_to_world(sample):
    """Spawn Clearpath Jackal in place of Carter. Sets ``sample.jackal``."""
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("[cobot3.spot/jackal] spawn: Isaac assets root 못 찾음")
        return

    jackal_usd = assets_root_path + JACKAL_USD_NUCLEUS_PATH
    add_reference_to_stage(usd_path=jackal_usd, prim_path=JACKAL_PRIM_PATH)

    sample.jackal = SingleArticulation(
        prim_path=JACKAL_PRIM_PATH,
        name="Jackal",
        position=np.array(JACKAL_SPAWN_POSITION, dtype=np.float64),
        orientation=_yaw_to_quat_wxyz(JACKAL_SPAWN_YAW_DEG),
    )
    print(f"[cobot3.spot/jackal] spawn: {jackal_usd}")
