"""Scene helper for co-spawning Nova Carter as the secondary robot.

Mirrors spot/jackal/scene.py. The Carter panel sets
``sample._secondary_spawn = add_carter_to_world`` before
``load_world_async()``; spot/scene.py invokes whatever callback is registered.
"""

from __future__ import annotations

import math

import carb
import numpy as np

from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path

from .constants import (
    CARTER_PRIM_PATH,
    CARTER_SPAWN_POSITION,
    CARTER_SPAWN_YAW_DEG,
    CARTER_USD_NUCLEUS_PATH,
)


def _yaw_to_quat_wxyz(yaw_deg):
    half_yaw = math.radians(yaw_deg) * 0.5
    return np.array([math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)], dtype=np.float64)


def add_carter_to_world(sample):
    """Co-spawn Nova Carter alongside Spot. Sets ``sample.carter``."""
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("[cobot3.spot/carter] co-spawn: Isaac assets root 못 찾음")
        return

    carter_usd = assets_root_path + CARTER_USD_NUCLEUS_PATH
    add_reference_to_stage(usd_path=carter_usd, prim_path=CARTER_PRIM_PATH)

    sample.carter = SingleArticulation(
        prim_path=CARTER_PRIM_PATH,
        name="Carter",
        position=np.array(CARTER_SPAWN_POSITION, dtype=np.float64),
        orientation=_yaw_to_quat_wxyz(CARTER_SPAWN_YAW_DEG),
    )
    print(f"[cobot3.spot/carter] co-spawn: {carter_usd}")
