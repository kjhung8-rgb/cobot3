"""Small utility helpers for the Cobot3 Spot Isaac Sim extension."""

from __future__ import annotations

import os

from .constants import DEFAULT_ROS_DOMAIN_ID


def get_ros_domain_id() -> int:
    """Return ROS_DOMAIN_ID from the shell that launched Isaac Sim."""
    raw = os.environ.get("ROS_DOMAIN_ID", str(DEFAULT_ROS_DOMAIN_ID))
    try:
        return int(raw)
    except (TypeError, ValueError):
        print(f"[cobot3.spot] ⚠️ invalid ROS_DOMAIN_ID={raw!r}; fallback={DEFAULT_ROS_DOMAIN_ID}")
        return DEFAULT_ROS_DOMAIN_ID
