"""Shared utility helpers for the cobot3.spot Isaac Sim extension.

Lives at the cobot3/ package root so spot, carter, and jackal subpackages
can all import from it without crossing each other's namespaces.
"""

from __future__ import annotations

import os

DEFAULT_ROS_DOMAIN_ID = 141


def get_ros_domain_id() -> int:
    """Return ROS_DOMAIN_ID from the shell that launched Isaac Sim."""
    raw = os.environ.get("ROS_DOMAIN_ID", str(DEFAULT_ROS_DOMAIN_ID))
    try:
        return int(raw)
    except (TypeError, ValueError):
        print(f"[cobot3] ⚠️ invalid ROS_DOMAIN_ID={raw!r}; fallback={DEFAULT_ROS_DOMAIN_ID}")
        return DEFAULT_ROS_DOMAIN_ID
