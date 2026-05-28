"""Compatibility wrappers for the Carter ROS graph helpers.

Carter support now lives in ``cobot3.carter`` next to the Jackal package.
Keep this module so older imports under ``cobot3.spot`` do not break extension
activation after merges.
"""

from __future__ import annotations

from ..carter.ros_graphs import (
    setup_carter_cmd_vel_graph,
    setup_carter_odom_tf_graph,
    setup_carter_scan_tf_graph,
)

__all__ = [
    "setup_carter_cmd_vel_graph",
    "setup_carter_odom_tf_graph",
    "setup_carter_scan_tf_graph",
]
