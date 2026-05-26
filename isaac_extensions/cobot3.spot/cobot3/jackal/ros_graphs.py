"""ROS2 OmniGraph setup helpers for the optional Clearpath Jackal robot.

Mirror of carter_ros_graphs.py — Jackal is a smaller, skid-steer alternative
that drops into the same /map produced by spot SLAM. Run either Carter OR
Jackal, not both at once (separate UI buttons in extension.py).

Notes:

* All TF timestamps come from IsaacReadSystemTime — same reason as Carter:
  spot graphs and LidarHelper use system time, mixing sim/system breaks AMCL.
* Jackal is skid-steer (4 wheels). The DifferentialController outputs 2
  velocities (left, right) and the ArticulationController broadcasts each
  side's velocity to its two wheels via the joint name ordering in
  JACKAL_WHEEL_JOINT_NAMES. If wheels spin opposite directions or the robot
  doesn't move, inspect the actual joint names in the imported USD.
* Namespace isolation via /jackal_0 prefixes keeps Jackal apart from spot's
  /spot_0 topics on the shared ROS_DOMAIN_ID.
"""

from __future__ import annotations

import traceback

import omni.graph.core as og
import omni.usd
import usdrt

from .constants import (
    JACKAL_BASE_LINK_FRAME,
    JACKAL_CHASSIS_PRIM_PATH,
    JACKAL_CMD_VEL_GRAPH_PATH,
    JACKAL_CMD_VEL_TOPIC,
    JACKAL_LIDAR_FRAME,
    JACKAL_LIDAR_SENSOR_HINTS,
    JACKAL_LIDAR_TF_ROTATION_XYZW,
    JACKAL_LIDAR_TRANSLATION,
    JACKAL_MAX_ANGULAR_SPEED,
    JACKAL_MAX_LINEAR_SPEED,
    JACKAL_ODOM_FRAME,
    JACKAL_ODOM_TF_GRAPH_PATH,
    JACKAL_ODOM_TOPIC,
    JACKAL_PRIM_PATH,
    JACKAL_SCAN_TF_GRAPH_PATH,
    JACKAL_SCAN_TOPIC,
    JACKAL_WHEEL_DISTANCE,
    JACKAL_WHEEL_JOINT_NAMES,
    JACKAL_WHEEL_RADIUS,
)
from ..utils import get_ros_domain_id


def _jackal_on_stage() -> bool:
    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath(JACKAL_PRIM_PATH).IsValid():
        print(
            f"[cobot3.spot/jackal] {JACKAL_PRIM_PATH} not on stage — "
            "먼저 Load Scene을 눌러주세요 (scene.py에서 Jackal co-spawn 필요)."
        )
        return False
    return True


# ─────────────────────────────────────────────
# /jackal_0/cmd_vel -> differential drive (skid-steer 4-wheel)
# ─────────────────────────────────────────────
def setup_jackal_cmd_vel_graph():
    if not _jackal_on_stage():
        return

    graph_path = JACKAL_CMD_VEL_GRAPH_PATH
    domain_id = get_ros_domain_id()
    stage = omni.usd.get_context().get_stage()

    if stage.GetPrimAtPath(graph_path).IsValid():
        stage.RemovePrim(graph_path)

    keys = og.Controller.Keys
    try:
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                    ("BreakLinVel", "omni.graph.nodes.BreakVector3"),
                    ("BreakAngVel", "omni.graph.nodes.BreakVector3"),
                    ("DiffController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                    # Concat DiffController's [L, R] with itself → [L, R, L, R] for the
                    # 4-wheel skid-steer joint order (front_left, front_right, rear_left,
                    # rear_right).
                    ("ConcatLR", "omni.graph.nodes.AppendArray"),
                    ("ArtController", "isaacsim.core.nodes.IsaacArticulationController"),
                ],
                keys.SET_VALUES: [
                    ("ROS2Context.inputs:domain_id", domain_id),
                    ("SubscribeTwist.inputs:topicName", JACKAL_CMD_VEL_TOPIC),
                    ("DiffController.inputs:wheelRadius", float(JACKAL_WHEEL_RADIUS)),
                    ("DiffController.inputs:wheelDistance", float(JACKAL_WHEEL_DISTANCE)),
                    ("DiffController.inputs:maxLinearSpeed", float(JACKAL_MAX_LINEAR_SPEED)),
                    ("DiffController.inputs:maxAngularSpeed", float(JACKAL_MAX_ANGULAR_SPEED)),
                    ("ArtController.inputs:jointNames", JACKAL_WHEEL_JOINT_NAMES),
                    ("ArtController.inputs:targetPrim", [usdrt.Sdf.Path(JACKAL_CHASSIS_PRIM_PATH)]),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ArtController.inputs:execIn"),
                    ("ROS2Context.outputs:context", "SubscribeTwist.inputs:context"),
                    ("SubscribeTwist.outputs:execOut", "DiffController.inputs:execIn"),

                    ("SubscribeTwist.outputs:linearVelocity", "BreakLinVel.inputs:tuple"),
                    ("BreakLinVel.outputs:x", "DiffController.inputs:linearVelocity"),

                    ("SubscribeTwist.outputs:angularVelocity", "BreakAngVel.inputs:tuple"),
                    ("BreakAngVel.outputs:z", "DiffController.inputs:angularVelocity"),

                    # [L, R] + [L, R] = [L, R, L, R] → 4-wheel ArtController
                    ("DiffController.outputs:velocityCommand", "ConcatLR.inputs:input0"),
                    ("DiffController.outputs:velocityCommand", "ConcatLR.inputs:input1"),
                    ("ConcatLR.outputs:array", "ArtController.inputs:velocityCommand"),
                ],
            },
        )
        print(f"[cobot3.spot/jackal] ✅ cmd_vel graph: {JACKAL_CMD_VEL_TOPIC} (domain={domain_id})")
    except Exception as exc:
        print(f"[cobot3.spot/jackal] ❌ cmd_vel graph 생성 실패: {exc}")
        traceback.print_exc()


# ─────────────────────────────────────────────
# /jackal_0/odom + /tf (jackal_0/odom -> jackal_0/base_link)
# ─────────────────────────────────────────────
def setup_jackal_odom_tf_graph():
    if not _jackal_on_stage():
        return

    graph_path = JACKAL_ODOM_TF_GRAPH_PATH
    domain_id = get_ros_domain_id()
    stage = omni.usd.get_context().get_stage()

    if stage.GetPrimAtPath(graph_path).IsValid():
        stage.RemovePrim(graph_path)

    keys = og.Controller.Keys
    try:
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("ReadSystemTime", "isaacsim.core.nodes.IsaacReadSystemTime"),
                    ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                    ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                    ("PublishOdomTf", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ],
                keys.SET_VALUES: [
                    ("ROS2Context.inputs:domain_id", domain_id),
                    ("ComputeOdom.inputs:chassisPrim", [usdrt.Sdf.Path(JACKAL_CHASSIS_PRIM_PATH)]),
                    ("PublishOdom.inputs:topicName", JACKAL_ODOM_TOPIC),
                    ("PublishOdom.inputs:odomFrameId", JACKAL_ODOM_FRAME),
                    ("PublishOdom.inputs:chassisFrameId", JACKAL_BASE_LINK_FRAME),
                    ("PublishOdom.inputs:publishRawVelocities", True),
                    ("PublishOdomTf.inputs:topicName", "/tf"),
                    ("PublishOdomTf.inputs:parentFrameId", JACKAL_ODOM_FRAME),
                    ("PublishOdomTf.inputs:childFrameId", JACKAL_BASE_LINK_FRAME),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ComputeOdom.inputs:execIn"),
                    ("ComputeOdom.outputs:execOut", "PublishOdom.inputs:execIn"),
                    ("ComputeOdom.outputs:execOut", "PublishOdomTf.inputs:execIn"),
                    ("ROS2Context.outputs:context", "PublishOdom.inputs:context"),
                    ("ROS2Context.outputs:context", "PublishOdomTf.inputs:context"),
                    ("ReadSystemTime.outputs:systemTime", "PublishOdom.inputs:timeStamp"),
                    ("ReadSystemTime.outputs:systemTime", "PublishOdomTf.inputs:timeStamp"),

                    ("ComputeOdom.outputs:position", "PublishOdom.inputs:position"),
                    ("ComputeOdom.outputs:orientation", "PublishOdom.inputs:orientation"),
                    ("ComputeOdom.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
                    ("ComputeOdom.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),

                    ("ComputeOdom.outputs:position", "PublishOdomTf.inputs:translation"),
                    ("ComputeOdom.outputs:orientation", "PublishOdomTf.inputs:rotation"),
                ],
            },
        )
        print(
            f"[cobot3.spot/jackal] ✅ odom/TF graph: {JACKAL_ODOM_TOPIC} "
            f"({JACKAL_ODOM_FRAME} -> {JACKAL_BASE_LINK_FRAME})"
        )
    except Exception as exc:
        print(f"[cobot3.spot/jackal] ❌ odom/TF graph 생성 실패: {exc}")
        traceback.print_exc()


# ─────────────────────────────────────────────
# /jackal_0/scan + static TF (jackal_0/base_link -> jackal_0/laser)
# ─────────────────────────────────────────────
_LIDAR_PRIM_TYPE_HINTS = ("OmniLidar", "RtxLidar", "Lidar")


def _iter_descendants(prim):
    yield prim
    for child in prim.GetChildren():
        yield from _iter_descendants(child)


def _find_jackal_lidar_prim(stage):
    root_prim = stage.GetPrimAtPath(JACKAL_PRIM_PATH)
    if not root_prim.IsValid():
        return None

    preferred = None
    fallback = None
    for prim in _iter_descendants(root_prim):
        type_name = str(prim.GetTypeName())
        if not any(hint in type_name for hint in _LIDAR_PRIM_TYPE_HINTS):
            continue
        path_str = str(prim.GetPath())
        if any(hint in path_str.lower() for hint in (h.lower() for h in JACKAL_LIDAR_SENSOR_HINTS)):
            preferred = path_str
            break
        if fallback is None:
            fallback = path_str
    return preferred or fallback


def disable_jackal_lidar_viz():
    """Turn off the LiDAR ray viewport rendering on the USD prim.

    The Jackal USD has a SICK lidar that draws bright lines by default. This
    helper finds the lidar prim and sets the draw* attributes to False so the
    viewport stays clean. Safe to call without setting up any scan graph —
    we don't use the lidar for navigation in the spot-as-obstacle architecture.
    """
    if not _jackal_on_stage():
        return

    stage = omni.usd.get_context().get_stage()
    lidar_prim_path = _find_jackal_lidar_prim(stage)
    if not lidar_prim_path:
        print(f"[cobot3.spot/jackal] LiDAR prim 못 찾음 — viz off skip")
        return

    lidar_prim = stage.GetPrimAtPath(lidar_prim_path)
    for attr_name in (
        "drawLines", "drawPoints", "showLines", "showPoints",
        "omni:isaacSim:drawLines", "omni:isaacSim:drawPoints",
    ):
        attr = lidar_prim.GetAttribute(attr_name)
        if attr and attr.IsValid():
            try:
                attr.Set(False)
            except Exception:
                pass
    print(f"[cobot3.spot/jackal] LiDAR viz off: {lidar_prim_path}")


def setup_jackal_scan_tf_graph():
    if not _jackal_on_stage():
        return

    domain_id = get_ros_domain_id()
    stage = omni.usd.get_context().get_stage()

    lidar_prim_path = _find_jackal_lidar_prim(stage)
    if not lidar_prim_path:
        print(
            f"[cobot3.spot/jackal] ❌ {JACKAL_PRIM_PATH} 아래 LiDAR sensor prim 없음. "
            "Stage 패널에서 OmniLidar/RtxLidar prim path 확인 후 알려주세요."
        )
        return
    print(f"[cobot3.spot/jackal] LiDAR prim 발견: {lidar_prim_path}")

    disable_jackal_lidar_viz()

    graph_path = JACKAL_SCAN_TF_GRAPH_PATH
    if stage.GetPrimAtPath(graph_path).IsValid():
        stage.RemovePrim(graph_path)

    # jackal SICK LMS1xx is a PhysX RangeSensor (USD type "Lidar"), NOT an
    # RTX LiDAR (Camera + RtxLidarSensor schema). ROS2RtxLidarHelper rejects
    # this prim with "Render product not attached to RTX Lidar". So we use
    # the PhysX path: IsaacReadLidarBeams reads raw beams from the sensor,
    # and ROS2PublishLaserScan converts and publishes the LaserScan.
    keys = og.Controller.Keys
    try:
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("ReadSystemTime", "isaacsim.core.nodes.IsaacReadSystemTime"),
                    ("ReadLidar", "isaacsim.sensors.physx.IsaacReadLidarBeams"),
                    ("PublishScan", "isaacsim.ros2.bridge.ROS2PublishLaserScan"),
                    ("PublishLaserTf", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ],
                keys.SET_VALUES: [
                    ("ROS2Context.inputs:domain_id", domain_id),
                    ("ReadLidar.inputs:lidarPrim", [usdrt.Sdf.Path(lidar_prim_path)]),
                    ("PublishScan.inputs:topicName", JACKAL_SCAN_TOPIC),
                    ("PublishScan.inputs:frameId", JACKAL_LIDAR_FRAME),

                    ("PublishLaserTf.inputs:topicName", "/tf_static"),
                    ("PublishLaserTf.inputs:staticPublisher", True),
                    ("PublishLaserTf.inputs:parentFrameId", JACKAL_BASE_LINK_FRAME),
                    ("PublishLaserTf.inputs:childFrameId", JACKAL_LIDAR_FRAME),
                    ("PublishLaserTf.inputs:translation", list(JACKAL_LIDAR_TRANSLATION)),
                    ("PublishLaserTf.inputs:rotation", list(JACKAL_LIDAR_TF_ROTATION_XYZW)),
                ],
                keys.CONNECT: [
                    # ReadLidar pipeline
                    ("OnPlaybackTick.outputs:tick", "ReadLidar.inputs:execIn"),
                    ("ReadLidar.outputs:execOut", "PublishScan.inputs:execIn"),
                    ("ROS2Context.outputs:context", "PublishScan.inputs:context"),
                    ("ReadSystemTime.outputs:systemTime", "PublishScan.inputs:timeStamp"),

                    # Hand all the beam metadata to the LaserScan publisher.
                    ("ReadLidar.outputs:azimuthRange", "PublishScan.inputs:azimuthRange"),
                    ("ReadLidar.outputs:depthRange", "PublishScan.inputs:depthRange"),
                    ("ReadLidar.outputs:horizontalFov", "PublishScan.inputs:horizontalFov"),
                    ("ReadLidar.outputs:horizontalResolution", "PublishScan.inputs:horizontalResolution"),
                    ("ReadLidar.outputs:linearDepthData", "PublishScan.inputs:linearDepthData"),
                    ("ReadLidar.outputs:numCols", "PublishScan.inputs:numCols"),
                    ("ReadLidar.outputs:numRows", "PublishScan.inputs:numRows"),
                    ("ReadLidar.outputs:rotationRate", "PublishScan.inputs:rotationRate"),

                    # Static TF for the laser frame
                    ("OnPlaybackTick.outputs:tick", "PublishLaserTf.inputs:execIn"),
                    ("ROS2Context.outputs:context", "PublishLaserTf.inputs:context"),
                    ("ReadSystemTime.outputs:systemTime", "PublishLaserTf.inputs:timeStamp"),
                ],
            },
        )
        print(f"[cobot3.spot/jackal] ✅ scan/TF graph (PhysX): {JACKAL_SCAN_TOPIC} (frame={JACKAL_LIDAR_FRAME})")
    except Exception as exc:
        print(f"[cobot3.spot/jackal] ❌ scan/TF graph 생성 실패: {exc}")
        traceback.print_exc()
