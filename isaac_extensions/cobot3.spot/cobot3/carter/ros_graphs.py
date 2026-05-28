"""ROS2 OmniGraph setup helpers for the co-spawned Nova Carter robot.

Design rules learned the hard way:

* All TF timestamps come from IsaacReadSystemTime — NOT IsaacReadSimulationTime.
  Spot's graphs use system time too, and LidarHelper publishes scans with
  useSystemTime=True. Mixing sim/system time makes AMCL drop every scan with
  "earlier than all data in the transform cache".
* Carter and Spot share the same ROS_DOMAIN_ID (read from env). Namespace
  isolation via /carter_0 prefixes is what keeps the two stacks apart.
"""

from __future__ import annotations

import traceback

import omni.graph.core as og
import omni.usd
import usdrt

from .constants import (
    CARTER_BASE_LINK_FRAME,
    CARTER_CHASSIS_PRIM_PATH,
    CARTER_CMD_VEL_GRAPH_PATH,
    CARTER_CMD_VEL_TOPIC,
    CARTER_LIDAR_FRAME,
    CARTER_LIDAR_SENSOR_HINTS,
    CARTER_LIDAR_TF_ROTATION_XYZW,
    CARTER_LIDAR_TRANSLATION,
    CARTER_MAX_ANGULAR_SPEED,
    CARTER_MAX_LINEAR_SPEED,
    CARTER_ODOM_FRAME,
    CARTER_ODOM_TF_GRAPH_PATH,
    CARTER_ODOM_TOPIC,
    CARTER_PRIM_PATH,
    CARTER_SCAN_TF_GRAPH_PATH,
    CARTER_SCAN_TOPIC,
    CARTER_WHEEL_DISTANCE,
    CARTER_WHEEL_JOINT_NAMES,
    CARTER_WHEEL_RADIUS,
)
from ..utils import get_ros_domain_id


def _carter_on_stage() -> bool:
    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath(CARTER_PRIM_PATH).IsValid():
        print(
            f"[cobot3.spot/carter] {CARTER_PRIM_PATH} not on stage — "
            "먼저 Load Scene을 눌러주세요."
        )
        return False
    return True


# ─────────────────────────────────────────────
# /carter_0/cmd_vel -> differential drive
# ─────────────────────────────────────────────
def setup_carter_cmd_vel_graph():
    if not _carter_on_stage():
        return

    graph_path = CARTER_CMD_VEL_GRAPH_PATH
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
                    ("ArtController", "isaacsim.core.nodes.IsaacArticulationController"),
                ],
                keys.SET_VALUES: [
                    ("ROS2Context.inputs:domain_id", domain_id),
                    ("SubscribeTwist.inputs:topicName", CARTER_CMD_VEL_TOPIC),
                    ("DiffController.inputs:wheelRadius", float(CARTER_WHEEL_RADIUS)),
                    ("DiffController.inputs:wheelDistance", float(CARTER_WHEEL_DISTANCE)),
                    ("DiffController.inputs:maxLinearSpeed", float(CARTER_MAX_LINEAR_SPEED)),
                    ("DiffController.inputs:maxAngularSpeed", float(CARTER_MAX_ANGULAR_SPEED)),
                    ("ArtController.inputs:jointNames", CARTER_WHEEL_JOINT_NAMES),
                    ("ArtController.inputs:targetPrim", [usdrt.Sdf.Path(CARTER_CHASSIS_PRIM_PATH)]),
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

                    ("DiffController.outputs:velocityCommand", "ArtController.inputs:velocityCommand"),
                ],
            },
        )
        print(f"[cobot3.spot/carter] ✅ cmd_vel graph: {CARTER_CMD_VEL_TOPIC} (domain={domain_id})")
    except Exception as exc:
        print(f"[cobot3.spot/carter] ❌ cmd_vel graph 생성 실패: {exc}")
        traceback.print_exc()


# ─────────────────────────────────────────────
# /carter_0/odom + /tf (carter_0/odom -> carter_0/base_link)
# ─────────────────────────────────────────────
def setup_carter_odom_tf_graph():
    if not _carter_on_stage():
        return

    graph_path = CARTER_ODOM_TF_GRAPH_PATH
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
                    ("ComputeOdom.inputs:chassisPrim", [usdrt.Sdf.Path(CARTER_CHASSIS_PRIM_PATH)]),
                    ("PublishOdom.inputs:topicName", CARTER_ODOM_TOPIC),
                    ("PublishOdom.inputs:odomFrameId", CARTER_ODOM_FRAME),
                    ("PublishOdom.inputs:chassisFrameId", CARTER_BASE_LINK_FRAME),
                    ("PublishOdom.inputs:publishRawVelocities", True),
                    ("PublishOdomTf.inputs:topicName", "/tf"),
                    ("PublishOdomTf.inputs:parentFrameId", CARTER_ODOM_FRAME),
                    ("PublishOdomTf.inputs:childFrameId", CARTER_BASE_LINK_FRAME),
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
            f"[cobot3.spot/carter] ✅ odom/TF graph: {CARTER_ODOM_TOPIC} "
            f"({CARTER_ODOM_FRAME} -> {CARTER_BASE_LINK_FRAME})"
        )
    except Exception as exc:
        print(f"[cobot3.spot/carter] ❌ odom/TF graph 생성 실패: {exc}")
        traceback.print_exc()


# ─────────────────────────────────────────────
# /carter_0/scan + static TF (carter_0/base_link -> carter_0/laser)
# ─────────────────────────────────────────────
_LIDAR_PRIM_TYPE_HINTS = ("OmniLidar", "RtxLidar", "Lidar")


def _iter_descendants(prim):
    yield prim
    for child in prim.GetChildren():
        yield from _iter_descendants(child)


def _find_carter_lidar_prim(stage):
    root_prim = stage.GetPrimAtPath(CARTER_PRIM_PATH)
    if not root_prim.IsValid():
        return None

    preferred = None
    fallback = None
    for prim in _iter_descendants(root_prim):
        type_name = str(prim.GetTypeName())
        if not any(hint in type_name for hint in _LIDAR_PRIM_TYPE_HINTS):
            continue
        path_str = str(prim.GetPath())
        if any(hint in path_str.lower() for hint in (h.lower() for h in CARTER_LIDAR_SENSOR_HINTS)):
            preferred = path_str
            break
        if fallback is None:
            fallback = path_str
    return preferred or fallback


def setup_carter_scan_tf_graph():
    if not _carter_on_stage():
        return

    domain_id = get_ros_domain_id()
    stage = omni.usd.get_context().get_stage()

    lidar_prim_path = _find_carter_lidar_prim(stage)
    if not lidar_prim_path:
        print(
            f"[cobot3.spot/carter] ❌ {CARTER_PRIM_PATH} 아래 LiDAR sensor prim 없음. "
            "Stage 패널에서 OmniLidar/RtxLidar prim path 확인 후 알려주세요."
        )
        return
    print(f"[cobot3.spot/carter] LiDAR prim 발견: {lidar_prim_path}")

    graph_path = CARTER_SCAN_TF_GRAPH_PATH
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
                    ("CreateLidarRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("LidarHelper", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                    ("PublishLaserTf", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ],
                keys.SET_VALUES: [
                    ("ROS2Context.inputs:domain_id", domain_id),
                    ("CreateLidarRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(lidar_prim_path)]),
                    ("CreateLidarRenderProduct.inputs:enabled", True),
                    ("LidarHelper.inputs:frameId", CARTER_LIDAR_FRAME),
                    ("LidarHelper.inputs:topicName", CARTER_SCAN_TOPIC),
                    ("LidarHelper.inputs:type", "laser_scan"),
                    ("LidarHelper.inputs:useSystemTime", True),

                    ("PublishLaserTf.inputs:topicName", "/tf_static"),
                    ("PublishLaserTf.inputs:staticPublisher", True),
                    ("PublishLaserTf.inputs:parentFrameId", CARTER_BASE_LINK_FRAME),
                    ("PublishLaserTf.inputs:childFrameId", CARTER_LIDAR_FRAME),
                    ("PublishLaserTf.inputs:translation", list(CARTER_LIDAR_TRANSLATION)),
                    ("PublishLaserTf.inputs:rotation", list(CARTER_LIDAR_TF_ROTATION_XYZW)),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "CreateLidarRenderProduct.inputs:execIn"),
                    ("CreateLidarRenderProduct.outputs:execOut", "LidarHelper.inputs:execIn"),
                    ("CreateLidarRenderProduct.outputs:renderProductPath", "LidarHelper.inputs:renderProductPath"),
                    ("ROS2Context.outputs:context", "LidarHelper.inputs:context"),

                    ("OnPlaybackTick.outputs:tick", "PublishLaserTf.inputs:execIn"),
                    ("ROS2Context.outputs:context", "PublishLaserTf.inputs:context"),
                    ("ReadSystemTime.outputs:systemTime", "PublishLaserTf.inputs:timeStamp"),
                ],
            },
        )
        print(f"[cobot3.spot/carter] ✅ scan/TF graph: {CARTER_SCAN_TOPIC} (frame={CARTER_LIDAR_FRAME})")
    except Exception as exc:
        print(f"[cobot3.spot/carter] ❌ scan/TF graph 생성 실패: {exc}")
        traceback.print_exc()
