"""ROS2 OmniGraph setup helpers for the Cobot3 Spot extension.

Important:
    Do NOT import rclpy inside Isaac Sim extension code.
    Isaac Sim 5.x runs its own Python, and ROS Humble apt rclpy is usually
    built for system Python 3.10. Importing it from Isaac's Python can cause
    `No module named rclpy._rclpy_pybind11`.

    This module uses Isaac Sim ROS2 OmniGraph bridge nodes only.
"""

from __future__ import annotations

import traceback

import numpy as np
import omni.graph.core as og
import omni.usd
from pxr import Gf, UsdGeom

from .constants import (
    BASE_LINK_FRAME,
    CAMERA_GRAPH_PATH,
    CAMERA_INFO_TOPIC,
    CMD_VEL_GRAPH_PATH,
    CMD_VEL_TOPIC,
    CMD_VEL_TO_POLICY_ANGULAR_Z_SCALE,
    CMD_VEL_TO_POLICY_LINEAR_X_SCALE,
    CMD_VEL_TO_POLICY_LINEAR_Y_SCALE,
    COLOR_IMAGE_TOPIC,
    DEPTH_IMAGE_TOPIC,
    FRONT_CAM_FRAME,
    FRONT_CAMERA_PRIM_PATH,
    FRONT_CAMERA_RENDER_HEIGHT,
    FRONT_CAMERA_RENDER_WIDTH,
    FRONT_CAMERA_TF_ROTATION_XYZW,
    FRONT_CAMERA_TRANSLATION,
    LIDAR_FRAME,
    LIDAR_PRIM_PATH,
    ODOM_FRAME,
    ODOM_TOPIC,
    SCAN_TOPIC,
    SLAM_GRAPH_PATH,
    SPOT_PRIM_PATH,
)
from .utils import get_ros_domain_id


# ─────────────────────────────────────────────
# /spot_0/cmd_vel subscriber
# ─────────────────────────────────────────────
def setup_cmd_vel_graph(sample):
    """Subscribe /spot_0/cmd_vel and update sample._base_command."""
    if sample is None:
        print("[cobot3.spot] Load Scene 먼저 클릭하세요!")
        return

    graph_path = CMD_VEL_GRAPH_PATH
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
                    ("ROS2SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ],
                keys.SET_VALUES: [
                    ("ROS2Context.inputs:domain_id", domain_id),
                    ("ROS2SubscribeTwist.inputs:topicName", CMD_VEL_TOPIC),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ROS2SubscribeTwist.inputs:execIn"),
                    ("ROS2Context.outputs:context", "ROS2SubscribeTwist.inputs:context"),
                ],
            },
        )

        def ros2_cmd_callback(step_size):
            if sample is None or not sample._physics_ready:
                return
            try:
                twist_node = og.Controller.node(f"{graph_path}/ROS2SubscribeTwist")
                lin_vel = og.Controller.attribute("outputs:linearVelocity", twist_node).get()
                ang_vel = og.Controller.attribute("outputs:angularVelocity", twist_node).get()
                if lin_vel is None or ang_vel is None:
                    return

                lin_x = float(lin_vel[0])
                lin_y = float(lin_vel[1])
                ang_z = float(ang_vel[2])

                # Spot policy command: [forward, lateral, yaw]
                sample._base_command = np.array(
                    [
                        lin_x * CMD_VEL_TO_POLICY_LINEAR_X_SCALE,
                        lin_y * CMD_VEL_TO_POLICY_LINEAR_Y_SCALE,
                        ang_z * CMD_VEL_TO_POLICY_ANGULAR_Z_SCALE,
                    ],
                    dtype=np.float32,
                )
            except Exception:
                pass

        world = sample.get_world()
        if world:
            if world.physics_callback_exists("ros2_cmd_callback"):
                world.remove_physics_callback("ros2_cmd_callback")
            world.add_physics_callback("ros2_cmd_callback", ros2_cmd_callback)

        print("[cobot3.spot] ✅ ROS2 cmd_vel 구독 시작")
        print(f"[cobot3.spot]   domain_id={domain_id}, topic={CMD_VEL_TOPIC}")
    except Exception as exc:
        print(f"[cobot3.spot] ❌ ROS2 cmd_vel graph 생성 실패: {exc}")
        traceback.print_exc()


# ─────────────────────────────────────────────
# Camera publishers
# ─────────────────────────────────────────────
def setup_camera_graph(sample):
    """Publish RGB/depth/camera_info under /spot_0/front_cam/*."""
    if sample is None:
        print("[cobot3.spot] Load Scene 먼저 클릭하세요!")
        return

    graph_path = CAMERA_GRAPH_PATH
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
                    ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("CameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("CameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    # camera_info는 ROS2CameraHelper type='camera_info'가 아니라
                    # 전용 ROS2CameraInfoHelper를 써야 한다.
                    ("CameraInfoHelper", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "RenderProduct.inputs:execIn"),
                    ("RenderProduct.outputs:execOut", "CameraHelperRgb.inputs:execIn"),
                    ("RenderProduct.outputs:execOut", "CameraHelperDepth.inputs:execIn"),
                    ("RenderProduct.outputs:execOut", "CameraInfoHelper.inputs:execIn"),
                    ("ROS2Context.outputs:context", "CameraHelperRgb.inputs:context"),
                    ("ROS2Context.outputs:context", "CameraHelperDepth.inputs:context"),
                    ("ROS2Context.outputs:context", "CameraInfoHelper.inputs:context"),
                    ("RenderProduct.outputs:renderProductPath", "CameraHelperRgb.inputs:renderProductPath"),
                    ("RenderProduct.outputs:renderProductPath", "CameraHelperDepth.inputs:renderProductPath"),
                    ("RenderProduct.outputs:renderProductPath", "CameraInfoHelper.inputs:renderProductPath"),
                ],
                keys.SET_VALUES: [
                    ("ROS2Context.inputs:domain_id", domain_id),
                    ("RenderProduct.inputs:cameraPrim", [FRONT_CAMERA_PRIM_PATH]),
                    ("RenderProduct.inputs:enabled", True),
                    ("RenderProduct.inputs:width", FRONT_CAMERA_RENDER_WIDTH),
                    ("RenderProduct.inputs:height", FRONT_CAMERA_RENDER_HEIGHT),
                    ("CameraHelperRgb.inputs:frameId", FRONT_CAM_FRAME),
                    ("CameraHelperRgb.inputs:topicName", COLOR_IMAGE_TOPIC),
                    ("CameraHelperRgb.inputs:type", "rgb"),
                    ("CameraHelperDepth.inputs:frameId", FRONT_CAM_FRAME),
                    ("CameraHelperDepth.inputs:topicName", DEPTH_IMAGE_TOPIC),
                    ("CameraHelperDepth.inputs:type", "depth"),
                    ("CameraInfoHelper.inputs:frameId", FRONT_CAM_FRAME),
                    ("CameraInfoHelper.inputs:topicName", CAMERA_INFO_TOPIC),
                ],
            },
        )
        print("[cobot3.spot] ✅ Camera graph 생성")
        print(f"[cobot3.spot]   {COLOR_IMAGE_TOPIC}")
        print(f"[cobot3.spot]   {DEPTH_IMAGE_TOPIC}")
        print(f"[cobot3.spot]   {CAMERA_INFO_TOPIC}")
        print(
            f"[cobot3.spot]   frame_id={FRONT_CAM_FRAME}, "
            f"resolution={FRONT_CAMERA_RENDER_WIDTH}x{FRONT_CAMERA_RENDER_HEIGHT}"
        )
    except Exception as exc:
        print(f"[cobot3.spot] ❌ Camera graph 생성 실패: {exc}")
        traceback.print_exc()


# ─────────────────────────────────────────────
# LiDAR helper
# ─────────────────────────────────────────────
LIDAR_TRANSLATION = Gf.Vec3d(0.25, 0.0, 0.35)
LIDAR_ORIENTATION = Gf.Quatd(1.0, 0.0, 0.0, 0.0)
LIDAR_TF_ROTATION_XYZW = [0.0, 0.0, 0.0, 1.0]


def _set_lidar_transform(stage):
    lidar_prim = stage.GetPrimAtPath(LIDAR_PRIM_PATH)
    if not lidar_prim.IsValid():
        return

    xform = UsdGeom.Xformable(lidar_prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(LIDAR_TRANSLATION)
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(LIDAR_ORIENTATION)


def _ensure_lidar_prim():
    import omni.kit.commands

    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(LIDAR_PRIM_PATH).IsValid():
        _set_lidar_transform(stage)
        return

    created = False
    for kwargs in [
        dict(path="spot_lidar", parent="/World/Spot/body", config="Example_Rotary_2D"),
        dict(path=LIDAR_PRIM_PATH, parent=None, config="Example_Rotary_2D"),
        dict(path="spot_lidar", parent="/World/Spot/body", config="Example Rotary 2D"),
        dict(path=LIDAR_PRIM_PATH, parent=None, config="Example Rotary 2D"),
    ]:
        try:
            omni.kit.commands.execute(
                "IsaacSensorCreateRtxLidar",
                translation=LIDAR_TRANSLATION,
                orientation=LIDAR_ORIENTATION,
                visibility=True,
                **kwargs,
            )
            created = True
            break
        except Exception:
            continue

    if not created:
        raise RuntimeError(
            "RTX LiDAR 생성 실패. Create > Sensors > RTX Lidar > NVIDIA > Example Rotary 2D로 "
            f"수동 생성 후 prim을 {LIDAR_PRIM_PATH} 위치로 맞춰줘."
        )

    _set_lidar_transform(stage)


# ─────────────────────────────────────────────
# LiDAR + odom + tf graph
# ─────────────────────────────────────────────
def setup_slam_sensors(sample):
    """Create /spot_0/scan, /spot_0/odom and TF graph without importing rclpy."""
    if sample is None:
        print("[cobot3.spot] Load Scene 먼저 클릭하세요!")
        return

    domain_id = get_ros_domain_id()
    stage = omni.usd.get_context().get_stage()

    try:
        _ensure_lidar_prim()
        print(f"[cobot3.spot] ✅ LiDAR prim 준비: {LIDAR_PRIM_PATH}")
    except Exception as exc:
        print(f"[cobot3.spot] ❌ LiDAR prim 생성 실패: {exc}")
        traceback.print_exc()
        return

    if stage.GetPrimAtPath(SLAM_GRAPH_PATH).IsValid():
        stage.RemovePrim(SLAM_GRAPH_PATH)

    keys = og.Controller.Keys
    try:
        og.Controller.edit(
            {"graph_path": SLAM_GRAPH_PATH, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("ReadSystemTime", "isaacsim.core.nodes.IsaacReadSystemTime"),

                    # LiDAR LaserScan
                    ("CreateLidarRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("LidarHelper", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),

                    # Odometry: Isaac Compute Odometry -> ROS2 Publish Odometry
                    ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                    ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),

                    # TF: odom -> base_link, base_link -> sensors
                    ("PublishOdomTf", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                    ("PublishLidarStaticTf", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                    ("PublishCameraStaticTf", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ],
                keys.CONNECT: [
                    # Execution
                    ("OnPlaybackTick.outputs:tick", "CreateLidarRenderProduct.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
                    ("ComputeOdometry.outputs:execOut", "PublishOdometry.inputs:execIn"),
                    ("ComputeOdometry.outputs:execOut", "PublishOdomTf.inputs:execIn"),
                    ("ComputeOdometry.outputs:execOut", "PublishLidarStaticTf.inputs:execIn"),
                    ("ComputeOdometry.outputs:execOut", "PublishCameraStaticTf.inputs:execIn"),
                    ("CreateLidarRenderProduct.outputs:execOut", "LidarHelper.inputs:execIn"),

                    # ROS2 context
                    ("ROS2Context.outputs:context", "LidarHelper.inputs:context"),
                    ("ROS2Context.outputs:context", "PublishOdometry.inputs:context"),
                    ("ROS2Context.outputs:context", "PublishOdomTf.inputs:context"),
                    ("ROS2Context.outputs:context", "PublishLidarStaticTf.inputs:context"),
                    ("ROS2Context.outputs:context", "PublishCameraStaticTf.inputs:context"),

                    # Timestamps
                    ("ReadSystemTime.outputs:systemTime", "PublishOdometry.inputs:timeStamp"),
                    ("ReadSystemTime.outputs:systemTime", "PublishOdomTf.inputs:timeStamp"),
                    ("ReadSystemTime.outputs:systemTime", "PublishLidarStaticTf.inputs:timeStamp"),
                    ("ReadSystemTime.outputs:systemTime", "PublishCameraStaticTf.inputs:timeStamp"),

                    # LiDAR render product
                    ("CreateLidarRenderProduct.outputs:renderProductPath", "LidarHelper.inputs:renderProductPath"),

                    # Odometry values
                    ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
                    ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
                    ("ComputeOdometry.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
                    ("ComputeOdometry.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),

                    # Dynamic TF odom -> base_link
                    ("ComputeOdometry.outputs:position", "PublishOdomTf.inputs:translation"),
                    ("ComputeOdometry.outputs:orientation", "PublishOdomTf.inputs:rotation"),
                ],
                keys.SET_VALUES: [
                    ("ROS2Context.inputs:domain_id", domain_id),

                    # LaserScan
                    ("CreateLidarRenderProduct.inputs:cameraPrim", [LIDAR_PRIM_PATH]),
                    ("CreateLidarRenderProduct.inputs:enabled", True),
                    ("LidarHelper.inputs:frameId", LIDAR_FRAME),
                    ("LidarHelper.inputs:topicName", SCAN_TOPIC),
                    ("LidarHelper.inputs:type", "laser_scan"),
                    ("LidarHelper.inputs:useSystemTime", True),

                    # Odom
                    ("ComputeOdometry.inputs:chassisPrim", [SPOT_PRIM_PATH]),
                    ("PublishOdometry.inputs:topicName", ODOM_TOPIC),
                    ("PublishOdometry.inputs:odomFrameId", ODOM_FRAME),
                    ("PublishOdometry.inputs:chassisFrameId", BASE_LINK_FRAME),
                    ("PublishOdometry.inputs:publishRawVelocities", True),

                    # /tf: odom -> spot_0/base_link
                    ("PublishOdomTf.inputs:topicName", "/tf"),
                    ("PublishOdomTf.inputs:parentFrameId", ODOM_FRAME),
                    ("PublishOdomTf.inputs:childFrameId", BASE_LINK_FRAME),

                    # /tf_static: spot_0/base_link -> spot_0/lidar_link
                    ("PublishLidarStaticTf.inputs:topicName", "/tf_static"),
                    ("PublishLidarStaticTf.inputs:staticPublisher", True),
                    ("PublishLidarStaticTf.inputs:parentFrameId", BASE_LINK_FRAME),
                    ("PublishLidarStaticTf.inputs:childFrameId", LIDAR_FRAME),
                    ("PublishLidarStaticTf.inputs:translation", list(LIDAR_TRANSLATION)),
                    ("PublishLidarStaticTf.inputs:rotation", LIDAR_TF_ROTATION_XYZW),

                    # /tf_static: spot_0/base_link -> spot_0/front_cam_link
                    ("PublishCameraStaticTf.inputs:topicName", "/tf_static"),
                    ("PublishCameraStaticTf.inputs:staticPublisher", True),
                    ("PublishCameraStaticTf.inputs:parentFrameId", BASE_LINK_FRAME),
                    ("PublishCameraStaticTf.inputs:childFrameId", FRONT_CAM_FRAME),
                    ("PublishCameraStaticTf.inputs:translation", list(FRONT_CAMERA_TRANSLATION)),
                    ("PublishCameraStaticTf.inputs:rotation", list(FRONT_CAMERA_TF_ROTATION_XYZW)),
                ],
            },
        )

        print("[cobot3.spot] ✅ SLAM sensors graph 생성 완료 - no rclpy")
        print(f"[cobot3.spot]   {SCAN_TOPIC}: frame_id={LIDAR_FRAME}")
        print(f"[cobot3.spot]   {ODOM_TOPIC}: frame_id={ODOM_FRAME}, child_frame_id={BASE_LINK_FRAME}")
        print(f"[cobot3.spot]   /tf: {ODOM_FRAME} -> {BASE_LINK_FRAME}")
        print(f"[cobot3.spot]   /tf_static: {BASE_LINK_FRAME} -> {LIDAR_FRAME}")
        print(f"[cobot3.spot]   /tf_static: {BASE_LINK_FRAME} -> {FRONT_CAM_FRAME}")
    except Exception as exc:
        print(f"[cobot3.spot] ❌ SLAM sensors graph 생성 실패: {exc}")
        traceback.print_exc()
