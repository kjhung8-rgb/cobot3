"""ROS2 OmniGraph/rclpy setup helpers for the Cobot3 Spot extension."""

from __future__ import annotations

import traceback

import numpy as np
import omni.usd
import omni.graph.core as og
from pxr import Gf

from .constants import (
    BASE_LINK_FRAME,
    CAMERA_GRAPH_PATH,
    CAMERA_INFO_TOPIC,
    CMD_VEL_GRAPH_PATH,
    CMD_VEL_TOPIC,
    COLOR_IMAGE_TOPIC,
    DEPTH_IMAGE_TOPIC,
    FRONT_CAM_FRAME,
    FRONT_CAMERA_PRIM_PATH,
    LIDAR_FRAME,
    LIDAR_PRIM_PATH,
    ODOM_FRAME,
    ODOM_TOPIC,
    SCAN_TOPIC,
    SLAM_GRAPH_PATH,
)
from .utils import get_ros_domain_id


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
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    ("ros2_context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("ros2_subscribe_twist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ],
                keys.SET_VALUES: [
                    ("ros2_context.inputs:domain_id", domain_id),
                    ("ros2_subscribe_twist.inputs:topicName", CMD_VEL_TOPIC),
                ],
                keys.CONNECT: [
                    ("on_playback_tick.outputs:tick", "ros2_subscribe_twist.inputs:execIn"),
                    ("ros2_context.outputs:context", "ros2_subscribe_twist.inputs:context"),
                ],
            },
        )

        def ros2_cmd_callback(step_size):
            if sample is None or not sample._physics_ready:
                return
            try:
                twist_node = og.Controller.node(f"{graph_path}/ros2_subscribe_twist")
                lin_vel = og.Controller.attribute("outputs:linearVelocity", twist_node).get()
                ang_vel = og.Controller.attribute("outputs:angularVelocity", twist_node).get()
                if lin_vel is None or ang_vel is None:
                    return

                lin_x = float(lin_vel[0])
                lin_y = float(lin_vel[1])
                ang_z = float(ang_vel[2])

                # Spot policy command: [forward, lateral, yaw]
                sample._base_command = np.array([lin_x * 2.0, lin_y * 2.0, ang_z * 2.0], dtype=np.float32)
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
                    ("CameraHelperInfo", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "RenderProduct.inputs:execIn"),
                    ("RenderProduct.outputs:execOut", "CameraHelperRgb.inputs:execIn"),
                    ("RenderProduct.outputs:execOut", "CameraHelperDepth.inputs:execIn"),
                    ("RenderProduct.outputs:execOut", "CameraHelperInfo.inputs:execIn"),
                    ("ROS2Context.outputs:context", "CameraHelperRgb.inputs:context"),
                    ("ROS2Context.outputs:context", "CameraHelperDepth.inputs:context"),
                    ("ROS2Context.outputs:context", "CameraHelperInfo.inputs:context"),
                    ("RenderProduct.outputs:renderProductPath", "CameraHelperRgb.inputs:renderProductPath"),
                    ("RenderProduct.outputs:renderProductPath", "CameraHelperDepth.inputs:renderProductPath"),
                    ("RenderProduct.outputs:renderProductPath", "CameraHelperInfo.inputs:renderProductPath"),
                ],
                keys.SET_VALUES: [
                    ("ROS2Context.inputs:domain_id", domain_id),
                    ("RenderProduct.inputs:cameraPrim", [FRONT_CAMERA_PRIM_PATH]),
                    ("RenderProduct.inputs:enabled", True),
                    ("CameraHelperRgb.inputs:frameId", FRONT_CAM_FRAME),
                    ("CameraHelperRgb.inputs:topicName", COLOR_IMAGE_TOPIC),
                    ("CameraHelperRgb.inputs:type", "rgb"),
                    ("CameraHelperDepth.inputs:frameId", FRONT_CAM_FRAME),
                    ("CameraHelperDepth.inputs:topicName", DEPTH_IMAGE_TOPIC),
                    ("CameraHelperDepth.inputs:type", "depth"),
                    ("CameraHelperInfo.inputs:frameId", FRONT_CAM_FRAME),
                    ("CameraHelperInfo.inputs:topicName", CAMERA_INFO_TOPIC),
                    ("CameraHelperInfo.inputs:type", "camera_info"),
                ],
            },
        )
        print("[cobot3.spot] ✅ Camera graph 생성")
        print(f"[cobot3.spot]   {COLOR_IMAGE_TOPIC}")
        print(f"[cobot3.spot]   {DEPTH_IMAGE_TOPIC}")
        print(f"[cobot3.spot]   {CAMERA_INFO_TOPIC}")
        print(f"[cobot3.spot]   frame_id={FRONT_CAM_FRAME}")
    except Exception as exc:
        print(f"[cobot3.spot] ❌ Camera graph 생성 실패: {exc}")


def _ensure_lidar_prim():
    import omni.kit.commands

    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(LIDAR_PRIM_PATH).IsValid():
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
                translation=Gf.Vec3d(0.25, 0.0, 0.35),
                orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
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


def setup_slam_sensors(sample):
    """Create LaserScan graph and publish /spot_0/odom + TF tree."""
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
                    ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("LidarHelper", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                    ("CreateRenderProduct.outputs:execOut", "LidarHelper.inputs:execIn"),
                    ("ROS2Context.outputs:context", "LidarHelper.inputs:context"),
                    ("CreateRenderProduct.outputs:renderProductPath", "LidarHelper.inputs:renderProductPath"),
                ],
                keys.SET_VALUES: [
                    ("ROS2Context.inputs:domain_id", domain_id),
                    ("CreateRenderProduct.inputs:cameraPrim", [LIDAR_PRIM_PATH]),
                    ("CreateRenderProduct.inputs:enabled", True),
                    ("LidarHelper.inputs:frameId", LIDAR_FRAME),
                    ("LidarHelper.inputs:topicName", SCAN_TOPIC),
                    ("LidarHelper.inputs:type", "laser_scan"),
                    ("LidarHelper.inputs:useSystemTime", True),
                ],
            },
        )
        print(f"[cobot3.spot] ✅ LiDAR graph 생성: {SCAN_TOPIC}, frame_id={LIDAR_FRAME}")
    except Exception as exc:
        print(f"[cobot3.spot] ❌ LiDAR graph 생성 실패: {exc}")
        traceback.print_exc()
        return

    try:
        import rclpy
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

        if not rclpy.ok():
            rclpy.init(args=None)

        if sample._slam_ros_node is None:
            sample._slam_ros_node = rclpy.create_node("spot0_isaac_slam_bridge")
            sample._slam_odom_pub = sample._slam_ros_node.create_publisher(Odometry, ODOM_TOPIC, 10)
            sample._slam_tf_pub = TransformBroadcaster(sample._slam_ros_node)
            sample._slam_static_tf_pub = StaticTransformBroadcaster(sample._slam_ros_node)

        node = sample._slam_ros_node

        def send_static_tf():
            now = node.get_clock().now().to_msg()

            lidar_tf = TransformStamped()
            lidar_tf.header.stamp = now
            lidar_tf.header.frame_id = BASE_LINK_FRAME
            lidar_tf.child_frame_id = LIDAR_FRAME
            lidar_tf.transform.translation.x = 0.25
            lidar_tf.transform.translation.y = 0.0
            lidar_tf.transform.translation.z = 0.35
            lidar_tf.transform.rotation.w = 1.0

            cam_tf = TransformStamped()
            cam_tf.header.stamp = now
            cam_tf.header.frame_id = BASE_LINK_FRAME
            cam_tf.child_frame_id = FRONT_CAM_FRAME
            cam_tf.transform.translation.x = 0.5
            cam_tf.transform.translation.y = 0.0
            cam_tf.transform.translation.z = 0.3
            cam_tf.transform.rotation.w = 1.0

            sample._slam_static_tf_pub.sendTransform([lidar_tf, cam_tf])

        send_static_tf()

        def slam_pub_callback(step_size):
            if sample is None or sample.spot is None or not sample._physics_ready:
                return

            try:
                pos, quat = sample.spot.robot.get_world_pose()
                qw, qx, qy, qz = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
                now_msg = node.get_clock().now().to_msg()

                odom = Odometry()
                odom.header.stamp = now_msg
                odom.header.frame_id = ODOM_FRAME
                odom.child_frame_id = BASE_LINK_FRAME
                odom.pose.pose.position.x = float(pos[0])
                odom.pose.pose.position.y = float(pos[1])
                odom.pose.pose.position.z = float(pos[2])
                odom.pose.pose.orientation.x = qx
                odom.pose.pose.orientation.y = qy
                odom.pose.pose.orientation.z = qz
                odom.pose.pose.orientation.w = qw
                odom.twist.twist.linear.x = float(sample._base_command[0])
                odom.twist.twist.linear.y = float(sample._base_command[1])
                odom.twist.twist.angular.z = float(sample._base_command[2])
                sample._slam_odom_pub.publish(odom)

                tf = TransformStamped()
                tf.header.stamp = now_msg
                tf.header.frame_id = ODOM_FRAME
                tf.child_frame_id = BASE_LINK_FRAME
                tf.transform.translation.x = float(pos[0])
                tf.transform.translation.y = float(pos[1])
                tf.transform.translation.z = float(pos[2])
                tf.transform.rotation.x = qx
                tf.transform.rotation.y = qy
                tf.transform.rotation.z = qz
                tf.transform.rotation.w = qw
                sample._slam_tf_pub.sendTransform(tf)

                sample._slam_static_tf_count += 1
                if sample._slam_static_tf_count % 250 == 0:
                    send_static_tf()

                rclpy.spin_once(node, timeout_sec=0.0)
            except Exception as exc:
                print(f"[cobot3.spot] SLAM odom/tf publish 실패: {exc}")

        world = sample.get_world()
        if world:
            if world.physics_callback_exists("spot_slam_pub_callback"):
                world.remove_physics_callback("spot_slam_pub_callback")
            world.add_physics_callback("spot_slam_pub_callback", slam_pub_callback)

        print("[cobot3.spot] ✅ Odometry/TF publish 시작")
        print(f"[cobot3.spot]   {ODOM_TOPIC}: frame_id={ODOM_FRAME}, child_frame_id={BASE_LINK_FRAME}")
        print(f"[cobot3.spot]   /tf: {ODOM_FRAME} -> {BASE_LINK_FRAME}")
        print(f"[cobot3.spot]   /tf_static: {BASE_LINK_FRAME} -> {LIDAR_FRAME}")
        print(f"[cobot3.spot]   /tf_static: {BASE_LINK_FRAME} -> {FRONT_CAM_FRAME}")
    except Exception as exc:
        print(f"[cobot3.spot] ❌ Odometry/TF publisher 생성 실패: {exc}")
        traceback.print_exc()
