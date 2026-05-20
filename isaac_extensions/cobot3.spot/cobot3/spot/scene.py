"""Scene and Spot policy runtime for the Cobot3 Spot extension."""

from __future__ import annotations

import numpy as np
import omni
import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom

from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy

from .constants import FRONT_CAMERA_PRIM_PATH, SPOT_BODY_PRIM_PATH, SPOT_PRIM_PATH


class SpotFireRescue(BaseSample):
    """Spot fire-rescue simulation runtime.

    This class owns the Isaac world, Spot RL policy, and physics callbacks.
    ROS graph creation lives in ros_graphs.py so extension.py can stay small.
    """

    def __init__(self):
        super().__init__()
        self._world_settings["stage_units_in_meters"] = 1.0
        self._world_settings["physics_dt"] = 1.0 / 500.0
        self._world_settings["rendering_dt"] = 10.0 / 500.0

        # Spot policy command: [forward, lateral, yaw]
        self._base_command = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self._physics_ready = False
        self.spot = None
        self._event_timer_callback = None

        # Optional ROS objects created by ros_graphs.setup_slam_sensors().
        self._slam_ros_node = None
        self._slam_odom_pub = None
        self._slam_tf_pub = None
        self._slam_static_tf_pub = None
        self._slam_static_tf_count = 0

        # Reserved for later joint-level stand/sit motions.
        self._control_mode = "policy"
        self._joint_motion = None

    def setup_scene(self):
        self._world.scene.add_default_ground_plane(
            z_position=0,
            name="default_ground_plane",
            prim_path="/World/defaultGroundPlane",
            static_friction=0.2,
            dynamic_friction=0.2,
            restitution=0.01,
        )

        add_reference_to_stage(
            usd_path="/home/katze/dev_ws/cobot3/isaac_extensions/cobot3.spot/usd/sujung_warehouse.usd",
            prim_path="/World/Warehouse",
        )
        print("[cobot3.spot] Warehouse 로드 완료")

        self.spot = SpotFlatTerrainPolicy(
            prim_path=SPOT_PRIM_PATH,
            name="Spot",
            position=np.array([0, 0, 0.8]),
        )
        print("[cobot3.spot] Spot RL 정책 로드 완료")

        self._add_front_camera()

        timeline = omni.timeline.get_timeline_interface()
        self._event_timer_callback = timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PLAY), self._on_timeline_play
        )

    def _add_front_camera(self):
        """Add a front camera prim used by YOLO/depth localization."""
        import omni.kit.commands

        stage = omni.usd.get_context().get_stage()
        if not stage.GetPrimAtPath(FRONT_CAMERA_PRIM_PATH).IsValid():
            omni.kit.commands.execute(
                "CreatePrimWithDefaultXform",
                prim_type="Camera",
                prim_path=FRONT_CAMERA_PRIM_PATH,
            )
            cam_prim = stage.GetPrimAtPath(FRONT_CAMERA_PRIM_PATH)
            xform = UsdGeom.Xformable(cam_prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(Gf.Vec3d(0.5, 0.0, 0.3))
            xform.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            print("[cobot3.spot] 전방 카메라 추가 완료")
        else:
            print("[cobot3.spot] 전방 카메라 이미 존재")

    async def setup_post_load(self):
        self._physics_ready = False
        if not self.get_world().physics_callback_exists("spot_physics_step"):
            self.get_world().add_physics_callback("spot_physics_step", self._on_physics_step)
        await self.get_world().play_async()
        print("[cobot3.spot] ✅ 로드 완료!")
        print("[cobot3.spot] 순서: Setup ROS2 → Setup Camera → Setup LiDAR/SLAM")

    async def setup_post_reset(self):
        self._physics_ready = False
        self._base_command = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        await self._world.play_async()
        if not self.get_world().physics_callback_exists("spot_physics_step"):
            self.get_world().add_physics_callback("spot_physics_step", self._on_physics_step)

    def _on_physics_step(self, step_size):
        if self._physics_ready:
            try:
                pos = self.spot.robot.get_world_pose()[0]
                if pos[2] < 0.2:
                    print("[cobot3.spot] 넘어짐 감지! 자동 복구 필요")
                    self._physics_ready = False
                    self._base_command = np.array([0.0, 0.0, 0.0], dtype=np.float32)
                    return
            except Exception:
                pass

            # If joint-level stand/sit motions are added later, branch here.
            # For now, RL policy owns Spot locomotion.
            self.spot.forward(step_size, self._base_command)
        else:
            self._physics_ready = True
            self.spot.initialize()
            self.spot.post_reset()
            self.spot.robot.set_joints_default_state(self.spot.default_pos)

    def _on_timeline_play(self, event):
        self._physics_ready = False
        if not self.get_world().physics_callback_exists("spot_physics_step"):
            self.get_world().add_physics_callback("spot_physics_step", self._on_physics_step)

    def reset_command(self):
        self._base_command = np.array([0.0, 0.0, 0.0], dtype=np.float32)

    def world_cleanup(self):
        self._event_timer_callback = None
        world = self.get_world()
        for cb_name in ["spot_physics_step", "ros2_cmd_callback", "spot_slam_pub_callback"]:
            if world.physics_callback_exists(cb_name):
                world.remove_physics_callback(cb_name)
