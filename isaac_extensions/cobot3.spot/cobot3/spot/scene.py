"""Scene and Spot policy runtime for the Cobot3 Spot extension."""

from __future__ import annotations

import math
from pathlib import Path

import carb
import numpy as np
import omni
import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom

from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy
from isaacsim.storage.native import get_assets_root_path

from .constants import (
    CAMERA_SPECS,
    CARTER_PRIM_PATH,
    CARTER_SPAWN_POSITION,
    CARTER_SPAWN_YAW_DEG,
    CARTER_USD_NUCLEUS_PATH,
    SPOT_PRIM_PATH,
    SPOT_SPAWN_POSITION,
    SPOT_SPAWN_YAW_DEG,
)


SPOT_EXTENSION_DIR = Path(__file__).resolve().parents[2]
WAREHOUSE_USD_PATH = SPOT_EXTENSION_DIR / "usd" / "warehouse_small.usd"


def _yaw_to_quat_wxyz(yaw_deg):
    half_yaw = math.radians(yaw_deg) * 0.5
    return np.array([math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)], dtype=np.float64)


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
            usd_path=str(WAREHOUSE_USD_PATH),
            prim_path="/World/Warehouse",
        )
        print("[cobot3.spot] Warehouse 로드 완료")

        # Spawn at the warehouse entrance facing the interior (-Y direction).
        # Z is set well above floor so physics
        # drops Spot onto the floor cleanly.
        self.spot = SpotFlatTerrainPolicy(
            prim_path=SPOT_PRIM_PATH,
            name="Spot",
            position=np.array(SPOT_SPAWN_POSITION, dtype=np.float64),
        )
        print("[cobot3.spot] Spot RL 정책 로드 완료")

        self._add_cameras()
        self._add_carter()

        timeline = omni.timeline.get_timeline_interface()
        self._event_timer_callback = timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PLAY), self._on_timeline_play
        )

    def _add_carter(self):
        """Spawn Nova Carter co-located with Spot. cobot3.carter [TEST]
        extension owns all ROS graph setup against /World/Carter."""
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("[cobot3.spot] Carter co-spawn: Isaac assets root 못 찾음")
            return

        carter_usd = assets_root_path + CARTER_USD_NUCLEUS_PATH
        add_reference_to_stage(usd_path=carter_usd, prim_path=CARTER_PRIM_PATH)

        self.carter = SingleArticulation(
            prim_path=CARTER_PRIM_PATH,
            name="Carter",
            position=np.array(CARTER_SPAWN_POSITION, dtype=np.float64),
            orientation=_yaw_to_quat_wxyz(CARTER_SPAWN_YAW_DEG),
        )
        print(f"[cobot3.spot] Carter co-spawn: {carter_usd}")

    def _add_cameras(self):
        """Add camera prims used by YOLO/depth localization."""
        import omni.kit.commands

        stage = omni.usd.get_context().get_stage()
        for spec in CAMERA_SPECS:
            prim_path = spec["prim_path"]
            created = False
            if not stage.GetPrimAtPath(prim_path).IsValid():
                omni.kit.commands.execute(
                    "CreatePrimWithDefaultXform",
                    prim_type="Camera",
                    prim_path=prim_path,
                )
                created = True

            cam_prim = stage.GetPrimAtPath(prim_path)
            xform = UsdGeom.Xformable(cam_prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(Gf.Vec3d(*spec["translation"]))
            xform.AddRotateXYZOp().Set(Gf.Vec3f(*spec["usd_rotation_xyz_deg"]))

            action = "추가" if created else "pose 업데이트"
            print(f"[cobot3.spot] {spec['label']} 카메라 {action} 완료")

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
            self.spot.robot.set_world_pose(
                position=np.array(SPOT_SPAWN_POSITION, dtype=np.float64),
                orientation=_yaw_to_quat_wxyz(SPOT_SPAWN_YAW_DEG),
            )
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
