"""Scene and ANYmal C policy runtime for the Cobot3 ANYmal extension."""

from __future__ import annotations

import numpy as np
import omni
import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom

from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.policy.examples.robots import AnymalFlatTerrainPolicy

# ── Rough terrain policy (optional upgrade) ────────────────────────────────
# Isaac Sim 5.x ships only AnymalFlatTerrainPolicy (flat warehouse).
# To use the rough terrain policy from Isaac Lab (Isaac-Velocity-Rough-Anymal-C-v0):
#   1. Train or download the policy ONNX/PT from Isaac Lab
#   2. Implement AnymalRoughTerrainPolicy subclassing PolicyController,
#      with the 235-dim observation that includes height scan samples
#   3. Swap the import and class below: AnymalRoughTerrainPolicy(...)
# For Nav2 integration the cmd_vel interface is identical regardless of policy.
# ──────────────────────────────────────────────────────────────────────────

from .constants import (
    ANYMAL_PRIM_PATH,
    ANYMAL_SPAWN_HEIGHT,
    FRONT_CAMERA_PRIM_PATH,
)


class AnymalNavScene(BaseSample):
    """ANYmal C simulation runtime for Nav2 integration.

    Owns the Isaac world, ANYmal C RL policy, and physics callbacks.
    ROS graph creation lives in ros_graphs.py; extension.py stays thin.
    """

    def __init__(self):
        super().__init__()
        self._world_settings["stage_units_in_meters"] = 1.0
        # ANYmal policy needs faster physics than rendering for stability.
        self._world_settings["physics_dt"] = 1.0 / 500.0
        self._world_settings["rendering_dt"] = 10.0 / 500.0

        # ANYmal policy command: [forward, lateral, yaw]
        self._base_command = np.zeros(3, dtype=np.float32)
        self._physics_ready = False
        self.anymal = None
        self._event_timer_callback = None

    def setup_scene(self):
        self._world.scene.add_default_ground_plane(
            z_position=0,
            name="default_ground_plane",
            prim_path="/World/defaultGroundPlane",
            static_friction=0.5,
            dynamic_friction=0.5,
            restitution=0.01,
        )

        assets_root = get_assets_root_path()
        # warehouse.usd matches the carter_warehouse_navigation map origin/layout.
        # full_warehouse.usd has a different layout and causes AMCL mismatch.
        add_reference_to_stage(
            usd_path=assets_root + "/Isaac/Environments/Simple_Warehouse/warehouse.usd",
            prim_path="/World/Warehouse",
        )
        print("[cobot3.anymal] Warehouse 로드 완료")

        self.anymal = AnymalFlatTerrainPolicy(
            prim_path=ANYMAL_PRIM_PATH,
            name="Anymal",
            position=np.array([0, 0, ANYMAL_SPAWN_HEIGHT]),
        )
        print("[cobot3.anymal] ANYmal C RL 정책 로드 완료")

        self._add_front_camera()

        timeline = omni.timeline.get_timeline_interface()
        self._event_timer_callback = timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PLAY), self._on_timeline_play
        )

    def _add_front_camera(self):
        """Add a front camera prim for RGB/depth publishing."""
        import omni.kit.commands

        stage = omni.usd.get_context().get_stage()
        if not stage.GetPrimAtPath(FRONT_CAMERA_PRIM_PATH).IsValid():
            # Ensure parent prim exists first
            base_path = "/World/Anymal/base"
            if not stage.GetPrimAtPath(base_path).IsValid():
                print(f"[cobot3.anymal] ⚠️ base prim {base_path} not found; skipping camera creation")
                return
            omni.kit.commands.execute(
                "CreatePrimWithDefaultXform",
                prim_type="Camera",
                prim_path=FRONT_CAMERA_PRIM_PATH,
            )
            cam_prim = stage.GetPrimAtPath(FRONT_CAMERA_PRIM_PATH)
            xform = UsdGeom.Xformable(cam_prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(Gf.Vec3d(0.4, 0.0, 0.15))
            xform.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            print("[cobot3.anymal] 전방 카메라 추가 완료")
        else:
            print("[cobot3.anymal] 전방 카메라 이미 존재")

    async def setup_post_load(self):
        self._physics_ready = False
        if not self.get_world().physics_callback_exists("anymal_physics_step"):
            self.get_world().add_physics_callback("anymal_physics_step", self._on_physics_step)
        await self.get_world().play_async()
        print("[cobot3.anymal] ✅ 로드 완료!")
        print("[cobot3.anymal] 순서: Setup ROS2 → Setup Camera → Setup LiDAR/SLAM")

    async def setup_post_reset(self):
        self._physics_ready = False
        self._base_command = np.zeros(3, dtype=np.float32)
        await self._world.play_async()
        if not self.get_world().physics_callback_exists("anymal_physics_step"):
            self.get_world().add_physics_callback("anymal_physics_step", self._on_physics_step)

    def _on_physics_step(self, step_size):
        if self._physics_ready:
            try:
                pos = self.anymal.robot.get_world_pose()[0]
                if pos[2] < 0.15:
                    print("[cobot3.anymal] 넘어짐 감지! 명령 차단 중")
                    self._physics_ready = False
                    self._base_command = np.zeros(3, dtype=np.float32)
                    return
            except Exception:
                pass

            self.anymal.forward(step_size, self._base_command)
        else:
            self._physics_ready = True
            self.anymal.initialize()
            self.anymal.post_reset()
            self.anymal.robot.set_joints_default_state(self.anymal.default_pos)

    def _on_timeline_play(self, event):
        self._physics_ready = False
        if not self.get_world().physics_callback_exists("anymal_physics_step"):
            self.get_world().add_physics_callback("anymal_physics_step", self._on_physics_step)

    def reset_command(self):
        self._base_command = np.zeros(3, dtype=np.float32)

    def world_cleanup(self):
        self._event_timer_callback = None
        world = self.get_world()
        for cb_name in ["anymal_physics_step", "ros2_cmd_callback"]:
            if world.physics_callback_exists(cb_name):
                world.remove_physics_callback(cb_name)
