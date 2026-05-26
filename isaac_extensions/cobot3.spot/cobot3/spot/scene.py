"""Scene and Spot policy runtime for the Cobot3 Spot extension."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import omni
import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom, UsdLux

from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy

from .constants import (
    CAMERA_SPECS,
    HEADLIGHT_COLOR_RGB,
    HEADLIGHT_CONE_ANGLE_DEG,
    HEADLIGHT_INTENSITY,
    HEADLIGHT_PRIM_PATH,
    HEADLIGHT_RADIUS,
    HEADLIGHT_ROTATION_XYZ_DEG,
    HEADLIGHT_TRANSLATION,
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

    Optional secondary robots (Carter, Jackal) plug in via the
    ``_secondary_spawn`` callback: the panel module sets it to a free
    function ``(sample) -> None`` before ``load_world_async()`` runs.
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

        # Optional secondary-robot spawn callback. Set externally by a
        # carter/jackal panel handler BEFORE load_world_async() kicks off
        # setup_scene(). Signature: callable(self) -> None. Leave as None to
        # spawn Spot alone.
        self._secondary_spawn = None

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
        self._add_headlight()

        if self._secondary_spawn is not None:
            self._secondary_spawn(self)

        timeline = omni.timeline.get_timeline_interface()
        self._event_timer_callback = timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PLAY), self._on_timeline_play
        )

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

    def _add_headlight(self):
        """Add a forward-facing spotlight under /World/Spot/body so it moves with Spot.

        Uses SphereLight + ShapingAPI cone (USD has no standalone SpotLight prim type).
        Cone opens along the light's local -Z axis; we rotate (0, 90, 0) to redirect it
        to the body's +X (forward) direction.
        """
        import omni.kit.commands

        stage = omni.usd.get_context().get_stage()
        created = False
        if not stage.GetPrimAtPath(HEADLIGHT_PRIM_PATH).IsValid():
            omni.kit.commands.execute(
                "CreatePrimWithDefaultXform",
                prim_type="SphereLight",
                prim_path=HEADLIGHT_PRIM_PATH,
            )
            created = True

        light_prim = stage.GetPrimAtPath(HEADLIGHT_PRIM_PATH)

        # CreatePrimWithDefaultXform sometimes sets visibility=invisible —
        # explicitly inherit so the light actually renders.
        UsdGeom.Imageable(light_prim).MakeVisible()

        xform = UsdGeom.Xformable(light_prim)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(*HEADLIGHT_TRANSLATION))
        xform.AddRotateXYZOp().Set(Gf.Vec3f(*HEADLIGHT_ROTATION_XYZ_DEG))

        sphere_light = UsdLux.SphereLight(light_prim)
        sphere_light.CreateIntensityAttr().Set(HEADLIGHT_INTENSITY)
        sphere_light.CreateRadiusAttr().Set(HEADLIGHT_RADIUS)
        sphere_light.CreateColorAttr().Set(Gf.Vec3f(*HEADLIGHT_COLOR_RGB))

        shaping = UsdLux.ShapingAPI.Apply(light_prim)
        shaping.CreateShapingConeAngleAttr().Set(HEADLIGHT_CONE_ANGLE_DEG)
        shaping.CreateShapingConeSoftnessAttr().Set(0.2)
        shaping.CreateShapingFocusAttr().Set(0.0)

        action = "추가" if created else "pose/속성 업데이트"
        print(f"[cobot3.spot] 헤드라이트 {action} 완료 ({HEADLIGHT_PRIM_PATH})")

    def _disable_other_lights(self):
        """Set intensity=0 on every light in the scene except Spot's headlight.

        This produces the dark-warehouse + flashlight-cone effect. The warehouse
        USD ships with ceiling lights and Isaac defaults to a DomeLight, both of
        which overwhelm the headlight. We don't delete them — just zero them out
        so re-enabling later is trivial (set intensity back to original).
        """
        stage = omni.usd.get_context().get_stage()
        disabled = 0
        for prim in stage.Traverse():
            type_name = str(prim.GetTypeName())
            if "Light" not in type_name:
                continue
            if str(prim.GetPath()) == HEADLIGHT_PRIM_PATH:
                continue
            light = UsdLux.LightAPI(prim)
            intensity_attr = light.GetIntensityAttr()
            if intensity_attr and intensity_attr.IsValid():
                intensity_attr.Set(0.0)
                disabled += 1
        print(f"[cobot3.spot] 어둠 모드: 기존 조명 {disabled}개 OFF")

    async def setup_post_load(self):
        self._physics_ready = False
        if not self.get_world().physics_callback_exists("spot_physics_step"):
            self.get_world().add_physics_callback("spot_physics_step", self._on_physics_step)
        await self.get_world().play_async()
        # warehouse 자체 천장 조명/DomeLight를 모두 꺼서 헤드라이트만 보이게.
        # play_async 이후에 호출 — 그래야 USD reference 완전 로드 후 traversal.
        self._disable_other_lights()
        print("[cobot3.spot] ✅ 로드 완료!")
        # Default viewport sits at world origin — frame it on Spot so the
        # user immediately sees the robot instead of empty warehouse.
        self._frame_viewport_on_spot()
        print("[cobot3.spot] 순서: Setup ROS2 → Setup Camera → Setup LiDAR/SLAM")

    def _frame_viewport_on_spot(self):
        try:
            from omni.kit.viewport.utility import (
                get_active_viewport,
                frame_viewport_selection,
            )
            omni.usd.get_context().get_selection().set_selected_prim_paths(
                [SPOT_PRIM_PATH], True
            )
            viewport = get_active_viewport()
            if viewport is not None:
                frame_viewport_selection(viewport)
            # Clear selection so subsequent Property panel use isn't sticky.
            omni.usd.get_context().get_selection().set_selected_prim_paths([], True)
            print(f"[cobot3.spot] Viewport framed on {SPOT_PRIM_PATH}")
        except Exception as exc:
            print(f"[cobot3.spot] viewport frame skipped: {exc}")

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
