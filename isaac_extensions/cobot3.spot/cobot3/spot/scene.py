"""Scene and Spot policy runtime for the Cobot3 Spot extension."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import omni
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdLux

from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy

from .constants import (
    CAMERA_FOCAL_LENGTH_MM,
    CAMERA_HORIZONTAL_APERTURE_MM,
    CAMERA_SPECS,
    CAMERA_VERTICAL_APERTURE_MM,
    CEILING_LIGHT_COLOR_RGB,
    CEILING_LIGHT_INTENSITY,
    CEILING_LIGHT_POSITIONS,
    CEILING_LIGHT_RADIUS,
    CEILING_LIGHTS_GROUP_PATH,
    FIRE_LIGHT_COLOR_RGB,
    FIRE_LIGHT_INTENSITY,
    FIRE_LIGHT_POSITIONS,
    FIRE_LIGHT_RADIUS,
    FIRE_LIGHTS_GROUP_PATH,
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
WAREHOUSE_USD_PATH = SPOT_EXTENSION_DIR / "usd" / "g8.usd"
# Pre-tuned scene snapshot was attempted (g8_1.usd) but save_as_stage overwrite
# corrupted it; reverting to code-based tuning which is git-tracked + reproducible.
TUNED_SCENE_USD_PATH = None


def _yaw_to_quat_wxyz(yaw_deg):
    half_yaw = math.radians(yaw_deg) * 0.5
    return np.array([math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)], dtype=np.float64)


class SpotFireRescue(BaseSample):
    """Spot fire-rescue simulation runtime.

    This class owns the Isaac world, Spot RL policy, and physics callbacks.
    ROS graph creation lives in ros_graphs.py so extension.py can stay small.

    Jackal plugs in via the ``_secondary_spawn`` callback: the panel module
    sets it to a free function ``(sample) -> None`` before
    ``load_world_async()`` runs.
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

        # Optional Jackal spawn callback. Set externally by the Jackal panel
        # handler BEFORE load_world_async() kicks off setup_scene().
        # Signature: callable(self) -> None. Leave as None to spawn Spot alone.
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

        # 두 가지 로드 모드:
        # 1) TUNED_SCENE_USD_PATH가 존재하면 미리 튜닝된 풀 씬(warehouse + spot +
        #    jackal + headlights + fire lights + rescue box + OmniGraphs)을
        #    /World에 한 번에 reference로 박는다. spot/jackal prim이 이미 있으니
        #    SpotFlatTerrainPolicy/SingleArticulation은 existing prim을 wrap.
        # 2) 없으면 legacy: warehouse만 로드 후 spot/jackal/light 코드로 추가.
        use_tuned = TUNED_SCENE_USD_PATH and TUNED_SCENE_USD_PATH.exists()

        if use_tuned:
            # save_as_stage로 저장된 USD는 defaultPrim이 박혀있지 않을 수 있음.
            # add_reference_to_stage는 defaultPrim 기본 참조라 실패함. 그래서
            # 명시적으로 referenced layer의 /World prim을 target으로 지정해서
            # 우리 stage의 /World에 합성.
            stage = omni.usd.get_context().get_stage()
            world_prim = stage.GetPrimAtPath("/World")
            if not world_prim.IsValid():
                world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
            world_prim.GetReferences().AddReference(
                assetPath=str(TUNED_SCENE_USD_PATH),
                primPath="/World",
            )
            print(f"[cobot3.spot] 튜닝 씬 로드: {TUNED_SCENE_USD_PATH.name}")
        else:
            add_reference_to_stage(
                usd_path=str(WAREHOUSE_USD_PATH),
                prim_path="/World/Warehouse",
            )
            print(f"[cobot3.spot] 기본 warehouse 로드: {WAREHOUSE_USD_PATH.name}")

        # SpotFlatTerrainPolicy: prim 있든 없든 attach 가능 (기존이면 wrap)
        self.spot = SpotFlatTerrainPolicy(
            prim_path=SPOT_PRIM_PATH,
            name="Spot",
            position=np.array(SPOT_SPAWN_POSITION, dtype=np.float64),
        )
        print("[cobot3.spot] Spot RL 정책 로드 완료")

        # 튜닝 씬이면 카메라/헤드라이트 prim 이미 USD에 있음 — 이 함수들은
        # idempotent하니 재호출해도 transform/속성만 갱신.
        self._add_cameras()
        self._add_headlight()
        self._add_fire_lights()
        self._add_ceiling_lights()

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

            # FOV 90° — 인접 카메라(90° 간격)와 빈틈 없이 파노라마 연결.
            cam = UsdGeom.Camera(cam_prim)
            cam.CreateFocalLengthAttr().Set(CAMERA_FOCAL_LENGTH_MM)
            cam.CreateHorizontalApertureAttr().Set(CAMERA_HORIZONTAL_APERTURE_MM)
            cam.CreateVerticalApertureAttr().Set(CAMERA_VERTICAL_APERTURE_MM)

            action = "추가" if created else "pose 업데이트"
            print(f"[cobot3.spot] {spec['label']} 카메라 {action} 완료 (hFOV 90°)")

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

        # 기존 op precision 변경하면 AddXformOp 에러 — 있는 op는 Set만, 없는 op만 Add.
        xform = UsdGeom.Xformable(light_prim)
        existing = {op.GetOpName(): op for op in xform.GetOrderedXformOps()}

        if "xformOp:translate" in existing:
            existing["xformOp:translate"].Set(Gf.Vec3d(*HEADLIGHT_TRANSLATION))
        else:
            xform.AddTranslateOp().Set(Gf.Vec3d(*HEADLIGHT_TRANSLATION))

        if "xformOp:rotateXYZ" in existing:
            existing["xformOp:rotateXYZ"].Set(Gf.Vec3f(*HEADLIGHT_ROTATION_XYZ_DEG))
        else:
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

    def _add_fire_lights(self):
        """화재 분위기 SphereLight 8개를 /World/FireLights/ 아래 생성.

        FIRE_LIGHT_POSITIONS의 (name, (x,y,z))로 배치. Idempotent — 이미 있으면
        translate/intensity만 갱신.
        """
        import omni.kit.commands

        stage = omni.usd.get_context().get_stage()

        # 그룹 Xform 보장
        if not stage.GetPrimAtPath(FIRE_LIGHTS_GROUP_PATH).IsValid():
            omni.kit.commands.execute(
                "CreatePrim",
                prim_type="Xform",
                prim_path=FIRE_LIGHTS_GROUP_PATH,
            )

        created_count = 0
        for name, pos in FIRE_LIGHT_POSITIONS:
            path = f"{FIRE_LIGHTS_GROUP_PATH}/{name}"
            if not stage.GetPrimAtPath(path).IsValid():
                omni.kit.commands.execute(
                    "CreatePrimWithDefaultXform",
                    prim_type="SphereLight",
                    prim_path=path,
                )
                created_count += 1
            prim = stage.GetPrimAtPath(path)
            UsdGeom.Imageable(prim).MakeVisible()

            # 기존 xform op는 Set, 없으면 Add (precision mismatch 회피)
            xform = UsdGeom.Xformable(prim)
            existing = {op.GetOpName(): op for op in xform.GetOrderedXformOps()}
            if "xformOp:translate" in existing:
                existing["xformOp:translate"].Set(Gf.Vec3d(*pos))
            else:
                xform.AddTranslateOp().Set(Gf.Vec3d(*pos))

            sl = UsdLux.SphereLight(prim)
            sl.CreateIntensityAttr().Set(FIRE_LIGHT_INTENSITY)
            sl.CreateRadiusAttr().Set(FIRE_LIGHT_RADIUS)
            sl.CreateColorAttr().Set(Gf.Vec3f(*FIRE_LIGHT_COLOR_RGB))

        print(f"[cobot3.spot] 화염 조명 {len(FIRE_LIGHT_POSITIONS)}개 적용 "
              f"(신규 {created_count}개)")

    def _add_ceiling_lights(self):
        """천장 SphereLight 격자를 /World/CeilingLights/ 아래 생성.

        호러식 깜빡임은 _start_atmosphere_flicker가 "CeilingLights" path token으로
        매칭해 자동 적용. Idempotent.
        """
        import omni.kit.commands

        stage = omni.usd.get_context().get_stage()

        if not stage.GetPrimAtPath(CEILING_LIGHTS_GROUP_PATH).IsValid():
            omni.kit.commands.execute(
                "CreatePrim",
                prim_type="Xform",
                prim_path=CEILING_LIGHTS_GROUP_PATH,
            )

        created_count = 0
        for name, pos in CEILING_LIGHT_POSITIONS:
            path = f"{CEILING_LIGHTS_GROUP_PATH}/{name}"
            if not stage.GetPrimAtPath(path).IsValid():
                omni.kit.commands.execute(
                    "CreatePrimWithDefaultXform",
                    prim_type="SphereLight",
                    prim_path=path,
                )
                created_count += 1
            prim = stage.GetPrimAtPath(path)
            UsdGeom.Imageable(prim).MakeVisible()

            xform = UsdGeom.Xformable(prim)
            existing = {op.GetOpName(): op for op in xform.GetOrderedXformOps()}
            if "xformOp:translate" in existing:
                existing["xformOp:translate"].Set(Gf.Vec3d(*pos))
            else:
                xform.AddTranslateOp().Set(Gf.Vec3d(*pos))

            sl = UsdLux.SphereLight(prim)
            sl.CreateIntensityAttr().Set(CEILING_LIGHT_INTENSITY)
            sl.CreateRadiusAttr().Set(CEILING_LIGHT_RADIUS)
            sl.CreateColorAttr().Set(Gf.Vec3f(*CEILING_LIGHT_COLOR_RGB))

        print(f"[cobot3.spot] 천장 조명 {len(CEILING_LIGHT_POSITIONS)}개 적용 "
              f"(신규 {created_count}개)")

    def _disable_other_lights(self):
        """Set intensity=0 on every light in the scene except Spot's headlight.

        This produces the dark-warehouse + flashlight-cone effect. The warehouse
        USD ships with ceiling lights and Isaac defaults to a DomeLight, both of
        which overwhelm the headlight. We don't delete them — just zero them out
        so re-enabling later is trivial (set intensity back to original).
        """
        # Light들을 모두 끄되, "robot headlight"로 표시된 것들은 유지.
        # spot HEADLIGHT_PRIM_PATH + jackal 등 보조 로봇 헤드라이트 (lazy import
        # — jackal 패키지 미설치/미로드 시 ImportError 무시).
        keep_paths = {HEADLIGHT_PRIM_PATH}
        try:
            from ..jackal.constants import JACKAL_HEADLIGHT_PRIM_PATH
            keep_paths.add(JACKAL_HEADLIGHT_PRIM_PATH)
        except Exception:
            pass

        # 우리 분위기 조명들은 예외 — path에 token 포함되면 모두 keep:
        #   - FireLights: 화염등 (flame flicker 대상)
        #   - CeilingLights: 추가 천장등 (horror flicker 대상)
        KEEP_TOKENS = ("FireLights", "CeilingLights")

        stage = omni.usd.get_context().get_stage()
        disabled = 0
        kept_token = 0
        for prim in stage.Traverse():
            type_name = str(prim.GetTypeName())
            if "Light" not in type_name:
                continue
            path_str = str(prim.GetPath())
            if path_str in keep_paths:
                continue
            if any(tok in path_str for tok in KEEP_TOKENS):
                kept_token += 1
                continue
            light = UsdLux.LightAPI(prim)
            intensity_attr = light.GetIntensityAttr()
            if intensity_attr and intensity_attr.IsValid():
                intensity_attr.Set(0.0)
                disabled += 1
        print(f"[cobot3.spot] 어둠 모드: 기존 조명 {disabled}개 OFF "
              f"(헤드라이트 {len(keep_paths)}개 + 분위기 조명 {kept_token}개 유지)")

    async def setup_post_load(self):
        self._physics_ready = False
        if not self.get_world().physics_callback_exists("spot_physics_step"):
            self.get_world().add_physics_callback("spot_physics_step", self._on_physics_step)
        await self.get_world().play_async()
        # warehouse 자체 천장 조명/DomeLight를 모두 꺼서 헤드라이트만 보이게.
        # play_async 이후에 호출 — 그래야 USD reference 완전 로드 후 traversal.
        self._disable_other_lights()
        # 천장등 + 화염등 깜빡임 자동 시작 (어둠+화재 분위기)
        self._start_atmosphere_flicker()
        print("[cobot3.spot] ✅ 로드 완료!")

    def _start_atmosphere_flicker(self):
        """천장 RectLight = horror random on/off, 화염 SphereLight = sine flame.

        Isaac update event stream에 구독해 매 frame intensity 갱신.
        구독은 self._flicker_sub에 보관 — Stop/Reset 시 정리.
        """
        import math
        import random
        import time

        import omni.kit.app

        stage = omni.usd.get_context().get_stage()

        # 천장 = warehouse USD의 RectLight + 우리가 추가한 /World/CeilingLights/* SphereLight
        ceiling_paths = []
        fire_paths = []
        for p in stage.Traverse():
            tn = str(p.GetTypeName())
            if "Light" not in tn:
                continue
            path = str(p.GetPath())
            if "FireLights" in path:
                fire_paths.append(path)
            elif tn == "RectLight" or "CeilingLights" in path:
                ceiling_paths.append(path)
        if not ceiling_paths and not fire_paths:
            print("[cobot3.spot] flicker: 대상 light 없음 — 구독 미생성")
            return

        # 천장등 horror state: random ON 짧고 OFF 길게
        ceiling_state = {
            path: {
                "next_toggle": time.time() + random.uniform(0.5, 2.5),
                "on": False,
            }
            for path in ceiling_paths
        }
        # 화염등 flame state: sine + noise modulation
        fire_state = {
            path: {
                "phase": random.uniform(0.0, math.tau),
                "freq": random.uniform(5.0, 12.0),
            }
            for path in fire_paths
        }

        CEILING_ON_INTENSITY = 25000.0   # 50000 → 25000: 어둡게
        CEILING_ON_RANGE = (0.04, 0.20)  # ON 시간 살짝 짧게
        CEILING_OFF_RANGE = (0.8, 3.0)   # OFF 시간 살짝 길게
        FIRE_BASE_INTENSITY = 80000.0    # 150000 → 80000: 화염도 약하게
        FIRE_VARIANCE = 70000.0
        start_time = time.time()

        def _update(_event):
            now = time.time()
            # 천장등
            for path, st in ceiling_state.items():
                if now < st["next_toggle"]:
                    continue
                st["on"] = not st["on"]
                if st["on"]:
                    st["next_toggle"] = now + random.uniform(*CEILING_ON_RANGE)
                else:
                    st["next_toggle"] = now + random.uniform(*CEILING_OFF_RANGE)
                prim = stage.GetPrimAtPath(Sdf.Path(path))
                if not prim.IsValid():
                    continue
                attr = UsdLux.LightAPI(prim).GetIntensityAttr()
                if attr and attr.IsValid():
                    attr.Set(CEILING_ON_INTENSITY if st["on"] else 0.0)
            # 화염등
            t = now - start_time
            for path, st in fire_state.items():
                wave = math.sin(t * st["freq"] + st["phase"])
                noise = random.uniform(-0.3, 0.3)
                factor = 1.0 + 0.5 * wave + 0.2 * noise
                intensity = max(0.0, FIRE_BASE_INTENSITY + FIRE_VARIANCE * (factor - 1.0))
                prim = stage.GetPrimAtPath(Sdf.Path(path))
                if not prim.IsValid():
                    continue
                attr = UsdLux.LightAPI(prim).GetIntensityAttr()
                if attr and attr.IsValid():
                    attr.Set(intensity)

        # 이전 구독 정리 (Stop/Reset 후 재로드)
        if hasattr(self, "_flicker_sub") and self._flicker_sub is not None:
            try:
                self._flicker_sub.unsubscribe()
            except Exception:
                pass
        self._flicker_sub = (
            omni.kit.app.get_app().get_update_event_stream()
            .create_subscription_to_pop(_update, name="cobot3_atmosphere_flicker")
        )
        print(f"[cobot3.spot] 깜빡임 시작: 천장등 {len(ceiling_paths)}개 "
              f"+ 화염등 {len(fire_paths)}개")
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
