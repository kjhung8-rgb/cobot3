"""
Cobot3 Spot Extension
=====================
공장 화재 현장 생존자 탐색 로봇 시뮬레이션

[아키텍처]
/spot_0/cmd_vel 토픽으로 모든 제어 통일
  - teleop (현재)     → 키보드로 수동 조종
  - Nav2 (추후)       → 자율주행 경로 계획
  - YOLOv8 (추후)     → 생존자 감지 기반 추적

[ROS2 토픽]
발행:
  /spot_0/front_cam/color_image  ← RGB 카메라 (YOLOv8 입력)
  /spot_0/front_cam/depth_image  ← Depth 카메라 (거리 계산)

구독:
  /spot_0/cmd_vel                ← 이동 명령 (teleop/Nav2/YOLOv8)
"""

import os
import asyncio
import numpy as np
import omni
import omni.ext
import omni.ui as ui
import omni.usd
import omni.timeline
import omni.graph.core as og
from pxr import UsdGeom, Gf

from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy


class SpotFireRescue(BaseSample):
    """
    Spot 화재 현장 탐색 로봇 시뮬레이션 클래스
    BaseSample 상속으로 Isaac Sim physics 타이밍 보장
    """
    def __init__(self):
        super().__init__()
        self._world_settings["stage_units_in_meters"] = 1.0
        self._world_settings["physics_dt"] = 1.0 / 500.0
        self._world_settings["rendering_dt"] = 10.0 / 500.0

        # Spot 제어 명령 [forward, lateral, yaw]
        # /spot_0/cmd_vel 토픽에서 업데이트됨
        self._base_command = np.array([0.0, 0.0, 0.0])
        self._physics_ready = False
        self.spot = None
        self._event_timer_callback = None

    def setup_scene(self):
        # 바닥
        self._world.scene.add_default_ground_plane(
            z_position=0,
            name="default_ground_plane",
            prim_path="/World/defaultGroundPlane",
            static_friction=0.2,
            dynamic_friction=0.2,
            restitution=0.01,
        )

        # Full Warehouse 환경 (화재 현장)
        assets_root = get_assets_root_path()
        add_reference_to_stage(
            usd_path=assets_root + "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd",
            prim_path="/World/Warehouse"
        )
        print("[cobot3.spot] Warehouse 로드 완료")

        # Spot + Isaac Lab 사전학습 RL 정책
        self.spot = SpotFlatTerrainPolicy(
            prim_path="/World/Spot",
            name="Spot",
            position=np.array([0, 0, 0.8]),
        )
        print("[cobot3.spot] Spot RL 정책 로드 완료")

        # 전방 카메라 추가 (YOLOv8 입력용)
        self._add_front_camera()

        # Timeline 이벤트 콜백
        timeline = omni.timeline.get_timeline_interface()
        self._event_timer_callback = timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PLAY), self._on_timeline_play
        )

    def _add_front_camera(self):
        """Spot 전방 카메라 추가 (YOLOv8 생존자 감지용)"""
        import omni.kit.commands
        stage = omni.usd.get_context().get_stage()
        camera_path = "/World/Spot/body/front_camera"

        if not stage.GetPrimAtPath(camera_path).IsValid():
            omni.kit.commands.execute(
                "CreatePrimWithDefaultXform",
                prim_type="Camera",
                prim_path=camera_path,
            )
            cam_prim = stage.GetPrimAtPath(camera_path)
            xform = UsdGeom.Xformable(cam_prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(Gf.Vec3d(0.5, 0.0, 0.3))
            xform.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            print("[cobot3.spot] 전방 카메라 추가 완료")
        else:
            print("[cobot3.spot] 전방 카메라 이미 존재")

    async def setup_post_load(self):
        """World 초기화 완료 후 physics callback 등록"""
        self._physics_ready = False
        if not self.get_world().physics_callback_exists("spot_physics_step"):
            self.get_world().add_physics_callback("spot_physics_step", self._on_physics_step)
        await self.get_world().play_async()
        print("[cobot3.spot] ✅ 로드 완료!")
        print("[cobot3.spot] Setup ROS2 → Setup Camera 클릭 후")
        print("[cobot3.spot] 터미널: python3 tasks/spot_teleop.py")

    async def setup_post_reset(self):
        self._physics_ready = False
        self._base_command = np.array([0.0, 0.0, 0.0])
        await self._world.play_async()
        if not self.get_world().physics_callback_exists("spot_physics_step"):
            self.get_world().add_physics_callback("spot_physics_step", self._on_physics_step)

    def _on_physics_step(self, step_size):
        if self._physics_ready:
            try:
                # 넘어짐 감지 (z높이 0.2m 이하)
                pos = self.spot.robot.get_world_pose()[0]
                if pos[2] < 0.2:
                    print("[cobot3.spot] 넘어짐 감지! 자동 복구...")
                    self._physics_ready = False
                    self._base_command = np.array([0.0, 0.0, 0.0])
                    return
            except:
                pass
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

    def world_cleanup(self):
        self._event_timer_callback = None
        world = self.get_world()
        if world.physics_callback_exists("spot_physics_step"):
            world.remove_physics_callback("spot_physics_step")
        if world.physics_callback_exists("ros2_cmd_callback"):
            world.remove_physics_callback("ros2_cmd_callback")


class Cobot3SpotExtension(omni.ext.IExt):
    """
    Isaac Sim Extension UI
    버튼 순서: Load Scene → Setup ROS2 → Setup Camera
    """

    def on_startup(self, ext_id):
        print("[cobot3.spot] Extension 시작")
        self._sample = None

        self._window = ui.Window("Cobot3 Spot - Fire Rescue", width=360, height=320)
        with self._window.frame:
            with ui.VStack(spacing=6):
                ui.Label("🔥 Cobot3 Spot - Fire Rescue", style={"font_size": 16})
                ui.Spacer(height=4)

                ui.Label("[ Isaac Sim 설정 ]", style={"font_size": 12})
                ui.Button("1. Load Scene",   clicked_fn=self._load_scene)
                ui.Button("2. Setup ROS2",   clicked_fn=self._setup_ros2)
                ui.Button("3. Setup Camera", clicked_fn=self._setup_camera)
                ui.Spacer(height=4)

                ui.Label("[ 제어 ]", style={"font_size": 12})
                ui.Button("Reset",           clicked_fn=self._reset)
                ui.Button("Stop",            clicked_fn=self._stop)
                ui.Spacer(height=4)

                ui.Label("[ 터미널 명령어 ]", style={"font_size": 12})
                ui.Label("python3 tasks/spot_teleop.py", style={"font_size": 11})
                ui.Label("ros2 run rqt_image_view rqt_image_view", style={"font_size": 11})

        print("[cobot3.spot] UI 준비 완료")

    def on_shutdown(self):
        print("[cobot3.spot] Extension 종료")
        self._window = None

    def _load_scene(self):
        """Spot + Warehouse 로드 (RL 정책 포함)"""
        self._sample = SpotFireRescue()
        asyncio.ensure_future(self._sample.load_world_async())
        print("[cobot3.spot] Scene 로딩 중... (잠시 기다려주세요)")

    def _reset(self):
        if self._sample:
            self._sample._base_command = np.array([0.0, 0.0, 0.0])
            asyncio.ensure_future(self._sample.reset_async())

    def _stop(self):
        if self._sample:
            self._sample._base_command = np.array([0.0, 0.0, 0.0])
        omni.timeline.get_timeline_interface().stop()

    # ── ROS2 cmd_vel 구독 ──────────────────────────────────
    def _setup_ros2(self):
        """
        /spot_0/cmd_vel 구독 Graph 생성
        teleop / Nav2 / YOLOv8 등 모든 제어가 이 토픽으로 통일
        """
        if self._sample is None:
            print("[cobot3.spot] Load Scene 먼저 클릭하세요!")
            return

        graph_path = "/World/Spot_CmdVel_Graph"
        stage = omni.usd.get_context().get_stage()
        if stage.GetPrimAtPath(graph_path).IsValid():
            stage.RemovePrim(graph_path)

        keys = og.Controller.Keys
        try:
            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("on_playback_tick",    "omni.graph.action.OnPlaybackTick"),
                        ("ros2_context",         "isaacsim.ros2.bridge.ROS2Context"),
                        ("ros2_subscribe_twist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                    ],
                    keys.SET_VALUES: [
                        ("ros2_context.inputs:domain_id",         int(os.environ.get("ROS_DOMAIN_ID", "141"))),
                        ("ros2_subscribe_twist.inputs:topicName", "/spot_0/cmd_vel"),
                    ],
                    keys.CONNECT: [
                        ("on_playback_tick.outputs:tick",  "ros2_subscribe_twist.inputs:execIn"),
                        ("ros2_context.outputs:context",   "ros2_subscribe_twist.inputs:context"),
                    ],
                },
            )

            # /spot_0/cmd_vel → Spot _base_command 변환 콜백
            # BreakVector3 대신 벡터 직접 인덱싱 (타입 불일치 문제 해결)
            def ros2_cmd_callback(step_size):
                if self._sample is None:
                    return
                if not self._sample._physics_ready:
                    return
                try:
                    lin_vel = og.Controller.attribute(
                        "outputs:linearVelocity",
                        og.Controller.node(f"{graph_path}/ros2_subscribe_twist")
                    ).get()
                    ang_vel = og.Controller.attribute(
                        "outputs:angularVelocity",
                        og.Controller.node(f"{graph_path}/ros2_subscribe_twist")
                    ).get()

                    if lin_vel is None or ang_vel is None:
                        return

                    # linear.x → 전후진
                    # angular.z → 좌우회전
                    lin_x = float(lin_vel[0])
                    ang_z = float(ang_vel[2])

                    # Spot RL 정책 명령 스케일링
                    # [forward, lateral, yaw]
                    self._sample._base_command = np.array([
                        lin_x * 2.0,
                        0.0,
                        ang_z * 2.0
                    ])
                except Exception:
                    pass

            world = self._sample.get_world()
            if world and not world.physics_callback_exists("ros2_cmd_callback"):
                world.add_physics_callback("ros2_cmd_callback", ros2_cmd_callback)

            print("[cobot3.spot] ✅ ROS2 cmd_vel 구독 시작!")
            print("[cobot3.spot] 토픽: /spot_0/cmd_vel")
            print("[cobot3.spot] 조종: python3 tasks/spot_teleop.py")
            print("[cobot3.spot] Nav2 연동 준비 완료 (추후 확장)")

        except Exception as e:
            print(f"[cobot3.spot] ❌ ROS2 Graph 생성 실패: {e}")

    # ── 카메라 토픽 발행 ───────────────────────────────────
    def _setup_camera(self):
        """
        카메라 토픽 발행 Graph 생성
        /spot_0/front_cam/color_image → YOLOv8 생존자 감지 (추후)
        /spot_0/front_cam/depth_image → 거리 계산 (추후)
        """
        graph_path  = "/World/Spot_Camera_Graph"
        camera_prim = "/World/Spot/body/front_camera"
        stage = omni.usd.get_context().get_stage()

        if stage.GetPrimAtPath(graph_path).IsValid():
            stage.RemovePrim(graph_path)

        keys = og.Controller.Keys
        try:
            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick",  "omni.graph.action.OnPlaybackTick"),
                        ("ROS2Context",     "isaacsim.ros2.bridge.ROS2Context"),
                        ("RenderProduct",   "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                        ("CameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                        ("CameraHelperDep", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick",             "RenderProduct.inputs:execIn"),
                        ("RenderProduct.outputs:execOut",           "CameraHelperRgb.inputs:execIn"),
                        ("RenderProduct.outputs:execOut",           "CameraHelperDep.inputs:execIn"),
                        ("ROS2Context.outputs:context",             "CameraHelperRgb.inputs:context"),
                        ("ROS2Context.outputs:context",             "CameraHelperDep.inputs:context"),
                        ("RenderProduct.outputs:renderProductPath", "CameraHelperRgb.inputs:renderProductPath"),
                        ("RenderProduct.outputs:renderProductPath", "CameraHelperDep.inputs:renderProductPath"),
                    ],
                    keys.SET_VALUES: [
                        ("RenderProduct.inputs:cameraPrim",     [camera_prim]),
                        ("RenderProduct.inputs:enabled",        True),
                        ("CameraHelperRgb.inputs:frameId",      "spot_front_camera"),
                        ("CameraHelperRgb.inputs:topicName",    "/spot_0/front_cam/color_image"),
                        ("CameraHelperRgb.inputs:type",         "rgb"),
                        ("CameraHelperDep.inputs:frameId",      "spot_front_camera"),
                        ("CameraHelperDep.inputs:topicName",    "/spot_0/front_cam/depth_image"),
                        ("CameraHelperDep.inputs:type",         "depth"),
                    ],
                },
            )
            print("[cobot3.spot] ✅ Camera Graph 생성 완료!")
            print("[cobot3.spot] /spot_0/front_cam/color_image → YOLOv8 입력 (추후)")
            print("[cobot3.spot] /spot_0/front_cam/depth_image → 거리 계산 (추후)")
            print("[cobot3.spot] 확인: ros2 run rqt_image_view rqt_image_view")
        except Exception as e:
            print(f"[cobot3.spot] ❌ Camera Graph 생성 실패: {e}")