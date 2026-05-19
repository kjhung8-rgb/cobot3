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
import os
import shlex
import shutil
import subprocess
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
        self._teleop_process = None

        self._window = ui.Window("Cobot3 Spot - Fire Rescue", width=390, height=380)
        with self._window.frame:
            with ui.VStack(spacing=6):
                ui.Label("🔥 Cobot3 Spot - Fire Rescue", style={"font_size": 16})
                ui.Spacer(height=4)

                ui.Label("[ Isaac Sim 설정 ]", style={"font_size": 12})
                ui.Button("1. Load Scene",   clicked_fn=self._load_scene)
                ui.Button("2. Setup ROS2",   clicked_fn=self._setup_ros2)
                ui.Button("3. Setup Camera", clicked_fn=self._setup_camera)
                ui.Button("4. Setup LiDAR/SLAM", clicked_fn=self._setup_slam_sensors)
                ui.Spacer(height=4)

                ui.Label("[ 제어 ]", style={"font_size": 12})
                ui.Button("Reset",           clicked_fn=self._reset)
                ui.Button("Stop",            clicked_fn=self._stop)
                ui.Button("🚀 Start Teleop Terminal", clicked_fn=self._start_spot_teleop)
                ui.Button("🛑 Stop Teleop Terminal",  clicked_fn=self._stop_spot_teleop)
                ui.Spacer(height=4)

                ui.Label("[ 터미널 명령어 ]", style={"font_size": 12})
                ui.Label("버튼: Start Teleop Terminal", style={"font_size": 11})
                ui.Label("ros2 run rqt_image_view rqt_image_view", style={"font_size": 11})

        print("[cobot3.spot] UI 준비 완료")

    def on_shutdown(self):
        print("[cobot3.spot] Extension 종료")
        self._stop_spot_teleop()
        self._window = None


    def _spot_teleop_script_path(self):
        """현재 extension.py 기준으로 tasks/spot_teleop.py 절대경로 반환"""
        return os.path.join(os.path.dirname(__file__), "tasks", "spot_teleop.py")

    def _terminal_command(self, bash_cmd):
        """설치된 터미널 에뮬레이터를 찾아 teleop 실행 명령 생성"""
        candidates = [
            ("gnome-terminal", ["gnome-terminal", "--", "bash", "-lc", bash_cmd]),
            ("terminator",     ["terminator", "-x", "bash", "-lc", bash_cmd]),
            ("x-terminal-emulator", ["x-terminal-emulator", "-e", "bash", "-lc", bash_cmd]),
            ("konsole",        ["konsole", "-e", "bash", "-lc", bash_cmd]),
            ("xterm",          ["xterm", "-e", "bash", "-lc", bash_cmd]),
        ]
        for exe, cmd in candidates:
            if shutil.which(exe):
                return cmd
        return None

    def _start_spot_teleop(self):
        """UI 버튼으로 새 터미널을 열고 spot_teleop.py 실행"""
        if self._teleop_process is not None and self._teleop_process.poll() is None:
            print("[cobot3.spot] spot_teleop이 이미 실행 중입니다.")
            return

        script_path = self._spot_teleop_script_path()
        if not os.path.exists(script_path):
            print(f"[cobot3.spot] ❌ spot_teleop.py를 찾을 수 없음: {script_path}")
            return

        domain_id = os.environ.get("ROS_DOMAIN_ID", "141")
        python_exe = shutil.which("python3") or "python3"

        # 새 터미널에서 ROS 환경 source 후 실행.
        # teleop은 stdin을 직접 읽기 때문에 백그라운드가 아니라 터미널 안에서 실행해야 함.
        bash_cmd = (
            f"export ROS_DOMAIN_ID={shlex.quote(domain_id)}; "
            f"source /opt/ros/humble/setup.bash; "
            f"echo '[cobot3.spot] ROS_DOMAIN_ID='\150; "
            f"echo '[cobot3.spot] running: {python_exe} {shlex.quote(script_path)}'; "
            f"{python_exe} {shlex.quote(script_path)}; "
            f"echo; echo '[cobot3.spot] spot_teleop 종료됨. Enter를 누르면 창이 닫힙니다.'; read"
        )

        cmd = self._terminal_command(bash_cmd)
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = domain_id

        try:
            if cmd is None:
                print("[cobot3.spot] ⚠️ 사용 가능한 터미널 에뮬레이터를 못 찾음")
                print(f"[cobot3.spot] 수동 실행: python3 {script_path}")
                return

            self._teleop_process = subprocess.Popen(cmd, env=env)
            print(f"[cobot3.spot] ✅ spot_teleop 터미널 실행: {script_path}")
            print(f"[cobot3.spot] ROS_DOMAIN_ID={domain_id}, topic=/spot_0/cmd_vel")
        except Exception as e:
            print(f"[cobot3.spot] ❌ spot_teleop 실행 실패: {e}")

    def _stop_spot_teleop(self):
        """UI 버튼으로 teleop 터미널 프로세스 종료 시도"""
        proc = getattr(self, "_teleop_process", None)
        if proc is None:
            return
        if proc.poll() is not None:
            self._teleop_process = None
            return
        try:
            proc.terminate()
            print("[cobot3.spot] spot_teleop 종료 요청")
        except Exception as e:
            print(f"[cobot3.spot] spot_teleop 종료 실패: {e}")
        finally:
            self._teleop_process = None

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


    # ── LiDAR / Odometry / TF for SLAM ─────────────────────
    def _setup_slam_sensors(self):
        """
        1차 SLAM 준비:
          - /spot_0/scan       : RTX 2D LiDAR LaserScan
          - /spot_0/odom       : Spot world pose 기반 Odometry
          - /tf                : odom -> spot_0/base_link, spot_0/base_link -> spot_0/lidar

        주의:
          - Isaac Sim을 ROS_DOMAIN_ID가 export된 같은 bash에서 실행해야 함.
          - LiDAR는 RTX sensor라 Play 상태에서 몇 프레임 지나야 scan이 나오기 시작함.
        """
        if self._sample is None:
            print("[cobot3.spot] Load Scene 먼저 클릭하세요!")
            return

        import traceback
        import omni.kit.commands

        robot_prim = "/World/Spot"
        lidar_path = "/World/Spot/body/spot_lidar"
        graph_path = "/World/Spot_SLAM_Graph"
        domain_id = int(os.environ.get("ROS_DOMAIN_ID", "141"))

        # 1) RTX 2D LiDAR prim 생성/복구
        stage = omni.usd.get_context().get_stage()
        try:
            if not stage.GetPrimAtPath(lidar_path).IsValid():
                created = False
                # Isaac Sim 5.x recommended command. Config name may differ slightly by install.
                for kwargs in [
                    dict(path="spot_lidar", parent="/World/Spot/body", config="Example_Rotary_2D"),
                    dict(path=lidar_path, parent=None, config="Example_Rotary_2D"),
                    dict(path="spot_lidar", parent="/World/Spot/body", config="Example Rotary 2D"),
                    dict(path=lidar_path, parent=None, config="Example Rotary 2D"),
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
                        f"수동 생성 후 prim을 {lidar_path} 위치로 맞춰줘."
                    )

            print(f"[cobot3.spot] ✅ LiDAR prim 준비: {lidar_path}")
        except Exception as e:
            print(f"[cobot3.spot] ❌ LiDAR prim 생성 실패: {e}")
            traceback.print_exc()
            return

        # 2) RTX LiDAR -> /spot_0/scan Graph 생성
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
                        ("CreateRenderProduct.inputs:cameraPrim", [lidar_path]),
                        ("CreateRenderProduct.inputs:enabled", True),
                        ("LidarHelper.inputs:frameId", "spot_0/lidar"),
                        ("LidarHelper.inputs:topicName", "/spot_0/scan"),
                        ("LidarHelper.inputs:type", "laser_scan"),
                        ("LidarHelper.inputs:useSystemTime", True),
                    ],
                },
            )
            print("[cobot3.spot] ✅ LiDAR LaserScan graph 생성: /spot_0/scan")
        except Exception as e:
            print(f"[cobot3.spot] ❌ LiDAR graph 생성 실패: {e}")
            traceback.print_exc()
            return

        # 3) Odometry + TF publisher: rclpy를 Isaac physics callback에서 직접 publish
        try:
            import rclpy
            from nav_msgs.msg import Odometry
            from geometry_msgs.msg import TransformStamped
            from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

            if not rclpy.ok():
                rclpy.init(args=None)

            # 재호출 방지: 기존 node 있으면 그대로 사용
            if not hasattr(self._sample, "_slam_ros_node") or self._sample._slam_ros_node is None:
                self._sample._slam_ros_node = rclpy.create_node("spot0_isaac_slam_bridge")
                self._sample._slam_odom_pub = self._sample._slam_ros_node.create_publisher(
                    Odometry, "/spot_0/odom", 10
                )
                self._sample._slam_tf_pub = TransformBroadcaster(self._sample._slam_ros_node)
                self._sample._slam_static_tf_pub = StaticTransformBroadcaster(self._sample._slam_ros_node)
                self._sample._slam_last_pose = None
                self._sample._slam_last_time = None

            node = self._sample._slam_ros_node

            # Static TF: base_link -> lidar
            static_tf = TransformStamped()
            static_tf.header.stamp = node.get_clock().now().to_msg()
            static_tf.header.frame_id = "spot_0/base_link"
            static_tf.child_frame_id = "spot_0/lidar"
            static_tf.transform.translation.x = 0.25
            static_tf.transform.translation.y = 0.0
            static_tf.transform.translation.z = 0.35
            static_tf.transform.rotation.w = 1.0
            self._sample._slam_static_tf_pub.sendTransform(static_tf)

            def slam_pub_callback(step_size):
                if self._sample is None or self._sample.spot is None:
                    return
                if not self._sample._physics_ready:
                    return

                try:
                    pos, quat = self._sample.spot.robot.get_world_pose()
                    # Isaac quaternion is commonly WXYZ. ROS wants XYZW.
                    qw, qx, qy, qz = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
                    now_msg = node.get_clock().now().to_msg()

                    odom = Odometry()
                    odom.header.stamp = now_msg
                    odom.header.frame_id = "odom"
                    odom.child_frame_id = "spot_0/base_link"
                    odom.pose.pose.position.x = float(pos[0])
                    odom.pose.pose.position.y = float(pos[1])
                    odom.pose.pose.position.z = float(pos[2])
                    odom.pose.pose.orientation.x = qx
                    odom.pose.pose.orientation.y = qy
                    odom.pose.pose.orientation.z = qz
                    odom.pose.pose.orientation.w = qw

                    # Rough velocity estimate from policy command; enough for first SLAM bring-up.
                    odom.twist.twist.linear.x = float(self._sample._base_command[0])
                    odom.twist.twist.linear.y = float(self._sample._base_command[1])
                    odom.twist.twist.angular.z = float(self._sample._base_command[2])
                    self._sample._slam_odom_pub.publish(odom)

                    tf = TransformStamped()
                    tf.header.stamp = now_msg
                    tf.header.frame_id = "odom"
                    tf.child_frame_id = "spot_0/base_link"
                    tf.transform.translation.x = float(pos[0])
                    tf.transform.translation.y = float(pos[1])
                    tf.transform.translation.z = float(pos[2])
                    tf.transform.rotation.x = qx
                    tf.transform.rotation.y = qy
                    tf.transform.rotation.z = qz
                    tf.transform.rotation.w = qw
                    self._sample._slam_tf_pub.sendTransform(tf)

                    rclpy.spin_once(node, timeout_sec=0.0)
                except Exception as exc:
                    print(f"[cobot3.spot] SLAM odom/tf publish 실패: {exc}")

            world = self._sample.get_world()
            if world:
                if world.physics_callback_exists("spot_slam_pub_callback"):
                    world.remove_physics_callback("spot_slam_pub_callback")
                world.add_physics_callback("spot_slam_pub_callback", slam_pub_callback)

            print("[cobot3.spot] ✅ Odometry/TF publish 시작")
            print("[cobot3.spot]   /spot_0/odom")
            print("[cobot3.spot]   /tf: odom -> spot_0/base_link -> spot_0/lidar")
            print("[cobot3.spot] 다음 확인:")
            print("[cobot3.spot]   ros2 topic list | grep -E 'scan|odom|tf'")
            print("[cobot3.spot]   ros2 topic info /spot_0/scan")
            print("[cobot3.spot]   ros2 topic echo /spot_0/odom --once")
        except Exception as e:
            print(f"[cobot3.spot] ❌ Odometry/TF publisher 생성 실패: {e}")
            traceback.print_exc()
            return


    # ── 카메라 토픽 발행 ───────────────────────────────────
    def _setup_camera(self):
        """
        ROS2 토픽 인터페이스 v0.1 기준 카메라 Graph 생성
          - /spot_0/front_cam/color_image   sensor_msgs/Image, rgb
          - /spot_0/front_cam/depth_image   sensor_msgs/Image, depth
          - /spot_0/front_cam/camera_info   sensor_msgs/CameraInfo
          - frame_id: spot_0/front_cam_link
        """
        if self._sample is None:
            print("[cobot3.spot] Load Scene 먼저 클릭하세요!")
            return

        graph_path  = "/World/Spot_Camera_Graph"
        camera_prim = "/World/Spot/body/front_camera"
        domain_id = int(os.environ.get("ROS_DOMAIN_ID", "141"))
        stage = omni.usd.get_context().get_stage()

        if stage.GetPrimAtPath(graph_path).IsValid():
            stage.RemovePrim(graph_path)

        keys = og.Controller.Keys
        try:
            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick",    "omni.graph.action.OnPlaybackTick"),
                        ("ROS2Context",       "isaacsim.ros2.bridge.ROS2Context"),
                        ("RenderProduct",     "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                        ("CameraHelperRgb",   "isaacsim.ros2.bridge.ROS2CameraHelper"),
                        ("CameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                        ("CameraHelperInfo",  "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick",             "RenderProduct.inputs:execIn"),
                        ("RenderProduct.outputs:execOut",           "CameraHelperRgb.inputs:execIn"),
                        ("RenderProduct.outputs:execOut",           "CameraHelperDepth.inputs:execIn"),
                        ("RenderProduct.outputs:execOut",           "CameraHelperInfo.inputs:execIn"),
                        ("ROS2Context.outputs:context",             "CameraHelperRgb.inputs:context"),
                        ("ROS2Context.outputs:context",             "CameraHelperDepth.inputs:context"),
                        ("ROS2Context.outputs:context",             "CameraHelperInfo.inputs:context"),
                        ("RenderProduct.outputs:renderProductPath", "CameraHelperRgb.inputs:renderProductPath"),
                        ("RenderProduct.outputs:renderProductPath", "CameraHelperDepth.inputs:renderProductPath"),
                        ("RenderProduct.outputs:renderProductPath", "CameraHelperInfo.inputs:renderProductPath"),
                    ],
                    keys.SET_VALUES: [
                        ("ROS2Context.inputs:domain_id",          domain_id),
                        ("RenderProduct.inputs:cameraPrim",      [camera_prim]),
                        ("RenderProduct.inputs:enabled",         True),

                        ("CameraHelperRgb.inputs:frameId",       "spot_0/front_cam_link"),
                        ("CameraHelperRgb.inputs:topicName",     "/spot_0/front_cam/color_image"),
                        ("CameraHelperRgb.inputs:type",          "rgb"),

                        ("CameraHelperDepth.inputs:frameId",     "spot_0/front_cam_link"),
                        ("CameraHelperDepth.inputs:topicName",   "/spot_0/front_cam/depth_image"),
                        ("CameraHelperDepth.inputs:type",        "depth"),

                        ("CameraHelperInfo.inputs:frameId",      "spot_0/front_cam_link"),
                        ("CameraHelperInfo.inputs:topicName",    "/spot_0/front_cam/camera_info"),
                        ("CameraHelperInfo.inputs:type",         "camera_info"),
                    ],
                },
            )
            print("[cobot3.spot] ✅ Camera Graph 생성 완료! / frame_id=spot_0/front_cam_link")
            print("[cobot3.spot]   /spot_0/front_cam/color_image")
            print("[cobot3.spot]   /spot_0/front_cam/depth_image")
            print("[cobot3.spot]   /spot_0/front_cam/camera_info")
        except Exception as e:
            print(f"[cobot3.spot] ❌ Camera Graph 생성 실패: {e}")

    # ── LiDAR / Odometry / TF for SLAM ─────────────────────
    def _setup_slam_sensors(self):
        """
        ROS2 토픽 인터페이스 v0.1 기준 SLAM 준비:
          - /spot_0/scan : sensor_msgs/LaserScan, frame_id=spot_0/lidar_link
          - /spot_0/odom : nav_msgs/Odometry, frame_id=odom, child=spot_0/base_link
          - /tf          : odom -> spot_0/base_link
          - /tf_static   : spot_0/base_link -> spot_0/lidar_link
                            spot_0/base_link -> spot_0/front_cam_link
        """
        if self._sample is None:
            print("[cobot3.spot] Load Scene 먼저 클릭하세요!")
            return

        import traceback
        import omni.kit.commands

        lidar_path = "/World/Spot/body/spot_lidar"
        graph_path = "/World/Spot_SLAM_Graph"
        domain_id = int(os.environ.get("ROS_DOMAIN_ID", "141"))
        stage = omni.usd.get_context().get_stage()

        # 1) RTX 2D LiDAR prim 생성/복구
        try:
            if not stage.GetPrimAtPath(lidar_path).IsValid():
                created = False
                # Isaac Sim 설치별 config 이름 차이를 고려한 fallback
                for kwargs in [
                    dict(path="spot_lidar", parent="/World/Spot/body", config="Example_Rotary_2D"),
                    dict(path=lidar_path, parent=None, config="Example_Rotary_2D"),
                    dict(path="spot_lidar", parent="/World/Spot/body", config="Example Rotary 2D"),
                    dict(path=lidar_path, parent=None, config="Example Rotary 2D"),
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
                        f"수동 생성 후 prim을 {lidar_path} 위치로 맞춰줘."
                    )

            print(f"[cobot3.spot] ✅ LiDAR prim 준비: {lidar_path}")
        except Exception as e:
            print(f"[cobot3.spot] ❌ LiDAR prim 생성 실패: {e}")
            traceback.print_exc()
            return

        # 2) RTX LiDAR -> /spot_0/scan Graph 생성
        if stage.GetPrimAtPath(graph_path).IsValid():
            stage.RemovePrim(graph_path)

        keys = og.Controller.Keys
        try:
            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick",      "omni.graph.action.OnPlaybackTick"),
                        ("ROS2Context",         "isaacsim.ros2.bridge.ROS2Context"),
                        ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                        ("LidarHelper",         "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick",                   "CreateRenderProduct.inputs:execIn"),
                        ("CreateRenderProduct.outputs:execOut",           "LidarHelper.inputs:execIn"),
                        ("ROS2Context.outputs:context",                   "LidarHelper.inputs:context"),
                        ("CreateRenderProduct.outputs:renderProductPath", "LidarHelper.inputs:renderProductPath"),
                    ],
                    keys.SET_VALUES: [
                        ("ROS2Context.inputs:domain_id",                  domain_id),
                        ("CreateRenderProduct.inputs:cameraPrim",         [lidar_path]),
                        ("CreateRenderProduct.inputs:enabled",            True),
                        ("LidarHelper.inputs:frameId",                    "spot_0/lidar_link"),
                        ("LidarHelper.inputs:topicName",                  "/spot_0/scan"),
                        ("LidarHelper.inputs:type",                       "laser_scan"),
                        ("LidarHelper.inputs:useSystemTime",              True),
                    ],
                },
            )
            print("[cobot3.spot] ✅ LiDAR LaserScan graph 생성: /spot_0/scan, frame_id=spot_0/lidar_link")
        except Exception as e:
            print(f"[cobot3.spot] ❌ LiDAR graph 생성 실패: {e}")
            traceback.print_exc()
            return

        # 3) Odometry + TF publisher: rclpy를 Isaac physics callback에서 직접 publish
        try:
            import rclpy
            from nav_msgs.msg import Odometry
            from geometry_msgs.msg import TransformStamped
            from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

            if not rclpy.ok():
                rclpy.init(args=None)

            if not hasattr(self._sample, "_slam_ros_node") or self._sample._slam_ros_node is None:
                self._sample._slam_ros_node = rclpy.create_node("spot0_isaac_slam_bridge")
                self._sample._slam_odom_pub = self._sample._slam_ros_node.create_publisher(
                    Odometry, "/spot_0/odom", 10
                )
                self._sample._slam_tf_pub = TransformBroadcaster(self._sample._slam_ros_node)
                self._sample._slam_static_tf_pub = StaticTransformBroadcaster(self._sample._slam_ros_node)

            node = self._sample._slam_ros_node

            def _send_static_tf():
                now = node.get_clock().now().to_msg()

                lidar_tf = TransformStamped()
                lidar_tf.header.stamp = now
                lidar_tf.header.frame_id = "spot_0/base_link"
                lidar_tf.child_frame_id = "spot_0/lidar_link"
                lidar_tf.transform.translation.x = 0.25
                lidar_tf.transform.translation.y = 0.0
                lidar_tf.transform.translation.z = 0.35
                lidar_tf.transform.rotation.w = 1.0

                cam_tf = TransformStamped()
                cam_tf.header.stamp = now
                cam_tf.header.frame_id = "spot_0/base_link"
                cam_tf.child_frame_id = "spot_0/front_cam_link"
                cam_tf.transform.translation.x = 0.5
                cam_tf.transform.translation.y = 0.0
                cam_tf.transform.translation.z = 0.3
                cam_tf.transform.rotation.w = 1.0

                self._sample._slam_static_tf_pub.sendTransform([lidar_tf, cam_tf])

            _send_static_tf()

            def slam_pub_callback(step_size):
                if self._sample is None or self._sample.spot is None:
                    return
                if not self._sample._physics_ready:
                    return

                try:
                    pos, quat = self._sample.spot.robot.get_world_pose()
                    # Isaac quaternion is commonly WXYZ. ROS wants XYZW.
                    qw, qx, qy, qz = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
                    now_msg = node.get_clock().now().to_msg()

                    odom = Odometry()
                    odom.header.stamp = now_msg
                    odom.header.frame_id = "odom"
                    odom.child_frame_id = "spot_0/base_link"
                    odom.pose.pose.position.x = float(pos[0])
                    odom.pose.pose.position.y = float(pos[1])
                    odom.pose.pose.position.z = float(pos[2])
                    odom.pose.pose.orientation.x = qx
                    odom.pose.pose.orientation.y = qy
                    odom.pose.pose.orientation.z = qz
                    odom.pose.pose.orientation.w = qw

                    # 1차 bring-up용 속도 추정: cmd_vel 기반. SLAM에는 pose/tf가 더 중요함.
                    odom.twist.twist.linear.x = float(self._sample._base_command[0])
                    odom.twist.twist.linear.y = float(self._sample._base_command[1])
                    odom.twist.twist.angular.z = float(self._sample._base_command[2])
                    self._sample._slam_odom_pub.publish(odom)

                    tf = TransformStamped()
                    tf.header.stamp = now_msg
                    tf.header.frame_id = "odom"
                    tf.child_frame_id = "spot_0/base_link"
                    tf.transform.translation.x = float(pos[0])
                    tf.transform.translation.y = float(pos[1])
                    tf.transform.translation.z = float(pos[2])
                    tf.transform.rotation.x = qx
                    tf.transform.rotation.y = qy
                    tf.transform.rotation.z = qz
                    tf.transform.rotation.w = qw
                    self._sample._slam_tf_pub.sendTransform(tf)

                    # tf_static은 latched지만, late-joiner 대비 가끔 재송신
                    if not hasattr(self._sample, "_slam_static_tf_count"):
                        self._sample._slam_static_tf_count = 0
                    self._sample._slam_static_tf_count += 1
                    if self._sample._slam_static_tf_count % 250 == 0:
                        _send_static_tf()

                    rclpy.spin_once(node, timeout_sec=0.0)
                except Exception as exc:
                    print(f"[cobot3.spot] SLAM odom/tf publish 실패: {exc}")

            world = self._sample.get_world()
            if world:
                if world.physics_callback_exists("spot_slam_pub_callback"):
                    world.remove_physics_callback("spot_slam_pub_callback")
                world.add_physics_callback("spot_slam_pub_callback", slam_pub_callback)

            print("[cobot3.spot] ✅ Odometry/TF publish 시작")
            print("[cobot3.spot]   /spot_0/odom frame_id=odom child_frame_id=spot_0/base_link")
            print("[cobot3.spot]   /tf: odom -> spot_0/base_link")
            print("[cobot3.spot]   /tf_static: spot_0/base_link -> spot_0/lidar_link")
            print("[cobot3.spot]   /tf_static: spot_0/base_link -> spot_0/front_cam_link")
            print("[cobot3.spot] 다음 확인:")
            print("[cobot3.spot]   ros2 topic info /spot_0/scan")
            print("[cobot3.spot]   ros2 topic echo /spot_0/odom --once")
            print("[cobot3.spot]   ros2 run tf2_ros tf2_echo odom spot_0/base_link")
        except Exception as e:
            print(f"[cobot3.spot] ❌ Odometry/TF publisher 생성 실패: {e}")
            traceback.print_exc()
            return
