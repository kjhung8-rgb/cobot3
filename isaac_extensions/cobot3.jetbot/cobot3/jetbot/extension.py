import omni.ext
import omni.ui as ui
import omni.usd
import omni.timeline
from pxr import UsdGeom, Gf, Sdf
import omni.graph.core as og

from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.api.objects import GroundPlane


class Cobot3JetbotExtension(omni.ext.IExt):

    def _create_cmd_vel_graph(self):
        graph_path = "/World/JetBot_CmdVel_Graph"
        robot_path = "/World/jetbot"
        keys = og.Controller.Keys
        try:
            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("on_playback_tick",       "omni.graph.action.OnPlaybackTick"),
                        ("ros2_context",            "isaacsim.ros2.bridge.ROS2Context"),
                        ("ros2_subscribe_twist",    "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                        ("break_linear",            "omni.graph.nodes.BreakVector3"),
                        ("break_angular",           "omni.graph.nodes.BreakVector3"),
                        ("joint_name_array",        "omni.graph.nodes.ConstructArray"),
                        ("joint_name_array_add",    "omni.graph.nodes.ArrayInsertValue"),
                        ("differential_controller", "isaacsim.robot.wheeled_robots.DifferentialController"),
                        ("articulation_controller", "isaacsim.core.nodes.IsaacArticulationController"),
                    ],
                    keys.SET_VALUES: [
                        ("ros2_context.inputs:domain_id",               0),
                        ("ros2_subscribe_twist.inputs:topicName",       "cmd_vel"),
                        ("articulation_controller.inputs:targetPrim",   [Sdf.Path(robot_path)]),
                        ("joint_name_array.inputs:arrayType",           "token[]"),
                        ("joint_name_array.inputs:input0",              "left_wheel_joint"),
                        ("joint_name_array_add.inputs:index",           1),
                        ("joint_name_array_add.inputs:value",           "right_wheel_joint"),
                        ("differential_controller.inputs:wheelDistance",    0.1125),
                        ("differential_controller.inputs:wheelRadius",      0.03),
                        ("differential_controller.inputs:maxLinearSpeed",   1.0),
                        ("differential_controller.inputs:maxAngularSpeed",  2.0),
                    ],
                    keys.CONNECT: [
                        ("on_playback_tick.outputs:tick",               "ros2_subscribe_twist.inputs:execIn"),
                        ("on_playback_tick.outputs:tick",               "differential_controller.inputs:execIn"),
                        ("on_playback_tick.outputs:tick",               "articulation_controller.inputs:execIn"),
                        ("on_playback_tick.outputs:deltaSeconds",       "differential_controller.inputs:dt"),
                        ("ros2_context.outputs:context",                "ros2_subscribe_twist.inputs:context"),
                        ("ros2_subscribe_twist.outputs:linearVelocity", "break_linear.inputs:tuple"),
                        ("ros2_subscribe_twist.outputs:angularVelocity","break_angular.inputs:tuple"),
                        ("break_linear.outputs:x",                      "differential_controller.inputs:linearVelocity"),
                        ("break_angular.outputs:z",                     "differential_controller.inputs:angularVelocity"),
                        ("differential_controller.outputs:velocityCommand", "articulation_controller.inputs:velocityCommand"),
                        ("joint_name_array.outputs:array",              "joint_name_array_add.inputs:array"),
                        ("joint_name_array_add.outputs:array",          "articulation_controller.inputs:jointNames"),
                    ],
                },
            )
            print("[cobot3.jetbot] cmd_vel graph OK")
        except Exception as e:
            print(f"[cobot3.jetbot] cmd_vel graph FAIL: {e}")

    def _create_camera_graph(self):
        graph_path  = "/World/JetBot_Camera_Graph"
        camera_prim = "/World/jetbot/chassis/rgb_camera/jetbot_camera"
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
                        ("CameraHelperRgb.inputs:frameId",      "jetbot_camera"),
                        ("CameraHelperRgb.inputs:topicName",    "/rgb"),
                        ("CameraHelperRgb.inputs:type",         "rgb"),
                        ("CameraHelperDep.inputs:frameId",      "jetbot_camera"),
                        ("CameraHelperDep.inputs:topicName",    "/depth"),
                        ("CameraHelperDep.inputs:type",         "depth"),
                    ],
                },
            )
            print("[cobot3.jetbot] Camera graph OK")
        except Exception as e:
            print(f"[cobot3.jetbot] Camera graph FAIL: {e}")

    def on_startup(self, ext_id):
        print("[cobot3.jetbot] startup")
        self._window = ui.Window("Cobot3 JetBot", width=320, height=240)
        with self._window.frame:
            with ui.VStack(spacing=8):
                ui.Label("Cobot3 JetBot Controller", style={"font_size": 16})
                ui.Spacer(height=4)
                ui.Button("1. Load JetBot",  clicked_fn=self._load_jetbot)
                ui.Button("2. Play",         clicked_fn=self._play)
                ui.Button("3. Setup ROS2",   clicked_fn=self._setup_ros2)
                ui.Spacer(height=4)
                ui.Button("Stop",            clicked_fn=self._stop)

    def on_shutdown(self):
        print("[cobot3.jetbot] shutdown")
        self._window = None

    def _load_jetbot(self):
        stage = omni.usd.get_context().get_stage()
        try:
            GroundPlane(prim_path="/World/GroundPlane", size=20.0)
        except Exception as e:
            print(f"[cobot3.jetbot] GroundPlane skip: {e}")

        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            print("[cobot3.jetbot] assets_root_path None")
            return

        jetbot_asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"
        add_reference_to_stage(usd_path=jetbot_asset_path, prim_path="/World/jetbot")

        prim = stage.GetPrimAtPath("/World/jetbot")
        xform = UsdGeom.Xformable(prim)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.1))
        print("[cobot3.jetbot] JetBot loaded")

    def _setup_ros2(self):
        self._create_cmd_vel_graph()
        self._create_camera_graph()
        print("[cobot3.jetbot] ROS2 Setup done")

    def _play(self):
        omni.timeline.get_timeline_interface().play()

    def _stop(self):
        omni.timeline.get_timeline_interface().stop()
