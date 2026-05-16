import omni.graph.core as og
import omni.usd
import omni.kit.app

# 기존 그래프 삭제
stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath("/World/JetbotCmdVelGraph")
if prim.IsValid():
    stage.RemovePrim("/World/JetbotCmdVelGraph")
omni.kit.app.get_app().update()

og.Controller.edit(
    {"graph_path": "/World/JetbotCmdVelGraph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick",  "omni.graph.action.OnPlaybackTick"),
            ("ROS2Context",     "isaacsim.ros2.bridge.ROS2Context"),
            ("SubscribeTwist",  "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
            ("DiffController",  "isaacsim.robot.wheeled_robots.DifferentialController"),
            ("ArticController", "isaacsim.core.nodes.IsaacArticulationController"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick",            "SubscribeTwist.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick",            "DiffController.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick",            "ArticController.inputs:execIn"),
            ("ROS2Context.outputs:context",            "SubscribeTwist.inputs:context"),
            ("SubscribeTwist.outputs:linearVelocity",  "DiffController.inputs:linearVelocity"),
            ("SubscribeTwist.outputs:angularVelocity", "DiffController.inputs:angularVelocity"),
            ("DiffController.outputs:velocityCommand", "ArticController.inputs:velocityCommand"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("SubscribeTwist.inputs:topicName",     "/cmd_vel"),
            ("DiffController.inputs:wheelRadius",   0.03),
            ("DiffController.inputs:wheelDistance", 0.1125),
            ("ArticController.inputs:robotPath",    "/World/jetbot"),
            ("ArticController.inputs:jointNames",   ["left_wheel_joint", "right_wheel_joint"]),
        ],
    }
)
print("Done!")