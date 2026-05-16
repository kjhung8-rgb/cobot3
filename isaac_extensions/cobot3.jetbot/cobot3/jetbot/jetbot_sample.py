from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.robot.wheeled_robots.robots import WheeledRobot


class Cobot3JetbotSample(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self._jetbot = None
        return

    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()

        assets_root_path = get_assets_root_path()

        if assets_root_path is None:
            print("❌ Isaac Sim assets root path를 찾지 못함")
            return

        jetbot_asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"

        self._jetbot = world.scene.add(
            WheeledRobot(
                prim_path="/World/jetbot",
                name="jetbot",
                wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                create_robot=True,
                usd_path=jetbot_asset_path,
            )
        )

        print("✅ Cobot3 JetBot loaded at /World/jetbot")
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        self._jetbot = self._world.scene.get_object("jetbot")

        print("✅ Cobot3 JetBot setup_post_load 완료")
        return