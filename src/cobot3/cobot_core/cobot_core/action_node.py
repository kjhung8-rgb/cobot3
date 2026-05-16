# ~/dev_ws/src/artbot_motion/artbot_motion/motion_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .action_manager import ActionManager


class ActionNode(Node):
    def __init__(self):
        super().__init__("action_node")

        self.manager = ActionManager(self)

        self.command_sub = self.create_subscription(
            String,
            "/jetbot/action_command",
            self.command_callback,
            10
        )

        self.get_logger().info("Action Node 시작됨")

    def command_callback(self, msg):
        command = msg.data.strip()

        self.get_logger().info(f"명령 수신: {command}")

        success = self.manager.perform(command)

        if not success:
            self.get_logger().warn(f"명령 실행 실패: {command}")


def main(args=None):
    rclpy.init(args=args)

    node = ActionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.manager.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()