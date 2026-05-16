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

        if command == "forward":
            self.manager.perform("move_forward", speed=0.25, duration=2.0)

        elif command == "backward":
            self.manager.perform("move_backward", speed=0.2, duration=1.0)

        elif command == "left":
            self.manager.perform("rotate_left", speed=0.7, duration=1.0)

        elif command == "right":
            self.manager.perform("rotate_right", speed=0.7, duration=1.0)

        elif command == "stop":
            self.manager.perform("stop")

        elif command == "patrol":
            self.manager.perform("patrol")

        elif command == "greeting":
            self.manager.perform("greeting_motion")

        else:
            self.get_logger().warn(f"알 수 없는 명령: {command}")


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