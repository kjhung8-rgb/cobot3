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
            "action_command",
            self.command_callback,
            10,
        )

        self.get_logger().info("✅ action_node started")
        self.get_logger().info(f"📡 Namespace: {self.get_namespace()} / Subscribed: action_command")

    def command_callback(self, msg):
        command = msg.data.strip()

        if not command:
            self.get_logger().warn("빈 명령 수신")
            return

        self.get_logger().info(f"명령 수신: {command}")

        success = self.manager.perform(command)

        if success:
            self.get_logger().info(f"명령 완료: {command}")
        else:
            self.get_logger().warn(f"명령 실패: {command}")


def main(args=None):
    rclpy.init(args=args)

    node = ActionNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt - action_node 종료")

    finally:
        node.manager.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
