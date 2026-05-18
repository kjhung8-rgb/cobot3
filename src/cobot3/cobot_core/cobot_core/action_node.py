import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .action_manager import ActionManager


class ActionNode(Node):
    def __init__(self):
        super().__init__("action_node")

        # robot_type은 launch에서 auto/jetbot/spot으로 넘길 수 있다.
        # auto면 namespace(/jetbot, /spot)를 보고 ActionManager가 자동 선택한다.
        self.declare_parameter("robot_type", "auto")

        self.manager = ActionManager(self)

        self.command_sub = self.create_subscription(
            String,
            "action_command",
            self.command_callback,
            10,
        )

        self.get_logger().info("✅ action_node started")
        self.get_logger().info(
            f"📡 Namespace: {self.get_namespace()} / Subscribed: action_command"
        )

    def command_callback(self, msg):
        raw = msg.data.strip()

        if not raw:
            self.get_logger().warn("빈 명령 수신")
            return

        command = raw
        params = {}

        # 기존 호환: "move_forward" 같은 순수 문자열도 그대로 지원.
        # 확장 호환: {"command": "move_forward", "params": {...}} 도 지원.
        if raw.startswith("{"):
            try:
                data = json.loads(raw)
                command = str(data.get("command") or data.get("action") or "").strip()
                params = data.get("params") or {}
            except json.JSONDecodeError as exc:
                self.get_logger().warn(f"JSON 명령 파싱 실패: {exc} / raw={raw}")
                return

        if not command:
            self.get_logger().warn(f"명령 이름이 비어있음: {raw}")
            return

        self.get_logger().info(f"명령 수신: {command}, params={params}")

        success = self.manager.perform(command, **params)

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
