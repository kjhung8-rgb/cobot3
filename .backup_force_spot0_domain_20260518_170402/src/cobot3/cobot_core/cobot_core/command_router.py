import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CommandRouter(Node):
    """Route common JSON robot commands to each robot namespace.

    Input topic:
        /robot_command

    Input JSON example:
        {"robot": "jetbot", "command": "move_forward", "params": {"duration": 1.0}}

    Output topic:
        /{robot}/action_command

    Output message:
        JSON string with command/action and params.
    """

    def __init__(self):
        super().__init__("command_router")

        self.input_topic = self.declare_parameter(
            "input_topic",
            "/robot_command",
        ).get_parameter_value().string_value

        self.default_robot = self.declare_parameter(
            "default_robot",
            "jetbot",
        ).get_parameter_value().string_value

        self._router_publishers = {}

        self.command_sub = self.create_subscription(
            String,
            self.input_topic,
            self.command_callback,
            10,
        )

        self.get_logger().info("✅ command_router started")
        self.get_logger().info(f"📥 Subscribed: {self.input_topic}")
        self.get_logger().info("📤 Output: /{robot}/action_command")

    def _get_publisher(self, robot_name: str):
        topic = f"/{robot_name}/action_command"

        if topic not in self._router_publishers:
            self._router_publishers[topic] = self.create_publisher(String, topic, 10)
            self.get_logger().info(f"📡 Created publisher: {topic}")

        return topic, self._router_publishers[topic]

    def command_callback(self, msg: String):
        raw = msg.data.strip()

        if not raw:
            self.get_logger().warn("빈 JSON 명령 수신")
            return

        try:
            data = json.loads(raw)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"JSON 파싱 실패: {exc} / raw={raw}")
            return

        robot_name = str(
            data.get("robot") or data.get("namespace") or self.default_robot
        ).strip().strip("/")

        command = str(
            data.get("command") or data.get("action") or ""
        ).strip()

        params = data.get("params") or {}
        if not isinstance(params, dict):
            self.get_logger().warn(f"params가 dict가 아님. 무시함: {params}")
            params = {}

        if not robot_name:
            self.get_logger().warn(f"robot 값이 비어있음: {data}")
            return

        if not command:
            self.get_logger().warn(f"command/action 값이 비어있음: {data}")
            return

        topic, publisher = self._get_publisher(robot_name)

        out = String()
        out.data = json.dumps(
            {
                "command": command,
                "params": params,
            },
            ensure_ascii=False,
        )
        publisher.publish(out)

        self.get_logger().info(
            f"🚦 Routed: {self.input_topic} -> {topic} / command={command}, params={params}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CommandRouter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt - command_router 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
