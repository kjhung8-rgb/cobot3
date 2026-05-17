import json
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


class JsonCommandSender(Node):
    def __init__(self):
        super().__init__("json_command_sender")
        self.pub = self.create_publisher(String, "/robot_command", 10)

    def resolve_json_path(self, arg: str) -> Path:
        path = Path(arg).expanduser()

        # 1. 사용자가 실제 경로를 준 경우
        if path.exists():
            return path

        # 2. 확장자 없으면 .json 붙이기
        filename = arg if arg.endswith(".json") else f"{arg}.json"

        # 3. install/share 쪽 test_commands에서 찾기
        share_dir = Path(get_package_share_directory("cobot_core"))
        candidate = share_dir / "test_commands" / filename
        if candidate.exists():
            return candidate

        # 4. 혹시 symlink/source 환경에서 src 쪽도 fallback으로 찾기
        src_candidate = (
            Path.home()
            / "dev_ws"
            / "cobot3"
            / "src"
            / "cobot3"
            / "cobot_core"
            / "test_commands"
            / filename
        )
        if src_candidate.exists():
            return src_candidate

        raise FileNotFoundError(
            f"JSON file not found: {arg}\n"
            f"tried:\n"
            f"- {candidate}\n"
            f"- {src_candidate}"
        )

    def send_file(self, arg: str):
        json_path = self.resolve_json_path(arg)

        payload = json_path.read_text(encoding="utf-8").strip()

        # JSON 유효성 검사
        parsed = json.loads(payload)
        payload = json.dumps(parsed, ensure_ascii=False)

        msg = String()
        msg.data = payload

        # publisher discovery 대기용
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)

        self.pub.publish(msg)

        self.get_logger().info(f"📨 Sent JSON file: {json_path}")
        self.get_logger().info("📤 Topic: /robot_command")
        self.get_logger().info(f"📦 Payload: {payload}")


def main(args=None):
    rclpy.init(args=args)

    node = JsonCommandSender()

    try:
        if len(sys.argv) < 2:
            node.get_logger().error(
                "Usage: ros2 run cobot_core send_json_command <json_name_or_path>\n"
                "Example: ros2 run cobot_core send_json_command jetbot_move_forward"
            )
            return

        node.send_file(sys.argv[1])

    except Exception as e:
        node.get_logger().error(str(e))

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()