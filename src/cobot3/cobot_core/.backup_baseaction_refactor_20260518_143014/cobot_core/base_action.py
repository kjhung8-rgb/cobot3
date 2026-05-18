"""Robot-specific low-level base actions.

공통 부모(MobileBaseAction)는 ROS2 cmd_vel publisher 생성/발행처럼
JetBot/Spot 모두 반드시 필요한 최소 기능만 담당한다.

실제 저수준 액션(stop/move_forward/rotate_left 등)은
각 로봇 클래스(JetbotBaseAction, SpotBaseAction)에 둔다.
그래야 로봇별 속도, 제한, 전처리/후처리 로직이 달라져도
고수준 task(Patrol 등)는 같은 메서드 이름만 호출하면 된다.
"""

import time

from geometry_msgs.msg import Twist


class MobileBaseAction:
    """cmd_vel 기반 모바일 로봇 공통 기반.

    ROS2에서 상대 토픽명 "cmd_vel"로 publisher를 만들면,
    action_node 네임스페이스에 따라 실제 토픽이 자동으로 달라진다.

    예:
      namespace=/jetbot -> /jetbot/cmd_vel
      namespace=/spot   -> /spot/cmd_vel
    """

    robot_type = "mobile"

    def __init__(self, node, cmd_vel_topic="cmd_vel"):
        self.node = node
        self.cmd_vel_topic = cmd_vel_topic
        self.cmd_pub = node.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.node.get_logger().info(
            f"✅ {self.__class__.__name__} ready / publish: {self._display_cmd_vel_topic()}"
        )

    def _display_cmd_vel_topic(self):
        """로그용 실제 cmd_vel 토픽 문자열."""
        if self.cmd_vel_topic.startswith("/"):
            return self.cmd_vel_topic

        namespace = self.node.get_namespace().strip("/")
        if namespace:
            return f"/{namespace}/{self.cmd_vel_topic}"
        return f"/{self.cmd_vel_topic}"

    def publish_cmd(self, linear_x=0.0, angular_z=0.0):
        """Twist 메시지를 cmd_vel로 발행한다."""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)
        return True


class JetbotBaseAction(MobileBaseAction):
    """JetBot용 저수준 액션.

    JetBot은 differential drive라서 cmd_vel의 linear.x/angular.z를
    단순하게 사용한다.
    """

    robot_type = "jetbot"
    default_linear_speed = 0.2
    default_angular_speed = 0.6
    command_period = 0.05

    def stop(self):
        self.publish_cmd(0.0, 0.0)
        return True

    def move_forward(self, speed=None, duration=1.0):
        if speed is None:
            speed = self.default_linear_speed

        start = time.time()
        while time.time() - start < float(duration):
            self.publish_cmd(linear_x=abs(float(speed)), angular_z=0.0)
            time.sleep(self.command_period)

        self.stop()
        return True

    def move_backward(self, speed=None, duration=1.0):
        if speed is None:
            speed = self.default_linear_speed

        start = time.time()
        while time.time() - start < float(duration):
            self.publish_cmd(linear_x=-abs(float(speed)), angular_z=0.0)
            time.sleep(self.command_period)

        self.stop()
        return True

    def rotate_left(self, speed=None, duration=1.0):
        if speed is None:
            speed = self.default_angular_speed

        start = time.time()
        while time.time() - start < float(duration):
            self.publish_cmd(linear_x=0.0, angular_z=abs(float(speed)))
            time.sleep(self.command_period)

        self.stop()
        return True

    def rotate_right(self, speed=None, duration=1.0):
        if speed is None:
            speed = self.default_angular_speed

        start = time.time()
        while time.time() - start < float(duration):
            self.publish_cmd(linear_x=0.0, angular_z=-abs(float(speed)))
            time.sleep(self.command_period)

        self.stop()
        return True

    def wait(self, duration=1.0):
        self.stop()
        time.sleep(float(duration))
        return True


class SpotBaseAction(MobileBaseAction):
    """Spot용 저수준 액션.

    현재 1차 목표는 cmd_vel 기반 이동이므로 JetBot과 같은 Twist 토픽을 쓰되,
    기본 속도/회전값과 추후 Spot 전용 stand/sit/gait/body pose 로직은
    여기에서만 관리한다.
    """

    robot_type = "spot"
    default_linear_speed = 0.25
    default_angular_speed = 0.5
    command_period = 0.05

    def stop(self):
        # 나중에 Spot SDK/Isaac Spot 전용 정지 로직이 필요하면 여기만 수정한다.
        self.publish_cmd(0.0, 0.0)
        return True

    def move_forward(self, speed=None, duration=1.0):
        if speed is None:
            speed = self.default_linear_speed

        # 나중에 stand_if_needed(), gait 설정 등이 필요하면 여기 앞에 추가한다.
        start = time.time()
        while time.time() - start < float(duration):
            self.publish_cmd(linear_x=abs(float(speed)), angular_z=0.0)
            time.sleep(self.command_period)

        self.stop()
        return True

    def move_backward(self, speed=None, duration=1.0):
        if speed is None:
            speed = self.default_linear_speed

        start = time.time()
        while time.time() - start < float(duration):
            self.publish_cmd(linear_x=-abs(float(speed)), angular_z=0.0)
            time.sleep(self.command_period)

        self.stop()
        return True

    def rotate_left(self, speed=None, duration=1.0):
        if speed is None:
            speed = self.default_angular_speed

        start = time.time()
        while time.time() - start < float(duration):
            self.publish_cmd(linear_x=0.0, angular_z=abs(float(speed)))
            time.sleep(self.command_period)

        self.stop()
        return True

    def rotate_right(self, speed=None, duration=1.0):
        if speed is None:
            speed = self.default_angular_speed

        start = time.time()
        while time.time() - start < float(duration):
            self.publish_cmd(linear_x=0.0, angular_z=-abs(float(speed)))
            time.sleep(self.command_period)

        self.stop()
        return True

    def wait(self, duration=1.0):
        self.stop()
        time.sleep(float(duration))
        return True

    # Spot 전용 액션은 여기부터 추가하면 된다.
    def stand(self):
        self.node.get_logger().info("Spot stand requested - TODO: Spot 전용 stand 로직 연결")
        return True

    def sit(self):
        self.stop()
        self.node.get_logger().info("Spot sit requested - TODO: Spot 전용 sit 로직 연결")
        return True


ROBOT_BASE_ACTIONS = {
    "jetbot": JetbotBaseAction,
    "spot": SpotBaseAction,
}


def _normalize_robot_type(robot_type):
    return str(robot_type or "").strip().strip("/").lower()


def infer_robot_type(node):
    """action_node의 parameter/namespace로 robot_type을 결정한다."""
    robot_type = ""

    try:
        robot_type = node.declare_parameter("robot_type", "auto").value
    except Exception:
        # 이미 다른 곳에서 선언됐거나 파라미터 접근이 실패해도 namespace fallback 사용
        try:
            robot_type = node.get_parameter("robot_type").value
        except Exception:
            robot_type = "auto"

    robot_type = _normalize_robot_type(robot_type)
    if robot_type and robot_type != "auto":
        return robot_type

    namespace = _normalize_robot_type(node.get_namespace())
    if namespace:
        # /team/spot 같은 namespace도 마지막 이름 기준으로 판단
        candidate = namespace.split("/")[-1]
        if candidate:
            return candidate

    return "jetbot"


def create_base_action(node, robot_type=None):
    """로봇 타입에 맞는 BaseAction 인스턴스를 생성한다."""
    selected_type = _normalize_robot_type(robot_type) or infer_robot_type(node)
    action_cls = ROBOT_BASE_ACTIONS.get(selected_type)

    if action_cls is None:
        node.get_logger().warn(
            f"알 수 없는 robot_type={selected_type!r}. "
            "JetbotBaseAction으로 fallback합니다. "
            f"사용 가능: {list(ROBOT_BASE_ACTIONS.keys())}"
        )
        selected_type = "jetbot"
        action_cls = JetbotBaseAction

    node.get_logger().info(f"🤖 Selected robot base action: {selected_type}")
    return action_cls(node)


# 기존 코드 호환용 alias.
# 예전 코드가 `from .base_action import BaseAction`을 해도 깨지지 않게 둔다.
BaseAction = JetbotBaseAction
