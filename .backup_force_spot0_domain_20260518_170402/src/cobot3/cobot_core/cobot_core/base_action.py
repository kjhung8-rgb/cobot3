from .robot.jetbot_base_action import JetbotBaseAction
from .robot.spot_base_action import SpotBaseAction


ROBOT_BASE_ACTIONS = {
    "jetbot": JetbotBaseAction,
    "spot": SpotBaseAction,
}


def _get_param_value(node, name, default=None):
    try:
        if not node.has_parameter(name):
            node.declare_parameter(name, default)
        return node.get_parameter(name).value
    except Exception:
        return default


def infer_robot_type(node):
    """
    Decide which BaseAction class to use.

    robot_type controls behavior class:
      - jetbot -> JetbotBaseAction
      - spot_0   -> SpotBaseAction

    namespace controls topic name:
      - /jetbot  -> /jetbot/cmd_vel
      - /spot_0  -> /spot_0/cmd_vel
      - /spot_1  -> /spot_1/cmd_vel
    """
    robot_type = str(_get_param_value(node, "robot_type", "auto") or "auto").lower()
    if robot_type and robot_type != "auto":
        return robot_type

    namespace = node.get_namespace().strip("/").lower()

    if namespace.startswith("spot_0"):
        return "spot_0"
    if namespace.startswith("jetbot"):
        return "jetbot"

    return "spot_0"


def get_robot_namespace(node):
    namespace = node.get_namespace().strip("/")
    if namespace:
        return namespace
    return infer_robot_type(node)


def create_base_action(node):
    robot_type = infer_robot_type(node)
    cls = ROBOT_BASE_ACTIONS.get(robot_type)

    if cls is None:
        node.get_logger().warn(
            f"알 수 없는 robot_type={robot_type}, SpotBaseAction으로 fallback"
        )
        cls = SpotBaseAction
        robot_type = "spot_0"

    node.get_logger().info(f"🤖 Selected robot base action: {robot_type}")
    return cls(node)


# Compatibility for older imports: from .base_action import BaseAction
BaseAction = SpotBaseAction
