from .robot.jetbot_base_action import JetbotBaseAction
from .robot.spot_base_action import SpotBaseAction


ROBOT_BASE_ACTIONS = {
    "jetbot": JetbotBaseAction,
    "spot": SpotBaseAction,
}


def _get_robot_type_param(node):
    """Return robot_type parameter value if available, otherwise 'auto'."""
    try:
        value = node.get_parameter("robot_type").value
        return str(value).strip().lower()
    except Exception:
        return "auto"


def _normalize_robot_type(name: str) -> str:
    """Map namespace-like names to robot type.

    Examples:
      spot_0  -> spot
      spot_12 -> spot
      spot    -> spot
      jetbot  -> jetbot
      jetbot_0 -> jetbot
    """
    name = (name or "").strip().strip("/").lower()

    if name.startswith("spot"):
        return "spot"
    if name.startswith("jetbot"):
        return "jetbot"

    return name


def infer_robot_type(node):
    robot_type = _get_robot_type_param(node)
    if robot_type and robot_type != "auto":
        return _normalize_robot_type(robot_type)

    namespace = node.get_namespace().strip("/").lower()
    if namespace:
        return _normalize_robot_type(namespace)

    return "spot"


def create_base_action(node):
    robot_type = infer_robot_type(node)
    action_cls = ROBOT_BASE_ACTIONS.get(robot_type)

    if action_cls is None:
        node.get_logger().warn(
            f"알 수 없는 robot_type='{robot_type}'. SpotBaseAction으로 fallback"
        )
        action_cls = SpotBaseAction
        robot_type = "spot"

    node.get_logger().info(f"🤖 Selected robot base action: {robot_type}")
    return action_cls(node)


# 기존 코드 호환용 alias.
BaseAction = SpotBaseAction
