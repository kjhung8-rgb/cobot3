from .robot.jetbot_base_action import JetbotBaseAction
from .robot.spot_base_action import SpotBaseAction
from .robot.anymalc_base_action import AnymalCBaseAction


ROBOT_BASE_ACTIONS = {
    "jetbot": JetbotBaseAction,
    "spot": SpotBaseAction,
    "anymalc": AnymalCBaseAction,
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
      anymalc_0  -> anymalc
      anymalc_12 -> anymalc
      anymalc    -> anymalc
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
    if name.startswith("anymalc"):
        return "anymalc"

    return name


def infer_robot_type(node):
    robot_type = _get_robot_type_param(node)
    if robot_type and robot_type != "auto":
        return _normalize_robot_type(robot_type)

    namespace = node.get_namespace().strip("/").lower()
    if namespace:
        return _normalize_robot_type(namespace)

    return "anymalc"  # default fallback


def create_base_action(node):
    robot_type = infer_robot_type(node)
    action_cls = ROBOT_BASE_ACTIONS.get(robot_type)

    if action_cls is None:
        node.get_logger().warn(
            f"알 수 없는 robot_type='{robot_type}'. AnymalCBaseAction으로 fallback"
        )
        action_cls = AnymalCBaseAction
        robot_type = " anymalc"

    node.get_logger().info(f"🤖 Selected robot base action: {robot_type}")
    return action_cls(node)


# 기존 코드 호환용 alias.
BaseAction = AnymalCBaseAction
