from .robot.jetbot_base_action import JetbotBaseAction
from .robot.spot_base_action import SpotBaseAction


ROBOT_BASE_ACTIONS = {
    "jetbot": JetbotBaseAction,
    "spot": SpotBaseAction,
}


def _get_robot_type_param(node):
    """Return robot_type parameter value if available, otherwise 'auto'."""
    try:
        return str(node.get_parameter("robot_type").value).strip().lower()
    except Exception:
        return "auto"


def infer_robot_type(node):
    """Infer robot type from ROS parameter first, then namespace.

    Priority:
      1. robot_type parameter if not empty and not 'auto'
      2. namespace name, e.g. /jetbot or /spot
      3. fallback to spot
    """
    robot_type = _get_robot_type_param(node)
    if robot_type and robot_type != "auto":
        return robot_type

    namespace = node.get_namespace().strip("/").lower()
    if namespace:
        return namespace

    return "spot"  # default fallback


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
# 예전 파일이 `from .base_action import BaseAction` 해도 바로 죽지 않게 둠.
BaseAction = SpotBaseAction
