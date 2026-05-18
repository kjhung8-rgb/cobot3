from .base_action import create_base_action
from .task.patrol import Patrol


class ActionManager:
    def __init__(self, node):
        self.node = node
        self.base_action = create_base_action(node)
        self.actions = {}
        self._register_actions()

    def __getattr__(self, name):
        """Delegate unknown methods to selected robot base action.

        예: self.move_forward() -> self.base_action.move_forward()
        """
        return getattr(self.base_action, name)

    def _add_action_if_exists(self, name, method_name=None):
        method_name = method_name or name
        if hasattr(self.base_action, method_name):
            self.actions[name] = getattr(self.base_action, method_name)

    def _register_actions(self):
        # =========================
        # 저수준 이동 액션
        # =========================
        self._add_action_if_exists("stop")

        self._add_action_if_exists("move_forward")
        self._add_action_if_exists("forward", "move_forward")

        self._add_action_if_exists("move_backward")
        self._add_action_if_exists("backward", "move_backward")

        self._add_action_if_exists("rotate_left")
        self._add_action_if_exists("left", "rotate_left")

        self._add_action_if_exists("rotate_right")
        self._add_action_if_exists("right", "rotate_right")

        self._add_action_if_exists("wait")

        # Spot 등 특정 로봇 전용 액션. 없으면 등록 안 됨.
        self._add_action_if_exists("stand")
        self._add_action_if_exists("sit")

        # =========================
        # 고수준 task 액션
        # =========================
        self.actions[Patrol.action_name] = Patrol(self).execute

    def perform(self, action_name, **kwargs):
        action_name = str(action_name).strip()

        if action_name not in self.actions:
            self.node.get_logger().error(
                f"없는 액션: {action_name} / 사용 가능: {list(self.actions.keys())}"
            )
            return False

        try:
            self.node.get_logger().info(
                f"액션 실행: {action_name}, params={kwargs}"
            )

            result = self.actions[action_name](**kwargs)

            if result is False:
                self.node.get_logger().error(f"액션 실패: {action_name}")
                self.stop()
                return False

            return True

        except Exception as e:
            self.node.get_logger().error(
                f"액션 예외 발생: {action_name} / {e}"
            )
            self.stop()
            return False
