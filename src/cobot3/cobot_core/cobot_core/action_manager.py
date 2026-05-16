from .base_action import BaseAction
from .tasks.patrol import Patrol



class ActionManager(BaseAction):
    def __init__(self, node):
        super().__init__(node)

        self.actions = {}
        self._register_actions()

    def _register_actions(self):
        # 저수준 액션
        self.actions["stop"] = self.stop
        self.actions["move_forward"] = self.move_forward
        self.actions["move_backward"] = self.move_backward
        self.actions["rotate_left"] = self.rotate_left
        self.actions["rotate_right"] = self.rotate_right

        # 고수준 task 추가
        self.actions[Patrol.action_name] = Patrol(self).execute


    def perform(self, action_name, **kwargs):
        if action_name not in self.actions:
            self.node.get_logger().error(f"없는 액션: {action_name}")
            return False

        try:
            self.node.get_logger().info(f"액션 실행: {action_name}, params={kwargs}")
            result = self.actions[action_name](**kwargs)

            if result is False:
                self.node.get_logger().error(f"액션 실패: {action_name}")
                self.stop()
                return False

            return True

        except Exception as e:
            self.node.get_logger().error(f"액션 예외 발생: {action_name} / {e}")
            self.stop()
            return False