# ~/dev_ws/src/artbot_motion/artbot_motion/action_manager.py

from .base_action import BaseAction


class ActionManager(BaseAction):
    def __init__(self, node):
        super().__init__(node)
        self.actions = {}
        self._register_actions()

    def _register_actions(self):
        self.actions["stop"] = self.stop
        self.actions["move_forward"] = self.move_forward
        self.actions["move_backward"] = self.move_backward
        self.actions["rotate_left"] = self.rotate_left
        self.actions["rotate_right"] = self.rotate_right
        self.actions["wait"] = self.wait

        # logical actions
        self.actions["patrol"] = self.patrol
        self.actions["greeting_motion"] = self.greeting_motion

    def perform(self, action_name, **kwargs):
        if action_name not in self.actions:
            self.node.get_logger().error(f"없는 액션임: {action_name}")
            return False

        self.node.get_logger().info(f"액션 실행: {action_name}, params={kwargs}")

        try:
            result = self.actions[action_name](**kwargs)
            if result is False:
                self.node.get_logger().error(f"액션 실패: {action_name}")
                return False
            return True

        except Exception as e:
            self.node.get_logger().error(f"액션 예외 발생: {action_name} / {e}")
            self.stop()
            return False

    def patrol(self):
        """
        간단 순찰 모션:
        전진 → 회전 → 전진 → 회전
        """
        if not self.move_forward(speed=0.25, duration=2.0): return False
        if not self.rotate_left(speed=0.7, duration=1.2): return False
        if not self.move_forward(speed=0.25, duration=2.0): return False
        if not self.rotate_left(speed=0.7, duration=1.2): return False
        return True

    def greeting_motion(self):
        """
        사용자 앞에서 인사하는 느낌:
        좌우로 살짝 흔들기
        """
        if not self.rotate_left(speed=0.6, duration=0.5): return False
        if not self.rotate_right(speed=0.6, duration=1.0): return False
        if not self.rotate_left(speed=0.6, duration=0.5): return False
        self.stop()
        return True