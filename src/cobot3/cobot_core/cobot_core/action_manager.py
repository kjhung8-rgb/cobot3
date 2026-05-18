from .base_action import create_base_action
from .task.patrol import Patrol


class ActionManager:
    """고수준 action/task를 등록하고 실행하는 공통 매니저.

    로봇별 저수준 동작은 self.base_action에 위임한다.
    따라서 Patrol 같은 task 파일은 JetBot/Spot 구분 없이
    manager.move_forward(), manager.rotate_left() 같은 공통 메서드만 호출하면 된다.
    """

    def __init__(self, node):
        self.node = node
        self.base_action = create_base_action(node)
        self.actions = {}
        self._register_actions()

    # =========================
    # 로봇별 저수준 액션 위임
    # =========================
    def stop(self, **kwargs):
        return self.base_action.stop(**kwargs)

    def move_forward(self, **kwargs):
        return self.base_action.move_forward(**kwargs)

    def move_backward(self, **kwargs):
        return self.base_action.move_backward(**kwargs)

    def rotate_left(self, **kwargs):
        return self.base_action.rotate_left(**kwargs)

    def rotate_right(self, **kwargs):
        return self.base_action.rotate_right(**kwargs)

    def wait(self, **kwargs):
        return self.base_action.wait(**kwargs)

    def publish_cmd(self, **kwargs):
        return self.base_action.publish_cmd(**kwargs)

    def _register_actions(self):
        # =========================
        # 저수준 이동 액션
        # =========================
        self.actions["stop"] = self.stop

        self.actions["move_forward"] = self.move_forward
        self.actions["forward"] = self.move_forward

        self.actions["move_backward"] = self.move_backward
        self.actions["backward"] = self.move_backward

        self.actions["rotate_left"] = self.rotate_left
        self.actions["left"] = self.rotate_left

        self.actions["rotate_right"] = self.rotate_right
        self.actions["right"] = self.rotate_right

        self.actions["wait"] = self.wait

        # =========================
        # 고수준 task 액션
        # =========================
        self.actions[Patrol.action_name] = Patrol(self).execute

    def perform(self, action_name, **kwargs):
        action_name = action_name.strip()

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
