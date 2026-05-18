import time

from .mobile_base_action import MobileBaseAction


class SpotBaseAction(MobileBaseAction):
    """Spot 전용 저수준 액션.

    현재는 Isaac Sim/ROS2 cmd_vel 기반 placeholder 구현.
    나중에 실제 Spot SDK를 붙이면 이 파일 내부 구현만 교체하면 된다.
    """

    robot_type = "spot"
    default_linear_speed = 0.25
    default_angular_speed = 0.5

    def _run_for_duration(self, linear_x=0.0, angular_z=0.0, duration=None):
        if duration is None:
            return self.publish_cmd(linear_x, angular_z)

        end_time = time.time() + float(duration)
        while time.time() < end_time:
            self.publish_cmd(linear_x, angular_z)
            time.sleep(0.05)

        self.stop()
        return True

    def stop(self, **kwargs):
        return self.publish_cmd(0.0, 0.0)

    def move_forward(self, speed=None, duration=1.0, **kwargs):
        v = self.default_linear_speed if speed is None else float(speed)
        return self._run_for_duration(linear_x=abs(v), angular_z=0.0, duration=duration)

    def move_backward(self, speed=None, duration=1.0, **kwargs):
        v = self.default_linear_speed if speed is None else float(speed)
        return self._run_for_duration(linear_x=-abs(v), angular_z=0.0, duration=duration)

    def rotate_left(self, speed=None, duration=1.0, **kwargs):
        w = self.default_angular_speed if speed is None else float(speed)
        return self._run_for_duration(linear_x=0.0, angular_z=abs(w), duration=duration)

    def rotate_right(self, speed=None, duration=1.0, **kwargs):
        w = self.default_angular_speed if speed is None else float(speed)
        return self._run_for_duration(linear_x=0.0, angular_z=-abs(w), duration=duration)

    def wait(self, duration=1.0, **kwargs):
        self.stop()
        time.sleep(float(duration))
        return True

    def stand(self, **kwargs):
        self.node.get_logger().info("Spot stand requested - sim placeholder")
        return True

    def sit(self, **kwargs):
        self.node.get_logger().info("Spot sit requested - sim placeholder")
        self.stop()
        return True
