import time

from .mobile_base_action import MobileBaseAction


class JetbotBaseAction(MobileBaseAction):
    """JetBot 전용 저수준 액션."""

    robot_type = "jetbot"
    default_linear_speed = 0.2
    default_angular_speed = 0.6

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
