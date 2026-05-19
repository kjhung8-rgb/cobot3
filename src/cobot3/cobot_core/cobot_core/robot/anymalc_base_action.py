class AnymalCBaseAction(MobileBaseAction):
    robot_type = "anymalc"

    default_linear_speed = 0.25
    default_lateral_speed = 0.15
    default_angular_speed = 0.5

    def stop(self, **kwargs):
        self.publish_cmd(0.0, 0.0, 0.0)
        return True

    def move_forward(self, speed=None, duration=None, **kwargs):
        v = self.default_linear_speed if speed is None else float(speed)
        return self.run_cmd_for_duration(v, 0.0, 0.0, duration)

    def move_backward(self, speed=None, duration=None, **kwargs):
        v = self.default_linear_speed if speed is None else float(speed)
        return self.run_cmd_for_duration(-v, 0.0, 0.0, duration)

    def move_left(self, speed=None, duration=None, **kwargs):
        v = self.default_lateral_speed if speed is None else float(speed)
        return self.run_cmd_for_duration(0.0, v, 0.0, duration)

    def move_right(self, speed=None, duration=None, **kwargs):
        v = self.default_lateral_speed if speed is None else float(speed)
        return self.run_cmd_for_duration(0.0, -v, 0.0, duration)

    def rotate_left(self, speed=None, duration=None, **kwargs):
        w = self.default_angular_speed if speed is None else float(speed)
        return self.run_cmd_for_duration(0.0, 0.0, w, duration)

    def rotate_right(self, speed=None, duration=None, **kwargs):
        w = self.default_angular_speed if speed is None else float(speed)
        return self.run_cmd_for_duration(0.0, 0.0, -w, duration)

    def stand(self, **kwargs):
        self.node.get_logger().info("ANYmal C stand requested")
        return True

    def sit(self, **kwargs):
        self.node.get_logger().info("ANYmal C sit requested")
        self.stop()
        return True