# cobot_core/task/patrol.py

class Patrol:
    action_name = "patrol"

    def __init__(self, manager):
        self.manager = manager

    def execute(self, **kwargs):
        if not self.manager.move_forward(speed=0.25, duration=2.0):
            return False

        if not self.manager.rotate_left(speed=0.6, duration=1.0):
            return False

        if not self.manager.move_forward(speed=0.25, duration=2.0):
            return False

        if not self.manager.rotate_left(speed=0.6, duration=1.0):
            return False

        self.manager.stop()
        return True