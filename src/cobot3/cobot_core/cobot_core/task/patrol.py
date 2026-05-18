class Patrol:
    action_name = "patrol"

    def __init__(self, manager):
        self.manager = manager

    def execute(self, forward_duration=2.0, turn_duration=1.0, repeat=2, **kwargs):
        for _ in range(int(repeat)):
            if not self.manager.move_forward(duration=forward_duration):
                return False

            if not self.manager.rotate_left(duration=turn_duration):
                return False

        self.manager.stop()
        return True
