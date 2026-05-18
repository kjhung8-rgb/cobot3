# ~/dev_ws/src/artbot_motion/artbot_motion/base_action.py

import time
from geometry_msgs.msg import Twist


class JetbotBaseAction:
    def __init__(self, node):
        self.node = node
        self.cmd_pub = node.create_publisher(Twist, "cmd_vel", 10)

    def publish_cmd(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def stop(self):
        self.publish_cmd(0.0, 0.0)
        return True
    
    def move_forward(self, speed=0.2, duration=1.0):
        start = time.time()

        while time.time() - start < duration:
            self.publish_cmd(linear_x=speed, angular_z=0.0)
            time.sleep(0.05)

        self.stop()
        return True

    def move_backward(self, speed=0.2, duration=1.0):
        return self.move_forward(speed=-abs(speed), duration=duration)

    def rotate_left(self, speed=0.6, duration=1.0):
        start = time.time()

        while time.time() - start < duration:
            self.publish_cmd(linear_x=0.0, angular_z=abs(speed))
            time.sleep(0.05)

        self.stop()
        return True

    def rotate_right(self, speed=0.6, duration=1.0):
        start = time.time()

        while time.time() - start < duration:
            self.publish_cmd(linear_x=0.0, angular_z=-abs(speed))
            time.sleep(0.05)

        self.stop()
        return True

    def wait(self, duration=1.0):
        self.stop()
        time.sleep(duration)
        return True