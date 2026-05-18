from geometry_msgs.msg import Twist


class MobileBaseAction:
    """
    Common ROS2 cmd_vel publisher layer.

    Only shared responsibility here:
      - choose cmd_vel topic from node namespace
      - publish geometry_msgs/msg/Twist

    Robot-specific actions such as move_forward/stop/rotate_left belong in
    JetbotBaseAction or SpotBaseAction.
    """

    robot_type = "mobile"

    def __init__(self, node):
        self.node = node

        namespace = node.get_namespace().strip("/")
        if namespace:
            self.robot_name = namespace
        else:
            self.robot_name = getattr(self, "robot_type", "robot")

        self.cmd_vel_topic = f"/{self.robot_name}/cmd_vel"
        self.cmd_vel_pub = self.node.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.node.get_logger().info(
            f"✅ {self.__class__.__name__} ready / publish: {self.cmd_vel_topic}"
        )

    def publish_cmd(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)
        return True
