from geometry_msgs.msg import Twist


class MobileBaseAction:
    """Common minimal base for mobile robots.

    여기에는 로봇별 동작 정책을 넣지 않는다.
    공통으로 필요한 cmd_vel publisher 생성과 Twist 발행만 담당한다.
    """

    robot_type = "mobile"
    cmd_vel_topic = "cmd_vel"  # relative topic: /{namespace}/cmd_vel 로 발행됨

    def __init__(self, node):
        self.node = node
        self.cmd_pub = self.node.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.node.get_logger().info(
            f"✅ {self.__class__.__name__} ready / publish: "
            f"{self.node.get_namespace().rstrip('/')}/{self.cmd_vel_topic}"
        )

    def publish_cmd(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)
        return True
