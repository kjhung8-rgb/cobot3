from geometry_msgs.msg import Twist


class MobileBaseAction:
    """Common minimal base for mobile robots.

    공통에는 로봇별 동작 정책을 넣지 않는다.
    여기서는 cmd_vel publisher 생성과 Twist 발행만 담당한다.
    """

    robot_type = "mobile"
    default_namespace = "anymalc_0"

    def __init__(self, node):
        self.node = node

        namespace = self.node.get_namespace().strip("/")
        if not namespace:
            namespace = self.default_namespace

        self.robot_namespace = namespace
        self.cmd_vel_topic = f"/{namespace}/cmd_vel"
        self.cmd_pub = self.node.create_publisher(Twist, self.cmd_vel_topic, 10)

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
        self.cmd_pub.publish(msg)
        return True
