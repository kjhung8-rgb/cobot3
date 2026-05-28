"""Constants for the optional Nova Carter secondary robot.

Co-spawned with Spot; shares the same ROS_DOMAIN_ID. Namespace isolation
via /carter_0 prefixes keeps Carter apart from spot's /spot_0 topics.
"""

CARTER_NS = "carter_0"
CARTER_CMD_VEL_TOPIC = f"/{CARTER_NS}/cmd_vel"
CARTER_ODOM_TOPIC = f"/{CARTER_NS}/odom"
CARTER_SCAN_TOPIC = f"/{CARTER_NS}/scan"

CARTER_BASE_LINK_FRAME = f"{CARTER_NS}/base_link"
# Namespaced odom frame so it doesn't clash with spot's plain "odom" frame.
CARTER_ODOM_FRAME = f"{CARTER_NS}/odom"
CARTER_LIDAR_FRAME = f"{CARTER_NS}/laser"

# Stage prim paths.
CARTER_PRIM_PATH = "/World/Carter"
CARTER_CHASSIS_PRIM_PATH = f"{CARTER_PRIM_PATH}/chassis_link"
# BASE nova_carter.usd — Nova_Carter_ROS.usd's built-in publishers would
# clash with our OmniGraph stack. Relative to Isaac's assets root.
CARTER_USD_ASSET_REL_PATH = "Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
CARTER_SPAWN_POSITION = (24.0, 29.0, 0.5)
CARTER_SPAWN_YAW_DEG = 0.0

# Differential drive params (Nova Carter nominal — tune if drive feels off).
CARTER_WHEEL_JOINT_NAMES = ["joint_wheel_left", "joint_wheel_right"]
CARTER_WHEEL_RADIUS = 0.14
CARTER_WHEEL_DISTANCE = 0.42
CARTER_MAX_LINEAR_SPEED = 1.0   # m/s  (0.0 = unlimited)
CARTER_MAX_ANGULAR_SPEED = 1.5  # rad/s (0.0 = unlimited)

# LiDAR mounting on /World/Carter — sensor prim auto-discovered at setup.
CARTER_LIDAR_SENSOR_HINTS = ("front_2d_lidar", "2d_lidar", "front_lidar")
CARTER_LIDAR_TRANSLATION = (0.0, 0.0, 0.4)
CARTER_LIDAR_TF_ROTATION_XYZW = (0.0, 0.0, 0.0, 1.0)

# OmniGraph paths.
CARTER_CMD_VEL_GRAPH_PATH = "/World/Carter_CmdVel_Graph"
CARTER_ODOM_TF_GRAPH_PATH = "/World/Carter_OdomTf_Graph"
CARTER_SCAN_TF_GRAPH_PATH = "/World/Carter_ScanTf_Graph"
