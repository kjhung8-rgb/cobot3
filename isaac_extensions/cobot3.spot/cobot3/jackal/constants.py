"""Constants for the optional Clearpath Jackal secondary robot.

Same architecture as Carter (differential drive, 2D LiDAR), tuned for Jackal's
smaller footprint (0.51 x 0.43 m) and lighter dynamics. Jackal and Carter are
mutually exclusive — pick one before calling load_world_async().
"""

JACKAL_NS = "jackal_0"
JACKAL_CMD_VEL_TOPIC = f"/{JACKAL_NS}/cmd_vel"
JACKAL_ODOM_TOPIC = f"/{JACKAL_NS}/odom"
JACKAL_SCAN_TOPIC = f"/{JACKAL_NS}/scan"

JACKAL_BASE_LINK_FRAME = f"{JACKAL_NS}/base_link"
JACKAL_ODOM_FRAME = f"{JACKAL_NS}/odom"
JACKAL_LIDAR_FRAME = f"{JACKAL_NS}/laser"

# Stage prim paths. Verify after first import; some Clearpath USDs nest the
# chassis under a different name (base_link vs chassis_link).
JACKAL_PRIM_PATH = "/World/Jackal"
JACKAL_CHASSIS_PRIM_PATH = f"{JACKAL_PRIM_PATH}/base_link"
# TODO(verify): browser-confirm the actual Nucleus path in your Isaac Sim
# install. Common locations seen in the wild:
#   /Isaac/Robots/Clearpath/Jackal/jackal.usd
#   /Isaac/Robots/Clearpath/Jackal/Jackal.usd
JACKAL_USD_NUCLEUS_PATH = "/Isaac/Robots/Clearpath/Jackal/jackal.usd"
JACKAL_SPAWN_POSITION = (24.0, 29.0, 0.2)
JACKAL_SPAWN_YAW_DEG = 0.0

# Differential drive params (Jackal nominal specs from Clearpath URDF).
# Jackal is 4-wheel skid-steer. DifferentialController outputs [L, R] (2 values),
# so the cmd_vel OmniGraph expands it to [L, R, L, R] via ArrayGetItem+ConstructArray
# nodes before reaching ArtController. This order matches the joint name list below:
#   [front_left, front_right, rear_left, rear_right] ← [L, R, L, R] mapping.
# If your USD has different joint names, dump them with:
#   stage.GetPrimAtPath(JACKAL_PRIM_PATH).GetAllChildren() and inspect.
JACKAL_WHEEL_JOINT_NAMES = [
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint",
]
JACKAL_WHEEL_RADIUS = 0.098
JACKAL_WHEEL_DISTANCE = 0.37
JACKAL_MAX_LINEAR_SPEED = 2.0   # m/s (Jackal real-world spec ~2.0)
JACKAL_MAX_ANGULAR_SPEED = 2.5  # rad/s

# LiDAR mounting on /World/Jackal — sensor prim auto-discovered at setup.
# Hints match common Jackal USD lidar prim names.
JACKAL_LIDAR_SENSOR_HINTS = ("front_laser", "laser", "lidar", "sick")
JACKAL_LIDAR_TRANSLATION = (0.0, 0.0, 0.25)
JACKAL_LIDAR_TF_ROTATION_XYZW = (0.0, 0.0, 0.0, 1.0)

# OmniGraph paths.
JACKAL_CMD_VEL_GRAPH_PATH = "/World/Jackal_CmdVel_Graph"
JACKAL_ODOM_TF_GRAPH_PATH = "/World/Jackal_OdomTf_Graph"
JACKAL_SCAN_TF_GRAPH_PATH = "/World/Jackal_ScanTf_Graph"
