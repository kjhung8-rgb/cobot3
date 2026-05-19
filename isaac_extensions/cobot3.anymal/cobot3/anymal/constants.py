"""Shared constants for the Cobot3 ANYmal C Isaac Sim extension."""

ROBOT_NS = "anymal_0"
CMD_VEL_TOPIC = f"/{ROBOT_NS}/cmd_vel"

BASE_LINK_FRAME = f"{ROBOT_NS}/base_link"
LIDAR_FRAME = f"{ROBOT_NS}/lidar_link"
FRONT_CAM_FRAME = f"{ROBOT_NS}/front_cam_link"
ODOM_FRAME = "odom"
MAP_FRAME = "map"

SCAN_TOPIC = f"/{ROBOT_NS}/scan"
ODOM_TOPIC = f"/{ROBOT_NS}/odom"
COLOR_IMAGE_TOPIC = f"/{ROBOT_NS}/front_cam/color_image"
DEPTH_IMAGE_TOPIC = f"/{ROBOT_NS}/front_cam/depth_image"
CAMERA_INFO_TOPIC = f"/{ROBOT_NS}/front_cam/camera_info"

# ANYmal C prim paths (note: IsaacSim uses "Anymal" in standalone example)
# ANYMAL_PRIM_PATH  : top-level USD container prim (Xform); used by AnymalFlatTerrainPolicy
# ANYMAL_BODY_PRIM_PATH: ArticulationRoot prim (anymal_c.usd has root API at "base");
#                        used by IsaacComputeOdometry chassisPrim
ANYMAL_PRIM_PATH = "/World/Anymal"
ANYMAL_BODY_PRIM_PATH = "/World/Anymal/base"
FRONT_CAMERA_PRIM_PATH = "/World/Anymal/base/front_camera"
LIDAR_PRIM_PATH = "/World/Anymal/base/anymal_lidar"

# IsaacComputeOdometry needs the ArticulationRoot prim, not the container.
# ANYmal C USD has ArticulationRootAPI at /World/Anymal/base (confirmed by unit tests).
ODOMETRY_CHASSIS_PRIM_PATH = ANYMAL_BODY_PRIM_PATH

CMD_VEL_GRAPH_PATH = "/World/Anymal_CmdVel_Graph"
CAMERA_GRAPH_PATH = "/World/Anymal_Camera_Graph"
SLAM_GRAPH_PATH = "/World/Anymal_SLAM_Graph"

# ANYmal C spawn height (flat terrain): 0.7 m above ground
ANYMAL_SPAWN_HEIGHT = 0.7

# cmd_vel velocity limits fed to the policy
# ANYmal C RL policy uses normalized commands: magnitude ~1.0 = full speed
# Keep well below 1.0 when coming from Nav2 to avoid instability
VX_MAX = 0.5   # m/s forward
VX_MIN = -0.3  # m/s backward
VY_MAX = 0.3   # m/s lateral
VY_MIN = -0.3
WZ_MAX = 0.5   # rad/s yaw
WZ_MIN = -0.5

# Scale: ANYmal policy base_command = [fwd, lat, yaw] in normalized units.
# AnymalFlatTerrainPolicy uses the same convention as anymal_standalone.py:
# command [1.0, 0.0, 0.0] → full forward speed (~0.5-0.6 m/s in real world).
# Multiply by a scale so Nav2 m/s maps to appropriate policy command magnitude.
CMD_VEL_SCALE = 1.0  # start at 1:1, tune if ANYmal is too slow/fast

DEFAULT_ROS_DOMAIN_ID = 141
