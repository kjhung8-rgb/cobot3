"""Shared constants for the Cobot3 Spot Isaac Sim extension."""

ROBOT_NS = "spot_0"
CMD_VEL_TOPIC = f"/{ROBOT_NS}/cmd_vel"

BASE_LINK_FRAME = f"{ROBOT_NS}/base_link"
LIDAR_FRAME = f"{ROBOT_NS}/lidar_link"
FRONT_CAM_FRAME = f"{ROBOT_NS}/front_cam_link"
LEFT_CAM_FRAME = f"{ROBOT_NS}/left_cam_link"
RIGHT_CAM_FRAME = f"{ROBOT_NS}/right_cam_link"
ODOM_FRAME = "odom"
MAP_FRAME = "map"

SCAN_TOPIC = f"/{ROBOT_NS}/scan"
ODOM_TOPIC = f"/{ROBOT_NS}/odom"
COLOR_IMAGE_TOPIC = f"/{ROBOT_NS}/front_cam/color_image"
DEPTH_IMAGE_TOPIC = f"/{ROBOT_NS}/front_cam/depth_image"
CAMERA_INFO_TOPIC = f"/{ROBOT_NS}/front_cam/camera_info"
LEFT_COLOR_IMAGE_TOPIC = f"/{ROBOT_NS}/left_cam/color_image"
LEFT_DEPTH_IMAGE_TOPIC = f"/{ROBOT_NS}/left_cam/depth_image"
LEFT_CAMERA_INFO_TOPIC = f"/{ROBOT_NS}/left_cam/camera_info"
RIGHT_COLOR_IMAGE_TOPIC = f"/{ROBOT_NS}/right_cam/color_image"
RIGHT_DEPTH_IMAGE_TOPIC = f"/{ROBOT_NS}/right_cam/depth_image"
RIGHT_CAMERA_INFO_TOPIC = f"/{ROBOT_NS}/right_cam/camera_info"

SPOT_PRIM_PATH = "/World/Spot"
SPOT_BODY_PRIM_PATH = "/World/Spot/body"
FRONT_CAMERA_PRIM_PATH = "/World/Spot/body/front_camera"
LEFT_CAMERA_PRIM_PATH = "/World/Spot/body/left_camera"
RIGHT_CAMERA_PRIM_PATH = "/World/Spot/body/right_camera"
LIDAR_PRIM_PATH = "/World/Spot/body/spot_lidar"

# Edit these when moving Spot's initial pose in the warehouse.
SPOT_SPAWN_POSITION = (21.5, 29.0, 0.8)
SPOT_SPAWN_YAW_DEG = 0.0

# Front camera pose relative to /World/Spot/body.
FRONT_CAMERA_TRANSLATION = (0.5, 0.0, 0.3)
LEFT_CAMERA_TRANSLATION = (0.25, 0.22, 0.3)
RIGHT_CAMERA_TRANSLATION = (0.25, -0.22, 0.3)

# USD cameras look along local -Z with local +Y as image-up. This points the
# optical axis along Spot body's +X direction and keeps image-up near +Z.
FRONT_CAMERA_ROTATION_XYZ_DEG = (90.0, 0.0, -90.0)
LEFT_CAMERA_ROTATION_XYZ_DEG = (90.0, 0.0, 0.0)
RIGHT_CAMERA_ROTATION_XYZ_DEG = (90.0, 0.0, 180.0)

# ROS algorithms use each *_cam_link as a logical x-forward camera frame.
FRONT_CAMERA_TF_ROTATION_XYZW = (0.0, 0.0, 0.0, 1.0)
LEFT_CAMERA_TF_ROTATION_XYZW = (0.0, 0.0, 0.70710678118, 0.70710678118)
RIGHT_CAMERA_TF_ROTATION_XYZW = (0.0, 0.0, -0.70710678118, 0.70710678118)
FRONT_CAMERA_RENDER_WIDTH = 640
FRONT_CAMERA_RENDER_HEIGHT = 360

CAMERA_SPECS = (
    {
        "name": "front",
        "label": "전방",
        "prim_path": FRONT_CAMERA_PRIM_PATH,
        "frame": FRONT_CAM_FRAME,
        "translation": FRONT_CAMERA_TRANSLATION,
        "usd_rotation_xyz_deg": FRONT_CAMERA_ROTATION_XYZ_DEG,
        "tf_rotation_xyzw": FRONT_CAMERA_TF_ROTATION_XYZW,
        "color_topic": COLOR_IMAGE_TOPIC,
        "depth_topic": DEPTH_IMAGE_TOPIC,
        "camera_info_topic": CAMERA_INFO_TOPIC,
    },
    {
        "name": "left",
        "label": "좌측",
        "prim_path": LEFT_CAMERA_PRIM_PATH,
        "frame": LEFT_CAM_FRAME,
        "translation": LEFT_CAMERA_TRANSLATION,
        "usd_rotation_xyz_deg": LEFT_CAMERA_ROTATION_XYZ_DEG,
        "tf_rotation_xyzw": LEFT_CAMERA_TF_ROTATION_XYZW,
        "color_topic": LEFT_COLOR_IMAGE_TOPIC,
        "depth_topic": LEFT_DEPTH_IMAGE_TOPIC,
        "camera_info_topic": LEFT_CAMERA_INFO_TOPIC,
    },
    {
        "name": "right",
        "label": "우측",
        "prim_path": RIGHT_CAMERA_PRIM_PATH,
        "frame": RIGHT_CAM_FRAME,
        "translation": RIGHT_CAMERA_TRANSLATION,
        "usd_rotation_xyz_deg": RIGHT_CAMERA_ROTATION_XYZ_DEG,
        "tf_rotation_xyzw": RIGHT_CAMERA_TF_ROTATION_XYZW,
        "color_topic": RIGHT_COLOR_IMAGE_TOPIC,
        "depth_topic": RIGHT_DEPTH_IMAGE_TOPIC,
        "camera_info_topic": RIGHT_CAMERA_INFO_TOPIC,
    },
)

CMD_VEL_GRAPH_PATH = "/World/Spot_CmdVel_Graph"
CAMERA_GRAPH_PATH = "/World/Spot_Camera_Graph"
SLAM_GRAPH_PATH = "/World/Spot_SLAM_Graph"

DEFAULT_ROS_DOMAIN_ID = 141

# ─────────────────────────────────────────────
# Carter (Nova Carter) — co-spawned by SpotFireRescue.setup_scene().
# Shares the same ROS_DOMAIN_ID as spot; namespace isolation via /carter_0.
# ─────────────────────────────────────────────
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
# clash with our OmniGraph stack.
CARTER_USD_NUCLEUS_PATH = "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
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

# ROS cmd_vel to Spot policy command scale.
CMD_VEL_TO_POLICY_LINEAR_X_SCALE = 2.0
CMD_VEL_TO_POLICY_LINEAR_Y_SCALE = 2.0
CMD_VEL_TO_POLICY_ANGULAR_Z_SCALE = 2.0
