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

# Front-facing headlight on Spot's head. Parented under /World/Spot/body so it
# moves with the robot. SphereLight + ShapingAPI gives a directional cone.
# ShapingAPI cone narrows the light → intensity must be MUCH higher than a
# bare SphereLight to be visible. Warehouse scenes also need viewport's
# "Lights Off" toggle to be set to ON.
HEADLIGHT_PRIM_PATH = "/World/Spot/body/headlight"
HEADLIGHT_TRANSLATION = (0.5, 0.0, 0.35)        # 앞·중앙·머리 위
HEADLIGHT_ROTATION_XYZ_DEG = (0.0, 90.0, 0.0)   # -Z 기본 cone을 +X(전방)로
HEADLIGHT_INTENSITY = 800000.0                   # cone으로 좁힌 효과를 보상 (10배 상향)
HEADLIGHT_RADIUS = 0.1                           # 더 큰 emitter → 부드러운 빛
HEADLIGHT_CONE_ANGLE_DEG = 60.0                  # cone 반각 (전방 시야 충분히 커버)
HEADLIGHT_COLOR_RGB = (1.0, 1.0, 0.92)           # 약간 warm white

CMD_VEL_GRAPH_PATH = "/World/Spot_CmdVel_Graph"
CAMERA_GRAPH_PATH = "/World/Spot_Camera_Graph"
SLAM_GRAPH_PATH = "/World/Spot_SLAM_Graph"

# ROS cmd_vel to Spot policy command scale.
CMD_VEL_TO_POLICY_LINEAR_X_SCALE = 2.0
CMD_VEL_TO_POLICY_LINEAR_Y_SCALE = 2.0
CMD_VEL_TO_POLICY_ANGULAR_Z_SCALE = 2.0
