"""Shared constants for the Cobot3 Spot Isaac Sim extension."""

ROBOT_NS = "spot_0"
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

SPOT_PRIM_PATH = "/World/Spot"
SPOT_BODY_PRIM_PATH = "/World/Spot/body"
FRONT_CAMERA_PRIM_PATH = "/World/Spot/body/front_camera"
LIDAR_PRIM_PATH = "/World/Spot/body/spot_lidar"

# Edit these when moving Spot's initial pose in the warehouse.
SPOT_SPAWN_POSITION = (21.5, 29.0, 0.8)
SPOT_SPAWN_YAW_DEG = 0.0

# Front camera pose relative to /World/Spot/body.
# USD cameras look along local -Z, so Y=-90 deg points the image forward
# along Spot body's +X direction.
FRONT_CAMERA_TRANSLATION = (0.5, 0.0, 0.3)
FRONT_CAMERA_ROTATION_XYZ_DEG = (90.0, 0.0, -90.0)

CMD_VEL_GRAPH_PATH = "/World/Spot_CmdVel_Graph"
CAMERA_GRAPH_PATH = "/World/Spot_Camera_Graph"
SLAM_GRAPH_PATH = "/World/Spot_SLAM_Graph"

DEFAULT_ROS_DOMAIN_ID = 141
