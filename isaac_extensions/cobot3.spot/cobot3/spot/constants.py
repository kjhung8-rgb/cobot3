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
# 90° 간격 + 90° FOV — 같은 위치(같은 시점) 기준이라 인접 image가 빈틈/중복
# 없이 정확히 tile됨. 총 270° 파노라마 (좌 90°, 중앙 90°, 우 90°).
#   Front: rotZ=-90 (→ 카메라 +X)
#   Left:  rotZ=0   (Front 대비 +90° yaw)
#   Right: rotZ=180 (Front 대비 -90° yaw)
FRONT_CAMERA_ROTATION_XYZ_DEG = (90.0, 0.0, -90.0)
LEFT_CAMERA_ROTATION_XYZ_DEG = (90.0, 0.0, 0.0)
RIGHT_CAMERA_ROTATION_XYZ_DEG = (90.0, 0.0, 180.0)

# ROS *_cam_link TF — base_link 대비 yaw 회전 (camera_link는 x-forward 규약).
#   Front: yaw 0
#   Left:  yaw +90° → quat z = sin(45°) ≈ 0.7071, w = cos(45°) ≈ 0.7071
#   Right: yaw -90° → quat z = -0.7071, w = 0.7071
FRONT_CAMERA_TF_ROTATION_XYZW = (0.0, 0.0, 0.0, 1.0)
LEFT_CAMERA_TF_ROTATION_XYZW = (0.0, 0.0, 0.7071067811865475, 0.7071067811865476)
RIGHT_CAMERA_TF_ROTATION_XYZW = (0.0, 0.0, -0.7071067811865475, 0.7071067811865476)
FRONT_CAMERA_RENDER_WIDTH = 640
FRONT_CAMERA_RENDER_HEIGHT = 360

# 90° 간격 + 90° FOV → exact tile (overlap 0, gap 0). 총 270° 파노라마.
# Pinhole: hFOV = 2·atan(hAperture / (2·focalLength))
#   focalLength 18.14756 mm + hAperture 36.29512 → hFOV = 90°
CAMERA_FOCAL_LENGTH_MM = 18.14756
CAMERA_HORIZONTAL_APERTURE_MM = 36.29512  # → hFOV 90°
CAMERA_VERTICAL_APERTURE_MM = 20.42       # 16:9 비율 유지

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
HEADLIGHT_TRANSLATION = (0.89904, 0.13904, 0.10086)      # 사용자 Isaac UI 튜닝값 고정
HEADLIGHT_ROTATION_XYZ_DEG = (60.0, 367.0, -100.0)        # 사용자 Isaac UI 튜닝값 고정
HEADLIGHT_INTENSITY = 2000000.0                  # 2.5배 더 밝게 (구조 시나리오에서 어두운 창고 비춤)
HEADLIGHT_RADIUS = 0.1                           # 더 큰 emitter → 부드러운 빛
HEADLIGHT_CONE_ANGLE_DEG = 30.0                  # 콘 반각 30° (총 60°): 좁고 집중된 정면 빔
HEADLIGHT_COLOR_RGB = (1.0, 1.0, 0.92)           # 약간 warm white

# ─────────────────────────────────────────────
# Fire lights — 화재 분위기 SphereLight 8개. /World/FireLights/* 아래 생성.
# 코드로 박는 이유: USD 저장본(g8_1.usd)이 save_as_stage overwrite로 깨졌고
# 다시는 같은 함정 안 빠지려고 git-tracked로 박음. 위치 튜닝 후 좌표 알려주면 갱신.
# ─────────────────────────────────────────────
FIRE_LIGHTS_GROUP_PATH = "/World/FireLights"
FIRE_LIGHT_INTENSITY = 150000.0
FIRE_LIGHT_RADIUS = 0.4
FIRE_LIGHT_COLOR_RGB = (1.0, 0.25, 0.05)
# (prim_name, (x, y, z))
FIRE_LIGHT_POSITIONS = (
    # 17개 — 사용자 Isaac UI 튜닝값 고정 (2026-05-27 dump)
    ("fire_01", (18.9994, 11.6080, 5.1846)),
    ("fire_02", (15.0000, 36.0000, 0.8000)),
    ("fire_03", (28.0000, 22.0000, 4.3697)),
    ("fire_04", (26.91861, 37.59353, 0.8000)),
    ("fire_05", (10.0000, 29.1747, 0.3780)),
    ("fire_06", (32.0000, 29.0000, 1.2000)),
    ("fire_07", (-2.7403, 18.6205, 0.8967)),
    ("fire_08", (21.5000, 40.0000, 1.0000)),
    ("fire_09", (16.6408, -28.5853, 0.0592)),
    ("fire_10", (31.6303, -28.5853, 0.0592)),
    ("fire_11", (31.6303, -1.8247, 0.0592)),
    ("fire_12", (33.1289, -1.8247, 7.7100)),
    ("fire_13", (24.7319, 2.6662, 0.7454)),
    ("fire_14", (6.6743, -14.5536, 0.7454)),
    ("fire_15", (-8.8384, 0.0025, 0.6169)),
    ("fire_16", (1.0378, 5.6983, 5.8197)),
    ("fire_17", (-5.5844, -20.5139, 0.6169)),
)

# 천장 조명 — warehouse 내부 격자 SphereLight (호러식 깜빡임 자동 적용).
# 기존 warehouse USD의 RectLight 6개와 별개로 추가.
CEILING_LIGHTS_GROUP_PATH = "/World/CeilingLights"
CEILING_LIGHT_INTENSITY = 80000.0       # 깜빡일 때 ON intensity
CEILING_LIGHT_RADIUS = 0.3
CEILING_LIGHT_COLOR_RGB = (1.0, 0.98, 0.95)  # 약간 cool white
# 천장 z=4.5m, warehouse 내부 격자
CEILING_LIGHT_POSITIONS = (
    ("ceil_01", (14.0, 22.0, 4.5)),
    ("ceil_02", (14.0, 30.0, 4.5)),
    ("ceil_03", (14.0, 38.0, 4.5)),
    ("ceil_04", (22.0, 22.0, 4.5)),
    ("ceil_05", (22.0, 30.0, 4.5)),
    ("ceil_06", (22.0, 38.0, 4.5)),
    ("ceil_07", (30.0, 22.0, 4.5)),
    ("ceil_08", (30.0, 30.0, 4.5)),
    ("ceil_09", (30.0, 38.0, 4.5)),
)

CMD_VEL_GRAPH_PATH = "/World/Spot_CmdVel_Graph"
CAMERA_GRAPH_PATH = "/World/Spot_Camera_Graph"
SLAM_GRAPH_PATH = "/World/Spot_SLAM_Graph"

# ROS cmd_vel to Spot policy command scale.
CMD_VEL_TO_POLICY_LINEAR_X_SCALE = 2.0
CMD_VEL_TO_POLICY_LINEAR_Y_SCALE = 2.0
CMD_VEL_TO_POLICY_ANGULAR_Z_SCALE = 2.0
