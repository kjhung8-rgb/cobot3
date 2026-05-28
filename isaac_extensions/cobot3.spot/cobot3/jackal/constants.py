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
# TODO(verify): browser-confirm the actual asset path in your Isaac Sim
# install. Common paths seen in the wild, relative to Isaac's assets root:
#   Isaac/Robots/Clearpath/Jackal/jackal.usd
#   Isaac/Robots/Clearpath/Jackal/Jackal.usd
JACKAL_USD_ASSET_REL_PATH = "Isaac/Robots/Clearpath/Jackal/jackal.usd"
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
JACKAL_LIDAR_TRANSLATION = (0.0, 0.0, 0.30)
JACKAL_LIDAR_TF_ROTATION_XYZW = (0.0, 0.0, 0.0, 1.0)

# OmniGraph paths.
JACKAL_CMD_VEL_GRAPH_PATH = "/World/Jackal_CmdVel_Graph"
JACKAL_ODOM_TF_GRAPH_PATH = "/World/Jackal_OdomTf_Graph"
JACKAL_SCAN_TF_GRAPH_PATH = "/World/Jackal_ScanTf_Graph"

# ─────────────────────────────────────────────
# Headlight (mirror of spot's pattern — SphereLight + ShapingAPI cone parented
# under jackal chassis so it follows motion). Translation/rotation defaults are
# sensible starting points; tune in Isaac UI then copy back here.
# ─────────────────────────────────────────────
JACKAL_HEADLIGHT_PRIM_PATH = f"{JACKAL_CHASSIS_PRIM_PATH}/headlight"
JACKAL_HEADLIGHT_TRANSLATION = (0.25, 0.0, 0.25)              # 사용자 Isaac UI 튜닝값 고정
JACKAL_HEADLIGHT_ROTATION_XYZ_DEG = (-0.06097, 90.94314, -0.06097)  # 사용자 튜닝
JACKAL_HEADLIGHT_SCALE = (0.95094, 5.26113, 3.36257)          # 사용자 튜닝 — Y/Z 늘려 시야 확장
JACKAL_HEADLIGHT_INTENSITY = 500000.0               # 1.5M → 500k (사용자 피드백: 너무 밝음)
JACKAL_HEADLIGHT_RADIUS = 0.08                      # jackal 작아서 emitter도 작게
JACKAL_HEADLIGHT_CONE_ANGLE_DEG = 30.0              # cone 미적용 (omni 모드) — 보존만
JACKAL_HEADLIGHT_COLOR_RGB = (1.0, 1.0, 0.92)       # warm white

# Rescue box: jackal 위에 얹는 빨간 상자 (visual only).
JACKAL_RESCUE_BOX_PATH = f"{JACKAL_CHASSIS_PRIM_PATH}/rescue_box"
JACKAL_RESCUE_BOX_TRANSLATION = (0.0, 0.0, 0.20)
JACKAL_RESCUE_BOX_ROTATION_XYZ_DEG = (0.0, 0.0, 0.0)
# Cube default size=2 — scale의 절반이 실제 dimension. 0.3 x 0.25 x 0.15 m 상자.
JACKAL_RESCUE_BOX_SCALE = (0.15, 0.125, 0.075)
JACKAL_RESCUE_BOX_COLOR_RGB = (0.9, 0.05, 0.05)     # 빨강
