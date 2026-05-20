"""
spot_yolo_demo.py
Spot 로봇의 front RGB 카메라로 bus.jpg 속 사람을 YOLO로 감지.
- Spot USD: NVIDIA S3 에셋 자동 다운로드
- 수직 평면에 bus.jpg 텍스처 → Spot 전방에 배치
- 카메라를 월드에 독립 배치 (Spot body 계층 외부)

실행:
  cd /home/kim/dev_ws/isaac_sim/duck
  /home/kim/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh spot_yolo_demo.py
"""
import os
import cv2
import numpy as np
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.usd
from pxr import UsdGeom, UsdShade, UsdLux, Sdf, Gf
from isaacsim.core.api import World
from isaacsim.core.api.materials.omni_pbr import OmniPBR
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.sensors.camera import Camera
from isaacsim.storage.native import get_assets_root_path
import isaacsim.core.utils.numpy.rotations as rot_utils
from ultralytics import YOLO

# ── 경로 ────────────────────────────────────────────────────
SCRIPT_DIR    = os.path.dirname(os.path.abspath(__file__))
IMAGE_PATH    = os.path.join(SCRIPT_DIR, "bus.jpg")
MODEL_PATH    = os.path.join(SCRIPT_DIR, "yolov8s.pt")

# ── 카메라 파라미터 ──────────────────────────────────────────
WIDTH, HEIGHT = 1280, 960
FX, FY        = 1000.0, 1000.0
CX, CY        = WIDTH / 2, HEIGHT / 2
CONF_THRESH   = 0.3
PERSON_CLASS  = 0
CAPTURE_STEP  = 120

# ── Spot USD 경로 ────────────────────────────────────────────
assets_root = get_assets_root_path()
SPOT_USD     = assets_root + "/Isaac/Robots/BostonDynamics/spot/spot.usd"
SPOT_PATH    = "/World/Spot"

# Spot front camera 월드 위치: Spot body(z=0.65) + front offset
# 카메라는 Spot body 계층 외부에 독립적으로 배치
CAMERA_PATH    = "/World/FrontCamera"
SPOT_BODY_Z    = 0.65
CAM_WORLD_POS  = np.array([0.44, 0.075, SPOT_BODY_Z + 0.01])

print(f"[Spot] USD: {SPOT_USD}")

# ── World 초기화 ─────────────────────────────────────────────
world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()

# ── 조명 ────────────────────────────────────────────────────
dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
dome.CreateIntensityAttr(500.0)

# ── Spot 로봇 로드 ───────────────────────────────────────────
print("[Spot] 로봇 로드 중 (첫 실행 시 S3 다운로드 발생)...")
add_reference_to_stage(usd_path=SPOT_USD, prim_path=SPOT_PATH)

# Spot 위치: 원점, 서 있는 높이 z=0.65
spot_prim = stage.GetPrimAtPath(SPOT_PATH)
xf = UsdGeom.Xformable(spot_prim)
xf.ClearXformOpOrder()
xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.65))
# Spot은 기본적으로 +X 방향을 바라봄

# ── 카메라 (월드 독립 배치, +X 방향 바라봄) ──────────────────
# euler [0,0,0] → 카메라 기본 방향인 +X 를 그대로 사용
camera = Camera(
    prim_path=CAMERA_PATH,
    position=CAM_WORLD_POS,
    frequency=20,
    resolution=(WIDTH, HEIGHT),
    orientation=rot_utils.euler_angles_to_quats(
        np.array([0.0, 0.0, 0.0]), degrees=True
    ),
)

world.reset()
camera.initialize()
camera.set_opencv_pinhole_properties(
    cx=CX, cy=CY, fx=FX, fy=FY, pinhole=[0.0] * 12,
)

# ── 버스 이미지 평면 계산 ────────────────────────────────────
img    = cv2.imread(IMAGE_PATH)
ih, iw = img.shape[:2]

# 카메라 월드 X ≈ 0.44, 평면을 X=3 에 배치 → 거리 ≈ 2.56m
PLANE_X      = 3.0
CAM_WORLD_X  = CAM_WORLD_POS[0]
DIST         = PLANE_X - CAM_WORLD_X

plane_h  = 2.0 * DIST * (HEIGHT / 2) / FY  # FOV에 꽉 찰 높이
plane_w  = plane_h * (iw / ih)
hw, hh   = plane_w / 2, plane_h / 2

# 평면 중심 높이 = 카메라 높이
PLANE_CZ = CAM_WORLD_POS[2]

# ── 수직 평면 메시 (YZ 평면, X=PLANE_X) ─────────────────────
mesh = UsdGeom.Mesh.Define(stage, "/World/ImagePlane")
mesh.GetPointsAttr().Set([
    Gf.Vec3f(PLANE_X, -hw, PLANE_CZ - hh),
    Gf.Vec3f(PLANE_X,  hw, PLANE_CZ - hh),
    Gf.Vec3f(PLANE_X,  hw, PLANE_CZ + hh),
    Gf.Vec3f(PLANE_X, -hw, PLANE_CZ + hh),
])
mesh.GetFaceVertexCountsAttr().Set([4])
mesh.GetFaceVertexIndicesAttr().Set([0, 1, 2, 3])
mesh.GetNormalsAttr().Set([Gf.Vec3f(-1, 0, 0)] * 4)  # 카메라 방향(-X)으로 노멀
pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
    "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
pv.Set([(0, 0), (1, 0), (1, 1), (0, 1)])

# ── OmniPBR 텍스처 머티리얼 ─────────────────────────────────
material = OmniPBR(
    prim_path="/World/Looks/BusMat",
    name="bus_material",
    texture_path=IMAGE_PATH,
    texture_scale=[1.0, 1.0],
)
material.set_project_uvw(False)
material.set_reflection_roughness(1.0)
mat_usd = UsdShade.Material(stage.GetPrimAtPath("/World/Looks/BusMat"))
UsdShade.MaterialBindingAPI(mesh).Bind(mat_usd)

print(f"[Scene] 평면 {plane_w:.2f}x{plane_h:.2f}m @ X={PLANE_X}")
print(f"[Scene] Camera: {CAMERA_PATH}  pos={CAM_WORLD_POS}")

# ── YOLO 모델 ────────────────────────────────────────────────
model = YOLO(MODEL_PATH)
print("[YOLO] 모델 로드 완료")
print(f"[Isaac] {CAPTURE_STEP} 스텝 워밍업 중...")

step = 0
while simulation_app.is_running():
    world.step(render=True)
    step += 1

    if step % 30 == 0 and step < CAPTURE_STEP:
        print(f"  워밍업 {step}/{CAPTURE_STEP} ...")

    if step < CAPTURE_STEP:
        continue

    rgba = camera.get_rgba()
    if rgba is None:
        continue

    bgr = cv2.cvtColor(rgba[:, :, :3], cv2.COLOR_RGB2BGR)

    # 원본 저장 (회전 보정 전)
    cv2.imwrite(os.path.join(SCRIPT_DIR, "spot_debug_raw.jpg"), bgr)

    # euler [0,0,0] = +X 방향, 회전 보정 불필요
    cv2.imwrite(os.path.join(SCRIPT_DIR, "spot_debug_capture.jpg"), bgr)

    results = model(bgr, conf=CONF_THRESH, classes=[PERSON_CLASS])[0]
    boxes   = results.boxes
    n       = len(boxes)

    print(f"\n{'='*50}")
    print(f"  [Spot Camera YOLO] step={step}  감지된 사람 수: {n}")
    print(f"{'='*50}")
    for i, box in enumerate(boxes):
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        conf = float(box.conf[0])
        print(f"  [{i+1}] bbox=({x1},{y1},{x2},{y2})  conf={conf:.2f}")

    vis = bgr.copy()
    for box in boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        conf = float(box.conf[0])
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(vis, f"person {conf:.2f}",
                    (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imwrite(os.path.join(SCRIPT_DIR, "spot_yolo_result.jpg"), vis)
    cv2.imshow("Spot RGB Camera - YOLO Person Detection", vis)
    if cv2.waitKey(1) == ord('q'):
        break

    step = 0

cv2.destroyAllWindows()
simulation_app.close()
print("[완료]")
