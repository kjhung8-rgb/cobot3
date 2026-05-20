"""
isaac_yolo_plane.py
bus.jpg를 평면에 텍스처로 붙이고, Isaac Sim 카메라로 촬영 후 YOLO person detection.
실행: /home/kim/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh isaac_yolo_plane.py
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
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils
from ultralytics import YOLO

SCRIPT_DIR    = os.path.dirname(os.path.abspath(__file__))
IMAGE_PATH    = os.path.join(SCRIPT_DIR, "bus.jpg")
MODEL_PATH    = os.path.join(SCRIPT_DIR, "yolov8s.pt")
WIDTH, HEIGHT = 1280, 960
FX, FY        = 1000.0, 1000.0
CX, CY        = WIDTH / 2, HEIGHT / 2
CONF_THRESH   = 0.3
PERSON_CLASS  = 0
CAPTURE_STEP  = 120

world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()

# ── 조명 (너무 밝으면 텍스처가 날아감) ─────────────────────
dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
dome.CreateIntensityAttr(500.0)

# ── 카메라: [0,0,3]에서 -Z 방향 바라봄 (euler Y=90°) ──────
camera = Camera(
    prim_path="/World/camera",
    position=np.array([0.0, 0.0, 3.0]),
    frequency=20,
    resolution=(WIDTH, HEIGHT),
    orientation=rot_utils.euler_angles_to_quats(
        np.array([0.0, 90.0, 0.0]), degrees=True   # +X → -Z
    ),
)
world.reset()
camera.initialize()
camera.set_opencv_pinhole_properties(
    cx=CX, cy=CY, fx=FX, fy=FY, pinhole=[0.0] * 12,
)

# ── 평면 크기 계산 ───────────────────────────────────────────
img    = cv2.imread(IMAGE_PATH)
ih, iw = img.shape[:2]
plane_h = 2.0 * 3.0 * (HEIGHT / 2) / FY
plane_w = plane_h * (iw / ih)
hw, hh  = plane_w / 2, plane_h / 2

# ── 평면 메시 ────────────────────────────────────────────────
mesh = UsdGeom.Mesh.Define(stage, "/World/ImagePlane")
mesh.GetPointsAttr().Set([
    Gf.Vec3f(-hw, -hh, 0),
    Gf.Vec3f( hw, -hh, 0),
    Gf.Vec3f( hw,  hh, 0),
    Gf.Vec3f(-hw,  hh, 0),
])
mesh.GetFaceVertexCountsAttr().Set([4])
mesh.GetFaceVertexIndicesAttr().Set([0, 1, 2, 3])
mesh.GetNormalsAttr().Set([Gf.Vec3f(0, 0, 1)] * 4)
pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
    "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
pv.Set([(0, 0), (1, 0), (1, 1), (0, 1)])

# ── OmniPBR 머티리얼 ─────────────────────────────────────────
material = OmniPBR(
    prim_path="/World/Looks/BusMat",
    name="bus_material",
    texture_path=IMAGE_PATH,
    texture_scale=[1.0, 1.0],
)
# 메시 UV 좌표 사용 (project_uvw=False)
material.set_project_uvw(False)
material.set_reflection_roughness(1.0)

mat_usd = UsdShade.Material(stage.GetPrimAtPath("/World/Looks/BusMat"))
UsdShade.MaterialBindingAPI(mesh).Bind(mat_usd)

print(f"[Isaac] 평면 {plane_w:.2f}x{plane_h:.2f}m | 텍스처: {IMAGE_PATH}")

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
    # 카메라 회전 보정 (90° CW 보정 → 이미지 직립)
    bgr = cv2.rotate(bgr, cv2.ROTATE_90_CLOCKWISE)
    cv2.imwrite(os.path.join(SCRIPT_DIR, "debug_capture.jpg"), bgr)

    results = model(bgr, conf=CONF_THRESH, classes=[PERSON_CLASS])[0]
    boxes   = results.boxes
    n       = len(boxes)

    print(f"\n{'='*45}")
    print(f"  [YOLO] step={step}  감지된 사람 수: {n}")
    print(f"{'='*45}")
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

    cv2.imwrite(os.path.join(SCRIPT_DIR, "yolo_plane_result.jpg"), vis)
    cv2.imshow("Isaac Sim - YOLO Person Detection", vis)
    if cv2.waitKey(1) == ord('q'):
        break

    step = 0

cv2.destroyAllWindows()
simulation_app.close()
print("[완료]")
