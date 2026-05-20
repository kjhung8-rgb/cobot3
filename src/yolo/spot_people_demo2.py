"""
spot_people_demo.py
Spot 로봇 카메라로 DH_Characters_Extended 3D 사람을 YOLO로 감지.

실행:
  cd /home/kim/dev_ws/isaac_sim/duck
  /home/kim/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh spot_people_demo.py
"""
import os
import cv2
import numpy as np
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.usd
import omni.client
from pxr import UsdGeom, UsdLux, UsdPhysics, Gf
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.sensors.camera import Camera
from isaacsim.storage.native import get_assets_root_path
import isaacsim.core.utils.numpy.rotations as rot_utils
from ultralytics import YOLO

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH   = os.path.join(SCRIPT_DIR, "yolov8s.pt")

WIDTH, HEIGHT = 1280, 960
FX, FY        = 1000.0, 1000.0
CX, CY        = WIDTH / 2, HEIGHT / 2
CONF_THRESH   = 0.3
PERSON_CLASS  = 0

CAPTURE_STEP  = 120

assets_root = get_assets_root_path()
SPOT_USD    = assets_root + "/Isaac/Robots/BostonDynamics/spot/spot.usd"

# 가벼운 캐릭터 우선 (DH는 수백MB짜리라 로딩이 너무 오래 걸림)
CHARS_BASE  = assets_root + "/Isaac/People/Characters"
CHARS_BASE2 = assets_root + "/Isaac/People/DH_Characters"
CHARS_BASE3 = assets_root + "/Isaac/People/DH_Characters_Extended"

def find_character_usds(base_path, max_count=3):
    result, entries = omni.client.list(base_path)
    if result != omni.client.Result.OK:
        return []
    found = []
    for entry in entries:
        sub = base_path + "/" + entry.relative_path
        r2, subs = omni.client.list(sub)
        if r2 != omni.client.Result.OK:
            continue
        for se in subs:
            name = se.relative_path
            if name.endswith(".usd") and not name.endswith(".thumbnails.usd"):
                found.append(sub + "/" + name)
                break
        if len(found) >= max_count:
            break
    return found

print("[Chars] 캐릭터 USD 탐색 중...")
char_usds = find_character_usds(CHARS_BASE, max_count=4)
if not char_usds:
    print("[Chars] Characters 없음, DH_Characters 시도...")
    char_usds = find_character_usds(CHARS_BASE2, max_count=4)
if not char_usds:
    print("[Chars] DH_Characters 없음, DH_Characters_Extended 시도...")
    char_usds = find_character_usds(CHARS_BASE3, max_count=4)

if not char_usds:
    print("[Chars] 캐릭터를 찾을 수 없습니다. 경로를 확인하세요.")
    simulation_app.close()
    exit(1)

print(f"[Chars] {len(char_usds)}명 발견:")
for p in char_usds:
    print(f"  {p}")

# ── World 초기화 ─────────────────────────────────────────────
world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()

# ── 조명 ────────────────────────────────────────────────────
dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
dome.CreateIntensityAttr(1000.0)

# ── 바닥 ────────────────────────────────────────────────────
world.scene.add_default_ground_plane()

# ── Spot 로봇 ────────────────────────────────────────────────
print("[Spot] 로봇 로드 중...")
add_reference_to_stage(usd_path=SPOT_USD, prim_path="/World/Spot")
spot_prim = stage.GetPrimAtPath("/World/Spot")
xf = UsdGeom.Xformable(spot_prim)
xf.ClearXformOpOrder()
xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.75, 0.65))

# ── 캐릭터 배치 ──────────────────────────────────────────────
CHAR_X = 4.0
# 서있는 3명: 이미지 왼쪽(Y 음수), 누운 1명: 이미지 오른쪽(Y 양수, X 가까움)
configs = [
    {"x": CHAR_X, "y": -1.5, "z": 0.0, "face_cam": False, "lie_down": False},  # 옆모습
    {"x": CHAR_X, "y": -0.8, "z": 0.0, "face_cam": True,  "lie_down": False},  # 정면
    {"x": CHAR_X, "y": -0.1, "z": 0.0, "face_cam": False, "lie_down": False},  # 옆모습
    {"x": 3.0,    "y":  1.5, "z": 0.4, "face_cam": False, "lie_down": True },  # 누워있음
]

loaded_chars = 0
for i, char_usd in enumerate(char_usds):
    if i >= len(configs):
        break
    cfg = configs[i]
    prim_path = f"/World/Person_{i+1}"
    add_reference_to_stage(usd_path=char_usd, prim_path=prim_path)
    char_prim = stage.GetPrimAtPath(prim_path)
    if char_prim.IsValid():
        cxf = UsdGeom.Xformable(char_prim)
        cxf.ClearXformOpOrder()
        cxf.AddTranslateOp().Set(Gf.Vec3d(cfg["x"], cfg["y"], cfg["z"]))
        if cfg["lie_down"]:
            cxf.AddRotateXOp().Set(-90.0)   # 등 대고 누움
        elif cfg["face_cam"]:
            cxf.AddRotateZOp().Set(-90.0)   # 카메라 정면
        status = "누움" if cfg["lie_down"] else ("정면" if cfg["face_cam"] else "옆모습")
        print(f"[Chars] Person_{i+1} @ Y={cfg['y']} [{status}]")
        loaded_chars += 1

print(f"[Chars] 총 {loaded_chars}명 배치 완료")

# ── 카메라: Spot body에 부착 (/World/Spot/body/front_rgb) ──────
# body 프림이 없으면 Spot 루트에 부착
_body_prim = stage.GetPrimAtPath("/World/Spot/body")
CAM_PRIM_PATH = "/World/Spot/body/front_rgb" if _body_prim.IsValid() else "/World/Spot/front_rgb"
print(f"[Camera] 부착 경로: {CAM_PRIM_PATH}")

camera = Camera(
    prim_path=CAM_PRIM_PATH,
    position=np.array([0.41, 0.0, 0.07]),  # body 로컬 좌표
    frequency=20,
    resolution=(WIDTH, HEIGHT),
    orientation=rot_utils.euler_angles_to_quats(
        np.array([0.0, 0.0, 0.0]), degrees=True
    ),
)

# ── Spot 완전 고정 (모든 물리 설정을 world.reset() 이전에) ─────
spot_path_str = str(spot_prim.GetPath())
frozen = 0
joints_off = 0
for prim in stage.Traverse():
    if not str(prim.GetPath()).startswith(spot_path_str):
        continue
    # rigid body가 있으면 kinematic으로 설정 (static 방지)
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(True)
        frozen += 1
    # 조인트 프림 자체를 비활성화 → PhysX가 조인트 생성 시도 자체를 안 함
    if "Joint" in prim.GetTypeName():
        prim.SetActive(False)
        joints_off += 1

print(f"[Spot] kinematic: {frozen}개, 조인트 비활성화: {joints_off}개")

world.reset()  # 단일 reset: 위 설정 반영 후 물리 초기화

camera.initialize()

# world pose로 정확한 위치 설정: Spot body(0,0,0.65) + 로컬 오프셋(0.41,0,0.07)
camera.set_world_pose(
    position=np.array([0.41, 0.75, 0.72]),
    orientation=rot_utils.euler_angles_to_quats(np.array([0.0, 0.0, 0.0]), degrees=True),
)
print("[Camera] world pose 설정 완료 (0.41, 0.75, 0.72)")

camera.set_opencv_pinhole_properties(
    cx=CX, cy=CY, fx=FX, fy=FY, pinhole=[0.0] * 12,
)

# ── YOLO ────────────────────────────────────────────────────
model = YOLO(MODEL_PATH)
print(f"[YOLO] 모델 로드 완료")
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
    cv2.imwrite(os.path.join(SCRIPT_DIR, "people_debug.jpg"), bgr)

    results = model(bgr, conf=CONF_THRESH, classes=[PERSON_CLASS])[0]
    boxes   = results.boxes
    n       = len(boxes)

    print(f"\n{'='*50}")
    print(f"  [YOLO] step={step}  감지: {n}/{loaded_chars}명")
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

    cv2.imwrite(os.path.join(SCRIPT_DIR, "people_yolo_result.jpg"), vis)
    cv2.imshow("Spot Camera - YOLO 3D Person Detection", vis)
    if cv2.waitKey(1) == ord('q'):
        break

    step = 0

cv2.destroyAllWindows()
simulation_app.close()
print("[완료]")
