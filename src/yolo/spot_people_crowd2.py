"""
spot_people_crowd2.py - 다양한 가시성 시나리오
  - 완전 웅크림(deep crouch)
  - 상반신만 노출 (구덩이: z 오프셋 -0.85m)
  - 하반신만 노출 (벽 오클루전: 박스 장애물로 상반신 가림)
  - 차렷, 엎드림
실행:
  /home/kim/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh spot_people_crowd2.py
"""
import os, cv2, math, itertools
import numpy as np
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.extensions import enable_extension
for ext in ["omni.anim.graph.bundle", "omni.anim.graph.core",
            "omni.anim.retarget.bundle", "omni.anim.retarget.core"]:
    try: enable_extension(ext); simulation_app.update()
    except Exception as e: print(f"[ext] {ext}: {e}")

import omni.usd, omni.client, omni.timeline
from pxr import UsdGeom, UsdLux, UsdPhysics, UsdSkel, Usd, Gf, Vt, Sdf
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.sensors.camera import Camera
from isaacsim.storage.native import get_assets_root_path
import isaacsim.core.utils.numpy.rotations as rot_utils
from ultralytics import YOLO

SCRIPT_DIR    = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH    = os.path.join(SCRIPT_DIR, "yolov8s.pt")
WIDTH, HEIGHT = 1280, 960
FX, FY        = 700.0, 700.0
CX, CY        = WIDTH/2, HEIGHT/2
CONF_THRESH   = 0.25
PERSON_CLASS  = 0
CAPTURE_STEP  = 120

SPOT_X, SPOT_Y, SPOT_Z = -1.5, 0.75, 0.65
CAM_WORLD_POS = np.array([SPOT_X+0.41, SPOT_Y, SPOT_Z+0.07])
R_CAM_TO_WORLD = np.array([[0,0,1],[-1,0,0],[0,-1,0]], dtype=float)

def pixel_to_world(u, v, d, cam_pos):
    return cam_pos + R_CAM_TO_WORLD @ np.array([(u-CX)*d/FX, (v-CY)*d/FY, d])

assets_root = get_assets_root_path()
SPOT_USD = assets_root + "/Isaac/Robots/BostonDynamics/spot/spot.usd"
CH_BASE  = assets_root + "/Isaac/People/Characters"

# ── 군중 설정 ────────────────────────────────────────────────────
# pose:
#   attention  → 차렷 (팔 내리기)
#   crouch     → 일반 웅크림
#   deep_crouch→ 완전 쪼그려 앉기
#   tpose      → 기본 T포즈
#   prone      → 엎드림 (rx=-90)
#   upper_only → 상반신만: z_offset=-0.85 (구덩이 효과)
#   lower_only → 하반신만: 별도 벽 오클루더로 상반신 가림
CROWD = [
    # ── 일반 가시 ─────────────────────────────────────────────
    {"x":5.0,"y":-3.5,"z":0.0, "rz":-90,"rx":  0, "pose":"attention",   "label":"차렷"},
    {"x":5.0,"y":-2.0,"z":0.0, "rz":135,"rx":  0, "pose":"deep_crouch", "label":"완전웅크림"},
    {"x":7.0,"y":-1.5,"z":0.0, "rz":-90,"rx":  0, "pose":"crouch",      "label":"웅크림"},
    {"x":6.8,"y": 2.0,"z":0.0, "rz":  0,"rx":-90, "pose":"prone",       "label":"엎드림"},
    # ── 상반신만 (구덩이 효과: z=-0.85) ─────────────────────
    {"x":5.5,"y":-0.5,"z":-0.85,"rz":  0,"rx":0,  "pose":"tpose",      "label":"상반신(기본)"},
    {"x":5.5,"y": 1.0,"z":-0.85,"rz": 90,"rx":0,  "pose":"attention",  "label":"상반신(차렷)"},
    # ── 하반신만 (벽 오클루더로 상반신 가림) ──────────────────
    {"x":7.0,"y":-3.0,"z":0.0, "rz":-45,"rx":  0, "pose":"tpose",      "label":"하반신1"},
    {"x":7.0,"y": 0.5,"z":0.0, "rz": 45,"rx":  0, "pose":"attention",  "label":"하반신2"},
]

# 하반신 캐릭터 인덱스 (0-based) → 벽 오클루더 설정
LOWER_ONLY_IDX = {6, 7}  # CROWD 인덱스

# ── 캐릭터 USD 탐색 ──────────────────────────────────────────────
def find_char_usds(base, max_n=8):
    res, entries = omni.client.list(base)
    if res != omni.client.Result.OK: return []
    found = []
    for e in entries:
        sub = base + "/" + e.relative_path
        r2, subs = omni.client.list(sub)
        if r2 != omni.client.Result.OK: continue
        for se in subs:
            if se.relative_path.endswith(".usd") and ".thumbnails" not in se.relative_path:
                found.append(sub + "/" + se.relative_path); break
        if len(found) >= max_n: break
    return found

print("[Chars] 탐색...")
char_usds = find_char_usds(CH_BASE, 8)
if not char_usds:
    print("[ERROR] 캐릭터 없음!"); simulation_app.close(); raise SystemExit
char_cycle = list(itertools.islice(itertools.cycle(char_usds), 8))
print(f"[Chars] {len(char_usds)}종 발견")

# ── 골격 유틸 ────────────────────────────────────────────────────
def kw_find(joints, *kws):
    for kw in kws:
        kw_l = kw.lower()
        for j in joints:
            leaf = j.lower().split("/")[-1]
            if kw_l in leaf or kw_l in j.lower():
                return j
    return None

def find_skel_info(char_prim):
    skelroot_path = skel_path = None
    joints = []
    for prim in Usd.PrimRange(char_prim):
        if prim.IsA(UsdSkel.Root) and not skelroot_path:
            skelroot_path = str(prim.GetPath())
        if prim.IsA(UsdSkel.Skeleton) and not joints:
            skel_path = str(prim.GetPath())
            jts = UsdSkel.Skeleton(prim).GetJointsAttr().Get()
            if jts: joints = list(jts)
        if skelroot_path and joints: break
    return skelroot_path, skel_path, joints

def apply_rest_pose(stage, skel_path, joints, pose):
    """골격 restTransforms 직접 수정."""
    sp = stage.GetPrimAtPath(skel_path)
    if not sp.IsValid(): return False

    skel = UsdSkel.Skeleton(sp)
    rest = skel.GetRestTransformsAttr().Get()
    if not rest:
        rest = skel.GetBindTransformsAttr().Get()
    if not rest:
        print(f"  [rest] transforms 없음")
        return False

    jmap     = {j: i for i, j in enumerate(joints)}
    new_rest = list(rest)
    applied  = 0

    def modify_joint(jname, deg_x=0.0, deg_y=0.0, deg_z=0.0, dt_z=0.0):
        nonlocal applied
        if not jname or jname not in jmap:
            return False
        idx = jmap[jname]
        xf  = new_rest[idx]
        trans     = xf.ExtractTranslation()
        exist_rot = xf.ExtractRotation()

        extra = (Gf.Rotation(Gf.Vec3d(0,0,1), deg_z)
               * Gf.Rotation(Gf.Vec3d(0,1,0), deg_y)
               * Gf.Rotation(Gf.Vec3d(1,0,0), deg_x))
        new_rot   = extra * exist_rot
        new_trans = Gf.Vec3d(trans[0], trans[1], trans[2] + dt_z)

        m = Gf.Matrix4d()
        m.SetRotate(new_rot)
        m.SetTranslateOnly(new_trans)
        new_rest[idx] = m
        applied += 1
        return True

    # 공통 관절 탐색
    hip  = kw_find(joints, "hips","hip","pelvi","pelvis")
    l_th = kw_find(joints, "leftupleg","l_upleg","lupleg","leftthigh","l_thigh","lthigh",
                   "leftupperleg","l_upperleg","lupperleg")
    r_th = kw_find(joints, "rightupleg","r_upleg","rupleg","rightthigh","r_thigh","rthigh",
                   "rightupperleg","r_upperleg","rupperleg")
    l_sh = kw_find(joints, "leftshin","l_shin","lshin","leftcalf","l_calf","lcalf",
                   "leftleg","l_leg","lleg")
    r_sh = kw_find(joints, "rightshin","r_shin","rshin","rightcalf","r_calf","rcalf",
                   "rightleg","r_leg","rleg")
    l_arm = kw_find(joints, "leftupperarm","l_upperarm","lupperarm","leftarm","left_arm",
                    "l_arm","larm","leftshoulder","left_shoulder","l_shoulder")
    r_arm = kw_find(joints, "rightupperarm","r_upperarm","rupperarm","rightarm","right_arm",
                    "r_arm","rarm","rightshoulder","right_shoulder","r_shoulder")
    spine = kw_find(joints, "spine1","spine","torso","zspine")
    # thigh / shin 중복 방지
    if l_th and l_th == l_sh: l_sh = None
    if r_th and r_th == r_sh: r_sh = None

    if pose == "attention":
        print(f"  [attention] l_arm='{l_arm}' r_arm='{r_arm}'")
        modify_joint(l_arm, deg_z= 85)
        modify_joint(r_arm, deg_z=-85)

    elif pose == "crouch":
        print(f"  [crouch] hip='{hip}' l_th='{l_th}' r_th='{r_th}' l_sh='{l_sh}' r_sh='{r_sh}'")
        modify_joint(hip,  deg_x= 25)
        modify_joint(l_th, deg_x= 65)
        modify_joint(r_th, deg_x= 65)
        modify_joint(l_sh, deg_x=-95)
        modify_joint(r_sh, deg_x=-95)

    elif pose == "deep_crouch":
        # 완전 쪼그려 앉기: 고관절+무릎 최대 굽힘 + 힙 낮춤 + 척추 앞으로
        print(f"  [deep] hip='{hip}' l_th='{l_th}' r_th='{r_th}' l_sh='{l_sh}' r_sh='{r_sh}' spine='{spine}'")
        modify_joint(hip,   deg_x= 35, dt_z=-0.45)  # 힙 낮추기
        modify_joint(spine, deg_x= 20)               # 상체 앞으로
        modify_joint(l_th,  deg_x= 90)
        modify_joint(r_th,  deg_x= 90)
        modify_joint(l_sh,  deg_x=-130)
        modify_joint(r_sh,  deg_x=-130)
        modify_joint(l_arm, deg_z=  40)              # 팔 앞으로 자연스럽게
        modify_joint(r_arm, deg_z= -40)

    if applied == 0:
        print(f"  [rest] '{pose}' 매칭 실패 — 조인트 목록:")
        for j in joints[:30]: print(f"    {j}")
        return False

    skel.GetRestTransformsAttr().Set(Vt.Matrix4dArray(new_rest))
    print(f"  [rest] {applied}개 조인트 수정")
    return True


def add_occluder_wall(stage, path, x, y, z, sx, sy, sz):
    """상반신 가리는 불투명 박스 벽 (회색 콘크리트 느낌)."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
    xf.AddScaleOp().Set(Gf.Vec3d(sx, sy, sz))
    cube.CreateDisplayColorAttr([Gf.Vec3f(0.55, 0.52, 0.50)])
    return cube


# ── World 초기화 ─────────────────────────────────────────────────
world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()

dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
dome.CreateIntensityAttr(1400.0)
dist = UsdLux.DistantLight.Define(stage, "/World/Sun")
dist.CreateIntensityAttr(2000.0)
dist.CreateAngleAttr(0.53)
UsdGeom.Xformable(dist).AddRotateXYZOp().Set(Gf.Vec3f(-45, 0, 135))

world.scene.add_default_ground_plane()

# ── Spot ─────────────────────────────────────────────────────────
add_reference_to_stage(usd_path=SPOT_USD, prim_path="/World/Spot")
sp = stage.GetPrimAtPath("/World/Spot")
xf = UsdGeom.Xformable(sp)
xf.ClearXformOpOrder()
xf.AddTranslateOp().Set(Gf.Vec3d(SPOT_X, SPOT_Y, SPOT_Z))

# ── 캐릭터 배치 ──────────────────────────────────────────────────
_joints_printed = False
loaded = 0

for i, cfg in enumerate(CROWD):
    pp = f"/World/Person_{i+1}"
    add_reference_to_stage(usd_path=char_cycle[i], prim_path=pp)
    cp = stage.GetPrimAtPath(pp)
    if not cp.IsValid():
        print(f"[Chars] Person_{i+1} 로드 실패"); continue

    # 위치/회전
    pose  = cfg["pose"]
    cxf   = UsdGeom.Xformable(cp)
    cxf.ClearXformOpOrder()

    z_pos = 0.85 if pose == "prone" else cfg["z"]
    cxf.AddTranslateOp().Set(Gf.Vec3d(cfg["x"], cfg["y"], z_pos))
    if cfg["rx"] != 0: cxf.AddRotateXOp().Set(float(cfg["rx"]))
    if cfg["rz"] != 0: cxf.AddRotateZOp().Set(float(cfg["rz"]))

    # 스켈레톤
    _, skel_path, joints = find_skel_info(cp)

    if not _joints_printed and joints:
        _joints_printed = True
        print(f"\n{'='*60}")
        print(f"[Joints] Person_{i+1} 실제 조인트 ({len(joints)}개):")
        for j in joints: print(f"  {j}")
        print(f"{'='*60}\n")

    label = cfg["label"]
    if pose in ("attention", "crouch", "deep_crouch") and skel_path:
        ok = apply_rest_pose(stage, skel_path, joints, pose)
        print(f"[Chars] Person_{i+1} [{label}] rest={'OK' if ok else 'FAIL'}")
    elif pose == "prone":
        print(f"[Chars] Person_{i+1} [{label}] 엎드림(transform)")
    elif pose in ("upper_only","tpose"):
        print(f"[Chars] Person_{i+1} [{label}] {"상반신만(pit)" if cfg['z'] < 0 else "T포즈"}")
    else:
        print(f"[Chars] Person_{i+1} [{label}]")

    loaded += 1

print(f"\n[Chars] {loaded}명 배치 완료")

# ── 하반신만 오클루더 벽 배치 ────────────────────────────────────
# 카메라(x≈-1.1)와 캐릭터(x≈7.0) 사이에 캐릭터 바로 앞에 배치
# 벽: z=0.8~2.2m → 상반신 가림, 허벅지/정강이 노출
for i in LOWER_ONLY_IDX:
    cfg = CROWD[i]
    wx = cfg["x"] - 0.45      # 캐릭터 바로 앞
    wy = cfg["y"]
    wz = 1.5                   # 중심 z=1.5m (z범위 0.8~2.2m)
    path = f"/World/Wall_P{i+1}"
    add_occluder_wall(stage, path, wx, wy, wz, sx=0.25, sy=0.8, sz=0.7)
    print(f"[Wall] Person_{i+1} [{cfg['label']}] 오클루더 배치 x={wx:.1f} y={wy:.1f} z={wz}")

# ── 카메라 ───────────────────────────────────────────────────────
_body = stage.GetPrimAtPath("/World/Spot/body")
CAM_PATH = "/World/Spot/body/front_rgb" if _body.IsValid() else "/World/Spot/front_rgb"
camera = Camera(
    prim_path=CAM_PATH,
    position=np.array([0.41, 0.0, 0.07]),
    frequency=20,
    resolution=(WIDTH, HEIGHT),
    orientation=rot_utils.euler_angles_to_quats(np.array([0.,0.,0.]), degrees=True),
)

# ── Spot 고정 ────────────────────────────────────────────────────
for prim in stage.Traverse():
    p = str(prim.GetPath())
    if not p.startswith("/World/Spot"): continue
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(True)
    if "Joint" in prim.GetTypeName():
        prim.SetActive(False)

world.reset()
omni.timeline.get_timeline_interface().play()

camera.initialize()
camera.set_world_pose(
    position=CAM_WORLD_POS,
    orientation=rot_utils.euler_angles_to_quats(np.array([0.,0.,0.]), degrees=True),
)
camera.set_opencv_pinhole_properties(cx=CX, cy=CY, fx=FX, fy=FY, pinhole=[0.]*12)

import omni.replicator.core as rep
_rp          = rep.create.render_product(CAM_PATH, (WIDTH, HEIGHT))
_depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
_depth_annot.attach([_rp])

model = YOLO(MODEL_PATH)
print(f"[YOLO] 준비. {CAPTURE_STEP}스텝 워밍업...")

step = 0
while simulation_app.is_running():
    world.step(render=True)
    step += 1

    if step % 30 == 0 and step < CAPTURE_STEP:
        print(f"  워밍업 {step}/{CAPTURE_STEP}...")
    if step < CAPTURE_STEP:
        continue

    rgba = camera.get_rgba()
    if rgba is None: continue
    bgr  = cv2.cvtColor(rgba[:,:,:3], cv2.COLOR_RGB2BGR)

    _d = _depth_annot.get_data()
    depth_arr = None
    if _d is not None:
        if isinstance(_d, np.ndarray) and _d.size > 0:
            depth_arr = _d.reshape(HEIGHT, WIDTH)
        elif isinstance(_d, dict):
            raw = _d.get("data")
            if raw is not None and np.asarray(raw).size > 0:
                depth_arr = np.asarray(raw).reshape(HEIGHT, WIDTH)

    cv2.imwrite(os.path.join(SCRIPT_DIR, "crowd2_debug.jpg"), bgr)

    results = model(bgr, conf=CONF_THRESH, classes=[PERSON_CLASS])[0]
    boxes   = results.boxes
    n       = len(boxes)

    print(f"\n{'='*55}")
    print(f"  [YOLO] step={step}  감지: {n}/{loaded}명")
    print(f"{'='*55}")

    vis = bgr.copy()
    for idx, box in enumerate(boxes):
        x1,y1,x2,y2 = map(int, box.xyxy[0])
        conf = float(box.conf[0])
        uc, vc = (x1+x2)//2, (y1+y2)//2
        d = -1.0
        if depth_arr is not None:
            patch = depth_arr[max(0,vc-5):min(HEIGHT,vc+6),
                              max(0,uc-5):min(WIDTH,uc+6)]
            valid = patch[(patch>0.1)&(patch<60.0)]
            if len(valid): d = float(np.median(valid))
        if d > 0:
            wp = pixel_to_world(uc, vc, d, CAM_WORLD_POS)
            cs = f"({wp[0]:.2f},{wp[1]:.2f},{wp[2]:.2f})m"
        else:
            cs = "(depth없음)"
        print(f"  [{idx+1}] conf={conf:.2f}  depth={d:.2f}m  world={cs}")
        cv2.rectangle(vis,(x1,y1),(x2,y2),(0,255,0),2)
        cv2.putText(vis,f"p{idx+1} {conf:.2f}",(x1,y1-8),
                    cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
        cv2.putText(vis,cs,(x1,y2+18),
                    cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,255,255),2)

    cv2.imwrite(os.path.join(SCRIPT_DIR, "crowd2_yolo_result.jpg"), vis)
    cv2.imshow("Spot Camera - Crowd2 Detection", vis)
    if cv2.waitKey(1) == ord('q'): break
    step = 0

cv2.destroyAllWindows()
simulation_app.close()
print("[완료]")
