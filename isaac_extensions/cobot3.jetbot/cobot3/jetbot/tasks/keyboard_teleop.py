"""
JetBot 키보드 조종 스크립트
- Isaac Sim 5.1 전용
- 사용법: Isaac Sim Script Editor에 붙여넣고 Run
- 조종: W=전진 S=후진 A=좌회전 D=우회전 SPACE=정지

실행 순서:
  1. Isaac Sim 실행 (ROS2 source 후)
  2. Stage에 JetBot + 환경 로드
  3. ROS2 Bridge Extension 활성화
  4. Tools > Robotics > ROS2 OmniGraphs > Camera 로 카메라 그래프 생성
  5. Play(▶) 버튼 클릭 → JetBot 바닥 안착 대기
  6. Window > Script Editor 열고 이 스크립트 붙여넣고 Run
  7. Viewport 클릭 후 W/A/S/D 조종
"""

import carb
import omni
import omni.kit.app
import numpy as np
from omni.isaac.core.utils.stage import get_current_stage
from isaacsim.core.prims import SingleArticulation

# =============================================
#  설정값 (필요시 수정)
# =============================================
JETBOT_PRIM_PATH = "/World/jetbot"       # Stage에서 확인한 JetBot 경로
LEFT_WHEEL       = "left_wheel_joint"    # 왼쪽 바퀴 조인트 이름
RIGHT_WHEEL      = "right_wheel_joint"   # 오른쪽 바퀴 조인트 이름
LIN_SPEED        = 0.3                   # 전후진 속도 (m/s)
ANG_SPEED        = 1.2                   # 회전 속도 (rad/s)
WHEEL_RADIUS     = 0.03                  # 바퀴 반지름 (m)
WHEEL_BASE       = 0.1125                # 좌우 바퀴 간 거리 (m)
# =============================================

# 기존 subscription 정리 (재실행 시 중복 방지)
try:
    _kb_sub
    _kb_sub = None
except:
    pass
try:
    _update_sub
    _update_sub = None
except:
    pass

# JetBot Articulation 초기화
jetbot_art = SingleArticulation(prim_path=JETBOT_PRIM_PATH)
jetbot_art.initialize()

dof_names = jetbot_art.dof_names
print(f"DOF 목록: {dof_names}")

left_idx  = dof_names.index(LEFT_WHEEL)
right_idx = dof_names.index(RIGHT_WHEEL)
print(f"left_wheel index={left_idx}, right_wheel index={right_idx}")

# 키 상태
ks = {"v": 0.0, "w": 0.0}

# 키보드 콜백
def on_kb(event, *a, **kw):
    KI        = carb.input.KeyboardInput
    pressing  = event.type == carb.input.KeyboardEventType.KEY_PRESS
    releasing = event.type == carb.input.KeyboardEventType.KEY_RELEASE

    if event.input == KI.W:
        ks["v"] = LIN_SPEED if pressing else (0.0 if releasing else ks["v"])
    elif event.input == KI.S:
        ks["v"] = -LIN_SPEED if pressing else (0.0 if releasing else ks["v"])
    elif event.input == KI.A:
        ks["w"] = ANG_SPEED if pressing else (0.0 if releasing else ks["w"])
    elif event.input == KI.D:
        ks["w"] = -ANG_SPEED if pressing else (0.0 if releasing else ks["w"])
    elif event.input == KI.SPACE and pressing:
        ks["v"] = 0.0
        ks["w"] = 0.0
    return True

# 키보드 등록
aw      = omni.appwindow.get_default_app_window()
_kb_sub = carb.input.acquire_input_interface().subscribe_to_keyboard_events(
    aw.get_keyboard(), on_kb
)

# 매 프레임 바퀴 속도 적용
def on_update(e):
    lv = (ks["v"] - ks["w"] * WHEEL_BASE / 2.0) / WHEEL_RADIUS
    rv = (ks["v"] + ks["w"] * WHEEL_BASE / 2.0) / WHEEL_RADIUS
    vels = np.zeros(len(dof_names))
    vels[left_idx]  = lv
    vels[right_idx] = rv
    try:
        jetbot_art.set_joint_velocities(vels)
    except:
        pass

app         = omni.kit.app.get_app()
_update_sub = app.get_update_event_stream().create_subscription_to_pop(
    on_update, name="jetbot_teleop"
)

print("=" * 40)
print("  JetBot 키보드 조종 준비 완료!")
print("  W = 전진    S = 후진")
print("  A = 좌회전  D = 우회전")
print("  SPACE = 정지")
print("  ※ Viewport 클릭 후 사용하세요")
print("=" * 40)
