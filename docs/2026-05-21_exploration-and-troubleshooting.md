## 1. 프로젝트 배경## 2. 오늘 시작 시점의 상태

이전 세션에서 만든 것 (이미 작동 중):

- SLAM (slam_toolbox) + Nav2 동작
- explore_lite 기반 frontier 탐사 (불안정 상태)
- camera_coverage_tracker (카메라 본 영역 트래킹)
- coverage_path_planner (격자 기반 CPP, 초기 버전)
- cobot3 / cobot3_test 두 worktree 운영
- 사용자가 git pull 받았는데 빌드 + YOLO 작동 안 되는 상태

---

## 3. 오늘 한 작업 (시간순 상세)

### 3.1 git pull 후 빌드 / 구조 정리 (~30분)

**상황**: 사용자가 git pull로 팀의 새 정리된 구조를 받음.

**확인된 변경사항**:
1. `cobot_perception/cobot_perception/survivor_detector.py` 가 **삭제**됨
2. 새 패키지 `cobot3/src/yolo/` 가 추가됨
   - `yolo_detector.py` — YOLO 검출 + RGB-D 위치 추정
   - `survivor_pose_to_marker.py` — 검출 결과를 RViz Marker로 변환
3. `cobot_perception` 의 `package.xml` description 변경:
   > "Spot exploration perception utilities for camera coverage and arrival rotation."
4. `cobot_perception/launch/spot_explore.launch.py` 가 `pkg_yolo = get_package_share_directory("yolo")` 추가

**해석**: 팀이 의도적으로 검출(YOLO) 과 movement(coverage)를 분리. 깔끔한 책임 분리.

**작업**:
```bash
cd /home/rokey/dev_ws/cobot3
rm -rf build/cobot_perception install/cobot_perception  # stale egg-info 청소
source /opt/ros/humble/setup.bash
source /home/rokey/dev_ws/venv/perception/bin/activate
colcon build --packages-select cobot_perception
colcon build --symlink-install --packages-select cobot_core cobot3_navigation explore_lite explore_lite_msgs
colcon build --packages-select yolo  # 새 패키지!
```

**결과**: 5개 패키지 + yolo = 6개 패키지 모두 빌드 성공.

### 3.2 YOLO 영상 미수신 문제 (~15분)

**증상**: launch 후 RViz의 YOLO Survivor Detector 패널이 "No Image". `ros2 node list` 에 `/yolo_detector` 안 보이고 `/survivor_pose_to_marker` 만 있음.

**추적 과정**:
1. yolo 패키지 install 확인 → `survivor_pose_to_marker`, `yolo_detector` 둘 다 entry point 있음 ✓
2. launch에 yolo_detector Node 있는지 확인 → 있음 ✓
3. 그러면 노드가 crash 한 것 → 왜?
4. `yolo_detector.py` 의 imports 확인:
   ```python
   from ultralytics import YOLO  # line 21
   ```
5. `ultralytics` 는 system Python에 없고 **venv에만** 설치되어 있음
6. launch의 `Node(executable="yolo_detector", ...)` 에 `prefix=` 없음 → system Python으로 실행 → ImportError → crash

**해결**:
```python
# launch 파일에 상수 추가
PERCEPTION_VENV_PYTHON = "/home/rokey/dev_ws/venv/perception/bin/python"

# yolo_detector Node에 prefix 추가
Node(
    package="yolo",
    executable="yolo_detector",
    name="yolo_detector",
    output="screen",
    prefix=PERCEPTION_VENV_PYTHON,    # ← 추가
    parameters=[
        detector_params,
        {"model_path": yolo_model},
        {"use_sim_time": use_sim_time},
    ],
),
```

`prefix` 인자는 launch가 노드를 실행할 때 해당 명령을 앞에 붙임. 즉 `/home/rokey/dev_ws/venv/perception/bin/python <yolo_detector_script>` 로 실행됨.

### 3.3 모니터링 GUI 구현 (~1시간)

**동기**: 사용자가 "RViz 말고 좀 더 이쁘고 깔끔한 GUI를 만들고 싶다" 제안.

**옵션 분석**:

| 옵션 | 결과물 | 작업량 | 외관 | 결정 |
|---|---|---|---|---|
| A. Foxglove Studio | 데스크탑 앱 (드래그/드롭 패널) | 0줄 코드 | 매우 폴리시드 | 사용자가 sudo apt 설치 진행 |
| B. PyQt5 커스텀 GUI | 우리가 만든 Qt 윈도우 | ~500줄 | 완전 커스텀 | 작성함 |
| C. 웹 대시보드 | 브라우저 (Streamlit 등) | 1000+줄 | 가장 모던 | 너무 큰 작업, 보

### 시나리오
재난 현장 (실내, 화재/붕괴 가정) 에서 Spot 4족 로봇이 자율적으로 공간을 탐사하며 RGB 카메라로 사람(생존자)을 검출하는 시스템.

### 환경
- **시뮬레이터**: Isaac Sim 5.1 (sujung_warehouse.usd, ~30m × 30m 창고)
- **로봇**: Boston Dynamics Spot (Isaac Sim에서 RL 정책으로 보행)
- **센서**:
  - 360° LiDAR (25m 범위)
  - 전방 RGB 카메라 (70° FOV)
  - Depth 카메라 (RGB 동일 위치)
  - Odometry (IMU + 발 odometry)
- **계산**: NVIDIA RTX 5080 Laptop GPU (CUDA 13)
- **소프트웨어**:
  - Ubuntu 22.04, ROS 2 Humble
  - Python 3.10 (system) + Python 3.10 venv (`/home/rokey/dev_ws/venv/perception/`)
  - Isaac Sim의 Python은 3.11 — ROS rclpy와 호환 안 됨 (별도 다룸)

### 두 트랙 분리 (오늘 끝 시점)
1. **자율 탐사 트랙** — `cobot_perception` 패키지가 담당
2. **YOLO 검출 트랙** — `yolo` 패키지가 별도로 담당

이 둘은 독립적으로 동작하며 ROS 토픽으로만 연결.

---

## 2. 오늘 시작 시점의 상태

이전 세션에서 만든 것 (이미 작동 중):

- SLAM (slam_toolbox) + Nav2 동작
- explore_lite 기반 frontier 탐사 (불안정 상태)
- camera_coverage_tracker (카메라 본 영역 트래킹)
- coverage_path_planner (격자 기반 CPP, 초기 버전)
- cobot3 / cobot3_test 두 worktree 운영
- 사용자가 git pull 받았는데 빌드 + YOLO 작동 안 되는 상태

---

## 3. 오늘 한 작업 (시간순 상세)

### 3.1 git pull 후 빌드 / 구조 정리 (~30분)

**상황**: 사용자가 git pull로 팀의 새 정리된 구조를 받음.

**확인된 변경사항**:
1. `cobot_perception/cobot_perception/survivor_detector.py` 가 **삭제**됨
2. 새 패키지 `cobot3/src/yolo/` 가 추가됨
   - `yolo_detector.py` — YOLO 검출 + RGB-D 위치 추정
   - `survivor_pose_to_marker.py` — 검출 결과를 RViz Marker로 변환
3. `cobot_perception` 의 `package.xml` description 변경:
   > "Spot exploration perception utilities for camera coverage and arrival rotation."
4. `cobot_perception/launch/spot_explore.launch.py` 가 `pkg_yolo = get_package_share_directory("yolo")` 추가

**해석**: 팀이 의도적으로 검출(YOLO) 과 movement(coverage)를 분리. 깔끔한 책임 분리.

**작업**:
```bash
cd /home/rokey/dev_ws/cobot3
rm -rf build/cobot_perception install/cobot_perception  # stale egg-info 청소
source /opt/ros/humble/setup.bash
source /home/rokey/dev_ws/venv/perception/bin/activate
colcon build --packages-select cobot_perception
colcon build --symlink-install --packages-select cobot_core cobot3_navigation explore_lite explore_lite_msgs
colcon build --packages-select yolo  # 새 패키지!
```

**결과**: 5개 패키지 + yolo = 6개 패키지 모두 빌드 성공.

### 3.2 YOLO 영상 미수신 문제 (~15분)

**증상**: launch 후 RViz의 YOLO Survivor Detector 패널이 "No Image". `ros2 node list` 에 `/yolo_detector` 안 보이고 `/survivor_pose_to_marker` 만 있음.

**추적 과정**:
1. yolo 패키지 install 확인 → `survivor_pose_to_marker`, `yolo_detector` 둘 다 entry point 있음 ✓
2. launch에 yolo_detector Node 있는지 확인 → 있음 ✓
3. 그러면 노드가 crash 한 것 → 왜?
4. `yolo_detector.py` 의 imports 확인:
   ```python
   from ultralytics import YOLO  # line 21
   ```
5. `ultralytics` 는 system Python에 없고 **venv에만** 설치되어 있음
6. launch의 `Node(executable="yolo_detector", ...)` 에 `prefix=` 없음 → system Python으로 실행 → ImportError → crash

**해결**:
```python
# launch 파일에 상수 추가
PERCEPTION_VENV_PYTHON = "/home/rokey/dev_ws/venv/perception/bin/python"

# yolo_detector Node에 prefix 추가
Node(
    package="yolo",
    executable="yolo_detector",
    name="yolo_detector",
    output="screen",
    prefix=PERCEPTION_VENV_PYTHON,    # ← 추가
    parameters=[
        detector_params,
        {"model_path": yolo_model},
        {"use_sim_time": use_sim_time},
    ],
),
```

`prefix` 인자는 launch가 노드를 실행할 때 해당 명령을 앞에 붙임. 즉 `/home/rokey/dev_ws/venv/perception/bin/python <yolo_detector_script>` 로 실행됨.

### 3.3 모니터링 GUI 구현 (~1시간)

**동기**: 사용자가 "RViz 말고 좀 더 이쁘고 깔끔한 GUI를 만들고 싶다" 제안.

**옵션 분석**:

| 옵션 | 결과물 | 작업량 | 외관 | 결정 |
|---|---|---|---|---|
| A. Foxglove Studio | 데스크탑 앱 (드래그/드롭 패널) | 0줄 코드 | 매우 폴리시드 | 사용자가 sudo apt 설치 진행 |
| B. PyQt5 커스텀 GUI | 우리가 만든 Qt 윈도우 | ~500줄 | 완전 커스텀 | 작성함 |
| C. 웹 대시보드 | 브라우저 (Streamlit 등) | 1000+줄 | 가장 모던 | 너무 큰 작업, 보류 |

**B (PyQt5) 작성 내용** — `monitoring_gui.py`:

```
┌──────────── Spot 생존자 탐색 모니터 ────────────┐
│                                                  │
│  ┌─ YOLO 카메라 ──┐  ┌─ 맵 + 로봇 + 생존자 ──┐ │
│  │                │  │                         │ │
│  │ annotated      │  │  Map (SLAM 흑백)       │ │
│  │ image          │  │  Camera Coverage (초록) │ │
│  │ stream         │  │  Robot (노랑)          │ │
│  │                │  │  Survivor (빨강 X)     │ │
│  └────────────────┘  └─────────────────────────┘ │
│                                                  │
│  ┌─ 상태 ─────────────────────────────────────┐ │
│  │ 경과 시간: 02:35  현재 zone: (1, 0)       │ │
│  │ Waypoint 진행: 12/24 (50%)                │ │
│  │ 카메라 커버리지: 64%                       │ │
│  │ ─────────────────────────────────────────  │ │
│  │ 발견된 생존자: 2명                          │ │
│  │ • #1 (3.2, -1.5)  T+02:10                  │ │
│  │ • #2 (8.1, 4.0)   T+05:30                  │ │
│  └────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────┘
```

**구현 세부**:
- **ROS bridge**: 별도 thread에서 rclpy executor 돌리고, Qt timer로 200ms마다 snapshot 가져옴
- **이미지 표시**: cv_bridge로 ROS Image → numpy → QImage → QPixmap (scale-to-fit)
- **2D 맵 패널**: QPainter로 직접 그림 (SLAM map → QImage, camera coverage → 반투명 overlay, 로봇/생존자는 도형)
- **다크 테마**: stylesheet로 #111827 배경 + #e5e7eb 텍스트
- **survivor dedup**: 같은 위치(0.5m 이내) 중복 방지

### 3.4 시간 단축 + 경로 최적화 (~1시간)

**동기**: "재난 현장이라 시간 단축이 필요한데 정확도는 유지하고 싶다"

**현재 walltime 분석**:
```
한 waypoint 처리 = 이동(N초) + 회전(4초) + Nav2 오버헤드(~2초)
30×30m × 4m 간격 = ~49 waypoint × 11s/wp = ~9분
```

**제시한 옵션** (효과/위험 순):

| # | 변경 | 단축 효과 | 정확도 영향 |
|---|---|---|---|
| 1 | 속도 1.0 → 1.5 m/s | -30% | 없음 |
| 2 | spin_duration 4s → 2s | -15% | 약간 |
| 3 | spin 완전 제거 | -25% | 작음 |
| 4 | 회전 속도 1.0 → 1.5 rad/s | -5% | 없음 |
| 5 | waypoint 4m → 5m | -20% | 작음 |
| 6 | TSP 순서 최적화 | -10~20% | 없음 |

**사용자 결정**: 속도는 OK, 경로 자체 최적화.

**작업 — Boustrophedon 알고리즘 도입**:

이전 (greedy nearest):
```
지나간 곳:  ●─→●     ●        ← 왔다갔다 가능
                  ↘ ╱
                   ●     ●     ← "별 모양" 백트랙
            ●     ●  ↗ ╱
                  ●─→●
```

이후 (boustrophedon 지그재그):
```
지나간 곳:  ●→●→●→●→●  ← row 0
                       ↓
            ●←●←●←●←●  ← row 1 (역방향)
            ↓
            ●→●→●→●→●  ← row 2 (정방향)
```

**의사결정**:
- **cobot3** (main) = greedy 유지 (안정 baseline)
- **cobot3_test** = boustrophedon 실험
- 두 worktree 모두 빌드, 사용자가 비교 가능

### 3.5 Zone 분할 보정 시도 (~30분)

**상황**: 사용자가 스크린샷으로 zone 박스가 어색하게 분할되는 문제 제기.

**관찰**:
- 현재 zone 격자는 SLAM origin (0, 0) 기준 15m × 15m
- SLAM origin = 로봇 spawn 위치
- 따라서 로봇 spawn이 zone (0, 0)의 **모서리**에 위치
- 결과: (0, -1), (1, -2), (-1, -2) 같은 어색한 zone 분포

**구현 — 자동 정렬**:
```python
def _maybe_align_zone_grid(self, costmap_arr, res, ox, oy):
    if not (self._zone_enabled and self._zone_align):
        return
    if self._zone_offset_locked:
        return
    free_mask = (costmap_arr == 0)
    free_count = int(free_mask.sum())
    if free_count < self._zone_align_min_free:  # 500개 이상 되면
        return
    ys, xs = np.where(free_mask)
    cx_world = ox + (xs.mean() + 0.5) * res    # free centroid (world)
    cy_world = oy + (ys.mean() + 0.5) * res
    self._zone_offset_x = cx_world - self._zone_size / 2.0  # zone (0,0) 중심에
    self._zone_offset_y = cy_world - self._zone_size / 2.0
    self._zone_offset_locked = True  # 한 번만
```

**결과**: 사용자 테스트 후 "이전 버전이 더 잘 탐색"
**가설**:
1. free centroid가 robot spawn에서 멀어 zone (0,0) 자체가 spawn과 떨어진 위치 → 초기 이동 거리 증가
2. align 시점에 map이 부분적으로만 자랐을 수도 → centroid가 편향

**조치**: `zone_align_to_free_centroid` default를 False로 변경. 코드는 유지 (옵션으로 켤 수 있음).

---

## 4. 트러블슈팅 로그 (상세)

### Issue #1 — `survivor_detector` 가 install/lib/ 에 없음

**현상**:
```bash
$ ls /home/rokey/dev_ws/cobot3/install/cobot_perception/lib/cobot_perception/
camera_coverage_sweep
camera_coverage_tracker
coverage_path_planner
rotate_on_arrival
stop_watchdog
# survivor_detector 없음
```

**조사**:
- `cobot_perception/setup.py` 의 entry_points 확인 → survivor_detector 항목 없음
- `cobot_perception/cobot_perception/` 디렉터리 확인 → `survivor_detector.py` 파일 자체가 없음
- `cobot_perception/package.xml` description 변경됨 → 의도된 변경 확인

**원인**: 팀이 검출 부분을 별도 `yolo` 패키지로 분리. cobot_perception에서 의도적으로 제거.

**해결**: `yolo` 패키지를 추가로 빌드.
```bash
colcon build --packages-select yolo
ls install/yolo/lib/yolo/
# yolo_detector
# survivor_pose_to_marker
```

**검증**: launch 띄운 후 노드 리스트 확인.

---

### Issue #2 — `/yolo_detector` 노드 안 떠있음

**현상**:
```bash
$ ros2 node list | grep yolo
/survivor_pose_to_marker
# /yolo_detector 없음
```

`/spot_0/yolo/annotated_image` 토픽도 안 나옴 → RViz "No Image".

**조사 명령**:
```bash
ros2 topic info /spot_0/yolo/annotated_image
# Subscription count: 1 (rviz)
# Publisher count: 0 ← 발행자 없음

# launch 로그 확인 → yolo_detector가 import error로 crash
# (launch terminal에 에러 메시지 보였을 것)
```

**원인 분석**:
`yolo_detector.py` 코드:
```python
from ultralytics import YOLO  # line 21
```

`ultralytics` 는 ~2GB짜리 (torch + CUDA 의존성 포함) 라 system Python에 설치 안 함. venv `/home/rokey/dev_ws/venv/perception/` 에만 설치.

launch의 노드 설정:
```python
Node(
    package="yolo",
    executable="yolo_detector",
    name="yolo_detector",
    output="screen",
    # prefix= 없음! → system Python으로 실행 → ImportError → crash
    parameters=[...],
),
```

비교: cobot_perception은 다른 venv-only 노드들에 prefix 명시되어 있음 (이전에 설정).

**해결**: launch에 venv 상수 추가 + yolo_detector Node에 prefix 추가
```python
PERCEPTION_VENV_PYTHON = "/home/rokey/dev_ws/venv/perception/bin/python"

Node(
    package="yolo",
    executable="yolo_detector",
    name="yolo_detector",
    output="screen",
    prefix=PERCEPTION_VENV_PYTHON,    # ← 추가
    parameters=[...],
),
```

**검증**:
```bash
$ ros2 node list | grep yolo
/yolo_detector
/survivor_pose_to_marker

$ ros2 topic hz /spot_0/yolo/annotated_image
# ~30 Hz 나와야 정상
```

---

### Issue #3 — PyQt5 GUI Qt 플러그인 충돌

**현상**:
```
QObject::moveToThread: Current thread (...) is not the object's thread (...)
qt.qpa.plugin: Could not load the Qt platform plugin "xcb" in
"/home/rokey/dev_ws/venv/perception/lib/python3.10/site-packages/cv2/qt/plugins"
even though it was found.
This application failed to start because no Qt platform plugin could be initialized.
중지됨 (코어 덤프됨)
```

**조사**:
- 에러 메시지에 `cv2/qt/plugins` 명시 → opencv-python이 자체 Qt 플러그인 들고 옴
- 이 플러그인이 PyQt5의 Qt와 ABI 충돌

**원인 — Qt plugin path 우선순위 문제**:
- `import cv2` 가 일어나면 cv2가 `QT_PLUGIN_PATH` 를 자기 디렉터리로 설정
- 그 후 PyQt5가 import 되면 이 path를 사용
- cv2의 Qt는 PyQt5의 Qt와 컴파일 ABI 다름 → 로드 실패

우리 monitoring_gui.py:
```python
from cv_bridge import CvBridge  # 내부적으로 cv2 import
# ... 이후
from PyQt5 import QtCore, QtGui, QtWidgets  # QT_PLUGIN_PATH 이미 오염됨
```

**시도한 해결책**:
1. **env 변수 우회 (실패)**:
```bash
QT_QPA_PLATFORM_PLUGIN_PATH=/usr/lib/python3/dist-packages/PyQt5/Qt5/plugins/platforms \
python monitoring_gui
```
→ 여전히 cv2 path가 우선. cv2 import 시 env 덮어쓰기 때문.

2. **cv2 Qt 디렉터리 삭제 (간단하지만 임시)**:
```bash
rm -rf /home/rokey/dev_ws/venv/perception/lib/python3.10/site-packages/cv2/qt
```

3. **opencv-python-headless로 교체 (영구 해결)** ✓:
```bash
pip uninstall -y opencv-python
pip install opencv-python-headless
```
headless 버전은 Qt 플러그인을 아예 안 들고옴. cv_bridge는 cv2 import만 하지 GUI 기능 안 쓰니까 문제 없음.

**해결**: opencv-python-headless 설치.

---

### Issue #4 — opencv-headless 설치 후 cv_bridge 깨짐

**현상**:
```
A module that was compiled using NumPy 1.x cannot be run in
NumPy 2.2.6 as it may crash.
Traceback:
  File ".../cv_bridge/core.py", line 194, in imgmsg_to_cv2
    res = cvtColor2(im, img_msg.encoding, desired_encoding)
AttributeError: _ARRAY_API not found
세그멘테이션 오류 (코어 덤프됨)
```

**원인**: opencv-python-headless가 의존성으로 numpy>=2.0.0 을 요구함 → 설치 과정에서 venv의 numpy를 1.x → 2.x로 자동 업그레이드. 하지만:
- ROS Humble의 cv_bridge는 numpy 1.x로 컴파일된 C 확장 (`cv_bridge_boost`)
- numpy 2.x ABI 와 호환 안 됨 → `_ARRAY_API not found`

이 패턴은 **이전에도 만났던 거** — opencv 계열 설치할 때마다 numpy upgrade → ROS cv_bridge 깨짐.

**해결**:
```bash
/home/rokey/dev_ws/venv/perception/bin/pip install 'numpy<2'
```
→ numpy를 1.26.4로 다운그레이드. opencv-python-headless도 1.x에서 동작.

**검증**:
```bash
$ /home/rokey/dev_ws/venv/perception/bin/python -c "
import numpy as np
import cv2
from cv_bridge import CvBridge
print('numpy', np.__version__)
print('cv2', cv2.__version__)
print('cv_bridge OK')
"
# numpy 1.26.4
# cv2 4.13.0  (headless)
# cv_bridge OK
```

**교훈**: numpy 1.x를 명시적으로 pin해두는 게 좋음. requirements.txt나 setup.py에 `numpy<2` 명시.

---

### Issue #5 — Zone auto-align 효과 안 좋음

**현상**: cobot3_test에서 `zone_align_to_free_centroid: True` 로 자동 정렬 켰는데 사용자 평가 "이전 버전이 더 잘 탐색".

**가설**:

1. **첫 정렬 시점의 부분 map 문제**:
   - free 셀이 500개 이상 되면 align 발동
   - 하지만 이 시점에 SLAM은 아직 spawn 근처만 매핑한 상태일 수도
   - centroid가 spawn 근처로 편향 → align 효과 미미

2. **Robot spawn ↔ Zone center 거리 증가**:
   - 이전: robot이 zone 모서리 (spawn = zone (0,0) corner)
   - 이후: robot이 zone (0,0) 어딘가 근처지만 정확히 중심은 아님
   - 알고리즘이 current_zone 찾는 방식 영향

3. **Zone ID 변경으로 인한 visited flag 손실 가능성**:
   - align 후 zone offset 바뀌면, waypoint들의 zone 할당 변경
   - 코드는 wp_id 기준 visited 보존하지만 zone 매핑은 새로
   - current_zone이 갑자기 다른 zone 가리킬 수 있음

**해결**: `zone_align_to_free_centroid` default를 False. 코드는 유지 (다음에 다른 접근으로 시도 가능).

**향후 개선 아이디어** (지금은 안 함):
- align 시점을 더 늦게 (free 셀 2000개 이상 등)
- 또는 align 후 current_zone 명시적 재계산
- 또는 SLAM map의 bounding box 기반 정렬 (centroid 아니라 box center)

---

## 5. 자율 탐사 알고리즘 (현재 구조)

### 5.1 전체 데이터 플로우

```
┌────────────── Isaac Sim 시뮬레이션 ─────────────────┐
│                                                       │
│  Spot RL policy                                       │
│    └─ /spot_0/cmd_vel 받아서 보행                      │
│                                                       │
│  Front Camera                                         │
│    ├─ /spot_0/front_cam/color_image (RGB)            │
│    ├─ /spot_0/front_cam/depth_image (Depth)          │
│    └─ /spot_0/front_cam/camera_info                   │
│                                                       │
│  LiDAR (360°, 25m)                                    │
│    └─ /spot_0/scan                                    │
│                                                       │
│  Odometry + TF                                        │
│    ├─ /spot_0/odom                                    │
│    └─ tf: world → spot_0/base_link, lidar_link, etc.  │
└───────────────────────────────────────────────────────┘
                          │
                          ▼
┌────────────── ROS 2 노드 그래프 ─────────────────────┐
│                                                       │
│  scan_sanitizer ×2 (NaN/Inf 청소)                     │
│    ├─ /spot_0/scan_slam (SLAM용)                      │
│    └─ /spot_0/scan_nav  (Nav2 local_costmap용)        │
│                                                       │
│  slam_toolbox (online mapping)                        │
│    ├─ /map (점진적으로 자람)                          │
│    └─ TF: map → odom                                  │
│                                                       │
│  Nav2 stack                                           │
│    ├─ global_costmap (← /map, inflation 적용)         │
│    ├─ local_costmap  (← /spot_0/scan_nav)             │
│    ├─ planner_server (NavfnPlanner, allow_unknown)    │
│    ├─ controller_server (MPPI)                        │
│    ├─ bt_navigator (NavigateToPose action)            │
│    └─ behavior_server (Spin, BackUp 등)               │
│                                                       │
│  cmd_vel_relay                                        │
│    ├─ 입력: /cmd_vel + /spot_0/yolo/person_detected   │
│    └─ 출력: /spot_0/cmd_vel (사람 검출 시 0.3x 감속)  │
│                                                       │
│  camera_coverage_tracker (5Hz)                        │
│    ├─ 입력: /map, /camera_info, TF                    │
│    ├─ 동작: 카메라 부채꼴 raycast → seen cell 마킹     │
│    └─ 출력: /camera_coverage (OccupancyGrid)          │
│                                                       │
│  ★ coverage_path_planner (1.5Hz tick) ★              │
│    ├─ 입력: /global_costmap/costmap, /camera_coverage │
│    ├─ 책임:                                           │
│    │    1. waypoint 생성 (격자)                      │
│    │    2. zone 분할                                  │
│    │    3. 방문 순서 결정 (greedy 또는 boustrophedon) │
│    │    4. NavigateToPose action 발행                 │
│    │    5. 도착 후 360° spin                          │
│    │    6. 시각화 발행                                │
│    ├─ 출력 토픽:                                      │
│    │    /coverage_zones (MarkerArray)                 │
│    │    /coverage_waypoints (MarkerArray)             │
│    │    /cmd_vel (spin용)                            │
│    └─ Nav2와 통신: /navigate_to_pose action client    │
│                                                       │
│  yolo_detector (별도 트랙)                            │
│    ├─ 입력: /spot_0/front_cam/{color, depth, info}    │
│    ├─ YOLOv8s CUDA 추론                               │
│    ├─ /spot_0/yolo/annotated_image                    │
│    ├─ /spot_0/yolo/person_detected (Bool)             │
│    └─ /detected_survivor_pose (PoseStamped)           │
│                                                       │
│  survivor_pose_to_marker                              │
│    ├─ 입력: /detected_survivor_pose                   │
│    └─ 출력: /survivor_goal_marker (RViz용)            │
│                                                       │
│  monitoring_gui (선택적, PyQt5)                       │
│    └─ 위 토픽들 종합 시각화                           │
└───────────────────────────────────────────────────────┘
                          │
                          ▼
                 /spot_0/cmd_vel
                          │
                          ▼
                 Spot RL policy → 보행
                          │
                          ▼
                 (loop back to Isaac sensors)
```

### 5.2 핵심 노드 책임 매트릭스

| 노드 | 입력 | 출력 | 핵심 책임 |
|---|---|---|---|
| slam_toolbox | scan, odom | /map, TF | LiDAR로 맵 작성 |
| Nav2 | /map, scan, goal | /cmd_vel | 경로 계획 + 제어 |
| camera_coverage_tracker | /map, TF, camera_info | /camera_coverage | 카메라 본 영역 누적 |
| coverage_path_planner | costmap, coverage, TF | NavigateToPose goals, 시각화 | **탐사 전략 (메인)** |
| yolo_detector | camera images | annotated img, pose | 사람 검출 |
| cmd_vel_relay | /cmd_vel, detection | /spot_0/cmd_vel | Nav2→Spot bridge + 감속 |

---

## 6. Frontier 실패 원인 분석

### 6.1 Frontier 알고리즘의 가정

전통적 frontier exploration (Yamauchi 1997):

```
점유격자 (occupancy grid):
  □ = FREE
  ■ = OBSTACLE
  . = UNKNOWN

Frontier = FREE 셀 중 UNKNOWN 이웃을 가진 셀

LiDAR가 25m 등방 매핑:
  . . . . . . . . . .
  . . □ □ □ □ □ . . .
  . □ □ □ □ □ □ □ . .
  . □ □ □ R □ □ □ . .   R = 로봇 위치
  . □ □ □ □ □ □ □ . .   □ 가장자리 = frontier (멀리)
  . . □ □ □ □ □ . . .
  . . . . . . . . . .
```

**핵심 가정**:
1. 센서가 **등방(isotropic)** — 사방으로 비슷한 범위
2. 한 번에 **큰 영역**을 알게 됨 (LiDAR 25m이면 큰 원)
3. Frontier 는 **로봇으로부터 멀리** 형성됨
4. Robot이 frontier로 이동 → 거기서 또 큰 원형 매핑 → 새 frontier 더 멀리 → 반복

### 6.2 우리 시스템에서 깨진 가정

카메라 마스킹 도입:
```
camera_coverage_tracker 가 만든 /map_explorable:
  FREE  if SLAM도 free  AND  카메라도 봤음
  UNKNOWN  if SLAM은 free지만 카메라가 못 봤음
  OBSTACLE  if SLAM이 벽
```

이 마스킹 맵을 global_costmap.static_layer 에 먹임 → explore_lite는 이 작은 카메라 거품만 "FREE"로 인식.

**문제 시각화**:
```
실제 환경 (LiDAR가 모두 보임):     /map_explorable (카메라 본 영역만):

. . . . . . . . . .                 . . . . . . . . . .
. . . . . . . . . .                 . . . . . . . . . .
. . . . . . . . . .                 . . . . . . . . . .
. . . . . . . . . .                 . . . . . . . . . .
. . . . R . . . . .                 . . . . R . . . . .
. . . . . . . . . .                 . . . . □□□ . . . .  ← 정면 좁은 거리만
. . . . . . . . . .                 . . . □□□□□ . . .
. . . . . . . . . .                 . . . . . . . . . .

→ frontier = 큰 원 가장자리 (멀리)      → frontier = 작은 부채꼴 가장자리 (로봇 옆)
```

### 6.3 실측 데이터로 확인된 cascade

진단 스크립트로 본 실제 수치 (이전 세션 측정):

```
Costmap 1474 × 1512:
  FREE   : 6383 cells (0.3%)    ← 매우 작음
  UNKNOWN: 2,154,067 cells (97%)
  OBSTACLE: 26,826 cells (1.2%)

Frontier cells (FREE인데 UNKNOWN 이웃): 135개
  최소 거리 from robot: 0.34m   ← 25cm tolerance 살짝 밖
  평균 거리: 1.78m
  0.25m 안: 0개  (instant SUCCESS 자체는 아님)
  0.5m 안: 13개  ← 매우 가까움
```

**왜 그래도 robot이 거의 안 움직였나?**

xy_goal_tolerance = 0.25m. Robot 좌표가 (0.24, -0.06) 인데 frontier centroid가 0.34m 거리에 있다고 해도:
- Nav2 plan 생성 → 짧은 path
- Controller 일찍 멈춤 (path 따라 일부만 진행했는데 tolerance 진입)
- → SUCCEEDED with 5cm 이동
- 다음 makePlan → 또 비슷한 가까운 frontier
- 반복

추가 발견: **로봇 자기 셀이 UNKNOWN(-1)** 이었음.
원인: 카메라가 정면 raycast 만 → 자기 발 밑은 raycast 안 됨 → 로봇 위치는 항상 UNKNOWN.

→ patch: `camera_coverage_tracker` 에 robot self-marking (footprint 0.5m disk를 seen으로 마킹) 추가.
하지만 이건 한 증상 fix일 뿐, 근본 원인 (frontier centroid가 가까움) 은 그대로.

### 6.4 패치 누적 = 아키텍처 mismatch 신호

frontier 시도 중 누적된 patch들:
1. robot self-marking (UNKNOWN 셀 fix)
2. min_frontier_size 튜닝 (0.2 → 0.5 → 1.0 → 2.0 다 시도)
3. inflation_radius 줄임 (1.0 → 0.4m)
4. progress_timeout 튜닝 (30 → 5 → 15)
5. explore_lite 빈 frontier retry 패치
6. 사분면 zone 마스킹 (실패)
7. rotate_on_arrival 데드락 fix

각 패치는 합리적이지만 **누적된다는 사실 자체가 아키텍처 fit이 안 좋다** 는 신호.

### 6.5 결론

> **Frontier 알고리즘은 "센서가 큰 등방 영역을 한 번에 매핑한다" 는 전제를 갖고 있는데, 카메라 마스킹은 정반대로 "좁은 정면 영역만 알게 한다" 라서 알고리즘의 모든 동작 (frontier 위치, goal 선택, 진행 판정) 이 잘못된 가정 위에서 작동했고, 그래서 모든 증상이 동시에 터졌다.**

CPP로 전환 = 가정 자체를 폐기. frontier 개념 없이 "모든 격자점을 방문" 이라는 명시적 목표만 추구.

---

## 7. CPP 알고리즘 단계별 상세

### 7.1 핵심 파라미터

| 파라미터 | 기본값 | 의미 | 영향 |
|---|---|---|---|
| `waypoint_spacing_m` | 4.0 | 격자 간격 (= 카메라 raycast 반경) | 클수록 빠르지만 사각지대 위험 |
| `zone_size_m` | 15.0 | zone 한 칸 크기 | 작을수록 zone 전환 잦음 |
| `skip_already_seen` | True | 카메라 본 영역 waypoint 제외 | False면 강제 격자 모두 방문 |
| `seen_skip_radius_m` | 0.5 | 본 판정 반경 | 클수록 더 관대 (waypoint 더 많이 skip) |
| `do_spin_at_waypoint` | True | 도착 시 회전 | False면 더 빠르지만 사각지대 ↑ |
| `spin_duration_sec` | 4.0 | 회전 시간 | 길수록 카메라 더 많이 봄 |
| `spin_speed_rad_s` | 1.0 | 회전 속도 (4초 × 1 = 230°) | |
| `tick_period_sec` | 1.5 | 메인 루프 주기 | |
| `replan_period_sec` | 8.0 | waypoint 재생성 주기 | 짧을수록 새 영역 반영 빠름 |
| `startup_delay_sec` | 8.0 | Nav2 활성화 대기 | |
| `zone_align_to_free_centroid` | False | 자동 정렬 | 실험 결과 default off |

### 7.2 알고리즘 Step-by-Step

#### Step A — Waypoint 생성 (`_replan`)

**언제**:
- 노드 시작 후 startup_delay 경과 후 첫 호출
- 그 후 replan_period (8초) 마다
- 또는 waypoint 리스트 비어있을 때 즉시

**입력**: `/global_costmap/costmap` (Nav2가 만든 costmap, inflation 적용된 상태)

**알고리즘**:
```python
def _replan(self):
    info = self._costmap.info
    H, W = info.height, info.width
    res = info.resolution
    ox = info.origin.position.x
    oy = info.origin.position.y

    arr = np.array(self._costmap.data, dtype=np.int8).reshape((H, W))
    spacing_cells = max(1, int(round(self._wp_spacing / res)))
    # waypoint_spacing 4m, resolution 0.05m → spacing_cells = 80

    # zone offset 자동 정렬 (활성화 시)
    self._maybe_align_zone_grid(arr, res, ox, oy)

    # 카메라 coverage 정보 (skip_already_seen용)
    cov_arr = ... # /camera_coverage 데이터

    new_wps = {}
    skipped_seen = 0

    # 4m × 4m 윈도우마다 1개 waypoint
    half = spacing_cells // 2  # 40 cells = 2m
    for win_gy in range(half, H, spacing_cells):
        for win_gx in range(half, W, spacing_cells):
            # 윈도우 범위
            y0, y1 = max(0, win_gy - half), min(H, win_gy + half + 1)
            x0, x1 = max(0, win_gx - half), min(W, win_gx + half + 1)
            sub = arr[y0:y1, x0:x1]

            # 윈도우 내 FREE 셀 검색
            free_local_ys, free_local_xs = np.where(sub == 0)
            if len(free_local_ys) == 0:
                continue  # FREE 셀 없는 윈도우는 skip

            # 윈도우 중심에 가장 가까운 FREE 셀
            free_ys = free_local_ys + y0
            free_xs = free_local_xs + x0
            d2 = (free_ys - win_gy)**2 + (free_xs - win_gx)**2
            idx = int(np.argmin(d2))
            gy = int(free_ys[idx])
            gx = int(free_xs[idx])

            # skip_already_seen 체크
            if cov_arr is not None:
                # 그 셀 주변 0.5m disk에서 seen 비율
                sub_cov = cov_arr[gy-r:gy+r+1, gx-r:gx+r+1]
                if (sub_cov == 0).sum() / sub_cov.size >= 0.8:
                    skipped_seen += 1
                    continue

            # waypoint 등록 (이전 visited 플래그 보존)
            key = (gx, gy)
            wx = ox + (gx + 0.5) * res
            wy = oy + (gy + 0.5) * res
            existing = self._waypoints.get(key)
            visited = bool(existing and existing.get('visited'))
            zone = self._zone_of(wx, wy) if self._zone_enabled else None
            new_wps[key] = {'x': wx, 'y': wy, 'visited': visited, 'zone': zone}

    self._waypoints = new_wps
    self.get_logger().info(
        f'replan: {len(new_wps)} waypoints '
        f'({skipped_seen} skipped already-seen)'
    )
```

**왜 윈도우 내 가장 가까운 FREE 셀?**
- 격자 정확 셀이 FREE인 경우 거의 없음 (FREE 비율 < 1%)
- 윈도우 단위로 보면 거의 모든 윈도우에 FREE 1개 이상 있음
- 그 중 윈도우 중심에 가까운 거 = "이 영역의 대표점"

**왜 skip_already_seen?**
- 한 번 본 영역은 또 갈 필요 없음 → 시간 단축
- 80% threshold: 좀 보긴 했지만 부족하면 → 다시 가서 더 보기

#### Step B — Zone 할당 (`_zone_of`)

```python
def _zone_of(self, wx, wy):
    return (
        int(math.floor((wx - self._zone_offset_x) / self._zone_size)),
        int(math.floor((wy - self._zone_offset_y) / self._zone_size))
    )
```

기본 `offset = (0, 0)` → zone (0,0) 의 모서리가 map origin.

예시 (15m zone, offset=0):
- (2, 3) → zone (0, 0)
- (16, 5) → zone (1, 0)
- (-3, 8) → zone (-1, 0)
- (-3, -8) → zone (-1, -1)

#### Step C — 다음 Waypoint 선택

##### C-1. Greedy nearest (cobot3 main)

```python
def _pick_next_waypoint(self):
    rx, ry = self._get_robot_pose()

    if self._current_zone is None:
        # 시작 zone = 로봇 현재 위치의 zone
        self._current_zone = self._zone_of(rx, ry)
        self.get_logger().info(f'starting zone {self._current_zone}')

    # 현재 zone 내 unvisited 중 가장 가까운 것
    best_key, best_dist = None, float('inf')
    for key, wp in self._waypoints.items():
        if wp['visited'] or wp['zone'] != self._current_zone:
            continue
        d = math.hypot(wp['x'] - rx, wp['y'] - ry)
        if d < best_dist:
            best_dist = d
            best_key = key
    if best_key is not None:
        return best_key

    # 현재 zone 다 끝남 → 가장 가까운 unvisited zone center로 advance
    unvisited_zones = {wp['zone'] for wp in self._waypoints.values()
                       if not wp['visited']}
    if not unvisited_zones:
        return None  # 전부 끝

    def zone_center(z):
        return ((z[0] + 0.5) * self._zone_size + self._zone_offset_x,
                (z[1] + 0.5) * self._zone_size + self._zone_offset_y)

    new_zone = min(unvisited_zones,
                   key=lambda z: math.hypot(*[a-b for a,b in
                                              zip(zone_center(z), (rx, ry))]))
    self.get_logger().info(
        f'zone {self._current_zone} done → entering zone {new_zone}'
    )
    self._current_zone = new_zone
    # 새 zone에서 다시 nearest 선택
    ...
```

**약점**: zone 내에서 별 모양 백트랙 가능. 멀리 있는 waypoint도 매번 nearest로 가다보니 비효율.

##### C-2. Boustrophedon (cobot3_test 실험)

```python
def _pick_boustrophedon_in_zone(self, zone, rx, ry):
    # zone 내 unvisited waypoints
    zone_items = [(k, w) for k, w in self._waypoints.items()
                  if not w['visited'] and w['zone'] == zone]
    if not zone_items:
        return None

    # y 기준 row binning
    rows = {}
    for k, w in zone_items:
        row_idx = int(round(w['y'] / self._wp_spacing))
        rows.setdefault(row_idx, []).append((k, w))

    sorted_rows = sorted(rows.keys())
    robot_row = int(round(ry / self._wp_spacing))
    # 로봇과 가장 가까운 row index
    nearest_idx = min(range(len(sorted_rows)),
                      key=lambda i: abs(sorted_rows[i] - robot_row))

    ordered = []

    # 시작 row: 로봇 가까운 x 끝에서 시작
    first_row = sorted_rows[nearest_idx]
    first_row_wps = rows[first_row]
    xs = sorted({w['x'] for _, w in first_row_wps})
    ascending = abs(xs[0] - rx) <= abs(xs[-1] - rx)  # 어느 끝이 가까운지
    ordered.extend(sorted(first_row_wps,
                          key=lambda kw: kw[1]['x'],
                          reverse=not ascending))

    # 위쪽 rows (지그재그 alternating)
    cur_dir = not ascending
    for r in sorted_rows[nearest_idx + 1:]:
        ordered.extend(sorted(rows[r], key=lambda kw: kw[1]['x'],
                              reverse=cur_dir))
        cur_dir = not cur_dir

    # 아래쪽 rows
    cur_dir = not ascending
    for r in reversed(sorted_rows[:nearest_idx]):
        ordered.extend(sorted(rows[r], key=lambda kw: kw[1]['x'],
                              reverse=cur_dir))
        cur_dir = not cur_dir

    return ordered[0][0] if ordered else None
```

**장점**: 결정적, 백트랙 없음, 직관적.
**단점**: row 안의 일부 waypoint가 inflation으로 비어 있으면 점프 발생.

#### Step D — Goal 발행 (`_send_goal`)

```python
def _send_goal(self, key, wx, wy):
    if not self._nav_client.wait_for_server(timeout_sec=1.0):
        self.get_logger().warn('Nav2 server unavailable')
        return

    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = self._map_frame
    goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = wx
    goal_msg.pose.pose.position.y = wy

    # 핵심: orientation을 진행 방향으로
    # → MPPI가 옴니 모드라도 백워크 안 함
    pose = self._get_robot_pose()
    if pose:
        rx, ry = pose
        yaw = math.atan2(wy - ry, wx - rx)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    else:
        goal_msg.pose.pose.orientation.w = 1.0

    self._busy = True
    self._current_wp_key = key
    future = self._nav_client.send_goal_async(goal_msg)
    future.add_done_callback(self._on_goal_response)
```

**왜 yaw 설정?**: 이전엔 `orientation.w = 1.0` 만 줘서 (yaw=0 고정) Spot이 뒷걸음으로 갈 수 있었음. atan2로 진행 방향 yaw 계산해서 같이 보내면 Nav2가 도착 시 그 방향으로 회전 → 카메라가 진행 방향 봄.

#### Step E — 결과 처리 (`_on_goal_result`)

```python
def _on_goal_result(self, future):
    result = future.result()
    status = result.status

    if status == GoalStatus.STATUS_SUCCEEDED:
        if self._do_spin:
            self._mark_visited()  # visited flag set
            self._spinning = True  # spin tick이 cmd_vel 발행
            self._spin_start_ns = self.get_clock().now().nanoseconds
        else:
            self._mark_visited_and_release()  # 바로 다음 waypoint
    else:
        # ABORTED / CANCELED → visited 처리 (skip)
        # 핵심: 무한 retry 방지!
        self._mark_visited_and_release()
```

**왜 ABORTED도 visited?**:
- Nav2가 plan 못 만들거나 controller 실패한 도달 불가 waypoint
- visited 안 표시하면 다음 tick에 또 시도 → 무한 루프
- 한 번 실패하면 skip하고 다음으로 → 시스템 진행

#### Step F — 360° Spin

```python
def _spin_tick(self):
    if not self._spinning:
        return
    elapsed = (self.get_clock().now().nanoseconds - self._spin_start_ns) / 1e9
    if elapsed < self._spin_dur:
        # 4초 동안 angular cmd 발행
        t = Twist()
        t.angular.z = self._spin_speed  # 1.0 rad/s
        self._cmd_pub.publish(t)
    else:
        # 정지 신호 3회 발행 (안전)
        stop = Twist()
        for _ in range(3):
            self._cmd_pub.publish(stop)
        self._spinning = False
        self._current_wp_key = None
        self._busy = False  # 다음 waypoint 선택 가능
```

4초 × 1 rad/s = **230°** 회전. 거의 한 바퀴 (360° 못 되지만 카메라 FOV 70° 라서 충분히 cover).

### 7.3 Camera Coverage Tracker 동작

```python
def _tick(self):
    # 5Hz tick
    if self._map is None:
        return

    # TF로 카메라 위치/방향
    try:
        tf = self._tf_buffer.lookup_transform(
            self._map_frame, self._cam_frame, rclpy.time.Time()
        )
        x, y = tf.transform.translation.x, tf.transform.translation.y
        yaw = yaw_from_quat(tf.transform.rotation)
    except:
        return

    info = self._map.info
    res = info.resolution
    ox = info.origin.position.x
    oy = info.origin.position.y
    H, W = self._map_data.shape

    # 부채꼴 raycast
    n_steps = max(2, int(self._max_range / res))  # 4m / 0.05 = 80 steps
    angle_start = yaw - self._h_fov / 2.0  # -35° from yaw
    angle_step = self._h_fov / max(1, self._n_rays - 1)  # 70°/80 rays

    for i in range(self._n_rays):
        angle = angle_start + i * angle_step
        dx = math.cos(angle) * res
        dy = math.sin(angle) * res
        cx, cy = x, y
        for _ in range(n_steps):
            cx += dx
            cy += dy
            gx = int((cx - ox) / res)
            gy = int((cy - oy) / res)
            if gx < 0 or gx >= W or gy < 0 or gy >= H:
                break
            v = self._map_data[gy, gx]
            if v > 50:  # SLAM이 본 obstacle이면 ray stop
                break
            self._coverage[gy, gx] = 0  # 이 셀은 카메라가 봤음

    # 로봇 발 밑도 마킹 (카메라 raycast 사각지대 보완)
    try:
        base_tf = self._tf_buffer.lookup_transform(
            self._map_frame, self._fallback_frame, rclpy.time.Time()  # base_link
        )
        bx, by = base_tf.transform.translation.x, base_tf.transform.translation.y
        bgx = int((bx - ox) / res)
        bgy = int((by - oy) / res)
        # 0.5m disk를 seen으로
        r_cells = int(0.5 / res)
        for dyc in range(-r_cells, r_cells + 1):
            for dxc in range(-r_cells, r_cells + 1):
                if dxc**2 + dyc**2 > r_cells**2:
                    continue
                if 0 <= bgx+dxc < W and 0 <= bgy+dyc < H:
                    if self._map_data[bgy+dyc, bgx+dxc] <= 50:
                        self._coverage[bgy+dyc, bgx+dxc] = 0
    except:
        pass

    self._publish()
```

**파라미터**:
- `max_range_m: 4.0` → 4m 거리까지
- `n_rays: 80` → 80개 ray (해상도 0.875°/ray)
- `horizontal_fov_deg: 70.0` → camera_info에서 자동 갱신
- `update_rate_hz: 5.0` → 5Hz

### 7.4 Frontier vs CPP 비교표

| 측면 | Frontier (실패한 시도) | CPP (현재) |
|---|---|---|
| 목표 | 맵 unknown → known | **매 cell에 카메라 한 번씩** |
| 가정 | 큰 등방 센서 영역 | 좁은 카메라 FOV (4m, 70°) |
| 동작 단위 | frontier 군집 centroid | 격자 waypoint |
| 멈춤 위험 | instant SUCCESS, blacklist 폭주, paused 데드락 | ABORTED → visited 처리, 무한 루프 X |
| 예측 가능성 | 낮음 (점수 기반 greedy) | **결정적** (격자 + 정해진 순서) |
| "구석구석" 보장 | 불가능 (LiDAR 25m에서 본 영역 = known) | **가능** (모든 격자 직접 방문) |
| 디버깅 | 매우 어려움 (다층 cascade) | 의심 지점 적음 |
| Walltime | 예측 불가 (자주 정지) | 예측 가능 (waypoint × 평균 처리 시간) |

---

## 8. 모니터링 GUI

### 8.1 옵션 정리

| 옵션 | 결과물 | 작업량 | 외관 | 상태 |
|---|---|---|---|---|
| **A. Foxglove Studio** | apt 설치 + bridge + 데스크탑 앱 | 0줄 코드 | 매우 폴리시드 | 사용자가 설치 검토 중 |
| **B. PyQt5 커스텀 GUI** | `monitoring_gui.py` | ~500줄 | 완전 커스텀 | **작성 완료**, 실행 검증은 보류 |
| **C. 웹 대시보드** | Streamlit/React + rosbridge | 1000+줄 | 가장 모던 | 보류 |

### 8.2 Foxglove 설치 단계

```bash
# 1) bridge 설치
sudo apt install ros-humble-foxglove-bridge -y

# 2) Foxglove Studio 데스크탑 앱 받기
# 브라우저로 https://foxglove.dev/download → Linux .deb
# 또는: sudo snap install foxglove-studio

# 3) 데스크탑 앱 설치
sudo dpkg -i ~/Downloads/foxglove-*.deb
sudo apt -f install -y

# 4) bridge 실행
ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765

# 5) Foxglove Studio 열고 Open Connection → ws://localhost:8765
```

### 8.3 PyQt5 GUI 실행

빌드된 상태에서:

```bash
source /opt/ros/humble/setup.bash
source /home/rokey/dev_ws/cobot3/install/setup.bash

# numpy<2, opencv-headless 적용 후
/home/rokey/dev_ws/venv/perception/bin/python \
  /home/rokey/dev_ws/cobot3/install/cobot_perception/lib/cobot_perception/monitoring_gui
```

---

## 9. 현재 시스템 구성

### 9.1 Worktree 분리

| Worktree | 알고리즘 | Zone 정렬 | 비고 |
|---|---|---|---|
| **cobot3** (main) | greedy nearest | SLAM origin 기준 | 안정 baseline. GUI 코드 포함. yolo venv prefix 적용 |
| **cobot3_test** (실험) | **boustrophedon** | SLAM origin 기준 (auto-align off) | boustrophedon 실험 중 |

### 9.2 패키지 구성

```
cobot3/
├── src/
│   ├── cobot3/
│   │   ├── cobot_core/             ← scan_sanitizer 등 공통 유틸
│   │   ├── cobot3_navigation/      ← Nav2 params, launch
│   │   └── cobot_perception/       ← coverage_path_planner, GUI 등
│   ├── yolo/                       ← YOLO 검출 별도 패키지
│   ├── m-explore-ros2/             ← explore_lite (현재 미사용)
│   └── ...
├── docs/
│   └── 2026-05-21_exploration-and-troubleshooting.md  ← 이 문서
└── ...
```

### 9.3 cobot_perception 내부 노드

| 노드 | 역할 | 상태 |
|---|---|---|
| `coverage_path_planner` | CPP 메인 (격자, zone, 방문, spin) | 작동 |
| `camera_coverage_tracker` | 카메라 본 영역 트래킹 | 작동 |
| `stop_watchdog` | 멈춤 자동 진단 (옵션) | 작동 |
| `rotate_on_arrival` | 옛 회전 노드 (CPP에 통합되어 미사용) | 코드 잔존 |
| `camera_coverage_sweep` | 옛 2-phase sweep (미사용) | 코드 잔존 |
| `monitoring_gui` | PyQt5 대시보드 | 작동 (홀딩) |

### 9.4 venv 상태

`/home/rokey/dev_ws/venv/perception/`

| 패키지 | 버전 | 용도 |
|---|---|---|
| ultralytics | 8.4.51 | YOLO 추론 |
| torch | 2.12.0 + cu130 | ultralytics 의존성 |
| opencv-python-headless | 4.13.0 | cv_bridge 호환 (Qt 플러그인 없음) |
| numpy | 1.26.4 (`<2` 핀) | ROS cv_bridge 호환 |
| PyQt5 | (system-site-packages로 상속) | GUI |

빌드 시 항상 활성화:
```bash
source /home/rokey/dev_ws/venv/perception/bin/activate
colcon build --packages-select cobot_perception
```

---

## 10. 실행 방법

### 10.1 cobot3 (main) 실행

```bash
# 터미널 1 — main launch
source /opt/ros/humble/setup.bash
source /home/rokey/dev_ws/cobot3/install/setup.bash
ros2 launch cobot_perception spot_explore.launch.py
```

Isaac Sim 측:
1. Isaac Sim 켜기
2. `cobot3.spot` 익스텐션 활성화
3. 익스텐션 UI에서 4개 버튼 클릭:
   - Load Scene
   - Setup ROS2 CmdVel
   - Setup Camera
   - Setup LiDAR/SLAM
4. Play ▶ 버튼

### 10.2 cobot3_test (실험) 실행

```bash
# 터미널 1 — main launch
source /opt/ros/humble/setup.bash
source /home/rokey/dev_ws/cobot3/install/setup.bash       # 공통 패키지
source /home/rokey/dev_ws/cobot3_test/install/setup.bash  # cobot_perception override
ros2 launch cobot_perception spot_explore.launch.py
```

`ros2 pkg prefix cobot_perception` 으로 cobot3_test 잡혔는지 확인.

### 10.3 진단 도구

```bash
# 멈춤 감지 watchdog
> /tmp/spot_stops.log
ros2 run cobot_perception stop_watchdog

# 노드/토픽 상태 확인
ros2 node list
ros2 topic hz /spot_0/yolo/annotated_image
ros2 topic hz /camera_coverage
ros2 topic echo /coverage_zones --once
```

---

## 11. 알려진 한계 / 향후 개선

### 11.1 알려진 한계

1. **장애물 주변 사각지대**: 격자 4m 간격 + inflation 1.0m 이라 좁은 공간(<2m)은 waypoint 안 생기고 사각지대로 남음
2. **Zone 분할의 임의성**: 격자가 SLAM origin 기준이라 창고 구조와 무관하게 잘림
3. **첫 spawn 효율**: 로봇이 zone 모서리에 있어서 첫 zone 시작점까지 가는 거리가 멀 수 있음
4. **YOLO 정확도**: 4m 이상에서는 사람 검출 정확도 떨어짐 (모델 한계)
5. **TF timestamp 이슈 가능성**: Isaac이 system time, Nav2가 use_sim_time 혼재 가능 (현재는 사용자 설정 따라 다름)

### 11.2 향후 개선 가능 항목

#### 단기 (~1시간)

1. **장애물 주변 보완 sweep** — CPP가 격자 통과 후, 카메라 unseen 영역 클러스터 찾아 추가 waypoint 발행
2. **Zone-level boustrophedon** — zone 순서도 nearest 아닌 지그재그
3. **동적 spin duration** — 카메라 본 영역이 적은 waypoint는 더 오래 회전

#### 중기 (~반나절)

4. **Auto room detection** — 격자 대신 morphological erosion + connected components로 진짜 방 단위 분할
5. **TSP 경로 최적화** — zone 내 / zone 간 모두 2-opt 또는 정확한 TSP

#### 장기

6. **Adaptive waypoint density** — 장애물 많은 zone은 격자 좁게, 빈 공간은 sparse
7. **NBV (Next Best View)** — 정보 이득 최대화하는 다음 시점 선택
8. **Multi-robot 확장** — 여러 Spot이 zone 분담




