# Cobot3 — Spot + Jackal 멀티로봇 구조 시나리오

Isaac Sim 5.1 + ROS2 Humble 기반 **search-and-rescue 시뮬레이션**.

- **Spot**: 창고 자율탐사 (SLAM) + 3대 RGB-D 카메라 YOLO 생존자 감지
- **Jackal**: mission_manager가 좌표를 FIFO 큐로 받아 Nav2로 출동·구조
- **GUI (PyQt5)**: 통합 모니터링 — 맵/카메라/생존자/모드 제어 + 세션 영구화

---

## 📌 목차

1. [시나리오 개요](#-시나리오-개요)
2. [데이터 흐름](#-데이터-흐름)
3. [디렉터리 구조](#-디렉터리-구조)
4. [요구사항](#-요구사항)
5. [빌드 & 환경 설정](#-빌드--환경-설정)
6. [실행 순서](#-실행-순서)
7. [패키지/노드 상세](#-패키지노드-상세)
8. [토픽 · 액션 카탈로그](#-토픽--액션-카탈로그)
9. [GUI 사용법](#-gui-사용법)
10. [핵심 알고리즘](#-핵심-알고리즘)
11. [멀티 로봇 충돌 방지](#-멀티-로봇-충돌-방지-메커니즘)
12. [파라미터 튜닝](#-파라미터-튜닝)
13. [동작 검증 · 디버깅](#-동작-검증--디버깅)
14. [트러블슈팅](#-트러블슈팅)
15. [팀](#-팀-chapter-2)

---

## 🎯 시나리오 개요

```
┌─────────────────────────┐                     ┌──────────────────────────┐
│ Spot (정찰)              │                     │ Jackal (구조)             │
│ ─────────────────────── │                     │ ──────────────────────── │
│ • SLAM (/map)           │ ──/map 공유─────►   │ • global_costmap (정적)   │
│ • Coverage Path Planning│                     │ • mission_manager (FIFO 큐)│
│ • YOLOv8 person 검출    │ ──/detected_pose───►│ • Nav2 NavigateToPose     │
│ • 3대 카메라 (270° FOV) │                     │ • Local LiDAR 회피        │
└─────────────────────────┘                     └──────────────────────────┘
              ↑                                              ↑
              └─── 모니터링 GUI (PyQt5) ─────────────────────┘
                   · 맵+카메라+생존자 실시간 시각화
                   · 모드 전환 / 우클릭 골 / 키보드 teleop
                   · 세션 영구화 (~/.ros/cobot3_survivors.json)
```

### 동작 한 줄 요약
> Spot이 zone 단위로 자율탐사하면서 카메라+YOLO로 생존자를 발견 → 좌표를 mission_manager에 발행 → Jackal이 FIFO 큐 순서대로 구조 → 큐 비면 홈 복귀.

---

## 🔁 데이터 흐름

### 정찰 (Spot)
```
SpotFlatTerrainPolicy ──┐
                        │   /spot_0/cmd_vel
   coverage_path_planner ─► (cmd_vel_relay) ─► /spot_0/cmd_vel ─► Isaac
              ▲                                                       │
              │ /coverage_zones                                       │
              │ /coverage_waypoints                                   ▼
              │                                  /spot_0/scan → scan_sanitizer
              ▼                                          (jackal 마스킹)
   /camera_coverage ◄── camera_coverage_tracker          │
              ▲              ▲                            ├──► /spot_0/scan_slam (SLAM 입력)
              │              │                            └──► /spot_0/scan_nav  (Nav2 입력)
              │     /spot_0/{front,left,right}_cam/*
              │              │
              │              ▼
              │       yolo_detector ──► /detected_survivor_pose
              │              │
              │              └──► /spot_0/yolo/annotated_image
              ▼
        Nav2 (default-ns)                slam_toolbox ──► /map
        controller_server                                   │
        planner_server                                      ├─► spot global_costmap (static)
        bt_navigator                                        └─► jackal global_costmap (static, 공유)
```

### 구조 (Jackal)
```
/detected_survivor_pose ──►│
/jackal_0/manual_goal    ──►│ mission_manager
/jackal_0/return_home    ──►│ (FIFO 큐 + 상태머신)
/jackal_0/mission_resume ──►│
                            └──► /jackal_0/navigate_to_pose (action)
                                                │
                                                ▼
                            Jackal Nav2 stack (/jackal_0/* namespace)
                            • bt_navigator
                            • planner_server (NavfnPlanner, tolerance 1.5m)
                            • controller_server (MPPIController)
                            • behavior_server (spin/backup/wait/drive_on_heading)
                            • velocity_smoother
                                │
                                │ /jackal_0/cmd_vel_nav_smoothed
                                ▼
                            jackal_cmd_vel_relay
                            (autonomous/manual mux)
                                │
                                │  /jackal_0/cmd_vel
                                ▼
                            Isaac OmniGraph
                                │
                                ▼  /jackal_0/scan
                            jackal_nav_scan_sanitizer (spot 마스킹)
                                │  /jackal_0/scan_nav
                                ▼
                            Jackal local_costmap obstacle_layer
```

---

## 📁 디렉터리 구조

```
cobot3_test/
├── README.md                                    # 본 문서
├── .gitignore                                   # __pycache__/build/install/usd 백업 등
│
├── isaac_extensions/                            # Isaac Sim Omniverse 익스텐션
│   ├── cobot3.spot/                             # ★ 메인 익스텐션
│   │   ├── config/extension.toml                # 메타정보, 의존성
│   │   ├── usd/g8.usd                           # 창고 + spot/jackal 씬
│   │   └── cobot3/
│   │       ├── spot/                            # Spot 본체
│   │       │   ├── extension.py                 # UI 패널 통합 (Spot/Jackal/Carter)
│   │       │   ├── scene.py                     # 씬 로드 + 카메라/조명/spot policy
│   │       │   ├── ros_graphs.py                # cmd_vel/odom/scan/cam OmniGraph
│   │       │   ├── constants.py                 # 토픽/프레임/카메라 spec
│   │       │   ├── teleop_launcher.py           # 터미널 teleop subprocess
│   │       │   └── tasks/spot_teleop.py
│   │       ├── jackal/                          # Jackal 본체
│   │       │   ├── scene.py                     # Jackal USD ref + headlight + box
│   │       │   ├── ros_graphs.py                # cmd_vel + scan + TF graph
│   │       │   ├── panel.py                     # UI 블록 (Setup Jackal ROS 버튼)
│   │       │   └── constants.py                 # /jackal_0 토픽/프레임/LiDAR Z
│   │       ├── carter/                          # (deprecated, 사용 안 함)
│   │       ├── utils.py                         # get_ros_domain_id()
│   │       └── __init__.py
│   │
│   ├── cobot3.anymal/                           # ANYmal 익스텐션 (옵션, RL+rough terrain)
│   └── cobot3.jetbot/                           # JetBot 익스텐션 (deprecated)
│
└── src/
    ├── cobot3/
    │   ├── cobot_perception/                    # ★ 미션·탐사·모니터링
    │   │   ├── cobot_perception/
    │   │   │   ├── mission_manager.py           # Jackal 미션 dispatcher (FIFO 큐)
    │   │   │   ├── coverage_path_planner.py     # Spot 자율탐사 (grid+zone)
    │   │   │   ├── camera_coverage_tracker.py   # 카메라 본 영역 누적 → /camera_coverage
    │   │   │   ├── monitoring_gui.py            # PyQt5 통합 대시보드
    │   │   │   ├── rotate_on_arrival.py         # (옵션) 도착 후 회전
    │   │   │   ├── camera_coverage_sweep.py     # (옵션) Phase2 sweep
    │   │   │   ├── stop_watchdog.py             # cmd_vel 정지 진단 (옵션)
    │   │   │   └── spot_obstacle_publisher.py   # (옵션) spot→jackal dual cloud
    │   │   ├── launch/
    │   │   │   ├── jackal_full.launch.py        # ★ 메인 진입점 (pkill+세션리셋)
    │   │   │   ├── spot_explore.launch.py       # SLAM+Nav2+YOLO+CPP
    │   │   │   ├── jackal_localize.launch.py    # map→jackal_0/odom static TF
    │   │   │   ├── jackal_navigate.launch.py    # Jackal Nav2 + mission_manager
    │   │   │   ├── carter_localize.launch.py    # (deprecated)
    │   │   │   ├── carter_navigate.launch.py    # (deprecated)
    │   │   │   ├── cobot3_full.launch.py        # (deprecated, Spot+Carter)
    │   │   │   └── survivor_detector.launch.py  # YOLO only (테스트)
    │   │   ├── config/explore.yaml              # explore_lite 잔존 (사용 안 함)
    │   │   └── package.xml / setup.py
    │   │
    │   ├── cobot_core/                          # 공통 노드
    │   │   ├── cobot_core/
    │   │   │   ├── scan_sanitizer.py            # ★ 다중로봇 자기-마스킹 LaserScan filter
    │   │   │   ├── action_node.py               # JSON command 진입점
    │   │   │   ├── action_manager.py            # 로봇별 base_action 디스패치
    │   │   │   ├── command_router.py            # /robot_command → /{ns}/action_command
    │   │   │   ├── send_json_command.py         # CLI 테스트 유틸
    │   │   │   └── base_action.py
    │   │   ├── launch/                          # robot_system / *_slam launch
    │   │   └── config/                          # *_slam_toolbox.yaml
    │   │
    │   └── cobot3_navigation/                   # Nav2 파라미터 + cmd_vel relay
    │       ├── params/
    │       │   ├── spot_navigation_params.yaml  # Spot Nav2 (footprint/inflation/MPPI)
    │       │   ├── spot_slam_params.yaml        # Spot SLAM Toolbox
    │       │   ├── jackal_nav2_params.yaml      # Jackal Nav2 (global=static only)
    │       │   ├── jackal_amcl_params.yaml      # (옵션, 현재 미사용)
    │       │   ├── carter_*.yaml                # (deprecated)
    │       │   └── anymal_navigation_params.yaml
    │       ├── launch/                          # spot_navigation / spot_slam 등
    │       ├── scripts/cmd_vel_relay.py         # ★ autonomous/manual cmd_vel mux
    │       ├── rviz2/                           # rviz config (옵션)
    │       └── maps/                            # SLAM 동적 생성 (empty + .gitkeep)
    │
    ├── yolo/                                    # YOLOv8 검출 파이프라인
    │   ├── yolo/
    │   │   ├── yolo_detector.py                 # ★ RGB+Depth → /detected_survivor_pose
    │   │   └── survivor_pose_to_marker.py       # PoseStamped → RViz Marker
    │   ├── launch/yolo_pipeline.launch.py       # venv python 사용
    │   ├── config/
    │   │   ├── yolo_detector.yaml               # confidence/depth_range/카메라 토픽
    │   │   └── survivor_pose_to_marker.yaml
    │   ├── models/yolov8s.pt                    # COCO 학습 가중치
    │   └── dectected_person/                    # 캡쳐 이미지 저장 (gitignored)
    │
    └── m-explore-ros2/                          # (deprecated, coverage_path_planner로 대체)
```

---

## ✅ 요구사항

| 항목 | 버전 |
|------|------|
| Ubuntu | 22.04 LTS |
| ROS2 | Humble Hawksbill |
| Isaac Sim | 5.1.x |
| Python venv (PERCEPTION_VENV_PYTHON) | 3.10 + ultralytics |
| GPU | NVIDIA (CUDA 11+, Isaac Sim 요구사항) |
| PyQt5 | 시스템 패키지 |

추가 ROS2 패키지:
```bash
sudo apt install -y \
  ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-cv-bridge ros-humble-tf2-ros \
  ros-humble-rviz2 ros-humble-teleop-twist-keyboard
```

YOLO 의존성 (venv 권장):
```bash
python3 -m venv ~/dev_ws/venv/perception
source ~/dev_ws/venv/perception/bin/activate
pip install ultralytics opencv-python
```

---

## 🔨 빌드 & 환경 설정

```bash
cd ~/dev_ws/cobot3_test
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=141   # Spot/Jackal 모두 동일하게
```

특정 패키지만:
```bash
colcon build --symlink-install --packages-select cobot_perception cobot3_navigation cobot_core yolo
```

---

## 🚀 실행 순서

### 1) Isaac Sim 세팅
1. Isaac Sim 실행
2. `Window > Extensions > cobot3.spot` 검색 → **Enable**
3. 패널 (좌측)에서 순서대로 클릭:
   - **Load Scene (Spot)** — `g8.usd` 창고 로드 + Spot 추가
   - **Load Scene (Jackal)** — Jackal 추가
   - **Play** (timeline 재생) — 물리/렌더링 시작
   - **Setup Spot ROS** — 카메라/LiDAR/cmd_vel/odom OmniGraph 생성
   - **J. Setup Jackal ROS** — Jackal LiDAR/cmd_vel/scan-TF graph 생성

### 2) ROS2 풀 스택 (한 터미널)
```bash
ros2 launch cobot_perception jackal_full.launch.py
```

내부 자동 시퀀스:
| 시점 | 동작 |
|------|------|
| T+0s | **`_kill_prior_instances`** — 이전 세션 노드 SIGKILL + `~/.ros/cobot3_survivors.json` + capture 이미지 삭제 |
| T+1s | `jackal_localize` — `static_transform_publisher`로 map→jackal_0/odom 고정 TF |
| T+3s | `spot_explore` — SLAM + nav2 + scan_sanitizer + cmd_vel_relay + yolo + survivor_pose_to_marker + camera_coverage_tracker + coverage_path_planner |
| T+23s | `jackal_navigate` — Jackal Nav2 + jackal_cmd_vel_relay + mission_manager |

### 3) 모니터링 GUI
```bash
ros2 run cobot_perception monitoring_gui
```

### 4) (옵션) 더미 좌표로 jackal 단독 검증
```bash
ros2 topic pub --once /detected_survivor_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0}, orientation: {w: 1.0}}}"
```

---

## 📦 패키지/노드 상세

### `cobot_perception` (★ 핵심)
| 노드 | 역할 |
|------|------|
| `mission_manager` | Jackal 미션 dispatcher. `/detected_survivor_pose`·`/jackal_0/manual_goal`·`/jackal_0/return_home`·`/jackal_0/mission_resume` 구독. FIFO 큐 + 상태머신 (IDLE / BUSY{rescue,manual,home}). dedup, ABORT 패턴 알림(`/jackal_0/mission_alert`), 자동 홈 복귀, active goal 보존+재개. |
| `coverage_path_planner` | Spot 자율탐사. `/global_costmap/costmap` + `/map` + `/camera_coverage` 입력. **결정적 grid sampling** + zone 분할 + best-score selection. `/coverage_zones`·`/coverage_waypoints` marker 발행. `/navigate_to_pose`(spot) action 호출. |
| `camera_coverage_tracker` | 3대 카메라 TF로 ray-casting → 본 영역 OccupancyGrid 누적 → `/camera_coverage` 발행. CPP의 skip/cancel/종료 판정에 사용. |
| `monitoring_gui` | PyQt5 통합 대시보드 (카메라 파노라마, 맵, 상태, 생존자, 캡쳐). `/control_mode`·`/jackal_0/control_mode`·`/teleop_cmd_vel`·`/jackal_0/teleop_cmd_vel`·`/jackal_0/manual_goal`·`/jackal_0/return_home`·`/jackal_0/mission_resume` 발행. `/jackal_0/mission_alert` 구독 popup. |
| `rotate_on_arrival` | (옵션) Spin action으로 360° 회전 — 도착 시 시야 확보. |
| `camera_coverage_sweep` | (옵션) phase2 — 미관찰 free cell cluster 방문. |
| `stop_watchdog` | (옵션) cmd_vel 정지 감지 → 진단 스냅샷 로깅. |
| `spot_obstacle_publisher` | (옵션) Spot 위치를 dual PointCloud2 (marking/clearing)로 Jackal costmap에 주입. 현재는 scan_sanitizer 마스킹으로 대체. |

### `cobot_core`
| 노드 | 역할 |
|------|------|
| `scan_sanitizer` | LaserScan 필터. `mask_frames` 파라미터의 TF 위치를 LiDAR 좌표계로 변환 → 그 위치 주변 `mask_radius_m` 안의 ray를 `inf`로 치환. Spot scan(jackal mask) / Jackal scan(spot mask) 양방향 사용. |
| `action_node` | JSON `/action_command` 구독 → `ActionManager` 위임 (테스트용). |
| `command_router` | `/robot_command` 글로벌 명령 → `/{robot}/action_command` namespace 라우팅. |
| `send_json_command` | CLI 명령 발행 유틸. |

### `cobot3_navigation`
| 항목 | 역할 |
|------|------|
| `scripts/cmd_vel_relay.py` | autonomous/manual 모드 mux. `/control_mode` 구독 → autonomous면 `/cmd_vel`(nav2) forward, manual면 `/teleop_cmd_vel` forward → output topic으로. Spot/Jackal 양쪽 인스턴스. YOLO slowdown dampening 옵션. |
| `params/spot_navigation_params.yaml` | Spot Nav2: footprint 0.64×0.44, inflation 1.0m, MPPI critics, NavfnPlanner |
| `params/spot_slam_params.yaml` | Spot SLAM Toolbox: async 모드, scan topic `/spot_0/scan_slam` |
| `params/jackal_nav2_params.yaml` | Jackal Nav2: footprint 0.51×0.43, inflation 0.3m, **global=static only**, local=lidar |
| `launch/spot_navigation.launch.py` | Spot Nav2 stack include |
| `launch/spot_slam.launch.py` | Spot SLAM only |

### `yolo`
| 노드 | 역할 |
|------|------|
| `yolo_detector` | YOLOv8 추론 + RGB-Depth 동기화. 3카메라 라운드로빈. person 박스 → ROI depth median → 카메라 좌표 → TF로 map 좌표 변환 → `/detected_survivor_pose` 발행. `/spot_0/yolo/annotated_image` 시각화. 신뢰 기반 confirmed survivor 추적. |
| `survivor_pose_to_marker` | PoseStamped → Marker (빨강 X + 중심 sphere). `/survivor_goal_marker` 발행 (latched). |

### `isaac_extensions/cobot3.spot/cobot3/`
| 파일 | 역할 |
|------|------|
| `spot/extension.py` | Omniverse extension entry. 3개 로봇 패널 UI 생성 (Spot/Jackal/Carter). |
| `spot/scene.py` | `SpotFireRescue(BaseSample)`. 창고 로드 + Spot RL 정책 + 카메라/헤드라이트/화재광/천장광 추가. `_disable_other_lights`, `_start_atmosphere_flicker` 분위기 연출. |
| `spot/ros_graphs.py` | OmniGraph 생성 (`setup_camera_graph`, `setup_slam_sensors`, `setup_cmd_vel_odom_graph`). |
| `spot/constants.py` | 토픽 prefix `/spot_0`, 카메라 spec (90° FOV, 90° 간격), HEADLIGHT, FIRE_LIGHT, CEILING_LIGHT 좌표/강도. |
| `jackal/scene.py` | `add_jackal_to_world`. 헤드라이트 + 구호상자 추가. idempotent. |
| `jackal/ros_graphs.py` | `setup_jackal_all`: cmd_vel + scan + static TF graph 한꺼번에. PhysX Lidar 사용. |
| `jackal/constants.py` | `/jackal_0` namespace, LiDAR Z = 0.30m, footprint, headlight intensity. |
| `utils.py` | `get_ros_domain_id()` 환경변수 파싱. |

---

## 🔌 토픽 · 액션 카탈로그

### 입력 (사용자/감지)
| 토픽 | 타입 | 발행자 | 구독자 |
|------|------|--------|--------|
| `/detected_survivor_pose` | PoseStamped | yolo_detector | mission_manager, survivor_pose_to_marker, monitoring_gui |
| `/jackal_0/manual_goal` | PoseStamped | monitoring_gui (우클릭) | mission_manager |
| `/jackal_0/return_home` | Bool | monitoring_gui (버튼) | mission_manager |
| `/jackal_0/mission_resume` | Bool | monitoring_gui (자동/재개) | mission_manager |
| `/control_mode` | String | monitoring_gui | cmd_vel_relay (spot) |
| `/jackal_0/control_mode` | String | monitoring_gui | jackal_cmd_vel_relay |
| `/teleop_cmd_vel` | Twist | monitoring_gui (키보드) | cmd_vel_relay (spot) |
| `/jackal_0/teleop_cmd_vel` | Twist | monitoring_gui (키보드) | jackal_cmd_vel_relay |
| `/jackal_0/mission_alert` | String | mission_manager (ABORT 패턴) | monitoring_gui popup |

### Nav2 액션
| 액션 | 타입 | 클라이언트 | 서버 |
|------|------|------------|------|
| `/navigate_to_pose` | nav2_msgs/NavigateToPose | coverage_path_planner | bt_navigator (spot) |
| `/jackal_0/navigate_to_pose` | nav2_msgs/NavigateToPose | mission_manager | jackal_0/bt_navigator |

### 시각화/모니터링
| 토픽 | 타입 | 발행자 |
|------|------|--------|
| `/map` | OccupancyGrid | slam_toolbox (Spot) — **두 로봇 공유** |
| `/camera_coverage` | OccupancyGrid | camera_coverage_tracker |
| `/coverage_zones` | MarkerArray (latched) | coverage_path_planner |
| `/coverage_waypoints` | MarkerArray (latched) | coverage_path_planner |
| `/survivor_goal_marker` | Marker (latched) | survivor_pose_to_marker |
| `/global_costmap/costmap`, `/jackal_0/global_costmap/costmap` | OccupancyGrid | 각 Nav2 |

### 센서 (Isaac OmniGraph)
| 토픽 | 비고 |
|------|------|
| `/spot_0/{front,left,right}_cam/{color_image,depth_image,camera_info}` | 3대 카메라 RGB-D + intrinsics |
| `/spot_0/scan` (raw) | spot LiDAR |
| `/spot_0/scan_slam` | sanitizer 출력 → slam_toolbox 입력 (jackal masked) |
| `/spot_0/scan_nav` | sanitizer 출력 → spot Nav2 obstacle_layer 입력 |
| `/spot_0/odom` | Spot odometry |
| `/jackal_0/scan` (raw) | jackal LiDAR |
| `/jackal_0/scan_nav` | sanitizer 출력 → jackal local_costmap (spot masked) |
| `/jackal_0/odom` | Jackal odometry |
| `/jackal_0/cmd_vel` | 최종 Isaac 입력 (cmd_vel_relay 출력) |

### YOLO
| 토픽 | 타입 |
|------|------|
| `/spot_0/yolo/{annotated_image,left_annotated_image,right_annotated_image}` | Image (BGR) |
| `/spot_0/yolo/person_detected` | Bool |
| `/spot_0/yolo/slowdown_required` | Bool (cmd_vel_relay dampening 트리거) |

---

## 🖥 GUI 사용법

### 레이아웃
```
┌───────────────────────────────────────────────────────────┐
│   왼쪽 cam │ 중앙 cam │ 오른쪽 cam   (파노라마, 270°)        │
├───────────────────────────────────────────────────────────┤
│ [자율탐사][Spot 수동][Jackal 수동][Jackal 자동][Spot 복귀]   │
│ [Jackal 복귀]                              ⚠ 알림 라벨      │
├──────────┬─────────────────────────┬──────────────────────┤
│  상태     │                          │ 발견된 생존자          │
│  경과시간 │                          │  (목록, 삭제 가능)     │
│  Zone    │   맵 + spot/jackal + 생존자│                      │
│  Waypoint│                          │                       │
│  탐색률   │                          ├──────────────────────┤
│          │                          │ 최근 생존자 사진       │
└──────────┴─────────────────────────┴──────────────────────┘
```

### 모드 버튼
| 버튼 | 동작 |
|------|------|
| **자율탐사** | spot + jackal 모두 autonomous |
| **Spot 수동조작** | spot 키보드, jackal autonomous 유지 |
| **Jackal 수동조작** | jackal 키보드, spot autonomous 유지 |
| **Jackal 자동/재개** | (1) manual→auto OR (2) home 중에 cancel + 큐 dispatch — 통합 |
| **Spot 복귀** | spot 자율탐사 종료 + 홈 |
| **Jackal 복귀** | 진행 중 mission cancel + active를 큐 맨 앞에 보존 → home → 도착 후 큐 자동 재개 |

### 맵 우클릭
- jackal manual goal 발행 + 노란 J 핀 표시
- mission_manager의 dedup/큐 흐름 그대로 통과

### 키보드 (manual 모드에서)
| 키 | 동작 |
|----|------|
| W/S | 전진/후진 |
| A/D | 좌측/우측 평행이동 (spot only) |
| Q/E | 좌/우 회전 |
| X | 가속 |
| Z | 감속 |
| Space | 비상정지 |

### 세션 영구화
- `~/.ros/cobot3_survivors.json` — 발견된 생존자 + map_first_t (SLAM 시작 시각)
- GUI 재시작 시 자동 복원 → 경과시간 + 생존자 목록 유지
- **launcher 재시작 시 자동 삭제** → 새 세션 깨끗하게 시작

---

## ⚙ 핵심 알고리즘

### Spot 자율탐사 — `coverage_path_planner`
- **결정적 grid sampling** (frontier 사용 안 함)
- `waypoint_spacing_m: 4.5` — 카메라 시야 단위 (max_depth 8m의 절반)
- `zone_size_m: 20.0` — 한 zone 끝낸 후 가장 가까운 다음 zone으로 advance
- `zone_advance_visited_ratio: 0.8` — 80%+ visited면 남은 자투리 무시하고 강제 advance
- `skip_already_seen` + `seen_skip_min_ratio: 0.65` — 카메라가 65%+ 본 disk는 waypoint 후보에서 제외
- `_maybe_inflight_seen_cancel` — 주행 중 그 영역이 covered되면 현재 goal cancel → 다음 waypoint
- `auto_return_coverage_threshold: 0.95` — 전체 95% covered면 자동 홈 복귀 + 탐사 완료

### Jackal 미션 — `mission_manager`
- **상태**: `IDLE` ↔ `BUSY{kind ∈ rescue, manual, home}`
- **FIFO 큐 (`_pending_queue: list[PoseStamped]`)** — 미션 중 들어온 새 좌표 모두 append
- **dedup**: 0.5m 이내는 중복으로 skip (last_dispatched/active/queue 비교)
- **자동 홈 복귀**: rescue/manual mission 완료 후 큐 비어있으면 home
- **수동 강제 홈**: 진행 중 active goal을 큐 맨 앞에 보존 → home → 도착 후 자동 재개
- **resume**: home 가는 중 cancel + 큐 dispatch
- **ABORT 패턴**: N회 연속 ABORT 시 `/jackal_0/mission_alert` String 발행 → GUI popup

### 카메라 커버리지 — `camera_coverage_tracker`
- 3대 카메라 TF(`spot_0/{front,left,right}_cam_link`) lookup
- 각 카메라의 FOV(`camera_info`에서 자동 갱신) 안에서 ray-casting
- ray가 `/map`의 obstacle에 닿기 전까지 cell을 "seen(0)"으로 표시
- 누적 OccupancyGrid (-1 unseen / 0 seen) → `/camera_coverage` 발행 (latched)
- `camera_coverage` 비율을 coverage_path_planner의 자동 종료 조건으로 활용

---

## 🔗 멀티 로봇 충돌 방지 메커니즘

| 충돌 유형 | 해결 |
|----------|------|
| 노드 이름 충돌 | Jackal nav2 모든 노드를 `/jackal_0/*` namespace 분리 |
| nav2 nested plugin params | `RewrittenYaml(root_key=JACKAL_NS, ...)` |
| nav2_bringup의 Humble 버그 | Jackal은 nav2_bringup 우회 — 각 nav2 노드를 개별 `Node(...)` spawn |
| TF 프레임 충돌 | spot: `spot_0/*`, jackal: `jackal_0/*` prefix |
| cmd_vel 토픽 | spot: `/spot_0/cmd_vel`, jackal: `/jackal_0/cmd_vel` |
| scan 토픽 | spot: `/spot_0/scan`, jackal: `/jackal_0/scan` |
| 상대 로봇이 자기 LiDAR에 잡혀 영구 obstacle 학습 | `scan_sanitizer`로 mask_frames TF 위치 ±radius rays를 inf 치환 |
| Lifecycle manager 간섭 | spot/jackal 각자 자기 `lifecycle_manager_navigation` 보유 |
| `/map` 공유 | 의도적 공유 — 단일 좌표계, jackal global_costmap.static_layer가 직접 구독 |
| 액션 충돌 | spot: `/navigate_to_pose`, jackal: `/jackal_0/navigate_to_pose` |

---

## 🛠 파라미터 튜닝

### Spot 자율탐사 — `launch/spot_explore.launch.py`
| 파라미터 | 기본값 | 효과 |
|----------|--------|------|
| `waypoint_spacing_m` | 4.5 | grid 간격. 작을수록 촘촘 |
| `zone_size_m` | 20.0 | zone 크기 |
| `zone_advance_visited_ratio` | 0.8 | 80%+ visited면 강제 advance |
| `seen_skip_min_ratio` | 0.65 | 카메라 disk seen 비율 ≥ → skip |
| `seen_skip_radius_m` | 0.5 | seen 판단 반경 |
| `goal_clearance_radius_m` | 0.35 | waypoint immediate clearance 검사 반경 |
| `goal_clearance_min_free_ratio` | 0.3 | clearance 내 free 비율 최소 |
| `obstacle_clearance_m` | 0.0 (disabled) | strict lethal-cell 거리. 0이면 비활성 |
| `auto_return_coverage_threshold` | 0.95 | 자동 종료 임계 |
| `auto_return_hold_sec` | 5.0 | 임계 도달 후 hold 시간 |
| `replan_period_sec` | 8.0 | waypoint 재계산 주기 |
| `tick_period_sec` | 1.5 | 메인 tick 주기 |
| `do_spin_at_waypoint` | False | 도착 시 회전 (시간 절약 위해 OFF) |

### Jackal 미션 — `launch/jackal_navigate.launch.py`
| 파라미터 | 기본값 | 효과 |
|----------|--------|------|
| `dedup_distance_m` | 0.5 | 중복 좌표 판단 반경 |
| `rescue_max_retries` | 0 | ABORT 시 retry 횟수 (0=즉시 mission_complete) |
| `rescue_retry_step_m` | 2.0 | retry 시 jackal 위치에서 target 방향 step |
| `home_return_enabled` | True | 미션 후 자동 홈 |
| `home_arrival_radius_m` | 0.5 | 이미 home 근처면 home dispatch 생략 |
| `abort_alert_threshold` | 3 | 연속 ABORT N회 시 알림 |
| `manual_goal_topic` | `/jackal_0/manual_goal` | GUI 우클릭 input |
| `return_home_topic` | `/jackal_0/return_home` | home 트리거 |
| `resume_topic` | `/jackal_0/mission_resume` | home 중단+큐 dispatch |

### scan_sanitizer 마스킹
| 파라미터 | spot SLAM | spot Nav | jackal Nav |
|----------|----------|----------|------------|
| `mask_frames` | jackal_0/base_link | jackal_0/base_link | spot_0/base_link |
| `mask_radius_m` | 0.45 | 0.45 | 0.3 |
| `mask_max_range_m` | 20.0 | 20.0 | 20.0 |

### Jackal Nav2 — `cobot3_navigation/params/jackal_nav2_params.yaml`
- `global_costmap`: `static_layer + inflation_layer`만 (obstacle_layer 제거)
- `local_costmap`: `obstacle_layer (lidar) + inflation_layer`
- `inflation_radius`: 0.3m, `cost_scaling_factor`: 3.0
- `footprint`: `[[0.26, 0.22], [0.26, -0.22], [-0.26, -0.22], [-0.26, 0.22]]`
- MPPI controller, NavfnPlanner (`tolerance: 1.5`)
- `observation_persistence`: 1.0s (local_costmap에서 FOV 벗어나도 1초간 obstacle 유지)

### Isaac Sim 환경
| 항목 | 위치 | 비고 |
|------|------|------|
| 카메라 FOV/회전 | `spot/constants.py` | `CAMERA_HORIZONTAL_APERTURE_MM`, `*_CAMERA_ROTATION_XYZ_DEG` |
| Spot 헤드라이트 | `spot/constants.py` | `HEADLIGHT_INTENSITY` |
| Jackal 헤드라이트 | `jackal/constants.py` | `JACKAL_HEADLIGHT_INTENSITY` (200000) |
| Jackal LiDAR Z | `jackal/constants.py:50` | `JACKAL_LIDAR_TRANSLATION = (0, 0, 0.30)` |
| Spot LiDAR Z | `spot/ros_graphs.py:231` | `LIDAR_TRANSLATION = Gf.Vec3d(0.25, 0, 0.15)` |
| 화재광 강도 | `spot/scene.py:447` | `FIRE_BASE_INTENSITY = 80000` |
| 천장광 강도 | `spot/scene.py:444` | `CEILING_ON_INTENSITY = 25000` (flicker) |
| Ground friction | `spot/scene.py:103` | `static_friction = dynamic_friction = 0.2` |

---

## 🧪 동작 검증 · 디버깅

### 노드 살아있나
```bash
ros2 node list | grep -E "mission_manager|coverage_path_planner|slam_toolbox|yolo_detector"
```

### nav2 lifecycle
```bash
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /jackal_0/bt_navigator
ros2 lifecycle get /controller_server
ros2 lifecycle get /jackal_0/controller_server
```

### 카메라 영상 확인 (annotated)
```bash
ros2 run rqt_image_view rqt_image_view /spot_0/yolo/annotated_image
```

### 더미 생존자 발행 (jackal 단독 테스트)
```bash
ros2 topic pub --once /detected_survivor_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0}, orientation: {w: 1.0}}}"
```

### 수동 홈 트리거
```bash
ros2 topic pub --once /jackal_0/return_home std_msgs/msg/Bool '{data: true}'
```

### Coverage 진행률
```bash
ros2 topic echo /camera_coverage --once --field info
# width × height × resolution^2 × seen_ratio = covered area in m²
```

### 세션 데이터 초기화
```bash
rm ~/.ros/cobot3_survivors.json
rm -f src/yolo/dectected_person/survivor_*.{jpg,png,jpeg}
```

### TF tree 확인
```bash
ros2 run tf2_tools view_frames
# map → spot_0/odom, spot_0/base_link, spot_0/lidar_link, spot_0/{front,left,right}_cam_link
# map → jackal_0/odom (static) → jackal_0/base_link → jackal_0/laser
```

---

## 🐛 트러블슈팅

| 증상 | 원인 / 확인 |
|------|-------------|
| spot 안 움직임 | `ros2 lifecycle get /bt_navigator`가 `active`? `coverage_path_planner ready` 로그? `→ wp [N/M]` dispatch 로그? |
| jackal 안 움직임 | `/jackal_0/bt_navigator` lifecycle 확인. inactive면 launcher 재시작 (자동 활성화 가끔 실패) |
| 생존자 좌표 발행 안 됨 | YOLO 로그 `Detected person, but no valid person-depth sample` → depth NaN/0. 카메라 더 가까이/각도 조정 |
| coverage_path_planner가 `0 waypoints` | `goal_clearance_min_free_ratio` 너무 빡빡 (현재 0.3) — 더 낮추거나 `obstacle_clearance_m`을 0으로 |
| 모드 전환 후 jackal 멈춤 | `_set_jackal_auto_mode` 버그 (해결됨). 그래도 발생 시 `/jackal_0/control_mode` echo로 모드 확인 |
| 같은 곳 ABORT 반복 | `mission_alert` 3회면 popup 뜸. 키보드로 jackal 빼주거나 다른 좌표 dispatch |
| jackal LiDAR scan 안 나옴 | Isaac 콘솔에서 `Linear Depth data and Intensities data sizes do not match` 에러? Setup Jackal ROS 재클릭 |
| Isaac OmniGraph 멈춤 | Stop → Play 다시. 또는 익스텐션 reload |
| 다중 로봇 두 번째 launch 후 동작 안 함 | launcher의 `_kill_prior_instances`가 정상 동작했는지 콘솔 로그 확인 |
| GUI 시간이 0:00에서 안 올라감 | `/map`이 아직 publish 안 됨 (SLAM 시작 전). slam_toolbox 노드 확인 |

---

## 🧠 알려진 제약 / 향후 개선

| 항목 | 현재 | 개선 방향 |
|------|------|----------|
| Jackal 위치 추정 | static TF만 (`jackal_localize.launch.py`) | AMCL 도입 → odom drift 보정 |
| 2D LiDAR 한계 | 곡선/공중 obstacle 일부 miss | 3D PointCloud + voxel_layer |
| 좌표 보정 | nav2 planner tolerance 1.5m fallback에만 의존 | nearest-free shell 검색 (application 레벨) |
| Lifecycle 활성화 | 가끔 실패 (수동 복구) | startup 재시도 OpaqueFunction |
| 화재/연기 | SphereLight flicker (라이팅 한정) | Omni Flow plugin (실제 particle) |
| spot-jackal cooperative planning | scan_sanitizer 마스킹 (수동적) | Dynamic obstacle layer (서로 위치 explicit publish) |
| YOLO depth sampling | 거리값 NaN/0면 좌표 발행 안 됨 | depth interpolation 또는 다중 frame fusion |

---

## 👥 팀 (Chapter 2)

| 멤버 | 역할 |
|------|------|
| 김나경 (팀장) | Project Manager · Isaac Sim 모듈 · 자율탐사 알고리즘 · 기초 프레임워크 |
| 김재흥 | 시스템 통합 / Git · ROS2 모듈 · SLAM · 기초 프레임워크 |
| 이하영 | Nav2 자율주행 설계 · 모니터링 GUI |
| 김덕영 | YOLO 생존자 검출 · 파이프라인 · 인물 Asset |
| 안수빈 | Isaac Sim 재난 시나리오 · 환경 구축 · 장애물/화재 씬 모델링 |

---

## 📄 라이센스

내부 프로젝트 — 별도 명시 없음.
