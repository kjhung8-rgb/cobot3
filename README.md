# Cobot3 - Spot + Jackal Rescue Flow

## 현재 구성

```text
cobot3/
├── isaac_extensions/
│   └── cobot3.spot/                  # Spot + Jackal Isaac Sim extension
└── src/
    ├── cobot3/
    │   ├── cobot_perception/         # full launch, exploration, mission, GUI
    │   └── cobot3_navigation/        # Spot/Jackal Nav2 params, cmd_vel relay
    └── yolo/                         # YOLO detector + survivor marker
```

## Alias

```bash
alias perception="ros2 launch cobot_perception jackal_full.launch.py"
alias mon_gui='source /opt/ros/humble/setup.bash && source ~/dev_ws/cobot3/install/setup.bash && ros2 run cobot_perception monitoring_gui'
```

## 빌드

```bash
cd /home/rokey/dev_ws/cobot3
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

```bash
export PERCEPTION_VENV_PYTHON=/home/rokey/dev_ws/venv/perception/bin/python
```

```bash
export PERCEPTION_VENV_PYTHON=system
```

## 실행 순서

1. Isaac Sim 실행
2. `cobot3.spot` extension enable
3. `Load Scene (Jackal)`
4. Play
5. `Setup Spot ROS`
6. `J. Setup Jackal ROS`
7. `perception`
8. `mon_gui`

## Main Launch

`jackal_full.launch.py`

```text
jackal_localize.launch.py    # map -> jackal_0/odom static TF
spot_explore.launch.py       # Spot SLAM + Nav2 + coverage planner + YOLO
jackal_navigate.launch.py    # Jackal Nav2 + mission_manager
```

## 남은 ROS Executable

```text
cobot_perception camera_coverage_tracker
cobot_perception coverage_path_planner
cobot_perception mission_manager
cobot_perception monitoring_gui
cobot_perception scan_sanitizer
cobot_perception spot_obstacle_publisher
cobot3_navigation cmd_vel_relay.py
yolo yolo_detector
yolo survivor_pose_to_marker
```

## 빠른 점검

```bash
colcon list
ros2 launch cobot_perception jackal_full.launch.py --show-args
ros2 pkg executables cobot_perception
``
