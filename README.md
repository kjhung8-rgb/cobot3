# Cobot3 - Isaac Sim JetBot ROS2

## 구조
cobot3/
├── isaac_extensions/
│   └── cobot3.jetbot/        # Isaac Sim Extension
│       ├── cobot3/jetbot/
│       │   ├── tasks/        # 기능별 스크립트
│       │   └── extension.py  # 메인 (UI + ROS2 Graph)
│       ├── config/
│       └── usd/              # Scene 파일
└── src/
    └── cobot3/
        └── cobot_core/       # ROS2 패키지

## 실행 순서
1. source /opt/ros/humble/setup.bash
2. Isaac Sim 실행
3. Window > Extensions > cobot3 검색 > Enable
4. UI에서 1.Load JetBot > 2.Play > 3.Setup ROS2

## 터미널
ros2 run teleop_twist_keyboard teleop_twist_keyboard

ros2 run rqt_image_view rqt_image_view
