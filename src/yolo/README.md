# Spot YOLO People Detection

Isaac Sim에서 Spot 로봇 카메라로 사람을 감지하고 3D 공간 좌표를 출력하는 데모입니다.

---

## 파일 설명

| 파일 | 설명 |
|---|---|
| `spot_people_demo2.py` | Spot 카메라로 사람 YOLO 감지 (좌표 출력 없음) |
| `spot_people_depth.py` | 사람 감지 + **Isaac Sim 월드 좌표** (x, y, z) 출력 |
| `spot_people_depth2.py` | 사람 감지 + **Spot 기준 상대 좌표** (x, y, z) 출력 |

### 좌표 기준 차이

- **`spot_people_depth.py`** : Isaac Sim 씬 원점 `(0, 0, 0)` 기준 절대 좌표
- **`spot_people_depth2.py`** : Spot 로봇 몸통 위치 기준 상대 좌표

---

## 실행 방법

```bash
cd /home/kim/dev_ws/cobot3/src/yolo
```

```bash
# 월드 좌표 버전
/home/kim/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh spot_people_depth.py

# Spot 상대 좌표 버전
/home/kim/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh spot_people_depth2.py
```

### launch 파일로 실행

```bash
ros2 launch src/yolo/launch/spot_people_depth.launch.py
ros2 launch src/yolo/launch/spot_people_depth2.launch.py
```

---

## 출력 예시

```
=======================================================
  [YOLO] step=120  감지: 3/4명
=======================================================
  [1] conf=0.90  depth=3.21m  world=(3.62, -0.05, 0.72)m
  [2] conf=0.84  depth=2.86m  world=(2.86, 1.87, 0.45)m
  [3] conf=0.75  depth=3.58m  world=(3.99, -0.73, 0.69)m
```

---

## 환경

- Isaac Sim 5.1.0
- YOLOv8s
- Python 3.11 (Isaac Sim 내장)
- NVIDIA RTX 2070
