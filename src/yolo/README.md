# YOLO Survivor Detection

ROS2 package `yolo` detects people from the Spot front RGB-D camera, publishes an annotated image for RViz2, and publishes survivor coordinates in the `map` frame.

## Nodes

| Executable | Role |
|---|---|
| `yolo_detector` | Runs YOLO, uses depth to estimate a 3D person position, and publishes survivor poses |
| `survivor_pose_to_marker` | Converts survivor `PoseStamped` messages into RViz2 markers |

## Topics

| Topic | Type | Description |
|---|---|---|
| `/spot_0/yolo/annotated_image` | `sensor_msgs/Image` | RGB image with YOLO bounding boxes |
| `/spot_0/yolo/person_detected` | `std_msgs/Bool` | Person detection flag |
| `/spot_0/yolo/person_pose_base` | `geometry_msgs/PoseStamped` | Person pose in `spot_0/base_link` |
| `/detected_survivor_pose` | `geometry_msgs/PoseStamped` | Survivor pose in `map` |
| `/survivor_delete_id` | `std_msgs/Int32` | Delete survivor by ID; `0` clears all survivors |
| `/survivor_goal_marker` | `visualization_msgs/Marker` | Survivor marker for RViz2 |

## Run

```bash
source /home/rokey/dev_ws/cobot3/install/setup.bash
ros2 launch yolo yolo_pipeline.launch.py
```

The full exploration pipeline starts this package from `cobot_perception`:

```bash
source /home/rokey/dev_ws/cobot3/install/setup.bash
ros2 launch cobot_perception spot_explore.launch.py
```

## Manual Delete

Delete survivor marker and detector memory for survivor `#2`:

```bash
ros2 topic pub --once /survivor_delete_id std_msgs/msg/Int32 "{data: 2}"
```

Clear all survivor markers and detector memory:

```bash
ros2 topic pub --once /survivor_delete_id std_msgs/msg/Int32 "{data: 0}"
```

## Files

| Path | Role |
|---|---|
| `yolo/yolo_detector.py` | YOLO RGB-D detection and TF conversion node |
| `yolo/survivor_pose_to_marker.py` | `PoseStamped` to RViz2 `Marker` bridge |
| `config/yolo_detector.yaml` | Detector topics, frames, model, and confidence settings |
| `config/survivor_pose_to_marker.yaml` | Marker topic and visual settings |
| `launch/yolo_pipeline.launch.py` | Launches the detector and marker nodes |
| `yolov8s.pt` | Default YOLOv8 model file |
