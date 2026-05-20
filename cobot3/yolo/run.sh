#!/bin/bash
# Isaac Sim에서 isaac_yolo_plane.py 실행
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
/home/kim/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh isaac_yolo_plane.py
