"""
isaac_yolo_image_test.py
Isaac Sim 환경 안에서 bus.jpg를 YOLO로 추론해 person 감지 확인.
사용법: python3 isaac_yolo_image_test.py
"""
import sys
import os

# Isaac Sim 초기화 (헤드리스 — GUI 불필요)
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import cv2
import numpy as np
from ultralytics import YOLO

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
IMAGE_PATH   = os.path.join(SCRIPT_DIR, "bus.jpg")
MODEL_PATH   = os.path.join(SCRIPT_DIR, "yolov8s.pt")
CONF_THRESH  = 0.3
PERSON_CLASS = 0   # COCO person

print(f"\n[YOLO] 모델 로드: {MODEL_PATH}")
model = YOLO(MODEL_PATH)

print(f"[이미지] 로드: {IMAGE_PATH}")
img_bgr = cv2.imread(IMAGE_PATH)
if img_bgr is None:
    print("[ERROR] 이미지를 읽을 수 없습니다.")
    simulation_app.close()
    sys.exit(1)

print(f"[이미지] 크기: {img_bgr.shape[1]}x{img_bgr.shape[0]}")

# ── 추론 ──────────────────────────────────────────────────────
results = model(img_bgr, conf=CONF_THRESH, classes=[PERSON_CLASS])[0]
boxes   = results.boxes
n       = len(boxes)

print(f"\n{'='*40}")
print(f"  감지된 사람 수: {n}")
print(f"{'='*40}")

for i, box in enumerate(boxes):
    x1, y1, x2, y2 = map(int, box.xyxy[0])
    conf = float(box.conf[0])
    print(f"  [{i+1}] bbox=({x1:4d},{y1:4d},{x2:4d},{y2:4d})  confidence={conf:.3f}")

# ── 결과 이미지 저장 ──────────────────────────────────────────
vis = img_bgr.copy()
for box in boxes:
    x1, y1, x2, y2 = map(int, box.xyxy[0])
    conf = float(box.conf[0])
    cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.putText(vis, f"person {conf:.2f}",
                (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

out_path = os.path.join(SCRIPT_DIR, "bus_isaac_result.jpg")
cv2.imwrite(out_path, vis)
print(f"\n[저장] {out_path}")
print(f"\n[결론] Isaac Sim 환경에서 YOLOv8s가 {n}명을 감지했습니다.")

simulation_app.close()
print("[완료]")
