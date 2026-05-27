# Cobot3 full stack (Jackal variant) — single-terminal launcher.
#
# Mirrors cobot3_full.launch.py but swaps carter_localize/carter_navigate for
# the jackal equivalents.
#
# spot_explore.launch.py already spawns yolo_detector + survivor_pose_to_marker
# inline, so we do NOT include yolo_pipeline.launch.py here (would duplicate).
#
# Order (TimerAction-staged):
#   T+0s : jackal_localize (static map→jackal_0/odom TF). Published FIRST so
#          that scan_sanitizer's TF lookup (spot_0/lidar_link → jackal_0/base_link)
#          is resolvable from the very first scan processed by SLAM. Otherwise
#          SLAM bakes jackal into /map during the brief window before
#          jackal_localize comes up, and those cells persist.
#   T+2s : spot_explore (SLAM, exploration, default-ns nav2, yolo, sanitizers).
#          By now jackal_0/odom TF is up; static_transform_publisher latches the
#          message so subscribers get it on first connect.
#   T+22s: jackal_navigate (nav2 + mission_manager). Longer delay so spot's
#          nav2 lifecycle finishes activating first; otherwise jackal's
#          lifecycle change_state calls timeout under contention.

import os
import signal
import subprocess
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _include(pkg: str, launch_file: str) -> IncludeLaunchDescription:
    path = os.path.join(get_package_share_directory(pkg), "launch", launch_file)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path))


# 이전 런처가 살아있는 경우 nav2/SLAM 노드 중복 → topic 충돌 + lifecycle
# 활성화 실패. Python OpaqueFunction으로 직접 kill (bash -c 쓰면 cmdline에
# 패턴 문자열이 들어가 pkill -f가 자기 자신 죽임).
_PKILL_PATTERNS = [
    "async_slam_toolbox_node",
    "mission_manager",
    "coverage_path_planner",
    "camera_coverage_tracker",
    "survivor_pose_to_marker",
    "scan_sanitizer",
    "cmd_vel_relay",
    "yolo_detector",
    "controller_server",
    "planner_server",
    "smoother_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
    "lifecycle_manager",
    "static_transform_publisher",
]


def _kill_prior_instances(context, *args, **kwargs):
    my_pid = os.getpid()
    killed = []
    for pat in _PKILL_PATTERNS:
        try:
            out = subprocess.check_output(["pgrep", "-f", pat], text=True)
            pids = [int(p) for p in out.split() if p.strip()]
        except subprocess.CalledProcessError:
            continue
        for pid in pids:
            if pid == my_pid:
                continue  # 자기 자신 보호 — bash cmdline에 패턴 안 들어가도 ros2 launch python cmdline엔 들어감
            try:
                os.kill(pid, signal.SIGKILL)
                killed.append((pat, pid))
            except ProcessLookupError:
                pass
            except PermissionError:
                pass
    if killed:
        print(f"[jackal_full] killed prior: {len(killed)} procs ({sorted({p for p, _ in killed})})")

    # 새 launcher 세션 시작 — 이전 세션 GUI 상태 (생존자 좌표 + map_first_t) 초기화.
    survivors_path = Path.home() / ".ros" / "cobot3_survivors.json"
    try:
        if survivors_path.exists():
            survivors_path.unlink()
            print(f"[jackal_full] reset session data: {survivors_path}")
    except Exception as exc:
        print(f"[jackal_full] session reset 실패: {exc}")

    # YOLO 캡쳐 이미지도 정리 (LatestCapturePanel에 이전 사진 leak 방지).
    capture_candidates = [
        Path.cwd() / "src/yolo/dectected_person",
        Path(__file__).resolve().parents[4] / "src/yolo/dectected_person",
        Path.home() / "dev_ws" / "cobot3_test" / "src/yolo/dectected_person",
    ]
    seen_dirs = set()
    for d in capture_candidates:
        if d in seen_dirs or not d.is_dir():
            continue
        seen_dirs.add(d)
        try:
            removed = 0
            for pat in ("survivor_*.jpg", "survivor_*.jpeg", "survivor_*.png"):
                for f in d.glob(pat):
                    f.unlink()
                    removed += 1
            if removed:
                print(f"[jackal_full] cleared {removed} capture(s) in {d}")
        except Exception as exc:
            print(f"[jackal_full] capture cleanup 실패 ({d}): {exc}")

    time.sleep(0.5)
    return []


def generate_launch_description():
    spot_explore = _include("cobot_perception", "spot_explore.launch.py")
    jackal_localize = _include("cobot_perception", "jackal_localize.launch.py")
    jackal_navigate = _include("cobot_perception", "jackal_navigate.launch.py")

    return LaunchDescription(
        [
            OpaqueFunction(function=_kill_prior_instances),
            TimerAction(period=1.0, actions=[jackal_localize]),
            TimerAction(period=3.0, actions=[spot_explore]),
            TimerAction(period=23.0, actions=[jackal_navigate]),
        ]
    )
