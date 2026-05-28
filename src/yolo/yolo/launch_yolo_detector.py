"""Interpreter chooser for the YOLO detector.

The detector can run from a perception venv or from the sourced ROS/system
Python. This wrapper prefers a working venv when present, then falls back to
the current Python interpreter.
"""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys


DEFAULT_VENV_PYTHON = "/home/rokey/dev_ws/venv/perception/bin/python"
SYSTEM_TOKENS = {"", "0", "false", "none", "system", "python3"}
PRINT_FLAG = "--print-yolo-python"
PROBE_CODE = "import yolo.yolo_detector"


def _probe_env() -> dict[str, str]:
    env = os.environ.copy()
    env.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
    env.setdefault("YOLO_CONFIG_DIR", "/tmp/Ultralytics")
    return env


def _can_run_detector(python_path: str) -> bool:
    if not python_path:
        return False
    path = Path(python_path).expanduser()
    if not path.is_file() or not os.access(path, os.X_OK):
        return False

    try:
        result = subprocess.run(
            [str(path), "-c", PROBE_CODE],
            env=_probe_env(),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=8.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return result.returncode == 0


def _candidate_venv_pythons() -> list[str]:
    requested = os.environ.get("PERCEPTION_VENV_PYTHON", "").strip()
    candidates: list[str] = []
    if requested.lower() not in SYSTEM_TOKENS:
        candidates.append(requested)
    candidates.append(DEFAULT_VENV_PYTHON)

    unique: list[str] = []
    for item in candidates:
        if item and item not in unique:
            unique.append(item)
    return unique


def _select_venv_python() -> str | None:
    for python_path in _candidate_venv_pythons():
        if _can_run_detector(python_path):
            return str(Path(python_path).expanduser())
    return None


def main() -> int:
    args = [arg for arg in sys.argv[1:] if arg != PRINT_FLAG]
    selected = _select_venv_python()

    if PRINT_FLAG in sys.argv[1:]:
        print(selected or sys.executable)
        return 0

    if selected:
        os.execvpe(
            selected,
            [selected, "-m", "yolo.yolo_detector", *args],
            _probe_env(),
        )

    try:
        from yolo.yolo_detector import main as detector_main
    except Exception as exc:
        print(
            "Unable to import yolo_detector with system Python and no working "
            "PERCEPTION_VENV_PYTHON was found. Install ultralytics/cv2 in the "
            "active Python or set PERCEPTION_VENV_PYTHON to a valid venv. "
            f"Original error: {exc}",
            file=sys.stderr,
        )
        return 127

    return detector_main(args)


if __name__ == "__main__":
    raise SystemExit(main())
