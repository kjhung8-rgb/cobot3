"""Launch anymal_teleop.py from an Isaac UI button."""

from __future__ import annotations

import os
import shlex
import shutil
import signal
import subprocess
from pathlib import Path


class TeleopLauncher:
    def __init__(self):
        self._proc: subprocess.Popen | None = None

    @property
    def running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def start(self):
        if self.running:
            print("[cobot3.anymal] teleop terminal already running")
            return

        teleop_path = Path(__file__).resolve().parent / "tasks" / "anymal_teleop.py"
        if not teleop_path.exists():
            print(f"[cobot3.anymal] ❌ teleop script not found: {teleop_path}")
            return

        env = os.environ.copy()
        domain = env.get("ROS_DOMAIN_ID", "141")
        setup = "source /opt/ros/humble/setup.bash >/dev/null 2>&1; "
        cmd = f"export ROS_DOMAIN_ID={shlex.quote(domain)}; {setup} python3 {shlex.quote(str(teleop_path))}; exec bash"

        terminal_cmd = None
        if shutil.which("gnome-terminal"):
            terminal_cmd = ["gnome-terminal", "--", "bash", "-lc", cmd]
        elif shutil.which("terminator"):
            terminal_cmd = ["terminator", "-x", "bash", "-lc", cmd]
        elif shutil.which("konsole"):
            terminal_cmd = ["konsole", "-e", "bash", "-lc", cmd]
        elif shutil.which("xterm"):
            terminal_cmd = ["xterm", "-e", "bash", "-lc", cmd]

        try:
            if terminal_cmd:
                self._proc = subprocess.Popen(terminal_cmd, env=env, start_new_session=True)
                print(f"[cobot3.anymal] ✅ teleop terminal started / ROS_DOMAIN_ID={domain}")
            else:
                print("[cobot3.anymal] ⚠️ terminal emulator not found; launching without interactive stdin")
                self._proc = subprocess.Popen(["bash", "-lc", cmd], env=env, start_new_session=True)
        except Exception as exc:
            print(f"[cobot3.anymal] ❌ teleop terminal start failed: {exc}")

    def stop(self):
        if not self.running:
            print("[cobot3.anymal] teleop terminal is not running")
            self._proc = None
            return

        try:
            os.killpg(os.getpgid(self._proc.pid), signal.SIGTERM)
            print("[cobot3.anymal] teleop terminal stopped")
        except Exception as exc:
            print(f"[cobot3.anymal] teleop stop failed: {exc}")
        finally:
            self._proc = None
