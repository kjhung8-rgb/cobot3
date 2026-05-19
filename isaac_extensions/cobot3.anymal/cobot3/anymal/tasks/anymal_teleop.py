#!/usr/bin/env python3
"""
ANYmal C Teleop
===============
WASD + 방향키로 /anymal_0/cmd_vel 발행

실행:
  source /opt/ros/humble/setup.bash
  python3 anymal_teleop.py

키 매핑:
  W / 위방향키   = 전진
  S / 아래방향키 = 후진
  A / 왼방향키   = 좌회전
  D / 오른방향키 = 우회전
  Q              = 속도 증가
  Z              = 속도 감소
  SPACE          = 정지
  ESC / Ctrl+C   = 종료
"""

import sys
import tty
import termios
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ── 설정 ──────────────────────────────
TOPIC       = "/anymal_0/cmd_vel"
LIN_SPEED   = 0.3   # 기본 선속도 (m/s) — ANYmal은 Spot보다 보수적으로
ANG_SPEED   = 0.5   # 기본 각속도 (rad/s)
SPEED_STEP  = 0.05  # 속도 조절 단위
LIN_MAX     = 0.5
ANG_MAX     = 0.5
# ──────────────────────────────────────

MSG = """
╔══════════════════════════════════╗
║   ANYmal C Teleop                ║
║   Topic: /anymal_0/cmd_vel       ║
╠══════════════════════════════════╣
║  W / ↑  : 전진                  ║
║  S / ↓  : 후진                  ║
║  A / ←  : 좌회전                ║
║  D / →  : 우회전                ║
║  Q      : 속도 증가              ║
║  Z      : 속도 감소              ║
║  SPACE  : 정지                  ║
║  ESC / Ctrl+C : 종료             ║
╚══════════════════════════════════╝
"""

SPECIAL_KEYS = {
    "\x1b[A": "UP",
    "\x1b[B": "DOWN",
    "\x1b[C": "RIGHT",
    "\x1b[D": "LEFT",
}


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)

    if key == "\x1b":
        second = sys.stdin.read(1)
        third = sys.stdin.read(1)
        seq = "\x1b" + second + third
        key = SPECIAL_KEYS.get(seq, "")

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class AnymalTeleop(Node):
    def __init__(self):
        super().__init__("anymal_teleop")
        self._pub = self.create_publisher(Twist, TOPIC, 10)
        self._lin = LIN_SPEED
        self._ang = ANG_SPEED
        self._running = True

    def publish(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._pub.publish(msg)

    def stop(self):
        self.publish(0.0, 0.0)

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        print(MSG)
        print(f"현재 속도: 선속도={self._lin:.2f} m/s  각속도={self._ang:.2f} rad/s")

        try:
            while self._running:
                key = get_key(settings)

                if key in ("w", "W", "UP"):
                    self.publish(self._lin, 0.0)
                    print(f"\r전진  lin={self._lin:.2f}  ang=0.0    ", end="")

                elif key in ("s", "S", "DOWN"):
                    self.publish(-self._lin, 0.0)
                    print(f"\r후진  lin=-{self._lin:.2f}  ang=0.0   ", end="")

                elif key in ("a", "A", "LEFT"):
                    self.publish(0.0, self._ang)
                    print(f"\r좌회전  lin=0.0  ang={self._ang:.2f}  ", end="")

                elif key in ("d", "D", "RIGHT"):
                    self.publish(0.0, -self._ang)
                    print(f"\r우회전  lin=0.0  ang=-{self._ang:.2f} ", end="")

                elif key in ("q", "Q"):
                    self._lin = min(self._lin + SPEED_STEP, LIN_MAX)
                    self._ang = min(self._ang + SPEED_STEP, ANG_MAX)
                    self.stop()
                    print(f"\n속도 증가: 선속도={self._lin:.2f}  각속도={self._ang:.2f}")

                elif key in ("z", "Z"):
                    self._lin = max(self._lin - SPEED_STEP, 0.05)
                    self._ang = max(self._ang - SPEED_STEP, 0.05)
                    self.stop()
                    print(f"\n속도 감소: 선속도={self._lin:.2f}  각속도={self._ang:.2f}")

                elif key == " ":
                    self.stop()
                    print("\r정지                              ", end="")

                elif key in ("\x1b", "\x03"):
                    print("\n종료합니다...")
                    self._running = False
                    break

                else:
                    self.stop()
                    print(f"\r대기 중...                        ", end="")

        except Exception as e:
            print(f"\n에러: {e}")
        finally:
            self.stop()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main():
    rclpy.init()
    node = AnymalTeleop()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
