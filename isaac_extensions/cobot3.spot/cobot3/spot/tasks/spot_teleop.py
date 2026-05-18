#!/usr/bin/env python3
"""
Spot Custom Teleop
==================
WASD + 숫자패드 방향키로 /spot_0/cmd_vel 발행

실행:
  source /opt/ros/humble/setup.bash
  python3 spot_teleop.py

키 매핑:
  W / 숫자8 / 위방향키  = 전진
  S / 숫자2 / 아래방향키 = 후진
  A / 숫자4 / 왼방향키  = 좌회전
  D / 숫자6 / 오른방향키 = 우회전
  Q                     = 속도 증가
  Z                     = 속도 감소
  SPACE                 = 정지
  ESC / Ctrl+C          = 종료
"""

import sys
import tty
import termios
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ── 설정 ──────────────────────────────
TOPIC       = "/spot_0/cmd_vel"
LIN_SPEED   = 0.5   # 기본 선속도 (m/s)
ANG_SPEED   = 1.0   # 기본 각속도 (rad/s)
SPEED_STEP  = 0.1   # 속도 조절 단위
# ──────────────────────────────────────

MSG = """
╔══════════════════════════════════╗
║   Spot Custom Teleop             ║
║   Topic: /spot_0/cmd_vel         ║
╠══════════════════════════════════╣
║  W / ↑ / Num8  : 전진           ║
║  S / ↓ / Num2  : 후진           ║
║  A / ← / Num4  : 좌회전         ║
║  D / → / Num6  : 우회전         ║
║  Q             : 속도 증가       ║
║  Z             : 속도 감소       ║
║  SPACE         : 정지           ║
║  ESC / Ctrl+C  : 종료           ║
╚══════════════════════════════════╝
"""

# 특수키 시퀀스 매핑
SPECIAL_KEYS = {
    "\x1b[A":  "UP",       # 위 방향키
    "\x1b[B":  "DOWN",     # 아래 방향키
    "\x1b[C":  "RIGHT",    # 오른쪽 방향키
    "\x1b[D":  "LEFT",     # 왼쪽 방향키
    "\x1bOA":  "UP",       # 숫자패드 8
    "\x1bOB":  "DOWN",     # 숫자패드 2
    "\x1bOC":  "RIGHT",    # 숫자패드 6
    "\x1bOD":  "LEFT",     # 숫자패드 4
}


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)

    # ESC 시퀀스 처리 (방향키)
    if key == "\x1b":
        sys.stdin.read(1)   # [
        arrow = sys.stdin.read(1)
        seq = "\x1b[" + arrow
        key = SPECIAL_KEYS.get(seq, "")

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class SpotTeleop(Node):
    def __init__(self):
        super().__init__("spot_teleop")
        self._pub = self.create_publisher(Twist, TOPIC, 10)
        self._lin = LIN_SPEED
        self._ang = ANG_SPEED
        self._running = True

    def publish(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x  = linear_x
        msg.angular.z = angular_z
        self._pub.publish(msg)

    def stop(self):
        self.publish(0.0, 0.0)

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        print(MSG)
        print(f"현재 속도: 선속도={self._lin:.1f} m/s  각속도={self._ang:.1f} rad/s")

        try:
            while self._running:
                key = get_key(settings)

                if key in ("w", "W", "UP"):
                    self.publish(self._lin, 0.0)
                    print(f"\r전진  lin={self._lin:.1f}  ang=0.0    ", end="")

                elif key in ("s", "S", "DOWN"):
                    self.publish(-self._lin, 0.0)
                    print(f"\r후진  lin=-{self._lin:.1f}  ang=0.0   ", end="")

                elif key in ("a", "A", "LEFT"):
                    self.publish(0.0, self._ang)
                    print(f"\r좌회전  lin=0.0  ang={self._ang:.1f}  ", end="")

                elif key in ("d", "D", "RIGHT"):
                    self.publish(0.0, -self._ang)
                    print(f"\r우회전  lin=0.0  ang=-{self._ang:.1f} ", end="")

                elif key in ("q", "Q"):
                    self._lin = min(self._lin + SPEED_STEP, 3.0)
                    self._ang = min(self._ang + SPEED_STEP, 3.0)
                    self.stop()
                    print(f"\n속도 증가: 선속도={self._lin:.1f}  각속도={self._ang:.1f}")

                elif key in ("z", "Z"):
                    self._lin = max(self._lin - SPEED_STEP, 0.1)
                    self._ang = max(self._ang - SPEED_STEP, 0.1)
                    self.stop()
                    print(f"\n속도 감소: 선속도={self._lin:.1f}  각속도={self._ang:.1f}")

                elif key == " ":
                    self.stop()
                    print("\r정지                              ", end="")

                elif key in ("\x1b", "\x03"):  # ESC or Ctrl+C
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
    node = SpotTeleop()

    # ROS2 spin을 별도 스레드에서 실행
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
