"""Jackal mission dispatcher.

기능:
  1. RESCUE — /detected_survivor_pose로 들어온 좌표를 jackal nav action에 dispatch.
     dedup, pending queue, ABORT 재시도 (optional).
  2. MANUAL — 사용자 클릭 좌표 (RViz 2D Nav Goal 등을 /jackal_0/manual_goal
     PoseStamped로 발행) → 동일 dispatch 파이프라인.
  3. HOME 복귀 — mission 끝나면 자동으로 시작 위치 복귀 (param 토글).
     수동 트리거 토픽 (Bool /jackal_0/return_home)도 지원.

spot은 이 dispatcher의 관심 밖 — 별도로 coverage_path_planner가 zone 탐색을 계속함.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


GOAL_STATUS_ABORTED = 6


STATE_IDLE = "IDLE"
STATE_BUSY = "BUSY"  # rescue / manual / home 모두 같은 BUSY 상태로 통합 (kind로 구분)


KIND_RESCUE = "rescue"
KIND_MANUAL = "manual"
KIND_HOME = "home"


class MissionManager(Node):
    def __init__(self):
        super().__init__("mission_manager")

        self.declare_parameter("input_topic", "/detected_survivor_pose")
        self.declare_parameter("manual_goal_topic", "/jackal_0/manual_goal")
        self.declare_parameter("return_home_topic", "/jackal_0/return_home")
        self.declare_parameter("resume_topic", "/jackal_0/mission_resume")
        self.declare_parameter("rescue_action", "/jackal_0/navigate_to_pose")
        self.declare_parameter("dedup_distance_m", 0.5)
        # 0 = ABORT 즉시 mission_complete (retry 없음).
        self.declare_parameter("rescue_max_retries", 0)
        self.declare_parameter("rescue_retry_step_m", 2.0)
        # jackal 위치 lookup용.
        self.declare_parameter("robot_base_frame", "jackal_0/base_link")
        self.declare_parameter("map_frame", "map")
        # home 복귀 토글. True면 mission 끝날 때마다 자동으로 시작 위치로.
        self.declare_parameter("home_return_enabled", True)
        # 이미 home 근처면 복귀 skip 반경 (m).
        self.declare_parameter("home_arrival_radius_m", 0.5)

        in_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        manual_topic = self.get_parameter("manual_goal_topic").get_parameter_value().string_value
        home_trigger_topic = self.get_parameter("return_home_topic").get_parameter_value().string_value
        resume_topic = self.get_parameter("resume_topic").get_parameter_value().string_value
        rescue_action = self.get_parameter("rescue_action").get_parameter_value().string_value
        self._dedup_d = self.get_parameter("dedup_distance_m").get_parameter_value().double_value
        self._rescue_max_retries = int(self.get_parameter("rescue_max_retries").value)
        self._rescue_retry_step = float(self.get_parameter("rescue_retry_step_m").value)
        self._robot_base_frame = self.get_parameter("robot_base_frame").get_parameter_value().string_value
        self._map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        self._home_return_enabled = bool(self.get_parameter("home_return_enabled").value)
        self._home_arrival_r = float(self.get_parameter("home_arrival_radius_m").value)

        self._state = STATE_IDLE
        self._active_kind: str | None = None
        self._active_pose: PoseStamped | None = None
        self._rescue_target_original: PoseStamped | None = None
        self._rescue_retries = 0
        # FIFO 큐 — 미션 중 들어온 새 좌표는 append, mission_complete 시 pop(0).
        # 단일 슬롯이 아니라 진짜 큐 → 다수 detection 안 덮어씀.
        self._pending_queue: list[PoseStamped] = []
        self._last_dispatched: PoseStamped | None = None
        self._goal_handle = None
        # home pose는 첫 TF lookup 성공 시 capture (lazy). 한 번 잡으면 고정.
        self._home_pose: PoseStamped | None = None
        # 사용자가 수동 home 요청했지만 BUSY일 때 — mission_complete 후 처리.
        self._home_request_pending = False

        self._sub_survivor = self.create_subscription(PoseStamped, in_topic, self._on_pose, 10)
        self._sub_manual = self.create_subscription(PoseStamped, manual_topic, self._on_manual_goal, 10)
        self._sub_home = self.create_subscription(Bool, home_trigger_topic, self._on_home_trigger, 10)
        self._sub_resume = self.create_subscription(Bool, resume_topic, self._on_resume, 10)
        self._rescue_client = ActionClient(self, NavigateToPose, rescue_action)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # home capture timer — 첫 TF lookup 성공할 때까지 1Hz 재시도.
        self._home_capture_timer = self.create_timer(1.0, self._try_capture_home_pose)

        self.get_logger().info(
            f"MissionManager ready: survivor={in_topic}, manual={manual_topic}, "
            f"home_trigger={home_trigger_topic}, rescue={rescue_action} "
            f"(dedup {self._dedup_d:.2f}m, retries={self._rescue_max_retries}, "
            f"home_return={'on' if self._home_return_enabled else 'off'})"
        )

    # ── helpers ──────────────────────────────────────────────────────
    def _planar_distance(self, a: PoseStamped, b: PoseStamped) -> float:
        return math.hypot(
            a.pose.position.x - b.pose.position.x,
            a.pose.position.y - b.pose.position.y,
        )

    def _is_duplicate(self, msg: PoseStamped) -> bool:
        # last_dispatched + active + 큐의 모든 원소와 비교.
        refs: list[PoseStamped] = []
        if self._last_dispatched is not None:
            refs.append(self._last_dispatched)
        if self._active_pose is not None:
            refs.append(self._active_pose)
        refs.extend(self._pending_queue)
        for ref in refs:
            if self._planar_distance(msg, ref) < self._dedup_d:
                return True
        return False

    def _make_goal(self, msg: PoseStamped) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = msg.header.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose = msg.pose
        return goal

    def _lookup_robot_pose(self) -> PoseStamped | None:
        try:
            tf = self._tf_buffer.lookup_transform(self._map_frame, self._robot_base_frame, Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        pose = PoseStamped()
        pose.header.frame_id = self._map_frame
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = tf.transform.rotation
        return pose

    def _try_capture_home_pose(self) -> None:
        if self._home_pose is not None:
            self._home_capture_timer.cancel()
            return
        pose = self._lookup_robot_pose()
        if pose is None:
            return
        self._home_pose = pose
        self._home_capture_timer.cancel()
        self.get_logger().info(
            f"[HOME] start pose captured: "
            f"({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})"
        )

    # ── detection inflow ─────────────────────────────────────────────
    def _on_pose(self, msg: PoseStamped) -> None:
        self._intake(msg, label="survivor")

    def _on_manual_goal(self, msg: PoseStamped) -> None:
        self.get_logger().info(
            f"[MANUAL] user click: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
        )
        self._intake(msg, label="manual", manual=True)

    def _intake(self, msg: PoseStamped, *, label: str, manual: bool = False) -> None:
        if not manual and self._is_duplicate(msg):
            return
        if self._state != STATE_IDLE:
            self._pending_queue.append(msg)
            self.get_logger().info(
                f"Pending queued [{label}] ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}) — "
                f"queue size {len(self._pending_queue)}, state={self._state}/{self._active_kind}"
            )
            return
        kind = KIND_MANUAL if manual else KIND_RESCUE
        self._start_mission(msg, kind)

    def _on_home_trigger(self, msg: Bool) -> None:
        if not msg.data:
            return
        if self._state != STATE_IDLE:
            self.get_logger().info(
                "[HOME] manual trigger received — canceling current goal & forcing home"
            )
            # 진행 중이던 rescue/manual goal을 큐 맨 앞에 보존 → home 후 자동 재개.
            # home/이미 cancel 중이면 보존 안 함.
            if self._active_pose is not None and self._active_kind in (KIND_RESCUE, KIND_MANUAL):
                self._pending_queue.insert(0, self._active_pose)
                ap = self._active_pose.pose.position
                self.get_logger().info(
                    f"[HOME] active goal ({ap.x:.2f}, {ap.y:.2f}) 큐 맨 앞에 보존 "
                    f"→ home 후 재개. 큐 size {len(self._pending_queue)}"
                )
            self._home_request_pending = True
            if self._goal_handle is not None:
                try:
                    self._goal_handle.cancel_goal_async()
                except Exception as exc:
                    self.get_logger().warn(f"[HOME] cancel 실패: {exc}")
            return
        self._dispatch_home_if_needed(reason="manual")

    def _on_resume(self, msg: Bool) -> None:
        """home 가는 중에 resume 받으면 home 취소 → 큐의 다음 좌표로 재개."""
        if not msg.data:
            return
        if self._state != STATE_BUSY or self._active_kind != KIND_HOME:
            self.get_logger().info(
                f"[RESUME] 무시 — home 가는 중이 아님 (state={self._state}/{self._active_kind})"
            )
            return
        self.get_logger().info(
            f"[RESUME] home 중단 — 큐의 다음 좌표로 재개 (큐 {len(self._pending_queue)}개)"
        )
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(f"[RESUME] cancel 실패: {exc}")

    # ── dispatch ─────────────────────────────────────────────────────
    def _start_mission(self, msg: PoseStamped, kind: str) -> None:
        self._active_pose = msg
        self._last_dispatched = msg
        self._go_rescue(kind=kind)

    def _go_rescue(self, *, kind: str, retry: bool = False) -> None:
        """jackal에 navigate_to_pose dispatch. kind=rescue/manual/home로 결과 처리 분기."""
        self._state = STATE_BUSY
        self._active_kind = kind

        if not retry:
            self._rescue_target_original = self._active_pose
            self._rescue_retries = 0
            goal_pose = self._active_pose
        else:
            goal_pose = self._compute_retry_step()
            if goal_pose is None:
                self.get_logger().warn("[RESCUE] retry — jackal pose lookup 실패, 종료")
                self._mission_complete()
                return

        tag = {KIND_RESCUE: 'RESCUE', KIND_MANUAL: 'MANUAL', KIND_HOME: 'HOME'}[kind]
        retry_suffix = f' retry{self._rescue_retries}' if retry else ''
        self.get_logger().info(
            f"[{tag}{retry_suffix}] jackal → "
            f"({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})"
        )

        if not self._rescue_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f"[{tag}] jackal action server not ready — abort")
            self._mission_complete()
            return

        send_future = self._rescue_client.send_goal_async(self._make_goal(goal_pose))
        send_future.add_done_callback(self._on_goal_accepted)

    def _compute_retry_step(self) -> PoseStamped | None:
        cur = self._lookup_robot_pose()
        if cur is None or self._rescue_target_original is None:
            return None

        jx = cur.pose.position.x
        jy = cur.pose.position.y
        tx = self._rescue_target_original.pose.position.x
        ty = self._rescue_target_original.pose.position.y
        dx, dy = tx - jx, ty - jy
        d = math.hypot(dx, dy)
        if d < 0.1:
            return self._rescue_target_original
        step = min(self._rescue_retry_step, d)
        nx = jx + step * dx / d
        ny = jy + step * dy / d

        new_pose = PoseStamped()
        new_pose.header.frame_id = self._map_frame
        new_pose.pose.position.x = nx
        new_pose.pose.position.y = ny
        new_pose.pose.position.z = 0.0
        new_pose.pose.orientation = self._rescue_target_original.pose.orientation
        return new_pose

    def _on_goal_accepted(self, future) -> None:
        if self._state != STATE_BUSY:
            return
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn(f"[{self._active_kind}] goal rejected")
            self._mission_complete()
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_done)

    def _on_goal_done(self, future) -> None:
        if self._state != STATE_BUSY:
            return
        status = future.result().status
        kind = self._active_kind
        self.get_logger().info(
            f"[{kind}] done status={status} (retries={self._rescue_retries})"
        )

        # rescue/manual에만 retry 적용 (home은 단발).
        if (kind in (KIND_RESCUE, KIND_MANUAL)
                and status == GOAL_STATUS_ABORTED
                and self._rescue_retries < self._rescue_max_retries):
            self._rescue_retries += 1
            self.get_logger().info(
                f"[{kind}] ABORTED → retry {self._rescue_retries}/{self._rescue_max_retries} "
                f"(step {self._rescue_retry_step:.1f}m)"
            )
            self._go_rescue(kind=kind, retry=True)
            return

        self._mission_complete()

    def _mission_complete(self) -> None:
        prev_kind = self._active_kind
        self._state = STATE_IDLE
        self._active_kind = None
        self._active_pose = None
        self._rescue_target_original = None
        self._rescue_retries = 0
        self._goal_handle = None

        # 우선순위: 사용자 수동 home > pending 큐 > 자동 home (rescue/manual 끝 후)
        # 수동 home 클릭해도 큐는 그대로 유지 — home 도착 후 _mission_complete가
        # 다시 호출되며 큐에 좌표 남아있으면 차례대로 dispatch.
        if self._home_request_pending:
            self._home_request_pending = False
            if self._pending_queue:
                self.get_logger().info(
                    f"[HOME] pending queue 보존 ({len(self._pending_queue)} pose) — home 후 dispatch"
                )
            self._dispatch_home_if_needed(reason="manual force")
            return

        if self._pending_queue:
            pending = self._pending_queue.pop(0)
            self.get_logger().info(
                f"[NEXT] pending dispatch ({pending.pose.position.x:.2f}, "
                f"{pending.pose.position.y:.2f}) — {len(self._pending_queue)} left in queue"
            )
            self._start_mission(pending, KIND_RESCUE)
            return

        if self._home_return_enabled and prev_kind in (KIND_RESCUE, KIND_MANUAL):
            self._dispatch_home_if_needed(reason="auto after mission")

    def _dispatch_home_if_needed(self, *, reason: str) -> None:
        if self._home_pose is None:
            self.get_logger().warn(f"[HOME] {reason} — home pose 아직 capture 못함, skip")
            return
        cur = self._lookup_robot_pose()
        if cur is not None:
            d = self._planar_distance(cur, self._home_pose)
            if d < self._home_arrival_r:
                self.get_logger().info(
                    f"[HOME] {reason} — already at home (d={d:.2f}m < {self._home_arrival_r:.2f}m), skip"
                )
                return
        self.get_logger().info(f"[HOME] {reason} — returning to start")
        self._active_pose = self._home_pose
        # home은 dedup/last_dispatched에 영향 안 줌 (survivor 좌표와 무관)
        self._go_rescue(kind=KIND_HOME)


def main():
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
