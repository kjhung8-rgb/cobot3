"""YOLOv8 survivor detector and RGB-D localizer for Spot cameras."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Optional

import cv2
import message_filters
import numpy as np
import rclpy
import tf2_geometry_msgs
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Int32
from ultralytics import YOLO


PERSON_CLASS_ID = 0  # COCO 'person'


@dataclass
class ConfirmedSurvivor:
    survivor_id: int
    pose: PoseStamped


@dataclass
class SurvivorCandidate:
    pose: PoseStamped
    observations: int
    last_seen_ns: int
    samples: list[tuple[float, float, float]]


@dataclass
class CameraStream:
    name: str
    image_topic: str
    depth_topic: str
    camera_info_topic: str
    annotated_topic: str
    camera_info: Optional[CameraInfo] = None
    image_sub: object = None
    depth_sub: object = None
    sync: object = None
    pub_image: object = None
    last_inference_time: object = None
    last_annotated_pub_time: object = None
    last_pose_pub_time: object = None


class SurvivorDetector(Node):
    def __init__(self):
        super().__init__("yolo_detector")

        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("image_topic", "/spot_0/front_cam/color_image")
        self.declare_parameter("depth_topic", "/spot_0/front_cam/depth_image")
        self.declare_parameter("camera_info_topic", "/spot_0/front_cam/camera_info")
        self.declare_parameter("annotated_topic", "/spot_0/yolo/annotated_image")
        self.declare_parameter("multi_camera", False)
        self.declare_parameter("camera_names", ["front"])
        self.declare_parameter("image_topics", ["/spot_0/front_cam/color_image"])
        self.declare_parameter("depth_topics", ["/spot_0/front_cam/depth_image"])
        self.declare_parameter("camera_info_topics", ["/spot_0/front_cam/camera_info"])
        self.declare_parameter("annotated_topics", ["/spot_0/yolo/annotated_image"])
        self.declare_parameter("detected_topic", "/spot_0/yolo/person_detected")
        self.declare_parameter("slowdown_required_topic", "/spot_0/yolo/slowdown_required")
        self.declare_parameter("base_pose_topic", "/spot_0/yolo/person_pose_base")
        self.declare_parameter("survivor_pose_topic", "/detected_survivor_pose")
        self.declare_parameter("survivor_delete_topic", "/survivor_delete_id")
        self.declare_parameter("confidence_threshold", 0.4)
        self.declare_parameter("device", "cuda")
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("inference_period_sec", 0.1)
        self.declare_parameter("annotated_publish_period_sec", 0.1)
        self.declare_parameter("person_only", True)
        self.declare_parameter("base_frame", "spot_0/base_link")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("use_latest_tf", True)
        self.declare_parameter("tf_timeout_sec", 0.2)
        self.declare_parameter("sync_queue_size", 5)
        self.declare_parameter("sync_slop_sec", 0.08)
        self.declare_parameter("depth_sample_radius_px", 5)
        self.declare_parameter("min_depth_m", 0.2)
        self.declare_parameter("max_depth_m", 30.0)
        self.declare_parameter("pose_publish_period_sec", 1.0)
        self.declare_parameter("project_target_pose_to_ground", True)
        self.declare_parameter("optical_to_camera_link", True)
        self.declare_parameter("save_detection_images", True)
        self.declare_parameter("capture_dir", "src/yolo/dectected_person")
        self.declare_parameter("capture_period_sec", 2.0)
        self.declare_parameter("capture_annotated", True)
        self.declare_parameter("survivor_match_radius_m", 1.0)
        self.declare_parameter("candidate_match_radius_m", 0.8)
        self.declare_parameter("new_survivor_confirmations", 3)
        self.declare_parameter("candidate_ttl_sec", 10.0)
        self.declare_parameter("depth_roi_x_min_ratio", 0.30)
        self.declare_parameter("depth_roi_x_max_ratio", 0.70)
        self.declare_parameter("depth_roi_y_min_ratio", 0.25)
        self.declare_parameter("depth_roi_y_max_ratio", 0.75)
        self.declare_parameter("depth_percentile", 25.0)
        self.declare_parameter("depth_cluster_tolerance_m", 0.45)
        self.declare_parameter("min_depth_samples", 20)

        model_path = self.get_parameter("model_path").value
        image_topic = self.get_parameter("image_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        annotated_topic = self.get_parameter("annotated_topic").value
        multi_camera = bool(self.get_parameter("multi_camera").value)
        detected_topic = self.get_parameter("detected_topic").value
        slowdown_required_topic = self.get_parameter("slowdown_required_topic").value
        base_pose_topic = self.get_parameter("base_pose_topic").value
        survivor_pose_topic = self.get_parameter("survivor_pose_topic").value
        survivor_delete_topic = self.get_parameter("survivor_delete_topic").value

        self.conf_thresh = float(self.get_parameter("confidence_threshold").value)
        self.device = self.get_parameter("device").value
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.inference_period = float(
            self.get_parameter("inference_period_sec").value
        )
        self.annotated_period = float(
            self.get_parameter("annotated_publish_period_sec").value
        )
        self.person_only = bool(self.get_parameter("person_only").value)
        self.base_frame = self.get_parameter("base_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        self.use_latest_tf = bool(self.get_parameter("use_latest_tf").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self.depth_radius = int(self.get_parameter("depth_sample_radius_px").value)
        self.min_depth = float(self.get_parameter("min_depth_m").value)
        self.max_depth = float(self.get_parameter("max_depth_m").value)
        self.pose_period = float(self.get_parameter("pose_publish_period_sec").value)
        self.project_to_ground = bool(
            self.get_parameter("project_target_pose_to_ground").value
        )
        self.optical_to_camera_link = bool(
            self.get_parameter("optical_to_camera_link").value
        )
        self.save_detection_images = bool(
            self.get_parameter("save_detection_images").value
        )
        self.capture_dir = Path(str(self.get_parameter("capture_dir").value))
        self.capture_period = float(self.get_parameter("capture_period_sec").value)
        self.capture_annotated = bool(self.get_parameter("capture_annotated").value)
        self.survivor_match_radius = float(
            self.get_parameter("survivor_match_radius_m").value
        )
        self.candidate_match_radius = float(
            self.get_parameter("candidate_match_radius_m").value
        )
        self.new_survivor_confirmations = max(
            1, int(self.get_parameter("new_survivor_confirmations").value)
        )
        self.candidate_ttl = float(self.get_parameter("candidate_ttl_sec").value)
        self.depth_roi_x_min = float(
            self.get_parameter("depth_roi_x_min_ratio").value
        )
        self.depth_roi_x_max = float(
            self.get_parameter("depth_roi_x_max_ratio").value
        )
        self.depth_roi_y_min = float(
            self.get_parameter("depth_roi_y_min_ratio").value
        )
        self.depth_roi_y_max = float(
            self.get_parameter("depth_roi_y_max_ratio").value
        )
        self.depth_percentile = float(self.get_parameter("depth_percentile").value)
        self.depth_cluster_tolerance = float(
            self.get_parameter("depth_cluster_tolerance_m").value
        )
        self.min_depth_samples = max(
            1, int(self.get_parameter("min_depth_samples").value)
        )

        self.bridge = CvBridge()
        self.camera_streams = self._build_camera_streams(
            multi_camera,
            image_topic,
            depth_topic,
            camera_info_topic,
            annotated_topic,
        )
        old_time = self.get_clock().now() - Duration(seconds=9999.0)
        for stream in self.camera_streams:
            stream.last_inference_time = old_time
            stream.last_annotated_pub_time = old_time
            stream.last_pose_pub_time = old_time
        self._last_capture_time = self.get_clock().now() - Duration(seconds=9999.0)
        self._frame_count = 0
        self._capture_count = 0
        self._confirmed_survivors: list[ConfirmedSurvivor] = []
        self._pending_survivors: list[SurvivorCandidate] = []
        self._next_survivor_id = 1

        if self.save_detection_images:
            try:
                self.capture_dir.mkdir(parents=True, exist_ok=True)
            except Exception as exc:
                self.get_logger().warn(
                    f"Could not create capture directory {self.capture_dir}: {exc}"
                )
                self.save_detection_images = False

        self.get_logger().info(f"Loading YOLOv8 model: {model_path}")
        self.model = YOLO(model_path)
        try:
            self.model.to(self.device)
        except Exception as exc:
            self.get_logger().warn(
                f"Could not move model to {self.device}, falling back to CPU: {exc}"
            )
            self.device = "cpu"

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        annotated_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.camera_info_subs = []
        for stream in self.camera_streams:
            stream.image_sub = message_filters.Subscriber(
                self, Image, stream.image_topic, qos_profile=sensor_qos
            )
            stream.depth_sub = message_filters.Subscriber(
                self, Image, stream.depth_topic, qos_profile=sensor_qos
            )
            stream.sync = message_filters.ApproximateTimeSynchronizer(
                [stream.image_sub, stream.depth_sub],
                queue_size=int(self.get_parameter("sync_queue_size").value),
                slop=float(self.get_parameter("sync_slop_sec").value),
            )
            stream.sync.registerCallback(
                lambda image_msg, depth_msg, camera_stream=stream: self._on_rgbd(
                    camera_stream, image_msg, depth_msg
                )
            )
            self.camera_info_subs.append(
                self.create_subscription(
                    CameraInfo,
                    stream.camera_info_topic,
                    lambda msg, camera_stream=stream: self._on_camera_info(
                        camera_stream, msg
                    ),
                    reliable_qos,
                )
            )
            stream.pub_image = self.create_publisher(
                Image, stream.annotated_topic, annotated_qos
            )

        self.pub_detected = self.create_publisher(Bool, detected_topic, 10)
        self.pub_slowdown_required = self.create_publisher(
            Bool, slowdown_required_topic, 10
        )
        self.pub_base_pose = self.create_publisher(PoseStamped, base_pose_topic, 10)
        self.pub_survivor_pose = self.create_publisher(
            PoseStamped, survivor_pose_topic, 10
        )
        self.delete_sub = self.create_subscription(
            Int32, survivor_delete_topic, self._on_delete_survivor, reliable_qos
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        camera_summary = "; ".join(
            "%s(rgb=%s depth=%s info=%s annotated=%s)"
            % (
                stream.name,
                stream.image_topic,
                stream.depth_topic,
                stream.camera_info_topic,
                stream.annotated_topic,
            )
            for stream in self.camera_streams
        )
        self.get_logger().info(
            "subscribed cameras=%s; publishing detected=%s slowdown=%s "
            "base_pose=%s survivor_pose=%s target_frame=%s delete_topic=%s"
            % (
                camera_summary,
                detected_topic,
                slowdown_required_topic,
                base_pose_topic,
                survivor_pose_topic,
                self.target_frame,
                survivor_delete_topic,
            )
        )

    def _build_camera_streams(
        self,
        multi_camera: bool,
        image_topic: str,
        depth_topic: str,
        camera_info_topic: str,
        annotated_topic: str,
    ) -> list[CameraStream]:
        if not multi_camera:
            return [
                CameraStream(
                    name="front",
                    image_topic=image_topic,
                    depth_topic=depth_topic,
                    camera_info_topic=camera_info_topic,
                    annotated_topic=annotated_topic,
                )
            ]

        names = self._string_list_parameter("camera_names")
        image_topics = self._string_list_parameter("image_topics")
        depth_topics = self._string_list_parameter("depth_topics")
        camera_info_topics = self._string_list_parameter("camera_info_topics")
        annotated_topics = self._string_list_parameter("annotated_topics")

        counts = {
            "camera_names": len(names),
            "image_topics": len(image_topics),
            "depth_topics": len(depth_topics),
            "camera_info_topics": len(camera_info_topics),
            "annotated_topics": len(annotated_topics),
        }
        if not names or len(set(counts.values())) != 1:
            raise ValueError(f"multi_camera parameter lengths must match: {counts}")

        return [
            CameraStream(
                name=names[i],
                image_topic=image_topics[i],
                depth_topic=depth_topics[i],
                camera_info_topic=camera_info_topics[i],
                annotated_topic=annotated_topics[i],
            )
            for i in range(len(names))
        ]

    def _string_list_parameter(self, name: str) -> list[str]:
        value = self.get_parameter(name).value
        if isinstance(value, str):
            return [part.strip() for part in value.split(",") if part.strip()]
        return [str(part) for part in value]

    def _on_camera_info(self, stream: CameraStream, msg: CameraInfo):
        stream.camera_info = msg

    def _on_rgbd(self, stream: CameraStream, image_msg: Image, depth_msg: Image):
        if not self._period_due(stream.last_inference_time, self.inference_period):
            return
        stream.last_inference_time = self.get_clock().now()

        try:
            frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().error(f"cv_bridge failed: {exc}")
            return

        result = self._predict(frame)
        self._publish_annotated(stream, result, image_msg.header)

        boxes = [] if result.boxes is None else list(result.boxes)
        det_msg = Bool()
        det_msg.data = len(boxes) > 0
        self.pub_detected.publish(det_msg)

        self._frame_count += 1
        if not boxes:
            self._publish_slowdown_required(False)
            if self._frame_count % 30 == 0:
                self.get_logger().info(
                    f"{stream.name} frame {self._frame_count}: no survivor"
                )
            return

        if stream.camera_info is None:
            self._publish_slowdown_required(True)
            self.get_logger().warn(
                f"No CameraInfo yet for {stream.name}; cannot localize survivor"
            )
            return

        if not self._pose_publish_due(stream):
            return

        candidates = self._localizable_boxes(boxes, depth, depth_msg.encoding)
        if not candidates:
            self._publish_slowdown_required(True)
            self.get_logger().warn("Detected person, but no valid person-depth sample")
            return

        stream.last_pose_pub_time = self.get_clock().now()
        frame_targets: list[PoseStamped] = []
        localized_any = False
        slowdown_required = False

        for box, u, v, depth_m in candidates:
            camera_point = self._pixel_depth_to_camera_point(
                stream.camera_info,
                u,
                v,
                depth_m,
                image_msg.header.frame_id,
                image_msg.header.stamp,
            )
            base_pose = self._transform_point_to_pose(camera_point, self.base_frame)
            if base_pose is None:
                continue
            self.pub_base_pose.publish(base_pose)

            target_pose = self._transform_pose(base_pose, self.target_frame)
            if target_pose is None:
                continue
            localized_any = True
            if self.project_to_ground:
                target_pose.pose.position.z = 0.0

            same_frame_index, _ = self._nearest_pose_index(
                target_pose, frame_targets, self.candidate_match_radius
            )
            if same_frame_index is not None:
                continue
            frame_targets.append(self._copy_pose(target_pose))

            known_index, _ = self._nearest_confirmed_survivor_index(
                target_pose, self.survivor_match_radius
            )
            if known_index is not None:
                continue

            slowdown_required = True
            survivor_id = self._accept_new_survivor(target_pose)
            if survivor_id is None:
                continue

            self.pub_survivor_pose.publish(target_pose)
            self._save_detection_capture(
                stream, frame, result, survivor_id=survivor_id, force=True
            )

            conf = float(box.conf[0])
            bp = base_pose.pose.position
            tp = target_pose.pose.position
            self.get_logger().info(
                "new survivor #%d camera=%s conf=%.2f depth=%.2fm pixel=(%d,%d) "
                "base=(%.2f, %.2f, %.2f) %s=(%.2f, %.2f, %.2f)"
                % (
                    survivor_id,
                    stream.name,
                    conf,
                    depth_m,
                    u,
                    v,
                    bp.x,
                    bp.y,
                    bp.z,
                    self.target_frame,
                    tp.x,
                    tp.y,
                    tp.z,
                )
            )

        self._publish_slowdown_required(slowdown_required or not localized_any)

    def _predict(self, frame):
        classes = [PERSON_CLASS_ID] if self.person_only else None
        return self.model.predict(
            frame,
            conf=self.conf_thresh,
            classes=classes,
            device=self.device,
            imgsz=self.imgsz,
            verbose=False,
        )[0]

    def _publish_slowdown_required(self, required: bool):
        msg = Bool()
        msg.data = bool(required)
        self.pub_slowdown_required.publish(msg)

    def _publish_annotated(self, stream: CameraStream, result, header):
        if stream.pub_image.get_subscription_count() == 0:
            return
        if not self._period_due(stream.last_annotated_pub_time, self.annotated_period):
            return
        annotated = result.plot()
        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out_msg.header = header
        stream.pub_image.publish(out_msg)
        stream.last_annotated_pub_time = self.get_clock().now()

    def _save_detection_capture(
        self,
        stream: CameraStream,
        frame,
        result,
        survivor_id: int | None = None,
        force=False,
    ):
        if not self.save_detection_images:
            return
        if not force and not self._period_due(self._last_capture_time, self.capture_period):
            return

        image = result.plot() if self.capture_annotated else frame
        now = self.get_clock().now()
        self._capture_count += 1
        prefix = (
            f"survivor_{survivor_id:03d}_{stream.name}"
            if survivor_id is not None
            else f"person_{stream.name}"
        )
        capture_path = self.capture_dir / (
            f"{prefix}_{now.nanoseconds}_{self._capture_count:06d}.jpg"
        )

        try:
            if not cv2.imwrite(str(capture_path), image):
                self.get_logger().warn(f"Failed to save detection image: {capture_path}")
                return
        except Exception as exc:
            self.get_logger().warn(f"Failed to save detection image {capture_path}: {exc}")
            return

        self._last_capture_time = now
        self.get_logger().info(f"saved detection image: {capture_path}")

    def _accept_new_survivor(self, pose: PoseStamped) -> Optional[int]:
        self._prune_pending_survivors()

        known_index, known_dist = self._nearest_confirmed_survivor_index(
            pose, self.survivor_match_radius
        )
        if known_index is not None:
            survivor_id = self._confirmed_survivors[known_index].survivor_id
            self.get_logger().info(
                "duplicate survivor ignored: existing #%d distance=%.2fm"
                % (survivor_id, known_dist)
            )
            return None

        now_ns = self.get_clock().now().nanoseconds
        pending_index, pending_dist = self._nearest_candidate_index(
            pose, self._pending_survivors, self.candidate_match_radius
        )
        if pending_index is None:
            candidate = SurvivorCandidate(
                pose=self._copy_pose(pose),
                observations=1,
                last_seen_ns=now_ns,
                samples=[self._pose_xyz(pose)],
            )
            self._pending_survivors.append(candidate)
            if self.new_survivor_confirmations > 1:
                self.get_logger().info(
                    "new survivor candidate 1/%d at %s=(%.2f, %.2f, %.2f)"
                    % (
                        self.new_survivor_confirmations,
                        pose.header.frame_id,
                        pose.pose.position.x,
                        pose.pose.position.y,
                        pose.pose.position.z,
                    )
                )
                return None
        else:
            candidate = self._pending_survivors[pending_index]
            self._merge_candidate(candidate, pose, now_ns)
            if candidate.observations < self.new_survivor_confirmations:
                self.get_logger().info(
                    "new survivor candidate %d/%d distance=%.2fm"
                    % (
                        candidate.observations,
                        self.new_survivor_confirmations,
                        pending_dist,
                    )
                )
                return None

        candidate_pose = candidate.pose
        survivor_id = self._next_survivor_id
        self._next_survivor_id += 1
        self._confirmed_survivors.append(
            ConfirmedSurvivor(
                survivor_id=survivor_id, pose=self._copy_pose(candidate_pose)
            )
        )
        if pending_index is None:
            self._pending_survivors = [
                c for c in self._pending_survivors if c is not candidate
            ]
        else:
            self._pending_survivors.pop(pending_index)

        pose.header = candidate_pose.header
        pose.pose = candidate_pose.pose
        self.get_logger().info(
            "confirmed survivor #%d; total=%d"
            % (survivor_id, len(self._confirmed_survivors))
        )
        return survivor_id

    def _on_delete_survivor(self, msg: Int32):
        survivor_id = int(msg.data)
        if survivor_id == 0:
            confirmed_count = len(self._confirmed_survivors)
            pending_count = len(self._pending_survivors)
            self._confirmed_survivors.clear()
            self._pending_survivors.clear()
            self._next_survivor_id = 1
            self.get_logger().info(
                "deleted all survivors from detector memory "
                f"(confirmed={confirmed_count}, pending={pending_count})"
            )
            return

        if survivor_id < 0:
            self.get_logger().warn(
                f"ignoring invalid survivor delete id {survivor_id}; use 0 for all"
            )
            return

        for index, survivor in enumerate(self._confirmed_survivors):
            if survivor.survivor_id != survivor_id:
                continue
            removed = self._confirmed_survivors.pop(index)
            p = removed.pose.pose.position
            self.get_logger().info(
                "deleted survivor #%d from detector memory at %s=(%.2f, %.2f, %.2f); "
                "remaining=%d"
                % (
                    survivor_id,
                    removed.pose.header.frame_id,
                    p.x,
                    p.y,
                    p.z,
                    len(self._confirmed_survivors),
                )
            )
            return

        self.get_logger().warn(
            f"delete requested for unknown survivor #{survivor_id}; no detector entry removed"
        )

    def _prune_pending_survivors(self):
        if self.candidate_ttl <= 0.0:
            return
        now_ns = self.get_clock().now().nanoseconds
        ttl_ns = int(self.candidate_ttl * 1e9)
        self._pending_survivors = [
            c for c in self._pending_survivors if now_ns - c.last_seen_ns <= ttl_ns
        ]

    def _nearest_pose_index(self, pose, poses, max_dist: float):
        nearest_index = None
        nearest_dist = math.inf
        for index, candidate in enumerate(poses):
            dist = self._pose_xy_distance(pose, candidate)
            if dist <= max_dist and dist < nearest_dist:
                nearest_index = index
                nearest_dist = dist
        return nearest_index, nearest_dist

    def _nearest_confirmed_survivor_index(self, pose, max_dist: float):
        nearest_index = None
        nearest_dist = math.inf
        for index, survivor in enumerate(self._confirmed_survivors):
            dist = self._pose_xy_distance(pose, survivor.pose)
            if dist <= max_dist and dist < nearest_dist:
                nearest_index = index
                nearest_dist = dist
        return nearest_index, nearest_dist

    def _nearest_candidate_index(self, pose, candidates, max_dist: float):
        nearest_index = None
        nearest_dist = math.inf
        for index, candidate in enumerate(candidates):
            dist = self._pose_xy_distance(pose, candidate.pose)
            if dist <= max_dist and dist < nearest_dist:
                nearest_index = index
                nearest_dist = dist
        return nearest_index, nearest_dist

    def _merge_candidate(self, candidate: SurvivorCandidate, pose: PoseStamped, now_ns: int):
        candidate.samples.append(self._pose_xyz(pose))
        xs, ys, zs = zip(*candidate.samples)
        old = candidate.pose.pose.position
        old.x = float(np.median(xs))
        old.y = float(np.median(ys))
        old.z = float(np.median(zs))
        candidate.pose.header = pose.header
        candidate.pose.pose.orientation = pose.pose.orientation
        candidate.observations += 1
        candidate.last_seen_ns = now_ns

    @staticmethod
    def _pose_xy_distance(a: PoseStamped, b: PoseStamped) -> float:
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        return math.hypot(dx, dy)

    @staticmethod
    def _copy_pose(pose: PoseStamped) -> PoseStamped:
        out = PoseStamped()
        out.header = pose.header
        out.pose.position.x = pose.pose.position.x
        out.pose.position.y = pose.pose.position.y
        out.pose.position.z = pose.pose.position.z
        out.pose.orientation = pose.pose.orientation
        return out

    @staticmethod
    def _pose_xyz(pose: PoseStamped) -> tuple[float, float, float]:
        p = pose.pose.position
        return p.x, p.y, p.z

    def _period_due(self, last_time, period_sec: float) -> bool:
        if period_sec <= 0.0:
            return True
        elapsed = self.get_clock().now() - last_time
        return elapsed.nanoseconds >= int(period_sec * 1e9)

    def _pose_publish_due(self, stream: CameraStream) -> bool:
        return self._period_due(stream.last_pose_pub_time, self.pose_period)

    def _localizable_boxes(self, boxes, depth, encoding):
        localized = []
        sorted_boxes = sorted(boxes, key=lambda b: float(b.conf[0]), reverse=True)
        for box in sorted_boxes:
            sample = self._sample_box_depth(depth, box, encoding)
            if sample is not None:
                u, v, depth_m = sample
                localized.append((box, u, v, depth_m))
        return localized

    def _sample_box_depth(self, depth, box, encoding: str):
        arr = np.asarray(depth)
        if arr.ndim != 2 or arr.size == 0:
            return None

        h, w = arr.shape
        x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
        x1 = max(0.0, min(float(w - 1), x1))
        x2 = max(0.0, min(float(w - 1), x2))
        y1 = max(0.0, min(float(h - 1), y1))
        y2 = max(0.0, min(float(h - 1), y2))
        if x2 <= x1 or y2 <= y1:
            return None

        roi = self._depth_roi_from_box(x1, y1, x2, y2, w, h)
        if roi is None:
            return None
        us, ue, vs, ve = roi
        patch = arr[vs:ve, us:ue].astype(np.float32, copy=False)

        if encoding.upper() == "16UC1":
            patch = patch / 1000.0

        valid_mask = (
            np.isfinite(patch)
            & (patch >= self.min_depth)
            & (patch <= self.max_depth)
        )
        valid = patch[valid_mask]
        if valid.size < self.min_depth_samples:
            return None

        percentile = min(100.0, max(0.0, self.depth_percentile))
        seed_depth = float(np.percentile(valid, percentile))
        cluster_mask = valid_mask & (
            np.abs(patch - seed_depth) <= self.depth_cluster_tolerance
        )
        cluster = patch[cluster_mask]
        if cluster.size < self.min_depth_samples:
            near_mask = valid_mask & (patch <= seed_depth + self.depth_cluster_tolerance)
            cluster = patch[near_mask]
            cluster_mask = near_mask

        if cluster.size < self.min_depth_samples:
            return None

        ys, xs = np.nonzero(cluster_mask)
        if xs.size == 0 or ys.size == 0:
            return None

        u = int(us + np.median(xs))
        v = int(vs + np.median(ys))
        depth_m = float(np.median(cluster))
        return u, v, depth_m

    def _depth_roi_from_box(self, x1: float, y1: float, x2: float, y2: float, w: int, h: int):
        rx1 = min(max(self.depth_roi_x_min, 0.0), 1.0)
        rx2 = min(max(self.depth_roi_x_max, 0.0), 1.0)
        ry1 = min(max(self.depth_roi_y_min, 0.0), 1.0)
        ry2 = min(max(self.depth_roi_y_max, 0.0), 1.0)
        if rx2 <= rx1:
            rx1, rx2 = 0.30, 0.70
        if ry2 <= ry1:
            ry1, ry2 = 0.25, 0.75

        bw = x2 - x1
        bh = y2 - y1
        us = int(max(0, min(w - 1, math.floor(x1 + bw * rx1))))
        ue = int(max(0, min(w, math.ceil(x1 + bw * rx2))))
        vs = int(max(0, min(h - 1, math.floor(y1 + bh * ry1))))
        ve = int(max(0, min(h, math.ceil(y1 + bh * ry2))))
        if ue <= us or ve <= vs:
            return None
        return us, ue, vs, ve

    def _pixel_depth_to_camera_point(
        self,
        camera_info: CameraInfo,
        u: int,
        v: int,
        depth_m: float,
        frame_id: str,
        stamp,
    ) -> PointStamped:
        k = camera_info.k
        fx, fy = float(k[0]), float(k[4])
        cx, cy = float(k[2]), float(k[5])

        x_opt = (float(u) - cx) * depth_m / fx
        y_opt = (float(v) - cy) * depth_m / fy
        z_opt = depth_m

        p = PointStamped()
        p.header.frame_id = frame_id or camera_info.header.frame_id
        p.header.stamp = stamp

        if self.optical_to_camera_link:
            p.point.x = z_opt
            p.point.y = -x_opt
            p.point.z = -y_opt
        else:
            p.point.x = x_opt
            p.point.y = y_opt
            p.point.z = z_opt
        return p

    def _transform_point_to_pose(
        self, point: PointStamped, target_frame: str
    ) -> Optional[PoseStamped]:
        transformed = self._transform_point(point, target_frame)
        if transformed is None:
            return None
        pose = PoseStamped()
        pose.header = transformed.header
        pose.pose.position = transformed.point
        pose.pose.orientation.w = 1.0
        return pose

    def _transform_pose(
        self, pose: PoseStamped, target_frame: str
    ) -> Optional[PoseStamped]:
        if pose.header.frame_id == target_frame:
            return pose
        stamp = Time() if self.use_latest_tf else Time.from_msg(pose.header.stamp)
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                pose.header.frame_id,
                stamp,
                timeout=Duration(seconds=self.tf_timeout),
            )
            out = tf2_geometry_msgs.do_transform_pose_stamped(pose, transform)
            out.header.stamp = self.get_clock().now().to_msg()
            return out
        except Exception as exc:
            self.get_logger().warn(
                f"TF pose transform failed {pose.header.frame_id}->{target_frame}: {exc}"
            )
            return None

    def _transform_point(
        self, point: PointStamped, target_frame: str
    ) -> Optional[PointStamped]:
        if point.header.frame_id == target_frame:
            return point
        stamp = Time() if self.use_latest_tf else Time.from_msg(point.header.stamp)
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                point.header.frame_id,
                stamp,
                timeout=Duration(seconds=self.tf_timeout),
            )
            out = tf2_geometry_msgs.do_transform_point(point, transform)
            out.header.stamp = self.get_clock().now().to_msg()
            return out
        except Exception as exc:
            self.get_logger().warn(
                f"TF point transform failed {point.header.frame_id}->{target_frame}: {exc}"
            )
            return None


def main(args=None):
    rclpy.init(args=args)
    node = SurvivorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
