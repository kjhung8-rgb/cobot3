"""YOLOv8 survivor detector and RGB-D localizer for the Spot front camera."""

from __future__ import annotations

import math
from typing import Optional

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
from std_msgs.msg import Bool
from ultralytics import YOLO


PERSON_CLASS_ID = 0  # COCO 'person'


class SurvivorDetector(Node):
    def __init__(self):
        super().__init__("yolo_detector")

        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("image_topic", "/spot_0/front_cam/color_image")
        self.declare_parameter("depth_topic", "/spot_0/front_cam/depth_image")
        self.declare_parameter("camera_info_topic", "/spot_0/front_cam/camera_info")
        self.declare_parameter("annotated_topic", "/spot_0/yolo/annotated_image")
        self.declare_parameter("detected_topic", "/spot_0/yolo/person_detected")
        self.declare_parameter("base_pose_topic", "/spot_0/yolo/person_pose_base")
        self.declare_parameter("survivor_pose_topic", "/detected_survivor_pose")
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

        model_path = self.get_parameter("model_path").value
        image_topic = self.get_parameter("image_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        annotated_topic = self.get_parameter("annotated_topic").value
        detected_topic = self.get_parameter("detected_topic").value
        base_pose_topic = self.get_parameter("base_pose_topic").value
        survivor_pose_topic = self.get_parameter("survivor_pose_topic").value

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

        self.bridge = CvBridge()
        self.camera_info: Optional[CameraInfo] = None
        self._last_inference_time = self.get_clock().now() - Duration(seconds=9999.0)
        self._last_annotated_pub_time = self.get_clock().now() - Duration(
            seconds=9999.0
        )
        self._last_pose_pub_time = self.get_clock().now() - Duration(seconds=9999.0)
        self._frame_count = 0

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

        self.image_sub = message_filters.Subscriber(
            self, Image, image_topic, qos_profile=sensor_qos
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, depth_topic, qos_profile=sensor_qos
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub],
            queue_size=int(self.get_parameter("sync_queue_size").value),
            slop=float(self.get_parameter("sync_slop_sec").value),
        )
        self.sync.registerCallback(self._on_rgbd)

        self.create_subscription(
            CameraInfo, camera_info_topic, self._on_camera_info, reliable_qos
        )
        self.pub_image = self.create_publisher(Image, annotated_topic, annotated_qos)
        self.pub_detected = self.create_publisher(Bool, detected_topic, 10)
        self.pub_base_pose = self.create_publisher(PoseStamped, base_pose_topic, 10)
        self.pub_survivor_pose = self.create_publisher(
            PoseStamped, survivor_pose_topic, 10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            "subscribed rgb=%s depth=%s camera_info=%s; publishing annotated=%s "
            "detected=%s base_pose=%s survivor_pose=%s target_frame=%s"
            % (
                image_topic,
                depth_topic,
                camera_info_topic,
                annotated_topic,
                detected_topic,
                base_pose_topic,
                survivor_pose_topic,
                self.target_frame,
            )
        )

    def _on_camera_info(self, msg: CameraInfo):
        self.camera_info = msg

    def _on_rgbd(self, image_msg: Image, depth_msg: Image):
        if not self._period_due(self._last_inference_time, self.inference_period):
            return
        self._last_inference_time = self.get_clock().now()

        try:
            frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().error(f"cv_bridge failed: {exc}")
            return

        result = self._predict(frame)
        self._publish_annotated(result, image_msg.header)

        boxes = [] if result.boxes is None else list(result.boxes)
        det_msg = Bool()
        det_msg.data = len(boxes) > 0
        self.pub_detected.publish(det_msg)

        self._frame_count += 1
        if not boxes:
            if self._frame_count % 30 == 0:
                self.get_logger().info(f"frame {self._frame_count}: no survivor")
            return

        if self.camera_info is None:
            self.get_logger().warn("No CameraInfo yet; cannot localize survivor")
            return

        if not self._pose_publish_due():
            return

        candidate = self._best_localizable_box(boxes, depth, depth_msg.encoding)
        if candidate is None:
            self.get_logger().warn("Detected person, but no valid depth sample")
            return

        box, u, v, depth_m = candidate
        camera_point = self._pixel_depth_to_camera_point(
            u, v, depth_m, image_msg.header.frame_id, image_msg.header.stamp
        )
        base_pose = self._transform_point_to_pose(camera_point, self.base_frame)
        if base_pose is None:
            return
        self.pub_base_pose.publish(base_pose)

        target_pose = self._transform_pose(base_pose, self.target_frame)
        if target_pose is None:
            return
        if self.project_to_ground:
            target_pose.pose.position.z = 0.0
        self.pub_survivor_pose.publish(target_pose)
        self._last_pose_pub_time = self.get_clock().now()

        conf = float(box.conf[0])
        bp = base_pose.pose.position
        tp = target_pose.pose.position
        self.get_logger().info(
            "survivor conf=%.2f depth=%.2fm pixel=(%d,%d) "
            "base=(%.2f, %.2f, %.2f) %s=(%.2f, %.2f, %.2f)"
            % (
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

    def _publish_annotated(self, result, header):
        if self.pub_image.get_subscription_count() == 0:
            return
        if not self._period_due(self._last_annotated_pub_time, self.annotated_period):
            return
        annotated = result.plot()
        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out_msg.header = header
        self.pub_image.publish(out_msg)
        self._last_annotated_pub_time = self.get_clock().now()

    def _period_due(self, last_time, period_sec: float) -> bool:
        if period_sec <= 0.0:
            return True
        elapsed = self.get_clock().now() - last_time
        return elapsed.nanoseconds >= int(period_sec * 1e9)

    def _pose_publish_due(self) -> bool:
        return self._period_due(self._last_pose_pub_time, self.pose_period)

    def _best_localizable_box(self, boxes, depth, encoding):
        sorted_boxes = sorted(boxes, key=lambda b: float(b.conf[0]), reverse=True)
        for box in sorted_boxes:
            x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
            u = int((x1 + x2) * 0.5)
            v = int((y1 + y2) * 0.5)
            depth_m = self._sample_depth(depth, u, v, encoding)
            if depth_m is not None:
                return box, u, v, depth_m
        return None

    def _sample_depth(self, depth, u: int, v: int, encoding: str) -> Optional[float]:
        arr = np.asarray(depth)
        if arr.ndim != 2 or arr.size == 0:
            return None

        h, w = arr.shape
        if u < 0 or v < 0 or u >= w or v >= h:
            return None

        r = max(0, self.depth_radius)
        us, ue = max(0, u - r), min(w, u + r + 1)
        vs, ve = max(0, v - r), min(h, v + r + 1)
        patch = arr[vs:ve, us:ue].astype(np.float32, copy=False)

        if encoding.upper() == "16UC1":
            patch = patch / 1000.0

        valid = patch[np.isfinite(patch)]
        valid = valid[(valid >= self.min_depth) & (valid <= self.max_depth)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _pixel_depth_to_camera_point(
        self, u: int, v: int, depth_m: float, frame_id: str, stamp
    ) -> PointStamped:
        k = self.camera_info.k
        fx, fy = float(k[0]), float(k[4])
        cx, cy = float(k[2]), float(k[5])

        x_opt = (float(u) - cx) * depth_m / fx
        y_opt = (float(v) - cy) * depth_m / fy
        z_opt = depth_m

        p = PointStamped()
        p.header.frame_id = frame_id or self.camera_info.header.frame_id
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
