"""YOLOv8 survivor detector node for Spot front camera."""

from __future__ import annotations

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

from ultralytics import YOLO


PERSON_CLASS_ID = 0  # COCO 'person'


class SurvivorDetector(Node):
    def __init__(self):
        super().__init__('survivor_detector')

        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('image_topic', '/spot_0/front_cam/color_image')
        self.declare_parameter('annotated_topic', '/spot_0/yolo/annotated_image')
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('person_only', True)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        annotated_topic = self.get_parameter('annotated_topic').get_parameter_value().string_value
        self.conf_thresh = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.person_only = self.get_parameter('person_only').get_parameter_value().bool_value

        self.bridge = CvBridge()
        self.get_logger().info(f'Loading YOLOv8 model: {model_path}')
        self.model = YOLO(model_path)
        try:
            self.model.to(self.device)
        except Exception as exc:
            self.get_logger().warn(f'Could not move model to {self.device}, falling back to CPU: {exc}')
            self.device = 'cpu'

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub = self.create_subscription(Image, image_topic, self._on_image, sensor_qos)
        self.pub_image = self.create_publisher(Image, annotated_topic, 10)

        self._frame_count = 0
        self.get_logger().info(
            f'subscribed image={image_topic} device={self.device} '
            f'conf>={self.conf_thresh} person_only={self.person_only}'
        )

    def _on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge failed: {exc}')
            return

        classes = [PERSON_CLASS_ID] if self.person_only else None
        results = self.model.predict(
            frame,
            conf=self.conf_thresh,
            classes=classes,
            device=self.device,
            verbose=False,
        )
        result = results[0]

        annotated = result.plot()
        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out_msg.header = msg.header
        self.pub_image.publish(out_msg)

        self._frame_count += 1
        if self._frame_count % 30 == 0:
            n = 0 if result.boxes is None else len(result.boxes)
            self.get_logger().info(f'frame {self._frame_count}: {n} survivor(s)')


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


if __name__ == '__main__':
    main()
