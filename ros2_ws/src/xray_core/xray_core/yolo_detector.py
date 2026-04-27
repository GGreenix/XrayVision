"""YOLO detector node — platform-agnostic.

Subscribes:  /xray/camera/image/compressed  (sensor_msgs/CompressedImage)
Publishes:   /xray/perception/detections_2d (xray_interfaces/Detection2DArray)

The output topic and message shape are identical to the fake_detector, so
everything downstream (static_camera_localizer, moving_camera_localizer,
world_model) is oblivious to which detector is running.

Detection2D coordinates are normalized to [0, 1] (x, y are bbox center;
width, height are bbox size) — same convention used by fake_detector.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from xray_interfaces.msg import Detection2D, Detection2DArray


class YoloDetector(Node):
    def __init__(self) -> None:
        super().__init__("yolo_detector")

        self.declare_parameter("input_topic", "/xray/camera/image/compressed")
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("iou_threshold", 0.5)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "")  # "" = auto, "cpu", "cuda:0", etc.
        self.declare_parameter("source_frame", "camera_link")
        self.declare_parameter(
            "allowed_classes", [""]  # empty string sentinel means "all"
        )

        model_path = str(self.get_parameter("model_path").value)
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        device = str(self.get_parameter("device").value) or None
        self.source_frame = str(self.get_parameter("source_frame").value)
        allowed = [str(c) for c in self.get_parameter("allowed_classes").value]
        self.allowed_classes = {c for c in allowed if c} or None

        # Imports deferred so unit tests / dry-runs don't require ultralytics.
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "ultralytics is not installed. Run: pip install ultralytics"
            ) from exc

        self.get_logger().info(f"Loading YOLO model from {model_path}")
        self.model = YOLO(model_path)
        if device:
            self.model.to(device)

        self.publisher = self.create_publisher(
            Detection2DArray, "/xray/perception/detections_2d", 10
        )
        input_topic = str(self.get_parameter("input_topic").value)
        self.create_subscription(CompressedImage, input_topic, self.on_image, 10)
        self.get_logger().info(f"YOLO subscribing to {input_topic}")

    def on_image(self, message: CompressedImage) -> None:
        try:
            import cv2
        except ImportError:
            self.get_logger().error("opencv-python is required")
            return

        buffer = np.frombuffer(message.data, dtype=np.uint8)
        frame = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn("Failed to decode JPEG frame")
            return

        height, width = frame.shape[:2]

        results = self.model.predict(
            frame,
            conf=self.confidence_threshold,
            iou=self.iou_threshold,
            imgsz=self.imgsz,
            verbose=False,
        )

        out = Detection2DArray()
        out.header.stamp = message.header.stamp
        out.header.frame_id = message.header.frame_id or self.source_frame

        if results:
            result = results[0]
            names = result.names  # {class_index: class_name}
            boxes = result.boxes
            if boxes is not None:
                xyxy = boxes.xyxy.cpu().numpy() if hasattr(boxes.xyxy, "cpu") else np.asarray(boxes.xyxy)
                confs = boxes.conf.cpu().numpy() if hasattr(boxes.conf, "cpu") else np.asarray(boxes.conf)
                clss = boxes.cls.cpu().numpy() if hasattr(boxes.cls, "cpu") else np.asarray(boxes.cls)

                for (x1, y1, x2, y2), conf, cls_idx in zip(xyxy, confs, clss):
                    class_name = names.get(int(cls_idx), str(int(cls_idx)))
                    if self.allowed_classes is not None and class_name not in self.allowed_classes:
                        continue

                    det = Detection2D()
                    det.observed_at = message.header.stamp
                    det.class_id = class_name
                    det.confidence = float(conf)
                    det.center_x = float(((x1 + x2) / 2.0) / width)
                    det.center_y = float(((y1 + y2) / 2.0) / height)
                    det.width = float((x2 - x1) / width)
                    det.height = float((y2 - y1) / height)
                    det.range_estimate_m = 0.0  # not used by the ground-plane localizers
                    det.source_frame = out.header.frame_id
                    out.detections.append(det)

        self.publisher.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
