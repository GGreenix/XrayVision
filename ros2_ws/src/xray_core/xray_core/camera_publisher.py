import io

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class CameraPublisher(Node):
    """Publishes JPEG frames from the Pi camera on /xray/camera/image/compressed.

    Uses picamera2 (Raspberry Pi's official libcamera-based Python library). Encodes
    JPEG in-camera to keep CPU usage low on the Pi and WiFi bandwidth manageable.
    """

    def __init__(self) -> None:
        super().__init__("camera_publisher")
        self.declare_parameter("topic", "/xray/camera/image/compressed")
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("jpeg_quality", 80)

        self.topic = str(self.get_parameter("topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        self.publisher = self.create_publisher(CompressedImage, self.topic, 10)
        self.camera = self._configure_camera()
        self.create_timer(1.0 / max(self.rate_hz, 1.0), self._publish_frame)
        self.get_logger().info(
            f"camera_publisher: {self.width}x{self.height}@{self.rate_hz:.0f}Hz, "
            f"JPEG q={self.jpeg_quality}, topic={self.topic}"
        )

    def _configure_camera(self):
        try:
            from picamera2 import Picamera2
        except ImportError as error:
            raise RuntimeError(
                "picamera2 is not installed. On the Pi: sudo apt install -y python3-picamera2"
            ) from error

        camera = Picamera2()
        config = camera.create_video_configuration(
            main={"size": (self.width, self.height), "format": "RGB888"}
        )
        camera.configure(config)
        camera.start()
        return camera

    def _publish_frame(self) -> None:
        stream = io.BytesIO()
        self.camera.capture_file(stream, format="jpeg")
        data = stream.getvalue()

        message = CompressedImage()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.format = "jpeg"
        message.data = data
        self.publisher.publish(message)

    def destroy_node(self) -> bool:
        try:
            self.camera.stop()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
