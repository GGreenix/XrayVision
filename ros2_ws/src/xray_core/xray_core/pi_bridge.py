"""Bridge a Pi station (HTTP/WebSocket) to ROS topics.

The Pi runs a non-ROS Python process that exposes:
    GET  /video.mjpg   -> rectified left feed as MJPEG
    WS   /stream       -> JSON detections with 3D world positions
    GET  /pose         -> the configured static camera pose

This node connects to those endpoints and republishes:
    /xray/camera/image/compressed   sensor_msgs/CompressedImage
    /xray/perception/objects_raw    xray_interfaces/TrackedObjectArray
    /xray/uav/odom                  nav_msgs/Odometry  (the static Pi pose)

Everything downstream (world_model, Unity bridge) sees the same topics it
saw when the Pi ran ROS itself.
"""

import json
import math
import threading
import time
import urllib.request
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from xray_core.math_utils import quaternion_from_euler
from xray_interfaces.msg import TrackedObject, TrackedObjectArray


class PiBridge(Node):
    def __init__(self) -> None:
        super().__init__("pi_bridge")
        self.declare_parameter("pi_host", "raspberrypi.local")
        self.declare_parameter("pi_port", 8765)
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_left_optical")
        self.declare_parameter("reconnect_delay_s", 2.0)
        self.declare_parameter("pose_publish_rate_hz", 2.0)

        self.pi_host = str(self.get_parameter("pi_host").value)
        self.pi_port = int(self.get_parameter("pi_port").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.reconnect_delay = float(self.get_parameter("reconnect_delay_s").value)
        pose_rate = float(self.get_parameter("pose_publish_rate_hz").value)

        self.image_pub = self.create_publisher(CompressedImage, "/xray/camera/image/compressed", 10)
        self.objects_pub = self.create_publisher(TrackedObjectArray, "/xray/perception/objects_raw", 10)
        self.odom_pub = self.create_publisher(Odometry, "/xray/uav/odom", 10)

        # Static pose fetched from the Pi at startup; republished at a steady rate.
        self.static_pose: Optional[dict] = None
        self.create_timer(1.0 / pose_rate, self._publish_pose)

        self._stop = threading.Event()
        self._video_thread = threading.Thread(target=self._video_loop, daemon=True, name="pi_video")
        self._stream_thread = threading.Thread(target=self._stream_loop, daemon=True, name="pi_stream")
        self._pose_thread = threading.Thread(target=self._pose_fetch_loop, daemon=True, name="pi_pose")
        self._video_thread.start()
        self._stream_thread.start()
        self._pose_thread.start()

    def destroy_node(self) -> None:
        self._stop.set()
        super().destroy_node()

    @property
    def base_url(self) -> str:
        return f"http://{self.pi_host}:{self.pi_port}"

    # ------- pose -------

    def _pose_fetch_loop(self) -> None:
        while not self._stop.is_set() and self.static_pose is None:
            try:
                with urllib.request.urlopen(f"{self.base_url}/pose", timeout=3.0) as resp:
                    self.static_pose = json.loads(resp.read().decode("utf-8"))
                    self.get_logger().info(f"Got Pi pose: {self.static_pose}")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"Pose fetch failed ({exc}); retrying...")
                time.sleep(self.reconnect_delay)

    def _publish_pose(self) -> None:
        if self.static_pose is None:
            return
        position = self.static_pose["position_m"]
        rpy = self.static_pose["rpy_deg"]
        qx, qy, qz, qw = quaternion_from_euler(
            math.radians(rpy[0]), math.radians(rpy[1]), math.radians(rpy[2])
        )
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = float(position[0])
        msg.pose.pose.position.y = float(position[1])
        msg.pose.pose.position.z = float(position[2])
        msg.pose.pose.orientation.x = float(qx)
        msg.pose.pose.orientation.y = float(qy)
        msg.pose.pose.orientation.z = float(qz)
        msg.pose.pose.orientation.w = float(qw)
        self.odom_pub.publish(msg)

    # ------- video (MJPEG) -------

    def _video_loop(self) -> None:
        url = f"{self.base_url}/video.mjpg"
        while not self._stop.is_set():
            try:
                self._consume_mjpeg(url)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"MJPEG stream error ({exc}); reconnecting in {self.reconnect_delay}s")
                time.sleep(self.reconnect_delay)

    def _consume_mjpeg(self, url: str) -> None:
        """Parse multipart/x-mixed-replace and emit one CompressedImage per frame."""
        with urllib.request.urlopen(url, timeout=10.0) as resp:
            content_type = resp.headers.get("Content-Type", "")
            boundary = self._extract_boundary(content_type)
            if boundary is None:
                raise RuntimeError(f"No boundary in Content-Type: {content_type}")
            sep = b"--" + boundary.encode("ascii")

            buffer = b""
            while not self._stop.is_set():
                chunk = resp.read(65536)
                if not chunk:
                    return
                buffer += chunk
                while True:
                    boundary_idx = buffer.find(sep)
                    if boundary_idx < 0:
                        break
                    after = boundary_idx + len(sep)
                    header_end = buffer.find(b"\r\n\r\n", after)
                    if header_end < 0:
                        break
                    headers_blob = buffer[after:header_end].decode("ascii", errors="ignore")
                    length = self._parse_content_length(headers_blob)
                    if length is None:
                        # Malformed; drop everything up to next boundary attempt.
                        buffer = buffer[after:]
                        continue
                    body_start = header_end + 4
                    body_end = body_start + length
                    if len(buffer) < body_end:
                        break
                    jpeg = buffer[body_start:body_end]
                    self._publish_image(jpeg)
                    buffer = buffer[body_end:]

    @staticmethod
    def _extract_boundary(content_type: str) -> Optional[str]:
        for part in content_type.split(";"):
            part = part.strip()
            if part.startswith("boundary="):
                return part.split("=", 1)[1].strip().strip('"')
        return None

    @staticmethod
    def _parse_content_length(headers_blob: str) -> Optional[int]:
        for line in headers_blob.split("\r\n"):
            if line.lower().startswith("content-length:"):
                try:
                    return int(line.split(":", 1)[1].strip())
                except ValueError:
                    return None
        return None

    def _publish_image(self, jpeg: bytes) -> None:
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.camera_frame
        msg.format = "jpeg"
        msg.data = jpeg
        self.image_pub.publish(msg)

    # ------- detections (WebSocket) -------

    def _stream_loop(self) -> None:
        try:
            from websockets.sync.client import connect
        except ImportError:
            self.get_logger().error(
                "websockets package missing — pip install websockets in this environment"
            )
            return

        url = f"ws://{self.pi_host}:{self.pi_port}/stream"
        while not self._stop.is_set():
            try:
                with connect(url, open_timeout=5.0) as ws:
                    self.get_logger().info(f"Connected to {url}")
                    while not self._stop.is_set():
                        text = ws.recv(timeout=10.0)
                        if isinstance(text, bytes):
                            text = text.decode("utf-8", errors="replace")
                        self._handle_detections(text)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(
                    f"WS error ({exc}); reconnecting in {self.reconnect_delay}s"
                )
                time.sleep(self.reconnect_delay)

    def _handle_detections(self, payload_text: str) -> None:
        try:
            payload = json.loads(payload_text)
        except json.JSONDecodeError:
            return

        out = TrackedObjectArray()
        stamp = self.get_clock().now().to_msg()
        out.header.stamp = stamp
        out.header.frame_id = self.world_frame

        for obj in payload.get("objects", []):
            tracked = TrackedObject()
            tracked.tracking_id = str(obj.get("tracking_id", ""))
            tracked.class_id = str(obj.get("class_id", ""))
            tracked.confidence = float(obj.get("confidence", 0.0))
            pose = Pose()
            pose.position.x = float(obj["x"])
            pose.position.y = float(obj["y"])
            pose.position.z = float(obj["z"])
            pose.orientation.w = 1.0
            tracked.pose = pose
            depth = float(obj.get("depth_m", 0.0))
            bbox = obj.get("bbox") or {}
            dims = Vector3()
            dims.x = max(float(bbox.get("w_norm", 0.05)) * depth, 0.1)
            dims.y = dims.x
            dims.z = max(float(bbox.get("h_norm", 0.1)) * depth, 0.1)
            tracked.dimensions = dims
            sigma = max(0.02, 0.02 * depth * depth)
            var = sigma * sigma
            tracked.position_covariance = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, var]
            tracked.last_observed = stamp
            tracked.is_persistent = False
            tracked.source_frame = self.camera_frame
            out.objects.append(tracked)

        self.objects_pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PiBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
