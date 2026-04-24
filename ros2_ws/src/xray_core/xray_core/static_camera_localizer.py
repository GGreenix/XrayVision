"""Adapter node: localize detections from a fixed, pre-surveyed camera.

Used on the Pi. Camera pose in the world frame is a parameter (same values
as static_pose_publisher), so we never touch TF or odom.
"""

import math

import rclpy
from geometry_msgs.msg import Pose, Vector3
from rclpy.node import Node

from xray_core.localization.camera_model import CameraIntrinsics, default_intrinsics_for_fov
from xray_core.localization.ground_plane import bbox_bottom_center_px, localize_pixel
from xray_core.math_utils import add_vectors, matrix_vector_multiply, rotation_matrix_from_euler
from xray_interfaces.msg import Detection2DArray, TrackedObject, TrackedObjectArray


class StaticCameraLocalizer(Node):
    def __init__(self) -> None:
        super().__init__("static_camera_localizer")

        self.declare_parameter("world_frame", "map")
        self.declare_parameter("ground_z_m", 0.0)

        # Camera pose in the world frame (same convention as static_pose_publisher).
        self.declare_parameter("position_m", [0.0, 0.0, 1.5])
        self.declare_parameter("rpy_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("camera_mount_rpy_deg", [0.0, 0.0, 0.0])

        # Intrinsics. If fx/fy are <= 0 we fall back to an FOV-based estimate.
        self.declare_parameter("image_width", 1280)
        self.declare_parameter("image_height", 720)
        self.declare_parameter("fx", 0.0)
        self.declare_parameter("fy", 0.0)
        self.declare_parameter("cx", 0.0)
        self.declare_parameter("cy", 0.0)
        self.declare_parameter("horizontal_fov_deg", 66.0)  # Pi Camera v2 default-ish

        self.world_frame = str(self.get_parameter("world_frame").value)
        self.ground_z = float(self.get_parameter("ground_z_m").value)

        position = [float(v) for v in self.get_parameter("position_m").value]
        rpy_deg = [float(v) for v in self.get_parameter("rpy_deg").value]
        mount_rpy_deg = [float(v) for v in self.get_parameter("camera_mount_rpy_deg").value]

        width = int(self.get_parameter("image_width").value)
        height = int(self.get_parameter("image_height").value)
        fx = float(self.get_parameter("fx").value)
        fy = float(self.get_parameter("fy").value)

        if fx > 0.0 and fy > 0.0:
            cx_param = float(self.get_parameter("cx").value) or None
            cy_param = float(self.get_parameter("cy").value) or None
            self.intrinsics = CameraIntrinsics.from_params(width, height, fx, fy, cx_param, cy_param)
        else:
            fov = float(self.get_parameter("horizontal_fov_deg").value)
            self.intrinsics = default_intrinsics_for_fov(width, height, fov)
            self.get_logger().warn(
                "No fx/fy parameters set; using FOV-based intrinsics "
                f"(hfov={fov} deg). Calibrate the camera for real deployment."
            )

        # Cache the static transforms — they never change for a fixed camera.
        self.world_from_body = rotation_matrix_from_euler(
            math.radians(rpy_deg[0]), math.radians(rpy_deg[1]), math.radians(rpy_deg[2])
        )
        self.body_from_camera_mount = rotation_matrix_from_euler(
            math.radians(mount_rpy_deg[0]),
            math.radians(mount_rpy_deg[1]),
            math.radians(mount_rpy_deg[2]),
        )
        # Pi's body origin == Pi's camera origin for this node (no offset arm).
        self.camera_origin_world = position

        self.publisher = self.create_publisher(TrackedObjectArray, "/xray/perception/objects_raw", 10)
        self.create_subscription(Detection2DArray, "/xray/perception/detections_2d", self.on_detections, 20)

    def on_detections(self, message: Detection2DArray) -> None:
        out = TrackedObjectArray()
        out.header.stamp = message.header.stamp
        out.header.frame_id = self.world_frame

        for index, det in enumerate(message.detections):
            u_px, v_px = bbox_bottom_center_px(
                det.center_x, det.center_y, det.width, det.height, self.intrinsics
            )
            result = localize_pixel(
                u_px=u_px,
                v_px=v_px,
                intrinsics=self.intrinsics,
                world_from_body=self.world_from_body,
                body_from_camera_mount=self.body_from_camera_mount,
                camera_origin_world=self.camera_origin_world,
                ground_z=self.ground_z,
            )
            if result is None:
                continue

            tracked = TrackedObject()
            tracked.tracking_id = f"{det.class_id}-{index}"
            tracked.class_id = det.class_id
            tracked.confidence = det.confidence
            tracked.pose = self._pose_from_position(result.world_position)
            tracked.dimensions = self._dimensions_from_detection(det, result.range_m)
            tracked.position_covariance = self._covariance_from_range(result.range_m)
            tracked.last_observed = det.observed_at
            tracked.is_persistent = False
            tracked.source_frame = det.source_frame or message.header.frame_id
            out.objects.append(tracked)

        self.publisher.publish(out)

    def _pose_from_position(self, position) -> Pose:
        pose = Pose()
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        pose.orientation.w = 1.0
        return pose

    def _dimensions_from_detection(self, detection, range_m: float) -> Vector3:
        # bbox width/height are normalized; rough physical size from range.
        dims = Vector3()
        dims.x = max(float(detection.width) * range_m, 0.1)
        dims.y = max(float(detection.width) * range_m, 0.1)
        dims.z = max(float(detection.height) * range_m, 0.1)
        return dims

    def _covariance_from_range(self, range_m: float) -> list[float]:
        # Shallow-angle ground-plane geometry is noisier at range.
        variance = max(0.25, (range_m * 0.15) ** 2)
        return [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StaticCameraLocalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
