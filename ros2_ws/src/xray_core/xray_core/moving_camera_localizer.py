"""Adapter node: localize detections from a moving camera (the drone).

Subscribes to /xray/uav/odom, caches the most recent pose, and feeds it
into the shared ground-plane localizer along with the body→camera mount
offset. Math is identical to the static Pi adapter.
"""

import math

import rclpy
from geometry_msgs.msg import Pose, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node

from xray_core.localization.camera_model import CameraIntrinsics, default_intrinsics_for_fov
from xray_core.localization.ground_plane import bbox_bottom_center_px, localize_pixel
from xray_core.math_utils import (
    add_vectors,
    matrix_vector_multiply,
    quaternion_to_matrix,
    rotation_matrix_from_euler,
)
from xray_interfaces.msg import Detection2DArray, TrackedObject, TrackedObjectArray


class MovingCameraLocalizer(Node):
    def __init__(self) -> None:
        super().__init__("moving_camera_localizer")

        self.declare_parameter("world_frame", "map")
        self.declare_parameter("ground_z_m", 0.0)

        # Camera rig offset relative to drone body.
        self.declare_parameter("camera_translation_m", [0.2, 0.0, 0.0])
        self.declare_parameter("camera_mount_rpy_deg", [0.0, -10.0, 0.0])

        self.declare_parameter("image_width", 1280)
        self.declare_parameter("image_height", 720)
        self.declare_parameter("fx", 0.0)
        self.declare_parameter("fy", 0.0)
        self.declare_parameter("cx", 0.0)
        self.declare_parameter("cy", 0.0)
        self.declare_parameter("horizontal_fov_deg", 90.0)

        self.world_frame = str(self.get_parameter("world_frame").value)
        self.ground_z = float(self.get_parameter("ground_z_m").value)
        self.camera_translation = [float(v) for v in self.get_parameter("camera_translation_m").value]
        mount_rpy_deg = [float(v) for v in self.get_parameter("camera_mount_rpy_deg").value]
        self.body_from_camera_mount = rotation_matrix_from_euler(
            math.radians(mount_rpy_deg[0]),
            math.radians(mount_rpy_deg[1]),
            math.radians(mount_rpy_deg[2]),
        )

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

        self.latest_odom: Odometry | None = None
        self.publisher = self.create_publisher(TrackedObjectArray, "/xray/perception/objects_raw", 10)
        self.create_subscription(Odometry, "/xray/uav/odom", self.on_odom, 20)
        self.create_subscription(Detection2DArray, "/xray/perception/detections_2d", self.on_detections, 20)

    def on_odom(self, message: Odometry) -> None:
        self.latest_odom = message

    def on_detections(self, message: Detection2DArray) -> None:
        if self.latest_odom is None:
            return

        pose = self.latest_odom.pose.pose
        body_position = [pose.position.x, pose.position.y, pose.position.z]
        world_from_body = quaternion_to_matrix(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        )
        camera_origin_world = add_vectors(
            body_position, matrix_vector_multiply(world_from_body, self.camera_translation)
        )

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
                world_from_body=world_from_body,
                body_from_camera_mount=self.body_from_camera_mount,
                camera_origin_world=camera_origin_world,
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
        dims = Vector3()
        dims.x = max(float(detection.width) * range_m, 0.1)
        dims.y = max(float(detection.width) * range_m, 0.1)
        dims.z = max(float(detection.height) * range_m, 0.1)
        return dims

    def _covariance_from_range(self, range_m: float) -> list[float]:
        variance = max(0.25, range_m * 0.1)
        return [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MovingCameraLocalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
