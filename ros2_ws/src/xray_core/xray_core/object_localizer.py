import math

import rclpy
from geometry_msgs.msg import Pose, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node

from xray_core.math_utils import (
    add_vectors,
    matrix_multiply,
    matrix_vector_multiply,
    normalize,
    quaternion_to_matrix,
    rotation_matrix_from_euler,
    scale_vector,
)
from xray_interfaces.msg import Detection2DArray, TrackedObject, TrackedObjectArray


class ObjectLocalizer(Node):
    def __init__(self) -> None:
        super().__init__("object_localizer")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("horizontal_fov_deg", 90.0)
        self.declare_parameter("vertical_fov_deg", 60.0)
        self.declare_parameter("min_range_m", 1.0)
        self.declare_parameter("camera_translation_m", [0.2, 0.0, 0.0])
        self.declare_parameter("camera_mount_rpy_deg", [0.0, -10.0, 0.0])

        self.world_frame = str(self.get_parameter("world_frame").value)
        self.horizontal_fov = math.radians(float(self.get_parameter("horizontal_fov_deg").value))
        self.vertical_fov = math.radians(float(self.get_parameter("vertical_fov_deg").value))
        self.min_range_m = float(self.get_parameter("min_range_m").value)
        self.camera_translation = [float(value) for value in self.get_parameter("camera_translation_m").value]
        mount_rpy_deg = [float(value) for value in self.get_parameter("camera_mount_rpy_deg").value]
        self.body_from_camera = rotation_matrix_from_euler(
            math.radians(mount_rpy_deg[0]),
            math.radians(mount_rpy_deg[1]),
            math.radians(mount_rpy_deg[2]),
        )

        self.latest_odom = None
        self.publisher = self.create_publisher(TrackedObjectArray, "/xray/perception/objects_raw", 10)
        self.create_subscription(Odometry, "/xray/uav/odom", self.odom_callback, 20)
        self.create_subscription(Detection2DArray, "/xray/perception/detections_2d", self.detections_callback, 20)

    def odom_callback(self, message: Odometry) -> None:
        self.latest_odom = message

    def detections_callback(self, message: Detection2DArray) -> None:
        if self.latest_odom is None:
            return

        pose = self.latest_odom.pose.pose
        drone_position = [pose.position.x, pose.position.y, pose.position.z]
        world_from_body = quaternion_to_matrix(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        )
        world_from_camera = matrix_multiply(world_from_body, self.body_from_camera)
        camera_world_position = add_vectors(drone_position, matrix_vector_multiply(world_from_body, self.camera_translation))

        array = TrackedObjectArray()
        array.header.stamp = message.header.stamp
        array.header.frame_id = self.world_frame

        for index, detection in enumerate(message.detections):
            horizontal_angle = (float(detection.center_x) - 0.5) * self.horizontal_fov
            vertical_angle = (0.5 - float(detection.center_y)) * self.vertical_fov
            ray_camera = normalize(
                [1.0, -math.tan(horizontal_angle), math.tan(vertical_angle)]
            )
            ray_world = matrix_vector_multiply(world_from_camera, ray_camera)
            range_estimate = max(float(detection.range_estimate_m), self.min_range_m)
            world_position = add_vectors(camera_world_position, scale_vector(ray_world, range_estimate))

            tracked_object = TrackedObject()
            tracked_object.tracking_id = f"{detection.class_id}-{index}"
            tracked_object.class_id = detection.class_id
            tracked_object.confidence = detection.confidence
            tracked_object.pose = self._pose_from_position(world_position)
            tracked_object.dimensions = self._dimensions_from_detection(detection)
            tracked_object.position_covariance = self._covariance_from_range(range_estimate)
            tracked_object.last_observed = detection.observed_at
            tracked_object.is_persistent = False
            tracked_object.source_frame = detection.source_frame or message.header.frame_id
            array.objects.append(tracked_object)

        self.publisher.publish(array)

    def _pose_from_position(self, position) -> Pose:
        pose = Pose()
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        pose.orientation.w = 1.0
        return pose

    def _dimensions_from_detection(self, detection) -> Vector3:
        dimensions = Vector3()
        dimensions.x = max(float(detection.width) * float(detection.range_estimate_m), 0.1)
        dimensions.y = max(float(detection.width) * float(detection.range_estimate_m), 0.1)
        dimensions.z = max(float(detection.height) * float(detection.range_estimate_m), 0.1)
        return dimensions

    def _covariance_from_range(self, range_estimate: float) -> list[float]:
        variance = max(0.25, range_estimate * 0.1)
        return [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectLocalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
