import math
from dataclasses import dataclass

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from xray_core.math_utils import (
    clamp,
    matrix_vector_multiply,
    quaternion_to_matrix,
    rotation_matrix_from_euler,
    subtract_vectors,
    transpose,
    vector_distance,
)
from xray_interfaces.msg import Detection2D, Detection2DArray


@dataclass
class WorldObject:
    class_id: str
    position: tuple[float, float, float]
    size_m: float
    confidence: float


class FakeDetector(Node):
    def __init__(self) -> None:
        super().__init__("fake_detector")
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("horizontal_fov_deg", 90.0)
        self.declare_parameter("vertical_fov_deg", 60.0)
        self.declare_parameter("camera_translation_m", [0.2, 0.0, 0.0])
        self.declare_parameter("camera_mount_rpy_deg", [0.0, -10.0, 0.0])
        self.declare_parameter(
            "objects",
            [
                "vehicle,12.0,6.0,0.0,2.5,0.95",
                "person,4.0,14.0,0.0,1.7,0.90",
                "landing_pad,-5.0,10.0,0.0,4.0,0.99",
            ],
        )

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.horizontal_fov = math.radians(float(self.get_parameter("horizontal_fov_deg").value))
        self.vertical_fov = math.radians(float(self.get_parameter("vertical_fov_deg").value))
        self.camera_translation = [float(value) for value in self.get_parameter("camera_translation_m").value]
        mount_rpy_deg = [float(value) for value in self.get_parameter("camera_mount_rpy_deg").value]
        self.body_from_camera = rotation_matrix_from_euler(
            math.radians(mount_rpy_deg[0]),
            math.radians(mount_rpy_deg[1]),
            math.radians(mount_rpy_deg[2]),
        )
        self.camera_from_body = transpose(self.body_from_camera)
        self.objects = self._parse_objects(self.get_parameter("objects").value)
        self.latest_odom = None

        self.publisher = self.create_publisher(Detection2DArray, "/xray/perception/detections_2d", 10)
        self.create_subscription(Odometry, "/xray/uav/odom", self.odom_callback, 10)
        self.create_timer(1.0 / max(self.rate_hz, 1.0), self.publish_detections)

    def _parse_objects(self, specs) -> list[WorldObject]:
        parsed = []
        for spec in specs:
            class_id, x_pos, y_pos, z_pos, size_m, confidence = [item.strip() for item in spec.split(",")]
            parsed.append(
                WorldObject(
                    class_id=class_id,
                    position=(float(x_pos), float(y_pos), float(z_pos)),
                    size_m=float(size_m),
                    confidence=float(confidence),
                )
            )
        return parsed

    def odom_callback(self, message: Odometry) -> None:
        self.latest_odom = message

    def publish_detections(self) -> None:
        if self.latest_odom is None:
            return

        now = self.get_clock().now().to_msg()
        pose = self.latest_odom.pose.pose
        drone_position = [pose.position.x, pose.position.y, pose.position.z]
        world_from_body = quaternion_to_matrix(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        )
        body_from_world = transpose(world_from_body)
        camera_world_offset = matrix_vector_multiply(world_from_body, self.camera_translation)
        camera_world_position = [drone_position[index] + camera_world_offset[index] for index in range(3)]

        array = Detection2DArray()
        array.header.stamp = now
        array.header.frame_id = self.frame_id

        for world_object in self.objects:
            delta_world = subtract_vectors(world_object.position, camera_world_position)
            delta_body = matrix_vector_multiply(body_from_world, delta_world)
            delta_camera = matrix_vector_multiply(self.camera_from_body, delta_body)

            forward = delta_camera[0]
            if forward <= 0.0:
                continue

            horizontal_angle = math.atan2(-delta_camera[1], forward)
            vertical_angle = math.atan2(delta_camera[2], forward)
            center_x = 0.5 + (horizontal_angle / self.horizontal_fov)
            center_y = 0.5 - (vertical_angle / self.vertical_fov)
            if center_x < 0.0 or center_x > 1.0 or center_y < 0.0 or center_y > 1.0:
                continue

            range_estimate = vector_distance(world_object.position, camera_world_position)
            box_height = clamp((world_object.size_m / max(range_estimate, 1.0)) * 0.8, 0.04, 0.35)
            box_width = clamp(box_height * 0.8, 0.03, 0.30)

            detection = Detection2D()
            detection.observed_at = now
            detection.class_id = world_object.class_id
            detection.confidence = world_object.confidence
            detection.center_x = float(center_x)
            detection.center_y = float(center_y)
            detection.width = float(box_width)
            detection.height = float(box_height)
            detection.range_estimate_m = float(range_estimate)
            detection.source_frame = self.frame_id
            array.detections.append(detection)

        self.publisher.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
