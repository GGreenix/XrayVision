import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

from xray_core.math_utils import quaternion_from_euler


class StaticPosePublisher(Node):
    """Publishes a fixed pose on /xray/uav/odom and the map->base_link TF.

    Use case: a Raspberry Pi camera station mounted at a known location. The
    pose never changes, but downstream nodes (object_localizer, Unity bridge)
    expect a heartbeat on /xray/uav/odom, so we republish at a low rate.
    """

    def __init__(self) -> None:
        super().__init__("static_pose_publisher")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("rate_hz", 2.0)
        self.declare_parameter("position_m", [0.0, 0.0, 1.5])
        self.declare_parameter("rpy_deg", [0.0, 0.0, 0.0])

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.child_frame_id = str(self.get_parameter("child_frame_id").value)
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.position = [float(value) for value in self.get_parameter("position_m").value]
        rpy_deg = [float(value) for value in self.get_parameter("rpy_deg").value]

        roll = rpy_deg[0] * 3.141592653589793 / 180.0
        pitch = rpy_deg[1] * 3.141592653589793 / 180.0
        yaw = rpy_deg[2] * 3.141592653589793 / 180.0
        self.quaternion = quaternion_from_euler(roll, pitch, yaw)

        self.publisher = self.create_publisher(Odometry, "/xray/uav/odom", 10)
        self.tf_static = StaticTransformBroadcaster(self)
        self._broadcast_static_tf()
        self.create_timer(1.0 / max(rate_hz, 0.1), self._publish_odom)

        self.get_logger().info(
            f"static_pose_publisher: position={self.position}, "
            f"quaternion={self.quaternion}, rate={rate_hz}Hz"
        )

    def _broadcast_static_tf(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.frame_id
        transform.child_frame_id = self.child_frame_id
        transform.transform.translation.x = self.position[0]
        transform.transform.translation.y = self.position[1]
        transform.transform.translation.z = self.position[2]
        transform.transform.rotation.x = self.quaternion[0]
        transform.transform.rotation.y = self.quaternion[1]
        transform.transform.rotation.z = self.quaternion[2]
        transform.transform.rotation.w = self.quaternion[3]
        self.tf_static.sendTransform(transform)

    def _publish_odom(self) -> None:
        odometry = Odometry()
        odometry.header.stamp = self.get_clock().now().to_msg()
        odometry.header.frame_id = self.frame_id
        odometry.child_frame_id = self.child_frame_id
        odometry.pose.pose.position.x = self.position[0]
        odometry.pose.pose.position.y = self.position[1]
        odometry.pose.pose.position.z = self.position[2]
        odometry.pose.pose.orientation.x = self.quaternion[0]
        odometry.pose.pose.orientation.y = self.quaternion[1]
        odometry.pose.pose.orientation.z = self.quaternion[2]
        odometry.pose.pose.orientation.w = self.quaternion[3]
        self.publisher.publish(odometry)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StaticPosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
