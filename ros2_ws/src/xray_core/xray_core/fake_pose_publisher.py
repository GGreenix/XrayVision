import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from xray_core.math_utils import quaternion_from_euler


class FakePosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("fake_pose_publisher")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("radius_m", 12.0)
        self.declare_parameter("angular_speed_rad_s", 0.15)
        self.declare_parameter("altitude_m", 8.0)

        self.frame_id = self.get_parameter("frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.radius_m = float(self.get_parameter("radius_m").value)
        self.angular_speed = float(self.get_parameter("angular_speed_rad_s").value)
        self.altitude_m = float(self.get_parameter("altitude_m").value)

        self.publisher = self.create_publisher(Odometry, "/xray/uav/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.start_time = self.get_clock().now()
        self.create_timer(1.0 / max(self.rate_hz, 1.0), self.publish_pose)

    def publish_pose(self) -> None:
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        theta = self.angular_speed * elapsed
        heading = theta + math.pi
        quaternion = quaternion_from_euler(0.0, 0.0, heading)

        x_position = self.radius_m * math.cos(theta)
        y_position = self.radius_m * math.sin(theta)
        z_position = self.altitude_m

        x_velocity = -self.radius_m * self.angular_speed * math.sin(theta)
        y_velocity = self.radius_m * self.angular_speed * math.cos(theta)

        odometry = Odometry()
        odometry.header.stamp = now.to_msg()
        odometry.header.frame_id = self.frame_id
        odometry.child_frame_id = self.child_frame_id
        odometry.pose.pose.position.x = x_position
        odometry.pose.pose.position.y = y_position
        odometry.pose.pose.position.z = z_position
        odometry.pose.pose.orientation.x = quaternion[0]
        odometry.pose.pose.orientation.y = quaternion[1]
        odometry.pose.pose.orientation.z = quaternion[2]
        odometry.pose.pose.orientation.w = quaternion[3]
        odometry.twist.twist.linear.x = x_velocity
        odometry.twist.twist.linear.y = y_velocity
        odometry.twist.twist.angular.z = self.angular_speed
        self.publisher.publish(odometry)

        transform = TransformStamped()
        transform.header.stamp = now.to_msg()
        transform.header.frame_id = self.frame_id
        transform.child_frame_id = self.child_frame_id
        transform.transform.translation.x = x_position
        transform.transform.translation.y = y_position
        transform.transform.translation.z = z_position
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakePosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
