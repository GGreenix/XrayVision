import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class DronePilot(Node):
    def __init__(self) -> None:
        super().__init__("drone_pilot")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("forward_speed_m_s", 1.0)
        self.declare_parameter("yaw_rate_rad_s", 0.2)
        self.declare_parameter("target_altitude_m", 8.0)
        self.declare_parameter("altitude_kp", 2.0)
        self.declare_parameter("altitude_max_velocity_m_s", 4.0)

        self.forward_speed = float(self.get_parameter("forward_speed_m_s").value)
        self.yaw_rate = float(self.get_parameter("yaw_rate_rad_s").value)
        self.target_altitude = float(self.get_parameter("target_altitude_m").value)
        self.altitude_kp = float(self.get_parameter("altitude_kp").value)
        self.altitude_max = float(self.get_parameter("altitude_max_velocity_m_s").value)
        rate_hz = float(self.get_parameter("rate_hz").value)

        self.current_altitude = None

        self.publisher = self.create_publisher(Twist, "/xray/uav/cmd_vel", 10)
        self.create_subscription(Odometry, "/xray/uav/odom", self.odom_callback, 10)
        self.create_timer(1.0 / max(rate_hz, 1.0), self.publish_command)

    def odom_callback(self, message: Odometry) -> None:
        self.current_altitude = message.pose.pose.position.z

    def publish_command(self) -> None:
        command = Twist()
        command.linear.x = self.forward_speed
        command.angular.z = self.yaw_rate

        if self.current_altitude is not None:
            altitude_error = self.target_altitude - self.current_altitude
            vertical_velocity = self.altitude_kp * altitude_error
            vertical_velocity = max(-self.altitude_max, min(self.altitude_max, vertical_velocity))
            command.linear.z = vertical_velocity

        self.publisher.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DronePilot()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
