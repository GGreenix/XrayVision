from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ros_tcp_port = LaunchConfiguration("ros_tcp_port")

    return LaunchDescription(
        [
            DeclareLaunchArgument("ros_tcp_port", default_value="10000"),
            Node(
                package="ros_tcp_endpoint",
                executable="default_server_endpoint",
                name="ros_tcp_endpoint",
                emulate_tty=True,
                parameters=[{"ROS_IP": "0.0.0.0"}, {"ROS_TCP_PORT": ros_tcp_port}],
            ),
        ]
    )

