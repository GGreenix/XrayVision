"""Bridge a non-ROS Pi station into the existing PC-side ROS stack."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share_dir = get_package_share_directory("xray_bringup")
    config_file = share_dir + "/config/pi_bridge.yaml"

    return LaunchDescription(
        [
            Node(
                package="xray_core",
                executable="pi_bridge",
                name="pi_bridge",
                parameters=[config_file],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "0.0", "0.0", "0.0",
                    "0.0", "0.0", "0.0",
                    "base_link", "camera_left_optical",
                ],
            ),
            Node(
                package="xray_core",
                executable="world_model",
                name="world_model",
                output="screen",
            ),
        ]
    )
