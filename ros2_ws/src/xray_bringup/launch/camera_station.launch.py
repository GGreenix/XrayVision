from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share_dir = get_package_share_directory("xray_bringup")
    config_file = share_dir + "/config/camera_station.yaml"

    return LaunchDescription(
        [
            Node(
                package="xray_core",
                executable="camera_publisher",
                name="camera_publisher",
                parameters=[config_file],
                output="screen",
            ),
            Node(
                package="xray_core",
                executable="static_pose_publisher",
                name="static_pose_publisher",
                parameters=[config_file],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "0.0", "0.0", "0.0",
                    "0.0", "0.0", "0.0",
                    "base_link", "camera_link",
                ],
            ),
            Node(
                package="xray_core",
                executable="yolo_detector",
                name="yolo_detector",
                parameters=[config_file],
                output="screen",
            ),
            Node(
                package="xray_core",
                executable="static_camera_localizer",
                name="static_camera_localizer",
                parameters=[config_file],
                output="screen",
            ),
            Node(
                package="xray_core",
                executable="world_model",
                name="world_model",
                output="screen",
            ),
        ]
    )
