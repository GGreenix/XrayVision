from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share_dir = get_package_share_directory("xray_bringup")
    use_fake_pose = LaunchConfiguration("use_fake_pose")
    use_fake_detection = LaunchConfiguration("use_fake_detection")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_fake_pose", default_value="false"),
            DeclareLaunchArgument("use_fake_detection", default_value="false"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0.2", "0.0", "0.0", "0.0", "0.785398", "0.0", "base_link", "camera_link"],
            ),
            Node(
                package="xray_core",
                executable="fake_pose_publisher",
                name="fake_pose_publisher",
                parameters=[share_dir + "/config/fake_pose.yaml"],
                condition=IfCondition(use_fake_pose),
            ),
            Node(
                package="xray_core",
                executable="fake_detector",
                name="fake_detector",
                parameters=[share_dir + "/config/fake_detector.yaml"],
                condition=IfCondition(use_fake_detection),
            ),
            Node(
                package="xray_core",
                executable="object_localizer",
                name="object_localizer",
                parameters=[share_dir + "/config/object_localizer.yaml"],
            ),
        ]
    )
