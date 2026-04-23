from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share_dir = get_package_share_directory("xray_bringup")
    backend = LaunchConfiguration("backend")
    use_fake_pose = LaunchConfiguration("use_fake_pose")
    use_fake_detection = LaunchConfiguration("use_fake_detection")
    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")
    gazebo_backend_launch = PathJoinSubstitution([FindPackageShare("xray_bringup"), "launch", "gazebo_backend.launch.py"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("backend", default_value="mock"),
            DeclareLaunchArgument("use_fake_pose", default_value="true"),
            DeclareLaunchArgument("use_fake_detection", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("headless", default_value="true"),
            LogInfo(msg=["Starting simulation backend: ", backend]),
            GroupAction(
                condition=IfCondition(PythonExpression(["'", backend, "' == 'mock'"])),
                actions=[
                    Node(
                        package="tf2_ros",
                        executable="static_transform_publisher",
                        arguments=["0.2", "0.0", "0.0", "0.0", "0.785398", "0.0", "base_link", "camera_link"],
                    ),
                    Node(
                        package="xray_core",
                        executable="fake_pose_publisher",
                        name="fake_pose_publisher",
                        parameters=[
                            share_dir + "/config/fake_pose.yaml",
                            {"use_sim_time": use_sim_time},
                        ],
                        condition=IfCondition(use_fake_pose),
                    ),
                    Node(
                        package="xray_core",
                        executable="fake_detector",
                        name="fake_detector",
                        parameters=[
                            share_dir + "/config/fake_detector.yaml",
                            {"use_sim_time": use_sim_time},
                        ],
                        condition=IfCondition(use_fake_detection),
                    ),
                    Node(
                        package="xray_core",
                        executable="object_localizer",
                        name="object_localizer",
                        parameters=[
                            share_dir + "/config/object_localizer.yaml",
                            {"use_sim_time": use_sim_time},
                        ],
                    ),
                ],
            ),
            GroupAction(
                condition=IfCondition(PythonExpression(["'", backend, "' == 'gazebo'"])),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(gazebo_backend_launch),
                        launch_arguments={
                            "use_fake_detection": use_fake_detection,
                            "use_sim_time": use_sim_time,
                            "headless": headless,
                        }.items(),
                    )
                ],
            ),
        ]
    )
