from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share_dir = get_package_share_directory("xray_bringup")
    mode = LaunchConfiguration("mode")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="sim"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            LogInfo(msg=["Starting ground stack in mode: ", mode]),
            Node(
                package="xray_core",
                executable="world_model",
                name="world_model",
                parameters=[
                    share_dir + "/config/world_model.yaml",
                    {"use_sim_time": use_sim_time},
                ],
            ),
        ]
    )
