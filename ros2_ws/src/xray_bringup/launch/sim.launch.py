from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    backend = LaunchConfiguration("backend")
    use_fake_pose = LaunchConfiguration("use_fake_pose")
    use_fake_detection = LaunchConfiguration("use_fake_detection")
    use_sim_time = LaunchConfiguration("use_sim_time")

    sim_backend_launch = PathJoinSubstitution([FindPackageShare("xray_bringup"), "launch", "sim_backend.launch.py"])
    ground_launch = PathJoinSubstitution([FindPackageShare("xray_bringup"), "launch", "ground.launch.py"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("backend", default_value="mock"),
            DeclareLaunchArgument("use_fake_pose", default_value="true"),
            DeclareLaunchArgument("use_fake_detection", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_backend_launch),
                launch_arguments={
                    "backend": backend,
                    "use_fake_pose": use_fake_pose,
                    "use_fake_detection": use_fake_detection,
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ground_launch),
                launch_arguments={"mode": "sim", "use_sim_time": use_sim_time}.items(),
            ),
        ]
    )
