from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share_dir = get_package_share_directory("xray_bringup")
    world_file = share_dir + "/worlds/xray_world.sdf"
    bridge_config = share_dir + "/config/gazebo_bridge.yaml"
    use_fake_detection = LaunchConfiguration("use_fake_detection")
    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")
    auto_pilot = LaunchConfiguration("auto_pilot")
    ros_gz_launch = get_package_share_directory("ros_gz_sim") + "/launch/gz_sim.launch.py"
    gz_args = PythonExpression(
        ["'-r -s ' + '", world_file, "' if '", headless, "' == 'true' else '-r ' + '", world_file, "'"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_fake_detection", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("auto_pilot", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ros_gz_launch),
                launch_arguments={"gz_args": gz_args}.items(),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="gazebo_bridge",
                output="screen",
                parameters=[{"config_file": bridge_config}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0.2", "0.0", "0.0", "0.0", "0.785398", "0.0", "base_link", "camera_link"],
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
            Node(
                package="xray_core",
                executable="drone_pilot",
                name="drone_pilot",
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(auto_pilot),
            ),
        ]
    )

