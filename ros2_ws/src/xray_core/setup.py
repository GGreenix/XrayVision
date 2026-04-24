from setuptools import setup

package_name = "xray_core"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name, f"{package_name}.localization"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="XrayVision",
    maintainer_email="dev@example.com",
    description="Core ROS 2 nodes for the XrayVision starter stack.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "fake_pose_publisher = xray_core.fake_pose_publisher:main",
            "fake_detector = xray_core.fake_detector:main",
            "object_localizer = xray_core.object_localizer:main",
            "static_camera_localizer = xray_core.static_camera_localizer:main",
            "moving_camera_localizer = xray_core.moving_camera_localizer:main",
            "yolo_detector = xray_core.yolo_detector:main",
            "world_model = xray_core.world_model:main",
            "drone_pilot = xray_core.drone_pilot:main",
            "camera_publisher = xray_core.camera_publisher:main",
            "static_pose_publisher = xray_core.static_pose_publisher:main",
        ]
    },
)

