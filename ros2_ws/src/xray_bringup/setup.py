from glob import glob
from setuptools import setup

package_name = "xray_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="XrayVision",
    maintainer_email="dev@example.com",
    description="Launch files and configuration for the XrayVision starter stack.",
    license="Apache-2.0",
)
