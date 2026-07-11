from glob import glob

from setuptools import find_packages, setup

package_name = "can_bridge_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="madderscientist",
    maintainer_email="liruigang20131115@126.com",
    description="Generic ROS2 CAN bus bridge (python-can <-> can_msgs/Frame).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bridge_node = can_bridge_ros.bridge_node:main",
        ],
    },
)
