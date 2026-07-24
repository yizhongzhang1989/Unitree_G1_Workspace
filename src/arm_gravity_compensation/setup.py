from glob import glob

from setuptools import find_packages, setup


package_name = "arm_gravity_compensation"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    package_data={package_name: ["static/*.html", "static/*.css", "static/*.js"]},
    include_package_data=True,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="madderscientist",
    maintainer_email="liruigang20131115@126.com",
    description="Torque-only Unitree G1 arm gravity parameter calibration.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gravity_calibration = arm_gravity_compensation.workflow_node:main",
        ],
    },
)