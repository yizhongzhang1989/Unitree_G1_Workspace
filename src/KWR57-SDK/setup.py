from setuptools import find_packages, setup

setup(
    name="kwr57-sensor",
    version="0.1.0",
    description="KWR57 六轴力/力矩传感器 CAN 通信纯 Python SDK",
    packages=find_packages(include=["kwr57_sensor", "kwr57_sensor.*"]),
    python_requires=">=3.8",
    install_requires=[
        "python-can>=4.0",
        "canalystii>=0.1",
        "libusb-package>=1.0.24",
    ],
)
