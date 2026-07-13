from setuptools import find_packages, setup

package_name = "can_sdk"

setup(
    name=package_name,
    version="0.1.0",
    description="Device-agnostic python-can backend and single-consumer transport",
    packages=find_packages(include=["can_sdk", "can_sdk.*"]),
    python_requires=">=3.8",
    install_requires=["python-can>=4.0"],
    extras_require={
        "canalystii": [
            "canalystii>=0.1",
            "libusb-package>=1.0.30",
        ],
    },
    zip_safe=True,
)
