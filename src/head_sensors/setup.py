import os
from glob import glob

from setuptools import setup

package_name = 'head_sensors'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Unitree G1 Workspace',
    maintainer_email='dev@example.com',
    description='G1 头部传感器调用层：Livox MID-360 雷达接入与 RealSense D435i 集成',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'head_lidar_node = head_sensors.head_lidar_node:main',
            'head_sensors_probe = head_sensors.probe:main',
        ],
    },
)
