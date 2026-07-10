from setuptools import find_packages, setup
from glob import glob

package_name = 'kwr57_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    # kwr57_sensor(SDK) + python-can 由 pip 装进 ROS 2 的 Python；见 README。
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='madderscientist',
    maintainer_email='liruigang20131115@126.com',
    description='ROS 2 driver for the KWR57 6-axis force/torque sensor (CAN).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ft_sensor_node = kwr57_ros.ft_sensor_node:main',
            'wrench_echo = kwr57_ros.wrench_echo:main',
            'web_wrench = kwr57_ros.web_wrench_node:main',
            'read_kwr57 = kwr57_sensor.cli:main',
        ],
    },
)
