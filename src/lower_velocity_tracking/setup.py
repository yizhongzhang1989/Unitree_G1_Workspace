from glob import glob

from setuptools import find_packages, setup

package_name = 'lower_velocity_tracking'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.onnx')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ruigangli',
    maintainer_email='ruigangli@microsoft.com',
    description='Independent G1 lower-body velocity tracking task.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'velocity_task = lower_velocity_tracking.velocity_task:main',
        'teleop_keyboard = lower_velocity_tracking.teleop_keyboard:main',
    ]},
)
