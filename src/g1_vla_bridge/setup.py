from glob import glob

from setuptools import find_packages, setup

package_name = 'g1_vla_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/backends', glob('config/backends/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ruigangli',
    maintainer_email='liruigang20131115@126.com',
    description='VLA 推理服务与 g1_motion_control 之间的桥。',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'vla_node = g1_vla_bridge.vla_node:main',
            'calibrate_frame = g1_vla_bridge.calibrate_frame:main',
        ],
    },
)
