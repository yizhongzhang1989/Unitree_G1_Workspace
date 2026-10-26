from glob import glob

from setuptools import find_packages, setup

package_name = 'upper_vla'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/upper_vla/online_rl', ['upper_vla/online_rl/README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/backends', glob('config/backends/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ruigangli',
    maintainer_email='liruigang20131115@126.com',
    description='VLA 上肢任务，使用共享任务基类与本地 50 Hz IK。',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'vla_node = upper_vla.vla_node:main',
            'vla_cli = upper_vla.vla_cli:main',
            'gripper_gate_node = upper_vla.gripper_gate_node:main',
            'online_rl_bridge = upper_vla.online_rl.bridge:main',
            'online_rl_cli = upper_vla.online_rl.cli:main',
        ],
    },
)
