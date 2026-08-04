from glob import glob

from setuptools import find_packages, setup

package_name = 'g1_motion_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        # 策略权重跟着包一起安装，部署机上不需要再拷贝一次。
        ('share/' + package_name + '/config', glob('config/*.onnx')),
        # WebXR 采集页与监控页，由 vr_teleop 节点内嵌的 aiohttp 直接托管。
        ('share/' + package_name + '/vr', glob('vr/*.html')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ruigangli',
    maintainer_email='ruigangli@microsoft.com',
    description='G1 整机 31 轴运动控制层（下肢策略 + 上肢 IK + 遥操），位于 forward_position_controller 之上。',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'policy_node = g1_motion_control.policy_node:main',
            'teleop_keyboard = g1_motion_control.teleop_keyboard:main',
            'vr_teleop = g1_motion_control.vr_teleop:main',
            'make_vr_cert = g1_motion_control.make_vr_cert:main',
        ],
    },
)
