from glob import glob

from setuptools import find_packages, setup

package_name = 'g1_gmt_tracking'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # 配置、策略权重与契约。contract 里有 ONNX metadata 缺的那几个键，缺它起不来。
        ('share/' + package_name + '/config',
         glob('config/*.yaml') + glob('config/*.onnx') + glob('config/*.json')),
        # 参考动作。换动作只要往这里放 NPZ，不需要重新导出策略。
        ('share/' + package_name + '/config/motions', glob('config/motions/*.npz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ruigangli',
    maintainer_email='ruigangli@microsoft.com',
    description='G1 全身动作跟踪（GMT），29 轴关节目标，位于 forward_position_controller 之上。',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tracking_node = g1_gmt_tracking.tracking_node:main',
            'teleop_keyboard = g1_gmt_tracking.teleop_keyboard:main',
        ],
    },
)
