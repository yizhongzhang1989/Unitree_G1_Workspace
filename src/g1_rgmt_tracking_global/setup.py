from glob import glob

from setuptools import find_packages, setup

package_name = 'g1_rgmt_tracking_global'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        # 策略权重与契约跟着包一起安装。
        ('share/' + package_name + '/config',
         glob('config/*.onnx') + glob('config/*.json')),
        # 参考动作。换动作只要往这里放 NPZ，不需要重新导出策略。
        ('share/' + package_name + '/config/motions', glob('config/motions/*.npz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ruigangli',
    maintainer_email='ruigangli@microsoft.com',
    description='G1 全身动作跟踪（RGMT + 全局位置），29 轴关节目标，需里程计。',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tracking_node = g1_rgmt_tracking_global.tracking_node:main',
            'teleop_keyboard = g1_rgmt_tracking_global.teleop_keyboard:main',
        ],
    },
)
