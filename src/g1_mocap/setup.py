from glob import glob

from setuptools import find_packages, setup

package_name = 'g1_mocap'
share = 'share/' + package_name

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (share, ['package.xml', 'README.md']),
        (share + '/launch', glob('launch/*.launch.py')),
        (share + '/config', glob('config/*.yaml')),
        # 面板页面。vendor 里是 three.js 与两个 addon，本地托管——机器人上没有外网。
        (share + '/static', glob('static/*.html') + glob('static/*.js')),
        (share + '/static/vendor', glob('static/vendor/*.js')),
        (share + '/static/vendor/addons/controls', glob('static/vendor/addons/controls/*.js')),
        (share + '/static/vendor/addons/loaders', glob('static/vendor/addons/loaders/*.js')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ruigangli',
    maintainer_email='ruigangli@microsoft.com',
    description='PICO 全身动捕重定向到 G1 29 轴关节角，不含策略逻辑。',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mocap_node = g1_mocap.mocap_node:main',
            'dashboard_node = g1_mocap.dashboard_node:main',
        ],
    },
)
