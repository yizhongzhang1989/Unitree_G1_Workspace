import os
from glob import glob

from setuptools import setup

package_name = 'g1_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Unitree G1 Workspace',
    maintainer_email='dev@example.com',
    description='G1 头部雷达世界定位：Point-LIO 之上的躯干位姿接口',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localization_node = g1_localization.localization_node:main',
        ],
    },
)
