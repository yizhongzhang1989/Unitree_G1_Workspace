import os
from glob import glob

from setuptools import setup

package_name = 'camera_calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_data={
        package_name: ['static/*.html', 'static/*.css', 'static/*.js'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='dev@example.com',
    description='G1 三相机内参标定与双腕相机外参标定',
    license='MIT',
    entry_points={
        'console_scripts': [
            'calib_node = camera_calibration.calib_node:main',
            'calib_tf_node = camera_calibration.calib_tf_node:main',
            'make_board = camera_calibration.make_board:main',
        ],
    },
)
