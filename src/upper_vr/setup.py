from glob import glob

from setuptools import find_packages, setup

package_name = 'upper_vr'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/vr', glob('vr/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ruigangli',
    maintainer_email='ruigangli@microsoft.com',
    description='WebXR upper-body task with local IK and masked FPC output.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'vr_teleop = upper_vr.vr_teleop:main',
        'make_vr_cert = upper_vr.make_vr_cert:main',
    ]},
)
