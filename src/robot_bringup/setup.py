from setuptools import find_packages, setup
from glob import glob

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name,
         ['package.xml', 'README.md', 'CAN_BUS_LOAD.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    package_data={package_name: ['web_dashboard.html']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='madderscientist',
    maintainer_email='liruigang20131115@126.com',
    description='Bringup for CAN devices and left/right IP cameras.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_dashboard = robot_bringup.web_dashboard_node:main',
        ],
    },
)
