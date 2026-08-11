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
    package_data={
        package_name + '.end_effectors': ['dashboard.html'],
        package_name + '.lowlevel': [
            'static/*.html', 'static/*.css', 'static/*.js'],
    },
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='madderscientist',
    maintainer_email='liruigang20131115@126.com',
    description='Robot bringup with separated whole-body and end-effector '
                'entry points.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'end_effectors_dashboard = '
            'robot_bringup.end_effectors.dashboard_node:main',
            'exit_debug_mode = '
            'robot_bringup.exit_debug_mode:main',
            'gravity_float_demo = '
            'robot_bringup.gravity_float_demo:main',
            'ikt_pose_commander = '
            'robot_bringup.ikt_pose_commander_compat:main',
            'ikt_pose_commander_dashboard = '
            'robot_bringup.ikt_pose_commander_compat:dashboard_main',
            'lowlevel_dashboard = '
            'robot_bringup.lowlevel.dashboard_node:main',
            'whole_body_dashboard = '
            'robot_bringup.dashboard_compat_node:main',
        ],
    },
)
