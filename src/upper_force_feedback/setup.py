from glob import glob

from setuptools import find_packages, setup

package_name = 'upper_force_feedback'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ruigangli',
    maintainer_email='ruigangli@microsoft.com',
    description='Bilateral arm joint position coupling for force feedback.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'force_feedback_node = upper_force_feedback.force_feedback_node:main',
    ]},
)
