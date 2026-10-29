from glob import glob

from setuptools import Extension, find_packages, setup

package_name = 'upper_insert'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    ext_modules=[
        Extension(
            'upper_insert._vision_native',
            ['upper_insert/vision/vision_native.cpp'],
            include_dirs=['/usr/include/opencv4'],
            libraries=['opencv_core', 'opencv_imgproc'],
            extra_compile_args=['-std=c++17', '-O3', '-ffp-contract=off'],
            language='c++',
        ),
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'TROUBLESHOOTING.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        (
            'share/' + package_name + '/config',
            glob('config/*.yaml')
            + glob('config/*.json')
            + glob('config/*.onnx')
            + glob('config/*.md'),
        ),
        ('share/' + package_name + '/static', glob('static/*.html')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ruigangli',
    maintainer_email='ruigangli@microsoft.com',
    description='6DoF ArUco plug-hole localization and 50Hz ONNX end-effector insertion control task.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'insert_node = upper_insert.insert_node:main',
            'generate_markers = upper_insert.vision.generate_markers:main',
        ],
    },
)
