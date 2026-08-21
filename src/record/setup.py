import os
from glob import glob

from setuptools import setup

package_name = 'record'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.instruction'],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))),
        # 物品库随包分发。上游导出包有 773 个文件 / 57 MB，只有 db 是代码要的，
        # preview/crops 那 55 MB 图与采集无关，入库前已删掉。
        (os.path.join('share', package_name, 'items'), ['items/README.md']),
        (os.path.join('share', package_name, 'items', 'item-library', 'db'),
            ['items/item-library/db/item_library.db']),
        # 导出机（Windows，无 ROS）把这个目录整个拷走就能读 session
        (os.path.join('share', package_name, 'tools'),
            glob(os.path.join('tools', '*.py'))),
    ],
    package_data={
        package_name: ['static/*.html', 'static/*.css', 'static/*.js',
                       'data/*.npz'],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='madderscientist',
    maintainer_email='liruigang20131115@126.com',
    description='上肢 VR 遥操作数据采集：三路视频 + 信号表 + 指令生成 + 采集面板',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'recorder = record.recorder_node:main',
        ],
    },
)
