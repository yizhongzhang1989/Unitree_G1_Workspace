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
        # 导出机（Windows，无 ROS）把这个目录整个拷走就能读 session。
        # 只捞 *.py 会漏掉 README.md 和 sync_and_convert.ps1，本地看不出来，
        # B 点下载到的是残缺的一份 —— 而 ps1 正是 README 让他敲的第一条命令。
        (os.path.join('share', package_name, 'tools'),
            glob(os.path.join('tools', '*.py')) + glob(os.path.join('tools', '*.md'))
            + glob(os.path.join('tools', '*.ps1'))),
    ] + [
        (os.path.join('share', package_name, os.path.dirname(path)), [path])
        for path in glob(os.path.join('tools', 'format', '*', '*'))
        if os.path.isfile(path)
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
            'data_manager = record.data_node:main',
            'verify_alignment = record.verify_alignment:main',
        ],
    },
)
