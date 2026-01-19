import os
from glob import glob
from setuptools import setup

package_name = 'a_star'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # 基础文件
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 安装所有 Launch 脚本到 launch 目录
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),

        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kongfu40',
    maintainer_email='CJKurt@github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'a_star_node = a_star.a_star_node:main'
        ],
    },
)
