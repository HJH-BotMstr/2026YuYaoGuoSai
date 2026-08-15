from setuptools import setup
import os
from glob import glob

package_name = 'block_align'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/block_align.launch.py']),
        ('share/' + package_name + '/config', ['config/block_align.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ysc',
    maintainer_email='2749456652@qq.com',
    description='Color-block visual alignment to place1 for Lite3',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'block_align_node = block_align.block_align_node:main',
        ],
    },
)
