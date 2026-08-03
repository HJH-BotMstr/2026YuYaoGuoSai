from setuptools import setup
import os
from glob import glob

package_name = 'apriltag_place1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/apriltag_place1.launch.py']),
        ('share/' + package_name + '/config', ['config/apriltag_place1.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ysc',
    maintainer_email='2749456652@qq.com',
    description='AprilTag visual alignment to place1 for Lite3',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_place1_node = apriltag_place1.apriltag_place1_node:main',
        ],
    },
)
