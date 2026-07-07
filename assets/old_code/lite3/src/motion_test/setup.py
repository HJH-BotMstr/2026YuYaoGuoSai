from setuptools import setup
import os
from glob import glob

package_name = 'motion_test'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YSC Team',
    maintainer_email='ysc@example.com',
    description='Closed-loop motion driver and action server for Lite3 quadruped',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lite3_driver_node = motion_test.lite3_driver_node:main',
            'motion_action_server = motion_test.motion_action_server:main',
        ],
    },
)
