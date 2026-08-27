from setuptools import setup
import os
from glob import glob

package_name = 'gauge_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ysc',
    maintainer_email='2749456652@qq.com',
    description='Gauge recognition service node for Lite3',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gauge_server = gauge_detector.gauge_server:main'
        ],
    },
)
