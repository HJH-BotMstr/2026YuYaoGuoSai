from setuptools import setup

package_name = 'gauge_yolo_detector'

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
    description='YOLOv8 gauge recognition service node for Lite3',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gauge_yolo_server = gauge_yolo_detector.gauge_yolo_server:main'
        ],
    },
)
