from setuptools import setup

package_name = 'pose_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pose_control.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ysc',
    maintainer_email='2749456652@qq.com',
    description='Position and yaw closed-loop controller for Lite3 with ultrasonic obstacle avoidance',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_control = pose_control.pose_controller_node:main',
            'start_pose_control = pose_control.start_pose_control:main',
        ],
    },
)
