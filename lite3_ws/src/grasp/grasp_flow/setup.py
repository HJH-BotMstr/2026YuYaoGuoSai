import os
from glob import glob
from setuptools import setup

package_name = 'grasp_flow'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ysc',
    maintainer_email='ysc@todo.todo',
    description='抓取全流程编排节点与一键 launch',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grasp_flow_node = grasp_flow.grasp_flow_node:main',
            'grasp_flow_node_b = grasp_flow.grasp_flow_node_b:main',
        ],
    },
)
