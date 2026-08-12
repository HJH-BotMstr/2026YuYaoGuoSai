from setuptools import setup
from glob import glob

package_name = 'abcd_task'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='ysc',
    maintainer_email='2749456652@qq.com',
    description='ABCD four-block pick-and-place orchestrator.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'abcd_task_node = abcd_task.abcd_task_node:main',
        ],
    },
)
