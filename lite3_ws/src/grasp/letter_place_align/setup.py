from setuptools import setup

package_name = 'letter_place_align'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/letter_place_align.launch.py']),
        ('share/' + package_name + '/config', ['config/letter_place_align.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ysc',
    maintainer_email='2749456652@qq.com',
    description='A4 letter visual alignment to place zone for Lite3',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'letter_place_align_node = letter_place_align.letter_place_align_node:main',
        ],
    },
)
