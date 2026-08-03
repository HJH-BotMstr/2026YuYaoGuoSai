#!/usr/bin/env python3
"""Launch file for grasp_task node."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('grasp_task')
    params = os.path.join(pkg_share, 'config', 'grasp_task.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'dry_run',
            default_value='false',
            description='true 时跳过真实机械臂和摄像头，仅用于 ROS 通信测试',
        ),
        Node(
            package='grasp_task',
            executable='grasp_node',
            name='grasp_task',
            output='screen',
            parameters=[
                params,
                {'dry_run': LaunchConfiguration('dry_run')},
            ],
        ),
    ])
