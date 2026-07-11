from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pose_control',
            executable='pose_control',
            name='pose_controller',
            output='screen',
            parameters=[{
                'enable_terminal': True,
                'obstacle_stop_dist': 0.35,
            }],
        ),
    ])
