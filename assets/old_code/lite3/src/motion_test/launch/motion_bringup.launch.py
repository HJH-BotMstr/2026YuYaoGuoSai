from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('motion_test'),
        'config',
        'motion_params.yaml'
    ])

    return LaunchDescription([
        Node(
            package='motion_test',
            executable='lite3_driver_node',
            name='lite3_driver_node',
            output='screen',
            parameters=[config_file],
        ),
        Node(
            package='motion_test',
            executable='motion_action_server',
            name='motion_action_server',
            output='screen',
            parameters=[config_file],
        ),
    ])
