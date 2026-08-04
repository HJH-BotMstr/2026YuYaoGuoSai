from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("block_align")
    params_file = os.path.join(pkg_dir, "config", "block_align.yaml")

    pose_controller_node = Node(
        package="pose_control",
        executable="pose_control",
        name="pose_controller",
        output="screen",
        parameters=[{
            "enable_terminal": False,
            "obstacle_stop_dist": 0.35,
            "command_topic": "/pose_control/command",
            "kp_dist": 2.0,
            "kp_lateral": 2.0,
            "dist_threshold": 0.015,
            "yaw_threshold": 0.025,
        }],
    )

    block_align_node = Node(
        package="block_align",
        executable="block_align_node",
        name="block_align_node",
        parameters=[params_file],
        output="screen",
    )

    return LaunchDescription([pose_controller_node, block_align_node])
