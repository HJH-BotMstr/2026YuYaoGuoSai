from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("apriltag_place1")
    params_file = os.path.join(pkg_dir, "config", "apriltag_place1.yaml")

    apriltag_node = Node(
        package="apriltag_place1",
        executable="apriltag_place1_node",
        name="apriltag_place1_node",
        parameters=[params_file],
        output="screen",
    )

    return LaunchDescription([apriltag_node])
