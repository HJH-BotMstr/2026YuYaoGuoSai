from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("letter_place_align")
    params_file = os.path.join(pkg_dir, "config", "letter_place_align.yaml")

    # 位姿控制器：依赖官方 ROS2 栈提供 /leg_odom2 与 /cmd_vel
    # 参数与 apriltag_place1.launch.py 已调好的一组保持一致
    pose_controller_node = Node(
        package="pose_control",
        executable="pose_control",
        name="pose_controller",
        output="screen",
        parameters=[{
            "enable_terminal": False,
            "obstacle_stop_dist": 0.35,
            "command_topic": "/pose_control/command",
            # 机械狗对低速指令响应差，适当提高位置环增益，让小距离也能产生可见速度
            "kp_dist": 2.0,
            "kp_lateral": 2.0,
            # 控制器内部到位阈值必须小于上层对齐节点的阈值，
            # 否则剩余距离在两者之间时控制器会直接停止，导致上层一直发指令却不动
            "dist_threshold": 0.015,
            "yaw_threshold": 0.025,
        }],
    )

    letter_place_node = Node(
        package="letter_place_align",
        executable="letter_place_align_node",
        name="letter_place_align_node",
        parameters=[params_file],
        output="screen",
    )

    return LaunchDescription([pose_controller_node, letter_place_node])
