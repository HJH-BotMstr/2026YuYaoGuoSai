#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键启动抓取全流程（放置对齐已解耦为外部接口版本）。

与 grasp_flow.launch.py 的差异：
  - 编排节点换成 grasp_flow_node_b。
  - 放置阶段不再拉起 letter_place_align：抓取完成后，本节点等待
    /grasp_flow/place_ready (std_msgs/Bool, data=true) 的外部触发，
    或在本终端敲回车作为临时占位触发，随即向 grasp_task 发
    /grasp/place，zone 硬编码为 HARDCODED_LETTER（默认 "B"，改
    grasp_flow/grasp_flow_node_b.py 里的常量即可）。

用法：
  ros2 launch grasp_flow grasp_flow_b.launch.py
  ros2 launch grasp_flow grasp_flow_b.launch.py dry_run:=true
  ros2 launch grasp_flow grasp_flow_b.launch.py start_dog_driver:=false
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    grasp_share = get_package_share_directory('grasp_task')
    grasp_params = os.path.join(grasp_share, 'config', 'grasp_task.yaml')

    start_dog_driver = LaunchConfiguration('start_dog_driver')
    dog_driver_path = LaunchConfiguration('dog_driver_path')
    dry_run = LaunchConfiguration('dry_run')

    dog_driver = ExecuteProcess(
        cmd=['python3', dog_driver_path],
        output='screen',
        condition=IfCondition(start_dog_driver),
    )

    pose_controller = Node(
        package='pose_control',
        executable='pose_control',
        name='pose_controller',
        output='screen',
        parameters=[{
            'enable_terminal': False,
            'show_display': False,
            'obstacle_stop_dist': 0.35,
            'command_topic': '/pose_control/command',
            'kp_dist': 2.0,
            'kp_lateral': 2.0,
            'dist_threshold': 0.015,
            'yaw_threshold': 0.025,
        }],
    )

    grasp_node = Node(
        package='grasp_task',
        executable='grasp_node',
        name='grasp_task',
        output='screen',
        parameters=[
            grasp_params,
            {'dry_run': dry_run},
        ],
    )

    # 硬编码字母版本编排器：不读 stdin，抓取完成后直接用 HARDCODED_LETTER 触发放置
    flow_node = Node(
        package='grasp_flow',
        executable='grasp_flow_node_b',
        name='grasp_flow_node_b',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_dog_driver',
            default_value='true',
            description='是否自动拉起 lite3_driver',
        ),
        DeclareLaunchArgument(
            'dog_driver_path',
            default_value='/home/ysc/2026YuYaoGuoSai/tools/lite3_driver.py',
            description='lite3_driver.py 路径',
        ),
        DeclareLaunchArgument(
            'dry_run',
            default_value='false',
            description='true 时 grasp_task 跳过真实机械臂/摄像头',
        ),
        dog_driver,
        pose_controller,
        grasp_node,
        flow_node,
    ])
