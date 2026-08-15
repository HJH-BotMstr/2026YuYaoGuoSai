#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键启动抓取全流程。

包含：
  1. lite3_driver      机械狗驱动，启动即自动执行唤醒序列并进入自主模式
  2. pose_controller   /move → /cmd_vel 位置环（全流程唯一实例）
  3. grasp_task        机械臂抓取/放置状态机（启动后自动进入准备姿态 STANDBY）
  4. grasp_flow_node   全流程编排器

block_align（抓取对齐）与 letter_place_align（放置对齐）两个节点
不在此处启动——它们由 grasp_flow_node 按需拉起/关闭，保证摄像头与
/move 指令总线任意时刻只有一个对齐节点占用。

用法：
  ros2 launch grasp_flow grasp_flow.launch.py                 # 真机全流程
  ros2 launch grasp_flow grasp_flow.launch.py dry_run:=true   # 无机械臂通信测试
  ros2 launch grasp_flow grasp_flow.launch.py start_dog_driver:=false  # 狗驱动外部启动
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

    # 1. 机械狗驱动：启动即自动执行 回零→站立→运动模式→自主模式 唤醒序列，
    #    并发布 /leg_odom2、订阅 /cmd_vel
    dog_driver = ExecuteProcess(
        cmd=['python3', dog_driver_path],
        output='screen',
        condition=IfCondition(start_dog_driver),
    )

    # 2. 位置环控制器（参数沿用 apriltag_place1 launch 中调好的一组）
    pose_controller = Node(
        package='pose_control',
        executable='pose_control',
        name='pose_controller',
        output='screen',
        parameters=[{
            'enable_terminal': False,
            'show_display': True,    # 调试期打开：10Hz 打印 state/cmd/source，观察 moving_x 时的 vx
            'obstacle_stop_dist': 0.35,
            'command_topic': '/pose_control/command',
            # 机械狗对低速指令响应差，适当提高位置环增益
            'kp_dist': 2.0,
            'kp_lateral': 2.0,
            # 控制器到位阈值必须小于上层对齐节点阈值，否则上层发指令狗不动
            'dist_threshold': 0.015,
            'yaw_threshold': 0.025,
        }],
    )

    # 3. 机械臂抓取/放置状态机：启动后自动 set_pose(0)→set_pose(2) 进入准备姿态
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

    # 4. 全流程编排器
    flow_node = Node(
        package='grasp_flow',
        executable='grasp_flow_node',
        name='grasp_flow_node',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_dog_driver',
            default_value='true',
            description='是否自动拉起 lite3_driver（启动即唤醒狗并进入自主模式）；'
                        '若狗驱动由外部启动则设 false',
        ),
        DeclareLaunchArgument(
            'dog_driver_path',
            default_value='/home/ysc/2026YuYaoGuoSai/tools/lite3_driver.py',
            description='lite3_driver.py 路径（start_dog_driver=true 时生效）',
        ),
        DeclareLaunchArgument(
            'dry_run',
            default_value='false',
            description='true 时 grasp_task 跳过真实机械臂和摄像头，仅用于通信测试',
        ),
        dog_driver,
        pose_controller,
        grasp_node,
        flow_node,
    ])
