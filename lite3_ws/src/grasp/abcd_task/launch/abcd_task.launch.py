#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键启动 ABCD 四轮抓取放置全流程。

启动组成：
  1. lite3_driver（ExecuteProcess，可通过 start_dog_driver 关闭）
  2. pose_controller（导航底层，与 grasp_flow_b 使用完全相同的 gain）
  3. grasp_task/grasp_node（机械臂 8-phase，max_rounds=4 支持四轮循环）
  4. apriltag_place1_node（Tag 视觉对齐，常驻，abcd_task_node 会按字母切 target_tag_id）
  5. abcd_task_node（顶层编排）

block_align 不在 launch 里 —— 由 abcd_task_node 每轮子进程拉起，抓取完成即
kill，避免 latched /grasp/start 污染下一轮。

用法：
  ros2 launch abcd_task abcd_task.launch.py
  ros2 launch abcd_task abcd_task.launch.py dry_run:=true dry_run_nav:=true
  ros2 launch abcd_task abcd_task.launch.py start_from:=C
  ros2 launch abcd_task abcd_task.launch.py start_dog_driver:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    abcd_share = get_package_share_directory("abcd_task")
    grasp_share = get_package_share_directory("grasp_task")
    apriltag_share = get_package_share_directory("apriltag_place1")

    abcd_params    = os.path.join(abcd_share, "config", "abcd_task.yaml")
    grasp_params   = os.path.join(grasp_share, "config", "grasp_task.yaml")
    apriltag_params = os.path.join(apriltag_share, "config", "apriltag_place1.yaml")

    start_dog_driver = LaunchConfiguration("start_dog_driver")
    dog_driver_path  = LaunchConfiguration("dog_driver_path")
    dry_run          = LaunchConfiguration("dry_run")
    dry_run_nav      = LaunchConfiguration("dry_run_nav")
    start_from       = LaunchConfiguration("start_from")
    max_rounds       = LaunchConfiguration("max_rounds")
    skip_on_error    = LaunchConfiguration("skip_on_error")

    dog_driver = ExecuteProcess(
        cmd=["python3", dog_driver_path],
        output="screen",
        condition=IfCondition(start_dog_driver),
    )

    pose_controller = Node(
        package="pose_control",
        executable="pose_control",
        name="pose_controller",
        output="screen",
        parameters=[{
            "enable_terminal":    False,
            "show_display":       False,
            "obstacle_stop_dist": 0.35,
            "command_topic":      "/pose_control/command",
            "kp_dist":            2.0,
            "kp_lateral":         2.0,
            "dist_threshold":     0.015,
            "yaw_threshold":      0.025,
        }],
    )

    grasp_node = Node(
        package="grasp_task",
        executable="grasp_node",
        name="grasp_task",
        output="screen",
        parameters=[
            grasp_params,
            {
                "dry_run":     dry_run,
                "max_rounds":  max_rounds,
                "inter_round_wait_s": 0.5,
            },
        ],
    )

    apriltag_node = Node(
        package="apriltag_place1",
        executable="apriltag_place1_node",
        name="apriltag_place1_node",
        output="screen",
        parameters=[apriltag_params],
    )

    abcd_node = Node(
        package="abcd_task",
        executable="abcd_task_node",
        name="abcd_task_node",
        output="screen",
        parameters=[
            abcd_params,
            {
                "start_from":    start_from,
                "dry_run_nav":   dry_run_nav,
                "skip_on_error": skip_on_error,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_dog_driver",
            default_value="true",
            description="是否自动拉起 lite3_driver.py",
        ),
        DeclareLaunchArgument(
            "dog_driver_path",
            default_value="/home/ysc/2026YuYaoGuoSai/tools/lite3_driver.py",
            description="lite3_driver.py 路径",
        ),
        DeclareLaunchArgument(
            "dry_run",
            default_value="false",
            description="true 时 grasp_task 跳过真实机械臂/摄像头",
        ),
        DeclareLaunchArgument(
            "dry_run_nav",
            default_value="false",
            description="true 时 abcd_task 只跑导航链路，跳过 tag/grasp/place",
        ),
        DeclareLaunchArgument(
            "start_from",
            default_value="A",
            description="从哪个字母开始（A/B/C/D），便于单轮/断点测试",
        ),
        DeclareLaunchArgument(
            "max_rounds",
            default_value="4",
            description="grasp_task 循环轮次；ABCD 全流程用 4，单字母测试用 1",
        ),
        DeclareLaunchArgument(
            "skip_on_error",
            default_value="false",
            description="单字母失败是否继续下一个",
        ),
        dog_driver,
        pose_controller,
        grasp_node,
        apriltag_node,
        abcd_node,
    ])
