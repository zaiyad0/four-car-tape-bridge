#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仿真 launch —— 不需要任何硬件，把整个任务流程跑一遍。

用途：硬件没到之前先验证「编排 + 状态机 + 时序」这套逻辑对不对。
四台车的里程计由 agent_node 内部假装积分产生（simulate:=true），
所以能看到 GO → 各自移动/转向 → ALIGNED → EXTEND → DOCKED 的完整流转。

用法:
    ros2 launch four_car_bringup sim.launch.py
    ros2 launch four_car_bringup sim.launch.py drive_distance_m:=2.0

⚠️ 踩过的坑：parameters 字典里的值**必须包 ParameterValue**，
   直接写裸的 int / bool / float 会报
   "Failed to normalize given item of type '<class 'int'>'"。
   list 类型的参数（如 car_ids）在这个版本上没法内联传，
   所以那些用节点里的默认值，不在这里重复声明。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    dist = LaunchConfiguration('drive_distance_m')
    speed = LaunchConfiguration('drive_speed_m_s')
    delay = LaunchConfiguration('autostart_delay')

    # 每台车的目标航向：1、3 右转 +90，2、4 左转 -90，两两相对
    TARGETS = {1: 90.0, 2: -90.0, 3: 90.0, 4: -90.0}
    # 前两台先直行一段，后两台一出发就转向
    SHOULD_DRIVE = {1: True, 2: True, 3: False, 4: False}

    def car(i):
        return Node(
            package='car_agent',
            executable='agent_node',
            name='car_agent',
            namespace='car%d' % i,
            output='screen',
            parameters=[{
                # ParameterValue 是「校验」类型不是「转换」类型：
                # 静态值要直接给 Python 对象（True 而非 'true'），
                # 只有 LaunchConfiguration 这种替换量才用字符串形式。
                'car_id': ParameterValue(i, value_type=int),
                'simulate': ParameterValue(True, value_type=bool),
                'mission.drive_distance_m': ParameterValue(dist, value_type=float),
                'mission.drive_speed_m_s': ParameterValue(speed, value_type=float),
                'mission.target_yaw_deg': ParameterValue(TARGETS[i],
                                                         value_type=float),
                'mission.should_drive': ParameterValue(SHOULD_DRIVE[i],
                                                       value_type=bool),
            }],
        )

    return LaunchDescription([
        DeclareLaunchArgument('drive_distance_m', default_value='3.0'),
        DeclareLaunchArgument('drive_speed_m_s', default_value='0.25'),
        DeclareLaunchArgument('autostart_delay', default_value='4.0'),

        car(1), car(2), car(3), car(4),

        # car_ids / pairs 用节点默认值 [1,2,3,4] 和 [[1,2],[3,4]]
        Node(
            package='mission_controller',
            executable='mission_node',
            name='mission_controller',
            output='screen',
            parameters=[{
                'autostart': ParameterValue(True, value_type=bool),
                'autostart_delay': ParameterValue(delay, value_type=float),
                'mission.mission_timeout': ParameterValue(30.0, value_type=float),
            }],
        ),
    ])
