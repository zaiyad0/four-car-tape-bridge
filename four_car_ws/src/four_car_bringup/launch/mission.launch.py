#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""编排层 launch —— 只在一台车（或上位机）上跑。

    ROS_DOMAIN_ID=0 ros2 launch four_car_bringup mission.launch.py

autostart:=false 时不会自动发 GO，改由你手动触发：

    ros2 topic pub --once /mission/state std_msgs/msg/String "{data: 'GO'}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('four_car_bringup')
    params = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='true',
                              description='启动后是否自动广播 GO'),
        DeclareLaunchArgument('autostart_delay', default_value='5.0',
                              description='等几秒再发 GO（留给各车启动的时间）'),

        Node(
            package='mission_controller',
            executable='mission_node',
            name='mission_controller',
            output='screen',
            parameters=[params, {
                'autostart': ParameterValue(LaunchConfiguration('autostart'),
                                            value_type=bool),
                'autostart_delay': ParameterValue(LaunchConfiguration('autostart_delay'),
                                                  value_type=float),
            }],
        ),
    ])
