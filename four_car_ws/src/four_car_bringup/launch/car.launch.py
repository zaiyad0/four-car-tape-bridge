#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单车 launch —— 每台树莓派上跑这一个。

四台车跑**同一套代码**，靠环境变量 CAR_ID 区分自己是几号车：

    export CAR_ID=2
    ros2 launch four_car_bringup car.launch.py

会自动把节点放进 /carN 命名空间，所以 topic 变成 /car2/odom、/car2/status ...

用法:
    CAR_ID=2 ros2 launch four_car_bringup car.launch.py
    CAR_ID=2 ros2 launch four_car_bringup car.launch.py simulate:=true

⚠️ 命名空间是这套架构的关键，别去掉
------------------------------------
    有命名空间：              没命名空间：
      /car1/odom                /odom      ← 两台车都往这发，数据混在一起
      /car2/odom                /odom
      /car1/status              /status    ← 主机分不清是哪台车报的
      /car2/status              /status
      /car1/cmd_vel             /cmd_vel   ← 一个指令两台车都执行
      /car2/cmd_vel             /cmd_vel

    这个 bug 单车测试发现不了，**必须两台车同时跑才会暴露**。
    曾经 sim.launch.py 有 namespace、car.launch.py 漏了，就是靠
    两台真机一起跑才抓到的。

    `/mission/state` 是**绝对话题名**，不受命名空间影响，4 台车都能收到广播。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 车号从环境变量读（4 台车各设一次，代码不用改）
    try:
        car_id = int(os.environ.get('CAR_ID', '1'))
    except ValueError:
        car_id = 1
    namespace = 'car%d' % car_id

    simulate = LaunchConfiguration('simulate')

    pkg_share = get_package_share_directory('four_car_bringup')
    params = os.path.join(pkg_share, 'config', 'params.yaml')

    # 每台车的目标朝向：1、3 右转 +90；2、4 左转 -90（两两相对）
    target_yaw = {1: 90.0, 2: -90.0, 3: 90.0, 4: -90.0}.get(car_id, 90.0)
    # 前两台要先直行一段，后两台一出发就转向
    should_drive = car_id in (1, 2)

    common_params = [params, {
        'car_id': ParameterValue(car_id, value_type=int),
        'simulate': ParameterValue(simulate, value_type=bool),
    }]

    return LaunchDescription([
        DeclareLaunchArgument(
            'simulate', default_value='false',
            description='true = 不用硬件，内部假装在动'),

        # 底盘：串口 <-> 麦轮解算 <-> 里程计
        Node(
            package='car_base',
            executable='base_node',
            name='base_controller',
            namespace=namespace,          # ★ 关键：/carN 命名空间
            output='screen',
            parameters=common_params,
        ),

        # 上层：舵机俯仰 + 卷尺伸出 + 电磁铁
        Node(
            package='car_arm',
            executable='arm_node',
            name='arm_controller',
            namespace=namespace,          # ★
            output='screen',
            parameters=common_params,
        ),

        # 本地状态机
        Node(
            package='car_agent',
            executable='agent_node',
            name='car_agent',
            namespace=namespace,          # ★
            output='screen',
            parameters=[params, {
                'car_id': ParameterValue(car_id, value_type=int),
                'simulate': ParameterValue(simulate, value_type=bool),
                'mission.target_yaw_deg': ParameterValue(target_yaw,
                                                         value_type=float),
                'mission.should_drive': ParameterValue(should_drive,
                                                       value_type=bool),
            }],
        ),
    ])
