#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""car_agent —— 单车的本地状态机。

这是每台车自己跑的那一半。它只关心两件事：
    1. 我自己的动作做完了没？（靠编码器里程 / IMU 航向，本地闭环）
    2. 编排层让我进下一步了吗？（靠 /mission/state 广播）

它**不关心**别的车在干什么 —— 那是 mission_controller 的事。
这个分工是整个架构的关键：加第 5 台车不用改这里的逻辑。

状态流转
--------
    READY ──收到 GO──► MOVING ──里程达标──► TURNING ──航向达标──► ALIGNED
                                                                            │
                                      收到 EXTEND ◄────────────────────────┘
                                            │
                                            ▼
                                       EXTENDING ──霍尔触发+吸合──► DOCKED

话题
----
订阅  /mission/state   std_msgs/String    编排层广播
订阅  odom             nav_msgs/Odometry  自己的里程计
发布  status           std_msgs/String    状态上报（**周期性重发**，不能只发一次）
发布  cmd_vel          geometry_msgs/Twist
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


# 状态常量
READY = 'READY'
MOVING = 'MOVING'
TURNING = 'TURNING'
ALIGNED = 'ALIGNED'
EXTENDING = 'EXTENDING'
DOCKED = 'DOCKED'
ERROR = 'ERROR'


def wrap_deg(d):
    """收敛到 (-180, 180]"""
    return (d + 180.0) % 360.0 - 180.0


class CarAgent(Node):

    def __init__(self):
        super().__init__('car_agent')

        # ---------------- 参数 ----------------
        self.declare_parameter('car_id', 1)
        self.declare_parameter('simulate', False)      # True = 不用硬件，内部假装在动
        self.declare_parameter('status_period', 1.0)
        self.declare_parameter('tick_hz', 20.0)

        self.declare_parameter('mission.drive_distance_m', 3.0)
        self.declare_parameter('mission.drive_speed_m_s', 0.25)
        self.declare_parameter('mission.turn_speed_rad_s', 0.5)
        self.declare_parameter('mission.yaw_tolerance_deg', 2.0)
        self.declare_parameter('mission.settle_time', 0.5)
        self.declare_parameter('mission.target_yaw_deg', 90.0)
        # 本车要不要先直行一段？前两台(1,2)要，后两台(3,4)一出发就转
        self.declare_parameter('mission.should_drive', True)

        g = self.get_parameter
        self.car_id = int(g('car_id').value)
        self.simulate = bool(g('simulate').value)
        self.status_period = float(g('status_period').value)
        tick_hz = float(g('tick_hz').value)

        self.drive_distance = float(g('mission.drive_distance_m').value)
        self.drive_speed = float(g('mission.drive_speed_m_s').value)
        self.turn_speed = float(g('mission.turn_speed_rad_s').value)
        self.yaw_tol = float(g('mission.yaw_tolerance_deg').value)
        self.settle_time = float(g('mission.settle_time').value)
        self.target_yaw = float(g('mission.target_yaw_deg').value)
        self.should_drive = bool(g('mission.should_drive').value)

        # ---------------- 状态 ----------------
        self.state = READY
        self.odom = {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'path': 0.0}
        self.state_entered = self.get_clock().now()
        self.last_cmd = Twist()

        # ---------------- 接口 ----------------
        self.create_subscription(String, '/mission/state', self.on_mission, 10)
        if not self.simulate:
            self.create_subscription(Odometry, 'odom', self.on_odom, 10)

        self.pub_status = self.create_publisher(String, 'status', 10)
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_timer(1.0 / tick_hz, self.tick)
        self.create_timer(self.status_period, self.republish_status)

        self.get_logger().info(
            'car%d 就绪 | 目标航向 %+.0f° | %s | %s'
            % (self.car_id, self.target_yaw,
               '先直行 %.2fm' % self.drive_distance if self.should_drive
               else '原地转向（不直行）',
               '【仿真模式】' if self.simulate else '实机'))

    # ============================================================
    #  输入
    # ============================================================
    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # 只取 yaw，四元数绕 z 的分量
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.odom['x'] = p.x
        self.odom['y'] = p.y
        self.odom['yaw'] = math.atan2(siny, cosy)

    def on_mission(self, msg):
        cmd = msg.data.strip().upper()
        self.get_logger().info('收到编排层指令: %s' % cmd)

        if cmd == 'GO' and self.state == READY:
            # 前两台（should_drive=True）先直行一段；后两台一出发就转向
            self._enter(MOVING if self.should_drive else TURNING)
        elif cmd == 'EXTEND' and self.state == ALIGNED:
            self._enter(EXTENDING)
        elif cmd == 'ABORT':
            self._enter(ERROR)
        else:
            # 重复收到同一条指令是正常的（编排层可能重发），静默忽略
            pass

    # ============================================================
    #  主循环
    # ============================================================
    def tick(self):
        dt = 1.0 / 20.0
        if self.simulate:
            self._sim_step(dt)

        handler = {
            MOVING: self._tick_moving,
            TURNING: self._tick_turning,
            EXTENDING: self._tick_extending,
        }.get(self.state)
        if handler:
            handler()

        self.pub_cmd.publish(self.last_cmd)

    # ---------------- MOVING：直行到里程达标 ----------------
    def _tick_moving(self):
        if self.odom['path'] >= self.drive_distance:
            self._stop()
            self._enter(TURNING)
            return
        cmd = Twist()
        cmd.linear.x = self.drive_speed
        self.last_cmd = cmd

    # ---------------- TURNING：转到目标航向 ----------------
    def _tick_turning(self):
        err = wrap_deg(self.target_yaw - math.degrees(self.odom['yaw']))

        if abs(err) <= self.yaw_tol:
            self._stop()
            self._enter(ALIGNED)
            return

        cmd = Twist()
        # 快到了就减速，避免冲过头来回振荡
        scale = min(1.0, abs(err) / 15.0)
        cmd.angular.z = math.copysign(self.turn_speed * scale, err)
        self.last_cmd = cmd

    # ---------------- EXTENDING：伸出卷尺 + 吸合 ----------------
    def _tick_extending(self):
        # 这里调 arm 层的接口（TapeDriver / Magnet）。
        # 真实流程见 tape_driver.py：
        #   ① extend_mm(短一点)  ② approach() 低速蹭
        #   ③ 轮询 hall.triggered → stop()  ④ magnet.on() → DOCKED
        #
        # TODO(物料到位后): 接上 arm_controller，这里改成真正的调用
        self._stop()
        self._enter(DOCKED)

    # ============================================================
    #  状态切换 / 上报
    # ============================================================
    def _enter(self, new_state):
        if new_state == self.state:
            return
        self.get_logger().info('%s -> %s' % (self.state, new_state))
        self.state = new_state
        self.state_entered = self.get_clock().now()
        self.publish_status()

    def publish_status(self):
        m = String()
        m.data = self.state
        self.pub_status.publish(m)

    def republish_status(self):
        """周期性重发。

        ⚠️ 这是必须的，不是冗余。WiFi 丢一个包，编排层就会永远等不齐
            → 整个任务卡死。每秒重发一次，丢包自愈。
        """
        self.publish_status()

    def _stop(self):
        self.last_cmd = Twist()

    # ============================================================
    #  仿真：没有硬件时让状态机能跑完整个流程
    # ============================================================
    def _sim_step(self, dt):
        v = self.last_cmd.linear.x
        w = self.last_cmd.angular.z
        self.odom['yaw'] += w * dt
        self.odom['x'] += v * math.cos(self.odom['yaw']) * dt
        self.odom['y'] += v * math.sin(self.odom['yaw']) * dt
        self.odom['path'] += abs(v) * dt


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CarAgent()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
