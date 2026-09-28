#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arm_node —— 上层：舵机俯仰 + 卷尺伸出 + 电磁铁。

硬件
----
    俯仰舵机   总线舵机（ZDT 那套是底盘电机，**别搞混**）
               → /dev/ttyAMA0（GPIO14/15，物理脚 8/10）
               开机设一个固定角度就完事，之后不用管

    卷尺电机   JGY-370（24 RPM）+ L298N
               ENA → GPIO  PWM 调速
               IN1/IN2 → GPIO  方向
               ★ 控制方式就是「通电 N 秒」—— 唯一要实测的参数

    电磁铁     GPIO → 驱动模块 → 电磁铁
               伸出时间到了就通电吸合

伸出流程（简化版，没有霍尔、没有编码器）
--------------------------------------
    ① 卷尺电机正转（PWM 全速）
    ② 通电 extend_seconds 秒   ← ★ 你实测调的就是这个
    ③ 停电机
    ④ 电磁铁通电
    ⑤ 等 magnet_settle 秒确认吸稳 → DOCKED

    实测调参步骤：
      1. 先把 extend_seconds 设小一点（比如 3 秒），跑一次看伸出来多少
      2. 一点点加（每次 +1 秒），直到卷尺刚好伸到位
      3. 记下这个值，写进 params.yaml
      4. 注意电池电量会影响转速 —— 调试和实跑尽量保持电量差不多

话题
----
    订阅  arm_cmd    std_msgs/String   EXTEND / RETRACT / PITCH / DOCK / RELEASE
    发布  arm_state  std_msgs/String   IDLE / EXTENDING / DOCKED / ERROR
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from car_arm.l298n import L298N, L298NError
from car_arm.magnet import Magnet, MagnetError
from car_arm.servo_driver import BusServo, ServoError
from car_arm.tape_driver import TapeDriver

IDLE = 'IDLE'
EXTENDING = 'EXTENDING'
DOCKING = 'DOCKING'
DOCKED = 'DOCKED'
ERROR = 'ERROR'


class ArmNode(Node):

    def __init__(self):
        super().__init__('arm_controller')

        # ---------------- 参数 ----------------
        d = self.declare_parameter
        d('simulate', False)

        # 俯仰舵机（跟底盘电机是**两回事**，串口也不同）
        d('arm.servo_port', '/dev/ttyAMA0')
        d('arm.servo_baudrate', 9600)
        d('arm.servo_id', 1)
        d('arm.servo_pitch_deg', 0.0)

        # 卷尺电机（L298N）
        d('arm.ena_pin', 12)              # PWM 调速
        d('arm.in1_pin', 23)              # 方向
        d('arm.in2_pin', 24)              # 方向
        d('arm.pwm_freq', 1000)
        d('arm.extend_seconds', 8.0)      # ★ 唯一要实测的参数
        d('arm.retract_seconds', 8.0)
        d('arm.extend_duty', 1.0)         # 伸出占空比，想慢点就降到 0.7
        d('arm.retract_duty', 1.0)

        # 电磁铁
        d('arm.magnet_pin', 17)
        d('arm.magnet_active_high', True)
        d('arm.magnet_settle', 1.0)       # 通电后等多久算吸稳

        g = self.get_parameter
        self.simulate = bool(g('simulate').value)
        self.pitch_deg = float(g('arm.servo_pitch_deg').value)
        self.magnet_settle = float(g('arm.magnet_settle').value)
        self.extend_seconds = float(g('arm.extend_seconds').value)

        # ---------------- 硬件 ----------------
        self.servo = None
        self.motor = None
        self.tape = None
        self.magnet = None

        # 电磁铁：即使别的初始化失败也尽量建起来（安全相关）
        try:
            self.magnet = Magnet(
                pin=int(g('arm.magnet_pin').value),
                active_high=bool(g('arm.magnet_active_high').value),
                simulate=self.simulate)
        except MagnetError as exc:
            self.get_logger().error('电磁铁初始化失败: %s' % exc)

        if not self.simulate:
            self._init_servo(g)
            self._init_tape(g)

        # ---------------- 状态 ----------------
        self.state = IDLE
        self._docking_since = None

        # ---------------- 接口 ----------------
        self.create_subscription(String, 'arm_cmd', self.on_cmd, 10)
        self.pub_state = self.create_publisher(String, 'arm_state', 10)
        self.create_timer(0.1, self.tick)

        self._set_pitch()

        self.get_logger().info(
            '上层就绪 | 俯仰 %+.1f° | 伸出时间 %.1fs | %s'
            % (self.pitch_deg, self.extend_seconds,
               '【仿真】' if self.simulate else '实机'))

    # ============================================================
    def _init_servo(self, g):
        try:
            self.servo = BusServo(port=g('arm.servo_port').value,
                                  baudrate=int(g('arm.servo_baudrate').value))
        except ServoError as exc:
            self.get_logger().error('舵机串口打不开: %s' % exc)
            self.get_logger().error(
                '检查：serial-getty 是否占着 /dev/ttyAMA0；用户在不在 dialout 组')

    def _init_tape(self, g):
        try:
            self.motor = L298N(
                ena_pin=int(g('arm.ena_pin').value),
                in1_pin=int(g('arm.in1_pin').value),
                in2_pin=int(g('arm.in2_pin').value),
                pwm_freq=int(g('arm.pwm_freq').value))
            self.tape = TapeDriver(
                self.motor,
                extend_seconds=float(g('arm.extend_seconds').value),
                retract_seconds=float(g('arm.retract_seconds').value),
                extend_duty=float(g('arm.extend_duty').value),
                retract_duty=float(g('arm.retract_duty').value))
        except L298NError as exc:
            self.get_logger().error('卷尺电机初始化失败: %s' % exc)

    # ============================================================
    def _set_pitch(self):
        """舵机只需开机设一次固定俯仰角。"""
        if self.servo is None:
            return
        try:
            self.servo.move_angle(int(self.get_parameter('arm.servo_id').value),
                                  self.pitch_deg, duration_ms=1500)
            self.get_logger().info('俯仰角设为 %+.1f°' % self.pitch_deg)
        except Exception as exc:
            self.get_logger().error('设置俯仰角失败: %s' % exc)

    # ============================================================
    def on_cmd(self, msg):
        cmd = msg.data.strip().upper()
        self.get_logger().info('上层收到: %s' % cmd)

        if cmd == 'EXTEND' and self.state in (IDLE, DOCKED):
            self._start_extend()
        elif cmd == 'RETRACT':
            self._retract()
        elif cmd == 'PITCH':
            self._set_pitch()
        elif cmd == 'DOCK':
            self._energize()
        elif cmd == 'RELEASE':
            if self.magnet:
                self.magnet.off()
            self._enter(IDLE)

    # ============================================================
    def _start_extend(self):
        """① 电机正转，开始计时。"""
        if self.simulate:
            self._enter(EXTENDING)
            return
        if self.tape is None:
            self.get_logger().error('卷尺驱动不可用，无法伸出')
            self._enter(ERROR)
            return
        secs = self.tape.start_extend()
        self.get_logger().info('卷尺伸出中，通电 %.1f 秒...' % secs)
        self._enter(EXTENDING)

    def _stop_and_dock(self):
        """② 时间到：停电机，磁铁通电。"""
        if self.tape:
            self.tape.stop()
        self.get_logger().info('伸出时间到，电机停')
        self._energize()

    def _energize(self):
        if self.magnet:
            self.magnet.on()
            self.get_logger().info('电磁铁通电，等 %.1fs 吸稳' % self.magnet_settle)
        self._docking_since = self.get_clock().now()
        self._enter(DOCKING)

    def _retract(self):
        if self.tape:
            self.tape.start_retract()
            self.get_logger().info('卷尺收回中，通电 %.1f 秒'
                                   % self.tape.retract_seconds)
            self._enter(EXTENDING)      # 复用这个状态表示"电机在转"
        else:
            self._enter(IDLE)
        if self.magnet:
            self.magnet.off()

    # ============================================================
    def tick(self):
        """心跳：查时间到没到。"""
        if self.state == EXTENDING:
            if self.simulate:
                self._stop_and_dock()
                return
            if self.tape and self.tape.finished():
                self._stop_and_dock()

        elif self.state == DOCKING:
            elapsed = (self.get_clock().now()
                       - self._docking_since).nanoseconds / 1e9
            if elapsed >= self.magnet_settle:
                self._enter(DOCKED)

    # ============================================================
    def _enter(self, s):
        if s == self.state:
            return
        self.get_logger().info('%s -> %s' % (self.state, s))
        self.state = s
        m = String()
        m.data = s
        self.pub_state.publish(m)

    def destroy_node(self):
        for dev in (self.tape, self.motor, self.magnet, self.servo):
            if dev is not None:
                try:
                    dev.close()
                except Exception:
                    pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArmNode()
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
