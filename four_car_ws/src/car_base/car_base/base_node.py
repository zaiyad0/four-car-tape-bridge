#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""base_node —— 底盘节点。

    /carN/cmd_vel  ─► 麦轮逆解算 ─► 轮速(rad/s) ─► ZDT速度模式 ─► 4个电机
                                                         │
    实时转速 ◄───────────────────────────────────────────┘
         └─► 正解算 + IMU 融合 ─► /carN/odom

⚠️ 一个性能约束（读回速度的代价）
--------------------------------
电机是**闭环伺服**，`F6` 速度模式内部自带速度环，所以**不用我们写 PID**。

但要拿真实转速做里程计，就得一个个地址去问（读命令每个地址都会应答）。
4 个电机 ≈ 4 × 9 字节 ≈ 3ms 线上时间 + 每个电机的处理延迟，
实测一轮下来 **10~30ms**。

所以 `odom_hz` **不要超过 20**（周期 50ms）。想要更快只有两条路：
  ① 用 `AA` 多电机命令一条包把 4 个读取请求发出去（手册 5.2.1，但帧长
     字段的算法在 OCR 过的手册里看不清楚，**上硬件实测后再启用**）
  ② 降低读取频率，靠轮速插值

安全：cmd_vel 有超时。多久没收到新指令就自动停车 —— 上位机崩了、
WiFi 断了，车不会一直往前冲。
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

from car_base.mecanum import MecanumKinematics
# 注意：OdometryTracker 是我自己的类，跟 nav_msgs.msg.Odometry **不是一回事**。
# 曾经因为名字重名（都叫 Odometry）踩过坑，见 odometry.py 的类注释。
from car_base.odometry import OdometryTracker


class BaseNode(Node):

    def __init__(self):
        super().__init__('base_controller')

        # ---------------- 参数 ----------------
        d = self.declare_parameter
        d('car_id', 1)
        d('simulate', False)
        # 麦轮几何（米）
        d('base.wheel_radius', 0.0485)      # 麦轮直径 97mm
        d('base.half_length', 0.08006)      # 前后中心距 160.12mm / 2
        d('base.half_width', 0.1385)        # 左右距离 277mm / 2
        # 轮子最大角速度。电机输出轴上限 ≈160 RPM（3000/18.8）→ 16.7 rad/s
        # ≈ 车速 0.81 m/s。这里留点余量取 16.0
        d('base.max_wheel_rad_s', 16.0)
        d('base.flip_vy', False)
        d('base.flip_wz', False)
        # 串口（底盘用，**不是**舵机那个）
        d('base.port', '/dev/ttyUSB0')
        d('base.baudrate', 115200)
        # 电机
        d('base.gear_ratio', 18.8)          # 520 减速电机
        d('base.addr_map', [1, 2, 3, 4])    # FL FR RL RR 对应的电机地址
        # ⚠️ 用**字符串**而不是数组：ROS 2 参数的空数组 [] 推断不出元素类型，
        #    declare_parameter 会直接报错。逗号分隔的字符串没这个问题。
        d('base.invert', '')                # 哪几个轮子要反相，如 "FL,RL"
        d('base.accel_rpm_s', 2000)         # 加速度
        d('base.setup_gear', True)          # 开机自动把驱动器减速比设成 gear_ratio
        # 输出轴(轮子)最大转速。3000 电机RPM / 18.8 ≈ 160 输出RPM ≈ 0.8 m/s
        d('base.max_output_rpm', 160.0)
        d('base.use_current_limit', False)  # 是否用 C6 限流（防堵转）
        d('base.max_current_ma', 3000)
        # 控制环
        d('base.control_hz', 30.0)
        d('base.odom_hz', 20.0)             # ← 别超 20，见文件头说明
        d('base.cmd_timeout', 0.5)

        g = self.get_parameter
        self.simulate = bool(g('simulate').value)
        self.max_wheel = float(g('base.max_wheel_rad_s').value)
        self.cmd_timeout = float(g('base.cmd_timeout').value)
        control_hz = float(g('base.control_hz').value)
        odom_hz = float(g('base.odom_hz').value)

        # ---------------- 运动学 ----------------
        self.kin = MecanumKinematics(
            wheel_radius=float(g('base.wheel_radius').value),
            half_length=float(g('base.half_length').value),
            half_width=float(g('base.half_width').value),
            flip_vy=bool(g('base.flip_vy').value),
            flip_wz=bool(g('base.flip_wz').value))
        self.get_logger().info(str(self.kin))

        self.odo = OdometryTracker(self.kin, use_imu_yaw=True)

        # ---------------- 硬件 ----------------
        self.driver = None
        if not self.simulate:
            from car_base.base_driver import BaseDriver
            from car_base.zdt_driver import ZDTError
            try:
                self.driver = BaseDriver(
                    port=g('base.port').value,
                    baudrate=int(g('base.baudrate').value),
                    addr_map=[int(x) for x in g('base.addr_map').value],
                    gear_ratio=float(g('base.gear_ratio').value),
                    setup_gear=bool(g('base.setup_gear').value),
                    invert=[s.strip().upper() for s in
                            str(g('base.invert').value).split(',') if s.strip()],
                    accel_rpm_s=int(g('base.accel_rpm_s').value),
                    max_output_rpm=float(g('base.max_output_rpm').value),
                    max_current_ma=int(g('base.max_current_ma').value),
                    use_current_limit=bool(g('base.use_current_limit').value),
                ).open()

                # ★ 先校验驱动器里的减速比 —— 设错了车速会差 18.8 倍
                gear = self.driver.check_gear_ratios()
                self.get_logger().info('减速比检查: %s' % gear)
                bad = {k: v for k, v in gear.items() if v != 'ok'}
                if bad:
                    self.get_logger().warn(
                        '有电机的减速比不正常: %s —— 跑一下 '
                        'tools/zdt_id_tool.py gear 18.8' % bad)

                # 使能电机（锁轴）。不使能轴是松的，车推得动但控制不了
                ok = self.driver.enable_all(True)
                self.get_logger().info('电机使能: %s' % ok)

            except ZDTError as exc:
                # 硬件不在/串口打不开时，给一个干净的提示，
                # 不要甩一屏 Python traceback（那样看不出该干什么）
                self.get_logger().fatal('底盘初始化失败：%s' % exc)
                self.get_logger().fatal(
                    '节点退出。想先不接硬件调试的话，加 -p simulate:=true')
                raise SystemExit('底盘初始化失败')

        # ---------------- 状态 ----------------
        self.cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.imu_yaw = None
        self._sim_wheels = (0.0,) * 4
        self._last_odom_t = None
        self.read_ok = True

        # ---------------- 接口 ----------------
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        if not self.simulate:
            self.create_subscription(Imu, 'imu', self.on_imu, 10)
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)

        self.create_timer(1.0 / control_hz, self.control_loop)
        self.create_timer(1.0 / odom_hz, self.odom_loop)

        self.get_logger().info(
            '底盘就绪 | %s | 控制 %.0fHz 里程计 %.0fHz | cmd_vel 超时 %.1fs 自动停车'
            % ('仿真' if self.simulate else '实机 %s' % g('base.port').value,
               control_hz, odom_hz, self.cmd_timeout))

    # ============================================================
    def on_cmd_vel(self, msg):
        self.cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def on_imu(self, msg):
        q = msg.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.imu_yaw = math.atan2(siny, cosy)

    # ============================================================
    def control_loop(self):
        """把 cmd_vel 变成四个轮速发下去。"""
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if age > self.cmd_timeout:
            self.cmd = Twist()          # 安全：超时自动停车

        wheels = self.kin.inverse(self.cmd.linear.x, self.cmd.linear.y,
                                  self.cmd.angular.z)
        wheels = self.kin.clamp(wheels, self.max_wheel)   # 等比例限幅，保方向

        if self.simulate:
            self._sim_wheels = wheels
            return

        try:
            self.driver.set_wheels(*wheels)
        except Exception as exc:
            self.get_logger().error('下发轮速失败: %s' % exc, once=True)

    def odom_loop(self):
        now = self.get_clock().now()
        if self._last_odom_t is None:
            self._last_odom_t = now
            return
        dt = (now - self._last_odom_t).nanoseconds / 1e9
        self._last_odom_t = now

        if self.simulate:
            wheels = self._sim_wheels
        else:
            # ★ 读**真实转速**（电机内部闭环报出来的），不是我们下发的目标值
            try:
                wheels = self.driver.read_wheel_speeds()
            except Exception as exc:
                self.get_logger().error('读轮速失败: %s' % exc, once=True)
                wheels = (None,) * 4

            if any(w is None for w in wheels):
                # 读不到就保持上一次的位姿，不要用 0 冒充 —— 那会让里程计跳变
                if self.read_ok:
                    self.get_logger().warn('有电机读不到转速，里程计暂停更新')
                    self.read_ok = False
                self._publish_odom()
                return
            self.read_ok = True

        self.odo.update(dt, wheels, imu_yaw=self.imu_yaw)
        self._publish_odom()

    def _publish_odom(self):
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'odom'
        m.child_frame_id = 'base_link'
        m.pose.pose.position.x = self.odo.x
        m.pose.pose.position.y = self.odo.y
        half = self.odo.theta / 2.0
        m.pose.pose.orientation.z = math.sin(half)
        m.pose.pose.orientation.w = math.cos(half)
        self.pub_odom.publish(m)

    def destroy_node(self):
        if self.driver:
            self.driver.close()     # 里面会先停车再松轴
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BaseNode()
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
