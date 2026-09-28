#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""里程计：轮编码器 + IMU 融合成位姿。

不依赖 ROS，可以直接 import 单测（见 selftest.py）。

融合策略（简单实用版）
----------------------
- **位置 (x, y)**：用轮子里程计积分。轮子短时间精度够，长时间会因打滑漂。
- **朝向 theta**：优先用 IMU 的 yaw。IMU 短期会漂但不会因打滑跳变，
  而轮子积分碰上一次打滑就永久错了。所以朝向信 IMU。

这是「够用优先」的取舍，不是最优解。真要做 SLAM 那种精度得上
扩展卡尔曼滤波（EKF），但对本项目「走 3 米 + 转 90 度」完全够。
"""

import math


class OdometryTracker:
    """维护一个位姿 (x, y, theta) 和累计路径长度。

    坐标：世界系。初始位置为原点，初始朝向为 +x 方向。

    ⚠️ 为什么叫 OdometryTracker 而不是 Odometry？
       `nav_msgs/msg/Odometry` 是 ROS 的标准消息类型。如果这个类也叫
       `Odometry`，在节点里同时 import 两者时，**后 import 的会遮蔽先 import 的**：

           from nav_msgs.msg import Odometry        # ROS 消息
           from car_base.odometry import Odometry   # ← 把它覆盖了！

       然后 `create_publisher(Odometry, ...)` 拿到的是这个类，报
           AttributeError: type object 'type' has no attribute '_TYPE_SUPPORT'
       而报错信息会误导你去查「ROS 1/2 工作空间 sourcing 顺序」，
       其实是名字冲突。加个 Tracker 后缀一劳永逸。
    """

    def __init__(self, kinematics, use_imu_yaw=True):
        self.k = kinematics
        self.use_imu_yaw = use_imu_yaw

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0          # rad，逆时针为正
        self.path = 0.0           # 累计路径长度 (m)，走歪了也算进去
        self.t = 0.0              # 累计时间 (s)

        # 上一次的轮速，用于一阶保持插值（没有编码器增量时的退路）
        self._last_wheels = (0.0, 0.0, 0.0, 0.0)

    # ---------------- 核心 ----------------
    def update(self, dt, wheels_rad_s, imu_yaw=None):
        """推进一个时间步。

        dt           : 距上次调用的时间 (s)
        wheels_rad_s : 四个轮的**当前角速度** (rad/s)，顺序 (FL, FR, RL, RR)
        imu_yaw      : IMU 的 yaw (rad)。为 None 时退化成纯轮式里程计。

        返回 (x, y, theta)。
        """
        if dt <= 0.0:
            return self.pose

        vx, vy, wz_odom = self.k.forward(*wheels_rad_s)

        # 用当前轮速算本步位移。严格说应该用「上一步和这一步的平均」，
        # 因为编码器给的是区间平均速度时已经是平均值了 —— 这里按
        # 区间速度处理，够用。
        c, s = math.cos(self.theta), math.sin(self.theta)
        dx = (vx * c - vy * s) * dt      # 机体系 → 世界系
        dy = (vx * s + vy * c) * dt

        self.x += dx
        self.y += dy
        self.path += math.hypot(dx, dy)
        self.t += dt

        # 朝向：有 IMU 信 IMU，否则用轮子积分
        if self.use_imu_yaw and imu_yaw is not None:
            self.theta = _wrap(imu_yaw)
        else:
            self.theta = _wrap(self.theta + wz_odom * dt)

        self._last_wheels = tuple(wheels_rad_s)
        return self.pose

    def reset(self, x=0.0, y=0.0, theta=0.0):
        """回到原点。每次任务开始前调一次。"""
        self.x, self.y, self.theta = float(x), float(y), float(theta)
        self.path = 0.0
        self.t = 0.0
        self._last_wheels = (0.0, 0.0, 0.0, 0.0)

    # ---------------- 查询 ----------------
    @property
    def pose(self):
        """(x, y, theta)"""
        return (self.x, self.y, self.theta)

    @property
    def displacement(self):
        """离原点的直线距离 (m) —— 走歪了会比 path 小"""
        return math.hypot(self.x, self.y)

    @property
    def heading_deg(self):
        return math.degrees(self.theta)

    def heading_error_deg(self, target_deg):
        """当前朝向和目标朝向的夹角，范围 (-180, 180]。分正负，便于判断该往哪边转。"""
        return math.degrees(_wrap(self.theta - math.radians(target_deg)))

    # ---------------- 给状态机用的判断 ----------------
    def reached(self, target_x, target_y, tol=0.05):
        """有没有走到某个点（tol 单位 m）"""
        return math.hypot(self.x - target_x, self.y - target_y) <= tol

    def driven(self, meters):
        """累计路径有没有走够 meters（本项目「直行 3 米」用的是这个）"""
        return self.path >= meters

    def headed_to(self, target_deg, tol_deg=2.0):
        """朝向有没有转到目标角度（tol 单位度）"""
        return abs(self.heading_error_deg(target_deg)) <= tol_deg

    def as_dict(self):
        return {
            'x': round(self.x, 4),
            'y': round(self.y, 4),
            'theta_deg': round(self.heading_deg, 2),
            'path': round(self.path, 4),
            't': round(self.t, 3),
        }


def _wrap(a):
    """把角度收敛到 (-pi, pi]。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi
