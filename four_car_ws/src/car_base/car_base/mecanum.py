#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""麦轮运动学：正解算 / 逆解算。

不依赖 ROS，可以直接 import 单测（见工作空间根目录的 selftest.py）。

符号约定（俯视，车头朝上）

        FL ●────● FR          vx : 前进为正   (m/s)
           │    │             vy : 左移为正   (m/s)
           │    │             wz : 逆时针为正 (rad/s)
        RL ●────● RR          r  : 轮半径     (m)
                              L  : 半长 + 半宽 = Lx + Ly (m)

⚠️ 符号跟辊子朝向有关。上面是「45° 辊子、X 型布局」的标准式，
   不同厂家的辊子方向可能相反，``vy`` / ``wz`` 的符号要对实物标定。
   标定方法：只发一种运动，看车实际往哪走，不对就把对应 flip_* 打开。
       前进 vx>0          → 应往车头方向走
       横移 vy>0          → 应往左平移
       自转 wz>0          → 应逆时针转
"""

import math


class MecanumKinematics:
    """四轮麦轮底盘的运动学。

    参数
    ----
    wheel_radius : 轮半径 r (m)
    half_length  : 轮轴到车中心的纵向距离 Lx (m)
    half_width   : 轮轴到车中心的横向距离 Ly (m)
    flip_vy      : 实测横移方向相反时置 True
    flip_wz      : 实测自转方向相反时置 True
    """

    #: 轮序，和 inverse() / forward() 的返回值一一对应
    WHEEL_NAMES = ('FL', 'FR', 'RL', 'RR')

    def __init__(self, wheel_radius, half_length, half_width,
                 flip_vy=False, flip_wz=False):
        if wheel_radius <= 0:
            raise ValueError('wheel_radius 必须为正')
        self.r = float(wheel_radius)
        self.Lx = float(half_length)
        self.Ly = float(half_width)
        self.L = self.Lx + self.Ly
        self.flip_vy = bool(flip_vy)
        self.flip_wz = bool(flip_wz)

    # ---------------- 解算 ----------------
    def inverse(self, vx, vy=0.0, wz=0.0):
        """(vx, vy, wz) → 四个轮的角速度 (rad/s)，顺序 (FL, FR, RL, RR)。

        这是底盘层的核心：把 ROS 传来的 Twist 变成四个轮子该转多快。
        """
        if self.flip_vy:
            vy = -vy
        if self.flip_wz:
            wz = -wz
        L, r = self.L, self.r
        return (
            (vx - vy - wz * L) / r,   # FL
            (vx + vy + wz * L) / r,   # FR
            (vx + vy - wz * L) / r,   # RL
            (vx - vy + wz * L) / r,   # RR
        )

    def forward(self, w_fl, w_fr, w_rl, w_rr):
        """四个轮角速度 (rad/s) → (vx, vy, wz)。里程计用这个。"""
        r, L = self.r, self.L
        vx = r / 4.0 * (w_fl + w_fr + w_rl + w_rr)
        vy = r / 4.0 * (-w_fl + w_fr + w_rl - w_rr)
        wz = r / (4.0 * L) * (-w_fl + w_fr - w_rl + w_rr)
        if self.flip_vy:
            vy = -vy
        if self.flip_wz:
            wz = -wz
        return vx, vy, wz

    # ---------------- 辅助 ----------------
    def clamp(self, wheels, max_rad_s):
        """等比例缩放四个轮速，保证不超电机最大转速。

        用「等比例」而不是直接截断，是为了**保持运动方向不变** ——
        直接截断会改变 vx/vy/wz 的比例，车就走歪了。
        """
        peak = max(abs(w) for w in wheels)
        if peak <= max_rad_s or peak == 0.0:
            return tuple(wheels)
        k = max_rad_s / peak
        return tuple(w * k for w in wheels)

    @staticmethod
    def rad_s_to_rpm(w):
        return w * 60.0 / (2.0 * math.pi)

    @staticmethod
    def rpm_to_rad_s(rpm):
        return rpm * 2.0 * math.pi / 60.0

    def __repr__(self):
        return ('MecanumKinematics(r=%.4f, Lx=%.4f, Ly=%.4f, L=%.4f, '
                'flip_vy=%s, flip_wz=%s)'
                % (self.r, self.Lx, self.Ly, self.L,
                   self.flip_vy, self.flip_wz))
