#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""卷尺伸出机构 —— 按**时间**控制（不用编码器、不用霍尔、不用标定表）。

硬件
----
    JGY-370 减速电机（深圳新永泰，带编码器但**本项目不用**）
      输出轴 24 RPM → 转一圈 2.5 秒
    L298N 双 H 桥模块的一路驱动
      正转 = 伸出卷尺，反转 = 收回

控制方式：**通电 N 秒就停**
--------------------------
    N 是你实测出来的：从短开始，一点点加，看到卷尺伸到位就记下这个值。
    JGY-370 是 24 RPM，所以可以先用 2.5 秒/圈 估个数：
        转动 N 秒 ≈ N / 2.5 圈
        例：要转 10 圈 → 约 25 秒

    时间会随电池电压、卷尺松紧、机构阻力变化，所以：
    · 每次跑之前先确认电池电量差不多
    · 留一点余量，别卡在"刚好伸出来"的临界值
    · 需要更准的话，以后加编码器或霍尔，见文件末尾的说明

为什么不用编码器
----------------
JGY-370 自带的编码器是**电机轴**的，接到树莓派要占 GPIO 中断或者额外单片机。
原型阶段没必要 —— 先用时间跑通，量出「到位需要几秒」，
等整套流程稳定了再考虑要不要闭环。
"""

import time


class TapeDriver:
    """卷尺机构控制。

    用法（在 ROS 节点里）::

        tape = TapeDriver(motor, extend_seconds=8.0)
        tape.start_extend()
        ...
        if tape.finished():        # 每周期查一次
            tape.stop()
            magnet.on()
    """

    def __init__(self, motor, extend_seconds=8.0, retract_seconds=8.0,
                 extend_duty=1.0, retract_duty=1.0):
        """
        motor           L298N 实例
        extend_seconds  ★ 伸出要通电多少秒 —— 唯一需要你实测的参数
        retract_seconds 收回要通电多少秒（一般比伸出略长，收到底靠堵转停）
        extend_duty     伸出时的 PWM 占空比 0~1。1.0 = 全速。
                        想慢一点、稳一点，可以降到 0.6~0.8
        retract_duty    收回时的占空比
        """
        self.motor = motor
        self.extend_seconds = float(extend_seconds)
        self.retract_seconds = float(retract_seconds)
        self.extend_duty = float(extend_duty)
        self.retract_duty = float(retract_duty)

        self._phase = None          # None / 'extend' / 'retract'
        self._t0 = 0.0
        self._target = 0.0

    # ============================================================
    #  动作
    # ============================================================
    def start_extend(self):
        """开始伸出（正转）。"""
        self._phase = 'extend'
        self._t0 = time.monotonic()
        self._target = self.extend_seconds
        self.motor.forward(self.extend_duty)
        return self._target

    def start_retract(self):
        """开始收回（反转）。"""
        self._phase = 'retract'
        self._t0 = time.monotonic()
        self._target = self.retract_seconds
        self.motor.reverse(self.retract_duty)
        return self._target

    def stop(self):
        """停电机。**伸出时间到、或者出异常时都要立刻调它。**"""
        self.motor.stop()
        self._phase = None

    # ============================================================
    #  查询（节点每个周期查一次）
    # ============================================================
    @property
    def running(self):
        return self._phase is not None

    @property
    def phase(self):
        return self._phase

    @property
    def elapsed(self):
        """已经转了多少秒。"""
        return time.monotonic() - self._t0 if self._phase else 0.0

    @property
    def remaining(self):
        """还剩多少秒。"""
        return max(0.0, self._target - self.elapsed) if self._phase else 0.0

    @property
    def progress(self):
        """0~1 的进度，用来打日志。"""
        if not self._phase or self._target <= 0:
            return 0.0
        return min(1.0, self.elapsed / self._target)

    def finished(self):
        """时间到了没。到了**不会自动停** —— 由调用方 stop()，
        这样日志和状态切换能控制在一个地方。"""
        return self._phase is not None and self.elapsed >= self._target

    # ============================================================
    #  参数在线调整（实测时用 ros2 param set 边跑边改，不用重启）
    # ============================================================
    def set_extend_seconds(self, seconds):
        self.extend_seconds = max(0.1, float(seconds))
        if self._phase == 'extend':
            self._target = self.extend_seconds
        return self.extend_seconds


# ============================================================
#  备选方案：如果以后想精确控制"伸出多少毫米"
# ============================================================
class TapeCalibration:
    """圈数 ↔ 伸出长度的换算（**当前项目没用到**，留作以后扩展）。

    需要的前提：能读到电机转了多少圈（编码器 / 霍尔 / 步进）。
    本项目现在没有，所以是纯时间控制。

    真要精确控制长度时，注意放卷机构的开环误差：
        每圈放出的长度**不是常数**，随卷径减小而减小。
        按「中点半径」估算几乎就是对的，但更靠谱的是实测标定表。

    保留这个类是因为它已经过单元测试（见工作空间根目录 selftest.py），
    以后加编码器时直接就能用。
    """

    def __init__(self, full_radius_mm, core_radius_mm, gear_ratio=1.0):
        self.R = float(full_radius_mm)
        self.r = float(core_radius_mm)
        self.gear_ratio = float(gear_ratio)

    @property
    def mean_radius_mm(self):
        return (self.R + self.r) / 2.0

    def mm_per_output_rev(self):
        """输出轴每圈放出多少毫米（用中点半径估算）。"""
        import math
        return 2.0 * math.pi * self.mean_radius_mm

    def estimate_seconds(self, target_mm, output_rpm=24.0):
        """★ 估算「伸出 target_mm 大约要通电多少秒」—— 给实测一个起点。

        JGY-370 是 24 RPM，所以：秒数 = 圈数 × 60 / 24 = 圈数 × 2.5

        这**只是起点**，不是最终值。实际会因电池电压、卷尺松紧、
        机构阻力而偏，所以还是要实测微调。
        """
        revs = float(target_mm) / self.mm_per_output_rev()
        return revs * 60.0 / float(output_rpm)

    def report(self, target_mm=None, output_rpm=24.0):
        """打印一张估算表，帮你选 extend_seconds 的初值。"""
        lines = ['  卷盘: 满卷半径 %.1fmm, 卷芯 %.1fmm, 中点 %.1fmm'
                 % (self.R, self.r, self.mean_radius_mm),
                 '  每圈放出 ≈ %.1f mm' % self.mm_per_output_rev(),
                 '  电机 %.0f RPM → 每圈 %.2f 秒' % (output_rpm, 60.0 / output_rpm),
                 '']
        for mm in (100, 200, 300, 400, 500, 800):
            lines.append('  伸出 %4d mm → 约 %5.1f 圈 → 约 %5.1f 秒'
                         % (mm, mm / self.mm_per_output_rev(),
                            self.estimate_seconds(mm, output_rpm)))
        return '\n'.join(lines)
