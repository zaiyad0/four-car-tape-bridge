#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘驱动 —— 4 个 ZDT_M370 闭环伺服接麦轮。

硬件拓扑
--------
    树莓派 ──串口(TTL/RS485)──► 扩展板 ──┬── 电机1 (前左 FL)
                                          ├── 电机2 (前右 FR)
                                          ├── 电机3 (后左 RL)
                                          └── 电机4 (后右 RR)

    四个电机**并联在同一条串口总线上**，靠**地址(ID)**区分。
    地址用 `tools/zdt_id_tool.py` 探测和设置。

⚠️ 串口分配（车上两个串口设备，别搞混）
    底盘  →  /dev/ttyUSB0      USB 转 TTL/RS485 模块
             或 Pi 5 的 /dev/ttyAMA2~4（要 dtoverlay 开）
    舵机  →  /dev/ttyAMA0      GPIO14/15，物理脚 8/10

★ 减速比：一个必须先在驱动器里设好的东西
----------------------------------------
手册 5.5.7 原文：

    「设置减速比后，所有速度和位置等都会根据减速比自动缩放，
      对应电机减速后的**输出轴**的速度和位置。」

**意思是：把驱动器里的减速比设成 18.8 之后，你发出去的「速度」和读回来的
「转速」就都是输出轴（轮子）的量了 —— 树莓派这边不用再乘除 18.8！**

如果没设（出厂默认 1.00），你拿到的是**电机轴**转速，会差 18.8 倍。

所以本类的做法是：
  1. `open()` 时**读一次每个电机的减速比**
  2. 跟 `gear_ratio` 参数（18.8）比对
  3. 不匹配 → 默认**自动设成 18.8**（`setup_gear=True`），并打日志
  4. 然后所有换算都是 **1:1**，不再乘除

这样无论驱动器是出厂状态还是已经设过，接上就能用。

单位约定
--------
对外一律用**轮子的角速度 rad/s**（车体侧的物理量）。
驱动器已按减速比缩放，所以：rad/s ──×60/(2π)──► 输出轴 RPM，就是协议值。

⚠️ 性能陷阱（手册 5.2.1）
------------------------
    「所有的运动命令**只有地址 1 电机会回复**确认收到命令」

    对地址 2/3/4 用「发完等应答」的写法，每条都要空等一个 timeout（100ms），
    4 个电机 400ms —— 控制环直接废掉。所以**只有地址 1 等应答，其余发完就走**。
"""

import math
import time

from car_base.zdt_driver import ZDTBus, ZDTError

#: 轮序，跟 mecanum.inverse() 的返回值一一对应
WHEEL_NAMES = ('FL', 'FR', 'RL', 'RR')


class BaseDriver:
    """4 电机麦轮底盘驱动。

    构造参数
    --------
    port        串口设备，**必须显式指定**（车上有两个串口）
    addr_map    轮序 → 电机地址。默认 FL=1, FR=2, RL=3, RR=4
    gear_ratio  物理减速比（520 电机 = 18.8）。用于**校验/写入驱动器**，
                不参与速度换算（换算由驱动器自己做）
    setup_gear  True = 开机时把驱动器减速比自动设成 gear_ratio
                False = 只读出来警告，不改（怕误改就先设 False 看看）
    invert      哪几个轮子的正转方向跟「车前进」相反，如 ['FL','RL']
    """

    def __init__(self, port, baudrate=115200,
                 addr_map=None, gear_ratio=18.8, setup_gear=True,
                 invert=None, accel_rpm_s=2000, max_output_rpm=160.0,
                 max_current_ma=3000, use_current_limit=False):
        self.port = port
        self.baudrate = baudrate
        self.gear_ratio = float(gear_ratio)
        self.setup_gear = bool(setup_gear)
        self.accel = int(accel_rpm_s)
        # 输出轴(轮子)最大转速。3000 电机RPM / 18.8 ≈ 160 输出RPM ≈ 0.8 m/s
        self.max_output_rpm = float(max_output_rpm)
        self.max_current_ma = int(max_current_ma)
        self.use_current_limit = bool(use_current_limit)

        self.addr = dict(zip(WHEEL_NAMES, addr_map or (1, 2, 3, 4)))
        inv = set(invert or ())
        self.invert = {name: (name in inv) for name in WHEEL_NAMES}

        self.bus = None
        self.gear_ok = {}
        self._last_wheels = (0.0, 0.0, 0.0, 0.0)

    # ============================================================
    #  连接
    # ============================================================
    def open(self):
        self.bus = ZDTBus(self.port, self.baudrate)
        return self

    def check_gear_ratios(self):
        """★ 逐个读驱动器里的减速比，不对就（按需）自动设置。

        必须在使能和运动之前调用 —— 设错减速比 = 车速差 18.8 倍。
        """
        if self.bus is None:
            raise ZDTError('串口没打开，先调 open()')

        result = {}
        for name in WHEEL_NAMES:
            a = self.addr[name]
            cur = self.bus.read_gear_ratio(a)

            if cur is None:
                # 读不到：可能地址不对，或驱动器不支持这条命令
                result[name] = 'read_failed'
                continue

            if abs(cur - self.gear_ratio) < 0.005:
                result[name] = 'ok'
                continue

            if self.setup_gear:
                ok = self.bus.set_gear_ratio(a, self.gear_ratio, store=True)
                time.sleep(0.05)
                back = self.bus.read_gear_ratio(a)
                result[name] = ('fixed %.2f->%.2f' % (cur, back)
                                if ok and back and abs(back - self.gear_ratio) < 0.005
                                else 'fix_failed(%.2f)' % cur)
            else:
                result[name] = 'MISMATCH %.2f(应为%.2f)' % (cur, self.gear_ratio)

        self.gear_ok = result
        return result

    def enable_all(self, on=True):
        """使能（锁轴）或松轴。上电后一般要显式使能一次。"""
        if self.bus is None:
            return {}
        return {n: self.bus.enable(self.addr[n], on) for n in WHEEL_NAMES}

    def close(self):
        """停车 + 松轴 + 关串口。"""
        try:
            self.stop()
            time.sleep(0.05)
            self.enable_all(False)
        except Exception:
            pass
        if self.bus:
            self.bus.close()

    @property
    def connected(self):
        return self.bus is not None

    # ============================================================
    #  单位换算（1:1 —— 驱动器已经按减速比缩放过了）
    # ============================================================
    @staticmethod
    def wheel_rad_s_to_output_rpm(w):
        """轮子角速度 (rad/s) → 输出轴 RPM。"""
        return w * 60.0 / (2.0 * math.pi)

    @staticmethod
    def output_rpm_to_wheel_rad_s(rpm):
        """输出轴 RPM → 轮子角速度 (rad/s)。"""
        return rpm * 2.0 * math.pi / 60.0

    @property
    def max_wheel_rad_s(self):
        """由输出轴最大 RPM 反推出的轮子最大角速度。"""
        return self.output_rpm_to_wheel_rad_s(self.max_output_rpm)

    # ============================================================
    #  输出：下发轮速
    # ============================================================
    def set_wheels(self, w_fl, w_fr, w_rl, w_rr):
        """下发四个轮子的角速度 (rad/s)。正 = 车前进方向。"""
        if self.bus is None:
            raise ZDTError('串口没打开，先调 open()')

        wheels = dict(zip(WHEEL_NAMES, (w_fl, w_fr, w_rl, w_rr)))
        results = {}

        for name, w in wheels.items():
            addr = self.addr[name]

            if self.invert[name]:       # 反相：让正的轮速永远表示车前进
                w = -w

            rpm = self.wheel_rad_s_to_output_rpm(w)

            # 限幅（保方向，不是截断）
            if abs(rpm) > self.max_output_rpm:
                rpm = math.copysign(self.max_output_rpm, rpm)

            # ★ 只有地址 1 会回确认。其余发完就走，不然控制环被拖垮。
            wait = 0.05 if addr == 1 else 0.0

            try:
                if self.use_current_limit:
                    ok = self.bus.set_speed_limit_current(
                        addr, rpm, self.max_current_ma,
                        accel_rpm_s=self.accel, sync=False)
                else:
                    ok = self.bus.set_speed(
                        addr, rpm, accel_rpm_s=self.accel,
                        sync=False, timeout=wait)
            except Exception:
                ok = False
            results[name] = ok

        self._last_wheels = (w_fl, w_fr, w_rl, w_rr)
        return results

    def stop(self):
        """停车。安全相关，任何时候都要能调。"""
        return self.set_wheels(0.0, 0.0, 0.0, 0.0)

    # ============================================================
    #  输入：读回轮速
    # ============================================================
    def read_output_rpms(self):
        """读四个电机输出轴转速 (RPM)。读不到的返回 None。"""
        if self.bus is None:
            return {n: None for n in WHEEL_NAMES}
        out = {}
        for name in WHEEL_NAMES:
            rpm = self.bus.read_speed(self.addr[name])
            if rpm is not None and self.invert[name]:
                rpm = -rpm             # 反相回来，跟 set_wheels 的约定一致
            out[name] = rpm
        return out

    def read_wheel_speeds(self):
        """★ 读四个轮子的实时角速度 (rad/s)。里程计用这个。

        电机是**闭环伺服**，这个值反映的是**真实转速**（内部编码器测的），
        不是我们下发的目标值 —— 所以拿来做里程计比开环估算准得多。
        读不到的位返回 None（不要用 0 冒充，那会让里程计跳变）。
        """
        rpm = self.read_output_rpms()
        return tuple(
            None if rpm[n] is None else self.output_rpm_to_wheel_rad_s(rpm[n])
            for n in WHEEL_NAMES)

    # ============================================================
    #  调试 / 诊断
    # ============================================================
    def scan_motors(self, addr_range=range(1, 33)):
        """扫描总线上在线的电机地址。"""
        if self.bus is None:
            raise ZDTError('串口没打开')
        return self.bus.scan(addr_range)

    def health_check(self):
        """每个电机的连通性 / 减速比 / 温度 / 转速。联调时先跑这个。"""
        if self.bus is None:
            return {}
        out = {}
        for name in WHEEL_NAMES:
            a = self.addr[name]
            online = self.bus.is_alive(a)
            out[name] = {
                'addr': a,
                'online': online,
                'gear': self.bus.read_gear_ratio(a) if online else None,
                'temp_c': self.bus.read_temperature(a) if online else None,
                'speed_rpm': self.bus.read_speed(a) if online else None,
            }
        return out

    @property
    def last_wheels(self):
        return self._last_wheels
