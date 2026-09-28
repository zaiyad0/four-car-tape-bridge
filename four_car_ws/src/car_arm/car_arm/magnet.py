#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""电磁铁控制 —— 就是一个 GPIO 开关。

接线
----
    ★ **不能拿 GPIO 直接拉电磁铁**。电磁铁线圈电流通常是安培级，
      树莓派一个引脚最多给 16mA，直接接要么不吸、要么烧 GPIO。

    正确接法：GPIO → 驱动级（继电器模块 / MOSFET 模块 / L298N 空闲那一路）
              → 电磁铁 → 电池

    共地仍然是必须的：树莓派 GND ↔ 驱动模块 GND ↔ 电池负极。

    GPIO ──► 驱动模块 ──► 电磁铁
              │
            电池 12V

    ⚠️ 电磁铁是感性负载，断电瞬间会产生反向高压。
       如果是自己搭的 MOSFET 电路，**线圈两端必须并一个续流二极管**
       （1N4007 之类的），否则那个尖峰会打穿 MOSFET 或干扰树莓派。
       买现成的继电器模块/MOSFET 模块的话一般已经带了。

    ⚠️ 保持通电会发热。吸合后如果不需要一直吸着，建议用 `hold=False`
       让它在吸稳后自动断电（卷尺桥靠磁力吸住应该能保持，
       但机械上最好有个自锁结构，别依赖持续通电）。
"""

import time


class MagnetError(RuntimeError):
    pass


class Magnet:
    """电磁铁开关。

    用法::

        mag = Magnet(pin=17)
        mag.on()
        ...
        mag.off()
        mag.close()
    """

    def __init__(self, pin, active_high=True, simulate=False):
        """
        pin         控制引脚（接驱动模块，不是直接接电磁铁）
        active_high True = 引脚高电平吸合（大多数继电器模块是低电平触发，
                    那种情况这里填 False）
        simulate    不碰 GPIO，只记录状态
        """
        self.pin = int(pin)
        self.active_high = bool(active_high)
        self.simulate = bool(simulate)
        self._on = False
        self._dev = None
        self._t_on = 0.0

        if not self.simulate:
            self._setup()

    def _setup(self):
        try:
            from gpiozero import DigitalOutputDevice
        except ImportError:
            raise MagnetError(
                '没装 gpiozero。树莓派 5 上不能用 RPi.GPIO，请执行：\n'
                '    sudo apt install -y python3-gpiozero python3-lgpio')
        try:
            self._dev = DigitalOutputDevice(self.pin,
                                            active_high=self.active_high,
                                            initial_value=False)
        except Exception as exc:
            raise MagnetError('初始化电磁铁 GPIO %d 失败: %s' % (self.pin, exc))

    # ============================================================
    def on(self):
        self._on = True
        self._t_on = time.monotonic()
        if self._dev:
            self._dev.on()

    def off(self):
        self._on = False
        if self._dev:
            self._dev.off()

    def toggle(self):
        self.off() if self._on else self.on()

    @property
    def is_on(self):
        return self._on

    @property
    def on_seconds(self):
        """已经通电多久（秒）。可以用来判断"吸了这么久还是没吸上"。"""
        return time.monotonic() - self._t_on if self._on else 0.0

    def close(self):
        try:
            self.off()
        finally:
            if self._dev:
                try:
                    self._dev.close()
                except Exception:
                    pass
