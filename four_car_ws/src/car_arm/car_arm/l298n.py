#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""L298N 双 H 桥电机驱动模块 —— 控制其中**一路**（一个电机）。

接线（一路）
------------
    L298N          树莓派              说明
    ─────────────────────────────────────────────
    ENA      ←     GPIO (PWM)          调速。**要把 ENA 上的跳线帽拔掉**，
                                       插着的话 ENA 常高，PWM 不起作用
    IN1      ←     GPIO                方向
    IN2      ←     GPIO                方向（和 IN1 相反 = 反转）
    GND      ←     GPIO GND            ★ 必须共地！不共地电机乱转或不动
    +12V     ←     电池正极
    GND      ←     电池负极
    OUT1/OUT2 →    电机两根线

    板载 5V 那一路：如果用跳线帽连着 12V，板上的稳压器会给逻辑供电。
    但**树莓派建议单独供电**，别从 L298N 的 5V 取电 —— 电机启停会把 5V 拉垮，
    树莓派会重启。

⚠️ Pi 5 注意事项
----------------
树莓派 5 的 GPIO 挂在 RP1 芯片上，**`RPi.GPIO` 完全不工作**。
必须用 `gpiozero`（底层走 `lgpio`）。装：
    sudo apt install -y python3-gpiozero python3-lgpio
"""


class L298NError(RuntimeError):
    pass


class L298N:
    """L298N 模块的一路（一个电机）。

    用法::

        m = L298N(ena_pin=12, in1_pin=23, in2_pin=24)
        m.set(0.8)        # 正转 80% 占空比
        m.set(-0.5)       # 反转 50%
        m.stop()
        m.close()
    """

    def __init__(self, ena_pin, in1_pin, in2_pin,
                 pwm_freq=1000, simulate=False, active_high=True):
        self.ena_pin = int(ena_pin)
        self.in1_pin = int(in1_pin)
        self.in2_pin = int(in2_pin)
        self.pwm_freq = int(pwm_freq)
        self.simulate = bool(simulate)

        self._ena = None
        self._in1 = None
        self._in2 = None
        self._duty = 0.0

        if not self.simulate:
            self._setup(active_high)

    def _setup(self, active_high):
        try:
            from gpiozero import DigitalOutputDevice, PWMOutputDevice
        except ImportError:
            raise L298NError(
                '没装 gpiozero。树莓派 5 上不能用 RPi.GPIO，请执行：\n'
                '    sudo apt install -y python3-gpiozero python3-lgpio')

        try:
            # 频率 1kHz 对电机够用；太高 L298N 也跟不上（它的开关损耗大）
            self._ena = PWMOutputDevice(self.ena_pin, frequency=self.pwm_freq)
            self._in1 = DigitalOutputDevice(self.in1_pin, active_high=active_high)
            self._in2 = DigitalOutputDevice(self.in2_pin, active_high=active_high)
        except Exception as exc:
            raise L298NError(
                '初始化 GPIO 失败 (%s)。排查：\n'
                '  · 引脚号是不是被别的功能占了（比如 UART/I2C 引脚）\n'
                '  · 用户在 gpio 组吗：sudo usermod -aG gpio $USER 后重登\n'
                '  · gpiochip 存在吗：ls /dev/gpiochip*' % exc)

    # ============================================================
    def set(self, duty):
        """设置输出。duty ∈ [-1, 1]：正 = 正转，负 = 反转，0 = 停。

        先设方向再给 PWM —— 顺序反了会有一下反向冲击。
        """
        duty = max(-1.0, min(1.0, float(duty)))
        self._duty = duty

        if self.simulate or self._ena is None:
            return duty

        if duty > 0.001:
            self._in1.on()
            self._in2.off()
            self._ena.value = duty
        elif duty < -0.001:
            self._in1.off()
            self._in2.on()
            self._ena.value = -duty
        else:
            # 停：先断 PWM 再把两个方向脚都拉低（滑行停车）
            self._ena.value = 0.0
            self._in1.off()
            self._in2.off()
        return duty

    def forward(self, duty=1.0):
        return self.set(abs(duty))

    def reverse(self, duty=1.0):
        return self.set(-abs(duty))

    def stop(self):
        return self.set(0.0)

    @property
    def duty(self):
        return self._duty

    @property
    def running(self):
        return abs(self._duty) > 0.001

    def close(self):
        try:
            self.stop()
        except Exception:
            pass
        for dev in (self._ena, self._in1, self._in2):
            if dev is not None:
                try:
                    dev.close()
                except Exception:
                    pass
