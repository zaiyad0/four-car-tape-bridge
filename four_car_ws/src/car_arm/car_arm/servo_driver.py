#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""总线舵机驱动 —— 从「总线舵机控制器」项目移植。

本项目只需要「开机设一个固定俯仰角」，所以只保留了位置控制。
协议依据厂商文档：帧格式 0x55 0x55 <数据长度> <指令> <参数...>，长度 = 参数个数 + 2。

⚠️ 移植时保留的两个坑（原项目踩过，这里同样成立）
------------------------------------------------
1. **串口被 getty 占用**：Ubuntu 上 `serial-getty@ttyAMA0` 会占住串口，
   权限变成 `crw--w---- root tty`（tty 组只有写、没有读），普通用户打不开。
   修复：
       sudo systemctl disable --now serial-getty@ttyAMA0.service
       echo 'KERNEL=="ttyAMA0", GROUP="dialout", MODE="0660"' \
         | sudo tee /etc/udev/rules.d/99-serial-servo.rules
       sudo udevadm control --reload-rules && sudo udevadm trigger
   然后**退出重登**（用户得在 dialout 组里）。

2. **读串口不能傻等 timeout**：`select()` 报可读但 `read()` 返回 0 字节时会抛
   "device reports readiness to read but returned no data"。改成轮询
   `in_waiting`，解析出完整帧**立刻返回**，否则定时器回调会堆积。
"""

import os
import time

BAUDRATE = 9600
FRAME = 0x55
CMD_SERVO_MOVE = 3
CMD_MULT_SERVO_POS_READ = 21
CMD_GET_BATTERY_VOLTAGE = 15

POS_MIN, POS_MAX, POS_MID = 0, 1000, 500


class ServoError(RuntimeError):
    pass


def find_port(candidates=('/dev/serial0', '/dev/ttyAMA0', '/dev/ttyS0')):
    for p in candidates:
        if os.path.exists(p):
            return p
    raise ServoError('找不到串口设备，试过：%s' % ', '.join(candidates))


class BusServo:
    """总线舵机。

    ⚠️ 本项目一台车有**两个串口设备**（底盘板 + 舵机），
       所以 port **必须显式指定**，不要用 find_port() 自动探测 ——
       探测到的可能是底盘板那个。
    """

    def __init__(self, port, baudrate=BAUDRATE, max_angle=240.0):
        if not port:
            raise ServoError('必须显式指定舵机串口，不能自动探测（车上有两个串口）')
        self.port = port
        self.max_angle = float(max_angle)
        try:
            import serial
            self.ser = serial.Serial(port, baudrate, timeout=0)
        except Exception as exc:
            raise ServoError('打开舵机串口 %s 失败: %s' % (port, exc))
        time.sleep(0.05)

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    # ---------------- 角度换算 ----------------
    def angle_to_pos(self, angle_deg):
        """以中位为零点、正负可变的「相对角」→ 位置 0~1000"""
        half = self.max_angle / 2.0
        a = max(-half, min(half, float(angle_deg)))
        return int(round(max(POS_MIN, min(POS_MAX, POS_MID + a / half * POS_MID))))

    # ---------------- 指令 ----------------
    def move(self, servo_id, position, duration_ms=1000):
        position = max(POS_MIN, min(POS_MAX, int(position)))
        t = max(0, min(30000, int(duration_ms)))
        buf = bytearray(b'\x55\x55')
        buf.append(0x08)
        buf.append(CMD_SERVO_MOVE)
        buf.append(0x01)
        buf += t.to_bytes(2, 'little')
        buf.append(servo_id if 1 <= servo_id <= 254 else 254)
        buf += position.to_bytes(2, 'little')
        self.ser.write(buf)

    def move_angle(self, servo_id, angle_deg, duration_ms=1000):
        self.move(servo_id, self.angle_to_pos(angle_deg), duration_ms)

    # ---------------- 读回（启用 read 的时候再放开） ----------------
    def _read_frame(self, timeout):
        """等一个完整应答帧，收到立刻返回 (cmd, params)；超时返回 None。

        见文件头「坑 2」——不要改成阻塞 read。
        """
        buf = bytearray()
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                n = self.ser.in_waiting
            except Exception:
                n = 0
            if n:
                try:
                    buf += self.ser.read(n)
                except Exception:
                    pass
                f = self._parse(bytes(buf))
                if f is not None:
                    return f
            else:
                time.sleep(0.003)
        return None

    @staticmethod
    def _parse(data):
        i = 0
        while i + 4 <= len(data):
            if data[i] == FRAME and data[i + 1] == FRAME:
                ln = data[i + 2]
                if i + ln + 2 <= len(data):
                    return data[i + 3], data[i + 4:i + ln + 2]
            i += 1
        return None

    def read_pos(self, ids, timeout=0.5):
        ids = [int(i) for i in ids]
        if not ids:
            return {}
        n = len(ids)
        self.ser.reset_input_buffer()
        self.ser.write(bytes([FRAME, FRAME, n + 3, CMD_MULT_SERVO_POS_READ, n] + ids))
        f = self._read_frame(timeout)
        if not f or f[0] != CMD_MULT_SERVO_POS_READ:
            return None
        p = f[1]
        return {p[1 + i * 3]: p[2 + i * 3] | (p[3 + i * 3] << 8)
                for i in range(p[0])}

    def read_battery_mv(self, timeout=0.5):
        self.ser.reset_input_buffer()
        self.ser.write(bytes([FRAME, FRAME, 0x02, CMD_GET_BATTERY_VOLTAGE]))
        f = self._read_frame(timeout)
        if not f or f[0] != CMD_GET_BATTERY_VOLTAGE or len(f[1]) < 2:
            return None
        return f[1][0] | (f[1][1] << 8)
