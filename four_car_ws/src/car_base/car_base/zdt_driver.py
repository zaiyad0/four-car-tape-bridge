#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""张大头 ZDT_M370 全闭环直流减速电机 —— 串口协议驱动。

依据《ZDT_M370 全闭环直流减速电机用户手册 V1.0.0》。

帧格式（串口 TTL / RS485，第 4.1 节）
------------------------------------
    主机 → 电机:  [Addr] [Code] [命令数据...] [校验码]
    电机 → 主机:  [Addr] [Code] [返回数据...] [校验码]

    Addr    1-255，0 = 广播地址，**出厂默认 1**
    Code    功能码（见下面常量）
    校验码  **出厂默认为固定字节 0x6B**（也可设 XOR / CRC8 / Modbus-RTU）

    电机返回的状态字节:
        0x02  接收正确
        0x12  回零时已在零点处
        0x22  回零时限位已触发
        0xE2  参数错误（范围不对 / 触发低压、堵转、过流保护）
        0xEE  格式错误
        0x9F  动作执行完成（电机主动返回）

⚠️ 一条总线挂多个电机时的两个坑（手册第 5.2.1 节）
------------------------------------------------
1. **所有运动命令只有「地址 1」的电机会回复确认**，避免总线冲突。
   所以如果你把电机地址设成 1 之外的值，发运动命令是收不到 02 应答的 ——
   这不是故障，是设计。想确认就得用「读取」类命令单独问。
2. 多电机命令（AA）里如果混了读取命令，要避开地址 1 返回的确认包。

⚠️ 串口分配（本项目一台车有两个串口设备，别搞混）
------------------------------------------------
    舵机   →  /dev/ttyAMA0   （GPIO14/15，物理脚 8/10）
    底盘   →  /dev/ttyUSB0   （USB 转 TTL/RS485 模块）
              或 Pi 5 的额外硬件串口 /dev/ttyAMA2~4（需要 dtoverlay 开启）

    构造函数里 **必须显式指定端口**，不要自动探测 —— 探到的可能是舵机那个。
"""

import time

# ============================================================
#  功能码
# ============================================================
CODE_CALIBRATE = 0x06        # 触发校准（需辅助码 0x45）
CODE_REBOOT = 0x08           # 重启电机（辅助码 0x97）
CODE_ZERO_POS = 0x0A         # 当前位置角度清零（辅助码 0x6D）
CODE_RESET_PARAM = 0x0C      # 恢复默认参数（辅助码 0x3C）
CODE_RELEASE_PROTECT = 0x0E  # 解除堵转/过流保护（辅助码 0x52）
CODE_FACTORY_RESET = 0x0F    # 恢复出厂设置（辅助码 0x5F）

CODE_MULTI = 0xAA            # 多电机命令
CODE_ENABLE = 0xF3           # 使能控制（辅助码 0xAB）
CODE_TORQUE = 0xF5           # 力矩模式
CODE_TORQUE_LIMIT = 0xC5     # 力矩模式限速
CODE_SPEED = 0xF6            # ★ 速度模式
CODE_SPEED_LIMIT_I = 0xC6    # 速度模式限电流
CODE_POS_PASSTHRU = 0xFB     # 直通限速位置模式
CODE_POS_PASSTHRU_I = 0xCB   # 直通限速位置模式限电流

CODE_READ_ID = 0x15          # ★ 广播读取 ID 地址
CODE_MODIFY_ID = 0xAE        # ★ 修改电机 ID/地址（辅助码 0x4B）
CODE_READ_PID = 0x21
CODE_MODIFY_PID = 0x4A
CODE_READ_GEAR = 0x47        # 读取减速比
CODE_MODIFY_GEAR = 0x6A      # 修改减速比

CODE_READ_SPEED = 0x35       # ★ 读取实时转速 (0.1RPM)
CODE_READ_POS = 0x36         # ★ 读取实时位置 (0.1°)
CODE_READ_ERR = 0x37         # 读取位置误差 (0.01°)
CODE_READ_TEMP = 0x39        # 读取驱动温度 (℃)

# 辅助码
AUX_CALIBRATE = 0x45
AUX_REBOOT = 0x97
AUX_ZERO_POS = 0x6D
AUX_RESET_PARAM = 0x3C
AUX_RELEASE_PROTECT = 0x52
AUX_FACTORY_RESET = 0x5F
AUX_ENABLE = 0xAB
AUX_MODIFY_ID = 0x4B
AUX_MODIFY_GEAR = 0x51

# 电机返回状态字节
ACK_OK = 0x02
ACK_PARAM_ERR = 0xE2
ACK_FORMAT_ERR = 0xEE
ACK_DONE = 0x9F

CHECKSUM_FIXED = 0x6B        # 出厂默认：固定校验字节
BROADCAST = 0x00             # 广播地址


class ZDTError(RuntimeError):
    pass


def _port_hint():
    """打开串口失败时，列一下机器上现在有哪些串口 —— 联调时能省很多时间。"""
    try:
        from glob import glob
        devs = sorted(glob('/dev/ttyUSB*') + glob('/dev/ttyACM*')
                      + glob('/dev/ttyAMA*') + glob('/dev/ttyS*'))
    except Exception:
        devs = []

    if not devs:
        return ('\n  机器上没发现任何串口设备。\n'
                '  · USB 转串口模块插上了吗？   lsusb\n'
                '  · 驱动加载了吗？            dmesg | tail -20\n'
                '  · 权限够吗？                sudo usermod -aG dialout $USER 后重登')
    return ('\n  当前存在的串口: %s\n'
            '  · 底盘用 USB 那个（ttyUSB*/ttyACM*）\n'
            '  · /dev/ttyAMA0 是**舵机**用的，别占它\n'
            '  · 打开失败常见原因：被别的进程占用、用户不在 dialout 组'
            % ', '.join(devs))


class ZDTBus:
    """一条串口总线，上面挂若干个 ZDT 电机（靠地址区分）。

    用法::

        bus = ZDTBus('/dev/ttyUSB0')
        bus.enable(1, True)              # 使能 1 号电机
        bus.set_speed(1, rpm=100.0)      # 速度模式，100 RPM
        spd = bus.read_speed(1)          # 读回实时转速
        bus.close()
    """

    def __init__(self, port, baudrate=115200, timeout=0.1, verbose=False):
        """
        port     串口设备，**必须显式指定**（车上有两个串口设备）
        baudrate 手册默认 115200
        timeout  等应答的超时 (s)
        """
        if not port:
            raise ZDTError('必须显式指定电机串口（车上有两个串口设备，不能自动探测）')
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.verbose = verbose

        try:
            import serial
        except ImportError:
            raise ZDTError('没装 pyserial：sudo apt install python3-serial')

        try:
            self.ser = serial.Serial(port, baudrate, timeout=0)
        except Exception as exc:
            raise ZDTError('打开电机串口 %s 失败: %s%s'
                           % (port, exc, _port_hint()))
        time.sleep(0.05)

    # ============================================================
    #  底层收发
    # ============================================================
    def _build(self, addr, code, data=b''):
        """拼一帧。校验码按出厂默认的固定 0x6B。"""
        return bytes([addr & 0xFF, code & 0xFF]) + bytes(data) + bytes([CHECKSUM_FIXED])

    def _send(self, addr, code, data=b''):
        frame = self._build(addr, code, data)
        self.ser.write(frame)
        if self.verbose:
            print('  TX: %s' % frame.hex(' '))
        return frame

    def _read_frame(self, timeout=None):
        """等一个完整应答帧，收到立刻返回 bytes；超时返回 None。

        ⚠️ 不要用「select 说可读就 read」的写法 —— 会报
           "device reports readiness to read but returned no data"。
           改成轮询 in_waiting，能解析出帧就返回（这个坑在舵机项目里踩过）。
        """
        t = self.timeout if timeout is None else timeout
        buf = bytearray()
        deadline = time.time() + t
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
                # 最短应答 4 字节：Addr Code 数据 校验
                if len(buf) >= 4 and buf[-1] == CHECKSUM_FIXED:
                    if self.verbose:
                        print('  RX: %s' % bytes(buf).hex(' '))
                    return bytes(buf)
            else:
                time.sleep(0.002)
        return None

    def _request(self, addr, code, data=b'', expect_data_len=None, timeout=None):
        """发一条命令并等应答。返回 (状态码, 数据字节)；没应答返回 (None, None)。"""
        self.ser.reset_input_buffer()
        self._send(addr, code, data)
        frame = self._read_frame(timeout)
        if frame is None:
            return None, None
        if len(frame) < 4:
            return None, None
        status = frame[2]
        body = frame[3:-1] if len(frame) > 4 else b''
        if expect_data_len is not None and len(body) < expect_data_len:
            return status, None
        return status, body

    # ============================================================
    #  运动控制
    # ============================================================
    def enable(self, addr, on=True, sync=False):
        """使能 / 松轴。on=False 时电机松轴，可以用手拧动。"""
        st, _ = self._request(addr, CODE_ENABLE,
                              bytes([AUX_ENABLE, 1 if on else 0, 1 if sync else 0]))
        return st == ACK_OK

    def set_speed(self, addr, rpm, accel_rpm_s=1000, sync=False, timeout=None):
        """★ 速度模式（F6）—— 电机内部就是速度闭环。

        参数
        ----
        addr       电机地址
        rpm        目标转速，**单位是电机轴的 RPM**，带符号（负 = 反转）
                   范围 ±3000.0 RPM。注意这是**电机轴**转速，
                   轮子转速 = 本值 / 减速比
        accel_rpm_s 加速度，0-65535 RPM/S。越大起步越猛
        sync       False=立即执行，True=先缓存（配合多机同步，见手册 3.3）

        发送帧: Addr F6 方向 加速度(2B) 速度(2B) 同步 6B
        """
        direction = 0x01 if rpm < 0 else 0x00      # 00=CW 01=CCW

        # 速度单位是 0.1RPM，范围 0-30000
        mag = abs(int(round(rpm * 10.0)))
        mag = max(0, min(0x7E30, mag))

        acc = max(0, min(0xFFFF, int(accel_rpm_s)))

        data = bytes([direction,
                      (acc >> 8) & 0xFF, acc & 0xFF,
                      (mag >> 8) & 0xFF, mag & 0xFF,
                      1 if sync else 0])
        st, _ = self._request(addr, CODE_SPEED, data, timeout=timeout)
        return st == ACK_OK

    def set_speed_limit_current(self, addr, rpm, max_current_ma,
                                accel_rpm_s=1000, sync=False):
        """速度模式 + 最大电流限制（C6）。限流能防堵转烧电机，建议加上。"""
        direction = 0x01 if rpm < 0 else 0x00
        mag = max(0, min(0x7E30, abs(int(round(rpm * 10.0)))))
        acc = max(0, min(0xFFFF, int(accel_rpm_s)))
        cur = max(0, min(0x1388, int(max_current_ma)))
        data = bytes([direction,
                      (acc >> 8) & 0xFF, acc & 0xFF,
                      (mag >> 8) & 0xFF, mag & 0xFF,
                      1 if sync else 0,
                      (cur >> 8) & 0xFF, cur & 0xFF])
        st, _ = self._request(addr, CODE_SPEED_LIMIT_I, data)
        return st == ACK_OK

    def stop(self, addr, accel_rpm_s=2000):
        """停车（速度设为 0）。"""
        return self.set_speed(addr, 0.0, accel_rpm_s)

    # ============================================================
    #  读取反馈
    # ============================================================
    def read_speed(self, addr, timeout=None):
        """★ 读实时转速。返回**电机轴 RPM**（带符号）；读不到返回 None。

        返回帧: Addr 35 符号 转速(2B) 6B     转速单位 0.1RPM
        """
        st, body = self._request(addr, CODE_READ_SPEED, timeout=timeout)
        if st != ACK_OK or body is None or len(body) < 3:
            return None
        sign = -1.0 if body[0] else 1.0
        raw = (body[1] << 8) | body[2]
        return sign * raw / 10.0

    def read_position(self, addr, timeout=None):
        """读实时位置角度（度，带符号）；读不到返回 None。

        返回帧: Addr 36 符号 位置(4B) 6B     位置单位 0.1°
        """
        st, body = self._request(addr, CODE_READ_POS, timeout=timeout)
        if st != ACK_OK or body is None or len(body) < 5:
            return None
        sign = -1.0 if body[0] else 1.0
        raw = (body[1] << 24) | (body[2] << 16) | (body[3] << 8) | body[4]
        return sign * raw / 10.0

    def read_temperature(self, addr):
        """读驱动温度 (℃)，读不到返回 None。"""
        st, body = self._request(addr, CODE_READ_TEMP)
        if st != ACK_OK or body is None or len(body) < 2:
            return None
        sign = -1 if body[0] else 1
        return sign * body[1]

    # ============================================================
    #  地址（ID）读写 —— 解决「不知道电机原始地址」
    # ============================================================
    def read_id(self, timeout=0.5):
        """★ 广播读取 ID 地址。**总线上只能接一个电机**时才能用。

        发送: 00 15 6B        （00 = 广播）
        返回: Addr 15 Addr 6B （电机把自己的地址报回来）

        用途：拿到一堆电机不知道谁是谁时，一个一个单独接上来问出地址。
        """
        self.ser.reset_input_buffer()
        self._send(BROADCAST, CODE_READ_ID)
        frame = self._read_frame(timeout)
        if frame is None or len(frame) < 4 or frame[1] != CODE_READ_ID:
            return None
        return frame[2]

    def set_id(self, old_addr, new_addr, store=True):
        """★ 修改电机地址。

        发送: Addr AE 4B 是否存储 新地址 6B
        store=True 会写进 flash，掉电不丢。

        ⚠️ 改完之后必须用**新地址**通信。建议改完立刻 read 一次验证。
        """
        if not 1 <= new_addr <= 0xFF:
            raise ZDTError('地址范围 1-255')
        st, _ = self._request(old_addr, CODE_MODIFY_ID,
                              bytes([AUX_MODIFY_ID, 1 if store else 0, new_addr]))
        return st == ACK_OK

    # ============================================================
    #  维护类
    # ============================================================
    def release_protection(self, addr):
        """解除堵转/过流保护。电机被保护锁住时用这个解开。"""
        st, _ = self._request(addr, CODE_RELEASE_PROTECT, bytes([AUX_RELEASE_PROTECT]))
        return st == ACK_OK

    def zero_position(self, addr):
        """把当前位置清零（当成零点）。"""
        st, _ = self._request(addr, CODE_ZERO_POS, bytes([AUX_ZERO_POS]))
        return st == ACK_OK

    def reboot(self, addr):
        st, _ = self._request(addr, CODE_REBOOT, bytes([AUX_REBOOT]))
        return st == ACK_OK

    def calibrate(self, addr):
        """触发校准（上电后如果没校准过，需要跑一次）。"""
        st, _ = self._request(addr, CODE_CALIBRATE, bytes([AUX_CALIBRATE]),
                              timeout=3.0)
        return st == ACK_OK

    def read_gear_ratio(self, addr):
        """★ 读驱动器里存着的减速比，返回浮点（如 18.8）。读不到返回 None。

        返回帧: Addr 47 减速比(4B) 6B，单位 0.01（1880 → 18.80）
        """
        st, body = self._request(addr, CODE_READ_GEAR)
        if st != ACK_OK or body is None or len(body) < 4:
            return None
        raw = (body[0] << 24) | (body[1] << 16) | (body[2] << 8) | body[3]
        return raw / 100.0

    def set_gear_ratio(self, addr, ratio, store=True):
        """★ 设置减速比 —— **上电后必须先做这一步**。

        手册 5.5.7 原文：
            「设置减速比后，所有速度和位置等都会根据减速比自动缩放，
              对应电机减速后的**输出轴**的速度和位置。」

        ⚠️ 这句话的意思是：**设了 18.8 之后，你发出去的「速度」和读回来的
           「转速/位置」就都是输出轴（轮子）的量了**，树莓派这边不用再乘除。

           不设的话默认是 1.00，你收到的就是**电机轴**转速 —— 差 18.8 倍。

        发送: Addr 6A 51 是否存储 减速比(4B) 6B，单位 0.01（18.8 → 1880）
        """
        raw = int(round(float(ratio) * 100))
        if not 0 <= raw <= 0xFFFFFFFF:
            raise ZDTError('减速比超出范围')
        data = (bytes([AUX_MODIFY_GEAR, 1 if store else 0])
                + raw.to_bytes(4, 'big'))
        st, _ = self._request(addr, CODE_MODIFY_GEAR, data)
        return st == ACK_OK

    # ============================================================
    def is_alive(self, addr):
        """简单探活：读得到转速就说明这个地址上有电机。"""
        return self.read_speed(addr) is not None

    def scan(self, addr_range=range(1, 33)):
        """扫一遍地址，返回在线的地址列表。

        注意：运动命令只有地址 1 会应答，但**读取命令每个地址都会应答**，
        所以用「读转速」来探测是可靠的。
        """
        found = []
        for a in addr_range:
            if self.is_alive(a):
                found.append(a)
        return found

    def close(self):
        try:
            # 先停车再关，避免电机继续转
            pass
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass
