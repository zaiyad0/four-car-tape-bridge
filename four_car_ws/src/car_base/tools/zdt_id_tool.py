#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ZDT_M370 电机地址(ID) 检测 / 设置工具。

解决的问题：拿到几个电机，**不知道它们的原始地址**，没法往上报。

用法（在树莓派上跑）:

    # ① 扫描总线，看在线的电机地址都是几
    python3 zdt_id_tool.py --port /dev/ttyUSB0 scan

    # ② 只接一个电机时，广播问它的地址（比扫描快，且不用猜范围）
    python3 zdt_id_tool.py --port /dev/ttyUSB0 readid

    # ③ 把地址 5 改成 1（并写入 flash，掉电不丢）
    python3 zdt_id_tool.py --port /dev/ttyUSB0 setid 5 1

    # ④ 一键把 4 个电机设成 1/2/3/4（按提示一个一个来）
    python3 zdt_id_tool.py --port /dev/ttyUSB0 setup

    # ⑤ 体检：每个电机的连通性、温度、当前转速
    python3 zdt_id_tool.py --port /dev/ttyUSB0 health

⚠️ 前提
--------
- 波特率默认 115200（手册出厂值）。不对就加 --baud 9600 之类试
- 校验字节默认固定 0x6B（出厂值）。如果被人改成 XOR/CRC8 了，这个脚本读不出，
  需要用官方上位机改回 0x6B
- **扫描时总线上可以挂多个电机**；但「readid」是广播命令，**只能接一个**
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..'))
from car_base.zdt_driver import ZDTBus, ZDTError     # noqa: E402

WHEEL_HINT = {
    1: '前左 FL',
    2: '前右 FR',
    3: '后左 RL',
    4: '后右 RR',
}


def cmd_scan(bus, args):
    print('扫描地址 %d ~ %d ...' % (args.scan_from, args.scan_to))
    print('（用「读转速」探测 —— 读取命令每个地址都会应答，可靠）')
    print()
    found = []
    for a in range(args.scan_from, args.scan_to + 1):
        rpm = bus.read_speed(a)
        if rpm is not None:
            temp = bus.read_temperature(a)
            found.append(a)
            print('  ✓ 地址 %-3d  转速 %8.1f RPM  温度 %s'
                  % (a, rpm, ('%s℃' % temp) if temp is not None else '—'))
        else:
            sys.stdout.write('.')
            sys.stdout.flush()
    print()
    print()
    if found:
        print('找到 %d 个电机，地址: %s' % (len(found), found))
        print()
        print('如果这就是你想要的 1/2/3/4，接下来确认轮子接线：')
        for a in found:
            if a in WHEEL_HINT:
                print('    地址 %d  →  应该是 %s' % (a, WHEEL_HINT[a]))
    else:
        print('没找到任何电机。排查：')
        print('  1. 串口对不对？    ls /dev/ttyUSB* /dev/ttyAMA*')
        print('  2. 波特率对不对？  试试 --baud 9600 / 57600 / 115200')
        print('  3. 电机上电了吗？测一下总线电压')
        print('  4. A/B 有没有接反？TTL 要交叉接（TX↔RX）')
        print('  5. 校验字节被改过？用官方上位机改回 0x6B')
    return found


def cmd_readid(bus, args):
    print('广播读取 ID（总线上必须只接一个电机）...')
    for i in range(3):
        a = bus.read_id(timeout=0.5)
        if a is not None:
            print()
            print('  这个电机的地址 = %d' % a)
            return a
        time.sleep(0.2)
    print()
    print('没读到。可能原因：')
    print('  · 总线上接了不止一个电机 —— 广播命令会撞车，请只接一个')
    print('  · 波特率/校验字节不对')
    print('  · A/B 接反了')
    print()
    print('试试改用扫描模式（能带多个电机）：')
    print('    python3 %s --port %s scan' % (sys.argv[0], args.port))
    return None


def cmd_setid(bus, args):
    old, new = args.old, args.new
    print('把地址 %d 改成 %d ...' % (old, new))

    if not bus.is_alive(old):
        print('  ✗ 地址 %d 上没有应答，先 scan 一下确认它现在是几号' % old)
        return False

    ok = bus.set_id(old, new, store=True)
    if not ok:
        print('  ✗ 改地址命令失败')
        return False

    time.sleep(0.3)
    print('  命令已发送，验证中...')
    if bus.is_alive(new):
        print('  ✓ 成功！现在地址是 %d（已写入 flash，掉电不丢）' % new)
        if not bus.is_alive(old):
            print('  ✓ 旧地址 %d 已无应答，确认切换完成' % old)
        return True
    print('  ? 新地址 %d 没应答。可能没存住，断电重启后再 scan 看看' % new)
    return False


def cmd_setup(bus, args):
    print('=' * 62)
    print('  一键设置 4 个电机地址为 1/2/3/4')
    print('=' * 62)
    print()
    print('这一步要**一个一个来**：每次只接一个电机到总线上。')
    print('（因为广播读 ID 不能有别的电机在总线上抢答）')
    print()

    for target in (1, 2, 3, 4):
        hint = WHEEL_HINT.get(target, '')
        while True:
            input('请把「%s」那台电机**单独**接到总线上，接好按回车...' % hint)
            a = bus.read_id(timeout=0.6)
            if a is None:
                print('  读不到，检查接线和供电，重来')
                continue
            print('  读到地址 = %d' % a)
            if a == target:
                print('  ✓ 已经是 %d，跳过' % target)
                break
            if bus.set_id(a, target, store=True):
                time.sleep(0.3)
                if bus.is_alive(target):
                    print('  ✓ 已改成 %d' % target)
                    break
            print('  改地址失败，重来')
        print()

    print('四个电机都设好了。接下来接回扩展板，跑 scan 验证：')
    print('    python3 %s --port %s scan' % (sys.argv[0], args.port))
    return True


def cmd_gear(bus, args):
    """读 / 设 4 个电机的减速比。

    ⚠️ 这一步**比设地址还重要**：手册 5.5.7 说设置减速比后，
       所有速度和位置都会按它自动缩放，对应**输出轴**的量。
       不设的话默认 1.00，你收到的是电机轴转速 —— 车速会差 18.8 倍。
    """
    addrs = [1, 2, 3, 4]
    names = ['FL', 'FR', 'RL', 'RR']

    if args.ratio is None:
        print('读取 4 个电机的减速比（驱动器里存的值）')
        print('-' * 58)
        for n, a in zip(names, addrs):
            g = bus.read_gear_ratio(a)
            if g is None:
                print('  %-4s 地址 %d   ✗ 读不到' % (n, a))
            else:
                flag = '  ✓' if abs(g - 18.8) < 0.005 else '  ← 不是 18.8！'
                print('  %-4s 地址 %d   %.2f%s' % (n, a, g, flag))
        print('-' * 58)
        print('如果不是 18.80，跑：python3 %s --port %s gear 18.8'
              % (sys.argv[0], args.port))
        return

    ratio = args.ratio
    print('把 4 个电机的减速比设成 %.2f ...' % ratio)
    print('-' * 58)
    for n, a in zip(names, addrs):
        if not bus.is_alive(a):
            print('  %-4s 地址 %d   ✗ 无应答，跳过' % (n, a))
            continue
        ok = bus.set_gear_ratio(a, ratio, store=True)
        time.sleep(0.1)
        back = bus.read_gear_ratio(a)
        good = back is not None and abs(back - ratio) < 0.005
        print('  %-4s 地址 %d   %s  读回 %.2f'
              % (n, a, '✓' if (ok and good) else '✗', back if back else -1))
    print('-' * 58)
    print('已写入 flash，掉电不丢。')
    print()
    print('接着设一下输出轴最大转速对应的电流限制（可选，防堵转）：')
    print('    python3 %s --port %s health' % (sys.argv[0], args.port))


def cmd_health(bus, args):
    print('电机体检')
    print('-' * 62)
    print('%-6s %-8s %-10s %-14s %-8s' % ('轮位', '地址', '在线', '转速(RPM)', '温度'))
    print('-' * 62)
    for name, addr in zip(('FL', 'FR', 'RL', 'RR'), (1, 2, 3, 4)):
        alive = bus.is_alive(addr)
        if alive:
            rpm = bus.read_speed(addr)
            temp = bus.read_temperature(addr)
            print('%-6s %-8d %-10s %-14s %-8s'
                  % (name, addr, '✓', '%.1f' % rpm if rpm is not None else '—',
                     '%s℃' % temp if temp is not None else '—'))
        else:
            print('%-6s %-8d %-10s %-14s %-8s' % (name, addr, '✗ 无应答', '—', '—'))
    print('-' * 62)
    print('温度超过 60℃ 要留意，80℃ 以上会触发保护。')


def main():
    ap = argparse.ArgumentParser(
        description='ZDT_M370 电机地址检测/设置工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument('--port', default='/dev/ttyUSB0',
                    help='串口设备（底盘用，不是舵机那个）')
    ap.add_argument('--baud', type=int, default=115200,
                    help='波特率，出厂默认 115200')
    ap.add_argument('-v', '--verbose', action='store_true')

    sub = ap.add_subparsers(dest='cmd', required=True)

    p = sub.add_parser('scan', help='扫描总线上的电机地址')
    p.add_argument('--from', dest='scan_from', type=int, default=1)
    p.add_argument('--to', dest='scan_to', type=int, default=16)
    p.set_defaults(func=cmd_scan)

    p = sub.add_parser('readid', help='广播读取单个电机的 ID')
    p.set_defaults(func=cmd_readid)

    p = sub.add_parser('setid', help='修改电机 ID')
    p.add_argument('old', type=int, help='原地址')
    p.add_argument('new', type=int, help='新地址 (1-255)')
    p.set_defaults(func=cmd_setid)

    p = sub.add_parser('setup', help='交互式把 4 个电机设成 1/2/3/4')
    p.set_defaults(func=cmd_setup)

    p = sub.add_parser('health', help='体检：连通性/转速/温度')
    p.set_defaults(func=cmd_health)

    p = sub.add_parser('gear', help='读/设减速比（上电必做，默认 18.8）')
    p.add_argument('ratio', type=float, nargs='?', default=None,
                   help='要设成的减速比，如 18.8。不填则只读')
    p.set_defaults(func=cmd_gear)

    args = ap.parse_args()

    try:
        bus = ZDTBus(args.port, args.baud, verbose=args.verbose)
    except ZDTError as exc:
        sys.exit('打开串口失败: %s' % exc)

    print('串口 %s @ %d' % (args.port, args.baud))
    print()
    try:
        args.func(bus, args)
    except KeyboardInterrupt:
        print('\n中断')
    finally:
        bus.close()


if __name__ == '__main__':
    main()
