#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""四车协同骨架 —— 纯逻辑自测（不需要 ROS，不需要硬件）。

用法：
    cd four_car_ws
    python selftest.py

只测「不依赖硬件」的部分：麦轮解算、里程计、卷尺标定。
这几块是后面所有逻辑的基础，先把它们验证对了再上硬件。
"""

import math
import os
import sys

# 把两个包加进 import 路径（还没 colcon build 也能跑）
HERE = os.path.dirname(os.path.abspath(__file__))
for pkg in ('car_base', 'car_arm'):
    sys.path.insert(0, os.path.join(HERE, 'src', pkg))

from car_base.mecanum import MecanumKinematics          # noqa: E402
from car_base.odometry import OdometryTracker           # noqa: E402
from car_arm.tape_driver import TapeCalibration         # noqa: E402

# ---------------- 测试框架（不依赖 pytest） ----------------
_pass = 0
_fail = 0


def check(name, got, want, tol=1e-6):
    global _pass, _fail
    if isinstance(got, (tuple, list)) and isinstance(want, (tuple, list)):
        ok = len(got) == len(want) and all(
            abs(g - w) <= tol for g, w in zip(got, want))
    else:
        ok = abs(got - want) <= tol
    if ok:
        _pass += 1
        print('  [OK]   %-46s %s' % (name, _fmt(got)))
    else:
        _fail += 1
        print('  [FAIL] %-46s 得到 %s，期望 %s' % (name, _fmt(got), _fmt(want)))


def check_true(name, cond, note=''):
    global _pass, _fail
    if cond:
        _pass += 1
        print('  [OK]   %-46s %s' % (name, note))
    else:
        _fail += 1
        print('  [FAIL] %-46s %s' % (name, note))


def _fmt(v):
    if isinstance(v, (tuple, list)):
        return '(' + ', '.join('%.4f' % x for x in v) + ')'
    return '%.4f' % v


def section(t):
    print('\n' + '=' * 68)
    print('  ' + t)
    print('=' * 68)


# ============================================================
#  测试参数（跟真实车不同也没关系，这里验的是数学）
# ============================================================
R = 0.05          # 轮半径 5cm
LX, LY = 0.12, 0.10
K = MecanumKinematics(R, LX, LY)
L = LX + LY


# ============================================================
section('1. 麦轮逆解算 —— 三种基本运动')
# ============================================================
print(f'  参数: r={R}, Lx={LX}, Ly={LY}, L={L}')

print('\n    - 只前进 (vx=1)：四个轮子应该一样快')
check('vx=1 → 全轮', K.inverse(1.0, 0.0, 0.0), (20.0, 20.0, 20.0, 20.0))

print('\n    - 只横移 (vy=1)：左侧和右侧反向')
check('vy=1 → FL,FR,RL,RR', K.inverse(0.0, 1.0, 0.0), (-20.0, 20.0, 20.0, -20.0))

print('\n    - 只自转 (wz=1)：左前右后 vs 右前左后')
# 注意 L = Lx + Ly = 0.22，所以 wz·L/r = 1×0.22/0.05 = 4.4
check('wz=1 → 左轮负、右轮正', K.inverse(0.0, 0.0, 1.0), (-4.4, 4.4, -4.4, 4.4))

print('\n    - 原地往左平移的本质：左边轮子往后、右边往前')


# ============================================================
section('2. 正解算 / 逆解算 往返一致')
# ============================================================
for vx, vy, wz in [(0.5, 0.0, 0.0), (0.0, 0.3, 0.0), (0.0, 0.0, 0.8),
                   (0.4, -0.2, 0.5)]:
    w = K.inverse(vx, vy, wz)
    back = K.forward(*w)
    check('往返 (%.1f, %.1f, %.1f)' % (vx, vy, wz), back, (vx, vy, wz), tol=1e-9)


# ============================================================
section('3. 限幅 —— 必须等比例缩放，不能截断')
# ============================================================
w = K.inverse(10.0, 0.0, 5.0)          # 故意超大
wc = K.clamp(w, 20.0)
check_true('峰值压到 20 以内', max(abs(x) for x in wc) <= 20.0 + 1e-9,
           '峰值 %.2f' % max(abs(x) for x in wc))

# 关键：缩放后方向不变（vx/wz 的比例要保持）
vx0, vy0, wz0 = K.forward(*w)
vx1, vy1, wz1 = K.forward(*wc)
ratio_ok = abs(wz0 / vx0 - wz1 / vx1) < 1e-9
check_true('缩放后 wz/vx 比例不变（方向没走歪）', ratio_ok,
           '前 %.4f → 后 %.4f' % (wz0 / vx0, wz1 / vx1))

w_zero = K.clamp((0.0, 0.0, 0.0, 0.0), 20.0)
check_true('零速不会被除零', w_zero == (0.0, 0.0, 0.0, 0.0))


# ============================================================
section('4. 里程计 —— 直行 3 米（本项目的第一个关键条件）')
# ============================================================
odo = OdometryTracker(K, use_imu_yaw=True)
dt, speed, seconds = 0.02, 0.5, 6.0          # 0.5 m/s × 6s = 3.0 m
for _ in range(int(seconds / dt)):
    odo.update(dt, K.inverse(speed, 0.0, 0.0), imu_yaw=0.0)

print('  ' + str(odo.as_dict()))
check('总路径长度', odo.path, 3.0, tol=1e-6)
check('x 位移', odo.x, 3.0, tol=1e-6)
check('y 位移（不该偏）', odo.y, 0.0, tol=1e-9)
# 注意：不要断言 driven(3.0) 恰好为真 —— 浮点累加 300 次后是 2.999999999999997，
# 卡在 >= 的边界上。真机上路径每 tick 涨约 0.01，跨过阈值后的下一 tick 就是 True，
# 边界本身是「测度为零」的事件，不该拿它当断言。
check_true('driven(2.99) 判定为真（刚过阈值）', odo.driven(2.99))
check_true('driven(3.01) 判定为假（还差一点）', not odo.driven(3.01))


# ============================================================
section('5. 里程计 —— 原地转 90 度（第二个关键条件）')
# ============================================================
odo.reset()
wz = math.pi / 2 / 2.0                        # 2 秒转 90°
for i in range(int(2.0 / dt)):
    yaw = wz * (i + 1) * dt                   # 模拟 IMU 报的 yaw
    odo.update(dt, K.inverse(0.0, 0.0, wz), imu_yaw=yaw)

print('  ' + str(odo.as_dict()))
check('转过的角度 (度)', odo.heading_deg, 90.0, tol=1e-6)
check_true('headed_to(90, tol=2) 判定为真', odo.headed_to(90.0, 2.0))
check_true('headed_to(0, tol=2) 判定为假', not odo.headed_to(0.0, 2.0))

print('\n    - 航向误差要分正负（状态机靠它判断往哪边转）')
check('当前 90°，目标 0° 的误差', odo.heading_error_deg(0.0), 90.0, tol=1e-6)
check('当前 90°，目标 180° 的误差', odo.heading_error_deg(180.0), -90.0, tol=1e-6)
check('当前 90°，目标 91° 的误差', odo.heading_error_deg(91.0), -1.0, tol=1e-6)


# ============================================================
section('6. 卷尺 —— 通电时间估算（给实测一个起点）')
# ============================================================
# 机构：JGY-370 电机（24 RPM）→ L298N → 卷盘放卷
# 控制方式：**通电 N 秒就停**。N 由实测确定，这里只是估个初值。
cal = TapeCalibration(full_radius_mm=30.0, core_radius_mm=10.0)
print('  ' + cal.report().replace('\n', '\n  '))
print()

check('中点半径 (R+r)/2', cal.mean_radius_mm, 20.0)
check('每圈放出 mm = 2π×中点半径', cal.mm_per_output_rev(), 2 * math.pi * 20, tol=0.01)

print('\n    - 24 RPM → 每圈 2.5 秒，据此估「要通电多少秒」')
check('伸出 400mm 约需秒数',
      cal.estimate_seconds(400.0, 24.0),
      400.0 / (2 * math.pi * 20) * 2.5, tol=1e-6)
check('伸出 200mm 约需秒数',
      cal.estimate_seconds(200.0, 24.0),
      200.0 / (2 * math.pi * 20) * 2.5, tol=1e-6)
check_true('距离翻倍时间也翻倍（线性关系）',
           abs(cal.estimate_seconds(400.0) - 2 * cal.estimate_seconds(200.0)) < 1e-9)

print('\n    - 时间参数的调整粒度')
_step = cal.estimate_seconds(10.0, 24.0)      # 多伸 10mm 需要多加多少秒
check_true('每 10mm ≈ 0.2 秒，时间参数够精细', 0.15 < _step < 0.25,
           '10mm ≈ %.3f 秒' % _step)


# ============================================================
section('7. 实测对表 —— 秒数 <-> 圈数 <-> 长度')
# ============================================================
print('  测出「伸出到位用了 N 秒」后，反推转了多少圈、放出多少毫米：')
print()
print('  %-10s %-12s %-14s' % ('通电秒数', '转了多少圈', '理论上放出'))
print('  ' + '-' * 40)
for sec in (2.5, 5.0, 7.5, 10.0, 20.0):
    revs = sec / 2.5                       # 24 RPM → 2.5 秒/圈
    print('  %-10.1f %-12.2f %-14.1f'
          % (sec, revs, revs * cal.mm_per_output_rev()))
print()

check('7.5 秒 → 3 圈', 7.5 / 2.5, 3.0)
check('3 圈放出长度', 3.0 * cal.mm_per_output_rev(),
      3 * 2 * math.pi * 20, tol=0.01)

print('  (注意) "理论上放出"只是估算 —— 卷尺有厚度、卷径会变、电池电压影响转速。')
print('     所以还是按你说的：**时间一点点加，实测为准**。')


# ============================================================
section('结果')
# ============================================================
total = _pass + _fail
print('  通过 %d / %d' % (_pass, total))
if _fail == 0:
    print('  [PASS] 全部通过 —— 纯逻辑层可用，可以上硬件了')
    sys.exit(0)
else:
    print('  [FAIL] 有 %d 项失败，先修这些再往下走' % _fail)
    sys.exit(1)
