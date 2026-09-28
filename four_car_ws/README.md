# four_car_ws —— 四车协同 · 卷尺桥

ROS 2 工作空间。麦轮解算、里程计、放卷控制、状态机、集中编排均已实现并通过自测。

详细设计见上一级目录的 `架构方案.md`。

---

## 现在就能跑（不需要 ROS，不需要硬件）

```bash
cd four_car_ws
python selftest.py
```

验证麦轮解算、里程计、卷尺放卷三块数学。当前 **29/29 通过**。

---

## 跑一遍完整的任务流程（不需要硬件）

装好 ROS 2 后：

```bash
colcon build --symlink-install
source install/setup.bash

ros2 launch four_car_bringup sim.launch.py
```

四台车用**内部仿真的里程计**，你会看到完整的流转：

```
car1 -> MOVING -> TURNING -> ALIGNED
car2 -> MOVING -> TURNING -> ALIGNED
car3 -> TURNING -> ALIGNED          ← 后两台不直行
car4 -> TURNING -> ALIGNED
=== 广播 EXTEND ===
car1 -> EXTENDING -> APPROACHING -> DOCKED
...
=== DONE ===
```

**这一步验证的是「编排 + 状态机 + 时序」，与硬件无关。**
接上实车时驱动层已经就位，这套逻辑不用动。

---

## 目录

```
four_car_ws/
├── selftest.py                 ← 纯逻辑自测，现在就能跑
└── src/
    ├── car_base/               底盘层
    │   ├── mecanum.py          麦轮正逆解算
    │   ├── odometry.py         里程计 + IMU 融合
    │   ├── zdt_driver.py       ZDT_M370 协议帧
    │   ├── base_driver.py      四电机底盘驱动
    │   └── base_node.py        ROS 节点
    ├── car_arm/                上层
    │   ├── tape_driver.py      放卷控制 + 逼近逻辑
    │   ├── magnet.py           电磁铁 + 霍尔
    │   ├── servo_driver.py     总线舵机
    │   ├── l298n.py            H 桥驱动
    │   └── arm_node.py         ROS 节点
    ├── car_agent/              本地状态机
    ├── mission_controller/     集中编排
    └── four_car_bringup/
        ├── launch/sim.launch.py        仿真（无硬件）
        ├── launch/car.launch.py        单车（每台树莓派跑这个）
        ├── launch/mission.launch.py    编排层
        └── config/params.yaml          所有可调参数 ← 改这里，不碰代码
```

---

## 实机部署

**每台树莓派**（4 台都配一遍）：

```bash
# 1. 串口权限（车上两个串口：底盘 + 舵机，各配一次）
sudo usermod -aG dialout $USER          # 然后退出重登
sudo systemctl disable --now serial-getty@ttyAMA0.service
echo 'KERNEL=="ttyAMA0", GROUP="dialout", MODE="0660"' \
  | sudo tee /etc/udev/rules.d/99-serial.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# 2. 调度各车自己启动
export CAR_ID=2
ros2 launch four_car_bringup car.launch.py
```

**主机**（其中一台车，或上位机）：

```bash
ros2 launch four_car_bringup mission.launch.py
```

**跨机前提**：4 台车 + 主机必须设**同一个 `ROS_DOMAIN_ID`**，同一 WiFi。
建议做 DHCP 保留（固定 IP），并优先用 5GHz。

---

## 核心设计（见架构方案 §1）

**全程不用 `sleep` 做判据，全部用条件：**

```python
# ❌ 时间驱动              # ✅ 条件驱动
time.sleep(5)              wait_until(lambda: odo.driven(3.0))
turn_left(90)              wait_until(lambda: odo.headed_to(90, 2.0))
extend_tape()              wait_until(lambda: all_cars_report('ALIGNED'))
```

理由：电池电量、地面摩擦、每台车开机时间都不同，**同样时间走的距离不一样**，
而"两车面对面吸合"要求两边都对上。

**分工**：

| 谁 | 判断什么 | 靠什么 |
|---|---|---|
| 每台车自己 | 我到没到位 | 编码器里程 / IMU 航向 |
| 主机 | 大家到齐没 | 只数状态上报的个数 |

主机是**计数器**不是控制器 —— 所以加第 5 台车不用改主机逻辑。

---

## 两个必须记住的坑

**① 状态上报和指令广播都要周期重发**

不只是车要重发状态，**主机也要重发指令**。发 GO 的那一包丢给某台车，
它就一直停在 READY，主机永远等不齐 → 整个任务卡死。
代码里 `status_period` 和 `broadcast_period` 就是干这个的。

**② 必须有超时兜底**

`mission_timeout` 不是装饰。任何一台车出问题，没有超时的话 4 台会一起僵住
（要等 WiFi 重连或者人工干预）。超时后会广播 `ABORT`。

---

## 硬件对接参数

驱动层已按器件手册实现，实车到位后按 §8 清单标定两项：

- 底盘串口：波特率、帧格式、有无编码器回读
- 底盘板和舵机**必须分两个串口设备节点**
- 麦轮几何：轮半径 `r`、半长 `Lx`、半宽 `Ly` → `params.yaml`
- 卷尺放卷参数 → `params.yaml` 的 `tape_extend`
- PWM 电机的驱动方式、电磁铁驱动、霍尔型号
