#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mission_controller —— 集中编排层。

这是「主机」跑的那一半。它只做一件事：

    广播阶段指令 → 等 4 台车都报到位 → 再广播下一步

它**不判断**「车走到哪了」—— 那是每台车自己的事。主机只是个**计数器**。
这样状态机极简单，而且加第 5 台车不用改这里。

两个容易做错的地方，这里都处理了：

1. **主机也要周期重发指令**。不光是车要重发状态 —— 你发 GO 的那一包
   也可能丢。丢了一个车收不到，它就永远不动，主机等不到它。
2. **必须有超时兜底**。任何一台车出问题，没有超时的话 4 台会一起僵住。

话题
----
发布  /mission/state   std_msgs/String   广播给所有车
订阅  /carN/status     std_msgs/String   各车上报
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ---- 编排层自己的阶段 ----
IDLE = 'IDLE'
WAIT_ALIGNED = 'WAIT_ALIGNED'
WAIT_DOCKED = 'WAIT_DOCKED'
DONE = 'DONE'
ABORTED = 'ABORTED'

# ---- 车辆上报的状态字符串 ----
# ⚠️ 这跟上面的「编排阶段」是**两套不同的东西**，别混用：
#    阶段是主机自己在哪一步；下面是车告诉主机它是哪一步。
#    名字恰好有一对重名（ALIGNED/DOCKED），所以加了 CAR_ 前缀区分。
CAR_ALIGNED = 'ALIGNED'
CAR_DOCKED = 'DOCKED'


class MissionController(Node):

    def __init__(self):
        super().__init__('mission_controller')

        self.declare_parameter('car_ids', [1, 2, 3, 4])
        # ⚠️ ROS 2 参数**不支持嵌套数组** —— 写 [[1,2],[3,4]] 会直接
        #    TypeError: not a list of one of the allowed types。
        #    所以配对关系用扁平列表表示，两两一组：(1,2) (3,4)
        self.declare_parameter('pairs_flat', [1, 2, 3, 4])
        self.declare_parameter('mission.mission_timeout', 30.0)
        self.declare_parameter('broadcast_period', 1.0)
        self.declare_parameter('autostart_delay', 3.0)   # 启动后等多久自动发 GO
        self.declare_parameter('autostart', True)

        g = self.get_parameter
        self.car_ids = [int(i) for i in g('car_ids').value]
        _flat = [int(x) for x in g('pairs_flat').value]
        self.pairs = [_flat[i:i + 2] for i in range(0, len(_flat) - 1, 2)]
        self.timeout = float(g('mission.mission_timeout').value)
        self.broadcast_period = float(g('broadcast_period').value)
        self.autostart = bool(g('autostart').value)
        self.autostart_delay = float(g('autostart_delay').value)

        # ---------------- 状态记录 ----------------
        self.stage = IDLE
        self.status = {i: None for i in self.car_ids}
        self.stage_since = self.get_clock().now()
        self.start_time = self.get_clock().now()

        # ---------------- 接口 ----------------
        self.pub = self.create_publisher(String, '/mission/state', 10)
        for i in self.car_ids:
            self.create_subscription(
                String, '/car%d/status' % i,
                lambda msg, car=i: self.on_status(car, msg), 10)

        self.create_timer(0.2, self.tick)                       # 评估用，5Hz
        self.create_timer(self.broadcast_period, self._rebroadcast)

        self.get_logger().info(
            '编排层就绪 | 车辆 %s | 配对 %s | 超时 %.0fs | %s'
            % (self.car_ids, self.pairs, self.timeout,
               '%.0fs 后自动开始' % self.autostart_delay if self.autostart else '等待手动触发'))

    # ============================================================
    #  输入
    # ============================================================
    def on_status(self, car, msg):
        st = msg.data.strip().upper()
        if self.status.get(car) != st:
            self.get_logger().info('car%d -> %s' % (car, st))
        self.status[car] = st

    # ============================================================
    #  主循环
    # ============================================================
    def tick(self):
        now = self.get_clock().now()

        if self.stage == IDLE:
            if not self.autostart:
                return
            if self._elapsed(self.start_time) < self.autostart_delay:
                return
            self._enter(WAIT_ALIGNED, 'GO')
            return

        if self.stage in (WAIT_ALIGNED, WAIT_DOCKED):
            want = CAR_ALIGNED if self.stage == WAIT_ALIGNED else CAR_DOCKED
            if self._all_report(want):
                if self.stage == WAIT_ALIGNED:
                    self._enter(WAIT_DOCKED, 'EXTEND')
                else:
                    self._enter(DONE, None)
                return

            # 超时兜底 —— 没有这个，一台车出问题 4 台一起僵住
            waited = self._elapsed(self.stage_since)
            if waited > self.timeout:
                missing = [i for i in self.car_ids if self.status.get(i) != want]
                self.get_logger().error(
                    '超时 %.0fs：等 %s，但 car%s 还没到位（当前 %s）→ ABORT'
                    % (waited, want, missing,
                       {i: self.status.get(i) for i in missing}))
                self._enter(ABORTED, 'ABORT')

    # ============================================================
    #  工具
    # ============================================================
    def _all_report(self, want):
        return all(self.status.get(i) == want for i in self.car_ids)

    def _enter(self, stage, broadcast):
        self.stage = stage
        self.stage_since = self.get_clock().now()
        if broadcast:
            self.get_logger().info('=== 广播 %s ===' % broadcast)
            self._broadcast(broadcast)
        else:
            self.get_logger().info('=== %s ===' % stage)

    def _broadcast(self, cmd):
        m = String()
        m.data = cmd
        self.pub.publish(m)

    def _rebroadcast(self):
        """周期性重发当前阶段的指令。

        不这么做的话：发 GO 时某台车恰好丢包 → 它永远停在 READY →
        主机永远等不到它报 ALIGNED → 整个任务卡死（直到超时 ABORT）。
        """
        if self.stage == WAIT_ALIGNED:
            self._broadcast('GO')
        elif self.stage == WAIT_DOCKED:
            self._broadcast('EXTEND')

    def _elapsed(self, since):
        return (self.get_clock().now() - since).nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MissionController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
