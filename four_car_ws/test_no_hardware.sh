#!/bin/bash
# ============================================================
#  硬件没到时的自检脚本
#
#  用法:
#      cd ~/four_car_ws
#      bash test_no_hardware.sh
#
#  它只做不需要硬件的事：逻辑自测、节点启动、接口连通性、仿真流程。
#  每一项都会打 [OK] 或 [FAIL]，最后给个总结。
# ============================================================

# 关闭交互提示，用独立的 DDS 域，避免和正在跑的东西互相干扰
export ROS_DOMAIN_ID=90
PASS=0; FAIL=0

ok()   { echo "  [OK]   $1"; PASS=$((PASS+1)); }
bad()  { echo "  [FAIL] $1"; FAIL=$((FAIL+1)); }
head_() { echo; echo "============================================================"; echo "  $1"; echo "============================================================"; }

cleanup() { pkill -f base_node 2>/dev/null; pkill -f arm_node 2>/dev/null
            pkill -f agent_node 2>/dev/null; pkill -f mission_node 2>/dev/null
            pkill -f 'topic pub' 2>/dev/null; sleep 0.5; }
trap cleanup EXIT

# ------------------------------------------------------------
head_ "0. 环境"
# ------------------------------------------------------------
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null
fi
if [ -z "$ROS_DISTRO" ]; then
    bad "ROS 2 环境没加载（先 source /opt/ros/jazzy/setup.bash）"; exit 1
fi
ok "ROS_DISTRO=$ROS_DISTRO"

if [ -f install/setup.bash ]; then
    source install/setup.bash
    ok "工作空间已加载"
else
    bad "没找到 install/setup.bash —— 先 colcon build"; exit 1
fi

if python3 -c "import gpiozero" 2>/dev/null; then
    ok "gpiozero 已装（真机 GPIO 需要）"
else
    echo "  [警告] gpiozero 没装 —— 仿真模式不受影响，真机要装："
    echo "         sudo apt install -y python3-gpiozero python3-lgpio"
fi

# ------------------------------------------------------------
head_ "1. 纯逻辑自测（麦轮/里程计/卷尺估算）"
# ------------------------------------------------------------
if OUT=$(python3 selftest.py 2>&1); then
    N=$(echo "$OUT" | grep -c '\[OK\]')
    ok "selftest 通过（$N 项）"
else
    bad "selftest 失败 —— 详情："
    echo "$OUT" | grep '\[FAIL\]' | head -5
fi

# ------------------------------------------------------------
head_ "2. 四个节点能不能起来（仿真模式）"
# ------------------------------------------------------------
for spec in "car_base base_node" "car_arm arm_node" "car_agent agent_node"; do
    set -- $spec
    cleanup
    timeout 12 ros2 run $1 $2 --ros-args -p simulate:=true > /tmp/_t.log 2>&1 &
    sleep 4
    if grep -q '就绪' /tmp/_t.log && ! grep -qE 'Traceback|ERROR' /tmp/_t.log; then
        ok "$2 启动正常"
    else
        bad "$2 启动异常："
        grep -E 'Traceback|ERROR' /tmp/_t.log | head -3
    fi
done

cleanup
timeout 12 ros2 run mission_controller mission_node --ros-args -p autostart:=false > /tmp/_t.log 2>&1 &
sleep 4
if grep -q '就绪' /tmp/_t.log; then ok "mission_node 启动正常"
else bad "mission_node 启动异常"; fi

# ------------------------------------------------------------
head_ "3. 接口连通性"
# ------------------------------------------------------------
# 3a. 底盘：cmd_vel -> odom
cleanup
timeout 25 ros2 run car_base base_node --ros-args -p simulate:=true > /tmp/_b.log 2>&1 &
sleep 4
timeout 15 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" >/dev/null 2>&1 &
sleep 3
X=$(timeout 8 ros2 topic echo --once /odom 2>/dev/null | grep -A3 'position:' | grep 'x:' | head -1 | awk '{print $2}')
pkill -f 'topic pub' 2>/dev/null
if [ -n "$X" ] && python3 -c "import sys; sys.exit(0 if float('$X') > 0.1 else 1)" 2>/dev/null; then
    ok "cmd_vel -> odom 有反应（x=$X）"
else
    bad "cmd_vel -> odom 没反应（读到 x='$X'）"
fi

# 3b. 上层：arm_cmd -> arm_state
cleanup
timeout 25 ros2 run car_arm arm_node --ros-args -p simulate:=true > /tmp/_a.log 2>&1 &
sleep 4
timeout 6 ros2 topic pub --once /arm_cmd std_msgs/msg/String "{data: 'EXTEND'}" >/dev/null 2>&1
sleep 3
if grep -q 'DOCKED' /tmp/_a.log; then ok "arm_cmd EXTEND -> 走到 DOCKED"
else bad "arm_cmd EXTEND 没走完（看 /tmp/_a.log）"; fi

# 3c. 状态机：/mission/state -> 状态切换
cleanup
timeout 20 ros2 run car_agent agent_node --ros-args -p simulate:=true > /tmp/_g.log 2>&1 &
sleep 4
timeout 6 ros2 topic pub --once /mission/state std_msgs/msg/String "{data: 'GO'}" >/dev/null 2>&1
sleep 3
if grep -q 'READY -> MOVING' /tmp/_g.log; then ok "/mission/state GO -> READY->MOVING"
else bad "状态机没响应 GO"; fi

# ------------------------------------------------------------
head_ "4. 参数系统"
# ------------------------------------------------------------
cleanup
timeout 25 ros2 run car_base base_node --ros-args -p simulate:=true > /dev/null 2>&1 &
sleep 5
NP=$(timeout 6 ros2 param list /base_controller 2>/dev/null | wc -l)
if [ "$NP" -gt 10 ]; then ok "参数可列出（$NP 个）"; else bad "参数列不出来"; fi
if timeout 6 ros2 param set /base_controller base.max_wheel_rad_s 8.0 2>&1 | grep -q successful; then
    ok "参数可在线修改"
else
    bad "参数改不了"
fi

# ------------------------------------------------------------
head_ "5. 完整任务流程（4 台假车 + 主机）"
# ------------------------------------------------------------
cleanup
timeout 50 ros2 launch four_car_bringup sim.launch.py > /tmp/_s.log 2>&1
if grep -q '=== DONE ===' /tmp/_s.log; then
    ok "仿真全流程走通（GO -> EXTEND -> DONE）"
    for st in '广播 GO' '广播 EXTEND' 'DONE'; do
        grep -q "$st" /tmp/_s.log && echo "        经过: $st"
    done
else
    bad "仿真流程没跑完，最后 5 行："
    tail -5 /tmp/_s.log
fi

# ------------------------------------------------------------
head_ "6. 硬件不在时的报错（应该干净，不是 traceback）"
# ------------------------------------------------------------
cleanup
OUT=$(timeout 10 ros2 run car_base base_node 2>&1)
cleanup
if echo "$OUT" | grep -q '底盘初始化失败'; then
    ok "缺串口时报错清晰"
    echo "$OUT" | grep -E 'FATAL' | head -2 | sed 's/^/        /'
elif echo "$OUT" | grep -q 'Traceback'; then
    bad "缺串口时甩了 Python traceback（应该更友好）"
else
    bad "缺串口时没给出明确提示"
fi

# ------------------------------------------------------------
head_ "总结"
# ------------------------------------------------------------
TOTAL=$((PASS+FAIL))
echo "  通过 $PASS / $TOTAL"
if [ "$FAIL" -eq 0 ]; then
    echo
    echo "  全部通过 —— 逻辑层和接口层没问题，可以等硬件了。"
    echo "  硬件到了之后按「使用手册」第三部分 A~I 阶段做真实标定。"
    exit 0
else
    echo
    echo "  有 $FAIL 项失败，上面的 [FAIL] 那几行是真问题。"
    exit 1
fi
