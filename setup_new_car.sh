#!/bin/bash
# ============================================================
#  新树莓派上车环境一键配置
#
#  用途：刚烧好的 Ubuntu 24.04 树莓派，跑这一个脚本就能变成
#        「四车协同」的一台车（或者任意一台新的开发机）
#
#  用法：
#      sudo bash setup_new_car.sh
#
#  做四件事：
#      1. Ubuntu 源换清华镜像（国内不换装不动 ROS）
#      2. 装 ROS 2 Jazzy（全程约 20~40 分钟）
#      3. 装本项目的 Python 依赖
#      4. 配串口权限（dialout 组 / 关 serial-getty / udev 规则）
#
#  跑完还要做（脚本会提示）：
#      · 退出重登（让 dialout 组生效）
#      · 传工作空间过去
#      · 设 CAR_ID
# ============================================================
set -u

log()  { echo; echo "########## $* ##########"; }
die()  { echo; echo "!!!!!!!!!! 失败: $* !!!!!!!!!!"; exit 1; }

[ "$(id -u)" -eq 0 ] || die "请用 sudo 运行: sudo bash setup_new_car.sh"

# 目标用户：sudo 的那个用户（脚本会写它的 .bashrc）
TARGET_USER="${SUDO_USER:-$(logname 2>/dev/null || echo zai)}"
TARGET_HOME=$(eval echo "~$TARGET_USER")
echo "目标用户: $TARGET_USER  (家目录 $TARGET_HOME)"

# ============================================================
log "[1/5] 检查系统和网络"
cat /etc/os-release | grep -E '^(NAME|VERSION)='
uname -m
echo
echo "外网连通性（清华镜像）:"
if curl -4 -sI --max-time 12 https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/dists/noble/Release 2>/dev/null | head -1; then
    echo "  ✓ 清华源可达"
else
    echo "  ! 清华源不可达 —— 检查网络（能不能 ping 通 192.168.1.1）"
    echo "    继续下去大概率会失败，建议先修网络。Ctrl+C 中止。"
    sleep 5
fi

# ============================================================
log "[2/5] Ubuntu 源换清华镜像"
SRC=/etc/apt/sources.list.d/ubuntu.sources
if [ -f "$SRC" ]; then
    [ -f "$SRC.orig" ] || cp "$SRC" "$SRC.orig"
    sed -i 's|http://ports.ubuntu.com/ubuntu-ports|https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports|g' "$SRC"
    grep '^URIs' "$SRC"
    echo "(原文件备份为 $SRC.orig)"
else
    echo "警告: 找不到 $SRC，跳过（可能不是标准 Ubuntu 结构）"
fi

apt update || apt update --allow-releaseinfo-change || die "apt update 失败"

# ============================================================
log "[3/5] 安装 ROS 2 Jazzy"
if [ -d /opt/ros/jazzy ]; then
    echo "已经装过了（/opt/ros/jazzy 存在），跳过"
else
    echo "--- 基础依赖 ---"
    apt install -y software-properties-common curl gnupg lsb-release \
        || die "基础依赖安装失败"

    echo "--- 导入 ROS GPG 密钥 ---"
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
         -o /usr/share/keyrings/ros-archive-keyring.gpg || die "密钥下载失败"

    echo "--- 添加清华 ROS 源 ---"
    echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu noble main" \
       > /etc/apt/sources.list.d/ros2.list
    apt update || die "加 ROS 源后 apt update 失败"

    echo "--- 装 ros-jazzy-ros-base（约 1GB，视网速 20~40 分钟）---"
    # 整车只需要通信和基础库，不装 desktop（那要 3GB 且树莓派跑不了 RViz）
    apt install -y ros-jazzy-ros-base ros-dev-tools \
        || die "ROS 安装失败"
fi

# ============================================================
log "[4/5] 安装本项目依赖"
apt install -y python3-serial python3-gpiozero python3-lgpio python3-colcon-common-extensions \
    || echo "  警告: 有包装失败，回头单独补"
echo
echo "验证:"
python3 -c "import serial; print('  pyserial OK')" 2>&1 | head -1
python3 -c "import gpiozero; print('  gpiozero OK')" 2>&1 | head -1
python3 -c "import lgpio; print('  lgpio OK')" 2>&1 | head -1

# ============================================================
log "[5/5] 串口配置"

echo "--- 启用 GPIO14/15 上的 UART（舵机用）---"
CFG=/boot/firmware/config.txt
if [ -f "$CFG" ]; then
    if ! grep -q '^enable_uart=1' "$CFG"; then
        [ -f "$CFG.orig" ] || cp "$CFG" "$CFG.orig"
        # 这三行跟第一台车保持一致（已实测可用）：
        #   enable_uart=1          启用 UART 硬件
        #   dtoverlay=uart0        把 uart0 映射到 GPIO14/15
        #   dtparam=uart0_console  声明为控制台设备（后面会关掉 getty）
        printf '\n# 四车协同：启用舵机串口 /dev/ttyAMA0\nenable_uart=1\ndtoverlay=uart0\ndtparam=uart0_console\n' >> "$CFG"
        echo "  已追加 3 行到 $CFG（原文件备份 $CFG.orig）"
        echo "  ⚠️ 需要重启才生效"
    else
        echo "  已经配过了，跳过"
    fi
else
    echo "  警告: 找不到 $CFG —— 树莓派 5 上应该在这个路径"
fi

echo "--- 加 dialout 组 ---"
usermod -aG dialout "$TARGET_USER" && echo "  $TARGET_USER 已加入 dialout"

echo "--- 关掉串口上的 getty（它会占住 /dev/ttyAMA0，导致权限变成 root:tty 无读）---"
for svc in serial-getty@ttyAMA0.service serial-getty@ttyS0.service; do
    if systemctl list-unit-files 2>/dev/null | grep -q "$svc"; then
        systemctl disable --now "$svc" 2>/dev/null && echo "  已停 $svc"
    fi
done

echo "--- udev 规则：把 ttyAMA0 固定成 dialout 组可读写 ---"
echo 'KERNEL=="ttyAMA0", GROUP="dialout", MODE="0660"' \
    > /etc/udev/rules.d/99-serial-servo.rules
udevadm control --reload-rules && udevadm trigger
echo "  已写入 /etc/udev/rules.d/99-serial-servo.rules"

echo "--- 环境变量：ROS 自动 source ---"
for f in "$TARGET_HOME/.bashrc"; do
    if [ -f "$f" ] && ! grep -q '/opt/ros/jazzy/setup.bash' "$f"; then
        printf '\n# ROS 2 Jazzy\nsource /opt/ros/jazzy/setup.bash\n' >> "$f"
        chown "$TARGET_USER:$TARGET_USER" "$f"
        echo "  已写入 $f"
    fi
done

# ============================================================
log "配置完成"
echo "机器信息:"
echo "  主机名 : $(hostname)"
echo "  IP     : $(hostname -I | awk '{print $1}')"
echo "  ROS    : $(ls /opt/ros/ 2>/dev/null || echo '未装')"
echo
echo "现存串口:"
ls /dev/ttyUSB* /dev/ttyACM* /dev/ttyAMA* 2>/dev/null | sed 's/^/  /' || echo "  (无)"
echo
echo "=========================================================="
echo "  接下来还要做（脚本做不了，需要你/我来）："
echo "=========================================================="
echo
echo "  ① 【必须】退出重登，让 dialout 组生效"
echo "         exit"
echo "         （然后重新 ssh 进来）"
echo "     验证：groups | grep dialout"
echo
echo "  ② 传工作空间过去（在第一台树莓派或电脑上执行）："
echo "         scp -r ~/four_car_ws $TARGET_USER@$(hostname -I | awk '{print $1}'):~/"
echo "     或者用 tar 更快："
echo "         cd ~ && tar czf - --exclude=__pycache__ --exclude=build --exclude=install --exclude=log four_car_ws \\"
echo "           | ssh $TARGET_USER@$(hostname -I | awk '{print $1}') 'cd ~ && tar xzf -'"
echo
echo "  ③ 编译 + 自测："
echo "         cd ~/four_car_ws && colcon build --symlink-install"
echo "         bash test_no_hardware.sh"
echo
echo "  ④ 设车号（这台是几号车就填几）："
echo "         echo 'export CAR_ID=2' >> ~/.bashrc"
echo
echo "  ⑤ 4 台车的 ROS_DOMAIN_ID 要一致："
echo "         echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc"
echo
