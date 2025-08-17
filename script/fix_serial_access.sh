#!/bin/bash
set -e

# 1) 把当前用户加入 dialout
echo "[1/4] Add user '$USER' to dialout group (if not already)."
sudo usermod -aG dialout "$USER" || true

# 2) 写入 udev 规则（固定名 + 权限 + 屏蔽 ModemManager）
echo "[2/4] Install udev rule."
sudo tee /etc/udev/rules.d/99-usb-speechNode.rules >/dev/null <<'RULE'
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
  SYMLINK+="ttyUSB_Speech", \
  GROUP="dialout", MODE="0660", \
  ENV{ID_MM_DEVICE_IGNORE}="1"
RULE

# 3) 重新加载规则并触发
echo "[3/4] Reload udev rules."
sudo udevadm control --reload-rules
sudo udevadm trigger

# 4) 提示用户重新登录让组生效
echo "[4/4] Done."
echo ">>> 请重新登录此用户会话 (或者运行 'newgrp dialout') 以应用组更改。"
echo ">>> 然后重新插入USB设备并检查:  ls -l /dev/ttyUSB_Speech  &&  ls -l /dev/serial/by-id/"
