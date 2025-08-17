#!/bin/bash

# 设置规则文件名和软链接名
RULE_FILE="/etc/udev/rules.d/99-usb-wheeltec.rules"
SYMLINK_NAME="ttyUSB_WHEELTEC"

# 创建规则内容
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="ttyUSB_Speech"' | sudo tee $RULE_FILE


# 重载规则并触发
echo "[INFO] udev规则写入完毕，正在重载并触发规则..."
sudo udevadm control --reload-rules
sudo udevadm trigger

# 检查是否生效
sleep 1
if [ -e /dev/$SYMLINK_NAME ]; then
    echo "[SUCCESS] 设备已成功链接为 /dev/$SYMLINK_NAME"
else
    echo "[WARNING] 软链接尚未出现，请检查设备是否插好，或尝试重新插拔设备。"
fi