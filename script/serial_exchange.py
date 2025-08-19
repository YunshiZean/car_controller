#!/usr/bin/env python3
#coding=utf-8

"""
文件名: serial_exchange.py
简介：串口数据交换节点
作者： 未定义实验室.Zean 罗灵轩
版本： 1.3
说明： 实现 /message_up → 串口 → /message_down 的实时桥接
更新内容： 新增了指令筛选，不是他喜欢的指令，直接拒绝转发
创建时间： 2025.8.6
最后更新时间： 2025.8.6
"""

import rospy
import serial
import threading
import time
from std_msgs.msg import String

class SerialBridge:
    def __init__(self):
        self.port = rospy.get_param("~port", "/dev/ttyUSB_Speech")  # "~"表示是私有命名空间
        self.baudrate = rospy.get_param("~baudrate", 115200) 
        self.port = self.port
        self.baudrate = self.baudrate
        self.serial_conn = None
        self.running = True
        self.serial_lock = threading.Lock()
        self.connected = False

        self.pub = rospy.Publisher('/message_down', String, queue_size=10)
        rospy.Subscriber('/message_up', String, self.forward_to_serial)

        threading.Thread(target=self.connection_loop, daemon=True).start()

    def forward_to_serial(self, msg: String):
        if self.connected:
            try:
                with self.serial_lock:
                    message = msg.data
                    if message == "/shut_up":
                        self.serial_conn.write(message.encode("utf-8"))
                    if " " in message:
                        A,_ = message.split(" ",1)
                        if A == "/arrive":
                            self.serial_conn.write((message + '\n').encode('utf-8'))
                            rospy.loginfo(f"[serial_exchange]接收到指令/arrive,转发成功!内容：\n{message}")
                        elif A == "/report":
                            self.serial_conn.write((message + '\n').encode('utf-8'))
                            rospy.loginfo(f"[serial_exchange]接收到指令/report,转发成功!内容：\n{message}")
                        else:
                            rospy.loginfo(f"[serial_exchange]接收到未知指令，不转发，内容为：\n{message}")
            except Exception as e:
                rospy.logerr(f"[串口写入失败] {e}")
                self.connected = False

    def connection_loop(self):
        while self.running and not rospy.is_shutdown():
            if not self.connected:
                try:
                    self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=0.1)
                    self.connected = True
                    rospy.logwarn(f"[串口连接成功] {self.port}")
                    threading.Thread(target=self.read_loop, daemon=True).start()
                except Exception as e:
                    rospy.logerr(f"[串口连接失败] {e}，继续尝试...")
                    time.sleep(2)
            else:
                time.sleep(1)

    def read_loop(self):
        try:
            while self.running and self.connected and not rospy.is_shutdown():
                try:
                    if self.serial_conn.in_waiting:
                        line = self.serial_conn.readline().decode('utf-8').strip()
                        if line:
                            self.pub.publish(line)
                            rospy.loginfo(f"[串口接收] {line}")
                except Exception as e:
                    rospy.logerr(f"[串口读取异常] {e}")
                    self.connected = False
                    break
        finally:
            self.connected = False
            rospy.logwarn("[串口监听线程退出]")

if __name__ == '__main__':
    rospy.init_node("serial_exchange")
    SerialBridge()
    rospy.spin()
