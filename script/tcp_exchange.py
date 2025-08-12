#!/usr/bin/env python3
#coding=utf-8

"""
文件名: tcp_exchange.py
简介：网络数据交换节点
作者： 未定义实验室.Zean 罗灵轩
版本： 0.4.2
说明： 
更新内容： 修复了漏洞
创建时间： 2025.8.5
最后更新时间： 2025.8.6
"""


import socket
import threading
import time
import re
import rospy
from std_msgs.msg import String

class CarClientROS:
    def __init__(self, car_id="base_car_name", broadcast_port=9999):
        self.car_id = car_id
        self.broadcast_port = broadcast_port
        self.server_ip = None
        self.server_port = None
        self.listen_port = None
        self.send_port = None
        self.alive_port = None
        self.running = False
        self.broadcast_pattern = re.compile(r"SERVER:(\d+\.\d+\.\d+\.\d+):(\d+)")

        self.linkbreak = False #专门给发送信道给个标志位
        
        self.message_down_pub = rospy.Publisher("/message_down", String, queue_size=10)
        rospy.Subscriber("/message_up", String, self.message_up_callback, queue_size=10)

    def start(self):
        self.running = True
        threading.Thread(target=self.broadcast_listener, daemon=True).start()
        threading.Thread(target=self.alive_loop, daemon=True).start()
        threading.Thread(target=self.send_loop, daemon=True).start()
        threading.Thread(target=self.receive_loop, daemon=True).start()
        rospy.loginfo("[CarClient] 节点已启动，监听服务器广播...")

    def stop(self):
        self.running = False
        rospy.logwarn("[CarClient] 停止运行")

    def broadcast_listener(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('', self.broadcast_port))

        while self.running:
            try:
                data, _ = s.recvfrom(1024)
                msg = data.decode('utf-8').strip()
                match = self.broadcast_pattern.match(msg)
                if match:
                    ip, port = match.groups()
                    port = int(port)
                    if ip != self.server_ip or port != self.server_port:
                        self.server_ip = ip
                        self.server_port = port
                        self.listen_port = port + 1
                        self.alive_port = port + 2
                        self.send_port = port
                        rospy.loginfo(f"[发现服务器] {ip}:{port}")
            except Exception as e:
                rospy.logwarn(f"[广播监听错误] {e}")
                time.sleep(1)
        s.close()

    def alive_loop(self):
        while self.running:
            if not self.server_ip or not self.alive_port:
                time.sleep(1)
                continue
            try:
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    sock.settimeout(10)
                    sock.connect((self.server_ip, self.alive_port))
                    rospy.loginfo(f"[验证连接] 成功连接到 {self.server_ip}:{self.alive_port}")
                    self.alive_socket = sock
                    while self.running:
                        #要给这个哈儿加个这个，不然它不会重新连接
                        if self.linkbreak:
                            self.linkbreak = False
                            break
                        self.alive_socket.sendall("/alive".encode("utf-8"))
                        time.sleep(5)  # 空转，等待ROS回调触发发送 #延时，每隔五秒发一个/nothing
            except Exception as e:
                rospy.logwarn(f"[验证通道断开] {e}")
                self.linkbreak = True
                time.sleep(2)

    def send_loop(self):
        while self.running:
            if not self.server_ip or not self.send_port:
                time.sleep(1)
                continue
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(10)
                sock.connect((self.server_ip, self.send_port))
                rospy.loginfo(f"[发送连接] 成功连接到 {self.server_ip}:{self.send_port}")
                self.send_socket = sock
                while self.running:
                    if self.linkbreak: #如果断开了,就退出重联
                        self.linkbreak = False
                        raise Exception("连结断开")
                    time.sleep(5)  # 空转，等待ROS回调触发发送
            except Exception as e:
                rospy.logwarn(f"[发送通道断开] {e}")
                time.sleep(2)

    def receive_loop(self):
        while self.running:
            if not self.server_ip or not self.listen_port:
                time.sleep(1)
                continue
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                    sock.settimeout(10)
                    sock.connect((self.server_ip, self.listen_port))
                    rospy.loginfo(f"[接收连接] 成功连接到 {self.server_ip}:{self.listen_port}")
                    while self.running:
                        try:
                            data = sock.recv(1024)
                            if not data:
                                raise ConnectionError("服务端关闭连接")
                            message = data.decode("utf-8")
                            self.message_down_pub.publish(message)
                            rospy.loginfo(f"[接收] {message}")
                        except socket.timeout:
                            continue
                        except Exception as e:
                            rospy.logwarn(f"[接收错误] {e}")
                            break
            except Exception as e:
                rospy.logwarn(f"[接收通道连接失败] {e}")
                self.linkbreak = True
                time.sleep(2)

    def message_up_callback(self, msg: String):
        if hasattr(self, 'send_socket') and self.send_socket:
            try:
                self.send_socket.sendall(msg.data.encode())
                rospy.loginfo(f"[发送] {msg.data}")
            except Exception as e:
                rospy.logwarn(f"[发送失败] {e}")


if __name__ == '__main__':
    rospy.init_node("tcp_exchange")
    client = CarClientROS(car_id="Car_A1")
    client.start()
    rospy.spin()
    client.stop()
