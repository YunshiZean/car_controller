#!/usr/bin/env python3
#coding=utf-8

"""
文件名: battery_check.py
简介： 电池点压检测节点
作者： 未定义实验室.Zean 罗灵轩
版本： 0.3.3
说明： 监测剩余电量
更新内容： 
创建时间： 2025.8.6
最后更新时间： 2025.8.7
"""


import rospy
from turn_on_wheeltec_robot.msg import VehicleStatus
from std_msgs.msg import String

def power_check(msg):
    rate.sleep()
    power_level = ""
    if msg.battery_voltage >= power_full_value:
        power_level = "full"
    elif msg.battery_voltage >= power_half_value:
        power_level = "enough"
    elif msg.battery_voltage >= power_attention_value:
        power_level = "attention"
    elif msg.battery_voltage >= power_low_value:
        power_level = "low"
    message = String()
    message.data = power_level
    battery_level_pub.publish(message)




if __name__ == "__main__":
    rospy.init_node("power_check")
    battery_level_pub = rospy.Publisher("/power_level", String, queue_size=10)
    rospy.Subscriber("/vehicle_status",VehicleStatus,power_check,queue_size=10)

    power_low_value = 10.0
    power_half_value = 11.75
    power_full_value = 12.6
    power_attention_value = 10.9

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        rospy.spin()