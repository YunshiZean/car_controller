#!/usr/bin/env python3
#coding=utf-8

"""
文件名: central_manager.py
简介： 中央管理器
作者： 未定义实验室.Zean 罗灵轩
版本： 2.0.0
说明： 中央管理器
更新内容： 为osm地图做适配
创建时间： 2025.8.5
最后更新时间： 2025.8.17
"""

from enum import Enum, auto
import time
import rospy
import threading
from geometry_msgs.msg import PointStamped
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalID
import os
from typing import Callable
from WayManager import WaypointManager
from std_msgs.msg import String
import json

"""
状态枚举
"""
class RobotState(Enum):
    IDLE = auto() #空闲
    CRUISE = auto() #巡航
    TASK = auto() #任务
    CARRYING = auto() #载客
    INIT = auto() #初始化
    POWER = auto() #充电
    UNKNOWN = auto()

"""
状态机类
"""
class StateMachine:
    def __init__(self):
        self.state = RobotState.IDLE
        self.laststate = None

    def switch_state(self, state: RobotState):
        self.laststate = self.state
        self.state = state

"""
数据交换类
"""
class Exchange:
    """
        数据交换类
    """
    def __init__(self):
        self.msg_queue = [] #消息队列
        self.lock = threading.Lock()
        self.up_pub = rospy.Publisher("/message_up", String, queue_size=10)
        rospy.Subscriber("/message_down", String, self.recv_cb)

    def trans(self, msg_bytes: bytes):
        self.up_pub.publish(msg_bytes.decode("utf-8"))

    def recv_cb(self, msg: String):
        with self.lock:
            self.msg_queue.append(msg.data)

    def recv(self) -> str:
        with self.lock:
            if self.msg_queue:
                return self.msg_queue.pop(0)
        return ""

"""
指令第调度器类
"""
class CommandDispatcher:
    """
        指令第调度器类
    """
    def __init__(self):
        self.handlers = {}

    def register(self, prefix, handler):
        self.handlers[prefix] = handler

    def dispatch(self, command):
        for prefix, handler in self.handlers.items():
            if command.startswith(prefix):
                return handler(command)
        return f"/error 未知指令: {command}\n"
    
class CarBattery(Enum):
    FULL = auto() #满电
    ENOUGH = auto() #高于一半
    ATTENTION = auto() #低于一半
    LOW = auto() #过低
    UNKNOW = auto() #不知道

"""
描述小车关键运行信息的类，用来上报给服务器
"""
class Car_info:
    """
        描述小车关键运行信息的类，用来上报给服务器
    """
    def __init__(self):
        self.current_point = None #当前位置
        self.current_state = RobotState.UNKNOWN #当前状态
        self.last_state = RobotState.UNKNOWN #上一次状态
        self.current_path = "" #当前路线
        self.cruise_index = 0 #巡航下标
        self.task_queue = [] #任务队列
        self.power_level: CarBattery = CarBattery.UNKNOW

    def to_json(self):
        data = {
            "current_point": self.current_point,
            "current_state": self.current_state.name,  # 用字符串保存
            "last_state": self.last_state.name,
            "current_path": self.current_path,
            "cruise_index": self.cruise_index,
            "task_queue": self.task_queue,
            "power_level": self.power_level.name
        }
        return json.dumps(data)

    def from_json(self, json_data):
        data = json.loads(json_data)
        self.current_point = data.get("current_point")
        self.current_state = RobotState[data.get("current_state")]
        self.last_state = RobotState[data.get("last_state")]
        self.current_path = data.get("current_path")
        self.cruise_index = data.get("cruise_index")
        self.task_queue = data.get("task_queue")
        self.power_level = CarBattery[data.get("power_level")]
        if self.power_level is None: #刷新信息的时候电压值是空的，默认为未知
            self.power_level = CarBattery.UNKNOW

"""
主要的运行类
"""
class PatrolController:
    """
        主要的运行类
    """
    def __init__(self, exchange: Exchange):
        self.carinfo = Car_info()
        self.way = WaypointManager(os.path.join(os.path.dirname(__file__), "..", "config"))
        try:
            self.current_path = self.way.get_path("path_1")
        except Exception:
            rospy.logerr("巡航路径初始化失败，跳过")
        
        self.carinfo.current_path = self.current_path 
        self.cruise_index = 0
        self.current_goal = None
        self.task_queue = []

        rospy.init_node('central_manager')
        self.exchange = exchange
        self.goal_pub = rospy.Publisher("/clicked_point", PointStamped, queue_size=10) #这里作出更改
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        """
        这里这个取消目标的功能暂时先不使用，改了架构后要另外处理
        """
        #rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_callback)
        
        rospy.Subscriber('/power_level', String, self.power_level_callback)
        self.fsm = StateMachine()#创建状态机对象

        self.grong_vjug = False #夺舍标记


        # 命令调度器CD
        self.dispatcher = CommandDispatcher()
        # self.dispatcher.register("/cruise", self.handle_cruise)
        # self.dispatcher.register("/!cruise", self.handle_cruise_exit)
        # self.dispatcher.register("/report", self.handle_report)
        # self.dispatcher.register("/init", self.handle_init)
        # self.dispatcher.register("/pause", self.handle_pause)
        # self.dispatcher.register("/stop", self.handle_stop)
        self.dispatcher.register("/task", self.handle_task)
        self.dispatcher.register("/carry", self.handle_carry)
        # self.dispatcher.register("/continue", self.handle_continue)
        # self.dispatcher.register("/switch", self.handle_switch)
        # self.dispatcher.register("/info",self.handle_info)
        # self.dispatcher.register("/go_power",self.handle_go_power)

        threading.Thread(target=self.listen_loop, daemon=True).start()


    def listen_loop(self):
        while not rospy.is_shutdown():
            line = self.exchange.recv()
            if line:
                if self.fsm.state != RobotState.TASK:
                    rospy.loginfo("收到指令: %s", line)
                    response = self.dispatcher.dispatch(line)
                    if response:
                        rospy.loginfo(response)
                else:
                    rospy.logwarn("执行任务中，不接受其他命令")

    def handle_cruise(self, cmd=None):
        self.fsm.switch_state(RobotState.CRUISE)
        self.current_goal = self.current_path[self.cruise_index]
        self.publish_goal()
        return "/ack 开始巡航"

    def handle_cruise_exit(self, cmd=None):
        #防止出bug，退出巡航后直接初始化
        self.handle_init()
        return "/ack 退出巡航"

    def handle_report(self, cmd=None):
        current = self.current_goal if self.current_goal else "-1"
        size = len(self.task_queue)
        self.exchange.trans(f"/report {current}|{size}".encode("utf-8"))
        return f"/report {current}|{size}"

    def handle_init(self, cmd=None):
        self.handle_stop()
        self.fsm.laststate = None
        self.cruise_index = 0
        self.current_goal = self.current_path[self.cruise_index]
        return "/ack 初始化完成"

    def handle_pause(self, cmd=None):
        self.cancel_pub.publish(GoalID())
        return "/ack 已暂停\n"

    def handle_stop(self, cmd=None):
        self.fsm.switch_state(RobotState.IDLE)
        self.cancel_pub.publish(GoalID())
        self.current_goal = None
        return "/ack 已停止\n"

    def handle_task(self, cmd=None):
        try:
            _, key = cmd.split()
            if key in self.way.get_all_points(): #可以前往任何位置，所以要参照所有点
                self.fsm.switch_state(RobotState.TASK)
                self.current_goal = key
                self.publish_goal()
                return "/ack 开始执行任务\n"
            else:
                return f"/error 无效点名: {key}\n"
        except:
            return "/error 格式错误，应为 /task 点名\n"

    def handle_carry(self, cmd=None):
        try:
            _, key = cmd.split()
            if key in self.way.get_all_points(): #可以前往任何位置，所以要参照所有点
                self.task_queue.append(key)
                if self.fsm.state != RobotState.CARRYING:
                    self.fsm.switch_state(RobotState.CARRYING)
                    self.current_goal = self.task_queue[0]
                self.publish_goal()
                return f"/ack 已添加任务: {key}\n"
            else:
                self.handle_continue()
                return f"/error 无效点名: {key}\n"
        except Exception as e:
            return f"/error 格式错误，应为 /carry 点名\nexception: {e}\n"

    def handle_continue(self, cmd=None):
        self.publish_goal()
        return "/ack 已继续\n"

    def handle_switch(self, cmd=None):
        try:
            _, name = cmd.split()
            if name in self.way.get_all_paths():
                self.current_path = self.way.get_path(name)
                self.cruise_index = 0
                self.current_goal = self.current_path[0]
                return f"/ack 已切换路径: {name}\n"
            else:
                return f"/error 路径 {name} 不存在\n"
        except:
            return "/error 格式错误，应为 /switch 路径名\n"

    def handle_info(self, info:str):
        self.carinfo.from_json(info) #夺舍者信息载入躯壳
        if self.fsm.state == RobotState.POWER: #如果在充电线，则进行夺舍者信息载入
            self.car_load_info()
            self.publish_goal()
        else:
            self.grong_vjug = True #标记夺舍

    def handle_go_power(self, cmd=None):
        """
        小车前往充电站
        :param cmd: 无
        :return: 无
        """
        self.fsm.switch_state(RobotState.POWER)
        self.current_goal = "power"
        self.publish_goal()
        threading.Thread(target=self.power_loop, daemon=True).start()

    def power_loop(self):
        """
        充电循环，每隔10秒发送一次自身信息，以便服务器解读，并更新小车信息
        """
        while self.fsm.state == RobotState.POWER:
            self.car_get_info()
            self.exchange.trans(f"/info {self.carinfo.to_json()}".encode("utf-8"))
            time.sleep(10)

#######################################################################

    def car_get_info(self):
        """
        从小车获取信息，并更新carinfo
        注意：此函数在调用时会载入当前的状态，所以一定是在状态机切换前调用
        :return: 无
        """
        self.carinfo.current_point = self.current_goal #当前位置在到达一个点后在状态还没清除前就是当前目标
        self.carinfo.current_path = self.fsm.state
        self.carinfo.last_state = self.fsm.laststate
        self.carinfo.current_path = self.current_path
        self.carinfo.cruise_index = self.cruise_index
        self.carinfo.task_queue = self.task_queue

    def car_load_info(self):
        """
        将carinfo中的信息载入小车
        用于被夺舍后的小车载入信息
        :return: 无
        """
        self.current_goal = self.carinfo.current_point #小车前往夺舍者的位置
        self.fsm.laststate = self.carinfo.last_state
        self.fsm.state = self.carinfo.current_state
        self.current_path = self.carinfo.current_path
        self.cruise_index = self.carinfo.cruise_index
        self.task_queue = self.carinfo.task_queue

    def publish_goal(self):
        if self.current_goal and self.current_goal in self.way.get_all_points():
            goal = self.way.get_pose(self.current_goal)
            goal.header.stamp = rospy.Time.now()
            self.goal_pub.publish(goal)
            rospy.loginfo("导航至: %s", self.current_goal)
        else:
            rospy.logerr(f"点位：{self.current_goal} 不存在。\n[解决]状态重置为IDLE空闲\n[解决]目标初始化")
            self.fsm.switch_state(RobotState.IDLE)
            self.current_goal = None

    def result_callback(self, msg):
        if msg.status.status == 3:  # SUCCEEDED
            self.exchange.trans(f"/arrive {self.current_goal}\n".encode("utf-8"))

            #如果被夺舍，那么就不需要上传当前信息了，而是载入信息后开始行动
            if self.grong_vjug:
                self.car_load_info()
                self.grong_vjug = False
                self.publish_goal() #开始行动
                return #直接退出

            self.car_get_info()
            self._info =  self.carinfo.to_json()
            self.exchange.trans(f"/info {self._info}\n".encode("utf-8")) #数据上报 
            
            self.current_goal = None 

            if self.fsm.state == RobotState.CARRYING:
                if self.task_queue:
                    self.task_queue.pop(0)
                if self.task_queue:
                    self.current_goal = self.task_queue.pop(0)
                else:
                    if self.fsm.laststate == RobotState.CRUISE:
                        self.handle_cruise()
                        return
                    else:
                        self.handle_init()
                        return

            elif self.fsm.state == RobotState.CRUISE:
                self.cruise_index = (self.cruise_index + 1) % len(self.current_path)
                self.current_goal = self.current_path[self.cruise_index]

            elif self.fsm.state == RobotState.TASK:
                if self.fsm.laststate == RobotState.CRUISE:
                    self.current_goal = self.current_path[self.cruise_index]
                    self.fsm.switch_state(self.fsm.laststate)
                elif self.fsm.laststate == RobotState.CARRYING:
                    self.current_goal = self.task_queue[0]
                    self.fsm.switch_state(self.fsm.laststate)
                elif self.fsm.laststate == RobotState.POWER: #如果是充电状态，则不进行任何操作
                    self.fsm.switch_state(self.fsm.laststate) #进入充电状态
                    return
                else:
                    self.fsm.state = RobotState.IDLE
                    return
            self.publish_goal()
        elif msg.status.status == 4: #失败结束
            self.publish_goal()
    def power_level_callback(self,msg:String):
        if msg.data == "full":
            self.carinfo.power_level = CarBattery.FULL
        elif msg.data == "enough":
            self.carinfo.power_level = CarBattery.ENOUGH
        elif msg.data == "attention":
            self.carinfo.power_level = CarBattery.ATTENTION
        elif msg.data == "low":
            self.carinfo.power_level = CarBattery.LOW
            if self.fsm.state != RobotState.POWER:
                self.exchange.trans("/power_low\n".encode("utf-8"))


if __name__ == '__main__':
    try:
        central = PatrolController(Exchange())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
