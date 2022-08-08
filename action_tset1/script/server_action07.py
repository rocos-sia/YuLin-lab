#! /usr/bin/env python
import threading
import time
from socket import *
import sys
import actionlib
from action_tset1.msg import *
import logging
from datetime import datetime
from asyncio.log import logger
import os
import subprocess
import os
import traceback, sys

##
sys.path.append('/home/sia/YuLin_lab/src/DucoCobotAPI_py/gen_py')
sys.path.append('/home/sia/YuLin_lab/src/DucoCobotAPI_py/lib')
sys.path.append('/home/sia/YuLin_lab/src/DucoCobotAPI_py')

from lab import *
# 后期全局变量可以放置到元组中
"""
1.导包
2.初始化ROS节点
3.单独封装一个类
4.类中创建action服务端对象；
5.处理请求，（1，解析目标值，发送连续反馈，响应最终结果）---回调函数
6.spin()
"""

##
def GK_poweroff():
    password = 'a'
    command = 'sudo poweroff'
    str = os.system('echo %s | sudo -S %s' % (password,command))
    print(str)
HOST = '127.0.0.1'

PORT = 60000

BUFFSIZE = 2048
ADDR1 = (HOST, PORT)
tt = socket(AF_INET, SOCK_STREAM)
tt.setsockopt(SOL_SOCKET,SO_REUSEADDR,1)
tt.bind(ADDR1)
tt.listen(3)
print(1)
ttClient, ttaddr = tt.accept()
print(ttClient)
record = [1]

model_flag = 1

class Myaction:
    def __init__(self):

        # SimpleActionServer(topic->name, ActionSpec, execute_cb=None, auto_start=True)
        self.server = actionlib.SimpleActionServer("Get_sample", AddintsAction, self.cb, False)
        self.server.start()
        self.sample = getsample()
        self.action_command = ["小车移动到取试管位置", "二维码矫正", "抓取试管到试管架", "试管架夹持机构加紧试管架", "试管加盖", "机械臂回到设置姿态"]
        # rospy.loginfo("服务器启动...")

    def cb(self, goal):
        # 解析目标值
        try:
            goal = goal.num
            goal_list = goal.split('!')
            if goal_list[0] == '':
                param = 5
            else:
                param = int(goal_list[0])
            goal_num = int(goal_list[-1])
            record.append(goal_num)
            # rospy.loginfo("接收到Get_sample第%d步指令",goal_num)
            # 反馈函数
            if goal_num == 1:
                rospy.loginfo("小车移动到取试管位置")
                duco_cobot.movej(midpose, 40, 20, 0, True)
                data = "11"
                ttClient.send(data.encode())
                plat = ttClient.recv(BUFFSIZE).decode()
                if plat != "00":
                    rospy.logerr("Get_sample %d", goal_num)
                 
            elif goal_num == 2:
                rospy.loginfo("二维码矫正")
                self.sample.point()
            elif goal_num == 3:
                rospy.loginfo("抓取试管到试管架")
                self.sample.get_tube(model_flag)
            elif goal_num == 4:
                rospy.loginfo("试管加盖")
                tube.lidput(model_flag)
            elif goal_num == 5:
                rospy.loginfo("机械臂回到设置姿态")
                self.sample.again()
                duco_cobot.movej(midpose, 50, 10, 0, True)
            else:
                # 后期可以添加封装发送函数，发送失败status，停止整个过程。
                rospy.logerr("流程操作失败，流程为%d,请输入有效流程", record[-2])
            # rospy.loginfo("发送结果")
            result = AddintsResult()  # 建立该类型变量
            result.result = 1
            self.server.set_succeeded(result)  # set_succeeded
            rospy.loginfo("响应结果：%s操作完成", self.action_command[goal_num - 1])
        except Exception as e:
            print(e)
            traceback.print_exc()  # 打印异常信息
            exc_type, exc_value, exc_traceback = sys.exc_info()
            error = str(repr(traceback.format_exception(exc_type, exc_value, exc_traceback)))  # 将异常信息转为字符串
            logger.error(error)
            logger.error("task!failed 03!+流程操作失败，流程为%s",self.action_command[record[-2]])
            
            # password = 'a'
            # command = 'sudo poweroff'
            # str = os.system('echo %s | sudo -S %s' % (password,command))
            # print(str)

class Myaction1:
    def __init__(self):
        # SimpleActionServer(topic->name, ActionSpec, execute_cb=None, auto_start=True)
        self.server = actionlib.SimpleActionServer("Centrifuge", AddintsAction, self.cb, False)
        self.server.start()
        self.cen = centrifuge()
        self.action_command = ["小车移动到离心机位置", "离心机二维码矫正", "按下离心机正面开盖按钮", "试管架上取下试管放置试管到离心机内部", "按下离心机正面运行按钮", "关闭离心机盖子"
                                                                                                             "按下离心机正面运行按钮",
                               "等待离心机完成操作", "按下离心机正面开盖按钮", "从离心机上取出试管放置到试管架", "关闭离心机盖子", "试管取盖", "机械臂恢复到设置状态"]
        # rospy.loginfo("服务器启动...")

    def cb(self, goal):
        # 解析目标值:
        try:
            goal = goal.num
            goal_list = goal.split('!')
            if goal_list[0] == '':
                param = 5
            else:
                param = int(goal_list[0])
            goal_num = int(goal_list[-1])
            record.append(goal_num)
            # 反馈函数
            if goal_num == 1:
                rospy.loginfo("小车移动到离心机位置..")
                data = "11"
                ttClient.send(data.encode())
                plat1 = ttClient.recv(BUFFSIZE).decode()
                if plat1 != "00":
                    rospy.logerr("Centrifuge %d", goal_num)
                    assert False
            elif goal_num == 2:
                rospy.loginfo("离心机二维码矫正")
                self.cen.point()
            elif goal_num == 3:
                rospy.loginfo("按下离心机正面开盖按钮")
                self.cen.open()
            elif goal_num == 4:
                rospy.loginfo("试管架上取下试管放置试管到离心机内部")
                self.cen.put(model_flag)
            elif goal_num == 5:
                rospy.loginfo("关闭离心机盖子")
                self.cen.close_door()
            elif goal_num == 6:
                rospy.loginfo("按下离心机正面运行按钮")
            elif goal_num == 7:
                rospy.loginfo("等待离心机完成操作")
                time.sleep(param)
            elif goal_num == 8:
                rospy.loginfo("按下离心机正面开盖按钮")
                self.cen.open()
            elif goal_num == 9:
                rospy.loginfo("从离心机上取出试管放置到试管架")
                self.cen.get(model_flag)
            elif goal_num == 10:
                rospy.loginfo("关闭离心机盖子")
                self.cen.close_door()
            elif goal_num == 11:
                rospy.loginfo("试管取盖")
                tube.lidget(model_flag)
            elif goal_num == 12:
                rospy.loginfo("机械臂恢复到设置状态")
                self.cen.again()
                duco_cobot.movej(midpose, 50, 10, 0, True)
            else:
                logger.error("流程操作失败，流程为%d,请输入有效流程", record[-2])
            result = AddintsResult()  # 建立该类型变量
            result.result = 1
            self.server.set_succeeded(result)  # set_succeeded
            rospy.loginfo("响应结果：%s操作完成", self.action_command[goal_num - 1])
        except Exception as e:
            print(e)
            traceback.print_exc()  # 打印异常信息
            exc_type, exc_value, exc_traceback = sys.exc_info()
            error = str(repr(traceback.format_exception(exc_type, exc_value, exc_traceback)))  # 将异常信息转为字符串
            logger.error(error)
            logger.error("task!failed 03!+流程操作失败，流程为%s",self.action_command[record[-2]])
            # print(e)
            # password = 'a'
            # command = 'sudo poweroff'
            # str = os.system('echo %s | sudo -S %s' % (password,command))
            # print(str)


class Myaction2:
    def __init__(self):
        # SimpleActionServer(topic->name, ActionSpec, execute_cb=None, auto_start=True)
        self.server = actionlib.SimpleActionServer("Seperation_acid", AddintsAction, self.cb, False)
        self.server.start()
        self.action_command = ["小车移动到废液池位置", "二维码矫正", "横向夹取试管进行倾倒", "机械臂恢复到设置状态"]
        self.sep = seperation()
        # rospy.loginfo("服务器启动...")

    def cb(self, goal):
        # 解析目标值
        try:
            goal = goal.num
            goal_list = goal.split('!')
            if goal_list[0] == '':
                param = 0
            else:
                param = int(goal_list[0])
            goal_num = int(goal_list[-1])
            record.append(goal_num)
            # rospy.loginfo("接收到第%d步指令",goal_num)
            # 反馈函数
            if goal_num == 1:
                data = "11"
                ttClient.send(data.encode())
                plat1 = ttClient.recv(BUFFSIZE).decode()
                if plat1 != "00":
                    rospy.logerr("Centrifuge %d", goal_num)
                    assert False
            elif goal_num == 2:
                self.sep.point()
                rospy.loginfo("二维码矫正")
            elif goal_num == 3:
                self.sep.throw(param,model_flag)
                rospy.loginfo("横向夹取试管进行倾倒,并放回")
            elif goal_num == 4:
                self.sep.again()
                duco_cobot.movej(midpose, 50, 10, 0, True)
                rospy.loginfo("机械臂恢复到设置状态")
            else:
                rospy.logerr("流程操作失败，流程为%d,请输入有效流程", record[-2])
            # rospy.loginfo("发送结果")
            result = AddintsResult()  # 建立该类型变量
            result.result = 1

            self.server.set_succeeded(result)  # set_succeeded
            rospy.loginfo("响应结果：%s操作完成", self.action_command[goal_num - 1])
        except:
            print(e)
            traceback.print_exc()  # 打印异常信息
            exc_type, exc_value, exc_traceback = sys.exc_info()
            error = str(repr(traceback.format_exception(exc_type, exc_value, exc_traceback)))  # 将异常信息转为字符串
            logger.error(error)
            logger.error("task!failed 03!+流程操作失败，流程为%s",self.action_command[record[-2]])
            # password = 'a'
            # command = 'sudo poweroff'
            # str = os.system('echo %s | sudo -S %s' % (password,command))
            # print(str)


class Myaction2_1:
    def __init__(self):
        # SimpleActionServer(topic->name, ActionSpec, execute_cb=None, auto_start=True)
        self.server = actionlib.SimpleActionServer("Seperation_alkali", AddintsAction, self.cb, False)
        self.server.start()
        self.action_command = ["小车移动到废液池位置", "二维码矫正", "横向夹取试管进行倾倒", "机械臂恢复到设置状态"]
        # rospy.loginfo("服务器启动...")

    def cb(self, goal):
        # 解析目标值
        try:
            goal = goal.num
            goal_list = goal.split('!')
            if goal_list[0] == '':
                param = 5
            else:
                param = int(goal_list[0])
            goal_num = int(goal_list[-1])
            record.append(goal_num)
            # rospy.loginfo("接收到第%d步指令",goal_num)
            # 反馈函数
            if goal_num == 1:
                rospy.loginfo("小车移动到废液池位置")
            elif goal_num == 2:
                # print(rtde_r.getActualTCPPose())
                rospy.loginfo("二维码矫正")
            elif goal_num == 3:
                rospy.loginfo("横向夹取试管进行倾倒,并放回")
            elif goal_num == 4:
                rospy.loginfo("机械臂恢复到设置状态")
            else:
                rospy.logerr("流程操作失败，流程为%d,请输入有效流程", record[-2])
            # rospy.loginfo("发送结果")
            result = AddintsResult()  # 建立该类型变量
            result.result = 1

            self.server.set_succeeded(result)  # set_succeeded
            rospy.loginfo("响应结果：%s操作完成", self.action_command[goal_num - 1])
        except:
            print(e)
            traceback.print_exc()  # 打印异常信息
            exc_type, exc_value, exc_traceback = sys.exc_info()
            error = str(repr(traceback.format_exception(exc_type, exc_value, exc_traceback)))  # 将异常信息转为字符串
            logger.error(error)
            logger.error("task!failed 03!+流程操作失败，流程为%s",self.action_command[record[-2]])
            # password = 'a'
            # command = 'sudo poweroff'
            # str = os.system('echo %s | sudo -S %s' % (password,command))
            # print(str)



class Myaction3:
    def __init__(self):
        # SimpleActionServer(topic->name, ActionSpec, execute_cb=None, auto_start=True)
        self.server = actionlib.SimpleActionServer("Drying", AddintsAction, self.cb, False)
        self.server.start()
        self.dry = drying()
        self.action_command = ["小车移动到烘干机位置", "二维码矫正", "打开烘干机门90度", "试管架夹持机构松开", "夹持试管架进入烘干机", "关闭烘干机的门", "机器人按下启动按钮",
                               "小车等待时间", "打开烘干机门90度", "试管架放置到小车", "夹持机构夹紧试管架", "关闭烘干机的门", "机械臂恢复到设置状态"]
        # rospy.loginfo("服务器启动...")

    def cb(self, goal):
        # 解析目标值
        try:
            goal = goal.num
            goal_list = goal.split('!')
            if goal_list[0] == '':
                param = 6
            else:
                param = int(goal_list[0])
            goal_num = int(goal_list[-1])
            record.append(goal_num)
            # rospy.loginfo("接收到第%d步指令",goal_num)
            # 反馈函数
            if goal_num == 1:
                rospy.loginfo("小车移动到烘干机位置..")
                data = "12"
                ttClient.send(data.encode())
                plat2 = ttClient.recv(BUFFSIZE).decode()
                if plat2 != "00":
                    rospy.logerr("Drying %d", goal_num)
                    assert False 
            elif goal_num == 2:
                rospy.loginfo("二维码矫正")
                self.dry.point()
            elif goal_num == 3:
                rospy.loginfo("打开烘干机门90度")
                self.dry.open_door()
            elif goal_num == 4:
                rospy.loginfo("试管架夹持机构松开")
            elif goal_num == 5:
                rospy.loginfo("夹持试管架进入烘干机")
                self.dry.tubejia_put()
            elif goal_num == 6:
                rospy.loginfo("关闭烘干机的门")
                self.dry.close_door()
            elif goal_num == 7:
                rospy.loginfo("机器人按下启动按钮")
                self.dry.open_power()
            elif goal_num == 8:
                rospy.loginfo("等待时间")
                time.sleep(param)
            elif goal_num == 9:
                rospy.loginfo("打开烘干机门90度")
                self.dry.open_door()
            elif goal_num == 10:
                rospy.loginfo("试管架放置到小车")
                self.dry.tubejia_get()
            elif goal_num == 11:
                rospy.loginfo("夹持机构夹紧试管架")
            elif goal_num == 12:
                rospy.loginfo("关闭烘干机的门")
                self.dry.close_door()
            elif goal_num == 13:
                self.dry.again()
                duco_cobot.movej(midpose, 50, 10, 0, True)
                rospy.loginfo("机械臂恢复到设置状态")
            else:
                rospy.logerr("流程操作失败，流程为%d,请输入有效流程", record[-2])
            result = AddintsResult()  # 建立该类型变量
            result.result = 1
            self.server.set_succeeded(result)  # set_succeeded
            rospy.loginfo("响应结果：%s操作完成", self.action_command[goal_num - 1])
        except:
            print(e)
            traceback.print_exc()  # 打印异常信息
            exc_type, exc_value, exc_traceback = sys.exc_info()
            error = str(repr(traceback.format_exception(exc_type, exc_value, exc_traceback)))  # 将异常信息转为字符串
            logger.error(error)
            logger.error("task!failed 03!+流程操作失败，流程为%s",self.action_command[record[-2]])
            # password = 'a'
            # command = 'sudo poweroff'
            # str = os.system('echo %s | sudo -S %s' % (password,command))
            # print(str)


class Myaction4:
    def __init__(self):
        # SimpleActionServer(topic->name, ActionSpec, execute_cb=None, auto_start=True)
        self.server = actionlib.SimpleActionServer("Recharge", AddintsAction, self.cb, False)
        self.server.start()
        self.action_command = ["小车移动到充电位置", "小车发送充电信号", "充电中", "充电结束"]
        # rospy.loginfo("服务器启动...")

    def cb(self, goal):
        try:
            # 解析目标值
            goal = goal.num
            goal_list = goal.split('!')
            if goal_list[0] == '':
                param = 5
            else:
                param = int(goal_list[0])
            goal_num = int(goal_list[-1])
            record.append(goal_num)
            # rospy.loginfo("机器人接收到第%d步指令",goal_num)
            # 反馈函数
            if goal_num == 1:
                rospy.loginfo("小车移动到充电位置")
                duco_cobot.movej(midpose, 20, 10, 0, True)
                data = "13"
                ttClient.send(data.encode())
                plat = ttClient.recv(BUFFSIZE).decode()
                if plat != "00":
                    rospy.logerr("Recharge %d", goal_num)
                    assert False
            elif goal_num == 2:
                rospy.loginfo("小车发送充电信号,充电中")
                # 充电
                data = "21"
                ttClient.send(data.encode())
                plat = ttClient.recv(BUFFSIZE).decode()
                if plat != "00":
                    rospy.logerr("Recharge %d", goal_num)
                    assert False
            elif goal_num == 3:
                rospy.loginfo("充电中")
            elif goal_num == 4:
                rospy.loginfo("充电结束")
            else:
                rospy.logerr("流程操作失败，流程为%d,请输入有效流程", record[-2])
            # rospy.loginfo("发送结果")
            result = AddintsResult()  # 建立该类型变量
            result.result = 1

            self.server.set_succeeded(result)  # set_succeeded
            rospy.loginfo("响应结果：%s操作完成", self.action_command[goal_num - 1])
        except:
            print(e)
            traceback.print_exc()  # 打印异常信息
            exc_type, exc_value, exc_traceback = sys.exc_info()
            error = str(repr(traceback.format_exception(exc_type, exc_value, exc_traceback)))  # 将异常信息转为字符串
            logger.error(error)
            logger.error("task!failed 03!+流程操作失败，流程为%s",self.action_command[record[-2]])
            # password = 'a'
            # command = 'sudo poweroff'
            # str = os.system('echo %s | sudo -S %s' % (password,command))
            # print(str)



class Myaction5:
    def __init__(self):
        # SimpleActionServer(topic->name, ActionSpec, execute_cb=None, auto_start=True)
        self.server = actionlib.SimpleActionServer("Put_sample", AddintsAction, self.cb, False)
        self.server.start()
        self.action_command = ["小车移动到工作站", "二维码识别", "夹取试管到工作站", "机械臂恢复到设置姿态"]
        self.putsam = putsample()
        # rospy.loginfo("服务器启动...")

    def cb(self, goal):
        # 解析目标值
        try:
            goal = goal.num
            goal_list = goal.split('!')
            if goal_list[0] == '':
                param = 5
            else:
                param = int(goal_list[0])
            goal_num = int(goal_list[-1])
            record.append(goal_num)
            # rospy.loginfo("机器人接收到第%d步指令",goal_num)
            # 反馈函数
            if goal_num == 1:
                rospy.loginfo("小车移动到取试管位置")
                duco_cobot.movej(midpose, 40, 20, 0, True)
                data = "11"
                ttClient.send(data.encode())
                plat = ttClient.recv(BUFFSIZE).decode()
                if plat != "00":
                    rospy.logerr("Get_sample %d", goal_num)
                    assert False
            elif goal_num == 2:
                self.putsam.point()
                rospy.loginfo("二维码识别")
            elif goal_num == 3:
                self.putsam.put_tube(model_flag)
                rospy.loginfo("夹取试管到工作站")
            elif goal_num == 4:
                self.putsam.again()
                duco_cobot.movej(midpose, 50, 10, 0, True)
                rospy.loginfo("小车恢复到设置姿态")
            else:
                rospy.logerr("流程操作失败，流程为%d,请输入有效流程", record[-2])
            # rospy.loginfo("发送结果")
            result = AddintsResult()  # 建立该类型变量
            result.result = 1

            self.server.set_succeeded(result)  # set_succeeded
            rospy.loginfo("响应结果：%s操作完成", self.action_command[goal_num - 1])
        except:
            print(e)
            traceback.print_exc()  # 打印异常信息
            exc_type, exc_value, exc_traceback = sys.exc_info()
            error = str(repr(traceback.format_exception(exc_type, exc_value, exc_traceback)))  # 将异常信息转为字符串
            logger.error(error)
            logger.error("task!failed 03!+流程操作失败，流程为%s",self.action_command[record[-2]])
            # password = 'a'
            # command = 'sudo poweroff'
            # str = os.system('echo %s | sudo -S %s' % (password,command))
            # print(str)



class Myaction7:
    def __init__(self) :
        #SimpleActionServer(topic->name, ActionSpec, execute_cb=None, auto_start=True)
        self.server=actionlib.SimpleActionServer("Poweroff",AddintsAction,self.cb,False)
        self.server.start()
        self.action_command=["电机归 0","关闭工控机 ","机械臂恢复到初始姿态"]
        # rospy.loginfo("服务器启动...")

    def cb(self,goal):
        #解析目标值
       
        goal=goal.num
        goal_list=goal.split('!')
        if goal_list[0]=='' :
            param=5
        else:
            param=int(goal_list[0])
        goal_num=int(goal_list[-1])
        record.append(goal_num)
        # rospy.loginfo("机器人接收到第%d步指令",goal_num)
        #反馈函数
        if goal_num==1:
            motor.motor_position(0,20)
            rospy.loginfo("电机->0")
        elif goal_num==2:
            rospy.loginfo("机械臂恢复到设置姿态")
            duco_cobot.movej(midpose, 50, 10, 0, True)
        elif goal_num==3:
            rospy.loginfo("机械臂关机 ")
            # duco_cobot.disable(True)
            # sleep(1)
            # duco_cobot.power_off(True)
            # sleep(1)
            # duco_cobot.shutdown(True)
            # sleep(5)
        elif goal_num==4:
            
            rospy.loginfo("工控机关机")
            password = 'a'
            command = 'sudo poweroff'
            str = os.system('echo %s | sudo -S %s' % (password,command))
            print(str)
        else:
            rospy.logerr ("流程操作失败，流程为%d,请输入有效流程",record[-2])
        # rospy.loginfo("发送结果")
        result=AddintsResult()#建立该类型变量
        result.result=1
        self.server.set_succeeded(result)#set_succeeded
        rospy.loginfo("响应结果：%s操作完成",self.action_command[goal_num-1])
        
class Myaction8:
    def __init__(self) :
        #SimpleActionServer(topic->name, ActionSpec, execute_cb=None, auto_start=True)
        self.server=actionlib.SimpleActionServer("model",AddintsAction,self.cb,False)
        self.server.start()
        self.action_command=["模式更改"]
        # rospy.loginfo("服务器启动...")

    def cb(self,goal):
        #解析目标值
        
        goal=goal.num
        goal_list=goal.split('!')
        if goal_list[0]=='' :
            param=1
        else:
            param=int(goal_list[0])
        goal_num=int(goal_list[-1])
        print(goal_num)
        record.append(goal_num)
        # rospy.loginfo("机器人接收到第%d步指令",goal_num)
        #反馈函数
        if goal_num==1:
            global model_flag
            model_flag=param
            print("model",model_flag)
            
        result=AddintsResult()#建立该类型变量
        result.result=1
        self.server.set_succeeded(result)#set_succeeded
        rospy.loginfo("响应结果：%s操作完成",self.action_command[goal_num-1])  
       
           


if __name__ == "__main__":
    rospy.init_node("action_server_py")
    # time.sleep(2)
    # # 激活夹爪
    try:
        # 机械臂心跳
        thd_B = threading.Thread(target=hearthread_fun,daemon=True)
        thd_B.daemon = True
        thd_B.start()
        server = Myaction()
        server1 = Myaction1()
        server2 = Myaction2() 
        server2_1 = Myaction2_1()
        server3 = Myaction3()
        server4 = Myaction4()
        server5 = Myaction5()
        server7 = Myaction7()
        server8=Myaction8()
       
        logger = logging.getLogger()
        logger.setLevel(logging.DEBUG)
       
        logFile = '/home/sia/YuLin_lab/src/action_tset1/' + str(datetime.now().date().isoformat()) + '.log'
        fh = logging.FileHandler(logFile, mode='a', encoding='utf-8')
        # 4，设置保存至文件的日志等级
        fh.setLevel(logging.ERROR)
        format = logging.Formatter('%(asctime)s %(filename)s[line:%(lineno)d] %(levelname)s %(message)s')
        # 6，设置 写入日志文件的Handler 的日志格式
        fh.setFormatter(format)
        # 第四步，将Handler添加至日志记录器logger里
        logger.addHandler(fh)
        logger.error("程序启动")
        motor.motor_position(179000,30)
        robotiq(0, 1, 120, 0, 50, 1)
        robotiq(1, 1, 255, 0, 50, 1)
        rospy.spin()
        # Close!
        duco_cobot.close()
    except Thrift.TException as tx:
        duco_cobot.close()
        
        print('%s' % tx.message)
        # password = 'a'
        # command = 'sudo poweroff'
        # str = os.system('echo %s | sudo -S %s' % (password,command))
        # print(str)

    except Exception as e:
       
        print(e)
        # password = 'a'
        # command = 'sudo poweroff'
        # str = os.system('echo %s | sudo -S %s' % (password,command))
        # print(str)
