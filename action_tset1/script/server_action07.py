#! /usr/bin/env python
import threading
import time
from socket import *

import actionlib
from action_tset1.msg import *

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
HOST = '127.0.0.1'

PORT = 60000

BUFFSIZE = 2048

ADDR1 = (HOST, PORT)
tt = socket(AF_INET, SOCK_STREAM)
tt.bind(ADDR1)
tt.listen(3)
print(1)
ttClient, ttaddr = tt.accept()
print(ttClient)
record = []


class Myaction:
    def __init__(self):
        # SimpleActionServer(topic->name, ActionSpec, execute_cb=None, auto_start=True)
        self.server = actionlib.SimpleActionServer("Get_sample", AddintsAction, self.cb, False)
        self.server.start()
        self.sample = getsample()
        rospy.loginfo("服务器启动...")

    def cb(self, goal):
        # 解析目标值
        goal_num = goal.num
        record.append(goal_num)
        rospy.loginfo("接收到第%d步指令", goal_num)
        # 反馈函数
        if goal_num == 1:

            rospy.loginfo("小车移动..")
            '''*****************'''
            duco_cobot.movej(midpose, 40, 20, 0, True)
            data = "13"
            ttClient.send(data.encode())
            plat = ttClient.recv(BUFFSIZE).decode()
            while plat != "00":
                sleep(3)
                print("小车返回状态错误")
            '''****'''
        elif goal_num == 2:
            self.sample.point()
            rospy.loginfo("二维码矫正")
        elif goal_num == 3:
            self.sample.get_tube()
            rospy.loginfo("抓取试管架")
        elif goal_num == 4:
            tube.lidput()
            rospy.loginfo("试管架放置到小车")
        elif goal_num == 5:
            rospy.loginfo("试管架夹持机构加紧试管")
        elif goal_num == 6:
            self.sample.defult()
            duco_cobot.movej(midpose, 30, 10, 0, True)
            rospy.loginfo("机械臂回到设置姿态")
        else:
            # 后期可以添加封装发送函数，发送失败status，停止整个过程。
            rospy.loginfo("流程操作失败，上一步流程为%d请输入有效流程", record[-2])
        rospy.loginfo("发送结果")
        result = AddintsResult()  # 建立该类型变量
        result.result = 1
        self.server.set_succeeded(result)  # set_succeeded
        rospy.loginfo("响应结果：第%d步完成", goal_num)


class Myaction1:
    def __init__(self):
        # SimpleActionServer(topic->name, ActionSpec, execute_cb=None, auto_start=True)
        self.server = actionlib.SimpleActionServer("Centrifuge", AddintsAction, self.cb, False)
        self.server.start()
        self.cen = centrifuge()
        rospy.loginfo("服务器启动...")

    def cb(self, goal):
        # 解析目标值
        goal_num = goal.num
        rospy.loginfo("接收到第%d步指令", goal_num)

        # 反馈函数
        if goal_num == 1:
            data = "12"
            ttClient.send(data.encode())
            plat1 = ttClient.recv(BUFFSIZE).decode()
            while plat1 != "00":
                sleep(3)
                print("小车返回状态错误")
            rospy.loginfo("小车移动到离心机位置..")
        elif goal_num == 2:
            self.cen.point()
            rospy.loginfo("离心机二维码矫正")
        elif goal_num == 3:
            self.cen.open()
            rospy.loginfo("按下离心机正面开盖按钮")
        elif goal_num == 4:
            rospy.loginfo("试管加盖")
        elif goal_num == 5:
            rospy.loginfo("试管取盖")
        elif goal_num == 6:
            rospy.loginfo("试管架上取下试管")
        elif goal_num == 7:
            self.cen.put()
            rospy.loginfo("放置试管到离心机内部")
        elif goal_num == 8:
            self.cen.close_door()
            rospy.loginfo("关闭离心机盖子")
        elif goal_num == 9:
            rospy.loginfo("按下离心机正面运行按钮")
        elif goal_num == 10:
            time.sleep(1)
            rospy.loginfo("等待离心机完成操作")
        elif goal_num == 11:
            self.cen.open()
            rospy.loginfo("按下离心机正面开盖按钮")
        elif goal_num == 12:
            self.cen.get()
            rospy.loginfo("从离心机上取出试管")
        elif goal_num == 13:
            rospy.loginfo("放置到试管架")
        elif goal_num == 14:
            self.cen.close_door()
            rospy.loginfo("关闭离心机盖子")
        elif goal_num == 15:
            tube.lidget()
            rospy.loginfo("取下试管盖子")
        elif goal_num == 16:
            self.cen.defult()
            duco_cobot.movej(midpose, 30, 10, 0, True)
            rospy.loginfo("机械臂恢复到设置状态")
        else:
            rospy.loginfo("流程操作失败，请输入有效流程")
        rospy.loginfo("发送结果")
        result = AddintsResult()  # 建立该类型变量
        result.result = 1
        self.server.set_succeeded(result)  # set_succeeded
        rospy.loginfo("响应结果：%d完成", goal_num)


class Myaction2:
    def __init__(self):
        # SimpleActionServer(topic->name, ActionSpec, execute_cb=None, auto_start=True)
        self.server = actionlib.SimpleActionServer("Seperation", AddintsAction, self.cb, False)
        self.server.start()
        rospy.loginfo("服务器启动...")

    def cb(self, goal):
        # 解析目标值
        goal_num = goal.num
        rospy.loginfo("接收到第%d步指令", goal_num)
        # 反馈函数
        if goal_num == 1:
            rospy.loginfo("小车移动到废液池位置..")
        elif goal_num == 2:
            rospy.loginfo("二维码矫正")
        elif goal_num == 3:
            rospy.loginfo("横向夹取试管到指定位置")
        elif goal_num == 4:
            rospy.loginfo("倾倒90度")
        elif goal_num == 5:
            rospy.loginfo("放置试管到试管架")
        elif goal_num == 6:
            rospy.loginfo("机械臂恢复到设置状态")
        else:
            rospy.loginfo("流程操作失败，请输入有效流程")
        rospy.loginfo("发送结果")
        result = AddintsResult()  # 建立该类型变量
        result.result = 1

        self.server.set_succeeded(result)  # set_succeeded
        rospy.loginfo("响应结果：%d完成", goal_num)


class Myaction3:
    def __init__(self):
        # SimpleActionServer(topic->name, ActionSpec, execute_cb=None, auto_start=True)
        self.server = actionlib.SimpleActionServer("Drying", AddintsAction, self.cb, False)
        self.server.start()
        self.dry = drying()
        rospy.loginfo("服务器启动...")

    def cb(self, goal):
        # 解析目标值
        goal_num = goal.num
        rospy.loginfo("接收到第%d步指令", goal_num)
        # 反馈函数
        if goal_num == 1:
            data = "11"
            ttClient.send(data.encode())
            plat2= ttClient.recv(BUFFSIZE).decode()
            while plat2 != "00":
                sleep(3)
                print("小车返回状态错误")
            rospy.loginfo("小车移动到烘干机位置..")
        elif goal_num == 2:
            self.dry.point()
            rospy.loginfo("二维码矫正")
        elif goal_num == 3:
            self.dry.open_door()
            rospy.loginfo("打开烘干机门90度")
        elif goal_num == 4:
            # motor.motor_position(0)
            rospy.loginfo("试管架夹持机构松开")
        elif goal_num == 5:
            self.dry.tubejia_put()
            rospy.loginfo("夹持试管架进入烘干机")
        elif goal_num == 6:
            self.dry.close_door()
            rospy.loginfo("关闭烘干机的门")
        elif goal_num == 7:
            self.dry.open_power()
            rospy.loginfo("通风按钮")
        elif goal_num == 8:
            time.sleep(1)
            rospy.loginfo("等待烘干机完成")
        elif goal_num == 9:
            self.dry.open_door()
            rospy.loginfo("打开烘干机的门")
        elif goal_num == 10:
            self.dry.tubejia_get()
            rospy.loginfo("试管架放置到小车")
        elif goal_num == 11:
            rospy.loginfo("夹持机构夹紧试管架")
        elif goal_num == 12:
            self.dry.close_door()
            rospy.loginfo("关闭烘干机的门")
        elif goal_num == 13:
            self.dry.defult()
            duco_cobot.movej(midpose, 30, 10, 0, True)
            rospy.loginfo("机械臂恢复到设置状态")
        else:
            rospy.loginfo("流程操作失败，请输入有效流程")
        rospy.loginfo("发送结果")
        result = AddintsResult()  # 建立该类型变量
        result.result = 1
        progress = ["移动位置", "烘干机操作"]
        self.server.set_succeeded(result)  # set_succeeded
        rospy.loginfo("响应结果：%d完成", goal_num)


class Myaction4:
    def __init__(self):
        # SimpleActionServer(topic->name, ActionSpec, execute_cb=None, auto_start=True)
        self.server = actionlib.SimpleActionServer("Prilling", AddintsAction, self.cb, False)
        self.server.start()
        rospy.loginfo("服务器启动...")

    def cb(self, goal):
        # 解析目标值
        goal_num = goal.num
        rospy.loginfo("接收到第%d步指令", goal_num)
        # 反馈函数
        if goal_num == 1:
            # print(rtde_r.getActualTCPForce())

            rospy.loginfo("小车移动到造粒机位置..")
            duco_cobot.movej(midpose, 20, 10, 0, True)
            data = "16"
            ttClient.send(data.encode())
            plat = ttClient.recv(BUFFSIZE).decode()
            while plat != "00":
                sleep(3)
                print("小车返回状态错误")
        elif goal_num == 2:
            # 充电
            data = "21"
            ttClient.send(data.encode())
            end = ttClient.recv(BUFFSIZE).decode()
            while end != "00":
                sleep(3)
                print("小车返回状态错误")
            sleep(1)
            rospy.loginfo("二维码矫正")
        elif goal_num == 3:
            data = "13"
            ttClient.send(data.encode())
            plat = ttClient.recv(BUFFSIZE).decode()
            while plat != "00":
                sleep(3)
                print("小车返回状态错误")
            rospy.loginfo("夹取试管到造粒机上")
        elif goal_num == 4:
            rospy.loginfo("倾倒试管")
        elif goal_num == 5:
            rospy.loginfo("放置试管到试管架")
        elif goal_num == 6:
            rospy.loginfo("机械臂恢复到设置状态")
        else:
            rospy.loginfo("流程操作失败，请输入有效流程")
        rospy.loginfo("发送结果")
        result = AddintsResult()  # 建立该类型变量
        result.result = 1

        self.server.set_succeeded(result)  # set_succeeded
        rospy.loginfo("响应结果：%s完成", goal_num)


class Myaction5:
    def __init__(self):
        # SimpleActionServer(topic->name, ActionSpec, execute_cb=None, auto_start=True)
        self.server = actionlib.SimpleActionServer("!robotstate", AddintsAction, self.cb, False)
        self.server.start()
        rospy.loginfo("服务器启动...")

    def cb(self, goal):
        # 解析目标值
        goal_num = goal.num
        feedback_obj = AddintsFeedback()  # 建立该类型变量
        feedback_obj.progess = "ggggg"
        self.server.publish_feedback(feedback_obj)
        # 反馈函数

        rospy.loginfo("信息发送")


if __name__ == "__main__":
    rospy.init_node("action_server_py")

    # # 激活夹爪
    robotiq(0, 1, 120, 0, 50, 1)
    robotiq(1, 1, 255, 0, 50, 1)
    try:
        # 机械臂心跳
        thd_B = threading.Thread(target=hearthread_fun)
        thd_B.daemon = True
        thd_B.start()
        server = Myaction()
        server1 = Myaction1()
        server2 = Myaction2()
        server3 = Myaction3()
        server4 = Myaction4()
        rospy.spin()
        # Close!
        # duco_cobot.close()
    except Thrift.TException as tx:
        print('%s' % tx.message)

    except Exception as e:
        print(e)
