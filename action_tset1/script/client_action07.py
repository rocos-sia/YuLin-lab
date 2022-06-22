# ! /usr/bin/env /python
# 思路1.添加字典switch判断
# 思路2.如果服务端正在运行但是客户端接收到了命令，堵塞并返回正在运行的操作
# 思路3.下发指令顺序不合理报错返回
# 思路4.与服务端另外建立通信，如果超时没有响应直接跳转停止的action
# 缺点：control无法操控退出
# 缺点：采用全局变量在多线程之间进行参数传递判断
import re
import threading
import time
from socket import *

import actionlib
##
import roslib
import rospy
from action_tset1.msg import *

roslib.load_manifest('robotiq_2f_gripper_control')
# from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg

##
sys.path.append('/home/sia/YuLin_lab/src/DucoCobotAPI_py/gen_py')
sys.path.append('/home/sia/YuLin_lab/src/DucoCobotAPI_py/lib')
sys.path.append('/home/sia/YuLin_lab/src/DucoCobotAPI_py')

from DucoCobot import DucoCobot

# 后期全局变量可以放置到元组中
Isruning = 0  # 0代表客户端和服务端之间没在运行
flag = 1  # 每次必须从1开始或者从规定流程开始
arr = ["Get_sample", "Centrifuge", "Seperation", "Drying", "Prilling", "robotstate"]  # 话题名称
command = ["取样品", "离心操作", "固液分离", "烘干操作", "造粒机操作"]  # 操作流程
flag_max = [6, 16, 6, 13, 6, 1]  # 代表每个大操作底下的具体流程个数
mid = 0  # 判断标志位
key = 0  # 指令：0->取样品,1->离心操作,2->固液分离,3->烘干操作,4->造粒机操作
list_key = []  # 存放已经接收成功的指令，如果实验室需要循环操作需要在最后给reset
quary = ''
# 通信
duco_cobot = DucoCobot('192.168.1.10', 7003)
duco_cobot.open()
duco_cobot.power_on(True)
duco_cobot.enable(True)
host = '192.168.224.9'
port = 50000
buffsize = 2048
ADDR = (host, port)
# /home/sun/榆林实验室/机械臂/DucoCobotAPI_py
tctime = socket(AF_INET, SOCK_STREAM)
tctime.bind(ADDR)
tctime.listen(3)
HOST = '127.0.0.1'

PORT = 61000

BUFFSIZE = 2048

ADDR1 = (HOST, PORT)
tt = socket(AF_INET, SOCK_STREAM)
tt.bind(ADDR1)
tt.listen(3)
print(1)
ttClient, ttaddr = tt.accept()
print(ttClient)


# /home/sun/榆林实验室/机械臂/DucoCobotAPI_py

def statusInterpreter(status):
    """Generate a string according to the current value of the status variables."""

    output = '\n-----\n2F gripper status interpreter\n-----\n'

    # gPO
    output += 'gPO = ' + str(status.gPO) + ': '
    output += 'Position of Fingers: ' + str(status.gPO) + '/255\n'

    return output


# 函数功能 ：对congtrol发送的指令进行逻辑判断，然后返回错误类型
def sequence(key, list_1):
    global Isruning
    if key == 0:
        if key in list_1:
            arr = "task!failed!01!"
            return arr

        else:
            # global Isruning
            Isruning = 1
            arr = "task!success!"
            list_1.append(key)
            return arr
    elif key == 1:
        if 0 in list_1:
            # global Isruning
            Isruning = 1
            arr = "task!success!"
            list_1.append(key)
            return arr
        else:
            arr = "task!failed!02!"
            return arr

    elif key == 2:
        if 0 in list_1 and 1 in list_1:
            # global Isruning
            Isruning = 1
            arr = "task!success!"
            list_1.append(key)
            return arr
        elif 0 in list_1:
            arr = "task!failed!03!"
            return arr

        else:
            arr = "task!failed!02!"
            return arr
    elif key == 3:
        if 0 in list_1 and 2 in list_1:
            # global Isruning
            Isruning = 1
            arr = "task!success!"
            list_1.append(key)
            return arr
        elif 0 in list_1:
            arr = "task!failed!04!"
            return arr
        else:
            arr = "task!failed!02!"
            return arr
    elif key == 4:
        if 0 in list_1 and 3 in list_1:
            # global Isruning
            Isruning = 1
            arr = "task!success!"
            # 当key等于4代表循环执行完毕，重新置0
            list_1.clear()

            return arr
        elif 0 in list_1:
            arr = "task!failed!05!"
            return arr
        else:
            arr = "task!failed!02!"
            return arr
    elif key == 5:

        # Connect!

        data1 = rospy.wait_for_message("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input)
        statusInterpreter(data1)
        joint = duco_cobot.get_actual_joints_position()
        pose = duco_cobot.get_tcp_pose()
        data = "0"
        forward = 'query!'
        ttClient.send(data.encode())
        print(1)
        plat = ttClient.recv(buffsize).decode()
        print(plat)
        add = re.findall(r"\d+\.?\d*", plat)
        print(add)
        if len(list_key) == 0 or (Isruning + mid == 0):  # 状态001
            command_send = str(0) + "01"
        else:
            command_send = str(list_key[-1] + 1) + "0" + str(flag)
        arr = forward + str(add[0]) + "%" + str(add[1]) + "!" + str(joint[0]) + "%" + str(joint[1]) + "%" + str(
            joint[2]) + "%" + str(joint[3]) + "%" + str(joint[4]) + "%" + str(joint[5]) + "!" + str(
            pose[0]) + "%" + str(pose[1]) + "%" + str(pose[2]) + "!" + str(data1.gPO) + "!" + command_send + "!" + str(
            add[2]) + "!"
        # arr=forward+str(add[0])+"%"+str(add[1])+"!"+str(joint[0])+"%"+str(joint[1])+"%"+str(joint[2])+"%"+str(joint[3])+"%"+str(joint[4])+"%"+str(joint[5])+"!"+str(pose[0])+"%"+str(pose[1])+"%"+str(pose[2])+"!"+"!"+command_send+"!"+str(add[2])+"!"
        # print(data1)
        print(arr)
        print(type(data))
        # time.sleep(1)

        # list_1.append(key)
        return arr
    else:
        arr = "task!no exist!08!"
        return arr


#   函数功能：转换接受信息，举例把task!get_sample!转换成key对应的0
def data_key(data):
    # if data=="interrupt":
    #     return "interrupt"
    # else:
    length = len(data)
    if length > 6:
        task = data[5:-1]
        task_order = ["get_sample", "centrifuge", "seperation", "drying", "prilling", "!robotstate"]
        if task in task_order:
            return task_order.index(task)
    else:
        return "error"
    # 函数功能：循环发送流程指令直到flag>指令下的流程数


def callback(flag_0):
    goal1 = AddintsGoal()
    goal1.num = flag_0

    if flag_0 <= flag_max[key]:
        client.send_goal(goal1, done_cb, active_cb, feedback_cb)
    else:
        global flag
        flag = 1
        global mid
        # 测试

        time.sleep(2)

        mid = 0
        rospy.loginfo("%s完成，请输入新的指令", command[key])
        time.sleep(0.1)
        print("新指令不该被阻塞")
        print(mid)


# 函数：三个函数为action的参数函数
def done_cb(status, result):
    if status == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("响应成功")
        global flag
        flag = flag + 1

        # rospy.loginfo("第%d步指令发送",flag)
        callback(flag)
    else:

        rospy.loginfo("%d步响应失败,请查看", flag)


def active_cb():
    rospy.loginfo("连接建立...")
    rospy.loginfo("第%d步指令发送", flag)


def feedback_cb(feedback):
    rospy.loginfo("进度：%2s", feedback.progess)
    quary = quary + feedback.progress


# 线程函数：和control进行TCP通信
def thread_fun():
    # host = '127.0.0.1'
    # port =12345
    # buffsize = 2048
    # ADDR = (host,port)
    # #
    # tctime = socket(AF_INET,SOCK_STREAM)
    # tctime.bind(ADDR)
    # tctime.listen(3)

    # while True:

    print('Wait for connection ...')
    tctimeClient, addr = tctime.accept()
    print("Connection from :", addr)
    while True:
        time.sleep(0.01)
        # global Isruning
        data = tctimeClient.recv(buffsize).decode()
        print("收到的指令为")
        if not data:
            break
        global key
        str0 = {5}
        key1 = data_key(data)
        if mid == 0 or key1 in str0:
            if key1 in str0:
                arr1 = sequence(key1, list_key)
                print(arr1)
                tctimeClient.send(arr1.encode())
            else:
                global Isruning
                # Isruning=1
                # global list_key
                key = key1
                arr1 = sequence(key, list_key)
                print(arr1)
                print(Isruning)
                print(list_key)
                # print("command%d",key)
                # arr="服务端接收指令成功"

                tctimeClient.send(arr1.encode())
            # list_key.append(key)
        # elif mid==1:
        elif mid == 1 and flag == 1:
            arr1 = "success"
            print("   ：", arr1)
            tctimeClient.send(arr1.encode())
        elif mid == 1 and key1 not in str0:
            arr = "进入阻塞状态,命令无效"
            tctimeClient.send(arr.encode())
        time.sleep(0.01)


# 单独开线程执行spin()
def ros_spin():
    rospy.spin()


if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node("action_client_py")
    # 开线程

    thd_A = threading.Thread(target=thread_fun)
    thd_A.start()
    # thd_B = threading.Thread(target=ros_spin)
    # thd_B.start() 
    while True:
        time.sleep(0.5)
        if Isruning == 1:
            str1 = {0, 1, 2, 3, 4}
            if key in str1:
                Isruning = 0  # 确保接受指令后，主程序只执行一次
                mid = 1  # 标志位
                time.sleep(1)
                goal = AddintsGoal()
                goal.num = flag
                client = actionlib.SimpleActionClient(arr[key], AddintsAction)
                client.wait_for_server()  # 等待服务器启动
                rospy.loginfo("正在发送:%s指令...", command[key])
                client.send_goal(goal, done_cb, active_cb, feedback_cb)


            elif key == "error":
                rospy.loginfo("请输入有效命令")
                time.sleep(5)  # 防止命令不正确，反复循环刷屏
# 麻烦，之前的task!sucess!应该在运行结束后发送，不是一接到指令就发送
# 麻烦，从服务端接受信息到客户端，客户端到control
