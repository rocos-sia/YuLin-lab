# ! /usr/bin/env /python
import re
import threading
import time
from socket import *
import os
import actionlib
##
import roslib
import rospy
from action_tset1.msg import *
from datetime import datetime
roslib.load_manifest('robotiq_2f_gripper_control')
# from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg

##
sys.path.append('/home/sia/YuLin_lab/src/DucoCobotAPI_py/gen_py')
sys.path.append('/home/sia/YuLin_lab/src/DucoCobotAPI_py/lib')
sys.path.append('/home/sia/YuLin_lab/src/DucoCobotAPI_py')

from DucoCobot import DucoCobot
Isruning = 0  # 0代表客户端和服务端之间没在运行
flag = 1  # 每次必须从1开始或者从规定流程开始
array=["Get_sample","Centrifuge","Seperation_acid","Seperation_alkali","Put_sample","Drying","Recharge","Poweroff","model","robotstate",]#,
command=["取样品","离心操作","固液分离_酸","固液分离_碱","放样品","烘干操作","充电操作","关机操作","模式"]#操作流程
flag_max=[5,12,4,4,4,13,4,4,1]#代表每个大操作底下的具体流程个数
mid = 0  # 判断标志位
key = 0  # 指令：0->取样品,1->离心操作,2->固液分离,3->烘干操作,4->造粒机操作
list_key = []  # 存放已经接收成功的指令
quary = ''
param=''
model_send='1'
# 通信
duco_cobot = DucoCobot('192.168.1.10', 7003)
duco_cobot.open()
duco_cobot.power_on(True)
duco_cobot.enable(True)
print("机械臂连接成功")
host = '192.168.224.6'
port = 50000
buffsize = 2048
ADDR = (host, port)
# /home/sun/榆林实验室/机械臂/DucoCobotAPI_py
# tctime = socket(AF_INET, SOCK_STREAM)
# # tctime=socket()
# tctime.setsockopt(SOL_SOCKET,SO_REUSEADDR,1)
# tctime.bind(ADDR)
# tctime.listen(3)
HOST = '127.0.0.1'

PORT = 61000

BUFFSIZE = 2048

ADDR1 = (HOST, PORT)
tt = socket(AF_INET, SOCK_STREAM)
tt.setsockopt(SOL_SOCKET,SO_REUSEADDR,1)
tt.bind(ADDR1)
tt.listen(3)
print(1)
ttClient, ttaddr = tt.accept()
print(ttClient)


# /home/sun/榆林实验室/机械臂/DucoCobotAPI_py
def read_log(filename):
    filesize = os.path.getsize(filename)
    if filesize == 0:
        arr = 'THE LOG IS NO EXIT'
        return arr
    else:
        fo = open(filename, "rb")  # 一定要用'rb'因为seek 是以bytes来计算的
        fo.seek(-2, 2)
        result = fo.readline()

        mid = result[0]
        i = -1
        while True:
            if i < -1024:
                break
            i = i - 1
            fo.seek(i, 2)
            result = fo.readline()
            mid = result[0]
            if result[0] == 10 or result[0] == 0:
                fo.seek(i + 1, 2)
                line = fo.readline()
                if len(line) == 1:
                    continue
                else:
                    break

        result = line[0:-1]
        result = result.decode()
        return result
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
    list_command_num = [0, 1, 2, 3, 4, 5, 6, 7,8]
    if key in list_command_num:
        if key in [8]:
            global model_send
            model_send=param
            Isruning = 1
            list_1.append(key)
            arr11 = "task!send success!"
            print('arr11',arr11)
        else:
            Isruning = 1
            list_1.append(key)
            arr11 = "task!send success!"
            print('arr11',arr11)

        return arr11

    elif key == 9:

        # Connect!
        filename = '/home/sia/YuLin_lab/src/action_tset1/' + str(datetime.now().date().isoformat()) + '.log'
        arr1= read_log(filename)
        
        data1 = rospy.wait_for_message("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input)
        statusInterpreter(data1)
        joint = duco_cobot.get_actual_joints_position()
        pose = duco_cobot.get_tcp_pose()
        data = "0"
        forward = 'query!'
        ttClient.send(data.encode())
        plat = ttClient.recv(buffsize).decode()
        add = re.findall(r"\d+\.?\d*", plat)
        if len(list_key)==0 or (Isruning + mid == 0):  # 状态001
            command_send = str(0) + "01"
        else:
            command_send = str(list_key[-1] + 1) + str(flag).zfill(2)    
        arr = forward + str(add[0]) + "%" + str(add[1]) + "!" + str(joint[0]) + "%" + str(joint[1]) + "%" + str(
            joint[2]) + "%" + str(joint[3]) + "%" + str(joint[4]) + "%" + str(joint[5]) + "!" + str(
            pose[0]) + "%" + str(pose[1]) + "%" + str(pose[2]) + "!" + str(data1.gPO) + "!" + command_send + "!" + str(
            add[2]) + "!"+model_send+"!"
        if "task!failed 03!" in arr1:
            arr_failed=arr1.find("task!failed 03!")
            arr_send=arr+'\n'+arr1[arr_failed:]
        else:
            arr_send=arr
        # arr=forward+str(add[0])+"%"+str(add[1])+"!"+str(joint[0])+"%"+str(joint[1])+"%"+str(joint[2])+"%"+str(joint[3])+"%"+str(joint[4])+"%"+str(joint[5])+"!"+str(pose[0])+"%"+str(pose[1])+"%"+str(pose[2])+"!"+"!"+command_send+"!"+str(add[2])+"!"
        # print(data1)
        print(arr_send)
        # time.sleep(1)

        # list_1.append(key)
        return arr_send
    else:
        arr="task!failed 02!"
        return arr


#   函数功能：转换接受信息，举例把task!get_sample!转换成key对应的0
def data_key(data):
    # if data=="interrupt":
    #     return "interrupt"
    # else:
    length=data.count('!')
    if length>=2:
        list_data = data.split('!')
        if len(list_data)>3:
            global param
            param = list_data[2]
        task=list_data[1]
        task_order=["get_sample","centrifuge","seperation_acid","seperation_alkali","put_sample","drying","recharge","poweroff","model","robotstate"]

        if task in task_order:

            return task_order.index(task)
    else:
        return "error"

# 函数功能：循环发送流程指令直到flag>指令下的流程数
def callback(flag_0):
    goal1 = AddintsGoal()
    global param
    goal1.num = param+"!"+str(flag_0)
    print("goal1.num",goal1.num)
    global key
    if flag_0 <= flag_max[key]:
        client.send_goal(goal1, done_cb, active_cb, feedback_cb)
    else:
        global flag
       
        global mid
        # 测试

        time.sleep(2)
        flag = 1
        mid = 0
        rospy.loginfo("%s完成，请输入新的指令", command[key])
        time.sleep(0.1)
        print("waiting for new command")
        # print(mid)


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



# 线程函数：和control进行TCP通信
def thread_fun():

    tctime = socket(AF_INET,SOCK_STREAM)
    tctime.setsockopt(SOL_SOCKET,SO_REUSEADDR , 1)
    while not rospy.is_shutdown():
        tctime = socket(AF_INET,SOCK_STREAM)
        tctime.setsockopt(SOL_SOCKET,SO_REUSEADDR , 1)
        tctime.bind(ADDR)
        tctime.listen(3)   
        print('Wait for connection ...')
        try:
            # tctime.settimeout(100)
            tctimeClient,addr = tctime.accept()
            print("Connection from :",addr)
        except :
            print("close")
            tctime.close()
            break
        time.sleep(2)
        while not rospy.is_shutdown():
            time.sleep(0.01)
            # global Isruning
            data = tctimeClient.recv(buffsize).decode()
            print("收到的指令为",data)
            print("收到的指令为",data)
            print("收到的指令为",data)
            if not data:
                break
            global key
            str0 = {9}
            key1 = data_key(data)
            if mid == 0 or key1 in str0:
                if key1 in str0:
                    arr1 = sequence(key1, list_key)
                    print(arr1)
                    tctimeClient.send(arr1.encode())
                else:
                    arr1=sequence(key1,list_key)
                    if arr1=="task!failed 02!":
                        tctimeClient.send(arr1.encode())
                    else:
                        key=key1
                        Isruning=1
                        tctimeClient.send(arr1.encode())
                # list_key.append(key)
            # elif mid==1:
            elif mid == 1 and flag == 1:
                arr1 = "请再输入一次指令"
                print("   ：", arr1)
                tctimeClient.send(arr1.encode())
            elif mid == 1 and key1 not in str0:
                arr="task!failed 01!"
                tctimeClient.send(arr.encode())
            time.sleep(0.01)


# 单独开线程执行spin()
# def ros_spin():
#     rospy.spin()
#     # Close!
#     duco_cobot.close()

if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node("action_client_py")
    # 开线程
  
    thd_A = threading.Thread(target=thread_fun,daemon=True)
    thd_A.start()
    # thd_B = threading.Thread(target=ros_spin)
    # thd_B.start() 
    while not rospy.is_shutdown():
        time.sleep(0.5)
        if Isruning == 1:
            str1 = {0,1,2,3,4,5,6,7,8}
            if key in str1:
                Isruning = 0  # 确保接受指令后，主程序只执行一次
                mid = 1  # 标志位
                time.sleep(1)
                goal = AddintsGoal()
                goal.num = param+"!"+str(flag)
                client = actionlib.SimpleActionClient(array[key], AddintsAction)
                client.wait_for_server()  # 等待服务器启动
                rospy.loginfo("正在发送:%s指令...", command[key])
                client.send_goal(goal, done_cb, active_cb, feedback_cb)

            elif key == "error":
                rospy.loginfo("请输入有效命令")
                time.sleep(5)  # 防止命令不正确，反复循环刷屏

