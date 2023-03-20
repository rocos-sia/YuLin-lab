# ! /usr/bin/env /pythonmi
import re
import threading
import time
from socket import *
import os
import actionlib
import logging
import datetime
from asyncio.log import logger
import roslib
import rospy
from action_tset1.msg import *
import datetime
roslib.load_manifest('robotiq_2f_gripper_control')
# from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg

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
key = 0  # 指令：0->取样品,1->离心操作,2，3->固液分离,4->放试管，5->烘干操作,6->充电，7->关机，8->模式切换，9->状态轮询
list_key = []  # 存放已经接收成功的指令
quary = ''
param=''
model_send='0'#模式0代表参观，1代表完整
# 通信

host = '192.168.224.6'
port = 50000
buffsize = 2048
ADDR = (host, port)
HOST = '127.0.0.1'
PORT = 61000
BUFFSIZE = 2048
ADDR_agv = (HOST, PORT)

server_agv = socket(AF_INET, SOCK_STREAM)
server_agv.setsockopt(SOL_SOCKET,SO_REUSEADDR,1)
server_agv.bind(ADDR_agv)
server_agv.listen(3)
server_agvClient, server_agvaddr = server_agv.accept()
# logging.basicConfig(level=logging.DEBUG,#控制台打印的日志级别
#                     filename="/home/sia/YuLin_lab/src/action_tset1/"
#                 + str(datetime.datetime.now().date().isoformat())
#                 + "info.log",
#                     filemode='a',##模式，有w和a，w就是写模式，每次都会重新写日志，覆盖之前的日志
#                     #a是追加模式，默认如果不写的话，就是追加模式
#                     format=
#                     "%(asctime)s %(filename)s[line:%(lineno)d] %(levelname)s %(message)s"
#                     #日志格式
#                     )
logging.basicConfig(filename="/home/sia/YuLin_lab/src/action_tset1/"
                + str(datetime.datetime.now().date().isoformat())
                + "info.log", level=logging.DEBUG)
logging.debug("运行状态记录")
# 函数功能 ：日志读取，读取最后一行
def read_log(filename):
    filesize = os.path.getsize(filename)
    if filesize == 0:
        arr = 'THE LOG IS NO EXIT'
        return arr
    else:
        fo = open(filename, "rb")  # 一定要用'rb'因为seek 是以bytes来计算的
        fo.seek(-2, 2)
        result = fo.readline()

        mid_log = result[0]
        i = -1
        while True:
            if i < -1024:
                break
            i = i - 1
            fo.seek(i, 2)
            result = fo.readline()
            mid_log = result[0]
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
# 函数功能 ：获得夹爪张开状态 0-255      
def statusInterpreter(status):
    """Generate a string according to the current value of the status variables."""

    output = '\n-----\n2F gripper status interpreter\n-----\n'

    # gPO
    output += 'gPO = ' + str(status.gPO) + ': '
    output += 'Position of Fingers: ' + str(status.gPO) + '/255\n'

    return output

isCobotOpen = False
ip='192.168.1.10'
duco_cobot = DucoCobot(ip, 7003)

# 函数功能 ：对总控发送的指令进行逻辑判断，执行或返回错误类型
def sequence(key, list_1):
    global Isruning
    global mid
    global param
    global isCobotOpen


    list_command_num = [0, 1, 2, 3, 4, 5, 6, 7,8]
    if key in list_command_num:
        if key in [8]:
            global model_send
            model_send=param
            Isruning = 1
            list_1.append(key)
            model_array = "task!send success!"
            print('model_array',model_array)
        else:
            Isruning = 1
            list_1.append(key)
            model_array = "task!send success!"
            print('model_array',model_array)

        return model_array

    elif key == 9:
        if isCobotOpen == False:
            if duco_cobot.open() == 0:
                isCobotOpen = True
            else:
                isCobotOpen = False

        if isCobotOpen == False:
            agv_query = "0"
            forward = 'query!'
            server_agvClient.send(agv_query.encode())
            plat = server_agvClient.recv(buffsize).decode()
            agv_state = re.findall(r"\d+\.?\d*", plat)
            robotiq_output = rospy.wait_for_message("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input)
            statusInterpreter(robotiq_output)

            arr_send= forward + str(agv_state[0]) + "%" + str(agv_state[1]) + "!" + str(0) + "%" + str(0) + "%" + str(
                0) + "%" + str(0) + "%" + str(0) + "%" + str(0) + "!" + str(
                0) + "%" + str(0) + "%" + str(0) + "!" + str(robotiq_output.gPO) + "!" + '002'+ "!" + str(
                agv_state[2]) + "!"+model_send+"!"
            # arr_send="task!failed 03!"+"robot status error"
            mid=1
       
            return arr_send
        else:
            arm_state=duco_cobot.get_robot_state()
            if arm_state[0]==6:
                print("机械臂状态正确，连接成功")
                filename = '/home/sia/YuLin_lab/src/action_tset1/' + str(datetime.datetime.now().date().isoformat()) + 'error.log'
                log_array= read_log(filename)
                
                robotiq_output = rospy.wait_for_message("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input)
                statusInterpreter(robotiq_output)

                arm_joint = duco_cobot.get_actual_joints_position()
                arm_pose = duco_cobot.get_tcp_pose()

                agv_query = "0"
                forward = 'query!'
                server_agvClient.send(agv_query.encode())
                plat = server_agvClient.recv(buffsize).decode()
                agv_state = re.findall(r"\d+\.?\d*", plat)
                if len(list_key)==0 or (Isruning + mid == 0):  # 状态001
                    command_send = str(0) + "01"
                else:
                    command_send = str(list_key[-1] + 1) + str(flag).zfill(2)    
                query_send = forward + str(agv_state[0]) + "%" + str(agv_state[1]) + "!" + str(arm_joint[0]) + "%" + str(arm_joint[1]) + "%" + str(
                    arm_joint[2]) + "%" + str(arm_joint[3]) + "%" + str(arm_joint[4]) + "%" + str(arm_joint[5]) + "!" + str(
                    arm_pose[0]) + "%" + str(arm_pose[1]) + "%" + str(arm_pose[2]) + "!" + str(robotiq_output.gPO) + "!" + command_send + "!" + str(
                    agv_state[2]) + "!"+model_send+"!"
                if "task!failed 03!" in log_array:
                    arr_failed=log_array.find("task!failed 03!")
                    arr_send=query_send+'\n'+log_array[arr_failed:]
                else:
                    arr_send=query_send
                print(arr_send)
                mid=0
                return arr_send
            else:
                agv_query = "0"
                forward = 'query!'
                server_agvClient.send(agv_query.encode())
                plat = server_agvClient.recv(buffsize).decode()
                agv_state = re.findall(r"\d+\.?\d*", plat)
                robotiq_output = rospy.wait_for_message("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input)
                statusInterpreter(robotiq_output)

                arr_send= forward + str(agv_state[0]) + "%" + str(agv_state[1]) + "!" + str(0) + "%" + str(0) + "%" + str(
                    0) + "%" + str(0) + "%" + str(0) + "%" + str(0) + "!" + str(
                    0) + "%" + str(0) + "%" + str(0) + "!" + str(robotiq_output.gPO) + "!" + '002'+ "!" + str(
                    agv_state[2]) + "!"+model_send+"!"
                # arr_send="task!failed 03!"+"robot status error"
                mid=1
        
                return arr_send
    elif key == 10:
        if param =='0':
            duco_cobot.power_off(True)
            duco_cobot.disable(True) 
            arr_send = "task!send success!，掉使能成功"
        elif param =='1':
            duco_cobot.power_on(True)
            duco_cobot.enable(True) 
            arr_send = "task!send success!，上使能成功"
        else:
            arr_send="task!failed 02!"
        return arr_send

    else:
        arr="task!failed 02!"
        return arr


#   函数功能：转换接受信息，举例把task!get_sample!转换成key对应的0
def trans_key(command):
    length=command.count('!')
    if length>=2:
        list_command = command.split('!')
        if len(list_command)>3:
            global param
            param = list_command[2]
        task=list_command[1]
        task_order=["get_sample","centrifuge","seperation_acid","seperation_alkali","put_sample","drying","recharge","poweroff","model","robotstate","enable"]

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
        time.sleep(0.5)
        print("waiting for new command")
        # print(mid)


# 函数：三个函数为action的参数函数
def done_cb(status, result):
    if status == actionlib.GoalStatus.SUCCEEDED:
        logging.debug("响应成功")
        global flag
        flag = flag + 1

        # rospy.loginfo("第%d步指令发送",flag)
        callback(flag)
    else:

        rospy.loginfo("%d步响应失败,请查看", flag)


def active_cb():
    logging.debug("连接建立...")
    rospy.loginfo("第%d步指令发送", flag)


def feedback_cb(feedback):
    rospy.loginfo("进度：%2s", feedback.progess)

# 线程函数：和control进行TCP通信
def thread_fun():
    tctime = socket(AF_INET,SOCK_STREAM)
    tctime.setsockopt(SOL_SOCKET,SO_REUSEADDR , 1)
    tctime.bind(ADDR)
    tctime.listen(3)   
    print('Wait for connection ...')
    while not rospy.is_shutdown():
        try:
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
            command_receive = tctimeClient.recv(buffsize).decode()
            logging.debug("指令序列")
            rospy.loginfo(command_receive)
            print("收到的指令为",command_receive)
            print("收到的指令为",command_receive)
            print("收到的指令为",command_receive)
            if not command_receive:
                break
            global key
            str0 = {9,10}
            command_key = trans_key(command_receive)
            
          
            if mid == 0 or command_key in str0:
                if command_key in str0:
                    arr_send = sequence(command_key, list_key)
                    print(arr_send)
                    tctimeClient.send(arr_send.encode())
                    logging.debug(arr_send)
                else:
                    arr_send=sequence(command_key,list_key)
                    if arr_send=="task!failed 02!":
                        tctimeClient.send(arr_send.encode())
                        logging.debug(arr_send)
                    else:
                        key=command_key
                        Isruning=1
                        tctimeClient.send(arr_send.encode())
                        logging.debug(arr_send)
                # list_key.append(key)
            # elif mid==1:
            elif mid == 1 and flag == 1:
                arr_send = "请检查机械臂状态是否正常，再输入一次指令"
                print("   ：", arr_send)
                logging.error("请检查机械臂状态是否正常，再输入一次指令")
                tctimeClient.send(arr_send.encode())
                logging.debug("请检查机械臂状态是否正常，再输入一次指令")
            elif mid == 1 and command_key not in str0:
                arr_send="task!failed 01!"
                tctimeClient.send(arr_send.encode())
                logging.debug(arr_send)
            time.sleep(0.01)


if __name__ == "__main__":
    # server_agv = socket(AF_INET, SOCK_STREAM)
    # server_agv.setsockopt(SOL_SOCKET,SO_REUSEADDR,1)
    # server_agv.bind(ADDR_agv)
    # server_agv.listen(3)
    # server_agvClient, server_agvaddr = server_agv.accept()
    # 初始化ROS节点
    rospy.init_node("action_client_py")
    # 开线程
    thd_A = threading.Thread(target=thread_fun,daemon=True)
    thd_A.start()
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

