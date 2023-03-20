
from tube_motor import motor
from aruco import *
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg
import csv
import threading
import roslib
sys.path.append("gen_py")
sys.path.append("lib")
from thrift import Thrift
roslib.load_manifest("robotiq_2f_gripper_control")
# 新松
# sys.path.append('gen_py')
# sys.path.append('lib')
# from DucoCobot import DucoCobot

# from thrift.transport import TSocket
# from thrift.transport import TTransport
# from thrift.protocol import TBinaryProtocol
# from gen_py.robot.ttypes import StateRobot, StateProgram, OperationMode, TaskState, Op

# # Connect!
# ip = '192.168.1.10'
# duco_cobot = DucoCobot(ip, 7003)
# duco_cobot.open()
# duco_cobot.power_on(True)
# duco_cobot.enable(True)
aruco = aruco()

stopheartthread = False
midpose = [
    1.668006181716919,
    0.10159657150506973,
    -1.3602924346923828,
    -0.30960047245025635,
    1.5602844953536987,
    -0.629919171333313,
]
header = ["x", "y", "z", "r", "p", "y"]
# csvflag = 1
speedl = 0.8
speedj = 80


def hearthread_fun():
    """新松心跳包线程"""
    duco_heartbeat = DucoCobot(ip, 7003)

    if duco_heartbeat.open() == 0:
        while not stopheartthread:
            duco_heartbeat.rpc_heartbeat()
            sleep(0.4)
        duco_heartbeat.close()


def mark_size(size):
    rospy.set_param("/aruco_single/marker_size", size)
    sleep(0.5)
    param = rospy.get_param("/aruco_single/marker_size")
    while param != size:
        rospy.set_param("/aruco_single/marker_size", size)
        sleep(0.5)
        param = rospy.get_param("/aruco_single/marker_size")


def mark_id(id):
    rospy.set_param("/aruco_single/marker_id", id)
    sleep(0.5)
    param = rospy.get_param("/aruco_single/marker_id")
    while param != id:
        rospy.set_param("/aruco_single/marker_id", id)
        sleep(0.5)
        param = rospy.get_param("/aruco_single/marker_id")


def robotiq(ACT, GTO, PR, ATR, SP, FR):
    """
    rACT: 复位:0,激活:1
    rGTO: 1
    rPR:  位置
    rATR: 0
    rSP:  速度
    rFR:  力
    """

    pub = rospy.Publisher(
        "Robotiq2FGripperRobotOutput",
        outputMsg.Robotiq2FGripper_robot_output,
        queue_size=1,
    )
    command = outputMsg.Robotiq2FGripper_robot_output()
    command.rACT = ACT
    command.rGTO = GTO
    command.rPR = PR
    command.rATR = ATR
    command.rSP = SP
    command.rFR = FR
    for i in range(3):
        pub.publish(command)
        rospy.sleep(0.1)
    sleep(2)


def key(mid, key_open, force):
    duco_cobot.movel(mid, speedl, 0.2, 0, [], "", "", True)
    ret = duco_cobot.movel(key_open, speedl - 0.2, 0.2, 0, [], "", "", True)
    print("到达按键上方", ret)
    # 力控按键
    duco_cobot.fc_config(
        [False, False, True, False, False, False],
        [0, 0, force, 0, 0, 0],
        [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
        [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198],
        [0, 0, 0, 0, 0, 0],
        "default",
        "default",
        0,
    )
    duco_cobot.fc_start()
    duco_cobot.fc_wait_pos([0, 0, 0.1, 0, 0, 0], [
                           0, 0, 0.005, 0, 0, 0], False, 0, 5000)
    duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                           0, 0, 0, 0, 0, 0], False, 0, 5000)
    duco_cobot.fc_wait_ft([0, 0, force, 0, 0, 0], [
                          0, 0, 2, 0, 0, 0], False, 1, 500000)
    duco_cobot.fc_wait_logic([0, 0, 1])
    duco_cobot.fc_move()
    duco_cobot.fc_stop()
    sleep(0.5)
    # 力控返回按键上方
    duco_cobot.fc_config(
        [False, False, True, False, False, False],
        [0, 0, -force, 0, 0, 0],
        [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
        [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198],
        [0, 0, 0, 0, 0, 0],
        "default",
        "default",
        0,
    )
    duco_cobot.fc_start()
    duco_cobot.fc_wait_pos([0, 0, 0.03, 0, 0, 0], [
                           0, 0, 0.01, 0, 0, 0], False, 0, 5000)
    duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                           0, 0, 0, 0, 0, 0], False, 0, 5000)
    duco_cobot.fc_wait_ft([0, 0, -force, 0, 0, 0],
                          [0, 0, 2, 0, 0, 0], False, 1, 500000)
    duco_cobot.fc_wait_logic([1, 0, 2])
    duco_cobot.fc_move()
    duco_cobot.fc_stop()
    sleep(0.5)
    duco_cobot.movel(mid, speedl, 0.2, 0, [], "", "", True)


def get_T(pose1, pose2):
    """
    获得过渡矩阵
    pose1:视觉定位点位姿
    pose2:目标点位姿
    返回 过渡矩阵T
    """
    pose1_T = T.rpy2T(pose1)  # 视觉定位点位姿
    pose2_T = T.rpy2T(pose2)  # 目标点位姿
    offset = np.dot(np.linalg.inv(pose1_T), pose2_T)  # 求的过渡矩阵
    # transition=T.T2rot(transition_T)
    return offset


def get_t(pose1, pose2_T):
    """
    相对于二维码的姿态
    pose1 视觉定位
    pose2 过渡矩阵
    返回 目标点pose
    """
    pose1_T = T.rpy2T(pose1)  # 视觉定位点位姿
    # pose2_T=T.rot2T(pose2)       #过渡矩阵
    transition_T = np.dot(pose1_T, pose2_T)  # 目标点旋转平移矩阵
    pose = T.T2rpy(transition_T)  # 目标点pose
    return pose


def circle(axis, edge, theta):
    """
    函数功能,从A或B旋转一定的角度angle
    axis: 圆心轴的位姿
    edge:   A或B点
    angle:  旋转角度（/度）
    """

    goal2baselink = T.rpy2T(axis)  # 目标相对于基座的T
    eelink2baselink = T.rpy2T(edge)  # 当前位姿转换成T
    eelink2goal = np.dot(np.linalg.inv(goal2baselink), eelink2baselink)
    P, M = T.T2rpy_2(eelink2goal)  # 旋转平移矩阵
    radius = math.sqrt(math.pow(P[0], 2) + math.pow(P[1], 2))  # R
    angle = math.atan2(P[1], P[0])  # rad

    tran = [radius * math.cos(angle + theta), radius *
            math.sin(angle + theta), P[2]]
    euler = [M[0], M[1], M[2] + theta]
    eelink2goal = [
        tran[0],
        tran[1],
        tran[2],
        euler[0],
        euler[1],
        euler[2],
    ]  # 转动后目标相对于基座的T
    eelink2goal = T.rpy2T(eelink2goal)
    target = np.dot(goal2baselink, eelink2goal)
    target = T.T2rpy(target)
    return target


class cvs_get(threading.Thread):
    """
    开线程CSV保存数据
    """

    def __init__(self):
        super().__init__()  # 必须有否这报错
        # threading.Thread.__init__(self)   #super().__init__()通用
        self.daemon = True
        self.thread_stop = False

    def run(self):
        duco_cobot = DucoCobot("192.168.1.10", 7003)
        duco_cobot.open()
        global csvflag
        with open(
            "/home/sia/YuLin_lab/src/DucoCobotAPI_py/cvs_test.csv",
            "a+",
            encoding="utf-8-sig",
        ) as file_obj:
            # 1:创建writer对象
            writer = csv.writer(file_obj)
            # 2:写表头
            writer.writerow(header)
            while csvflag:
                six_P = duco_cobot.get_tcp_pose()
                six_F = duco_cobot.get_tcp_force()
                writer.writerow(six_P)
                writer.writerow(six_F)
                sleep(0.1)
            print("csv threading end")

    def stop(self):
        print("Last thread stop!")
        self.thread_stop = True


class testtube(object):
    def __init__(self):
        # 离心机操作：取放试管中间点
        self.tube_middleA_j = [
            1.6679942607879639,
            0.13228817284107208,
            -1.5182205438613892,
            -0.1812257021665573,
            1.560260534286499,
            -0.630290687084198,
        ]
        self.tube_middleB_j = [
            1.8543009757995605,
            -0.9173381924629211,
            -1.2019808292388916,
            2.118163585662842,
            -1.3160853385925293,
            2.3607800006866455,
        ]
        self.tube_middleB_j1 = [
            1.7878724336624146,
            -0.264102041721344,
            -1.4782891273498535,
            1.6719069480895996,
            -0.03256284445524216,
            2.3607442378997803,
        ]
        self.tube_middleB_j2 = [
            0.7730482816696167,
            0.5933213233947754,
            -2.4068868160247803,
            1.8143993616104126,
            0.8782861828804016,
            2.3542966842651367,
        ]
        # 离心机操作：试管架取试管至离心机
        self.tube_getA1 = [
            -0.02102518640458584-0.003,
            -0.5755000114440918 - 0.0007+0.003,
            0.5018289187431335 - 0.002,
            -3.1415581703186035,
            -1.5060706573422067e-05,
            -3.1415655612945557,
        ]
        self.tube_getA2 = [
            -0.08533629775047302-0.003,
            -0.575955331325531 - 0.0007+0.003,
            0.5018990303993225 - 0.0003-0.001-0.0004,
            3.141589641571045,
            -2.3479329684050754e-05,
            -3.1415669918060303,
        ]
        self.tube_getA3 = [
            -0.15150830149650574-0.003,
            -0.5760762691497803 - 0.0007+0.003,
            0.5018643902778625 - 0.0005-0.001-0.0004,
            -3.1415746212005615,
            -1.9144245015922934e-05,
            -3.1415579319000244,
        ]
        # self.tube_getA4 = [-0.019586972892284393, -0.6398314237594604-0.0005, 0.5027056336402893, -3.141571521759033,
        #                    -1.261985471501248e-05, -3.141564130783081]
        self.tube_getB1 = [0.11685050278902054, -0.8751742839813232, 0.2274453043937683, 1.5708394050598145, 2.4872853828128427e-05, -2.603635311126709]
        
        self.tube_getB2 = [0.11685050278902054-0.065, -0.8751742839813232, 0.2274453043937683, 1.5708394050598145, 2.4872853828128427e-05, -2.603635311126709]
        self.tube_getB3 = [0.11685050278902054-0.13, -0.8751742839813232, 0.2274453043937683, 1.5708394050598145, 2.4872853828128427e-05, -2.603635311126709]
        # 离心机操作：离心机取试管放至试管架
        self.tube_putA1 = [
            -0.02102518640458584-0.003,
            -0.5755000114440918 - 0.0007+0.003,
            0.5018289187431335 - 0.002+0.15,
            -3.1415581703186035,
            -1.5060706573422067e-05,
            -3.1415655612945557,
        ]
        self.tube_putA2 = [
            -0.08533629775047302-0.003,
            -0.575955331325531 - 0.0007+0.003,
            0.5018990303993225 - 0.0003-0.001+0.15,
            3.141589641571045,
            -2.3479329684050754e-05,
            -3.1415669918060303,
        ]
        self.tube_putA3 = [
            -0.15150830149650574-0.003,
            -0.5760762691497803 - 0.0007+0.003,
            0.5018643902778625 - 0.0005+0.15,
            -3.1415746212005615,
            -1.9144245015922934e-05,
            -3.1415579319000244,
        ]
        self.tube_putB1 = [0.11685050278902054, -0.8751742839813232, 0.2274453043937683+0.13, 1.5708394050598145, 2.4872853828128427e-05, -2.603635311126709]
        self.tube_putB2 = [0.11685050278902054-0.065, -0.8751742839813232, 0.2274453043937683+0.13, 1.5708394050598145, 2.4872853828128427e-05, -2.603635311126709]
        self.tube_putB3 = [0.11685050278902054-0.13, -0.8751742839813232, 0.2274453043937683+0.13, 1.5708394050598145, 2.4872853828128427e-05, -2.603635311126709]
        # 放盖（试管上）
        self.lid_putA1 = [-0.022882957011461258, -0.5745800733566284, 0.4996538758277893+0.0004+0.05+0.035, -3.141507863998413, -4.914805685984902e-05, -3.1414849758148193]
        self.lid_putA2 = [-0.022882957011461258-0.065, -0.5745800733566284+0.0003, 0.4996538758277893+0.05+0.035, -3.141507863998413, -4.914805685984902e-05, -3.1414849758148193]
        self.lid_putA3 = [-0.022882957011461258-0.13, -0.5745800733566284+0.0003, 0.4996538758277893+0.05+0.035, -3.141507863998413, -4.914805685984902e-05, -3.1414849758148193]
        self.lid_putB1 = [-0.022882957011461258-0.0003, -0.5745800733566284-0.065, 0.4996538758277893+0.05+0.035, -3.141507863998413, -4.914805685984902e-05, -3.1414849758148193]
        self.lid_putB2 =[-0.022882957011461258-0.065-0.0003, -0.5745800733566284-0.065+0.0003, 0.4996538758277893+0.05+0.035, -3.141507863998413, -4.914805685984902e-05, -3.1414849758148193]
        self.lid_putB3 = [-0.022882957011461258-0.13-0.0003, -0.5745800733566284-0.065+0.0003, 0.4996538758277893+0.05+0.035, -3.141507863998413, -4.914805685984902e-05, -3.1414849758148193]
        # 取盖（试管上）
        self.lid_getA1 = [-0.022882957011461258, -0.5745800733566284, 0.4996538758277893+0.0004, -3.141507863998413, -4.914805685984902e-05, -3.1414849758148193]
        self.lid_getA2 = [-0.022882957011461258-0.065, -0.5745800733566284, 0.4996538758277893, -3.141507863998413, -4.914805685984902e-05, -3.1414849758148193]
        self.lid_getA3 = [-0.022882957011461258-0.13, -0.5745800733566284, 0.4996538758277893, -3.141507863998413, -4.914805685984902e-05, -3.1414849758148193]
        self.lid_getB1 = [-0.022882957011461258, -0.5745800733566284-0.065, 0.4996538758277893, -3.141507863998413, -4.914805685984902e-05, -3.1414849758148193]
        self.lid_getB2 = [-0.022882957011461258-0.065, -0.5745800733566284-0.065, 0.4996538758277893, -3.141507863998413, -4.914805685984902e-05, -3.1414849758148193]
        self.lid_getB3 = [-0.022882957011461258-0.13, -0.5745800733566284-0.065, 0.4996538758277893, -3.141507863998413, -4.914805685984902e-05, -3.1414849758148193]
        # 放盖（平台上）
        self.lid_middle_j = [
            2.1970736980438232,
            0.1850786805152893,
            -1.890318751335144 + 0.01,
            0.1344507783651352,
            1.5681461095809937,
            -0.15728531777858734,
        ]
        self.lid_put_A1 = [0.17379549145698547, -0.640283465385437, 0.34330105781555176, -3.141576051712036, -8.422746759606525e-06, 3.141589879989624]
        self.lid_put_A2 = [0.17379549145698547, -0.640283465385437+0.065, 0.34330105781555176, -3.141576051712036, -8.422746759606525e-06, 3.141589879989624]
        self.lid_put_A3 = [0.17379549145698547-0.0009, -0.640283465385437+0.13-0.0005, 0.34330105781555176, -3.141576051712036, -8.422746759606525e-06, 3.141589879989624]
        self.lid_put_B1 = [0.17379549145698547+0.065, -0.640283465385437, 0.34330105781555176, -3.141576051712036, -8.422746759606525e-06, 3.141589879989624]
        self.lid_put_B2 = [0.17379549145698547+0.065, -0.640283465385437+0.065, 0.34330105781555176, -3.141576051712036, -8.422746759606525e-06, 3.141589879989624]
        self.lid_put_B3 = [0.17379549145698547+0.065-0.0007, -0.640283465385437+0.13-0.0003, 0.34330105781555176, -3.141576051712036, -8.422746759606525e-06, 3.141589879989624]
        # 取盖（平台上）
        self.lid_get_A1 = [0.17379549145698547-0.0005, -0.640283465385437, 0.3194091215133667+0.01 , -3.141576051712036, -8.422746759606525e-06, 3.141589879989624]
        self.lid_get_A2 = [0.17379549145698547-0.0002-0.0005, -0.640283465385437+0.065-0.0002, 0.3194091215133667+0.01 , -3.141576051712036, -8.422746759606525e-06, 3.141589879989624]
        self.lid_get_A3 = [0.17379549145698547-0.0005-0.0009, -0.640283465385437+0.13-0.0005, 0.3194091215133667 +0.01, -3.141576051712036, -8.422746759606525e-06, 3.141589879989624]
        self.lid_get_B1 = [0.17379549145698547+0.065-0.0009, -0.640283465385437, 0.3194091215133667+0.01 , -3.141576051712036, -8.422746759606525e-06, 3.141589879989624]
        self.lid_get_B2 = [0.17379549145698547+0.065-0.0005, -0.640283465385437+0.065, 0.3194091215133667+0.01 , -3.141576051712036, -8.422746759606525e-06, 3.141589879989624]
        self.lid_get_B3 = [0.17379549145698547+0.065-0.0003-0.0009, -0.640283465385437+0.13-0.0003, 0.3194091215133667+0.01 , -3.141576051712036, -8.422746759606525e-06, 3.141589879989624]

    def get_tubeA(self, putpose, getpose):
        """
        离心机操作：试管架上取试管放入离心机A半圆3个试管
        """
        # 移动至夹试管中间点
        duco_cobot.movej(self.tube_middleA_j, speedj, 10, 0, True)
        robotiq(1, 1, 120, 0, 100, 1)
        # 夹试管
        duco_cobot.movel(putpose, speedl - 0.1, 0.2, 0, [], "", "", True)
        ret = duco_cobot.movel(getpose, speedl - 0.3, 0.1, 0, [], "", "", True)
        print("移动至试管夹取点，准备夹试管", ret)
        robotiq(1, 1, 255, 0, 100, 150)
        # 力控夹起试管
        curpose = duco_cobot.get_tcp_pose()
        duco_cobot.fc_config(
            [False, False, True, False, False, False],
            [0, 0, 35, 0, 0, 0],
            [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos(
            [0, 0, 0.15, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, 30, 0, 0, 0], [
                              0, 0, 2, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        print("试管架夹起试管forcemove stop")
        sleep(0.5)
        curpose1 = duco_cobot.get_tcp_pose()
        # 若试管未完全提起，受力停止了，报错
        if curpose1[2] - curpose[1] < 0.12:
            print("受力停止了，取出试管未成功")
            robotiq(1, 1, 150, 0, 100, 10)
            assert False
        # 移动至夹试管中间点
        duco_cobot.movej(self.tube_middleA_j, speedj - 20, 20, 0, True)

    def get_tubeB(self, putpose, getpose):
        """
        离心机操作：试管架上取试管放入离心机B半圆3个试管
        """
        # 移动至夹试管中间点
        duco_cobot.movej(self.tube_middleB_j2, speedj - 20, 10, 0, True)
        duco_cobot.movej(self.tube_middleB_j1, speedj - 10, 20, 0, True)
        duco_cobot.movej(self.tube_middleB_j, speedj - 10, 20, 0, True)
        robotiq(1, 1, 120, 0, 100, 1)
        # 夹试管
        duco_cobot.movel(putpose, speedl - 0.1, 0.2, 0, [], "", "", True)
        ret = duco_cobot.movel(getpose, speedl - 0.3, 0.2, 0, [], "", "", True)
        print("移动至试管夹取点，准备夹试管", ret)
        robotiq(1, 1, 255, 0, 100, 150)
        # 力控夹起试管
        curpose = duco_cobot.get_tcp_pose()
        duco_cobot.fc_config(
            [False, True, False, False, False, False],
            [0, -35, 0, 0, 0, 0],
            [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos(
            [0, 0, 0.15, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft(
            [0, -30, 0, 0, 0, 0], [0, 2, 0, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        sleep(1)
        curpose1 = duco_cobot.get_tcp_pose()
        # 若试管未完全提起，受力停止了，报错
        if curpose1[2] - curpose[1] < 0.12:
            print("受力停止了，取出试管未成功")
            robotiq(1, 1, 150, 0, 100, 10)
            assert False
        # 移动至离心机放试管中间点
        duco_cobot.movej(self.tube_middleB_j, 20, 10, 0, True)
        duco_cobot.movej(self.tube_middleB_j1, speedj - 10, 10, 0, True)
        duco_cobot.movej(self.tube_middleB_j2, speedj - 10, 10, 0, True)

    def put_tubeA(self, putpose):
        """
        离心机操作:从离心机A半圆(3个试管)取出试管放到试管架上
        """
        # 移动至夹试管中间点
        duco_cobot.movej(self.tube_middleA_j, speedj, 20, 0, True)
        # 放试管
        duco_cobot.movel(putpose, speedl - 0.3, 0.2, 0, [], "", "", True)
        curpose = duco_cobot.get_tcp_pose()
        duco_cobot.fc_config(
            [False, False, True, False, False, False],
            [0, 0, -25, 0, 0, 0],
            [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos(
            [0, 0, -0.155, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft(
            [0, 0, -20, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        print("forcemove stop")
        sleep(0.5)
        curpose1 = duco_cobot.get_tcp_pose()
        if curpose[2] - curpose1[2] < 0.08:
            robotiq(1, 1, 120, 0, 100, 1)
            print("受力停止了，取出试管未成功")
            assert False
        robotiq(1, 1, 120, 0, 100, 1)
        # 移动至夹试管中间点
        duco_cobot.movej(self.tube_middleA_j, speedj - 20, 10, 0, True)

    def put_tubeB(self, putpose):
        """
        离心机操作：从离心机B半圆（3个试管）取出试管放到试管架上
        """
        # 移动至夹试管中间点
        duco_cobot.movej(self.tube_middleB_j2, 20, 10, 0, True)
        duco_cobot.movej(self.tube_middleB_j1, speedj - 10, 10, 0, True)
        duco_cobot.movej(self.tube_middleB_j, speedj - 10, 10, 0, True)
        # 放试管
        duco_cobot.movel(putpose, speedl - 0.3, 0.2, 0, [], "", "", True)
        duco_cobot.fc_config(
            [False, True, False, False, False, False],
            [0, 25, 0, 0, 0, 0],
            [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos(
            [0, 0, -0.13, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 20, 0, 0, 0, 0], [
                              0, 1, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        print("forcemove stop")
        sleep(0.5)
        robotiq(1, 1, 120, 0, 100, 1)
        # 移动至离心机放试管中间点
        duco_cobot.movej(self.tube_middleB_j, 20, 10, 0, True)
        duco_cobot.movej(self.tube_middleB_j1, speedj - 10, 10, 0, True)
        duco_cobot.movej(self.tube_middleB_j2, speedj - 10, 10, 0, True)

    def lid_get(self, getlid, putlid, put_lid, num):
        """
        试管上取试管盖
        """
        # 移动至拔试管盖中间点
        duco_cobot.movej(self.tube_middleA_j, speedj, 10, 0, True)
        if num == 0:
            robotiq(1, 1, 120, 0, 200, 1)
        motor.motor_position(187500-1000, 0)
        # 拔试管盖
        duco_cobot.movel(putlid, speedl - 0.2, 0.2, 0, [], "", "", True)
        duco_cobot.movel(getlid, speedl - 0.3, 0.2, 0, [], "", "", True)
        robotiq(1, 1, 255, 0, 100, 250)
        sleep(0.5)
        curpose = duco_cobot.get_tcp_pose()
        duco_cobot.fc_config(
            [False, False, True, False, False, False],
            [0, 0, 60, 0, 0, 0],
            [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos(
            [0, 0, 0.1, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft(
            [100, 100, 100, 0, 0, 0], [5, 5, 5, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        print("forcemove stop")
        curpose1 = duco_cobot.get_tcp_pose()
        if curpose1[2] - curpose[2] < 0.07:
            robotiq(1, 1, 120, 0, 200, 1)
            print("取试管盖受力停止了，取出试管未成功")
            assert False
        motor.motor_position(179000-1000, 0)
        # robotiq(1, 1, 120, 0, 200, 1)
        # 放试管盖（平台上）
        duco_cobot.movej(self.lid_middle_j, speedj, 10, 0, True)
        duco_cobot.movel(put_lid, speedl - 0.3, 0.2, 0, [], "", "", True)
        duco_cobot.fc_config(
            [False, False, True, False, False, False],
            [0, 0, -20, 0, 0, 0],
            [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_ft([0, 0, -10, 0, 0, 0],
                              [0, 0, 2, 0, 0, 0], False, 0, 3000)
        duco_cobot.fc_wait_logic([0, 0, 1])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        print("forcemove stop")
        sleep(0.5)
        robotiq(1, 1, 120, 0, 100, 1)
        duco_cobot.movej(self.lid_middle_j, speedj - 10, 10, 0, True)

    def lid_put(self, putlid, put_lid, get_lid, num):
        """
        试管上放试管盖
        """
        # 移动至放试管盖中间点
        if num == 0:
            robotiq(1, 1, 120, 0, 100, 1)
        duco_cobot.movej(self.lid_middle_j, speedj, 10, 0, True)
        # 取试管盖
        duco_cobot.movel(put_lid, speedl - 0.1, 0.2, 0, [], "", "", True)
        duco_cobot.movel(get_lid, speedl - 0.3, 0.2, 0, [], "", "", True)
        robotiq(1, 1, 255, 0, 100, 200)
        # 力控夹起试管盖
        duco_cobot.fc_config(
            [False, False, True, False, False, False],
            [0, 0, 20, 0, 0, 0],
            [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos(
            [0, 0, 0.02, 0, 0, 0], [0, 0, 0.005, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, 0, 0, 0, 0], [
                              0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 0])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        sleep(1)
        duco_cobot.movej(self.lid_middle_j, speedj - 20, 10, 0, True)
        duco_cobot.movel(putlid, speedl - 0.3, 0.2, 0, [], "", "", True)
        # 力控盖盖子
        curpose = duco_cobot.get_tcp_pose()
        # -45N
        duco_cobot.fc_config(
            [False, False, True, False, False, False],
            [0, 0, -55, 0, 0, 0],
            [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 4000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, -50, 0, 0, 0],
                              [0, 0, 2, 0, 0, 0], False, 0, 4000)
        duco_cobot.fc_wait_logic([0, 0, 1])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        sleep(1)
        curpose1 = duco_cobot.get_tcp_pose()
        if curpose[2] - curpose1[2] < 0.039:
            print("放试管盖受力停止了，放试管盖未成功")
            robotiq(1, 1, 150, 0, 100, 10)
            assert False
        robotiq(1, 1, 120, 0, 100, 1)
        duco_cobot.movel(putlid, speedl - 0.1, 0.2, 0, [], "", "", True)

    def lidget(self, test):
        """
        从试管架上取6个盖,放在平台上
        """
        if test == 0:
            # 测试版
            self.lid_get(self.lid_getA1, self.lid_putA1, self.lid_put_A1, 0)
            robotiq(1, 1, 250, 0, 100, 1)
        if test == 1:
            # 正式版
            self.lid_get(self.lid_getA1, self.lid_putA1, self.lid_put_A1, 0)
            self.lid_get(self.lid_getB1, self.lid_putB1, self.lid_put_B1, 1)
            self.lid_get(self.lid_getA2, self.lid_putA2, self.lid_put_A2, 1)
            self.lid_get(self.lid_getB2, self.lid_putB2, self.lid_put_B2, 1)
            self.lid_get(self.lid_getA3, self.lid_putA3, self.lid_put_A3, 1)
            self.lid_get(self.lid_getB3, self.lid_putB3, self.lid_put_B3, 1)
            robotiq(1, 1, 250, 0, 100, 1)
        else:
            print("输入错误")

    def lidput(self, test):
        """
        从平台上取6个盖,放到试管上
        """
        motor.motor_position(183000-1000, 10)
        # 测试
        if test == 0:
            self.lid_put(self.lid_putA1, self.lid_put_A1, self.lid_get_A1, 0)
            motor.motor_position(179000-500, 0)
            # 取样品结束合并夹爪
            robotiq(1, 1, 255, 0, 100, 50)
        # 正式版
        elif test == 1:
            self.lid_put(self.lid_putA3, self.lid_put_A3, self.lid_get_A3, 0)
            self.lid_put(self.lid_putB3, self.lid_put_B3, self.lid_get_B3, 1)
            self.lid_put(self.lid_putA2, self.lid_put_A2, self.lid_get_A2, 1)
            self.lid_put(self.lid_putB2, self.lid_put_B2, self.lid_get_B2, 1)
            self.lid_put(self.lid_putA1, self.lid_put_A1, self.lid_get_A1, 1)
            self.lid_put(self.lid_putB1, self.lid_put_B1, self.lid_get_B1, 1)
            motor.motor_position(179000-500, 0)
            # 取样品结束合并夹爪
            robotiq(1, 1, 255, 0, 100, 50)
        else:
            print("输入错误")
        duco_cobot.movej(midpose, 50, 10, 0, True)


class getsample(object):
    def __init__(self):
        self.pose_aruco2_j1 = [
            0.9242292642593384,
            -0.5668954849243164,
            -1.6513054370880127,
            -0.9170810580253601,
            -2.2216973304748535,
            -0.7864810824394226,
        ]
        self.pose_aruco2_j2 = [
            1.0019710063934326,
            -0.6748853325843811,
            -1.4592461585998535,
            -1.0018454790115356,
            -2.145453691482544,
            -0.7875596880912781,
        ]
        self.getmid_j = [
            1.1011524200439453,
            -0.46504154801368713,
            -1.46921706199646,
            -1.2059367895126343,
            -0.4831337630748749,
            -0.7895490527153015,
        ]
        # 取试管
        self.getA1 = [
            [6.61171217e-01, 6.09278228e-04, -7.50234797e-01, -5.96091754e-02],
            [-8.07656040e-03, 9.99947502e-01, -6.30568329e-03, -4.70810306e-02],
            [7.50191570e-01, 1.02284529e-02, 6.61141428e-01, -1.37114747e-02],
            [0.00000000e00, 0.00000000e00, 0.00000000e00, 1.00000000e00],
        ]
        self.getA2 = [
            [6.61162620e-01, 5.99423119e-04, -7.50242381e-01, -1.25122500e-01],
            [-8.08026767e-03, 9.99947370e-01, -6.32192941e-03, -4.58370895e-02],
            [7.50199106e-01, 1.02419827e-02, 6.61132667e-01, -1.49801838e-02],
            [0.00000000e00, 0.00000000e00, 0.00000000e00, 1.00000000e00],
        ]
        self.getB1 = [
            [0.66117524, 0.00373551, -0.7502222, -0.06093247],
            [-0.0071647, 0.99997344, -0.00133522, -0.04756895],
            [0.75019729, 0.00625793, 0.66118444, 0.05144453],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.getB2 = [
            [3.07202185e-01, 2.05547756e-03, -9.51642051e-01, -7.66056430e-02],
            [-7.48489242e-03, 9.99971955e-01, -2.56352050e-04, -5.07886123e-02],
            [9.51614835e-01, 7.20169028e-03, 3.07208955e-01, 1.37125290e-01],
            [0.00000000e00, 0.00000000e00, 0.00000000e00, 1.00000000e00],
        ]
        self.getC1 = [
            [0.30536254, 0.00373149, -0.95222886, -0.01293747],
            [-0.00713032, 0.99997325, 0.00163202, -0.04976055],
            [0.95220947, 0.00629134, 0.30538098, 0.20379323],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.getC2 = [
            [0.30534482, 0.00372893, -0.95223455, -0.0780043],
            [-0.00710479, 0.99997342, 0.00163764, -0.04970142],
            [0.95221535, 0.00626538, 0.30536319, 0.20199268],
            [0.0, 0.0, 0.0, 1.0],
        ]
        # 取试管上方
        self.getA1_up = []
        self.getA2_up = []
        self.getB1_up = []
        self.getB2_up = []
        self.getC1_up = []
        self.getC2_up = []
        # 放试管
        # self.putA1 = [2.354503870010376, -0.2611652910709381, -1.840205430984497, -1.038557529449463, 0.7820654511451721, -0.7878983020782471]
        self.putA1 =[2.35495924949646, -0.2615368068218231, -1.8399537801742554, -1.0385814905166626, 0.7823650240898132, -0.7879102826118469]
        # self.putA2 = [2.2887585163116455, -0.34622934460639954, -1.7229278087615967, -1.0698963403701782, 0.7164038419723511, -0.7884136438369751]
        self.putA2 = [2.2890701293945312, -0.3462652862071991, -1.722903847694397, -1.069920301437378, 0.7166795134544373, -0.788425624370575]
        # self.putB1 = [2.2785000801086426, -0.2141392081975937, -1.900474190711975, -1.0248476266860962, 0.7061094045639038, -0.7879582047462463] 
        self.putB1=[2.2785000801086426, -0.2141392081975937, -1.9000426530838013, -1.0245479345321655, 0.7068763971328735, -0.7879342436790466]
        self.putB2 =[2.2154629230499268, -0.30411675572395325, -1.781470775604248, -1.0530943870544434, 0.6430963277816772, -0.7885334491729736]
        self.putC1 = [1.5838104486465454, -0.6967799067497253, -1.1719651222229004, -1.272645354270935, 1.5824079513549805, -0.7881619334220886]
        self.putC2 = [1.5829116106033325, -0.8152439594268799, -0.9641706347465515, -1.3624911308288574, 1.5812574625015259, -0.788425624370575]

        self.pose_aruco2 = []

    def point(self):
        """
        识别二维码
        """
        # 移动至aruco前方
        duco_cobot.movej(midpose, speedj, 20, 0, True)
        print("开始取样品操作")
        robotiq(1, 1, 255, 0, 100, 1)
        motor.motor_position(183900-2000, 10)
        motor.motor_position(177000-2000, 0)
        # 识别aruco
        mark_size(0.120)
        mark_id(580)
        duco_cobot.movej(self.pose_aruco2_j1, speedj, 20, 0, True)
        # duco_cobot.movej(self.pose_aruco2_j2, speedj - 30, 20, 0, True)
        sleep(0.5)
        aruco.mark(580, 150, 6)
        # 计算示教点
        self.pose_aruco2 = duco_cobot.get_tcp_pose()
        self.getA1 = get_t(self.pose_aruco2, self.getA1)
        self.getA2 = get_t(self.pose_aruco2, self.getA2)
        self.getB1 = get_t(self.pose_aruco2, self.getB1)
        self.getB2 = get_t(self.pose_aruco2, self.getB2)
        self.getB2[2]+=0.003
        self.getC1 = get_t(self.pose_aruco2, self.getC1)
        self.getC1[2]+=0.002
        self.getC2 = get_t(self.pose_aruco2, self.getC2)
        self.getC2[2]+=0.003
        # print("self.getA1",self.getA1)
        # print("self.getA2",self.getA2)
        # print("self.getB1",self.getB1)
        # print("self.getB2",self.getB2)
        # print("self.getC1",self.getC1)
        # print("self.getC2",self.getC2)

        self.getA1_up = self.getA1.copy()
        self.getA2_up = self.getA2.copy()
        self.getB1_up = self.getB1.copy()
        self.getB2_up = self.getB2.copy()
        self.getC1_up = self.getC1.copy()
        self.getC2_up = self.getC2.copy()
        self.getA1_up[2] += 0.12
        self.getA2_up[2] += 0.12
        self.getB1_up[2] += 0.12
        self.getB2_up[2] += 0.12
        self.getC1_up[2] += 0.12
        self.getC2_up[2] += 0.12

    def get(self, getpose_up, getpose, putpose_j, num):
        """
        取样品（单个）
        """
        if num:
            duco_cobot.movej(self.getmid_j, speedj, 10, 0, True)
        # 移动至待夹取的试管
        duco_cobot.movel(getpose_up, speedl - 0.3, 0.2, 0, [], "", "", True)
        # input("<<")
        if num == 0:
            robotiq(1, 1, 135, 0, 100, 1)
        # 力控向下
        # curpose = duco_cobot.get_tcp_pose()
        # input("<<")
        duco_cobot.movel(getpose, speedl - 0.3, 0.2, 0, [], "", "", True)
        # input("<<")
        robotiq(1, 1, 250, 0, 50, 70)
        # input("<<")
        curpose = duco_cobot.get_tcp_pose()
        duco_cobot.fc_config(
            [True, True, True, False, False, False],
            [0, -20, 0, 0, 0, 0],
            [2000, 2000, 2000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos(
            [0, 0, 0.10, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft(
            [0, -20, 0, 0, 0, 0], [0, -2, 0, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        sleep(0.5)
        curpose1 = duco_cobot.get_tcp_pose()
        # 判断是否夹起，否则报错
        if curpose1[2] - curpose[1] < 0.07:
            print("取样品受力停止了，取出试管未成功")
            robotiq(1, 1, 150, 0, 150, 10)
            assert False
        # 移动至中间点
        duco_cobot.movej(self.getmid_j, speedj, 10, 0, True)
        #把试管放到试管架上
        
        # # input('<<')

        duco_cobot.movej(putpose_j, speedj, 10, 0, True)
        # input('<<')
        # robotiq(1, 1, 250, 0, 150, 150)
        # 力控将试管放入试管架
        duco_cobot.fc_config(
            [False, True, False, False, False, False],
            [0, 32, 0, 0, 0, 0],
            [1000, 2000, 1000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos(
            [0, 0, -0.141, 0, 0, 0], [0, 0, 0.005, 0, 0, 0], False, 0, 500000
        )
        # duco_cobot.fc_wait_pos([0, 0, -0.14, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 30, 0, 0, 0, 0], [
                              0, 2, 0, 0, 0, 0], False, 0, 8000)
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        sleep(0.5)
        robotiq(1, 1, 135, 0, 100, 50)
        duco_cobot.movej(putpose_j, speedj, 10, 0, True)

    def get_tube(self, test):
        # 测试版
        if test == 0:
            self.get(self.getA1_up, self.getA1, self.putA1, 0)
            duco_cobot.movej(midpose, speedj, 10, 0, True)
        # 正式版
        elif test == 1:
            self.get(self.getA1_up, self.getA1, self.putC1, 0)
            self.get(self.getA2_up, self.getA2, self.putC2, 1)
            self.get(self.getB1_up, self.getB1, self.putB1, 1)
            self.get(self.getB2_up, self.getB2, self.putB2, 1)
            self.get(self.getC1_up, self.getC1, self.putA1, 1)
            self.get(self.getC2_up, self.getC2, self.putA2, 1)
            duco_cobot.movej(midpose, speedj, 10, 0, True)
        else:
            print("输入错误")

    def again(self):
        self.pose_aruco2_j1 = [
            0.9242292642593384,
            -0.5668954849243164,
            -1.6513054370880127,
            -0.9170810580253601,
            -2.2216973304748535,
            -0.7864810824394226,
        ]
        self.pose_aruco2_j2 = [
            1.0019710063934326,
            -0.6748853325843811,
            -1.4592461585998535,
            -1.0018454790115356,
            -2.145453691482544,
            -0.7875596880912781,
        ]
        self.getmid_j = [
            1.1011524200439453,
            -0.46504154801368713,
            -1.46921706199646,
            -1.2059367895126343,
            -0.4831337630748749,
            -0.7895490527153015,
        ]
        # 取试管
        self.getA1 = [
            [6.61171217e-01, 6.09278228e-04, -7.50234797e-01, -5.96091754e-02],
            [-8.07656040e-03, 9.99947502e-01, -6.30568329e-03, -4.70810306e-02],
            [7.50191570e-01, 1.02284529e-02, 6.61141428e-01, -1.37114747e-02],
            [0.00000000e00, 0.00000000e00, 0.00000000e00, 1.00000000e00],
        ]
        self.getA2 = [
            [6.61162620e-01, 5.99423119e-04, -7.50242381e-01, -1.25122500e-01],
            [-8.08026767e-03, 9.99947370e-01, -6.32192941e-03, -4.58370895e-02],
            [7.50199106e-01, 1.02419827e-02, 6.61132667e-01, -1.49801838e-02],
            [0.00000000e00, 0.00000000e00, 0.00000000e00, 1.00000000e00],
        ]
        self.getB1 = [
            [0.66117524, 0.00373551, -0.7502222, -0.06093247],
            [-0.0071647, 0.99997344, -0.00133522, -0.04756895],
            [0.75019729, 0.00625793, 0.66118444, 0.05144453],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.getB2 = [
            [3.07202185e-01, 2.05547756e-03, -9.51642051e-01, -7.66056430e-02],
            [-7.48489242e-03, 9.99971955e-01, -2.56352050e-04, -5.07886123e-02],
            [9.51614835e-01, 7.20169028e-03, 3.07208955e-01, 1.37125290e-01],
            [0.00000000e00, 0.00000000e00, 0.00000000e00, 1.00000000e00],
        ]
        self.getC1 = [
            [0.30536254, 0.00373149, -0.95222886, -0.01293747],
            [-0.00713032, 0.99997325, 0.00163202, -0.04976055],
            [0.95220947, 0.00629134, 0.30538098, 0.20379323],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.getC2 = [
            [0.30534482, 0.00372893, -0.95223455, -0.0780043],
            [-0.00710479, 0.99997342, 0.00163764, -0.04970142],
            [0.95221535, 0.00626538, 0.30536319, 0.20199268],
            [0.0, 0.0, 0.0, 1.0],
        ]
        # 取试管上方
        self.getA1_up = []
        self.getA2_up = []
        self.getB1_up = []
        self.getB2_up = []
        self.getC1_up = []
        self.getC2_up = []
        # 放试管
        # self.putA1 = [2.354503870010376, -0.2611652910709381, -1.840205430984497, -1.038557529449463, 0.7820654511451721, -0.7878983020782471]
        self.putA1 =[2.35495924949646, -0.2615368068218231, -1.8399537801742554, -1.0385814905166626, 0.7823650240898132, -0.7879102826118469]
        # self.putA2 = [2.2887585163116455, -0.34622934460639954, -1.7229278087615967, -1.0698963403701782, 0.7164038419723511, -0.7884136438369751]
        self.putA2 = [2.2890701293945312, -0.3462652862071991, -1.722903847694397, -1.069920301437378, 0.7166795134544373, -0.788425624370575]
        # self.putB1 = [2.2785000801086426, -0.2141392081975937, -1.900474190711975, -1.0248476266860962, 0.7061094045639038, -0.7879582047462463] 
        self.putB1=[2.2785000801086426, -0.2141392081975937, -1.9000426530838013, -1.0245479345321655, 0.7068763971328735, -0.7879342436790466]
        self.putB2 =[2.2154629230499268, -0.30411675572395325, -1.781470775604248, -1.0530943870544434, 0.6430963277816772, -0.7885334491729736]
        self.putC1 = [1.5838104486465454, -0.6967799067497253, -1.1719651222229004, -1.272645354270935, 1.5824079513549805, -0.7881619334220886]
        self.putC2 = [1.5829116106033325, -0.8152439594268799, -0.9641706347465515, -1.3624911308288574, 1.5812574625015259, -0.788425624370575]

        self.pose_aruco2 = []


class centrifuge(object):
    def __init__(self):
        # 旋转矩阵
        self.key_mid = np.array(
            [
                [0.88812738, -0.45794297, -0.03896145, 0.06287588],
                [0.41193665, 0.83075479, -0.37437238, 0.22213659],
                [0.20380861, 0.31644071, 0.92645957, 0.04626311],
                [0, 0, 0, 1],
            ]
        )  # 开盖
        self.key_open = np.array(
            [
                [0.88826375, -0.45770228, -0.0386799, 0.06150703],
                [0.41176459, 0.83076436, -0.37454038, 0.18800396],
                [0.20356187, 0.31676363, 0.92640346, 0.12834574],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.key_start = np.array(
            [
                [0.88828263, -0.45767131, -0.03861263, 0.04077991],
                [0.4118243, 0.83087261, -0.3742345, 0.19037508],
                [0.20335857, 0.31652438, 0.92652987, 0.12419927],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )  # 启动离心机

        self.key_start_mid = np.array(
            [
                [0.88828012, -0.45767738, -0.03859853, 0.04392342],
                [0.41183391, 0.83086667, -0.37423711, 0.22076065],
                [0.20335009, 0.3165312, 0.92652941, 0.04898253],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.axis = np.array(
            [
                [0.00521612, -0.9999716, -0.00543908, -0.09862462],
                [0.99987261, 0.0051334, 0.01511309, -0.17843551],
                [-0.01508474, -0.00551722, 0.999871, 0.13328981],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )  # 转子轴心

        # 取试管点
        self.getA1 = [
            [0.54681881, -0.72902678, 0.41171488, -0.2117069],
            [0.83680231, 0.49197752, -0.2402499, -0.10816282],
            [-0.02740585, 0.47589713, 0.87907385, 0.09669075],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.getA2 = [
            [0.98422844, -0.17686438, 0.00365717, -0.09989824],
            [0.15307185, 0.84109892, -0.51876933, -0.03113121],
            [0.08867577, 0.51114733, 0.85490643, 0.10509282],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.getA3 = [
            [0.43028277, 0.78788146, -0.4405673, 0.02186218],
            [-0.90173569, 0.35267199, -0.24999044, -0.10778585],
            [-0.04158708, 0.50484184, 0.86220951, 0.1029053],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.getB1 = [
            [-5.14426537e-01, -4.53653699e-01, -7.27711248e-01, 1.98808449e-01],
            [8.57032445e-01, -3.01015889e-01, -4.18192328e-01, -8.39373094e-04],
            [-2.93381517e-02, -8.38801381e-01, 5.43646500e-01, 1.85040799e-01],
            [0.00000000e00, 0.00000000e00, 0.00000000e00, 1.00000000e00],
        ]
        self.getB2 = [
            [-9.99939975e-01, 6.07346561e-04, -1.09397501e-02, -9.68663093e-02],
            [9.03484613e-03, -5.19134804e-01, -8.54644620e-01, 1.78197944e-01],
            [-6.19827048e-03, -8.54692159e-01, 5.19098155e-01, 1.93464804e-01],
            [0.00000000e00, 0.00000000e00, 0.00000000e00, 1.00000000e00],
        ]
        self.getB3 = [
            [-0.43805703, 0.48811838, 0.75488177, -0.41103515],
            [-0.89888767, -0.2475069, -0.36158165, -0.02345966],
            [0.0103438, -0.8369473, 0.54718574, 0.18644897],
            [0.0, 0.0, 0.0, 1.0],
        ]
        # 放试管点
        self.putA1 = [
            [0.54682881, -0.72902992, 0.41169603, -0.25349789],
            [0.83679598, 0.49198788, -0.24025072, -0.08377766],
            [-0.0273995, 0.4758816, 0.87908246, 0.00746794],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.putA2 = [
            [0.98422265, -0.1768969, 0.00364037, -0.1002888],
            [0.15308461, 0.84105825, -0.5188315, 0.02665626],
            [0.08871792, 0.511203, 0.85486877, 0.00987585],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.putA3 = [
            [0.43029389, 0.78786976, -0.44057736, 0.07009138],
            [-0.90173077, 0.35268222, -0.24999375, -0.08042695],
            [-0.04157871, 0.50485295, 0.8622034, 0.00853321],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.putB1 = [
            [-0.51442255, -0.45366181, -0.72770901, 0.05361048],
            [0.85703541, -0.30100133, -0.41819674, -0.09071073],
            [-0.02932149, -0.83880222, 0.5436461, 0.10548989],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.putB2 = [
            [-9.99939483e-01, 5.93995552e-04, -1.09853431e-02, -9.79013150e-02],
            [9.08031159e-03, -5.19194416e-01, -8.54607926e-01, 1.18587039e-02],
            [-6.21116208e-03, -8.54655957e-01, 5.19157602e-01, 1.12345144e-01],
            [0.00000000e00, 0.00000000e00, 0.00000000e00, 1.00000000e00],
        ]
        self.putB3 = [
            [-0.43806353, 0.48807144, 0.75490835, -0.25783287],
            [-0.89888309, -0.24760279, -0.36152739, -0.09902313],
            [0.01046622, -0.83694631, 0.54718491, 0.10802555],
            [0.0, 0.0, 0.0, 1.0],
        ]

        # 中间点
        self.put_middleA = []
        self.get_middleA = []
        self.get_middleB = []
        self.put_middleB = np.array(
            [
                [-0.99842942, -0.01541606, -0.05386133, -0.10597664],
                [0.05410575, -0.01590283, -0.99840857, 0.18961635],
                [0.01453498, -0.99975469, 0.01671195, 0.07029574],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        # 关门点
        self.close_door0 = [
            [0.9999243, -0.0049054, 0.01128388, -0.14780709],
            [0.01054562, 0.81413732, -0.58057662, 0.01879157],
            [-0.00633867, 0.58065167, 0.81412742, -0.29472134],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.close_door1 = np.array(
            [
                [0.99992339, -0.00493678, 0.0113512, -0.14286774],
                [0.01061016, 0.8141464, -0.58056271, -0.14728529],
                [-0.00637543, 0.58063867, 0.81413641, -0.29225329],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.close_door3 = np.array(
            [
                [0.99959752, -0.02304766, 0.01654071, -0.14592057],
                [0.022821, 0.99964482, 0.01376344, -0.14525979],
                [-0.01685205, -0.01338042, 0.99976846, -0.13248836],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        # 看aruco点
        self.watch_aruco_pose = np.array(
            [
                [9.99986468e-01, 5.18479587e-03, -4.26286242e-04, -1.04633858e-01],
                [-5.18581144e-03, 9.99983633e-01, -
                    2.41682813e-03, -7.53697580e-02],
                [4.13748504e-04, 2.41900606e-03, 9.99996989e-01, 1.34617099e-01],
                [0.00000000e00, 0.00000000e00, 0.00000000e00, 1.00000000e00],
            ]
        )
        self.pose_aruco2 = []
        self.mark2camera = []

    def point(self):
        """
        识别二维码
        """
        duco_cobot.movej(midpose, speedj, 20, 0, True)
        print("开始离心机操作")
        # 识别二维码
        mark_size(0.150)
        mark_id(582)
        duco_cobot.tcp_move([0, 0, -0.1, 0, 0, 0], 0.3, 0.2, 0, True)
        robotiq(1, 1, 255, 0, 100, 1)
        aruco_way = [
            0.2035699039697647,
            0.17634217441082,
            -1.4309754371643066,
            -0.3029015362262726,
            1.5594215393066406,
            -0.573725163936615,
        ]
        duco_cobot.movej(aruco_way, speedj, 10, 0, True)
        # 定位
        sleep(0.5)
        aruco.mark(582, 100, 5)
        # 计算示教点
        self.pose_aruco2 = duco_cobot.get_tcp_pose()
        # self.pose_aruco2=[-0.5931814312934875, 0.0753781720995903, 0.8268975615501404, -3.123075246810913, -0.0009860026184469461, 1.558491826057434]
        print('self.pose_aruco2',self.pose_aruco2)
        self.key_mid = get_t(self.pose_aruco2, self.key_mid)
        self.key_open = get_t(self.pose_aruco2, self.key_open)
        # 启动按键
        self.key_start = get_t(self.pose_aruco2, self.key_start)
        self.key_start_mid = get_t(self.pose_aruco2, self.key_start_mid)

        self.axis = get_t(self.pose_aruco2, self.axis)
        # 放试管点
        self.putA1 = get_t(self.pose_aruco2, self.putA1)
        self.putA2 = get_t(self.pose_aruco2, self.putA2)
        self.putA3 = get_t(self.pose_aruco2, self.putA3)
        self.putB1 = get_t(self.pose_aruco2, self.putB1)
        self.putB2 = get_t(self.pose_aruco2, self.putB2)
        self.putB3 = get_t(self.pose_aruco2, self.putB3)
        # 取试管点
        self.getA1 = get_t(self.pose_aruco2, self.getA1)
        self.getA2 = get_t(self.pose_aruco2, self.getA2)
        self.getA3 = get_t(self.pose_aruco2, self.getA3)
        self.getB1 = get_t(self.pose_aruco2, self.getB1)
        self.getB2 = get_t(self.pose_aruco2, self.getB2)
        self.getB3 = get_t(self.pose_aruco2, self.getB3)
        # 看aruco点
        self.watch_aruco_pose = get_t(self.pose_aruco2, self.watch_aruco_pose)
        self.watch_aruco_pose[2] += 0.25
        self.mark2camera = [-3.0391876, 0.04668069, 1.59130737]  # 取试管比较的旋转角度
        # 中间点
        self.put_middleA = self.watch_aruco_pose
        self.put_middleB = get_t(self.pose_aruco2, self.put_middleB)
        self.get_middleA = self.put_middleA
        self.get_middleB = self.put_middleB
        # 关门点
        self.close_door0 = get_t(self.pose_aruco2, self.close_door0)
        self.close_door1 = get_t(self.pose_aruco2, self.close_door1)
        # close_door2 = tcp_move  z轴 0.16
        self.close_door3 = get_t(self.pose_aruco2, self.close_door3)
        # print("self.close_door3", self.close_door3)

        # 识别完移动至二维码上方
        self.pose_aruco2[2] += 0.05
        ret = duco_cobot.movel(self.pose_aruco2, speedl,
                               0.2, 0, [], "", "", True)
        self.pose_aruco2[2] -= 0.05

    def open(self):

        key(self.key_mid, self.key_open, -20)
        print("盖子已打开")

    def start(self):

        key(self.key_start_mid, self.key_start, -20)
        print("离心机已启动")

    def put(self, test):

        # movel
        duco_cobot.movel(self.watch_aruco_pose, speedl -
                         0.1, 0.2, 0, [], "", "", True)
        self.watch_aruco_pose[2] -= 0.25
        duco_cobot.movel(self.watch_aruco_pose, speedl -
                         0.1, 0.2, 0, [], "", "", True)
        # 求旋转角度
        mark_size(0.0278)
        mark_id(582)
        mark2camera = aruco.test_mark(582)
        # 计算旋转角度
        diff = mark2camera[2] - self.mark2camera[2]
        self.watch_aruco_pose[2] += 0.25
        duco_cobot.movel(self.watch_aruco_pose, speedl -
                         0.1, 0.2, 0, [], "", "", True)
        # A半圆
        diff = diff * 180 / math.pi
        print("二维码旋转角度：", diff)
        # 求Angle
        theta = abs(diff) % 60
        if diff >= 0:
            if 30 > theta > 0:
                theta = -theta
            elif 60 > theta >= 30:
                theta = 60 - theta
            else:
                assert False
        else:
            if 30 >= theta > 0:
                theta = theta
            elif 60 > theta > 30:
                theta = theta - 60
            else:
                assert False
        print("机械臂旋转角度：", theta)
        theta = theta * math.pi / 180
        motor.motor_position(183000-1000, 10)
        motor.motor_position(179000-1000, 0)
        if test == 0:
            # 测试版
            tube.get_tubeA(tube.tube_putA1, tube.tube_getA1)
            self.put_tubeA(self.putA1, self.put_middleA, theta)
        elif test == 1:
            # 正式版
            # A半圆移动至平台试管架取试管再移动至离心机放试管
            tube.get_tubeA(tube.tube_putA1, tube.tube_getA1)
            self.put_tubeA(self.putA1, self.put_middleA, theta)
            tube.get_tubeA(tube.tube_putA2, tube.tube_getA2)
            self.put_tubeA(self.putA2, self.put_middleA, theta)
            tube.get_tubeA(tube.tube_putA3, tube.tube_getA3)
            self.put_tubeA(self.putA3, self.put_middleA, theta)
            # B半圆移动至平台试管架取试管再移动至离心机放试管
            tube.get_tubeB(tube.tube_putB1, tube.tube_getB1)
            self.put_tubeB(self.putB1, self.put_middleB, theta)
            tube.get_tubeB(tube.tube_putB2, tube.tube_getB2)
            self.put_tubeB(self.putB2, self.put_middleB, theta)
            tube.get_tubeB(tube.tube_putB3, tube.tube_getB3)
            self.put_tubeB(self.putB3, self.put_middleB, theta)
        else:
            print("输入错误")

    def put_tubeA(self, edge, middle, angle):
        """
        离心机放试管
        """
        # global csvflag
        dist = 0.13  # 位置判断
        duco_cobot.movel(middle, speedl + 0.2, 0.2, 0, [], "", "", True)
        # 旋转至第一个夹取点
        target = circle(self.axis, edge, angle)
        duco_cobot.movel(target, speedl - 0.1, 0.2, 0, [], "", "", True)
        # Forcemode
        ponit_pose = duco_cobot.get_tcp_pose()
        # cvs_F = cvs_get()
        # cvs_F.start()
        duco_cobot.fc_config(
            [False, False, True, False, False, False],
            [0, 0, -25, 0, 0, 0],
            [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos(
            [0, 0, -dist, 0, 0, 0], [0, 0, 0.02, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft(
            [0, 0, -15, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        # csvflag = 0
        sleep(1)
        # csvflag = 1
        if abs(duco_cobot.get_tcp_pose()[2] - ponit_pose[2]) >= dist - 0.05:
            robotiq(1, 1, 150, 0, 100, 1)
            print("试管放入成功")
            ret = duco_cobot.movel(
                ponit_pose, speedl - 0.1, 0.2, 0, [], "", "", True)
            print("返回放置点", ret)
            ret = duco_cobot.movel(middle, speedl - 0.1,
                                   0.2, 0, [], "", "", True)
            print("移动至中间点", ret)
        else:
            print("放试管error")
            robotiq(1, 1, 120, 0, 100, 1)
            duco_cobot.movel(middle, speedl + 0.2, 0.2, 0, [], "", "", True)
            assert False

    def put_tubeB(self, edge, middle, angle):
        dist = 0.12  # 位置判断
        duco_cobot.movel(middle, speedl, 0.2, 0, [], "", "", True)
        # 旋转至第一个夹取点
        target = circle(self.axis, edge, angle)
        duco_cobot.movel(target, 0.2, 0.1, 0, [], "", "", True)
        # Forcemode
        ponit_pose = duco_cobot.get_tcp_pose()
        duco_cobot.fc_config(
            [False, True, False, False, False, False],
            [0, 20, 0, 0, 0, 0],
            [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos(
            [0, 0, -dist, 0, 0, 0], [0, 0, 0.02, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 10, 0, 0, 0, 0], [
                              0, 2, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        sleep(1)

        if abs(duco_cobot.get_flange_pose()[2] - ponit_pose[2]) >= dist - 0.05:
            robotiq(1, 1, 175, 0, 100, 1)
            print("试管放入成功")
            ret = duco_cobot.movel(
                ponit_pose, speedl - 0.1, 0.2, 0, [], "", "", True)
            print("返回放置点", ret)
            ret = duco_cobot.movel(middle, 0.2, 0.2, 0, [], "", "", True)
            print("移动至中间点", ret)
        else:
            print("放试管error")
            robotiq(1, 1, 120, 0, 100, 1)
            duco_cobot.movel(middle, speedl, 0.2, 0, [], "", "", True)
            assert False

    def get(self, test):
        # movel
        duco_cobot.movel(self.watch_aruco_pose, speedl -
                         0.1, 0.2, 0, [], "", "", True)
        self.watch_aruco_pose[2] -= 0.25
        duco_cobot.movel(self.watch_aruco_pose, speedl -
                         0.1, 0.2, 0, [], "", "", True)
        # 求旋转角度
        mark_size(0.0278)
        mark_id(582)
        sleep(0.5)
        mark2camera = aruco.test_mark(582)
        # 计算旋转角度
        diff = mark2camera[2] - self.mark2camera[2]
        self.watch_aruco_pose[2] += 0.25
        duco_cobot.movel(self.watch_aruco_pose, speedl -
                         0.1, 0.2, 0, [], "", "", True)
        # A半圆
        diff = diff * 180 / math.pi
        print("二维码旋转角度：", diff)
        # 求Angle
        theta = abs(diff) % 60
        if diff >= 0:
            if 30 > theta > 0:
                theta = -theta
            elif 60 > theta >= 30:
                theta = 60 - theta
            else:
                assert False
        else:
            if 30 >= theta > 0:
                theta = theta
            elif 60 > theta > 30:
                theta = theta - 60
            else:
                assert False
        print("试管槽旋转角度：", theta)
        theta = theta * math.pi / 180
        if test == 0:
            # 测试版
            self.get_tubeA(self.getA1, self.get_middleA, theta)
            tube.put_tubeA(tube.tube_putA1)
        elif test == 1:
            # 正式版
            # 取A半圆
            self.get_tubeA(self.getA1, self.get_middleA, theta)
            tube.put_tubeA(tube.tube_putA1)
            self.get_tubeA(self.getA2, self.get_middleA, theta)
            tube.put_tubeA(tube.tube_putA2)
            self.get_tubeA(self.getA3, self.get_middleA, theta)
            tube.put_tubeA(
                tube.tube_putA3,
            )
            print("A半圆取完,取B半圆")
            # 取B半圆
            duco_cobot.movej(tube.tube_middleB_j1, speedj, 10, 0, True)
            duco_cobot.movej(tube.tube_middleB_j2, speedj, 10, 0, True)
            self.get_tubeB(self.getB1, self.get_middleB, theta, -0.002,  0.1)
            tube.put_tubeB(tube.tube_putB1)
            self.get_tubeB(self.getB2, self.get_middleB, theta, 0,  0.103)
            tube.put_tubeB(tube.tube_putB2)
            self.get_tubeB(self.getB3, self.get_middleB, theta, 0.003,  0.1)
            tube.put_tubeB(tube.tube_putB3)
        else:
            print("输入错误")

    def get_tubeA(self, edge, middle, angle):
        dist = 0.14
        duco_cobot.movel(middle, speedl, 0.2, 0, [], "", "", True)
        # 旋转至第一个夹取点
        target = circle(self.axis, edge, angle)
        duco_cobot.movel(target, speedl - 0.2, 0.2, 0, [], "", "", True)
        test_pose = duco_cobot.get_tcp_pose()
        robotiq(1, 1, 190, 0, 100, 1)
        # Forcemode
        duco_cobot.fc_config(
            [False, False, True, False, False, False],
            [0, 0, -20, 0, 0, 0],
            [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        # duco_cobot.fc_wait_pos([0, 0,-dist_q, 0, 0, 0], [0, 0, 0.001, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_pos([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft(
            [0, 0, -10, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000
        )
        # duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_wait_logic([0, 0, 1])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        sleep(0.5)
        # 上升一点点，夹住试管
        ret = duco_cobot.tcp_move([0, 0, -0.0035, 0, 0, 0], 0.05, 0.05, 0, True)
        print("move", ret)
        test_pose1 = duco_cobot.get_tcp_pose()
        offset = test_pose[2] - test_pose1[2]
        print("放试管的力控的位置判断距离", offset)
        # 夹爪闭合
        robotiq(1, 1, 250, 0, 100, 150)
        curpose = duco_cobot.get_tcp_pose()
        duco_cobot.fc_config(
            [False, False, True, False, False, False],
            [0, 0, 30, 0, 0, 0],
            [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos(
            [0, 0, dist, 0, 0, 0], [0, 0, 0.02, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, 25, 0, 0, 0], [
                              0, 0, 2, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        sleep(0.5)
        curpose1 = duco_cobot.get_tcp_pose()
        if curpose1[2] - curpose[1] < 0.105:
            print("受力停止了，取出试管未成功")
            robotiq(1, 1, 120, 0, 100, 150)
            duco_cobot.movel(middle, speedl, 0.2, 0, [], "", "", True)
            assert False
        duco_cobot.movel(middle, speedl - 0.1, 0.2, 0, [], "", "", True)

    def get_tubeB(self, edge, middle, angle, offset, distance):
        dist = 0.14
        # 从A点旋转到位置0
        target = circle(self.axis, edge, angle)
        duco_cobot.movel(self.get_middleB, speedl, 0.2, 0, [], "", "", True)
        duco_cobot.movel(target, 0.2, 0.1, 0, [], "", "", True)
        robotiq(1, 1, 170, 0, 10, 1)
        duco_cobot.tcp_move([0, offset, distance, 0, 0, 0], 0.05, 0.05, 0, True)
        # 夹爪闭合
        robotiq(1, 1, 250, 0, 200, 150)
        curpose = duco_cobot.get_tcp_pose()
        duco_cobot.fc_config(
            [False, True, False, False, False, False],
            [0, -30, 0, 0, 0, 0],
            [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos(
            [0, 0, dist, 0, 0, 0], [0, 0, 0.02, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft(
            [0, -25, 0, 0, 0, 0], [0, 2, 0, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        sleep(1)
        curpose1 = duco_cobot.get_tcp_pose()
        if curpose1[2] - curpose[1] < 0.105:
            print("受力停止了，取出试管未成功")
            robotiq(1, 1, 120, 0, 200, 150)
            duco_cobot.movel(middle, speedl, 0.2, 0, [], "", "", True)
            assert False
        duco_cobot.movel(middle, 0.2, 0.2, 0, [], "", "", True)

    def close_door(self):
        robotiq(1, 1, 255, 0, 100, 50)
        duco_cobot.movel(self.close_door0, speedl -
                         0.1, 0.2, 0, [], "", "", True)
        duco_cobot.movel(self.close_door1, speedl -
                         0.2, 0.2, 0, [], "", "", True)
        duco_cobot.tcp_move([0, 0, 0.08, 0, 0, 0], 0.3, 0.2, 0, True)
        duco_cobot.movel(self.close_door3, 0.1, 0.1, 0, [], "", "", True)
        # 向下按压
        duco_cobot.fc_config(
            [False, False, True, False, False, False],
            [0, 0, -40, 0, 0, 0],
            [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft(
            [0, 0, -20, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_logic([0, 0, 1])
        duco_cobot.fc_move()
        print("move stop")
        sleep(1)
        # 关上盖子
        duco_cobot.fc_config(
            [False, False, True, False, False, False],
            [0, 0, -210, 0, 0, 0],
            [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_wait_pos([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft(
            [0, 0, -200, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_logic([0, 0, 1])
        duco_cobot.fc_move()
        print("关上盖子")
        sleep(1)
        duco_cobot.fc_config(
            [False, False, True, False, False, False],
            [0, 0, 20, 0, 0, 0],
            [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_wait_pos(
            [0, 0, 0.02, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, 10, 0, 0, 0], [
                              0, 0, 2, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 0])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        duco_cobot.tcp_move([0, 0, -0.13, 0, 0, 0], 0.3, 0.2, 0, True)

    def again(self):
           # 旋转矩阵
        self.key_mid = np.array(
            [
                [0.88812738, -0.45794297, -0.03896145, 0.06287588],
                [0.41193665, 0.83075479, -0.37437238, 0.22213659],
                [0.20380861, 0.31644071, 0.92645957, 0.04626311],
                [0, 0, 0, 1],
            ]
        )  # 开盖
        self.key_open = np.array(
            [
                [0.88826375, -0.45770228, -0.0386799, 0.06150703],
                [0.41176459, 0.83076436, -0.37454038, 0.18800396],
                [0.20356187, 0.31676363, 0.92640346, 0.12834574],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.key_start = np.array(
            [
                [0.88828263, -0.45767131, -0.03861263, 0.04077991],
                [0.4118243, 0.83087261, -0.3742345, 0.19037508],
                [0.20335857, 0.31652438, 0.92652987, 0.12419927],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )  # 启动离心机

        self.key_start_mid = np.array(
            [
                [0.88828012, -0.45767738, -0.03859853, 0.04392342],
                [0.41183391, 0.83086667, -0.37423711, 0.22076065],
                [0.20335009, 0.3165312, 0.92652941, 0.04898253],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.axis = np.array(
            [
                [0.00521612, -0.9999716, -0.00543908, -0.09862462],
                [0.99987261, 0.0051334, 0.01511309, -0.17843551],
                [-0.01508474, -0.00551722, 0.999871, 0.13328981],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )  # 转子轴心

        # 取试管点
        self.getA1 = [
            [0.54681881, -0.72902678, 0.41171488, -0.2117069],
            [0.83680231, 0.49197752, -0.2402499, -0.10816282],
            [-0.02740585, 0.47589713, 0.87907385, 0.09669075],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.getA2 = [
            [0.98422844, -0.17686438, 0.00365717, -0.09989824],
            [0.15307185, 0.84109892, -0.51876933, -0.03113121],
            [0.08867577, 0.51114733, 0.85490643, 0.10509282],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.getA3 = [
            [0.43028277, 0.78788146, -0.4405673, 0.02186218],
            [-0.90173569, 0.35267199, -0.24999044, -0.10778585],
            [-0.04158708, 0.50484184, 0.86220951, 0.1029053],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.getB1 = [
            [-5.14426537e-01, -4.53653699e-01, -7.27711248e-01, 1.98808449e-01],
            [8.57032445e-01, -3.01015889e-01, -4.18192328e-01, -8.39373094e-04],
            [-2.93381517e-02, -8.38801381e-01, 5.43646500e-01, 1.85040799e-01],
            [0.00000000e00, 0.00000000e00, 0.00000000e00, 1.00000000e00],
        ]
        self.getB2 = [
            [-9.99939975e-01, 6.07346561e-04, -1.09397501e-02, -9.68663093e-02],
            [9.03484613e-03, -5.19134804e-01, -8.54644620e-01, 1.78197944e-01],
            [-6.19827048e-03, -8.54692159e-01, 5.19098155e-01, 1.93464804e-01],
            [0.00000000e00, 0.00000000e00, 0.00000000e00, 1.00000000e00],
        ]
        self.getB3 = [
            [-0.43805703, 0.48811838, 0.75488177, -0.41103515],
            [-0.89888767, -0.2475069, -0.36158165, -0.02345966],
            [0.0103438, -0.8369473, 0.54718574, 0.18644897],
            [0.0, 0.0, 0.0, 1.0],
        ]
        # 放试管点
        self.putA1 = [
            [0.54682881, -0.72902992, 0.41169603, -0.25349789],
            [0.83679598, 0.49198788, -0.24025072, -0.08377766],
            [-0.0273995, 0.4758816, 0.87908246, 0.00746794],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.putA2 = [
            [0.98422265, -0.1768969, 0.00364037, -0.1002888],
            [0.15308461, 0.84105825, -0.5188315, 0.02665626],
            [0.08871792, 0.511203, 0.85486877, 0.00987585],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.putA3 = [
            [0.43029389, 0.78786976, -0.44057736, 0.07009138],
            [-0.90173077, 0.35268222, -0.24999375, -0.08042695],
            [-0.04157871, 0.50485295, 0.8622034, 0.00853321],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.putB1 = [
            [-0.51442255, -0.45366181, -0.72770901, 0.05361048],
            [0.85703541, -0.30100133, -0.41819674, -0.09071073],
            [-0.02932149, -0.83880222, 0.5436461, 0.10548989],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.putB2 = [
            [-9.99939483e-01, 5.93995552e-04, -1.09853431e-02, -9.79013150e-02],
            [9.08031159e-03, -5.19194416e-01, -8.54607926e-01, 1.18587039e-02],
            [-6.21116208e-03, -8.54655957e-01, 5.19157602e-01, 1.12345144e-01],
            [0.00000000e00, 0.00000000e00, 0.00000000e00, 1.00000000e00],
        ]
        self.putB3 = [
            [-0.43806353, 0.48807144, 0.75490835, -0.25783287],
            [-0.89888309, -0.24760279, -0.36152739, -0.09902313],
            [0.01046622, -0.83694631, 0.54718491, 0.10802555],
            [0.0, 0.0, 0.0, 1.0],
        ]

        # 中间点
        self.put_middleA = []
        self.get_middleA = []
        self.get_middleB = []
        self.put_middleB = np.array(
            [
                [-0.99842942, -0.01541606, -0.05386133, -0.10597664],
                [0.05410575, -0.01590283, -0.99840857, 0.18961635],
                [0.01453498, -0.99975469, 0.01671195, 0.07029574],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        # 关门点
        self.close_door0 = [
            [0.9999243, -0.0049054, 0.01128388, -0.14780709],
            [0.01054562, 0.81413732, -0.58057662, 0.01879157],
            [-0.00633867, 0.58065167, 0.81412742, -0.29472134],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.close_door1 = np.array(
            [
                [0.99992339, -0.00493678, 0.0113512, -0.14286774],
                [0.01061016, 0.8141464, -0.58056271, -0.14728529],
                [-0.00637543, 0.58063867, 0.81413641, -0.29225329],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.close_door3 = np.array(
            [
                [0.99959752, -0.02304766, 0.01654071, -0.14592057],
                [0.022821, 0.99964482, 0.01376344, -0.14525979],
                [-0.01685205, -0.01338042, 0.99976846, -0.13248836],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        # 看aruco点
        self.watch_aruco_pose = np.array(
            [
                [9.99986468e-01, 5.18479587e-03, -4.26286242e-04, -1.04633858e-01],
                [-5.18581144e-03, 9.99983633e-01, -
                    2.41682813e-03, -7.53697580e-02],
                [4.13748504e-04, 2.41900606e-03, 9.99996989e-01, 1.34617099e-01],
                [0.00000000e00, 0.00000000e00, 0.00000000e00, 1.00000000e00],
            ]
        )
        self.pose_aruco2 = []
        self.mark2camera = []


class seperation(object):
    def __init__(self, s_or_a):
        self.s_or_a = s_or_a
        # 取放试管中间点
        self.middle = [
            2.108090877532959,
            -0.007148286327719688,
            -1.6144897937774658,
            -1.515860676765442,
            0.548012912273407,
            2.3568851947784424,
        ]
        # 试管架取试管
        self.getA1 = [0.2223978042602539, -0.5753006339073181-0.0002, 0.19308991730213165, -1.5707509517669678, -7.695273961871862e-05, 1.5708129405975342]
        self.getA2 = [-0.16078586876392365, -0.33843520283699036, 0.19466562569141388, -1.5708192586898804, 1.116746716434136e-05, -2.842353343963623]
        self.getA3 = [-0.16078586876392365-0.065, -0.33843520283699036, 0.19466562569141388, -1.5708192586898804, 1.116746716434136e-05, -2.842353343963623]
        self.getB1 =[0.2223978042602539, -0.5753006339073181-0.065+0.0005, 0.19308991730213165, -1.5707509517669678, -7.695273961871862e-05, 1.5708129405975342]
        self.getB2 = [-0.0014623674796894193, -0.8673410415649414, 0.19342006742954254, -1.5708242654800415, -1.2584327123477124e-05, 0.36597388982772827]
        self.getB3 = [-0.0667271688580513, -0.868273913860321, 0.193430095911026, -1.5708613395690918, -2.3153434085543267e-05, 0.36598673462867737]
        # 试管架放试管
        self.putA1_j = [2.35268235206604, -0.27618154883384705, -1.8510751724243164, -1.0120364427566528, 0.7803636789321899, 2.353670358657837]
        self.putA2_j =[1.4620627164840698, 0.4220433831214905, -2.4890553951263428, -1.0741387605667114, -1.9793157577514648, 2.3563668727874756]
        self.putA3_j =[1.2312105894088745, 0.30300405621528625, -2.422482967376709, -1.0219594240188599, -2.211150646209717, 2.3566784858703613]
        self.putB1_j =[2.2882909774780273, -0.35893261432647705, -1.7351757287979126, -1.0440583229064941, 0.7160203456878662, 2.353119134902954]
        self.putB2_j =[1.782317042350769, -0.7571444511413574, -1.0968959331512451, -1.2871463298797607, 1.4147486686706543, 2.3532748222351074]
        self.putB3_j = [1.7157326936721802, -0.7562456130981445, -1.0980823040008545, -1.2869545221328735, 1.348739504814148, 2.3532748222351074]
        # 看码中间点
        self.watch_aruco = [
            0.7470664978027344,
            -0.0761055201292038,
            -1.9241981506347656,
            0.4429369568824768,
            1.5510445833206177,
            0.013921312056481838,
        ]
        # 取放盖
        self.gai_mid = [
            0.9175900220870972,
            -0.1751791089773178,
            -1.7821491956710815,
            -1.1798828840255737,
            -0.6226181387901306,
            -0.7909152507781982,
        ]

        self.gai_putpose = []
        # 倾倒路点
        self.throw_way = [
            0.9639929533004761,
            -0.26671460270881653,
            -1.3903968334197998,
            -1.5031213760375977,
            -0.6077816486358643,
            2.361870765686035,
        ]
        self.throw = [
            [0.99867305, -0.0020833, -0.05145682, 0.04075777],
            [-0.05140077, 0.02135386, -0.99844979, -0.00116625],
            [0.00317887, 0.99976981, 0.02121844, 0.10664818],
            [0.0, 0.0, 0.0, 1.0],
        ]

    def point(self):
        duco_cobot.movej(midpose, speedj, 20, 0, True)
        print("开始固液分离操作")
        mark_size(0.085)
        # duco_cobot.tcp_move([0, 0, -0.1, 0, 0, 0], 0.3, 0.2, 0, True)
        robotiq(1, 1, 255, 0, 100, 1)
        duco_cobot.movej(self.watch_aruco, speedj, 10, 0, True)
        sleep(0.5)
        # 定位
        if self.s_or_a == 0:
            self.gai_getpose = [[-0.99975634, 0.00204597, -0.02197902, -0.00831166],
                                [0.02193014, -0.02144909, -0.99952939, -0.0048424],
                                [-0.00251644, -0.99976785, 0.021399, 0.17708649],
                                [0.0, 0.0, 0.0, 1.0]]
            mark_id(588)
            aruco.mark(588, 120, 6)
        else:
            self.gai_getpose =[[-0.99882067, -0.00738478, -0.04798672 ,-0.00140096-0.004],
                                [ 0.04813584, -0.02154962, -0.99860831 ,-0.00329974],
                                [ 0.00634041, -0.99974051,  0.02187968,  0.18167692],
                                [ 0.   ,       0.      ,    0.  ,        1.        ]]
            mark_id(589)
            aruco.mark(589, 120, 6)
        # 计算示教点
        self.pose_aruco2 = duco_cobot.get_tcp_pose()
        self.gai_getpose = get_t(self.pose_aruco2, self.gai_getpose)
        self.throw = get_t(self.pose_aruco2, self.throw)
        self.gai_getpose_up = self.gai_getpose.copy()
        self.gai_getpose_up[2] += 0.15
        # duco_cobot.tcp_move([0, 0, -0.3, 0, 0, 0], 0.3, 0.2, 0, True)
        duco_cobot.movej(self.watch_aruco, speedj, 10, 0, True)

    def gaiGet(self):
        duco_cobot.movej(self.gai_mid, speedj, 10, 0, True)
        robotiq(1, 1, 0, 0, 100, 1)
        duco_cobot.movel(self.gai_getpose_up, speedl -
                         0.1, 0.2, 0, [], "", "", True)
        duco_cobot.movel(self.gai_getpose, speedl -
                         0.1, 0.2, 0, [], "", "", True)
        robotiq(1, 1, 255, 0, 100, 5)
        duco_cobot.movel(self.gai_getpose_up, speedl -
                         0.1, 0.2, 0, [], "", "", True)
        curpose = duco_cobot.get_tcp_pose()
        curpose[0] -= 0.25
        duco_cobot.movel(curpose, speedl - 0.1, 0.2, 0, [], "", "", True)

        # 力控放盖子
        duco_cobot.fc_config(
            [False, True, False, False, False, False],
            [0, 15, 0, 0, 0, 0],
            [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos(
            [0, 0, 0.15, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 10, 0, 0, 0, 0], [
                              0, 2, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([0, 0, 1])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        self.gai_putpose = duco_cobot.get_tcp_pose()
        robotiq(1, 1, 0, 0, 100, 1)
        self.gai_putpose[2] += 0.10
        duco_cobot.movel(self.gai_putpose, speedl -
                         0.1, 0.2, 0, [], "", "", True)
        duco_cobot.movej(self.gai_mid, speedj, 10, 0, True)

    def gaiPut(self):
        duco_cobot.movej(self.gai_mid, speedj, 10, 0, True)
        robotiq(1, 1, 0, 0, 100, 1)
        duco_cobot.movel(self.gai_putpose, speedl -
                         0.1, 0.2, 0, [], "", "", True)
        self.gai_putpose[2] -= 0.1

        duco_cobot.movel(self.gai_putpose, speedl -
                         0.1, 0.2, 0, [], "", "", True)

        robotiq(1, 1, 255, 0, 100, 5)
        self.gai_putpose[2] += 0.1

        duco_cobot.movel(self.gai_putpose, speedl -
                         0.1, 0.2, 0, [], "", "", True)
        duco_cobot.movel(self.gai_getpose_up, speedl -
                         0.1, 0.2, 0, [], "", "", True)
        self.gai_getpose[2] += 0.09

        # 力控放盖子
        duco_cobot.fc_config(
            [False, True, False, False, False, False],
            [0, 15, 0, 0, 0, 0],
            [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos(
            [0, 0, 0.15, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 10, 0, 0, 0, 0], [
                              0, 2, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([0, 0, 1])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()

        robotiq(1, 1, 0, 0, 100, 1)
        duco_cobot.movel(self.gai_getpose_up, speedl -
                         0.1, 0.2, 0, [], "", "", True)
        duco_cobot.movej(self.gai_mid, speedj, 10, 0, True)
        robotiq(1, 1, 255, 0, 100, 1)

    def get(self, putpose_j, getpose, num):
        duco_cobot.movej(self.middle, speedj, 10, 0, True)
        if num == 0:
            robotiq(1, 1, 130, 0, 100, 75)
        duco_cobot.movej(putpose_j, speedj, 10, 0, True)
        motor.motor_position(183000-1000, 5)
        motor.motor_position(177000-1000, 0)
        duco_cobot.movel(getpose, speedl - 0.3, 0.2, 0, [], "", "", True)
        # 力控夹起试管
        robotiq(1, 1, 250, 0, 100, 75)
        curpose = duco_cobot.get_tcp_pose()
        duco_cobot.fc_config(
            [False, True, False, False, False, False],
            [0, 35, 0, 0, 0, 0],
            [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos(
            [0, 0, 0.12, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 25, 0, 0, 0, 0], [
                              0, 2, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        print("forcemove stop")
        sleep(0.5)
        curpose1 = duco_cobot.get_tcp_pose()
        if curpose1[2] - curpose[2] < 0.08:
            print("取试管受力停止了，取出试管未成功")
            robotiq(1, 1, 120, 0, 100, 75)
            duco_cobot.movej(self.middle, speedj, 10, 0, True)
            assert False
        duco_cobot.movej(self.middle, speedj, 10, 0, True)

    def put(self, putpose_j):
        duco_cobot.movej(self.middle, speedj, 10, 0, True)
        duco_cobot.movej(putpose_j, speedj, 10, 0, True)
        motor.motor_position(182800-1000, 5)
        motor.motor_position(177000-1000, 0)
        duco_cobot.fc_config(
            [False, True, False, False, False, False],
            [0, -30, 0, 0, 0, 0],
            [1000, 2000, 1000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos(
            [0, 0, -0.135, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft(
            [0, -25, 0, 0, 0, 0], [0, 2, 0, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        print("forcemove stop")
        sleep(0.5)
        robotiq(1, 1, 130, 0, 100, 75)
        curpose = duco_cobot.get_tcp_pose()
        curpose[2] += 0.05
        duco_cobot.movel(curpose, speedl - 0.1, 0.2, 0, [], "", "", True)
        duco_cobot.movej(putpose_j, 40, 10, 0, True)
        duco_cobot.movej(self.middle, speedj, 10, 0, True)

    def throw_single(self, angle):
        duco_cobot.movej(self.throw_way, speedj, 10, 0, True)
        duco_cobot.movel(self.throw, speedl - 0.1, 0.2, 0, [], "", "", True)
        throw_joint = duco_cobot.get_actual_joints_position()
        throw_joint[5] -= angle * math.pi / 180
        duco_cobot.movej(throw_joint, 20, 10, 0, True)
        sleep(3)
        throw_joint[5] += angle * math.pi / 180
        duco_cobot.movej(throw_joint, 30, 10, 0, True)
        duco_cobot.movej(self.throw_way, speedj, 10, 0, True)

    def throw_tube(self, angle, test):
        self.gaiGet()
        if test == 0:
            print("2")
            self.get(self.putA1_j, self.getA1, 0)
            self.throw_single(angle)
            self.put(self.putA1_j)
        elif test == 1:
            self.get(self.putA1_j, self.getA1, 0)
            self.throw_single(angle)
            self.put(self.putA1_j)
            self.get(self.putB1_j, self.getB1, 1)
            self.throw_single(angle)
            self.put(self.putB1_j)
            self.get(self.putA2_j, self.getA2, 1)
            self.throw_single(angle)
            self.put(self.putA2_j)
            self.get(self.putA3_j, self.getA3, 1)
            self.throw_single(angle)
            self.put(self.putA3_j)
            self.get(self.putB2_j, self.getB2, 1)
            self.throw_single(angle)
            self.put(self.putB2_j)
            self.get(self.putB3_j, self.getB3, 1)
            self.throw_single(angle)
            self.put(self.putB3_j)
        else:
            print("输入错误")
        self.gaiPut()
        duco_cobot.movej(self.middle, speedj, 10, 0, True)
        robotiq(1, 1, 250, 0, 100, 75)
        duco_cobot.movej(midpose, speedj, 10, 0, True)

    def again(self):
      # 取放试管中间点
        self.middle = [
            2.108090877532959,
            -0.007148286327719688,
            -1.6144897937774658,
            -1.515860676765442,
            0.548012912273407,
            2.3568851947784424,
        ]
        # 试管架取试管
        self.getA1 = [0.2223978042602539, -0.5753006339073181-0.0002, 0.19308991730213165, -1.5707509517669678, -7.695273961871862e-05, 1.5708129405975342]
        self.getA2 = [-0.16078586876392365, -0.33843520283699036, 0.19466562569141388, -1.5708192586898804, 1.116746716434136e-05, -2.842353343963623]
        self.getA3 = [-0.16078586876392365-0.065, -0.33843520283699036, 0.19466562569141388, -1.5708192586898804, 1.116746716434136e-05, -2.842353343963623]
        self.getB1 =[0.2223978042602539, -0.5753006339073181-0.065+0.0005, 0.19308991730213165, -1.5707509517669678, -7.695273961871862e-05, 1.5708129405975342]
        self.getB2 = [-0.0014134383527562022, -0.8690823316574097, 0.19343161582946777, -1.5708580017089844, -1.3515226555682602e-06, 0.365963876247406]
        self.getB3 = [-0.0667271688580513, -0.868273913860321, 0.193430095911026, -1.5708613395690918, -2.3153434085543267e-05, 0.36598673462867737]
        # 试管架放试管
        self.putA1_j = [2.35268235206604, -0.27618154883384705, -1.8510751724243164, -1.0120364427566528, 0.7803636789321899, 2.353670358657837]
        self.putA2_j =[1.4620627164840698, 0.4220433831214905, -2.4890553951263428, -1.0741387605667114, -1.9793157577514648, 2.3563668727874756]
        self.putA3_j =[1.2312105894088745, 0.30300405621528625, -2.422482967376709, -1.0219594240188599, -2.211150646209717, 2.3566784858703613]
        self.putB1_j =[2.2882909774780273, -0.35893261432647705, -1.7351757287979126, -1.0440583229064941, 0.7160203456878662, 2.353119134902954]
        self.putB2_j =[1.782317042350769, -0.7571444511413574, -1.0968959331512451, -1.2871463298797607, 1.4147486686706543, 2.3532748222351074]
        self.putB3_j = [1.7157326936721802, -0.7562456130981445, -1.0980823040008545, -1.2869545221328735, 1.348739504814148, 2.3532748222351074]
        # 看码中间点
        self.watch_aruco = [
            0.7470664978027344,
            -0.0761055201292038,
            -1.9241981506347656,
            0.4429369568824768,
            1.5510445833206177,
            0.013921312056481838,
        ]
        # 取放盖
        self.gai_mid = [
            0.9175900220870972,
            -0.1751791089773178,
            -1.7821491956710815,
            -1.1798828840255737,
            -0.6226181387901306,
            -0.7909152507781982,
        ]

        self.gai_putpose = []
        # 倾倒路点
        self.throw_way = [
            0.9639929533004761,
            -0.26671460270881653,
            -1.3903968334197998,
            -1.5031213760375977,
            -0.6077816486358643,
            2.361870765686035,
        ]
        self.throw = [
            [0.99867305, -0.0020833, -0.05145682, 0.04075777],
            [-0.05140077, 0.02135386, -0.99844979, -0.00116625],
            [0.00317887, 0.99976981, 0.02121844, 0.10664818],
            [0.0, 0.0, 0.0, 1.0],
        ]


class drying(object):
    def __init__(self):
        # 参数
        self.handle_offset = 0.1+0.005  # 路点到门把手距离
        # 过渡矩阵
        self.openpower = np.array(
            [
                [0.9984479, 0.00925176, 0.05491991, -0.64168806],
                [-0.00903779, 0.99995058, -0.0041431, 0.33587667],
                [-0.05495553, 0.00364032, 0.99848217, 0.05920001],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.closepower=[]
        self.handle = np.array(
            [
                [0.99967743, 0.01586798, 0.01983022, -0.53712757],
                [-0.01583831, 0.9998732, -0.0016527, 0.17841741],
                [-0.01985393, 0.00133809, 0.999802, -0.04919224],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.open_waypoint1 = np.array(
            [
                [0.73507608, 0.57945417, 0.35198867, -0.66300158],
                [-0.66570147, 0.51848966, 0.53666566, 0.02048671],
                [0.12847067, -0.62880946, 0.76687284, 0.060322],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.open_waypoint2 = np.array(
            [
                [0.73506929, 0.57946094, 0.3519917, -0.51573811],
                [-0.66570687, 0.51847575, 0.5366724, 0.0295821],
                [0.12848153, -0.62881469, 0.76686673, -0.24252797],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.open_waypoint3 = np.array(
            [
                [0.87617755, 0.47620137, -0.07446584, -0.67929367],
                [0.02228718, 0.11430381, 0.99319581, -0.30597807],
                [0.48147294, -0.8718755, 0.08953725, 0.00769625],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.open_waypoint4 = np.array(
            [
                [0.86786429, 0.49100267, -0.07568326, -0.39439769],
                [0.02223934, 0.11379185, 0.99325567, -0.34387848],
                [0.49630332, -0.86369427, 0.08783631, 0.12079046],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.open_waypoint5 = np.array(
            [
                [0.85845333, 0.51284122, 0.00719491, 0.23259358],
                [-0.00937571, 0.00166529, 0.99995466, -0.36989539],
                [0.51280599, -0.85848186, 0.00623783, -0.22511512],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.close_waypoint0 = [
            [0.71237156, -0.58417015, 0.388937, 0.15358778],
            [0.08451361, 0.62157144, 0.77878521, -0.44990721],
            [-0.6966952, -0.52191397, 0.49216014, -0.34738044],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.close_waypoint1 = np.array(
            [
                [0.71237551, -0.58416324, 0.38894014, 0.29495323],
                [0.08450973, 0.62157644, 0.77878164, -0.4514387],
                [-0.69669164, -0.52191574, 0.49216331, -0.34467472],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.close_waypoint2 = np.array(
            [
                [0.66371594, -0.56251054, 0.49301425, -0.39581818],
                [-0.05489129, 0.62071669, 0.78211107, -0.39202424],
                [-0.7459679, -0.54616177, 0.38110263, -0.10705829],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.close_waypoint3 = np.array(
            [
                [0.99976921, 0.00516625, -0.02085283, -0.42428608],
                [-0.00505706, 0.99997325, 0.00528524, -0.15127851],
                [0.02087957, -0.00517856, 0.99976859, -0.04807866],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

        # 放试管架
        self.waypoint1_j = [
            2.4160375595092773,
            -0.41018974781036377,
            -1.6827280521392822,
            -1.0518676042556763,
            0.8470672965049744,
            0.7902474403381348,
        ]
        self.jia_j = [2.4174211025238037, -0.5450835824012756, -1.7686476707458496, -0.8308948874473572, 0.8498122692108154, 0.8028038144111633]
        # self.jia_j = [2.417541027069092, -0.5449756979942322, -1.7688393592834473, -0.8308230042457581, 0.8499200940132141, 0.8028157949447632]
        self.jia_move = [
            0.2239157110452652,
            -0.5690872073173523,
            0.1765543669462204,
            -2.3725831508636475,
            -1.5707728862762451,
            2.3714568614959717,
        ]

        self.waypoint2_j = [
            2.3617489337921143,
            -0.2606026530265808,
            -1.882265329360962,
            -1.00188148021698,
            0.7927427887916565,
            0.7899118661880493,
        ]
        self.waypoint3_j = [
            0.8695452809333801,
            0.6958703398704529,
            -2.1559131145477295,
            -1.6796133518218994,
            -0.6991494297981262,
            0.7828651070594788,
        ]
        self.jia_close = [
            0.2792383134365082,
            0.054019197821617126,
            -1.1718405485153198,
            -0.4493247866630554,
            1.5603203773498535,
            -0.6294997334480286,
        ]
        self.tubejia_getp = []

        self.watch_aruco_j = []
        self.pose_aruco2 = []
        self.jia_put = []
        self.jia_fang = np.array([
            [-0.01590308, 0.99936596, 0.03185555, -0.21592303],
            [-0.99987128, -0.0159627, 0.00161822, 0.15920248],
            [0.00212569, -0.03182571, 0.99949117, 0.26181279],
            [0.0, 0.0, 0.0, 1.0],
        ]
        )
        

    def point(self):
        duco_cobot.movej(midpose, speedj, 20, 0, True)
        robotiq(1, 1, 255, 0, 100, 1)
        print("开始烘干机操作")
        mark_size(0.150)
        mark_id(587)
        # 移动至识别aruco姿态
        duco_cobot.movej(
            [
                -0.1358712762594223,
                0.41367778182029724,
                -2.256640672683716,
                1.8141716718673706,
                1.7438228130340576,
                -0.7815196514129639,
            ],
            speedj,
            10,
            0,
            True,
        )
        # self.watch_aruco_j = [
        #     -0.05049566552042961,
        #     0.15356017649173737,
        #     -2.0894246101379395,
        #     1.9346251487731934,
        #     1.6355094909667969,
        #     -0.7722198963165283,
        # ]
        # duco_cobot.movej(self.watch_aruco_j, speedj - 30, 10, 0, True)
        # 定位
        aruco.mark(587, 100, 5)
        sleep(0.5)
        # 计算示教点
        self.pose_aruco2 = duco_cobot.get_tcp_pose()
        self.openpower = get_t(self.pose_aruco2, self.openpower)
        self.openpower[2]+=0.007
        self.closepower=self.openpower.copy()
        self.closepower[2]-=0.027
        self.handle = get_t(self.pose_aruco2, self.handle)
        self.open_waypoint1 = get_t(self.pose_aruco2, self.open_waypoint1)
        self.open_waypoint2 = get_t(self.pose_aruco2, self.open_waypoint2)
        self.open_waypoint3 = get_t(self.pose_aruco2, self.open_waypoint3)
        self.open_waypoint4 = get_t(self.pose_aruco2, self.open_waypoint4)
        self.open_waypoint5 = get_t(self.pose_aruco2, self.open_waypoint5)
        self.close_waypoint0 = get_t(self.pose_aruco2, self.close_waypoint0)
        self.close_waypoint1 = get_t(self.pose_aruco2, self.close_waypoint1)
        self.close_waypoint2 = get_t(self.pose_aruco2, self.close_waypoint2)
        self.close_waypoint3 = get_t(self.pose_aruco2, self.close_waypoint3)
        self.jia_fang = get_t(self.pose_aruco2, self.jia_fang)

    def open_door(self):
        # global csvflag
        print("烘干机准备开门")
        # 移动至门把手
        duco_cobot.movel(self.handle, speedl, 0.2, 0, [], "", "", True)
        robotiq(1, 1, 0, 0, 100, 1)
        duco_cobot.tcp_move(
            [0, 0, self.handle_offset, 0, 0, 0], 0.2, 0.2, 0, True)
        # 合并夹爪，抓住门把手
        robotiq(1, 1, 200, 0, 100, 250)
        sleep(1)
        # 力控开门把手
        duco_cobot.fc_config(
            [True, True, True, False, False, False],
            [0, 10, 90, 0, 0, 0],
            [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos(
            [0.06, 0, 0, 0, 0, 0], [0.01, 0, 0, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft(
            [0, 0, 200, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        sleep(0.5)
        # 张开夹爪
        # duco_cobot.fc_config(
        #     [True, True, True, False, False, False],
        #     [0, 0, 0, 0, 0, 0],
        #     [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
        #     [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198],
        #     [0, 0, 0, 0, 0, 0],
        #     "default",
        #     "default",
        #     0,
        # )
     
        # sleep(1)
        duco_cobot.fc_stop()

        pub = rospy.Publisher(
            "Robotiq2FGripperRobotOutput",
            outputMsg.Robotiq2FGripper_robot_output,
            queue_size=1,
        )
        command = outputMsg.Robotiq2FGripper_robot_output()
        command.rACT = 1
        command.rGTO = 1
        command.rPR = 0
        command.rATR = 0
        command.rSP = 50
        command.rFR = 250
        for i in range(3):
            pub.publish(command)
            rospy.sleep(0.1)
        sleep(2)

        rotobiq1= rospy.wait_for_message("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input)
        while rotobiq1.gPO>100:
            print("gPO",rotobiq1.gPO)
            robotiq(1, 1, 0, 0, 100, 250)
        print("张开夹爪")
        # 移动至个路点
        duco_cobot.tcp_move([0, 0, -0.1, 0, 0, 0], 0.2, 0.2, 0, True)
        robotiq(1, 1, 255, 0, 100, 10)
        duco_cobot.movel(self.open_waypoint1, speedl -
                         0.5, 0.2, 0, [], "", "", True)
        duco_cobot.movel(self.open_waypoint2, speedl -
                         0.5, 0.2, 0, [], "", "", True)
        duco_cobot.tcp_move([0, 0, -0.12, 0, 0, 0], 0.2, 0.2, 0, True)
        duco_cobot.movel(self.open_waypoint3, speedl -
                         0.3, 0.2, 0, [], "", "", True)
        duco_cobot.movel(self.open_waypoint4, speedl -
                         0.4, 0.2, 0, [], "", "", True)
        duco_cobot.movel(self.open_waypoint5, 0.2, 0.2, 0, [], "", "", True)
        print("烘干机已开门")

    def close_door(self):
        print("烘干机准备关门")
        # robotiq(1, 1, 255, 0, 100, 1)
        duco_cobot.movel(self.close_waypoint0, speedl -
                         0.1, 0.2, 0, [], "", "", True)
        duco_cobot.movel(self.close_waypoint1, speedl -
                         0.1, 0.2, 0, [], "", "", True)
        duco_cobot.tcp_move([0, 0, 0.07, 0, 0, 0], 0.3, 0.2, 0, True)
        duco_cobot.movel(self.close_waypoint2, 0.2, 0.2, 0, [], "", "", True)
        duco_cobot.movel(self.close_waypoint3, 0.1, 0.1, 0, [], "", "", True)
        # 力控关门
        duco_cobot.fc_config(
            [False, False, True, False, False, False],
            [0, 0, -65, 0, 0, 0],
            [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft(
            [0, 0, -60, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_logic([0, 0, 1])
        duco_cobot.fc_move()
        duco_cobot.fc_config(
            [False, False, True, False, False, False],
            [0, 0, 20, 0, 0, 0],
            [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_wait_pos(
            [0.02, 0, 0, 0, 0, 0], [0.01, 0, 0, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, 0, 0, 0, 0], [
                              0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 0])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        duco_cobot.tcp_move([0, 0, -0.13, 0, 0, 0], 0.4, 0.2, 0, True)
        # 移动至门把手
        self.handle[2] -= 0.005
        duco_cobot.movel(self.handle, speedl - 0.1, 0.2, 0, [], "", "", True)
        duco_cobot.fc_config(
            [False, False, True, False, False, False],
            [0, 0, -60, 0, 0, 0],
            [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft(
            [0, 0, -60, 0, 0, 0], [0, 0, 5, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_logic([0, 0, 1])
        duco_cobot.fc_move()
        duco_cobot.fc_config(
            [False, False, True, False, False, False],
            [0, 0, 20, 0, 0, 0],
            [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_wait_pos(
            [0.02, 0, 0, 0, 0, 0], [0.01, 0, 0, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, 0, 0, 0, 0], [
                              0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 0])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        duco_cobot.tcp_move([0, 0, -0.06, 0, 0, 0], 0.3, 0.2, 0, True)
        self.handle[2] += 0.005
        print("烘干机已关门")

    def open_power(self):
        print("烘干机准备打开电源")
        duco_cobot.movel(self.openpower, speedl - 0.2,
                         0.2, 0, [], "", "", True)
        # 力控关门
        duco_cobot.fc_config(
            [False, False, True, False, False, False],
            [0, 0, -25, 0, 0, 0],
            [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft(
            [0, 0, -20, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_logic([0, 0, 1])
        duco_cobot.fc_move()
        sleep(1)
        duco_cobot.fc_config(
            [False, False, True, False, False, False],
            [0, 0, 20, 0, 0, 0],
            [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_wait_pos(
            [0.02, 0, 0, 0, 0, 0], [0.01, 0, 0, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, 0, 0, 0, 0], [
                              0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 0])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        duco_cobot.tcp_move([0, 0, -0.08, 0, 0, 0], 0.3, 0.2, 0, True)
        print("烘干机电源已打开")

    def close_power(self):
        print("烘干机准备打开电源")
        duco_cobot.movel(self.closepower, speedl - 0.2,
                         0.2, 0, [], "", "", True)
        # 力控关门
        duco_cobot.fc_config(
            [False, False, True, False, False, False],
            [0, 0, -25, 0, 0, 0],
            [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft(
            [0, 0, -20, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_logic([0, 0, 1])
        duco_cobot.fc_move()
        sleep(1)
        duco_cobot.fc_config(
            [False, False, True, False, False, False],
            [0, 0, 20, 0, 0, 0],
            [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_wait_pos(
            [0.02, 0, 0, 0, 0, 0], [0.01, 0, 0, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, 0, 0, 0, 0], [
                              0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 0])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        duco_cobot.tcp_move([0, 0, -0.08, 0, 0, 0], 0.3, 0.2, 0, True)
        print("烘干机电源已打开")


    def tubejia_put(self):
        print("夹取试管架放入烘干机")
        # duco_cobot.movej(midpose, 3, 3, 0, True)
        duco_cobot.movej(self.waypoint1_j, speedj, 10, 0, True)
        motor.motor_position(183000, 10)
        sleep(2)
        motor.motor_position(0, 10)
        duco_cobot.movej(self.jia_j, speedj - 50, 10, 0, True)
        print("移动至jia_j")
        robotiq(1, 1, 170, 0, 100, 250)
        print("开启力控")
        curpose = duco_cobot.get_tcp_pose()
        duco_cobot.fc_config(
            [False, False, True, False, False, False],
            [0, 0, -20, 0, 0, 0],
            [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos(
            [-0.104, 0, 0, 0, 0, 0], [0.005, 0, 0, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft(
            [0, 0, -10, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        print("力控停止")
        curpose1 = duco_cobot.get_tcp_pose()
        if abs(curpose1[0] - curpose[0]) < 0.085:
            print("受力停止了，夹爪未完全进入")
            assert False
        robotiq(1, 1, 255, 0, 20, 200)

        duco_cobot.movel(self.jia_move, speedl - 0.2, 0.2, 0, [], "", "", True)
        print("movel移动至中心")
        sleep(1)
        duco_cobot.movej(self.waypoint2_j, speedj - 30, 10, 0, True)
        duco_cobot.movej(self.waypoint3_j, speedj - 10, 10, 0, True)
        print("self.waypoint3_j",self.waypoint3_j)
        duco_cobot.movel(self.jia_fang, speedl - 0.1, 0.2, 0, [], "", "", True)
        print("self.jia_fang",self.jia_fang)
        # duco_cobot.tcp_move([-0.03, 0, 0.36, 0, 0, 0], 0.2, 0.2, 0, True)
        curpose = duco_cobot.get_tcp_pose()
        print("烘干机放试管夹点", curpose)
        self.jia_put = duco_cobot.get_tcp_pose()
        print("向下放试管架")
        duco_cobot.fc_config(
            [True, False, False, False, False, False],
            [20, 0, 0, 0, 0, 0],
            [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([10, 0, 0, 0, 0, 0], [
                              2, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([0, 0, 1])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        sleep(1)
        robotiq(1, 1, 160, 0, 100, 1)
        duco_cobot.tcp_move([0, 0, -0.11, 0, 0, 0], 0.05, 0.05, 0, True)
        self.tubejia_getp = duco_cobot.get_tcp_pose()
        duco_cobot.movej(self.waypoint3_j, speedj - 20, 10, 0, True)
        robotiq(1, 1, 255, 0, 100, 1)
        duco_cobot.movej(self.jia_close, speedj - 30, 10, 0, True)

    def tubejia_get(self):
        duco_cobot.movej(self.jia_close, speedj - 10, 10, 0, True)
        duco_cobot.movej(self.waypoint3_j, speedj - 20, 10, 0, True)
        robotiq(1, 1, 165, 0, 100, 1)
        duco_cobot.movel(self.tubejia_getp, speedl -
                         0.1, 0.2, 0, [], "", "", True)
        # 力控插入
        curpose = duco_cobot.get_tcp_pose()
        duco_cobot.fc_config(
            [False, False, True, False, False, False],
            [0, 0, -20, 0, 0, 0],
            [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos(
            [-0.115, 0, 0, 0, 0, 0], [0.005, 0, 0, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft(
            [0, 0, -12, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        curpose1 = duco_cobot.get_tcp_pose()
        if curpose[0] - curpose1[0] < 0.08:
            assert False
        robotiq(1, 1, 255, 0, 100, 250)
        duco_cobot.movel(self.jia_put, speedl - 0.1, 0.2, 0, [], "", "", True)
        duco_cobot.movej(self.waypoint3_j, speedj - 10, 10, 0, True)
        duco_cobot.movej(self.waypoint2_j, speedj, 10, 0, True)
        duco_cobot.movel(self.jia_move, speedl - 0.3, 0.2, 0, [], "", "", True)
        sleep(1)
        robotiq(1, 1, 160, 0, 100, 1)
        sleep(1)
        # 力控保护测试
        curpose = duco_cobot.get_tcp_pose()
        duco_cobot.fc_config(
            [False, False, True, False, False, False],
            [0, 0, 15, 0, 0, 0],
            [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos(
            [0.115, 0, 0, 0, 0, 0], [0.005, 0, 0, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, 10, 0, 0, 0], [
                              0, 0, 2, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        curpose1 = duco_cobot.get_tcp_pose()
        if curpose1[0] - curpose[0] < 0.085:
            assert False
        # duco_cobot.tcp_move([0, 0, -0.11, 0, 0, 0], 0.05, 0.05, 0, True)

        motor.motor_position(179000-500, 30)
        robotiq(1, 1, 255, 0, 100, 1)
        curr_pose = duco_cobot.get_tcp_pose()
        curr_pose[2] += 0.1
        duco_cobot.movel(curr_pose, speedl - 0.1, 0.2, 0, [], "", "", True)
        duco_cobot.movej(self.waypoint2_j, speedj - 10, 10, 0, True)
        duco_cobot.movej(self.waypoint3_j, speedj, 10, 0, True)
        duco_cobot.movej(self.jia_close, speedj, 10, 0, True)

    def again(self):
          # 参数
        self.handle_offset = 0.1+0.005  # 路点到门把手距离
        # 过渡矩阵
        self.openpower = np.array(
            [
                [0.9984479, 0.00925176, 0.05491991, -0.64168806],
                [-0.00903779, 0.99995058, -0.0041431, 0.33587667],
                [-0.05495553, 0.00364032, 0.99848217, 0.05920001],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.handle = np.array(
            [
                [0.99967743, 0.01586798, 0.01983022, -0.53712757],
                [-0.01583831, 0.9998732, -0.0016527, 0.17841741],
                [-0.01985393, 0.00133809, 0.999802, -0.04919224],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.open_waypoint1 = np.array(
            [
                [0.73507608, 0.57945417, 0.35198867, -0.66300158],
                [-0.66570147, 0.51848966, 0.53666566, 0.02048671],
                [0.12847067, -0.62880946, 0.76687284, 0.060322],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.open_waypoint2 = np.array(
            [
                [0.73506929, 0.57946094, 0.3519917, -0.51573811],
                [-0.66570687, 0.51847575, 0.5366724, 0.0295821],
                [0.12848153, -0.62881469, 0.76686673, -0.24252797],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.open_waypoint3 = np.array(
            [
                [0.87617755, 0.47620137, -0.07446584, -0.67929367],
                [0.02228718, 0.11430381, 0.99319581, -0.30597807],
                [0.48147294, -0.8718755, 0.08953725, 0.00769625],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.open_waypoint4 = np.array(
            [
                [0.86786429, 0.49100267, -0.07568326, -0.39439769],
                [0.02223934, 0.11379185, 0.99325567, -0.34387848],
                [0.49630332, -0.86369427, 0.08783631, 0.12079046],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.open_waypoint5 = np.array(
            [
                [0.85845333, 0.51284122, 0.00719491, 0.23259358],
                [-0.00937571, 0.00166529, 0.99995466, -0.36989539],
                [0.51280599, -0.85848186, 0.00623783, -0.22511512],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.close_waypoint0 = [
            [0.71237156, -0.58417015, 0.388937, 0.15358778],
            [0.08451361, 0.62157144, 0.77878521, -0.44990721],
            [-0.6966952, -0.52191397, 0.49216014, -0.34738044],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.close_waypoint1 = np.array(
            [
                [0.71237551, -0.58416324, 0.38894014, 0.29495323],
                [0.08450973, 0.62157644, 0.77878164, -0.4514387],
                [-0.69669164, -0.52191574, 0.49216331, -0.34467472],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.close_waypoint2 = np.array(
            [
                [0.66371594, -0.56251054, 0.49301425, -0.39581818],
                [-0.05489129, 0.62071669, 0.78211107, -0.39202424],
                [-0.7459679, -0.54616177, 0.38110263, -0.10705829],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.close_waypoint3 = np.array(
            [
                [0.99976921, 0.00516625, -0.02085283, -0.42428608],
                [-0.00505706, 0.99997325, 0.00528524, -0.15127851],
                [0.02087957, -0.00517856, 0.99976859, -0.04807866],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

        # 放试管架
        self.waypoint1_j = [
            2.4160375595092773,
            -0.41018974781036377,
            -1.6827280521392822,
            -1.0518676042556763,
            0.8470672965049744,
            0.7902474403381348,
        ]
        # self.jia_j = [2.4158217906951904, -0.5467020869255066, -1.7657907009124756, -0.8321366310119629, 0.8460485935211182, 0.790019690990448]
        self.jia_j = [2.4174211025238037, -0.5450835824012756, -1.7686476707458496, -0.8308948874473572, 0.8498122692108154, 0.8028038144111633]
        self.jia_move = [
            0.2239157110452652,
            -0.5690872073173523,
            0.1765543669462204,
            -2.3725831508636475,
            -1.5707728862762451,
            2.3714568614959717,
        ]

        self.waypoint2_j = [
            2.3617489337921143,
            -0.2606026530265808,
            -1.882265329360962,
            -1.00188148021698,
            0.7927427887916565,
            0.7899118661880493,
        ]
        self.waypoint3_j = [
            0.8695452809333801,
            0.6958703398704529,
            -2.1559131145477295,
            -1.6796133518218994,
            -0.6991494297981262,
            0.7828651070594788,
        ]
        self.jia_close = [
            0.2792383134365082,
            0.054019197821617126,
            -1.1718405485153198,
            -0.4493247866630554,
            1.5603203773498535,
            -0.6294997334480286,
        ]
        self.tubejia_getp = []

        self.watch_aruco_j = []
        self.pose_aruco2 = []
        self.jia_put = []
        self.jia_fang = np.array([
            [-0.01590308, 0.99936596, 0.03185555, -0.21592303],
            [-0.99987128, -0.0159627, 0.00161822, 0.15920248],
            [0.00212569, -0.03182571, 0.99949117, 0.26181279],
            [0.0, 0.0, 0.0, 1.0],
        ]
        )


class putsample(object):
    def __init__(self):
        # 取样品的点
        self.pose_aruco2_j1 = [
            0.9242292642593384,
            -0.5668954849243164,
            -1.6513054370880127,
            -0.9170810580253601,
            -2.2216973304748535,
            -0.7864810824394226,
        ]
        self.pose_aruco2_j2 = [
            1.0019710063934326,
            -0.6748853325843811,
            -1.4592461585998535,
            -1.0018454790115356,
            -2.145453691482544,
            -0.7875596880912781,
        ]
        self.getmid_j = [
            1.1011524200439453,
            -0.46504154801368713,
            -1.46921706199646,
            -1.2059367895126343,
            -0.4831337630748749,
            -0.7895490527153015,
        ]
        # 放试管
        self.getA1 = [
            [6.61171217e-01, 6.09278228e-04, -7.50234797e-01, -5.96091754e-02],
            [-8.07656040e-03, 9.99947502e-01, -6.30568329e-03, -4.70810306e-02],
            [7.50191570e-01, 1.02284529e-02, 6.61141428e-01, -1.37114747e-02],
            [0.00000000e00, 0.00000000e00, 0.00000000e00, 1.00000000e00],
        ]
        self.getA2 = [
            [6.61162620e-01, 5.99423119e-04, -7.50242381e-01, -1.25122500e-01],
            [-8.08026767e-03, 9.99947370e-01, -6.32192941e-03, -4.58370895e-02],
            [7.50199106e-01, 1.02419827e-02, 6.61132667e-01, -1.49801838e-02],
            [0.00000000e00, 0.00000000e00, 0.00000000e00, 1.00000000e00],
        ]
        self.getB1 = [
            [0.66117524, 0.00373551, -0.7502222, -0.06093247],
            [-0.0071647, 0.99997344, -0.00133522, -0.04756895],
            [0.75019729, 0.00625793, 0.66118444, 0.05144453],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.getB2 = [
            [3.07202185e-01, 2.05547756e-03, -9.51642051e-01, -7.66056430e-02],
            [-7.48489242e-03, 9.99971955e-01, -2.56352050e-04, -5.07886123e-02],
            [9.51614835e-01, 7.20169028e-03, 3.07208955e-01, 1.37125290e-01],
            [0.00000000e00, 0.00000000e00, 0.00000000e00, 1.00000000e00],
        ]
        self.getC1 = [
            [0.30536254, 0.00373149, -0.95222886, -0.01293747],
            [-0.00713032, 0.99997325, 0.00163202, -0.04976055],
            [0.95220947, 0.00629134, 0.30538098, 0.20379323],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.getC2 = [
            [0.30534482, 0.00372893, -0.95223455, -0.0780043],
            [-0.00710479, 0.99997342, 0.00163764, -0.04970142],
            [0.95221535, 0.00626538, 0.30536319, 0.20199268],
            [0.0, 0.0, 0.0, 1.0],
        ]

        self.pose_aruco2 = []

        # 试管架上的点
        # 放
        self.putA1_j = [1.5838104486465454, -0.6997160315513611, -1.1676387786865234, -1.2743111848831177, 1.5824198722839355, -0.7881619334220886]
        self.putA2_j =[1.5828756093978882, -0.8141413927078247, -0.9655008316040039, -1.3618559837341309, 1.581557035446167, -0.7884136438369751]
        self.putB1_j = [2.2785000801086426, -0.2141392081975937, -1.900474190711975, -1.0248476266860962, 0.7061094045639038, -0.7879582047462463] 
        self.putB2_j = [2.2154629230499268, -0.30411675572395325, -1.781470775604248, -1.0530943870544434, 0.6430963277816772, -0.7885334491729736]
        self.putC1_j =[2.354503870010376, -0.2611652910709381, -1.840205430984497, -1.038557529449463, 0.7820654511451721, -0.7878983020782471]
        self.putC2_j = [2.2887585163116455, -0.34622934460639954, -1.7229278087615967, -1.0698963403701782, 0.7164038419723511, -0.7884136438369751]

        # 取
        self.getA1_jia = [-0.15495067834854126, -0.8207182288169861, 0.19935764372348785, 1.5707206726074219, 3.546402876963839e-05, -3.1415910720825195]
        self.getA2_jia = [-0.15495380759239197, -0.8835901618003845, 0.19936208426952362, 1.5707207918167114, 4.465566235012375e-05, -3.1415822505950928]
        self.getB1_jia = [0.1580297201871872, -0.574856162071228, 0.20085518062114716, 1.5707998275756836, 7.524282409576699e-05, -1.5707906484603882]
        self.getB2_jia = [0.15803466737270355, -0.6398624777793884, 0.20083113014698029, 1.57086181640625, -6.9180359787424095e-06, -1.5707979202270508]
        self.getC1_jia = [0.22305577993392944, -0.5748451352119446, 0.2008548080921173, 1.5709035396575928, -4.972443275619298e-05, -1.5707769393920898]
        self.getC2_jia = [0.2230364829301834, -0.6398715376853943, 0.20087479054927826, 1.5709155797958374, -6.432332884287462e-05, -1.5707952976226807]

    def point(self):
        duco_cobot.movej(midpose, speedj, 20, 0, True)
        print("开始取样品操作")
        robotiq(1, 1, 255, 0, 100, 50)
        motor.motor_position(183900-1000, 10)
        motor.motor_position(177000-1000, 0)
        # 识别aruco
        mark_size(0.120)
        mark_id(580)
        duco_cobot.movej(self.pose_aruco2_j1, speedj, 20, 0, True)
        # duco_cobot.movej(self.pose_aruco2_j2, speedj - 40, 20, 0, True)
        sleep(0.5)
        aruco.mark(580, 100, 5)
        # 计算示教点
        self.pose_aruco2 = duco_cobot.get_tcp_pose()
        self.getA1 = get_t(self.pose_aruco2, self.getA1)
        self.getA2 = get_t(self.pose_aruco2, self.getA2)
        self.getB1 = get_t(self.pose_aruco2, self.getB1)
        self.getB2 = get_t(self.pose_aruco2, self.getB2)
        self.getC1 = get_t(self.pose_aruco2, self.getC1)
        self.getC2 = get_t(self.pose_aruco2, self.getC2)
        self.getA1_up = self.getA1.copy()
        self.getA2_up = self.getA2.copy()
        self.getB1_up = self.getB1.copy()
        self.getB2_up = self.getB2.copy()
        self.getC1_up = self.getC1.copy()
        self.getC2_up = self.getC2.copy()
        self.getA1_up[2] += 0.09
        self.getA2_up[2] += 0.09
        self.getB1_up[2] += 0.09
        self.getB2_up[2] += 0.09
        self.getC1_up[2] += 0.09
        self.getC2_up[2] += 0.09
        duco_cobot.movej(self.pose_aruco2_j1, speedj - 40, 20, 0, True)
        curpose = duco_cobot.get_tcp_pose()
        curpose[2] += 0.14
        duco_cobot.movel(curpose, speedl - 0.1, 0.2, 0, [], "", "", True)
        duco_cobot.movej(self.getmid_j, speedj, 10, 0, True)

    def put(self, jiaput_j, jiaget, putpose, flag):
        if flag:
            robotiq(1, 1, 130, 0, 100, 75)
        # duco_cobot.movej(self.jiamiddle, 40, 10, 0, True)
        duco_cobot.movej(jiaput_j, speedj, 10, 0, True)
        duco_cobot.movel(jiaget, speedl - 0.3, 0.2, 0, [], "", "", True)

        robotiq(1, 1, 250, 0, 100, 50)
        curpose = duco_cobot.get_tcp_pose()
        duco_cobot.fc_config(
            [False, True, False, False, False, False],
            [0, -30, 0, 0, 0, 0],
            [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos(
            [0, 0, 0.15, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft(
            [0, -20, 0, 0, 0, 0], [0, 2, 0, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        print("forcemove stop")
        sleep(0.5)
        curpose1 = duco_cobot.get_tcp_pose()
        if curpose1[2] - curpose[1] < 0.08:
            print("取试管受力停止了，取出试管未成功")
            robotiq(1, 1, 120, 0, 100, 50)
            assert False
        # duco_cobot.movej(self.jiamiddle, 40, 10, 0, True)

        
        duco_cobot.movej(self.getmid_j, speedj, 10, 0, True)
        duco_cobot.movel(putpose, speedl - 0.2, 0.2, 0, [], "", "", True)
        duco_cobot.fc_config(
            [True, True, True, False, False, False],
            [0, 20, 0, 0, 0, 0],
            [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
            [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198],
            [0, 0, 0, 0, 0, 0],
            "default",
            "default",
            0,
        )
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos(
            [0, 0, -0.08, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [
                               0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft(
            [12, 12, 12, 0, 0, 0], [2, 2, 2, 0, 0, 0], False, 0, 500000
        )
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        sleep(0.5)
        robotiq(1, 1, 130, 0, 100, 50)
        duco_cobot.movel(putpose, speedl - 0.1, 0.2, 0, [], "", "", True)
        duco_cobot.movej(self.getmid_j, speedj, 10, 0, True)
        # duco_cobot.movej(self.jiamiddle, 40, 10, 0, True)

    def put_tube(self, test):
        if test == 0:
            motor.motor_position(179000-1000, 10)
            self.put(self.putC1_j, self.getC1_jia, self.getA1_up, 1)
            robotiq(1, 1, 255, 0, 100, 1)
            duco_cobot.movej(midpose, speedj, 10, 0, True)
        elif test == 1:
            motor.motor_position(179000-1000, 10)
            self.put(self.putC2_j, self.getC2_jia, self.getC2_up, 1)
            self.put(self.putC1_j, self.getC1_jia, self.getC1_up, 0)
            self.put(self.putB2_j, self.getB2_jia, self.getB2_up, 0)
            self.put(self.putB1_j, self.getB1_jia, self.getB1_up, 0)
            self.put(self.putA2_j, self.getA2_jia, self.getA2_up, 0)
            self.put(self.putA1_j, self.getA1_jia, self.getA1_up, 0)
            robotiq(1, 1, 255, 0, 100, 1)
            duco_cobot.movej(midpose, speedj, 10, 0, True)
        else:
            print("输入错误")

    def again(self):
         # 取样品的点
        self.pose_aruco2_j1 = [
            0.9242292642593384,
            -0.5668954849243164,
            -1.6513054370880127,
            -0.9170810580253601,
            -2.2216973304748535,
            -0.7864810824394226,
        ]
        self.pose_aruco2_j2 = [
            1.0019710063934326,
            -0.6748853325843811,
            -1.4592461585998535,
            -1.0018454790115356,
            -2.145453691482544,
            -0.7875596880912781,
        ]
        self.getmid_j = [
            1.1011524200439453,
            -0.46504154801368713,
            -1.46921706199646,
            -1.2059367895126343,
            -0.4831337630748749,
            -0.7895490527153015,
        ]
        # 放试管
        self.getA1 = [
            [6.61171217e-01, 6.09278228e-04, -7.50234797e-01, -5.96091754e-02],
            [-8.07656040e-03, 9.99947502e-01, -6.30568329e-03, -4.70810306e-02],
            [7.50191570e-01, 1.02284529e-02, 6.61141428e-01, -1.37114747e-02],
            [0.00000000e00, 0.00000000e00, 0.00000000e00, 1.00000000e00],
        ]
        self.getA2 = [
            [6.61162620e-01, 5.99423119e-04, -7.50242381e-01, -1.25122500e-01],
            [-8.08026767e-03, 9.99947370e-01, -6.32192941e-03, -4.58370895e-02],
            [7.50199106e-01, 1.02419827e-02, 6.61132667e-01, -1.49801838e-02],
            [0.00000000e00, 0.00000000e00, 0.00000000e00, 1.00000000e00],
        ]
        self.getB1 = [
            [0.66117524, 0.00373551, -0.7502222, -0.06093247],
            [-0.0071647, 0.99997344, -0.00133522, -0.04756895],
            [0.75019729, 0.00625793, 0.66118444, 0.05144453],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.getB2 = [
            [3.07202185e-01, 2.05547756e-03, -9.51642051e-01, -7.66056430e-02],
            [-7.48489242e-03, 9.99971955e-01, -2.56352050e-04, -5.07886123e-02],
            [9.51614835e-01, 7.20169028e-03, 3.07208955e-01, 1.37125290e-01],
            [0.00000000e00, 0.00000000e00, 0.00000000e00, 1.00000000e00],
        ]
        self.getC1 = [
            [0.30536254, 0.00373149, -0.95222886, -0.01293747],
            [-0.00713032, 0.99997325, 0.00163202, -0.04976055],
            [0.95220947, 0.00629134, 0.30538098, 0.20379323],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.getC2 = [
            [0.30534482, 0.00372893, -0.95223455, -0.0780043],
            [-0.00710479, 0.99997342, 0.00163764, -0.04970142],
            [0.95221535, 0.00626538, 0.30536319, 0.20199268],
            [0.0, 0.0, 0.0, 1.0],
        ]

        self.pose_aruco2 = []

        # 试管架上的点
        # 放
        self.putA1_j = [1.5838104486465454, -0.6997160315513611, -1.1676387786865234, -1.2743111848831177, 1.5824198722839355, -0.7881619334220886]
        self.putA2_j =[1.5828756093978882, -0.8141413927078247, -0.9655008316040039, -1.3618559837341309, 1.581557035446167, -0.7884136438369751]
        self.putB1_j = [2.2785000801086426, -0.2141392081975937, -1.900474190711975, -1.0248476266860962, 0.7061094045639038, -0.7879582047462463] 
        self.putB2_j = [2.2154629230499268, -0.30411675572395325, -1.781470775604248, -1.0530943870544434, 0.6430963277816772, -0.7885334491729736]
        self.putC1_j =[2.354503870010376, -0.2611652910709381, -1.840205430984497, -1.038557529449463, 0.7820654511451721, -0.7878983020782471]
        self.putC2_j = [2.2887585163116455, -0.34622934460639954, -1.7229278087615967, -1.0698963403701782, 0.7164038419723511, -0.7884136438369751]

        # 取
        self.getA1_jia = [-0.15495067834854126, -0.8207182288169861, 0.19935764372348785, 1.5707206726074219, 3.546402876963839e-05, -3.1415910720825195]
        self.getA2_jia = [-0.15495380759239197, -0.8835901618003845, 0.19936208426952362, 1.5707207918167114, 4.465566235012375e-05, -3.1415822505950928]
        self.getB1_jia = [0.1580297201871872, -0.574856162071228, 0.20085518062114716, 1.5707998275756836, 7.524282409576699e-05, -1.5707906484603882]
        self.getB2_jia = [0.15803466737270355, -0.6398624777793884, 0.20083113014698029, 1.57086181640625, -6.9180359787424095e-06, -1.5707979202270508]
        self.getC1_jia = [0.22305577993392944, -0.5748451352119446, 0.2008548080921173, 1.5709035396575928, -4.972443275619298e-05, -1.5707769393920898]
        self.getC2_jia = [0.2230364829301834, -0.6398715376853943, 0.20087479054927826, 1.5709155797958374, -6.432332884287462e-05, -1.5707952976226807]


motor = motor()
tube = testtube()

if __name__ == "__main__":

    # 初始化节点
    # rospy.init_node("robot")
    try:
        thd_B = threading.Thread(target=hearthread_fun, daemon=True)
        thd_B.daemon = True
        thd_B.start()

        # sam=getsample()
        # sam.get_tube(1)

        # put.put_tube(1)
        # dry =drying()
        # sep=seperation(0)
        # sep.throw_tube(90,1)
        # jj=[1.782317042350769, -0.7571444511413574, -1.0968959331512451, -1.2871463298797607, 1.4147486686706543, 2.3532748222351074]

        A=  [
            0.8695452809333801,
            0.6958703398704529,
            -2.1559131145477295,
            -1.6796133518218994,
            -0.6991494297981262,
            0.7828651070594788,
        ]
        duco_cobot.movej(A, 20, 10, 0, True)

        B=[-0.8338399362986422, -0.013988382960223061, 0.5310756581842542, -2.6045557853009345, -1.5687765413807229, 2.5687156781698275]
        duco_cobot.movel(B, 0.2, 0.2, 0, [], "", "", True)
        # pose = duco_cobot.get_tcp_pose()
        # jj=[-0.0014623674796894193, -0.8673410415649414, 0.19342006742954254, -1.5708242654800415, -1.2584327123477124e-05, 0.36597388982772827]

       

        #         # duco_cobot.tcp_move([0, 0, 0.1, 0, 0, 0], 0.2, 0.2, 0, True)
        #         joint = duco_cobot.get_actual_joints_position()
        #         pose = duco_cobot.get_tcp_pose()
        #         print("joint", joint)
        #         print("pose", pose)
        # Close!
        pose = duco_cobot.get_tcp_pose()
        posej = duco_cobot.get_actual_joints_position()
        print("pose", pose)
        print("posej", posej)
        duco_cobot.close()
        # duco_cobot.disable(True)
        # duco_cobot.power_off(True)
        # duco_cobot.shutdown(True)
    except Thrift.TException as tx:
        print("%s" % tx.message)
