import csv
import threading

import roslib

roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from aruco import *
from tube_motor import motor

sys.path.append('gen_py')
sys.path.append('lib')
from thrift import Thrift

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
midpose = [1.668006181716919, 0.10159657150506973, -1.3602924346923828, -0.30960047245025635, 1.5602844953536987,
           -0.629919171333313]
header = ['x', 'y', 'z', 'r', 'p', 'y']
# csvflag = 1
speedl = 0.7
speedj = 70


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
    sleep(1)
    param = rospy.get_param("/aruco_single/marker_size")
    while param != size:
        rospy.set_param("/aruco_single/marker_size", size)
        sleep(1)
        param = rospy.get_param("/aruco_single/marker_size")
        


def robotiq(ACT, GTO, PR, ATR, SP, FR):
    """
    rACT: 复位:0,激活:1
    rGTO: 1
    rPR:  位置
    rATR: 0
    rSP:  速度
    rFR:  力
    """

    pub = rospy.Publisher('Robotiq2FGripperRobotOutput',
                          outputMsg.Robotiq2FGripper_robot_output, queue_size=1)
    command = outputMsg.Robotiq2FGripper_robot_output()
    for i in range(2):
        command = outputMsg.Robotiq2FGripper_robot_output()
        command.rACT = ACT
        command.rGTO = GTO
        command.rPR = PR
        command.rATR = ATR
        command.rSP = SP
        command.rFR = FR
        pub.publish(command)
        rospy.sleep(0.1)
    sleep(2)


def key(mid, key_open, force):
    duco_cobot.movel(mid, speedl, 0.2, 0, [], "", "", True)
    ret = duco_cobot.movel(key_open, speedl - 0.2, 0.2, 0, [], "", "", True)
    print("到达按键上方", ret)
    # 力控按键
    duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, force, 0, 0, 0],
                         [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
                         [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                         "default", 0)
    duco_cobot.fc_start()
    duco_cobot.fc_wait_pos([0, 0, 0.1, 0, 0, 0], [0, 0, 0.005, 0, 0, 0], False, 0, 5000)
    duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
    duco_cobot.fc_wait_ft([0, 0, force, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 1, 500000)
    duco_cobot.fc_wait_logic([0, 0, 1])
    duco_cobot.fc_move()
    duco_cobot.fc_stop()
    sleep(0.5)
    # 力控返回按键上方
    duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, -force, 0, 0, 0],
                         [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
                         [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                         "default", 0)
    duco_cobot.fc_start()
    duco_cobot.fc_wait_pos([0, 0, 0.03, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 5000)
    duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
    duco_cobot.fc_wait_ft([0, 0, -force, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 1, 500000)
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
    eelink2goal = [tran[0], tran[1], tran[2], euler[0], euler[1], euler[2]]  # 转动后目标相对于基座的T
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
        duco_cobot = DucoCobot('192.168.1.10', 7003)
        duco_cobot.open()
        global csvflag
        with open('/home/sia/YuLin_lab/src/DucoCobotAPI_py/cvs_test.csv', 'a+', encoding='utf-8-sig') as file_obj:
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
        print('Last thread stop!')
        self.thread_stop = True


class testtube(object):
    def __init__(self):
        # 离心机操作：取放试管中间点
        self.tube_middleA_j = [1.6679942607879639, 0.13228817284107208, -1.5182205438613892, -0.1812257021665573,
                               1.560260534286499, -0.630290687084198]
        self.tube_middleB_j = [1.8543009757995605, -0.9173381924629211, -1.2019808292388916, 2.118163585662842,
                               -1.3160853385925293, 2.3607800006866455]
        self.tube_middleB_j1 = [1.7878724336624146, -0.264102041721344, -1.4782891273498535, 1.6719069480895996,
                                -0.03256284445524216, 2.3607442378997803]
        self.tube_middleB_j2 = [0.7730482816696167, 0.5933213233947754, -2.4068868160247803, 1.8143993616104126,
                                0.8782861828804016, 2.3542966842651367]
        # 离心机操作：试管架取试管至离心机
        self.tube_getA1 = [-0.02102518640458584, -0.5755000114440918 - 0.0007, 0.5018289187431335 - 0.001,
                           -3.1415581703186035,
                           -1.5060706573422067e-05, -3.1415655612945557]
        self.tube_getA2 = [-0.08533629775047302, -0.575955331325531 - 0.0007, 0.5018990303993225 - 0.0003,
                           3.141589641571045,
                           -2.3479329684050754e-05, -3.1415669918060303]
        self.tube_getA3 = [-0.15150830149650574, -0.5760762691497803 - 0.0007, 0.5018643902778625 - 0.0005,
                           -3.1415746212005615,
                           -1.9144245015922934e-05, -3.1415579319000244]
        # self.tube_getA4 = [-0.019586972892284393, -0.6398314237594604-0.0005, 0.5027056336402893, -3.141571521759033,
        #                    -1.261985471501248e-05, -3.141564130783081]
        self.tube_getB1 = [0.11821344494819641, -0.873672604560852 - 0.0007, 0.2247268408536911, 1.570852518081665,
                           1.9484890799503773e-05, -2.6035842895507812]
        self.tube_getB2 = [0.054479941725730896, -0.8744994401931763 - 0.0007, 0.2240978181362152, 1.570809245109558,
                           2.603209736662393e-07, -2.6035852432250977]
        self.tube_getB3 = [-0.01158121507614851, -0.8735242486000061 - 0.0007, 0.22439850866794586, 1.570773959159851,
                           6.576499345101183e-06, -2.6048853397369385]
        # 离心机操作：离心机取试管放至试管架
        self.tube_putA1 = [-0.08533278107643127 + 0.065, -0.5764991173744202, 0.65245521068573, 3.141589403152466,
                           -1.6222867998294532e-05, -3.1415767669677734]
        self.tube_putA2 = [-0.08533278107643127, -0.5764991173744202, 0.65245521068573, 3.141589403152466,
                           -1.6222867998294532e-05, -3.1415767669677734]
        self.tube_putA3 = [-0.08533278107643127 - 0.065, -0.5764991173744202, 0.65245521068573, 3.141589403152466,
                           -1.6222867998294532e-05, -3.1415767669677734]
        self.tube_putB1 = [0.051635488867759705 + 0.065 + 0.001, -0.8761548399925232, 0.34584997177124023,
                           1.5708034038543701, 5.354250788514037e-06, -2.6035778522491455]
        self.tube_putB2 = [0.051635488867759705 + 0.0005, -0.8761548399925232, 0.34584997177124023, 1.5708034038543701,
                           5.354250788514037e-06, -2.6035778522491455]
        self.tube_putB3 = [0.051635488867759705 - 0.065 + 0.0008, -0.8761548399925232 - 0.0005, 0.34584997177124023,
                           1.5708034038543701, 5.354250788514037e-06, -2.6035778522491455]
        # 放盖（试管上）
        self.lid_putA1 = [-0.15130600790977478 + 0.13 + 0.0008, -0.5780059122657776 + 0.0005, 0.5333243451118469 + 0.01,
                          3.141587018966675,
                          -9.225173926097341e-06, 3.1415891647338867]
        self.lid_putA2 = [-0.15130600790977478 + 0.065, -0.5780059122657776 + 0.0005, 0.5333243451118469 + 0.01,
                          3.141587018966675,
                          -9.225173926097341e-06, 3.1415891647338867]
        self.lid_putA3 = [-0.15130600790977478, -0.5780059122657776, 0.5333243451118469 + 0.01, 3.141587018966675,
                          -9.225173926097341e-06, 3.1415891647338867]
        self.lid_putB1 = [-0.15104584531784058 + 0.13 - 0.001, -0.6420934796333313, 0.5335747442245483 + 0.01,
                          -3.1415863037109375,
                          -1.5198091205093078e-05, -3.141561985015869]
        self.lid_putB2 = [-0.15104584531784058 + 0.065 - 0.001, -0.6420934796333313 - 0.0005, 0.5335747442245483 + 0.01,
                          -3.1415863037109375,
                          -1.5198091205093078e-05, -3.141561985015869]
        self.lid_putB3 = [-0.15104584531784058, -0.6420934796333313, 0.5335747442245483 + 0.01, -3.1415863037109375,
                          -1.5198091205093078e-05, -3.141561985015869]
        # 取盖（试管上）
        self.lid_getA1 = [-0.019693441689014435, -0.5790014456748962, 0.5019154480934143 - 0.001, -3.141402244567871,
                          3.928263595298631e-06, -3.141526937484741]
        self.lid_getA2 = [-0.0853310078382492, -0.5780207741737366 - 0.0003, 0.5019515224456787 - 0.001,
                          -3.141307830810547,
                          -5.971960490569472e-06, -3.1415419578552246]
        self.lid_getA3 = [-0.1502656638622284, -0.5790724876403809, 0.50198624792099 - 0.001, -3.14141845703125,
                          -2.8963268050574698e-05, 3.141592502593994]
        self.lid_getB1 = [-0.019964830949902534, -0.644310509967804 + 0.001, 0.5020496543884277 - 0.001,
                          -3.141547203063965,
                          -1.1541584171936847e-05, -3.141584873199463]
        self.lid_getB2 = [-0.08533148467540741, -0.6442565247535706 + 0.001, 0.501956396484375 - 0.001,
                          -3.1414411067962646,
                          -6.6090374275518116e-06, -3.1415536403656006]
        self.lid_getB3 = [-0.08533148467540741 - 0.065, -0.6442565247535706 + 0.001, 0.501956396484375 - 0.001,
                          -3.1414411067962646,
                          -6.6090374275518116e-06, -3.1415536403656006]
        # 放盖（平台上）
        self.lid_middle_j = [2.1970736980438232, 0.1850786805152893, -1.890318751335144 + 0.01, 0.1344507783651352,
                             1.5681461095809937, -0.15728531777858734]
        self.lid_put_A1 = [0.17654745111465454, -0.6428665948867798, 0.3380849766731262 + 0.01, -3.14157772064209,
                           7.498136710637482e-06, -3.141587257385254]
        self.lid_put_A2 = [0.1761367436170578, -0.57834138237953186, 0.3329808223247528 + 0.01, -3.1415717601776123,
                           -1.3786975614493713e-06, 3.1415927410125732]
        self.lid_put_A3 = [0.1755574479341507, -0.5123878717422485, 0.3362674021720886 + 0.01, -3.141591787338257,
                           -3.309639578219503e-05, 3.141589641571045]
        self.lid_put_B1 = [0.24196427891254425, -0.642531249332428, 0.33565840244293213 + 0.01, -3.1415765285491943,
                           -1.5136298316065222e-05, -3.141591787338257]
        self.lid_put_B2 = [0.24161803722381592, -0.577235424041748, 0.3394046628475189 + 0.01, -3.1415646076202393,
                           -8.734304174140561e-06, -3.1415889263153076]
        self.lid_put_B3 = [0.24117670953273773, -0.5114811658859253, 0.3339354205131531 + 0.01, -3.141563653945923,
                           -6.808570105931722e-06, 3.1415908336639404]
        # 取盖（平台上）
        self.lid_get_A1 = [0.17644771933555603, -0.6428622437477112 - 0.001, 0.3194399272918701 + 0.012,
                           3.1415696144104004,
                           -2.301438507856801e-05, -3.1415607929229736]
        self.lid_get_A2 = [0.17595918476581573, -0.5775243738174438, 0.3194379369735718 + 0.012, -3.141399621963501,
                           0.00012959253217559308, -3.1415703296661377]
        self.lid_get_A3 = [0.175916388630867, -0.5125937129974365, 0.31944404644966125 + 0.012, -3.1415116786956787,
                           3.979872417403385e-05, -3.1415882110595703]
        self.lid_get_B1 = [0.24187880754470825, -0.6427266597747803, 0.3190431127548218 + 0.012, -3.141503095626831,
                           4.682899088948034e-05, -3.1415581703186035]
        self.lid_get_B2 = [0.24161997437477112, -0.5777313113212585, 0.31911434030532837 + 0.012, -3.1415576934814453,
                           4.088248897460289e-05, -3.1415789127349854]
        self.lid_get_B3 = [0.24118617177009583, -0.5116812850952148, 0.3194091215133667 + 0.012, -3.1414153575897217,
                           0.00011468186130514368, -3.1415674686431885]

    def get_tubeA(self, putpose, getpose):
        """
        离心机操作：试管架上取试管放入离心机A半圆3个试管
        """
        # 移动至夹试管中间点
        duco_cobot.movej(self.tube_middleA_j, speedj, 10, 0, True)
        robotiq(1, 1, 120, 0, 100, 1)
        # 夹试管
        duco_cobot.movel(putpose, speedl - 0.1, 0.2, 0, [], "", "", True)
        ret = duco_cobot.movel(getpose, 0.2, 0.1, 0, [], "", "", True)
        print("移动至试管夹取点，准备夹试管", ret)
        robotiq(1, 1, 255, 0, 100, 150)
        # 力控夹起试管
        curpose = duco_cobot.get_tcp_pose()
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, 25, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0.15, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, 15, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000)
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
        ret = duco_cobot.movel(getpose, 0.2, 0.2, 0, [], "", "", True)
        print("移动至试管夹取点，准备夹试管", ret)
        robotiq(1, 1, 255, 0, 100, 150)
        # 力控夹起试管
        curpose = duco_cobot.get_tcp_pose()
        duco_cobot.fc_config([False, True, False, False, False, False], [0, -25, 0, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0.15, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, -15, 0, 0, 0, 0], [0, 2, 0, 0, 0, 0], False, 0, 500000)
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
        duco_cobot.movel(putpose, 0.3, 0.2, 0, [], "", "", True)
        curpose = duco_cobot.get_tcp_pose()
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, -25, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, -0.155, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, -10, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000)
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
        duco_cobot.movel(putpose, 0.3, 0.2, 0, [], "", "", True)
        duco_cobot.fc_config([False, True, False, False, False, False], [0, 25, 0, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, -0.13, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 20, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], False, 0, 500000)
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
        motor.motor_position(187500, 0)
        # 拔试管盖
        duco_cobot.movel(putlid, speedl - 0.2, 0.2, 0, [], "", "", True)
        duco_cobot.movel(getlid, 0.2, 0.2, 0, [], "", "", True)
        robotiq(1, 1, 255, 0, 100, 250)
        sleep(0.5)
        curpose = duco_cobot.get_tcp_pose()
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, 60, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0.1, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([100, 100, 100, 0, 0, 0], [5, 5, 5, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        print("forcemove stop")
        curpose1 = duco_cobot.get_tcp_pose()
        if curpose1[2] - curpose[2] < 0.07:
            robotiq(1, 1, 120, 0, 200, 1)
            print("取试管盖受力停止了，取出试管未成功")
            assert False
        motor.motor_position(179000, 0)
        # robotiq(1, 1, 120, 0, 200, 1)
        # 放试管盖（平台上）
        duco_cobot.movej(self.lid_middle_j, speedj, 10, 0, True)
        duco_cobot.movel(put_lid, 0.2, 0.2, 0, [], "", "", True)
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, -20, 0, 0, 0],
                             [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_ft([0, 0, -10, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 3000)
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
        duco_cobot.movel(get_lid, 0.3, 0.2, 0, [], "", "", True)
        robotiq(1, 1, 255, 0, 100, 200)
        # 力控夹起试管盖
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, 20, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0.02, 0, 0, 0], [0, 0, 0.005, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 0])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        sleep(1)
        duco_cobot.movej(self.lid_middle_j, speedj - 20, 10, 0, True)
        duco_cobot.movel(putlid, 0.3, 0.2, 0, [], "", "", True)
        # 力控盖盖子
        curpose = duco_cobot.get_tcp_pose()
        # -45N
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, -45, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 4000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, -42, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 4000)
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
        motor.motor_position(183000, 10)
        # 测试
        if test == 0:
            self.lid_put(self.lid_putA1, self.lid_put_A1, self.lid_get_A1, 0)
            motor.motor_position(179000, 0)
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
            motor.motor_position(179000, 0)
            # 取样品结束合并夹爪
            robotiq(1, 1, 255, 0, 100, 50)
        else:
            print("输入错误")
        duco_cobot.movej(midpose, 50, 10, 0, True)


class getsample(object):
    def __init__(self):
        self.pose_aruco2_j1 = [0.9242292642593384, -0.5668954849243164, -1.6513054370880127, -0.9170810580253601,
                               -2.2216973304748535, -0.7864810824394226]
        self.pose_aruco2_j2 = [1.0019710063934326, -0.6748853325843811, -1.4592461585998535, -1.0018454790115356,
                               -2.145453691482544, -0.7875596880912781]
        self.getmid_j = [1.1011524200439453, -0.46504154801368713, -1.46921706199646, -1.2059367895126343,
                         -0.4831337630748749, -0.7895490527153015]
        # 取试管
        self.getA1 = [[0.66095653, -0.00496142, -0.75040779, -0.04822622],
                      [-0.0048543, 0.99992895, -0.01088682, -0.04246517],
                      [0.75040849, 0.01083842, 0.66088549, -0.01467518],
                      [0., 0., 0., 1.]]
        self.getA2 = [[0.66094795, -0.00497128, -0.75041528, -0.11374503],
                      [-0.00485806, 0.99992876, -0.0109031, -0.04158521],
                      [0.75041602, 0.01085195, 0.66087671, -0.01596513],
                      [0., 0., 0., 1.]]
        self.getB1 = [[0.66095548, -0.00183406, -0.75042288, -0.04956844],
                      [-0.00394244, 0.99997473, -0.00591639, -0.0430003],
                      [0.75041476, 0.00686897, 0.66093154, 0.05048007],
                      [0., 0., 0., 1.]]
        self.getB2 = [[0.30692276, -0.00351438, -0.95172794, -0.06525192],
                      [-0.00635653, 0.99996331, -0.00574241, -0.04635958],
                      [0.9517132, 0.00781216, 0.30688916, 0.13615358],
                      [0., 0., 0., 1.]]
        self.getC1 = [[0.30508097, -0.00183809, -0.95232464, -0.00161262],
                      [-0.00601256, 0.99997449, -0.00385621, -0.04501785],
                      [0.95230743, 0.00690237, 0.30506214, 0.20284351],
                      [0., 0., 0., 1.]]
        self.getC2 = [[0.3050631, -0.00184065, -0.95233036, -0.06667817],
                      [-0.00598714, 0.99997466, -0.00385061, -0.04531987],
                      [0.95231332, 0.00687642, 0.30504435, 0.20102115],
                      [0., 0., 0., 1.]]
        # 取试管上方
        self.getA1_up = []
        self.getA2_up = []
        self.getB1_up = []
        self.getB2_up = []
        self.getC1_up = []
        self.getC2_up = []
        # 放试管
        self.putA1 = [2.3517422676086426, -0.28223419189453125 + 0.0005, -1.8454258441925049, -1.0169093608856201,
                      0.7814416885375977, -0.7715128064155579]
        self.putA2 = [2.286440134048462, -0.3664233684539795 + 0.0005, -1.7275609970092773, -1.051507830619812,
                      0.7162235379219055, -0.7708176970481873]
        self.putB1 = [2.276193618774414, -0.23610690236091614 + 0.0008, -1.9065214395523071, -1.0024445056915283,
                      0.7059170603752136, -0.7713210582733154]
        self.putB2 = [2.2135400772094727, -0.32462239265441895 + 0.0005, -1.7869548797607422, -1.0342985391616821,
                      0.6432875394821167, -0.770530104637146]
        self.putC1 = [1.606718897819519, -0.7031920552253723 - 0.0006, -1.1894333362579346, -1.2490800619125366,
                      1.5409659147262573, -0.7810162901878357]
        self.putC2 = [1.604477882385254, -0.8198584914207458 - 0.0007, -0.9826695322990417, -1.3392730951309204,
                      1.538760781288147, -0.7807766199111938]

        self.pose_aruco2 = []

    def point(self):
        """
        识别二维码
        """
        # 移动至aruco前方
        duco_cobot.movej(midpose, speedj, 20, 0, True)
        print("开始取样品操作")
        robotiq(1, 1, 255, 0, 100, 1)
        motor.motor_position(183900, 10)
        motor.motor_position(177000, 0)
        # 识别aruco
        mark_size(0.130)
        duco_cobot.movej(self.pose_aruco2_j1, speedj, 20, 0, True)
        duco_cobot.movej(self.pose_aruco2_j2, speedj - 30, 20, 0, True)
        sleep(0.5)
        aruco.mark()
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
        duco_cobot.movel(getpose_up, 0.3, 0.2, 0, [], "", "", True)
        if num == 0:
            robotiq(1, 1, 135, 0, 100, 1)
        # 力控向下
        # curpose = duco_cobot.get_tcp_pose()
        duco_cobot.movel(getpose, 0.3, 0.2, 0, [], "", "", True)

        robotiq(1, 1, 250, 0, 50, 150)

        curpose = duco_cobot.get_tcp_pose()
        duco_cobot.fc_config([True, True, True, False, False, False], [0, -20, 0, 0, 0, 0],
                             [2000, 2000, 2000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0.10, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, -20, 0, 0, 0, 0], [0, -2, 0, 0, 0, 0], False, 0, 500000)
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
        # 把试管放到试管架上
        duco_cobot.movej(putpose_j, speedj, 10, 0, True)
        # robotiq(1, 1, 250, 0, 150, 150)
        # 力控将试管放入试管架
        duco_cobot.fc_config([False, True, False, False, False, False], [0, 32, 0, 0, 0, 0],
                             [1000, 2000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, -0.132, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000)
        # duco_cobot.fc_wait_pos([0, 0, -0.14, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 30, 0, 0, 0, 0], [0, 2, 0, 0, 0, 0], False, 0, 8000)
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
        self.pose_aruco2_j1 = [0.9242292642593384, -0.5668954849243164, -1.6513054370880127, -0.9170810580253601,
                               -2.2216973304748535, -0.7864810824394226]
        self.pose_aruco2_j2 = [1.0019710063934326, -0.6748853325843811, -1.4592461585998535, -1.0018454790115356,
                               -2.145453691482544, -0.7875596880912781]
        self.getmid_j = [1.1011524200439453, -0.46504154801368713, -1.46921706199646, -1.2059367895126343,
                         -0.4831337630748749, -0.7895490527153015]
        # 取试管
        self.getA1 = [[0.66095653, -0.00496142, -0.75040779, -0.04822622],
                      [-0.0048543, 0.99992895, -0.01088682, -0.04246517],
                      [0.75040849, 0.01083842, 0.66088549, -0.01467518],
                      [0., 0., 0., 1.]]
        self.getA2 = [[0.66094795, -0.00497128, -0.75041528, -0.11374503],
                      [-0.00485806, 0.99992876, -0.0109031, -0.04158521],
                      [0.75041602, 0.01085195, 0.66087671, -0.01596513],
                      [0., 0., 0., 1.]]
        self.getB1 = [[0.66095548, -0.00183406, -0.75042288, -0.04956844],
                      [-0.00394244, 0.99997473, -0.00591639, -0.0430003],
                      [0.75041476, 0.00686897, 0.66093154, 0.05048007],
                      [0., 0., 0., 1.]]
        self.getB2 = [[0.30692276, -0.00351438, -0.95172794, -0.06525192],
                      [-0.00635653, 0.99996331, -0.00574241, -0.04635958],
                      [0.9517132, 0.00781216, 0.30688916, 0.13615358],
                      [0., 0., 0., 1.]]
        self.getC1 = [[0.30508097, -0.00183809, -0.95232464, -0.00161262],
                      [-0.00601256, 0.99997449, -0.00385621, -0.04501785],
                      [0.95230743, 0.00690237, 0.30506214, 0.20284351],
                      [0., 0., 0., 1.]]
        self.getC2 = [[0.3050631, -0.00184065, -0.95233036, -0.06667817],
                      [-0.00598714, 0.99997466, -0.00385061, -0.04531987],
                      [0.95231332, 0.00687642, 0.30504435, 0.20102115],
                      [0., 0., 0., 1.]]
        # 取试管上方
        self.getA1_up = []
        self.getA2_up = []
        self.getB1_up = []
        self.getB2_up = []
        self.getC1_up = []
        self.getC2_up = []
        # 放试管
        self.putA1 = [2.3517422676086426, -0.28223419189453125 + 0.0005, -1.8454258441925049, -1.0169093608856201,
                      0.7814416885375977, -0.7715128064155579]
        self.putA2 = [2.286440134048462, -0.3664233684539795 + 0.0005, -1.7275609970092773, -1.051507830619812,
                      0.7162235379219055, -0.7708176970481873]
        self.putB1 = [2.276193618774414, -0.23610690236091614 + 0.0008, -1.9065214395523071, -1.0024445056915283,
                      0.7059170603752136, -0.7713210582733154]
        self.putB2 = [2.2135400772094727, -0.32462239265441895 + 0.0005, -1.7869548797607422, -1.0342985391616821,
                      0.6432875394821167, -0.770530104637146]
        self.putC1 = [1.606718897819519, -0.7031920552253723 - 0.0006, -1.1894333362579346, -1.2490800619125366,
                      1.5409659147262573, -0.7810162901878357]
        self.putC2 = [1.604477882385254, -0.8198584914207458 - 0.0007, -0.9826695322990417, -1.3392730951309204,
                      1.538760781288147, -0.7807766199111938]

        self.pose_aruco2 = []


class centrifuge(object):
    def __init__(self):
        # 旋转矩阵
        self.key_mid = np.array([[0.88812738, -0.45794297, -0.03896145, 0.06287588],
                                 [0.41193665, 0.83075479, -0.37437238, 0.22213659],
                                 [0.20380861, 0.31644071, 0.92645957, 0.04626311],
                                 [0, 0, 0, 1]])  # 开盖
        self.key_open = np.array([[0.88826375, -0.45770228, -0.0386799, 0.06150703],
                                  [0.41176459, 0.83076436, -0.37454038, 0.18800396],
                                  [0.20356187, 0.31676363, 0.92640346, 0.12834574],
                                  [0., 0., 0., 1.]])
        self.key_start = np.array([[ 0.88828263 ,-0.45767131, -0.03861263 , 0.04077991],
                                    [ 0.4118243 ,  0.83087261 ,-0.3742345  , 0.19037508],
                                    [ 0.20335857 , 0.31652438 , 0.92652987 , 0.12419927],
                                    [ 0.     ,     0.      ,    0.       ,   1.        ]])  # 启动离心机

        self.key_start_mid=np.array([[ 0.88828012, -0.45767738 ,-0.03859853 , 0.04392342],
                                    [ 0.41183391 , 0.83086667 ,-0.37423711 , 0.22076065],
                                    [ 0.20335009 , 0.3165312  , 0.92652941  ,0.04898253],
                                    [ 0.     ,     0.       ,   0.      ,    1.        ]])
        self.axis = np.array([[0.00521612, - 0.9999716, - 0.00543908, - 0.09862462],
                              [0.99987261, 0.0051334, 0.01511309, - 0.17843551],
                              [-0.01508474, - 0.00551722, 0.999871, 0.13328981],
                              [0., 0., 0., 1.]])  # 转子轴心

        # 取试管点
        self.getA1 = [[0.54681881, -0.72902678, 0.41171488, -0.2117069],
                      [0.83680231, 0.49197752, -0.2402499, -0.10816282],
                      [-0.02740585, 0.47589713, 0.87907385, 0.09669075],
                      [0., 0., 0., 1.]]
        self.getA2 = [[0.98422844, -0.17686438, 0.00365717, -0.09989824],
                      [0.15307185, 0.84109892, -0.51876933, -0.03113121],
                      [0.08867577, 0.51114733, 0.85490643, 0.10509282],
                      [0., 0., 0., 1.]]
        self.getA3 = [[0.43028277, 0.78788146, -0.4405673, 0.02186218],
                      [-0.90173569, 0.35267199, -0.24999044, -0.10778585],
                      [-0.04158708, 0.50484184, 0.86220951, 0.1029053],
                      [0., 0., 0., 1.]]
        self.getB1 = [[-5.14426537e-01, -4.53653699e-01, -7.27711248e-01, 1.98808449e-01],
                      [8.57032445e-01, -3.01015889e-01, -4.18192328e-01, -8.39373094e-04],
                      [-2.93381517e-02, -8.38801381e-01, 5.43646500e-01, 1.85040799e-01],
                      [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]
        self.getB2 = [[-9.99939975e-01, 6.07346561e-04, -1.09397501e-02, -9.68663093e-02],
                      [9.03484613e-03, -5.19134804e-01, -8.54644620e-01, 1.78197944e-01],
                      [-6.19827048e-03, -8.54692159e-01, 5.19098155e-01, 1.93464804e-01],
                      [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]
        self.getB3 = [[-0.43805703, 0.48811838, 0.75488177, -0.41103515],
                      [-0.89888767, -0.2475069, -0.36158165, -0.02345966],
                      [0.0103438, -0.8369473, 0.54718574, 0.18644897],
                      [0., 0., 0., 1.]]
        # 放试管点
        self.putA1 = [[0.54682881, -0.72902992, 0.41169603, -0.25349789],
                      [0.83679598, 0.49198788, -0.24025072, -0.08377766],
                      [-0.0273995, 0.4758816, 0.87908246, 0.00746794],
                      [0., 0., 0., 1.]]
        self.putA2 = [[0.98422265, -0.1768969, 0.00364037, -0.1002888],
                      [0.15308461, 0.84105825, -0.5188315, 0.02665626],
                      [0.08871792, 0.511203, 0.85486877, 0.00987585],
                      [0., 0., 0., 1.]]
        self.putA3 = [[0.43029389, 0.78786976, -0.44057736, 0.07009138],
                      [-0.90173077, 0.35268222, -0.24999375, -0.08042695],
                      [-0.04157871, 0.50485295, 0.8622034, 0.00853321],
                      [0., 0., 0., 1.]]
        self.putB1 = [[-0.51442255, -0.45366181, -0.72770901, 0.05361048],
                      [0.85703541, -0.30100133, -0.41819674, -0.09071073],
                      [-0.02932149, -0.83880222, 0.5436461, 0.10548989],
                      [0., 0., 0., 1.]]
        self.putB2 = [[-9.99939483e-01, 5.93995552e-04, -1.09853431e-02, -9.79013150e-02],
                      [9.08031159e-03, -5.19194416e-01, -8.54607926e-01, 1.18587039e-02],
                      [-6.21116208e-03, -8.54655957e-01, 5.19157602e-01, 1.12345144e-01],
                      [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]
        self.putB3 = [[-0.43806353, 0.48807144, 0.75490835, -0.25783287],
                      [-0.89888309, -0.24760279, -0.36152739, -0.09902313],
                      [0.01046622, -0.83694631, 0.54718491, 0.10802555],
                      [0., 0., 0., 1.]]

        # 中间点
        self.put_middleA = []
        self.get_middleA = []
        self.get_middleB = []
        self.put_middleB = np.array([[-0.99842942, -0.01541606, -0.05386133, -0.10597664],
                                     [0.05410575, -0.01590283, -0.99840857, 0.18961635],
                                     [0.01453498, -0.99975469, 0.01671195, 0.07029574],
                                     [0., 0., 0., 1.]])
        # 关门点
        self.close_door0 = [[0.9999243, -0.0049054, 0.01128388, -0.14780709],
                            [0.01054562, 0.81413732, -0.58057662, 0.01879157],
                            [-0.00633867, 0.58065167, 0.81412742, -0.29472134],
                            [0., 0., 0., 1.]]
        self.close_door1 = np.array([[0.99992339, -0.00493678, 0.0113512, -0.14286774],
                                     [0.01061016, 0.8141464, -0.58056271, -0.14728529],
                                     [-0.00637543, 0.58063867, 0.81413641, -0.29225329],
                                     [0., 0., 0., 1.]])
        self.close_door3 = np.array([[0.99959752, -0.02304766, 0.01654071, -0.14592057],
                                     [0.022821, 0.99964482, 0.01376344, -0.14525979],
                                     [-0.01685205, -0.01338042, 0.99976846, -0.13248836],
                                     [0., 0., 0., 1.]])
        # 看aruco点
        self.watch_aruco_pose = np.array([[9.99986468e-01, 5.18479587e-03, -4.26286242e-04, -1.04633858e-01],
                                          [-5.18581144e-03, 9.99983633e-01, -2.41682813e-03, -7.53697580e-02],
                                          [4.13748504e-04, 2.41900606e-03, 9.99996989e-01, 1.34617099e-01],
                                          [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
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

        duco_cobot.tcp_move([0, 0, -0.1, 0, 0, 0], 0.3, 0.2, 0, True)
        robotiq(1, 1, 255, 0, 100, 1)
        aruco_way = [0.2035699039697647, 0.17634217441082, -1.4309754371643066, -0.3029015362262726,
                     1.5594215393066406, -0.573725163936615]
        duco_cobot.movej(aruco_way, speedj, 10, 0, True)
        # 定位
        sleep(0.5)
        aruco.mark()
        sleep(0.5)
        # 计算示教点
        self.pose_aruco2 = duco_cobot.get_tcp_pose()
        self.key_mid = get_t(self.pose_aruco2, self.key_mid)
        self.key_open = get_t(self.pose_aruco2, self.key_open)
        #启动按键
        self.key_start=get_t(self.pose_aruco2, self.key_start)
        self.key_start_mid=get_t(self.pose_aruco2, self.key_start_mid)

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
        ret = duco_cobot.movel(self.pose_aruco2, speedl, 0.2, 0, [], "", "", True)
        self.pose_aruco2[2] -= 0.05

    def open(self):

        key(self.key_mid, self.key_open, -20)
        print("盖子已打开")

    def start(self):

        key(self.key_start_mid, self.key_start, -20)
        print("离心机已启动")

    def put(self, test):

        # movel
        duco_cobot.movel(self.watch_aruco_pose, speedl - 0.1, 0.2, 0, [], "", "", True)
        self.watch_aruco_pose[2] -= 0.25
        duco_cobot.movel(self.watch_aruco_pose, speedl - 0.1, 0.2, 0, [], "", "", True)
        # 求旋转角度
        mark_size(0.0278)
        mark2camera = aruco.test_mark()
        # 计算旋转角度
        diff = mark2camera[2] - self.mark2camera[2]
        self.watch_aruco_pose[2] += 0.25
        duco_cobot.movel(self.watch_aruco_pose, speedl - 0.1, 0.2, 0, [], "", "", True)
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
        motor.motor_position(183000, 10)
        motor.motor_position(179000, 0)
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
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, -25, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, -dist, 0, 0, 0], [0, 0, 0.02, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, -15, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        # csvflag = 0
        sleep(1)
        # csvflag = 1
        if abs(duco_cobot.get_tcp_pose()[2] - ponit_pose[2]) >= dist - 0.05:
            robotiq(1, 1, 150, 0, 100, 1)
            print("试管放入成功")
            ret = duco_cobot.movel(ponit_pose, speedl - 0.1, 0.2, 0, [], "", "", True)
            print("返回放置点", ret)
            ret = duco_cobot.movel(middle, speedl - 0.1, 0.2, 0, [], "", "", True)
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
        duco_cobot.fc_config([False, True, False, False, False, False], [0, 20, 0, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, -dist, 0, 0, 0], [0, 0, 0.02, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 10, 0, 0, 0, 0], [0, 2, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        sleep(1)

        if abs(duco_cobot.get_flange_pose()[2] - ponit_pose[2]) >= dist - 0.05:
            robotiq(1, 1, 175, 0, 100, 1)
            print("试管放入成功")
            ret = duco_cobot.movel(ponit_pose, speedl - 0.1, 0.2, 0, [], "", "", True)
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
        duco_cobot.movel(self.watch_aruco_pose, speedl - 0.1, 0.2, 0, [], "", "", True)
        self.watch_aruco_pose[2] -= 0.25
        duco_cobot.movel(self.watch_aruco_pose, speedl - 0.1, 0.2, 0, [], "", "", True)
        # 求旋转角度
        mark_size(0.0278)
        sleep(0.5)
        mark2camera = aruco.test_mark()
        # 计算旋转角度
        diff = mark2camera[2] - self.mark2camera[2]
        self.watch_aruco_pose[2] += 0.25
        duco_cobot.movel(self.watch_aruco_pose, speedl - 0.1, 0.2, 0, [], "", "", True)
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
            tube.put_tubeA(tube.tube_putA3, )
            print("A半圆取完，取B半圆")
            # 取B半圆
            duco_cobot.movej(tube.tube_middleB_j1, speedj, 10, 0, True)
            duco_cobot.movej(tube.tube_middleB_j2, speedj, 10, 0, True)
            self.get_tubeB(self.getB1, self.get_middleB, theta)
            tube.put_tubeB(tube.tube_putB1)
            self.get_tubeB(self.getB2, self.get_middleB, theta)
            tube.put_tubeB(tube.tube_putB2)
            self.get_tubeB(self.getB3, self.get_middleB, theta)
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
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, -20, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        # duco_cobot.fc_wait_pos([0, 0,-dist_q, 0, 0, 0], [0, 0, 0.001, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_pos([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, -10, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000)
        # duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_wait_logic([0, 0, 1])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        sleep(0.5)
        # 上升一点点，夹住试管
        ret = duco_cobot.tcp_move([0, 0, -0.003, 0, 0, 0], 0.05, 0.05, 0, True)
        print("move", ret)
        test_pose1 = duco_cobot.get_tcp_pose()
        offset = test_pose[2] - test_pose1[2]
        print("放试管的力控的位置判断距离", offset)
        # 夹爪闭合
        robotiq(1, 1, 250, 0, 100, 150)
        curpose = duco_cobot.get_tcp_pose()
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, 25, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, dist, 0, 0, 0], [0, 0, 0.02, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, 20, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000)
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

    def get_tubeB(self, edge, middle, angle):
        dist = 0.14
        # 从A点旋转到位置0
        target = circle(self.axis, edge, angle)
        duco_cobot.movel(self.get_middleB, speedl, 0.2, 0, [], "", "", True)
        duco_cobot.movel(target, 0.2, 0.1, 0, [], "", "", True)
        robotiq(1, 1, 170, 0, 10, 1)
        duco_cobot.tcp_move([0, 0, 0.1, 0, 0, 0], 0.05, 0.05, 0, True)
        # 夹爪闭合
        robotiq(1, 1, 250, 0, 200, 150)
        curpose = duco_cobot.get_tcp_pose()
        duco_cobot.fc_config([False, True, False, False, False, False], [0, -20, 0, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, dist, 0, 0, 0], [0, 0, 0.02, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, -20, 0, 0, 0, 0], [0, 2, 0, 0, 0, 0], False, 0, 500000)
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
        duco_cobot.movel(self.close_door0, speedl - 0.1, 0.2, 0, [], "", "", True)
        duco_cobot.movel(self.close_door1, speedl - 0.2, 0.2, 0, [], "", "", True)
        duco_cobot.tcp_move([0, 0, 0.08, 0, 0, 0], 0.3, 0.2, 0, True)
        duco_cobot.movel(self.close_door3, 0.1, 0.1, 0, [], "", "", True)
        # 向下按压
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, -40, 0, 0, 0],
                             [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, -20, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([0, 0, 1])
        duco_cobot.fc_move()
        print("move stop")
        sleep(1)
        # 关上盖子
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, -210, 0, 0, 0],
                             [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_wait_pos([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, -200, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([0, 0, 1])
        duco_cobot.fc_move()
        print("关上盖子")
        sleep(1)
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, 20, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_wait_pos([0, 0, 0.02, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, 10, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 0])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        duco_cobot.tcp_move([0, 0, -0.13, 0, 0, 0], 0.3, 0.2, 0, True)

    def again(self):
        # 旋转矩阵
        self.key_mid = np.array([[0.88812738, -0.45794297, -0.03896145, 0.06287588],
                                 [0.41193665, 0.83075479, -0.37437238, 0.22213659],
                                 [0.20380861, 0.31644071, 0.92645957, 0.04626311],
                                 [0, 0, 0, 1]])  # 开盖
        self.key_open = np.array([[0.88826375, -0.45770228, -0.0386799, 0.06150703],
                                  [0.41176459, 0.83076436, -0.37454038, 0.18800396],
                                  [0.20356187, 0.31676363, 0.92640346, 0.12834574],
                                  [0., 0., 0., 1.]])
        self.key_start = np.array([[ 0.88828263 ,-0.45767131, -0.03861263 , 0.04077991],
                                    [ 0.4118243 ,  0.83087261 ,-0.3742345  , 0.19037508],
                                    [ 0.20335857 , 0.31652438 , 0.92652987 , 0.12419927],
                                    [ 0.     ,     0.      ,    0.       ,   1.        ]])  # 启动离心机

        self.key_start_mid=np.array([[ 0.88828012, -0.45767738 ,-0.03859853 , 0.04392342],
                                    [ 0.41183391 , 0.83086667 ,-0.37423711 , 0.22076065],
                                    [ 0.20335009 , 0.3165312  , 0.92652941  ,0.04898253],
                                    [ 0.     ,     0.       ,   0.      ,    1.        ]])  # 启动离心机

        self.axis = np.array([[0.00521612, - 0.9999716, - 0.00543908, - 0.09862462],
                              [0.99987261, 0.0051334, 0.01511309, - 0.17843551],
                              [-0.01508474, - 0.00551722, 0.999871, 0.13328981],
                              [0., 0., 0., 1.]])  # 转子轴心

        # 取试管点
        self.getA1 = [[0.54681881, -0.72902678, 0.41171488, -0.2117069],
                      [0.83680231, 0.49197752, -0.2402499, -0.10816282],
                      [-0.02740585, 0.47589713, 0.87907385, 0.09669075],
                      [0., 0., 0., 1.]]
        self.getA2 = [[0.98422844, -0.17686438, 0.00365717, -0.09989824],
                      [0.15307185, 0.84109892, -0.51876933, -0.03113121],
                      [0.08867577, 0.51114733, 0.85490643, 0.10509282],
                      [0., 0., 0., 1.]]
        self.getA3 = [[0.43028277, 0.78788146, -0.4405673, 0.02186218],
                      [-0.90173569, 0.35267199, -0.24999044, -0.10778585],
                      [-0.04158708, 0.50484184, 0.86220951, 0.1029053],
                      [0., 0., 0., 1.]]
        self.getB1 = [[-5.14426537e-01, -4.53653699e-01, -7.27711248e-01, 1.98808449e-01],
                      [8.57032445e-01, -3.01015889e-01, -4.18192328e-01, -8.39373094e-04],
                      [-2.93381517e-02, -8.38801381e-01, 5.43646500e-01, 1.85040799e-01],
                      [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]
        self.getB2 = [[-9.99939975e-01, 6.07346561e-04, -1.09397501e-02, -9.68663093e-02],
                      [9.03484613e-03, -5.19134804e-01, -8.54644620e-01, 1.78197944e-01],
                      [-6.19827048e-03, -8.54692159e-01, 5.19098155e-01, 1.93464804e-01],
                      [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]
        self.getB3 = [[-0.43805703, 0.48811838, 0.75488177, -0.41103515],
                      [-0.89888767, -0.2475069, -0.36158165, -0.02345966],
                      [0.0103438, -0.8369473, 0.54718574, 0.18644897],
                      [0., 0., 0., 1.]]
        # 放试管点
        self.putA1 = [[0.54682881, -0.72902992, 0.41169603, -0.25349789],
                      [0.83679598, 0.49198788, -0.24025072, -0.08377766],
                      [-0.0273995, 0.4758816, 0.87908246, 0.00746794],
                      [0., 0., 0., 1.]]
        self.putA2 = [[0.98422265, -0.1768969, 0.00364037, -0.1002888],
                      [0.15308461, 0.84105825, -0.5188315, 0.02665626],
                      [0.08871792, 0.511203, 0.85486877, 0.00987585],
                      [0., 0., 0., 1.]]
        self.putA3 = [[0.43029389, 0.78786976, -0.44057736, 0.07009138],
                      [-0.90173077, 0.35268222, -0.24999375, -0.08042695],
                      [-0.04157871, 0.50485295, 0.8622034, 0.00853321],
                      [0., 0., 0., 1.]]
        self.putB1 = [[-0.51442255, -0.45366181, -0.72770901, 0.05361048],
                      [0.85703541, -0.30100133, -0.41819674, -0.09071073],
                      [-0.02932149, -0.83880222, 0.5436461, 0.10548989],
                      [0., 0., 0., 1.]]
        self.putB2 = [[-9.99939483e-01, 5.93995552e-04, -1.09853431e-02, -9.79013150e-02],
                      [9.08031159e-03, -5.19194416e-01, -8.54607926e-01, 1.18587039e-02],
                      [-6.21116208e-03, -8.54655957e-01, 5.19157602e-01, 1.12345144e-01],
                      [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]
        self.putB3 = [[-0.43806353, 0.48807144, 0.75490835, -0.25783287],
                      [-0.89888309, -0.24760279, -0.36152739, -0.09902313],
                      [0.01046622, -0.83694631, 0.54718491, 0.10802555],
                      [0., 0., 0., 1.]]

        # 中间点
        self.put_middleA = []
        self.get_middleA = []
        self.get_middleB = []
        self.put_middleB = np.array([[-0.99842942, -0.01541606, -0.05386133, -0.10597664],
                                     [0.05410575, -0.01590283, -0.99840857, 0.18961635],
                                     [0.01453498, -0.99975469, 0.01671195, 0.07029574],
                                     [0., 0., 0., 1.]])
        # 关门点
        self.close_door0 = [[0.9999243, -0.0049054, 0.01128388, -0.14780709],
                            [0.01054562, 0.81413732, -0.58057662, 0.01879157],
                            [-0.00633867, 0.58065167, 0.81412742, -0.29472134],
                            [0., 0., 0., 1.]]
        self.close_door1 = np.array([[0.99992339, -0.00493678, 0.0113512, -0.14286774],
                                     [0.01061016, 0.8141464, -0.58056271, -0.14728529],
                                     [-0.00637543, 0.58063867, 0.81413641, -0.29225329],
                                     [0., 0., 0., 1.]])
        self.close_door3 = np.array([[0.99959752, -0.02304766, 0.01654071, -0.14592057],
                                     [0.022821, 0.99964482, 0.01376344, -0.14525979],
                                     [-0.01685205, -0.01338042, 0.99976846, -0.13248836],
                                     [0., 0., 0., 1.]])
        # 看aruco点
        self.watch_aruco_pose = np.array([[9.99986468e-01, 5.18479587e-03, -4.26286242e-04, -1.04633858e-01],
                                          [-5.18581144e-03, 9.99983633e-01, -2.41682813e-03, -7.53697580e-02],
                                          [4.13748504e-04, 2.41900606e-03, 9.99996989e-01, 1.34617099e-01],
                                          [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        self.pose_aruco2 = []
        self.mark2camera = []


class seperation(object):
    def __init__(self):
        # 倾倒点
        self.middle = [2.108090877532959, -0.007148286327719688, -1.6144897937774658, -1.515860676765442,
                       0.548012912273407, 2.3568851947784424]
        # 倾倒取试管
        self.getA1 = [0.2264304757118225, -0.5757653713226318 - 0.0003, 0.19310222566127777, -1.5707921981811523,
                      -2.4697665139683522e-05, 1.5622894763946533]
        self.getA2 = [-0.158240407705307, -0.34018659591674805, 0.1985221654176712, -1.5707848072052002,
                      -2.0720669908769196e-06, -2.8423726558685303]
        self.getA3 = [-0.2237684279680252, -0.34005701541900635, 0.1985214352607727, -1.570785641670227,
                      -6.666150966339046e-06, -2.84234881401062]
        self.getB1 = [0.22605517506599426, -0.6407598257064819 - 0.0002, 0.1931076943874359, -1.5707927942276,
                      -2.5389628717675805e-05, 1.5622886419296265]
        self.getB2 = [0.002634721575304866, -0.8690693378448486 - 0.0003, 0.19239483773708344, -1.5724000930786133,
                      0.0002806083357427269, 0.36598125100135803]
        self.getB3 = [-0.06238335371017456, -0.869005024433136, 0.19238193333148956, -1.5723758935928345,
                      0.00028370501240715384, 0.36598101258277893]
        # 倾倒放试管
        self.putA1_j = [2.349848747253418, -0.29376301169395447, -1.8532036542892456, -0.9978065490722656,
                        0.7880449891090393, 2.360708236694336]
        self.putA2_j = [1.450636386871338, 0.4052288830280304, -2.4930293560028076, -1.0543361902236938,
                        -1.9895987510681152, 2.354236602783203]
        self.putA3_j = [1.2207189798355103, 0.28694459795951843, -2.425126791000366, -1.004805326461792,
                        -2.219588041305542, 2.3537333011627197]
        self.putB1_j = [2.284390926361084, -0.3769814670085907, -1.7352548837661743, -1.0331840515136719,
                        0.722611129283905, 2.3614392280578613]
        self.putB2_j = [1.782443642616272, -0.7671159505844116, -1.0987367630004883, -1.2747859954833984,
                        1.417192816734314, 2.3608040809631348]
        self.putB3_j = [1.7160149812698364, -0.7656897902488708, -1.101097583770752, -1.2740429639816284,
                        1.350776195526123, 2.3607561588287354]
        # 倾倒路点
        self.throw_point1 = [0.9639929533004761, -0.26671460270881653, -1.3903968334197998, -1.5031213760375977,
                             -0.6077816486358643, 2.361870765686035]
        # self.throw_point2 = [0.9388021230697632, -0.38902559876441956, -1.7746111154556274, -0.9733467698097229, -0.6390125751495361, 2.350449800491333]
        self.waypoint2_pose = [[0.99999386, -0.00321306, 0.00139786, -0.49939987],
                               [0.00145448, 0.01768805, -0.9998425, -0.05586221],
                               [0.00318783, 0.99983839, 0.01769261, 0.54037154],
                               [0., 0., 0., 1.]]

    def point(self):
        duco_cobot.movej(midpose, speedj, 20, 0, True)
        print("开始固液分离操作")
        mark_size(0.150)
        duco_cobot.tcp_move([0, 0, -0.1, 0, 0, 0], 0.3, 0.2, 0, True)
        robotiq(1, 1, 255, 0, 100, 1)
        duco_cobot.movej([0.2035699039697647, 0.17634217441082, -1.4309754371643066, -0.3029015362262726,
                          1.5594215393066406, -0.573725163936615], speedj, 10, 0, True)
        # 定位
        sleep(0.5)
        aruco.mark()
        sleep(0.5)
        # 计算示教点
        self.pose_aruco2 = duco_cobot.get_tcp_pose()
        self.waypoint2_pose = get_t(self.pose_aruco2, self.waypoint2_pose)
        duco_cobot.movej(midpose, speedj, 20, 0, True)

    def get(self, putpose_j, getpose, num):
        duco_cobot.movej(self.middle, speedj, 10, 0, True)
        if num == 0:
            robotiq(1, 1, 130, 0, 100, 75)
        duco_cobot.movej(putpose_j, speedj, 10, 0, True)
        motor.motor_position(183000, 5)
        motor.motor_position(177000, 0)
        duco_cobot.movel(getpose, speedl - 0.3, 0.2, 0, [], "", "", True)
        # 力控夹起试管
        robotiq(1, 1, 250, 0, 100, 75)
        curpose = duco_cobot.get_tcp_pose()
        duco_cobot.fc_config([False, True, False, False, False, False], [0, 25, 0, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0.12, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 10, 0, 0, 0, 0], [0, 2, 0, 0, 0, 0], False, 0, 500000)
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
        motor.motor_position(182800, 5)
        motor.motor_position(177000, 0)
        duco_cobot.fc_config([False, True, False, False, False, False], [0, -30, 0, 0, 0, 0],
                             [1000, 2000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, -0.11, 0, 0, 0], [0, 0, 0.005, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, -25, 0, 0, 0, 0], [0, 2, 0, 0, 0, 0], False, 0, 500000)
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

    def throw_single(self, angle):
        duco_cobot.movej(self.throw_point1, speedj, 10, 0, True)
        duco_cobot.movel(self.waypoint2_pose, speedl - 0.1, 0.2, 0, [], "", "", True)
        throw_joint = duco_cobot.get_actual_joints_position()
        throw_joint[5] -= angle * math.pi / 180
        duco_cobot.movej(throw_joint, 20, 10, 0, True)
        sleep(3)
        throw_joint[5] += angle * math.pi / 180
        duco_cobot.movej(throw_joint, 30, 10, 0, True)
        duco_cobot.movej(self.throw_point1, speedj, 10, 0, True)

    def throw(self, angle, test):
        if test == 0:
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
        duco_cobot.movej(self.middle, speedj, 10, 0, True)
        robotiq(1, 1, 250, 0, 100, 75)
        duco_cobot.movej(midpose, speedj, 10, 0, True)

    def again(self):
        # 倾倒点
        self.middle = [2.108090877532959, -0.007148286327719688, -1.6144897937774658, -1.515860676765442,
                       0.548012912273407, 2.3568851947784424]
        # 倾倒取试管
        self.getA1 = [0.2264304757118225, -0.5757653713226318 - 0.0003, 0.19310222566127777, -1.5707921981811523,
                      -2.4697665139683522e-05, 1.5622894763946533]
        self.getA2 = [-0.158240407705307, -0.34018659591674805, 0.1985221654176712, -1.5707848072052002,
                      -2.0720669908769196e-06, -2.8423726558685303]
        self.getA3 = [-0.2237684279680252, -0.34005701541900635, 0.1985214352607727, -1.570785641670227,
                      -6.666150966339046e-06, -2.84234881401062]
        self.getB1 = [0.22605517506599426, -0.6407598257064819 - 0.0002, 0.1931076943874359, -1.5707927942276,
                      -2.5389628717675805e-05, 1.5622886419296265]
        self.getB2 = [0.002634721575304866, -0.8690693378448486 - 0.0003, 0.19239483773708344, -1.5724000930786133,
                      0.0002806083357427269, 0.36598125100135803]
        self.getB3 = [-0.06238335371017456, -0.869005024433136, 0.19238193333148956, -1.5723758935928345,
                      0.00028370501240715384, 0.36598101258277893]
        # 倾倒放试管
        self.putA1_j = [2.349848747253418, -0.29376301169395447, -1.8532036542892456, -0.9978065490722656,
                        0.7880449891090393, 2.360708236694336]
        self.putA2_j = [1.450636386871338, 0.4052288830280304, -2.4930293560028076, -1.0543361902236938,
                        -1.9895987510681152, 2.354236602783203]
        self.putA3_j = [1.2207189798355103, 0.28694459795951843, -2.425126791000366, -1.004805326461792,
                        -2.219588041305542, 2.3537333011627197]
        self.putB1_j = [2.284390926361084, -0.3769814670085907, -1.7352548837661743, -1.0331840515136719,
                        0.722611129283905, 2.3614392280578613]
        self.putB2_j = [1.782443642616272, -0.7671159505844116, -1.0987367630004883, -1.2747859954833984,
                        1.417192816734314, 2.3608040809631348]
        self.putB3_j = [1.7160149812698364, -0.7656897902488708, -1.101097583770752, -1.2740429639816284,
                        1.350776195526123, 2.3607561588287354]
        # 倾倒路点
        self.throw_point1 = [0.9639929533004761, -0.26671460270881653, -1.3903968334197998, -1.5031213760375977,
                             -0.6077816486358643, 2.361870765686035]
        # self.throw_point2 = [0.9388021230697632, -0.38902559876441956, -1.7746111154556274, -0.9733467698097229, -0.6390125751495361, 2.350449800491333]
        self.waypoint2_pose = [[0.99999386, -0.00321306, 0.00139786, -0.49939987],
                               [0.00145448, 0.01768805, -0.9998425, -0.05586221],
                               [0.00318783, 0.99983839, 0.01769261, 0.54037154],
                               [0., 0., 0., 1.]]


class drying(object):
    def __init__(self):
        # 参数
        self.handle_offset = 0.1  # 路点到门把手距离
        # 过渡矩阵
        self.openpower = np.array([[0.9984479, 0.00925176, 0.05491991, -0.64168806],
                                   [-0.00903779, 0.99995058, -0.0041431, 0.33587667],
                                   [-0.05495553, 0.00364032, 0.99848217, 0.05920001],
                                   [0., 0., 0., 1.]])
        self.handle = np.array([[0.99967743, 0.01586798, 0.01983022, -0.53712757],
                                [-0.01583831, 0.9998732, -0.0016527, 0.17841741],
                                [-0.01985393, 0.00133809, 0.999802, -0.04919224],
                                [0., 0., 0., 1.]])
        self.open_waypoint1 = np.array([[0.73507608, 0.57945417, 0.35198867, -0.66300158],
                                        [-0.66570147, 0.51848966, 0.53666566, 0.02048671],
                                        [0.12847067, -0.62880946, 0.76687284, 0.060322],
                                        [0., 0., 0., 1.]])
        self.open_waypoint2 = np.array([[0.73506929, 0.57946094, 0.3519917, -0.51573811],
                                        [-0.66570687, 0.51847575, 0.5366724, 0.0295821],
                                        [0.12848153, -0.62881469, 0.76686673, -0.24252797],
                                        [0., 0., 0., 1.]])
        self.open_waypoint3 = np.array([[0.87617755, 0.47620137, -0.07446584, -0.67929367],
                                        [0.02228718, 0.11430381, 0.99319581, -0.30597807],
                                        [0.48147294, -0.8718755, 0.08953725, 0.00769625],
                                        [0., 0., 0., 1.]])
        self.open_waypoint4 = np.array([[0.86786429, 0.49100267, -0.07568326, -0.39439769],
                                        [0.02223934, 0.11379185, 0.99325567, -0.34387848],
                                        [0.49630332, -0.86369427, 0.08783631, 0.12079046],
                                        [0., 0., 0., 1.]])
        self.open_waypoint5 = np.array([[0.85845333, 0.51284122, 0.00719491, 0.23259358],
                                        [-0.00937571, 0.00166529, 0.99995466, -0.36989539],
                                        [0.51280599, -0.85848186, 0.00623783, -0.22511512],
                                        [0., 0., 0., 1.]])
        self.close_waypoint0 = [[0.71237156, -0.58417015, 0.388937, 0.15358778],
                                [0.08451361, 0.62157144, 0.77878521, -0.44990721],
                                [-0.6966952, -0.52191397, 0.49216014, -0.34738044],
                                [0., 0., 0., 1.]]
        self.close_waypoint1 = np.array([[0.71237551, -0.58416324, 0.38894014, 0.29495323],
                                         [0.08450973, 0.62157644, 0.77878164, -0.4514387],
                                         [-0.69669164, -0.52191574, 0.49216331, -0.34467472],
                                         [0., 0., 0., 1.]])
        self.close_waypoint2 = np.array([[0.66371594, -0.56251054, 0.49301425, -0.39581818],
                                         [-0.05489129, 0.62071669, 0.78211107, -0.39202424],
                                         [-0.7459679, -0.54616177, 0.38110263, -0.10705829],
                                         [0., 0., 0., 1.]])
        self.close_waypoint3 = np.array([[0.99976921, 0.00516625, -0.02085283, -0.42428608],
                                         [-0.00505706, 0.99997325, 0.00528524, -0.15127851],
                                         [0.02087957, -0.00517856, 0.99976859, -0.04807866],
                                         [0., 0., 0., 1.]])

        # 放试管架
        self.waypoint1_j = [2.4160375595092773, -0.41018974781036377, -1.6827280521392822, -1.0518676042556763,
                            0.8470672965049744, 0.7902474403381348]
        # self.jia_j = [2.4158217906951904, -0.5467020869255066, -1.7657907009124756, -0.8321366310119629, 0.8460485935211182, 0.790019690990448]
        self.jia_j = [2.4158217906951904, -0.5467020869255066, -1.7657907009124756, -0.8321366310119629,
                      0.8482015517616272, 0.802851456]
        self.jia_move = [0.2239157110452652, -0.5690872073173523, 0.1765543669462204, -2.3725831508636475,
                         -1.5707728862762451, 2.3714568614959717]

        self.waypoint2_j = [2.3617489337921143, -0.2606026530265808, -1.882265329360962, -1.00188148021698,
                            0.7927427887916565, 0.7899118661880493]
        self.waypoint3_j = [0.8695452809333801, 0.6958703398704529, -2.1559131145477295, -1.6796133518218994,
                            -0.6991494297981262, 0.7828651070594788]
        self.jia_close = [0.2792383134365082, 0.054019197821617126, -1.1718405485153198, -0.4493247866630554,
                          1.5603203773498535, -0.6294997334480286]
        self.tubejia_getp = []

        self.watch_aruco_j = []
        self.pose_aruco2 = []
        self.jia_put = []
        self.jia_fang = [[-0.01590308, 0.99936596, 0.03185555, -0.21592303],
                         [-0.99987128, -0.0159627, 0.00161822, 0.15920248],
                         [0.00212569, -0.03182571, 0.99949117, 0.26181279],
                         [0., 0., 0., 1.]]

    def point(self):
        duco_cobot.movej(midpose, speedj, 20, 0, True)
        robotiq(1, 1, 255, 0, 100, 1)
        print("开始烘干机操作")
        mark_size(0.150)
        # 移动至识别aruco姿态
        duco_cobot.movej([-0.1358712762594223, 0.41367778182029724, -2.256640672683716, 1.8141716718673706,
                          1.7438228130340576, -0.7815196514129639], speedj, 10, 0, True)
        self.watch_aruco_j = [-0.05049566552042961, 0.15356017649173737, -2.0894246101379395, 1.9346251487731934,
                              1.6355094909667969, -0.7722198963165283]
        duco_cobot.movej(self.watch_aruco_j, speedj - 30, 10, 0, True)
        sleep(1)
        # 定位
        aruco.mark()
        sleep(1)
        # 计算示教点
        self.pose_aruco2 = duco_cobot.get_tcp_pose()
        self.openpower = get_t(self.pose_aruco2, self.openpower)
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
        duco_cobot.tcp_move([0, 0, self.handle_offset, 0, 0, 0], 0.2, 0.2, 0, True)
        curp = duco_cobot.get_tcp_pose()
        print("门把手点", curp)
        # 合并夹爪，抓住门把手
        robotiq(1, 1, 250, 0, 100, 250)
        sleep(1)
        # 力控开门把手
        duco_cobot.fc_config([True, True, True, False, False, False], [0, 10, 90, 0, 0, 0],
                             [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0.06, 0, 0, 0, 0, 0], [0.01, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, 200, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        sleep(0.5)
        # 张开夹爪
        duco_cobot.fc_config([True, True, True, False, False, False], [0, 0, 0, 0, 0, 0],
                             [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        robotiq(1, 1, 0, 0, 100, 10)
        # sleep(1)
        duco_cobot.fc_stop()
        # 移动至个路点
        duco_cobot.tcp_move([0, 0, -0.1, 0, 0, 0], 0.2, 0.2, 0, True)
        robotiq(1, 1, 255, 0, 100, 10)
        duco_cobot.movel(self.open_waypoint1, speedl - 0.1, 0.2, 0, [], "", "", True)
        duco_cobot.movel(self.open_waypoint2, speedl - 0.1, 0.2, 0, [], "", "", True)
        duco_cobot.tcp_move([0, 0, -0.12, 0, 0, 0], 0.3, 0.2, 0, True)
        duco_cobot.movel(self.open_waypoint3, speedl - 0.1, 0.2, 0, [], "", "", True)
        duco_cobot.movel(self.open_waypoint4, speedl - 0.1, 0.2, 0, [], "", "", True)
        duco_cobot.movel(self.open_waypoint5, 0.3, 0.2, 0, [], "", "", True)
        print("烘干机已开门")

    def close_door(self):
        print("烘干机准备关门")
        # robotiq(1, 1, 255, 0, 100, 1)
        duco_cobot.movel(self.close_waypoint0, speedl - 0.1, 0.2, 0, [], "", "", True)
        duco_cobot.movel(self.close_waypoint1, speedl - 0.1, 0.2, 0, [], "", "", True)
        duco_cobot.tcp_move([0, 0, 0.07, 0, 0, 0], 0.3, 0.2, 0, True)
        duco_cobot.movel(self.close_waypoint2, 0.2, 0.2, 0, [], "", "", True)
        duco_cobot.movel(self.close_waypoint3, 0.1, 0.1, 0, [], "", "", True)
        # 力控关门
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, -65, 0, 0, 0],
                             [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, -60, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([0, 0, 1])
        duco_cobot.fc_move()
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, 20, 0, 0, 0],
                             [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_wait_pos([0.02, 0, 0, 0, 0, 0], [0.01, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 0])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        duco_cobot.tcp_move([0, 0, -0.13, 0, 0, 0], 0.4, 0.2, 0, True)
        # 移动至门把手
        self.handle[2] -= 0.005
        duco_cobot.movel(self.handle, speedl - 0.1, 0.2, 0, [], "", "", True)
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, -60, 0, 0, 0],
                             [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, -60, 0, 0, 0], [0, 0, 5, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([0, 0, 1])
        duco_cobot.fc_move()
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, 20, 0, 0, 0],
                             [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_wait_pos([0.02, 0, 0, 0, 0, 0], [0.01, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 0])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        duco_cobot.tcp_move([0, 0, -0.06, 0, 0, 0], 0.3, 0.2, 0, True)
        self.handle[2] += 0.005
        print("烘干机已关门")

    def open_power(self):
        print("烘干机准备打开电源")
        duco_cobot.movel(self.openpower, speedl - 0.2, 0.2, 0, [], "", "", True)
        # 力控关门
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, -25, 0, 0, 0],
                             [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, -20, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([0, 0, 1])
        duco_cobot.fc_move()
        sleep(1)
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, 20, 0, 0, 0],
                             [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_wait_pos([0.02, 0, 0, 0, 0, 0], [0.01, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 500000)
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
        sleep(1)
        motor.motor_position(0, 10)
        duco_cobot.movej(self.jia_j, speedj - 30, 10, 0, True)
        print('移动至jia_j')
        robotiq(1, 1, 170, 0, 100, 250)
        print("开启力控")
        curpose = duco_cobot.get_tcp_pose()
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, -20, 0, 0, 0],
                             [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([-0.104, 0, 0, 0, 0, 0], [0.005, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, -10, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        print('力控停止')
        curpose1 = duco_cobot.get_tcp_pose()
        if abs(curpose1[0] - curpose[0]) < 0.085:
            print("受力停止了，夹爪未完全进入")
            assert False
        robotiq(1, 1, 255, 0, 20, 200)

        duco_cobot.movel(self.jia_move, speedl - 0.2, 0.2, 0, [], "", "", True)
        print('movel移动至中心')
        sleep(1)
        duco_cobot.movej(self.waypoint2_j, speedj - 30, 10, 0, True)
        duco_cobot.movej(self.waypoint3_j, speedj - 10, 10, 0, True)
        duco_cobot.movel(self.jia_fang, speedl - 0.1, 0.2, 0, [], "", "", True)
        # duco_cobot.tcp_move([-0.03, 0, 0.36, 0, 0, 0], 0.2, 0.2, 0, True)
        curpose = duco_cobot.get_tcp_pose()
        print('烘干机放试管夹点', curpose)
        self.jia_put = duco_cobot.get_tcp_pose()
        print('向下放试管架')
        duco_cobot.fc_config([True, False, False, False, False, False], [20, 0, 0, 0, 0, 0],
                             [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([10, 0, 0, 0, 0, 0], [2, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([0, 0, 1])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        sleep(1)
        robotiq(1, 1, 165, 0, 100, 1)
        duco_cobot.tcp_move([0, 0, -0.11, 0, 0, 0], 0.05, 0.05, 0, True)
        self.tubejia_getp = duco_cobot.get_tcp_pose()
        duco_cobot.movej(self.waypoint3_j, speedj - 20, 10, 0, True)
        robotiq(1, 1, 255, 0, 100, 1)
        duco_cobot.movej(self.jia_close, speedj - 10, 10, 0, True)

    def tubejia_get(self):
        duco_cobot.movej(self.jia_close, speedj - 10, 10, 0, True)
        duco_cobot.movej(self.waypoint3_j, speedj - 20, 10, 0, True)
        robotiq(1, 1, 175, 0, 100, 1)
        duco_cobot.movel(self.tubejia_getp, speedl - 0.1, 0.2, 0, [], "", "", True)
        # 力控插入
        curpose = duco_cobot.get_tcp_pose()
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, -20, 0, 0, 0],
                             [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([-0.115, 0, 0, 0, 0, 0], [0.005, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, -12, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000)
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
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, 15, 0, 0, 0],
                             [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0.115, 0, 0, 0, 0, 0], [0.005, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, 10, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        curpose1 = duco_cobot.get_tcp_pose()
        if curpose1[0] - curpose[0] < 0.085:
            assert False
        # duco_cobot.tcp_move([0, 0, -0.11, 0, 0, 0], 0.05, 0.05, 0, True)

        motor.motor_position(179000, 30)
        robotiq(1, 1, 255, 0, 100, 1)
        curr_pose = duco_cobot.get_tcp_pose()
        curr_pose[2] += 0.1
        duco_cobot.movel(curr_pose, speedl - 0.1, 0.2, 0, [], "", "", True)
        duco_cobot.movej(self.waypoint2_j, speedj - 10, 10, 0, True)
        duco_cobot.movej(self.waypoint3_j, speedj, 10, 0, True)
        duco_cobot.movej(self.jia_close, speedj, 10, 0, True)

    def again(self):
        # 参数
        self.handle_offset = 0.1  # 路点到门把手距离
        # 过渡矩阵
        self.openpower = np.array([[0.9984479, 0.00925176, 0.05491991, -0.64168806],
                                   [-0.00903779, 0.99995058, -0.0041431, 0.33587667],
                                   [-0.05495553, 0.00364032, 0.99848217, 0.05920001],
                                   [0., 0., 0., 1.]])
        self.handle = np.array([[0.99967743, 0.01586798, 0.01983022, -0.53712757],
                                [-0.01583831, 0.9998732, -0.0016527, 0.17841741],
                                [-0.01985393, 0.00133809, 0.999802, -0.04919224],
                                [0., 0., 0., 1.]])
        self.open_waypoint1 = np.array([[0.73507608, 0.57945417, 0.35198867, -0.66300158],
                                        [-0.66570147, 0.51848966, 0.53666566, 0.02048671],
                                        [0.12847067, -0.62880946, 0.76687284, 0.060322],
                                        [0., 0., 0., 1.]])
        self.open_waypoint2 = np.array([[0.73506929, 0.57946094, 0.3519917, -0.51573811],
                                        [-0.66570687, 0.51847575, 0.5366724, 0.0295821],
                                        [0.12848153, -0.62881469, 0.76686673, -0.24252797],
                                        [0., 0., 0., 1.]])
        self.open_waypoint3 = np.array([[0.87617755, 0.47620137, -0.07446584, -0.67929367],
                                        [0.02228718, 0.11430381, 0.99319581, -0.30597807],
                                        [0.48147294, -0.8718755, 0.08953725, 0.00769625],
                                        [0., 0., 0., 1.]])
        self.open_waypoint4 = np.array([[0.86786429, 0.49100267, -0.07568326, -0.39439769],
                                        [0.02223934, 0.11379185, 0.99325567, -0.34387848],
                                        [0.49630332, -0.86369427, 0.08783631, 0.12079046],
                                        [0., 0., 0., 1.]])
        self.open_waypoint5 = np.array([[0.85845333, 0.51284122, 0.00719491, 0.23259358],
                                        [-0.00937571, 0.00166529, 0.99995466, -0.36989539],
                                        [0.51280599, -0.85848186, 0.00623783, -0.22511512],
                                        [0., 0., 0., 1.]])
        self.close_waypoint0 = [[0.71237156, -0.58417015, 0.388937, 0.15358778],
                                [0.08451361, 0.62157144, 0.77878521, -0.44990721],
                                [-0.6966952, -0.52191397, 0.49216014, -0.34738044],
                                [0., 0., 0., 1.]]
        self.close_waypoint1 = np.array([[0.71237551, -0.58416324, 0.38894014, 0.29495323],
                                         [0.08450973, 0.62157644, 0.77878164, -0.4514387],
                                         [-0.69669164, -0.52191574, 0.49216331, -0.34467472],
                                         [0., 0., 0., 1.]])
        self.close_waypoint2 = np.array([[0.66371594, -0.56251054, 0.49301425, -0.39581818],
                                         [-0.05489129, 0.62071669, 0.78211107, -0.39202424],
                                         [-0.7459679, -0.54616177, 0.38110263, -0.10705829],
                                         [0., 0., 0., 1.]])
        self.close_waypoint3 = np.array([[0.99976921, 0.00516625, -0.02085283, -0.42428608],
                                         [-0.00505706, 0.99997325, 0.00528524, -0.15127851],
                                         [0.02087957, -0.00517856, 0.99976859, -0.04807866],
                                         [0., 0., 0., 1.]])

        # 放试管架
        self.waypoint1_j = [2.4160375595092773, -0.41018974781036377, -1.6827280521392822, -1.0518676042556763,
                            0.8470672965049744, 0.7902474403381348]
        # self.jia_j = [2.4158217906951904, -0.5467020869255066, -1.7657907009124756, -0.8321366310119629, 0.8460485935211182, 0.790019690990448]
        self.jia_j = [2.4158217906951904, -0.5467020869255066, -1.7657907009124756, -0.8321366310119629,
                      0.8482015517616272, 0.802851456]
        self.jia_move = [0.2239157110452652, -0.5690872073173523, 0.1765543669462204, -2.3725831508636475,
                         -1.5707728862762451, 2.3714568614959717]

        self.waypoint2_j = [2.3617489337921143, -0.2606026530265808, -1.882265329360962, -1.00188148021698,
                            0.7927427887916565, 0.7899118661880493]
        self.waypoint3_j = [0.8695452809333801, 0.6958703398704529, -2.1559131145477295, -1.6796133518218994,
                            -0.6991494297981262, 0.7828651070594788]
        self.jia_close = [0.2792383134365082, 0.054019197821617126, -1.1718405485153198, -0.4493247866630554,
                          1.5603203773498535, -0.6294997334480286]
        self.tubejia_getp = []

        self.watch_aruco_j = []
        self.pose_aruco2 = []
        self.jia_put = []
        self.jia_fang = [[-0.01590308, 0.99936596, 0.03185555, -0.21592303],
                         [-0.99987128, -0.0159627, 0.00161822, 0.15920248],
                         [0.00212569, -0.03182571, 0.99949117, 0.26181279],
                         [0., 0., 0., 1.]]


class putsample(object):
    def __init__(self):
        # 取样品的点
        self.pose_aruco2_j1 = [0.9242292642593384, -0.5668954849243164, -1.6513054370880127, -0.9170810580253601,
                               -2.2216973304748535, -0.7864810824394226]
        self.pose_aruco2_j2 = [1.0019710063934326, -0.6748853325843811, -1.4592461585998535, -1.0018454790115356,
                               -2.145453691482544, -0.7875596880912781]
        self.getmid_j = [1.1011524200439453, -0.46504154801368713, -1.46921706199646, -1.2059367895126343,
                         -0.4831337630748749, -0.7895490527153015]
        # 放试管
        self.getA1 = [[0.66095653, -0.00496142, -0.75040779, -0.04822622],
                      [-0.0048543, 0.99992895, -0.01088682, -0.04246517],
                      [0.75040849, 0.01083842, 0.66088549, -0.01467518],
                      [0., 0., 0., 1.]]
        self.getA2 = [[0.66094795, -0.00497128, -0.75041528, -0.11374503],
                      [-0.00485806, 0.99992876, -0.0109031, -0.04158521],
                      [0.75041602, 0.01085195, 0.66087671, -0.01596513],
                      [0., 0., 0., 1.]]
        self.getB1 = [[0.66095548, -0.00183406, -0.75042288, -0.04956844],
                      [-0.00394244, 0.99997473, -0.00591639, -0.0430003],
                      [0.75041476, 0.00686897, 0.66093154, 0.05048007],
                      [0., 0., 0., 1.]]
        self.getB2 = [[0.30692276, -0.00351438, -0.95172794, -0.06525192],
                      [-0.00635653, 0.99996331, -0.00574241, -0.04635958],
                      [0.9517132, 0.00781216, 0.30688916, 0.13615358],
                      [0., 0., 0., 1.]]
        self.getC1 = [[0.30508097, -0.00183809, -0.95232464, -0.00161262],
                      [-0.00601256, 0.99997449, -0.00385621, -0.04501785],
                      [0.95230743, 0.00690237, 0.30506214, 0.20284351],
                      [0., 0., 0., 1.]]
        self.getC2 = [[0.3050631, -0.00184065, -0.95233036, -0.06667817],
                      [-0.00598714, 0.99997466, -0.00385061, -0.04531987],
                      [0.95231332, 0.00687642, 0.30504435, 0.20102115],
                      [0., 0., 0., 1.]]
        self.pose_aruco2 = []

        # 试管架上的点
        # 放
        self.putA1_j = [1.606718897819519, -0.7031920552253723, -1.1894333362579346, -1.2490800619125366,
                        1.5409659147262573, -0.7810162901878357]
        self.putA2_j = [1.6044658422470093, -0.8200023174285889, -0.9825257062911987, -1.3392254114151,
                        1.5387967824935913, -0.7807286977767944]
        self.putB1_j = [2.2701776027679443, -0.2403613030910492, -1.9009487628936768, -1.0037869215011597,
                        0.7113339304924011, -0.7806567549705505]
        self.putB2_j = [2.208207130432129, -0.32914045453071594, -1.7806631326675415, -1.0361204147338867,
                        0.6493635177612305, -0.7798658013343811]
        self.putC1_j = [2.3457741737365723, -0.28605714440345764, -1.84018874168396, -1.0185035467147827,
                        0.786954402923584, -0.7808365225791931]
        self.putC2_j = [2.2811670303344727, -0.3704620599746704, -1.7216527462005615, -1.0533177852630615,
                        0.722323477268219, -0.7801534533500671]
        # 取
        self.getA1_jia = [-0.1360524445772171, -0.8195698261260986, 0.19491897523403168, 1.5708345174789429,
                          2.954663068521768e-05, -3.0750486850738525]
        self.getA2_jia = [-0.1360524445772171, -0.8195698261260986 - 0.065, 0.19491897523403168, 1.5708345174789429,
                          2.954663068521768e-05, -3.0750486850738525]
        self.getB1_jia = [0.1605851650238037, -0.6422537565231323 + 0.065, 0.19780628383159637, 1.5708760023117065,
                          -3.7600224459310994e-05, -1.5821495056152344]
        self.getB2_jia = [0.1605851650238037, -0.6422537565231323, 0.19780628383159637, 1.5708760023117065,
                          -3.7600224459310994e-05, -1.5821495056152344]
        self.getC1_jia = [0.1605851650238037 + 0.065, -0.6422537565231323 + 0.065, 0.19780628383159637,
                          1.5708760023117065,
                          -3.7600224459310994e-05, -1.5821495056152344]
        self.getC2_jia = [0.1605851650238037 + 0.065, -0.6422537565231323, 0.19780628383159637, 1.5708760023117065,
                          -3.7600224459310994e-05, -1.5821495056152344]

    def point(self):
        duco_cobot.movej(midpose, speedj, 20, 0, True)
        print("开始取样品操作")
        robotiq(1, 1, 255, 0, 100, 50)
        motor.motor_position(183900, 10)
        motor.motor_position(177000, 0)
        # 识别aruco
        mark_size(0.130)
        duco_cobot.movej(self.pose_aruco2_j1, speedj, 20, 0, True)
        duco_cobot.movej(self.pose_aruco2_j2, speedj - 40, 20, 0, True)
        sleep(0.5)
        aruco.mark()
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
        duco_cobot.fc_config([False, True, False, False, False, False], [0, -20, 0, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0.15, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, -10, 0, 0, 0, 0], [0, 2, 0, 0, 0, 0], False, 0, 500000)
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
        duco_cobot.fc_config([True, True, True, False, False, False], [0, 20, 0, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, -0.095, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([12, 12, 12, 0, 0, 0], [2, 2, 2, 0, 0, 0], False, 0, 500000)
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
            motor.motor_position(179000, 10)
            self.put(self.putC1_j, self.getC1_jia, self.getA1_up, 1)
            robotiq(1, 1, 255, 0, 100, 1)
            duco_cobot.movej(midpose, speedj, 10, 0, True)
        elif test == 1:
            motor.motor_position(179000, 10)
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
        self.pose_aruco2_j1 = [0.9242292642593384, -0.5668954849243164, -1.6513054370880127, -0.9170810580253601,
                               -2.2216973304748535, -0.7864810824394226]
        self.pose_aruco2_j2 = [1.0019710063934326, -0.6748853325843811, -1.4592461585998535, -1.0018454790115356,
                               -2.145453691482544, -0.7875596880912781]
        self.getmid_j = [1.1011524200439453, -0.46504154801368713, -1.46921706199646, -1.2059367895126343,
                         -0.4831337630748749, -0.7895490527153015]
        # 放试管
        self.getA1 = [[0.66095653, -0.00496142, -0.75040779, -0.04822622],
                      [-0.0048543, 0.99992895, -0.01088682, -0.04246517],
                      [0.75040849, 0.01083842, 0.66088549, -0.01467518],
                      [0., 0., 0., 1.]]
        self.getA2 = [[0.66094795, -0.00497128, -0.75041528, -0.11374503],
                      [-0.00485806, 0.99992876, -0.0109031, -0.04158521],
                      [0.75041602, 0.01085195, 0.66087671, -0.01596513],
                      [0., 0., 0., 1.]]
        self.getB1 = [[0.66095548, -0.00183406, -0.75042288, -0.04956844],
                      [-0.00394244, 0.99997473, -0.00591639, -0.0430003],
                      [0.75041476, 0.00686897, 0.66093154, 0.05048007],
                      [0., 0., 0., 1.]]
        self.getB2 = [[0.30692276, -0.00351438, -0.95172794, -0.06525192],
                      [-0.00635653, 0.99996331, -0.00574241, -0.04635958],
                      [0.9517132, 0.00781216, 0.30688916, 0.13615358],
                      [0., 0., 0., 1.]]
        self.getC1 = [[0.30508097, -0.00183809, -0.95232464, -0.00161262],
                      [-0.00601256, 0.99997449, -0.00385621, -0.04501785],
                      [0.95230743, 0.00690237, 0.30506214, 0.20284351],
                      [0., 0., 0., 1.]]
        self.getC2 = [[0.3050631, -0.00184065, -0.95233036, -0.06667817],
                      [-0.00598714, 0.99997466, -0.00385061, -0.04531987],
                      [0.95231332, 0.00687642, 0.30504435, 0.20102115],
                      [0., 0., 0., 1.]]
        self.pose_aruco2 = []

        # 试管架上的点
        # 放
        self.putA1_j = [1.606718897819519, -0.7031920552253723, -1.1894333362579346, -1.2490800619125366,
                        1.5409659147262573, -0.7810162901878357]
        self.putA2_j = [1.6044658422470093, -0.8200023174285889, -0.9825257062911987, -1.3392254114151,
                        1.5387967824935913, -0.7807286977767944]
        self.putB1_j = [2.2701776027679443, -0.2403613030910492, -1.9009487628936768, -1.0037869215011597,
                        0.7113339304924011, -0.7806567549705505]
        self.putB2_j = [2.208207130432129, -0.32914045453071594, -1.7806631326675415, -1.0361204147338867,
                        0.6493635177612305, -0.7798658013343811]
        self.putC1_j = [2.3457741737365723, -0.28605714440345764, -1.84018874168396, -1.0185035467147827,
                        0.786954402923584, -0.7808365225791931]
        self.putC2_j = [2.2811670303344727, -0.3704620599746704, -1.7216527462005615, -1.0533177852630615,
                        0.722323477268219, -0.7801534533500671]
        # 取
        self.getA1_jia = [-0.1360524445772171, -0.8195698261260986, 0.19491897523403168, 1.5708345174789429,
                          2.954663068521768e-05, -3.0750486850738525]
        self.getA2_jia = [-0.1360524445772171, -0.8195698261260986 - 0.065, 0.19491897523403168, 1.5708345174789429,
                          2.954663068521768e-05, -3.0750486850738525]
        self.getB1_jia = [0.1605851650238037, -0.6422537565231323 + 0.065, 0.19780628383159637, 1.5708760023117065,
                          -3.7600224459310994e-05, -1.5821495056152344]
        self.getB2_jia = [0.1605851650238037, -0.6422537565231323, 0.19780628383159637, 1.5708760023117065,
                          -3.7600224459310994e-05, -1.5821495056152344]
        self.getC1_jia = [0.1605851650238037 + 0.065, -0.6422537565231323 + 0.065, 0.19780628383159637,
                          1.5708760023117065,
                          -3.7600224459310994e-05, -1.5821495056152344]
        self.getC2_jia = [0.1605851650238037 + 0.065, -0.6422537565231323, 0.19780628383159637, 1.5708760023117065,
                          -3.7600224459310994e-05, -1.5821495056152344]


motor = motor()
tube = testtube()

# if __name__ == '__main__':

# # #     # 初始化节点
# #     # rospy.init_node('robot')
#     try:
#         # key_mid = np.array([[0.88812738, -0.45794297, -0.03896145, 0.06287588],
#         #                          [0.41193665, 0.83075479, -0.37437238, 0.22213659],
#         #                          [0.20380861, 0.31644071, 0.92645957, 0.04626311],
#         #                          [0, 0, 0, 1]])  # 开盖
#         # key_open = np.array([[0.88826375, -0.45770228, -0.0386799, 0.06150703],
#         #                           [0.41176459, 0.83076436, -0.37454038, 0.18800396],
#         #                           [0.20356187, 0.31676363, 0.92640346, 0.12834574],
#         #                           [0., 0., 0., 1.]])
#         pose_aruco2=[-0.581345242792104, 0.025191099765841503, 0.8262101416479414, -3.125940594364204, -0.0030067658625620997, 1.5647662897127819]
        
#         # key_start_mid=duco_cobot.get_tcp_pose()
#         # key_start_mid=get_T(pose_aruco2,key_start_mid)
#         key_start_T=[[ 0.88828263 ,-0.45767131, -0.03861263 , 0.04077991],
#                         [ 0.4118243 ,  0.83087261 ,-0.3742345  , 0.19037508],
#                         [ 0.20335857 , 0.31652438 , 0.92652987 , 0.12419927],
#                         [ 0.     ,     0.      ,    0.       ,   1.        ]]
#         key_start_mid_T=[[ 0.88828012, -0.45767738 ,-0.03859853 , 0.04392342],
#                             [ 0.41183391 , 0.83086667 ,-0.37423711 , 0.22076065],
#                             [ 0.20335009 , 0.3165312  , 0.92652941  ,0.04898253],
#                             [ 0.     ,     0.       ,   0.      ,    1.        ]]
#         key_start_mid = get_t(pose_aruco2, key_start_mid_T)
#         key_start = get_t(pose_aruco2, key_start_T)

#         key(key_start_mid, key_start, -20)
#         print("盖子已打开")

      

# #         thd_B = threading.Thread(target=hearthread_fun)
# #         thd_B.daemon = True
# #         thd_B.start()
# #         dry = drying()

# #         # wayj = [0.9639929533004761, -0.26671460270881653, -1.3903968334197998, -1.5031213760375977, -0.6077816486358643, 2.361870765686035]
# #         duco_cobot.movej(dry.waypoint1_j, 20, 10, 0, True)
# #         # duco_cobot.movej(dry.jia_j, 20, 10, 0, True)
# #         # pose = duco_cobot.get_tcp_pose()
# #         # pose[2] = 0.32436954975128174
# #         # close = [0.05448080226778984, -0.8755008111000061, 0.34584997177124023, 1.5707937479019165,5.335685727914097e-06, -2.603585720062256]
# #         # duco_cobot.movel(sep.getB3, 0.2, 0.2, 0, [], "", "", True)
# #         # duco_cobot.tcp_move([0, 0, 0.1, 0, 0, 0], 0.2, 0.2, 0, True)
# #         joint = duco_cobot.get_actual_joints_position()
# #         pose = duco_cobot.get_tcp_pose()
# #         print("joint", joint)
# #         print("pose", pose)
# #         # print("force", force)
# #         # Close!
# #         duco_cobot.close()
# #         # duco_cobot.disable(True)
# #         # duco_cobot.power_off(True)
# #         # duco_cobot.shutdown(True)
#     except Thrift.TException as tx:
#         print('%s' % tx.message)