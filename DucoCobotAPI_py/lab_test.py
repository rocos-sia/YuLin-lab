import roslib

# sys.path.append('/home/sun/yulin/DucoCobotAPI_py')
roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg

# 旋转

# aruco
from aruco import *

# 新松
# sys.path.append('gen_py')
# sys.path.append('lib')
# from DucoCobot import DucoCobot

from tube_motor import motor

# from thrift import Thrift

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

midpose = [1.6679942607879639, 0.13228817284107208, -1.5182205438613892, -0.1812257021665573,
           1.560260534286499, -0.630290687084198]


def hearthread_fun():
    duco_heartbeat = DucoCobot(ip, 7003)

    if duco_heartbeat.open() == 0:
        while not stopheartthread:
            duco_heartbeat.rpc_heartbeat()
            sleep(0.4)
        duco_heartbeat.close()


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


def key(key_open, force):
    ret = duco_cobot.movel(key_open, 0.2, 0.2, 0, [], "", "", True)
    print("pose_key_open", ret)
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
    sleep(1)

    duco_cobot.movel(key_open, 0.2, 0.2, 0, [], "", "", True)


def get_T(pose1, pose2):
    """
    获得过渡矩阵
    pose1:视觉定位点位姿
    pose2:目标点位姿
    返回 过渡矩阵T
    """
    pose1_T = T.rpy2T(pose1)  # 视觉定位点位姿
    pose2_T = T.rpy2T(pose2)  # 目标点位姿
    offset = np.dot(np.linalg.inv(pose1_T), pose2_T)
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
    transition_T = np.dot(pose1_T, pose2_T)
    pose = T.T2rpy(transition_T)
    return pose


def circle(axis, edge, theta):
    """
        函数功能,从A或B旋转一定的角度angle
        axis: 圆心轴的位姿
        edge:   A或B点
        angle:       旋转角度（/度）
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


class testtube(object):
    def __init__(self):
        # 离心机取放试管中间点
        self.tube_middleA_j = [1.6679942607879639, 0.13228817284107208, -1.5182205438613892, -0.1812257021665573,
                               1.560260534286499, -0.630290687084198]
        self.tube_middleB_j = [1.8543009757995605, -0.9173381924629211, -1.2019808292388916, 2.118163585662842,
                               -1.3160853385925293, 2.3607800006866455]
        self.tube_middleB_j1 = [1.7878724336624146, -0.264102041721344, -1.4782891273498535, 1.6719069480895996,
                                -0.03256284445524216, 2.3607442378997803]
        self.tube_middleB_j2 = [0.7730482816696167, 0.5933213233947754, -2.4068868160247803, 1.8143993616104126,
                                0.8782861828804016, 2.3542966842651367]
        # 离心机取试管
        self.tube_getA1 = [-0.019964877516031265, -0.5754988193511963, 0.5015121102333069, -3.1415843963623047,
                           -9.538242011331022e-06, -3.14157772064209]
        self.tube_getA2 = [-0.08533627539873123, -0.5754979252815247, 0.5023555755615234, 3.1415882110595703,
                           -1.3000146282138303e-05, -3.1415750980377197]
        self.tube_getA3 = [-0.1502501666545868, -0.575501024723053, 0.5019562244415283, -3.1415867805480957,
                           -1.073271323548397e-05, -3.1415696144104004]
        self.tube_getA4 = [-0.019586972892284393, -0.6398314237594604, 0.5027056336402893, -3.141571521759033,
                           -1.261985471501248e-05, -3.141564130783081]
        self.tube_getB1 = [0.11972774565219879, -0.8744962215423584, 0.224714457988739, 1.5708364248275757,
                           1.4967412425903603e-05, -2.603595018386841]
        self.tube_getB2 = [0.054479941725730896, -0.8744994401931763, 0.2240978181362152, 1.570809245109558,
                           2.603209736662393e-07, -2.6035852432250977]
        self.tube_getB3 = [-0.01158121507614851, -0.8735242486000061, 0.22439850866794586, 1.570773959159851,
                           6.576499345101183e-06, -2.6048853397369385]
        # 离心机放试管
        self.tube_putA1 = [-0.01996312476694584, -0.5754976868629456, 0.6516258120536804, -3.141582727432251,
                           -1.2780274118995294e-05, -3.1415789127349854]
        self.tube_putA2 = [-0.08533278107643127, -0.5754991173744202, 0.65245521068573, 3.141589403152466,
                           -1.6222867998294532e-05, -3.1415767669677734]
        self.tube_putA3 = [-0.15025055408477783, -0.57550048828125, 0.6521590352058411, -3.1415858268737793,
                           -1.2663845154747833e-05, -3.141571283340454]
        self.tube_putA4 = [-0.019587069749832153, -0.6398272514343262, 0.6429442763328552, -3.1415536403656006,
                           -3.270020897616632e-05, -3.1415505409240723]
        self.tube_putB1 = [0.11972657591104507, -0.8744918704032898, 0.36496537923812866, 1.5708500146865845,
                           -3.1343593036581296e-06, -2.603590250015259]
        self.tube_putB2 = [0.05448080226778984, -0.8745008111000061, 0.36584997177124023, 1.5707937479019165,
                           5.335685727914097e-06, -2.603585720062256]
        self.tube_putB3 = [-0.011589700356125832, -0.8735164999961853, 0.34356293082237244, 1.5708290338516235,
                           -9.4243987405207e-06, -2.604895830154419]
        # 放盖（试管上）
        self.lid_putA1 = [-0.019687077030539513, -0.5766299366950989, 0.5387135148048401, -3.1415886878967285,
                          -3.997909516328946e-05, -3.1415398120880127]
        self.lid_putA2 = [-0.08533205837011337, -0.5767612457275391, 0.5631877779960632, 3.1415555477142334,
                          -3.843598096864298e-05, -3.1415507793426514]
        self.lid_putA3 = [-0.15025600790977478, -0.5762959122657776, 0.5623243451118469, 3.141587018966675,
                          -9.225173926097341e-06, 3.1415891647338867]
        self.lid_putB1 = [-0.019957304000854492, -0.6425226330757141, 0.5630006790161133, -3.141570806503296,
                          -1.454879566153977e-05, -3.141569137573242]
        self.lid_putB2 = [-0.08532851934432983, -0.6426718831062317, 0.5623757839202881, -3.1415839195251465,
                          -2.1270960132824257e-05, -3.141561508178711]
        self.lid_putB3 = [-0.15024584531784058, -0.6420934796333313, 0.5625747442245483, -3.1415863037109375,
                          -1.5198091205093078e-05, -3.141561985015869]
        # 取盖（试管上）
        self.lid_getA1 = [-0.01996023952960968, -0.5772994756698608, 0.5015154480934143, -3.14158296585083,
                          -1.4518182069878094e-05, -3.141566276550293]
        self.lid_getA2 = [-0.08533196896314621, -0.5767630934715271, 0.5023515224456787, 3.1415653228759766,
                          -2.342632615182083e-05, -3.1415634155273438]
        self.lid_getA3 = [-0.15026013553142548, -0.5762990713119507, 0.5014585852622986, -3.1415867805480957,
                          -8.023947884794325e-06, 3.1415767669677734]
        self.lid_getB1 = [-0.01995963416993618, -0.6425197124481201, 0.5020859241485596, -3.1415855884552,
                          -5.55909991817316e-06, -3.1415677070617676]
        self.lid_getB2 = [-0.08533016592264175, -0.6426683664321899, 0.501556396484375, 3.1415891647338867,
                          -2.283193498442415e-05, -3.141559362411499]
        self.lid_getB3 = [-0.15024349093437195, -0.6421019434928894, 0.5016224980354309, -3.1415746212005615,
                          -1.5055522453621961e-05, -3.1415483951568604]
        # 放盖（平台上）
        self.lid_middle_j = [2.1970736980438232, 0.1850786805152893, -1.890318751335144, 0.1344507783651352,
                             1.5681461095809937, -0.15728531777858734]
        # self.lid_put_A1 = [0.17644745111465454, -0.6427665948867798, 0.3580849766731262, -3.14157772064209,
        #                    7.498136710637482e-06, -3.141587257385254]
        # self.lid_put_A2 = [0.1759367436170578, -0.5772138237953186, 0.3529808223247528, -3.1415717601776123,
        #                    -1.3786975614493713e-06, 3.1415927410125732]
        # self.lid_put_A3 = [0.1759074479341507, -0.5123878717422485, 0.3462674021720886, -3.141591787338257,
        #                    -3.309639578219503e-05, 3.141589641571045]
        # self.lid_put_B1 = [0.24186427891254425, -0.642731249332428, 0.36565840244293213, -3.1415765285491943,
        #                    -1.5136298316065222e-05, -3.141591787338257]
        # self.lid_put_B2 = [0.24161803722381592, -0.577735424041748, 0.3494046628475189, -3.1415646076202393,
        #                    -8.734304174140561e-06, -3.1415889263153076]
        # self.lid_put_B3 = [0.24117670953273773, -0.5114811658859253, 0.3539354205131531, -3.141563653945923,
        #                    -6.808570105931722e-06, 3.1415908336639404]
        self.lid_put_A1 = [0.17644745111465454, -0.6427665948867798, 0.3380849766731262, -3.14157772064209,
                           7.498136710637482e-06, -3.141587257385254]
        self.lid_put_A2 = [0.1759367436170578, -0.5772138237953186, 0.3329808223247528, -3.1415717601776123,
                           -1.3786975614493713e-06, 3.1415927410125732]
        self.lid_put_A3 = [0.1759074479341507, -0.5123878717422485, 0.3362674021720886, -3.141591787338257,
                           -3.309639578219503e-05, 3.141589641571045]
        self.lid_put_B1 = [0.24186427891254425, -0.642731249332428, 0.33565840244293213, -3.1415765285491943,
                           -1.5136298316065222e-05, -3.141591787338257]
        self.lid_put_B2 = [0.24161803722381592, -0.577735424041748, 0.3394046628475189, -3.1415646076202393,
                           -8.734304174140561e-06, -3.1415889263153076]
        self.lid_put_B3 = [0.24117670953273773, -0.5114811658859253, 0.3339354205131531, -3.141563653945923,
                           -6.808570105931722e-06, 3.1415908336639404]
        # 取盖（平台上）
        self.lid_get_A1 = [0.17644771933555603, -0.6427622437477112, 0.3182399272918701, 3.1415696144104004,
                           -2.301438507856801e-05, -3.1415607929229736]
        self.lid_get_A2 = [0.17595918476581573, -0.5772243738174438, 0.3177379369735718, -3.141399621963501,
                           0.00012959253217559308, -3.1415703296661377]
        self.lid_get_A3 = [0.175916388630867, -0.5123937129974365, 0.31774404644966125, -3.1415116786956787,
                           3.979872417403385e-05, -3.1415882110595703]
        self.lid_get_B1 = [0.24187880754470825, -0.6427266597747803, 0.3180431127548218, -3.141503095626831,
                           4.682899088948034e-05, -3.1415581703186035]
        self.lid_get_B2 = [0.24161997437477112, -0.5777313113212585, 0.31811434030532837, -3.1415576934814453,
                           4.088248897460289e-05, -3.1415789127349854]
        self.lid_get_B3 = [0.24118617177009583, -0.5114812850952148, 0.3179091215133667, -3.1414153575897217,
                           0.00011468186130514368, -3.1415674686431885]

        # 倾倒点
        self.middle = [1.7606562376022339, 0.35980868339538574, -1.9709006547927856, -1.5158488750457764,
                       -1.4029709100723267, 2.3568732738494873]
        self.B23_mid = [1.696229100227356, -0.2898441553115845, -1.4265412092208862, -1.4126766920089722,
                        -1.412582278251648, -0.7934319376945496]
        # 倾倒取试管
        self.getA1_j = [2.3487939834594727, -0.42252153158187866, -1.9350558519363403, -0.7822585701942444,
                        0.7864270806312561, 2.3668200969696045]
        self.get_A2_j = [1.9364289045333862, 0.3621096611022949, -2.7668209075927734, -0.7215105295181274,
                         -1.2293435335159302, 2.355231285095215]
        self.getA3_j = [1.608720302581787, 0.2640427350997925, -2.6997811794281006, -0.7022039294242859,
                        -1.5462303161621094, 2.3547041416168213]
        self.getB1_j = [2.2836718559265137, -0.4949302077293396, -1.8126130104064941, -0.8326761722564697,
                        0.7213168144226074, 2.3670477867126465]
        self.getB2_j = [1.654320240020752, -0.8854960799217224, -1.1016489267349243, -1.1427080631256104,
                        1.6366599798202515, -0.7803811430931091]
        self.getB3_j = [1.5885268449783325, -0.8964736461639404, -1.0799814462661743, -1.1537455320358276,
                        1.570866584777832, -0.7811121940612793]
        # 倾倒放试管
        self.putA1_j = [2.348710298538208, -0.28018489480018616, -1.8458333015441895, -1.0139734745025635,
                        0.7870143055915833, 2.3670239448547363]
        self.put_A2_j = [1.9349309206008911, 0.6399519443511963, -2.5771586894989014, -1.1889551877975464,
                         -1.2299786806106567, 2.3549678325653076]
        self.putA3_j = [1.6077854633331299, 0.5063397884368896, -2.5509490966796875, -1.0933210849761963,
                        -1.5464340448379517, 2.35445237159729]
        self.putB1_j = [2.283612012863159, -0.36189332604408264, -1.722959041595459, -1.0555108785629272,
                        0.7219639420509338, 2.367239475250244]
        self.putB2_j = [1.6543681621551514, -0.806987464427948, -1.0073450803756714, -1.315544605255127,
                        1.6373430490493774, -0.7803092002868652]
        self.putB3_j = [1.588574767112732, -0.824700117111206, -0.9951691031455994, -1.3103673458099365,
                        1.5714777708053589, -0.7810522317886353]

    def get_tubeA(self, putpose, getpose):
        duco_cobot.movej(self.tube_middleA_j, 30, 10, 0, True)
        robotiq(1, 1, 120, 0, 100, 1)
        duco_cobot.movel(putpose, 0.2, 0.2, 0, [], "", "", True)
        ret = duco_cobot.movel(getpose, 0.2, 0.2, 0, [], "", "", True)
        print("移动至试管", ret)
        robotiq(1, 1, 255, 0, 100, 200)
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, 20, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)

        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0.15, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, 15, 0, 0, 0], [0, 0, 1, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        print("forcemove stop")
        sleep(1)
        duco_cobot.movej(self.tube_middleA_j, 10, 10, 0, True)

    def get_tubeB(self, putpose, getpose):
        duco_cobot.movej(self.tube_middleB_j2, 30, 10, 0, True)
        duco_cobot.movej(self.tube_middleB_j1, 20, 10, 0, True)
        duco_cobot.movej(self.tube_middleB_j, 20, 10, 0, True)
        robotiq(1, 1, 120, 0, 100, 200)
        duco_cobot.movel(putpose, 0.2, 0.2, 0, [], "", "", True)
        ret = duco_cobot.movel(getpose, 0.2, 0.2, 0, [], "", "", True)
        print("移动至试管", ret)
        robotiq(1, 1, 255, 0, 100, 1)
        duco_cobot.fc_config([False, True, False, False, False, False], [0, -20, 0, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)

        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0.15, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, -10, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        print("forcemove stop")
        sleep(1)
        duco_cobot.movej(self.tube_middleB_j, 10, 10, 0, True)
        duco_cobot.movej(self.tube_middleB_j1, 20, 10, 0, True)
        duco_cobot.movej(self.tube_middleB_j2, 20, 10, 0, True)

    def put_tubeA(self, putpose):
        duco_cobot.movej(self.tube_middleA_j, 30, 10, 0, True)
        duco_cobot.movel(putpose, 0.2, 0.2, 0, [], "", "", True)
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, -20, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)

        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, -0.135, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, -10, 0, 0, 0], [0, 0, 1, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        print("forcemove stop")
        sleep(1)
        robotiq(1, 1, 120, 0, 100, 1)
        duco_cobot.movej(self.tube_middleA_j, 20, 10, 0, True)

    def put_tubeB(self, putpose):
        duco_cobot.movej(self.tube_middleB_j2, 20, 10, 0, True)
        duco_cobot.movej(self.tube_middleB_j1, 20, 10, 0, True)
        duco_cobot.movej(self.tube_middleB_j, 20, 10, 0, True)
        duco_cobot.movel(putpose, 0.2, 0.2, 0, [], "", "", True)
        duco_cobot.fc_config([False, True, False, False, False, False], [0, 20, 0, 0, 0, 0],
                             [1000, 2000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)

        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, -0.13, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 10, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        print("forcemove stop")
        sleep(1)
        robotiq(1, 1, 120, 0, 100, 1)
        duco_cobot.movej(self.tube_middleB_j, 20, 10, 0, True)
        duco_cobot.movej(self.tube_middleB_j1, 20, 10, 0, True)
        duco_cobot.movej(self.tube_middleB_j2, 20, 10, 0, True)

    def lid_get(self, getlid, putlid, put_lid):
        """
        单个取试管盖
        """
        duco_cobot.movej(self.tube_middleA_j, 20, 10, 0, True)
        robotiq(1, 1, 120, 0, 200, 1)
        duco_cobot.movel(putlid, 0.2, 0.2, 0, [], "", "", True)
        duco_cobot.movel(getlid, 0.2, 0.2, 0, [], "", "", True)
        robotiq(1, 1, 255, 0, 100, 250)
        sleep(0.5)
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, 100, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)

        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0.07, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 0])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        print("forcemove stop")
        sleep(1)

        duco_cobot.movej(self.lid_middle_j, 20, 10, 0, True)
        duco_cobot.movel(put_lid, 0.2, 0.2, 0, [], "", "", True)

        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, -20, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)

        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, -10, 0, 0, 0], [0, 0, 1, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([0, 0, 1])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        print("forcemove stop")
        sleep(1)
        # pose = duco_cobot.get_tcp_pose()
        # print("pose", pose)
        robotiq(1, 1, 120, 0, 100, 255)
        duco_cobot.movej(self.lid_middle_j, 20, 10, 0, True)

    def lid_put(self, putlid, put_lid, get_lid):
        """
         单个放试管盖
        """
        robotiq(1, 1, 120, 0, 100, 200)
        duco_cobot.movej(self.lid_middle_j, 20, 10, 0, True)
        duco_cobot.movel(put_lid, 0.2, 0.2, 0, [], "", "", True)
        duco_cobot.movel(get_lid, 0.2, 0.2, 0, [], "", "", True)
        robotiq(1, 1, 255, 0, 100, 1)
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, 20, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.05, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)

        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0.07, 0, 0, 0], [0, 0, 0.005, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 0])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        print("forcemove stop")
        sleep(1)
        duco_cobot.movej(self.lid_middle_j, 20, 10, 0, True)
        duco_cobot.movel(putlid, 0.2, 0.2, 0, [], "", "", True)
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, -50, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)

        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, -45, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([0, 0, 1])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        print("forcemove stop")
        sleep(1)
        robotiq(1, 1, 120, 0, 100, 1)
        duco_cobot.movel(putlid, 0.2, 0.2, 0, [], "", "", True)

    def lidget(self):
        """
         取6个盖
         """
        motor.motor_position(163000)
        self.lid_get(self.lid_getA1, self.lid_putA1, self.lid_put_A1)
        # self.lid_get(self.lid_getA2, self.lid_putA2, self.lid_put_A2)
        # self.lid_get(self.lid_getA3, self.lid_putA3, self.lid_put_A3)
        # self.lid_get(self.lid_getB1, self.lid_putB1, self.lid_put_B1)
        # self.lid_get(self.lid_getB2, self.lid_putB2, self.lid_put_B2)
        # self.lid_get(self.lid_getB3, self.lid_putB3, self.lid_put_B3)
        motor.motor_position(155000)
        robotiq(1, 1, 255, 0, 100, 50)

    def lidput(self):
        """
           放6个盖
           """
        motor.motor_position(159000)
        self.lid_put(self.lid_putA1, self.lid_put_A1, self.lid_get_A1)
        # self.lid_put(self.lid_putA2, self.lid_put_A2, self.lid_get_A2)
        # self.lid_put(self.lid_putA3, self.lid_put_A3, self.lid_get_A3)
        # self.lid_put(self.lid_putB1, self.lid_put_B1, self.lid_get_B1)
        # self.lid_put(self.lid_putB2, self.lid_put_B2, self.lid_get_B2)
        # self.lid_put(self.lid_putB3, self.lid_put_B3, self.lid_get_B3)
        motor.motor_position(155000)


class getsample(object):
    def __init__(self):
        self.pose_aruco2_j = [0.9720103740692139, -0.5249746441841125, -1.7237380743026733, -0.8820751309394836,
                              -2.1653714179992676, -0.7871522307395935]
        # 取试管
        self.getA1 = [[0.66102145, -0.00486366, -0.75035124, -0.03326121],
                      [-0.00498256, 0.9999285, -0.01087077, -0.04433019],
                      [0.75035046, 0.01092448, 0.66094995, -0.01347388],
                      [0., 0., 0., 1.]]
        self.getA2 = [[0.66099894, -0.00485646, -0.75037112, -0.09963392],
                      [-0.00498823, 0.99992852, -0.01086573, -0.04578365],
                      [0.75037026, 0.01092526, 0.66092747, -0.01475848],
                      [0., 0., 0., 1.]]
        self.getB1 = [[0.66101055, -0.00484911, -0.75036094, -0.03508083],
                      [-0.00498511, 0.99992867, -0.01085341, -0.04489139],
                      [0.75036005, 0.01091485, 0.66093923, 0.0507833],
                      [0., 0., 0., 1.]]
        self.getB2 = [[0.34890099, -0.00611464, -0.93713964, -0.05501976],
                      [-0.00862115, 0.99991546, -0.00973393, -0.04441568],
                      [0.93711994, 0.0114754, 0.34881877, 0.12656609],
                      [0., 0., 0., 1.]]
        self.getC1 = [[0.39278735, -0.00587307, -0.91961057, 0.00516232],
                      [-0.00816063, 0.99991798, -0.00987155, -0.04390019],
                      [0.91959312, 0.01138202, 0.39270721, 0.18246029],
                      [0., 0., 0., 1.]]
        self.getC2 = [[0.39279172, -0.0058748, -0.9196087, -0.05977224],
                      [-0.00815618, 0.99991801, -0.00987158, -0.0466197],
                      [0.91959129, 0.01137797, 0.3927116, 0.18114367],
                      [0., 0., 0., 1.]]
        self.getmid_j = [1.1011524200439453, -0.46504154801368713, -1.46921706199646, -1.2059367895126343,
                         -0.4831337630748749, -0.7895490527153015]
        self.getA1_up_j = [1.1087743043899536, -0.5346339344978333, -1.5609922409057617, -1.0443775653839111,
                           -1.179597020149231, -0.7895370721817017]
        self.getA2_up_j = [1.0363057851791382, -0.591055691242218, -1.450222134590149, -1.0989896059036255,
                           -1.2519457340240479, -0.7896209955215454]
        self.getB1_up_j = [1.139370083808899, -0.6073662042617798, -1.4202735424041748, -1.1121243238449097,
                           -1.148881435394287, -0.7898606657981873]
        self.getB2_up_j = [1.1892603635787964, -0.754616379737854, -1.1621812582015991, -1.221923828125,
                           -0.7331126928329468, -0.7906156778335571]
        self.getC1_up_j = [1.264305591583252, -0.7818205952644348, -1.1114280223846436, -1.2447417974472046,
                           -0.7052972912788391, -0.7910950183868408]
        self.getC2_up_j = [1.2013405561447144, -0.8378348350524902, -1.0146315097808838, -1.2860034704208374,
                           -0.7682265043258667, -0.7909631729125977]

        self.getA1_up = []
        self.getA2_up = []
        self.getB1_up = []
        self.getB2_up = []
        self.getC1_up = []
        self.getC2_up = []

        # 放试管
        # self.putmid_j = []
        self.putA1 = [2.129542589187622, -0.2900838553905487, -1.8259155750274658, -1.0307035446166992,
                      0.5708668231964111, -0.7793264985084534]
        self.putA2 = [2.1868512630462646, -0.19653499126434326, -1.9493170976638794, -0.9999040365219116,
                      0.628103494644165, -0.780249297618866]
        self.putB1 = [2.2706329822540283, -0.23458491265773773, -1.900864839553833, -1.0097311735153198,
                      0.7118732333183289, -0.7806088328361511]
        self.putB2 = [2.2085306644439697, -0.3235797584056854, -1.7809627056121826, -1.0414294004440308,
                      0.6497829556465149, -0.7798538208007812]
        self.putC1 = [2.2812390327453613, -0.36456581950187683, -1.7228271961212158, -1.058099389076233,
                      0.7225032448768616, -0.780165433883667]
        self.putC2 = [2.3486382961273193, -0.279825359582901, -1.8410875797271729, -1.023848533630371,
                      0.7898545861244202, -0.7808604836463928]
        self.pose_aruco2 = []

    def point(self):
        motor.motor_position(153000)
        robotiq(1, 1, 255, 0, 100, 50)
        duco_cobot.movej(self.pose_aruco2_j, 20, 10, 0, True)
        rospy.set_param("/aruco_single/marker_size", 0.129)
        sleep(1)
        aruco.mark()
        rospy.set_param("/aruco_single/marker_size", 0.149)
        sleep(1)
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
        self.getA1_up[2] += 0.15
        self.getA2_up[2] += 0.15
        self.getB1_up[2] += 0.15
        self.getB2_up[2] += 0.15
        self.getC1_up[2] += 0.15
        self.getC2_up[2] += 0.15

        self.pose_aruco2[2] += 0.2
        # duco_cobot.movej(self.getmid_j, 20, 10, 0, True)
        # duco_cobot.movel(self.pose_aruco2, 0.1, 0.1, 0, [], "", "", True)

    def get(self, getpose_up_j, getpose, putpose_j):
        duco_cobot.movej(self.getmid_j, 20, 10, 0, True)
        # duco_cobot.movej(getpose_up_j, 20, 10, 0, True)
        duco_cobot.movel(getpose_up_j, 0.2, 0.2, 0, [], "", "", True)
        robotiq(1, 1, 145, 0, 100, 70)
        duco_cobot.movel(getpose, 0.2, 0.2, 0, [], "", "", True)
        robotiq(1, 1, 250, 0, 100, 70)
        duco_cobot.fc_config([True, True, True, False, False, False], [0, -20, 0, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)

        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0.13, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, -20, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        print("forcemove stop")
        sleep(1)
        duco_cobot.movej(self.getmid_j, 20, 10, 0, True)
        # 放
        duco_cobot.movej(putpose_j, 20, 10, 0, True)
        duco_cobot.fc_config([False, True, False, False, False, False], [0, 20, 0, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)

        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, -0.135, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 20, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        print("forcemove stop")
        sleep(1)
        robotiq(1, 1, 150, 0, 100, 50)
        duco_cobot.movej(putpose_j, 20, 10, 0, True)

    def get_lid(self):
        # self.get(self.getA1_up_j, self.getA1, self.putA1)
        # self.get(self.getA2_up_j, self.getA2, self.putA2)
        # self.get(self.getB1_up_j, self.getB1, self.putB1)
        # self.get(self.getB2_up_j, self.getB2, self.putB2)
        # self.get(self.getC1_up_j, self.getC1, self.putC1)
        # self.get(self.getC2_up_j, self.getC2, self.putC2)

        self.get(self.getA1_up, self.getA1, self.putC2)
        # self.get(self.getA2_up, self.getA2, self.putA2)
        # self.get(self.getB1_up, self.getB1, self.putB1)
        # self.get(self.getB2_up, self.getB2, self.putB2)
        # self.get(self.getC1_up, self.getC1, self.putC1)
        # self.get(self.getC2_up, self.getC2, self.putC2)
        duco_cobot.movej(midpose, 20, 10, 0, True)

    def defult(self):
        self.pose_aruco2_j = [0.9720103740692139, -0.5249746441841125, -1.7237380743026733, -0.8820751309394836,
                              -2.1653714179992676, -0.7871522307395935]
        # 取试管
        self.getA1 = [[0.66102145, -0.00486366, -0.75035124, -0.03326121],
                      [-0.00498256, 0.9999285, -0.01087077, -0.04433019],
                      [0.75035046, 0.01092448, 0.66094995, -0.01347388],
                      [0., 0., 0., 1.]]
        self.getA2 = [[0.66099894, -0.00485646, -0.75037112, -0.09963392],
                      [-0.00498823, 0.99992852, -0.01086573, -0.04578365],
                      [0.75037026, 0.01092526, 0.66092747, -0.01475848],
                      [0., 0., 0., 1.]]
        self.getB1 = [[0.66101055, -0.00484911, -0.75036094, -0.03508083],
                      [-0.00498511, 0.99992867, -0.01085341, -0.04489139],
                      [0.75036005, 0.01091485, 0.66093923, 0.0507833],
                      [0., 0., 0., 1.]]
        self.getB2 = [[0.34890099, -0.00611464, -0.93713964, -0.05501976],
                      [-0.00862115, 0.99991546, -0.00973393, -0.04441568],
                      [0.93711994, 0.0114754, 0.34881877, 0.12656609],
                      [0., 0., 0., 1.]]
        self.getC1 = [[0.39278735, -0.00587307, -0.91961057, 0.00516232],
                      [-0.00816063, 0.99991798, -0.00987155, -0.04390019],
                      [0.91959312, 0.01138202, 0.39270721, 0.18246029],
                      [0., 0., 0., 1.]]
        self.getC2 = [[0.39279172, -0.0058748, -0.9196087, -0.05977224],
                      [-0.00815618, 0.99991801, -0.00987158, -0.0466197],
                      [0.91959129, 0.01137797, 0.3927116, 0.18114367],
                      [0., 0., 0., 1.]]
        self.getmid_j = [1.1011524200439453, -0.46504154801368713, -1.46921706199646, -1.2059367895126343,
                         -0.4831337630748749, -0.7895490527153015]
        self.getA1_up_j = [1.1087743043899536, -0.5346339344978333, -1.5609922409057617, -1.0443775653839111,
                           -1.179597020149231, -0.7895370721817017]
        self.getA2_up_j = [1.0363057851791382, -0.591055691242218, -1.450222134590149, -1.0989896059036255,
                           -1.2519457340240479, -0.7896209955215454]
        self.getB1_up_j = [1.139370083808899, -0.6073662042617798, -1.4202735424041748, -1.1121243238449097,
                           -1.148881435394287, -0.7898606657981873]
        self.getB2_up_j = [1.1892603635787964, -0.754616379737854, -1.1621812582015991, -1.221923828125,
                           -0.7331126928329468, -0.7906156778335571]
        self.getC1_up_j = [1.264305591583252, -0.7818205952644348, -1.1114280223846436, -1.2447417974472046,
                           -0.7052972912788391, -0.7910950183868408]
        self.getC2_up_j = [1.2013405561447144, -0.8378348350524902, -1.0146315097808838, -1.2860034704208374,
                           -0.7682265043258667, -0.7909631729125977]

        self.getA1_up = []
        self.getA2_up = []
        self.getB1_up = []
        self.getB2_up = []
        self.getC1_up = []
        self.getC2_up = []

        # 放试管
        # self.putmid_j = []
        self.putA1 = [2.129542589187622, -0.2900838553905487, -1.8259155750274658, -1.0307035446166992,
                      0.5708668231964111, -0.7793264985084534]
        self.putA2 = [2.1868512630462646, -0.19653499126434326, -1.9493170976638794, -0.9999040365219116,
                      0.628103494644165, -0.780249297618866]
        self.putB1 = [2.2706329822540283, -0.23458491265773773, -1.900864839553833, -1.0097311735153198,
                      0.7118732333183289, -0.7806088328361511]
        self.putB2 = [2.2085306644439697, -0.3235797584056854, -1.7809627056121826, -1.0414294004440308,
                      0.6497829556465149, -0.7798538208007812]
        self.putC1 = [2.2812390327453613, -0.36456581950187683, -1.7228271961212158, -1.058099389076233,
                      0.7225032448768616, -0.780165433883667]
        self.putC2 = [2.3486382961273193, -0.279825359582901, -1.8410875797271729, -1.023848533630371,
                      0.7898545861244202, -0.7808604836463928]
        self.pose_aruco2 = []


class centrifuge(object):
    def __init__(self):
        print("开始离心机操作")
        # 旋转矩阵
        self.key_open = np.array([[0.88812738, -0.45794297, -0.03896145, 0.06287588],
                                  [0.41193665, 0.83075479, -0.37437238, 0.22213659],
                                  [0.20380861, 0.31644071, 0.92645957, 0.04626311],
                                  [0, 0, 0, 1]])
        self.key_start = np.array([])
        self.axis = np.array([[0.02845407, -0.99952615, 0.011741, -0.10179548],
                              [0.9994615, 0.02864034, 0.01601378, -0.17848992],
                              [-0.01634246, 0.01127902, 0.99980283, 0.13430834],
                              [0., 0., 0., 1.]])
        self.getA = np.array([[0.08670534, -0.85263, 0.51527106, -0.28070549],
                              [0.99620706, 0.07040118, -0.05113874, -0.15647593],
                              [0.00732673, 0.51775067, 0.85550018, 0.04193006],
                              [0., 0., 0., 1.]])
        self.getB = np.array([[-0.79930289, -0.33028191, -0.50202464, 0.10769505],
                              [0.60076288, -0.41958753, -0.68046327, 0.10479884],
                              [0.01410143, -0.84549403, 0.53379864, 0.1884626],
                              [0., 0., 0., 1.]])
        self.putA = self.getA
        self.putB = np.array([[-0.79898651, -0.33113412, -0.50196687, 0.00604547],
                              [0.60115704, -0.41873701, -0.68063905, -0.02848487],
                              [0.01519071, -0.84558235, 0.53362884, 0.11008111],
                              [0., 0., 0., 1.]])
        # self.put_middleA = np.array([[0.99254937, -0.12135759, 0.01086691, -0.10877843],
        #                              [0.12099719, 0.99222159, 0.02925741, -0.06116523],
        #                              [-0.01433299, -0.02772456, 0.99951284, -0.1318585],
        #                              [0., 0., 0., 1., ]])
        self.put_middleB = np.array([[-0.99842942, -0.01541606, -0.05386133, -0.10597664],
                                     [0.05410575, -0.01590283, -0.99840857, 0.18961635],
                                     [0.01453498, -0.99975469, 0.01671195, 0.07029574],
                                     [0., 0., 0., 1.]])
        # self.get_middleA = np.array([[0.99254937, -0.12135759, 0.01086691, -0.10712437],
        #                              [0.12099719, 0.99222159, 0.02925741, -0.05977473],
        #                              [-0.01433299, -0.02772456, 0.99951284, -0.03188185],
        #                              [0., 0., 0., 1.]])
        # self.get_middleB = np.array([[0.99782771, 0.02664519, 0.06024854, -0.08751736],
        #                              [0.0397348, 0.4860373, -0.8730343, 0.08593098],
        #                              [-0.0525452, 0.87353178, 0.48392275, 0.00844932],
        #                              [0., 0., 0., 1.]])
        self.watch_aruco_pose = np.array([[0.99864272, -0.0507411, 0.01175022, -0.09899948],
                                          [0.05055016, 0.99859313, 0.01601386, -0.06431557],
                                          [-0.01254625, -0.01539815, 0.99980272, 0.1690991],
                                          [0., 0., 0., 1.]])
        self.close_door1 = np.array([[0.99907925, -0.04256267, -0.00539094, -0.15217543],
                                     [0.0307517, 0.79806095, -0.60179154, -0.09484748],
                                     [0.02991616, 0.60107166, 0.79863501, -0.36161365],
                                     [0., 0., 0., 1.]])
        self.close_door3 = np.array([[0.99959752, -0.02304766, 0.01654071, -0.14592057],
                                     [0.022821, 0.99964482, 0.01376344, -0.14525979],
                                     [-0.01685205, -0.01338042, 0.99976846, -0.13248836],
                                     [0., 0., 0., 1.]])

        self.pose_aruco2 = []
        self.put_middleA = []
        self.get_middleA = []
        self.get_middleB = []
        self.mark2camera = []
        # 过渡矩阵求的示教点
        # self.key_start = get_t(self.pose_aruco2, self.key_start)

    def point(self):
        robotiq(1, 1, 255, 0, 100, 50)
        duco_cobot.movej(
            [0.12111843377351761, -0.00191118021029979, -1.4549438953399658, -0.09991273283958435, 1.57663094997406,
             -0.6489381790161133], 20, 10, 0, True)
        sleep(2)
        # 定位
        aruco.mark()
        sleep(1)
        joint = duco_cobot.get_actual_joints_position()
        print("joint", joint)
        self.pose_aruco2 = duco_cobot.get_tcp_pose()

        self.key_open = get_t(self.pose_aruco2, self.key_open)
        self.axis = get_t(self.pose_aruco2, self.axis)
        self.getA = get_t(self.pose_aruco2, self.getA)
        self.getB = get_t(self.pose_aruco2, self.getB)
        self.putA = get_t(self.pose_aruco2, self.putA)
        self.putB = get_t(self.pose_aruco2, self.putB)
        self.watch_aruco_pose = get_t(self.pose_aruco2, self.watch_aruco_pose)
        self.watch_aruco_pose[2] += 0.25
        self.put_middleA = self.watch_aruco_pose
        self.put_middleB = get_t(self.pose_aruco2, self.put_middleB)
        self.get_middleA = self.put_middleA
        self.get_middleB = self.put_middleB
        self.mark2camera = np.array([[0.27921038, 0.9559346, -0.09072269, -0.02314434],
                                     [0.95787014, -0.28389802, -0.04343624, 0.1882538],
                                     [-0.0672782, -0.07477271, -0.99492848, 1.35911388],
                                     [0., 0., 0., 1.]])  # 取试管旋转角度的比较矩阵
        self.close_door1 = get_t(self.pose_aruco2, self.close_door1)
        # close_door2 = tcp_move  z轴 0.16
        self.close_door3 = get_t(self.pose_aruco2, self.close_door3)

        # 移动至二维码上方
        self.pose_aruco2[2] += 0.05
        ret = duco_cobot.movel(self.pose_aruco2, 0.2, 0.2, 0, [], "", "", True)
        self.pose_aruco2[2] -= 0.05

    def open(self):

        key(self.key_open, -20)
        print("盖子已打开")

    def start(self):

        key(self.key_start, -10)
        print("离心机已启动")

    def put(self):
        motor.motor_position(159000)
        motor.motor_position(155000)
        num = 0  # 放入试管数量
        dist = 0.10  # 位置判断
        flag = 0  # 判断正半圆能够放几个的标志位
        # 移动至平台试管架取试管
        tube.get_tubeA(tube.tube_putA1, tube.tube_getA1)
        # 移动至离心机
        ret = duco_cobot.movel(self.put_middleA, 0.4, 0.2, 0, [], "", "", True)
        print("移动至put_middleA", ret)
        ret = duco_cobot.movel(self.putA, 0.2, 0.2, 0, [], "", "", True)
        print("移动至putA", ret)
        # 放A半圆
        offset = 20 * math.pi / 180  # 20度转弧度
        offset_sum = 0  # 旋转角度累加
        goal2baselink = T.rpy2T(self.axis)  # 圆心相对于基座的T
        eelink2baselink = T.rpy2T(self.putA)  # 当前位姿转换成T
        eelink2goal = np.dot(np.linalg.inv(goal2baselink), eelink2baselink)
        P, M = T.T2rpy_2(eelink2goal)  # 旋转平移矩阵
        radius = math.sqrt(math.pow(P[0], 2) + math.pow(P[1], 2))  # R
        angle = math.atan2(P[1], P[0])  # rad
        for i in range(10):
            tran = [radius * math.cos(angle + offset_sum),
                    radius * math.sin(angle + offset_sum), P[2]]
            euler = [M[0], M[1], M[2] + offset_sum]
            eelink2goal = [tran[0], tran[1], tran[2],
                           euler[0], euler[1], euler[2]]  # 转动后目标相对于基座的T
            eelink2goal = T.rpy2T(eelink2goal)
            target = np.dot(goal2baselink, eelink2goal)
            target = T.T2rpy(target)  # 目标位姿
            ret = duco_cobot.movel(target, 0.15, 0.15, 0, [], "", "", True)
            print("旋转固定角度", ret)
            sleep(1)

            # 判断画圆是否成功
            if not ret:
                offset_sum = offset_sum + offset
            else:
                # Foremode
                ponit_pose = duco_cobot.get_tcp_pose()
                duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, -20, 0, 0, 0],
                                     [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                                     [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                                     "default", 0)

                duco_cobot.fc_start()
                duco_cobot.fc_wait_pos([0, 0, -dist, 0, 0, 0], [0, 0, 0.02, 0, 0, 0], False, 0, 500000)
                duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
                duco_cobot.fc_wait_ft([0, 0, -10, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000)
                duco_cobot.fc_wait_logic([1, 0, 2])
                duco_cobot.fc_move()
                duco_cobot.fc_stop()
                print("forcemove stop")
                sleep(1)

                # 判断能否放入
                if abs(duco_cobot.get_tcp_pose()[2] - ponit_pose[2]) >= dist - 0.05:
                    robotiq(1, 1, 150, 0, 100, 1)
                    print("试管放入成功")
                    ret = duco_cobot.movel(ponit_pose, 0.2, 0.2, 0, [], "", "", True)
                    print("返回放置点", ret)
                    ret = duco_cobot.movel(self.put_middleA, 0.2, 0.2, 0, [], "", "", True)
                    print("移动至中间点", ret)
                    offset = 60 * math.pi / 180  # 圆的旋转角度为60度
                    num += 1  # 放置试管数量+1
                    if (num == 3) and (flag != 0):
                        print("放了3个")
                        break
                    if num == 4:
                        print("放了4个")
                        break
                    if num == 1:
                        break
                        # tube.get_tubeA(tube.tube_putA2, tube.tube_getA2)
                    if num == 2:
                        tube.get_tubeA(tube.tube_putA3, tube.tube_getA3)
                    if num == 3:
                        tube.get_tubeA(tube.tube_putA4, tube.tube_getA4)
                    duco_cobot.movel(self.put_middleA, 0.2, 0.2, 0, [], "", "", True)
                    # print("测试阶段夹试管，请随便输入并回车:")
                    # input()
                    # robotiq(1, 1, 250, 0, 10, 150)  # 加紧试管
                    # sleep(1)
                else:
                    print("试管不能放入，继续旋转到下一个角度")
                    flag = 1
                    ret = duco_cobot.movel(ponit_pose, 0.2, 0.2, 0, [], "", "", True)
                    print("返回放置点", ret)
            offset_sum = offset_sum - offset

        # # 放B半圆
        # ret = duco_cobot.movel(self.put_middleB, 0.2, 0.2, 0, [], "", "", True)
        # if num == 3:
        #     tube.get_tubeB(tube.tube_putB1, tube.tube_getB1)
        # if num == 4:
        #     tube.get_tubeB(tube.tube_putB2, tube.tube_getB2)
        # ret = duco_cobot.movel(self.put_middleB, 0.2, 0.2, 0, [], "", "", True)
        # print("移动至put_middleB", ret)
        # offset = 60 * math.pi / 180  # 60度转弧度
        # offset_sum = - (abs(offset_sum) % abs(60 * math.pi / 180))  # 旋转角度累加
        # # offset_sum = -(20 * math.pi / 180)
        # goal2baselink = T.rpy2T(self.axis)  # 圆心相对于基座的T
        # eelink2baselink = T.rpy2T(self.putB)  # 当前位姿转换成T
        # eelink2goal = np.dot(np.linalg.inv(goal2baselink), eelink2baselink)
        # P, M = T.T2rpy_2(eelink2goal)  # 旋转平移矩阵
        # radius = math.sqrt(math.pow(P[0], 2) + math.pow(P[1], 2))  # R
        # angle = math.atan2(P[1], P[0])  # rad
        # for i in range(10):
        #     tran = [radius * math.cos(angle + offset_sum),
        #             radius * math.sin(angle + offset_sum), P[2]]
        #     euler = [M[0], M[1], M[2] + offset_sum]
        #     eelink2goal = [tran[0], tran[1], tran[2],
        #                    euler[0], euler[1], euler[2]]  # 转动后目标相对于基座的T
        #     eelink2goal = T.rpy2T(eelink2goal)
        #     target = np.dot(goal2baselink, eelink2goal)
        #     target = T.T2rpy(target)  # 目标位姿
        #     ret = duco_cobot.movel(target, 0.15, 0.15, 0, [], "", "", True)
        #     print("旋转固定角度", ret)
        #     sleep(1)
        #
        #     # 判断画圆是否成功
        #     if not ret:
        #         offset_sum = offset_sum - offset
        #     else:
        #         # Foremode
        #         ponit_pose = duco_cobot.get_tcp_pose()
        #         duco_cobot.fc_config([False, True, False, False, False, False], [0, 20, 0, 0, 0, 0],
        #                              [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
        #                              [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
        #                              "default", 0)
        #
        #         duco_cobot.fc_start()
        #         duco_cobot.fc_wait_pos([0, 0, -dist, 0, 0, 0], [0, 0, 0.02, 0, 0, 0], False, 0, 500000)
        #         duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        #         duco_cobot.fc_wait_ft([0, 10, 0, 0, 0, 0], [0, 2, 0, 0, 0, 0], False, 0, 500000)
        #         duco_cobot.fc_wait_logic([1, 0, 2])
        #         # duco_cobot.fc_wait_logic([0, 0, 1])
        #         duco_cobot.fc_move()
        #         duco_cobot.fc_stop()
        #         print("forcemove stop")
        #         sleep(1)
        #
        #         # 判断能否放入
        #         if abs(duco_cobot.get_flange_pose()[2] - ponit_pose[2]) >= dist - 0.05:
        #             robotiq(1, 1, 175, 0, 100, 1)
        #             print("试管放入成功")
        #             ret = duco_cobot.movel(ponit_pose, 0.2, 0.2, 0, [], "", "", True)
        #             print("返回放置点", ret)
        #             ret = duco_cobot.movel(self.put_middleB, 0.2, 0.2, 0, [], "", "", True)
        #             print("移动至中间点", ret)
        #             offset = 60 * math.pi / 180  # 圆的旋转角度为60度
        #             num += 1  # 放置试管数量+1
        #             if num == 4:
        #                 tube.get_tubeB(tube.tube_putB2, tube.tube_getB2)
        #             if num == 5:
        #                 tube.get_tubeB(tube.tube_putB3, tube.tube_getB3)
        #             duco_cobot.movel(self.put_middleB, 0.2, 0.2, 0, [], "", "", True)
        #             # print("测试阶段夹试管，请随便输入并回车:")
        #             # input()
        #             # robotiq(1, 1, 250, 0, 10, 150)  # 加紧试管
        #             # sleep(1)
        #         else:
        #             print("试管不能放入，继续旋转到下一个角度")
        #             ret = duco_cobot.movel(ponit_pose, 0.2, 0.2, 0, [], "", "", True)
        #             print("返回放置点", ret)
        #     if num == 6:
        #         print("放试管操作完成")
        #         return True
        #
        #     offset_sum = offset_sum + offset

    def get(self):
        num = 0
        # movel
        duco_cobot.movel(self.watch_aruco_pose, 0.2, 0.2, 0, [], "", "", True)
        self.watch_aruco_pose[2] -= 0.25
        duco_cobot.movel(self.watch_aruco_pose, 0.2, 0.2, 0, [], "", "", True)
        # 求旋转角度
        rospy.set_param("/aruco_single/marker_size", 0.0274)
        sleep(1)
        mark2camera = aruco.test_mark()
        sleep(1)
        rospy.set_param("/aruco_single/marker_size", 0.149)
        # 计算旋转角度
        euler_T = np.dot(np.linalg.inv(self.mark2camera), mark2camera)  # 二维码旋转角度
        tran, euler = T.T2euler_2(euler_T)
        print("二维码旋转角度：", euler[2] * 180 / math.pi)
        self.watch_aruco_pose[2] += 0.3
        duco_cobot.movel(self.watch_aruco_pose, 0.2, 0.2, 0, [], "", "", True)
        # A半圆
        assert -math.pi <= euler[2] <= math.pi
        euler[2] = euler[2] * 180 / math.pi
        # 求Angle
        theta = abs(euler[2]) % 60

        if euler[2] >= 0:
            angleA = [-theta, -theta - 60, -theta - 120]
        else:
            angleA = [theta - 60, theta - 120, theta - 180]

        angleA = [angleA[0] * math.pi / 180, angleA[1] * math.pi / 180, angleA[2] * math.pi / 180]
        print('angleA', angleA)
        # for i in range(3):
        self.get_tubeA(self.getA, self.get_middleA, angleA[0])
        tube.put_tubeA(tube.tube_putA1)
        # num += 1
        # self.get_tubeA(self.getA, self.get_middleA, angleA[1])
        # tube.put_tubeA(tube.tube_putA2)
        # num += 1
        # self.get_tubeA(self.getA, self.get_middleA, angleA[2])
        # tube.put_tubeA(tube.tube_putA3, )
        # num += 1
        print('成功放入试管：', num)
        print("A半圆取完，取B半圆")

        # # 取B半圆
        # duco_cobot.movej(tube.tube_middleB_j1, 20, 10, 0, True)
        # duco_cobot.movej(tube.tube_middleB_j2, 20, 10, 0, True)
        # assert -math.pi / 3 < angleA[0] <= 0
        # angleB = [angleA[0], angleA[0] + 60 * math.pi / 180, angleA[0] + 120 * math.pi / 180]
        # # for i in range(3):
        # self.get_tubeB(self.getB, self.get_middleB, angleB[0])
        # tube.put_tubeB(tube.tube_putB1)
        # num += 1
        # self.get_tubeB(self.getB, self.get_middleB, angleB[1])
        # tube.put_tubeB(tube.tube_putB2)
        # num += 1
        # self.get_tubeB(self.getB, self.get_middleB, angleB[2])
        # tube.put_tubeB(tube.tube_putB3)
        # num += 1

    def get_tubeA(self, edge, middle, angle):
        dist = 0.14
        # 夹抓张开一定角度
        robotiq(1, 1, 150, 0, 100, 1)
        duco_cobot.movel(middle, 0.2, 0.2, 0, [], "", "", True)
        # 旋转至第一个夹取点
        target = circle(self.axis, edge, angle)
        duco_cobot.movel(target, 0.2, 0.2, 0, [], "", "", True)
        robotiq(1, 1, 190, 0, 100, 1)
        # Forcemode
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, -20, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)

        duco_cobot.fc_start()
        #
        duco_cobot.fc_wait_pos([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, -10, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000)
        # duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_wait_logic([0, 0, 1])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        print("forcemove stop")
        sleep(1)

        # 上升一点点，夹
        ret = duco_cobot.tcp_move([0, 0, -0.002, 0, 0, 0], 0.05, 0.05, 0, True)
        print("move", ret)
        # 夹爪闭合
        robotiq(1, 1, 250, 0, 100, 1)

        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, 25, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)

        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, dist, 0, 0, 0], [0, 0, 0.02, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, 20, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 2])
        # duco_cobot.fc_wait_logic([0, 0, 1])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        print("forcemove stop")
        sleep(1)

        # 放试管
        # print("试管取出成功,松开夹爪，请输入")
        # robotiq(1, 1, 150, 0, 10, 1)
        duco_cobot.movel(middle, 0.2, 0.2, 0, [], "", "", True)

    def get_tubeB(self, edge, middle, angle):
        dist = 0.14
        # 夹抓张开一定角度
        robotiq(1, 1, 150, 0, 100, 1)
        # 从A点旋转到位置0
        target = circle(self.axis, edge, angle)
        duco_cobot.movel(self.get_middleB, 0.2, 0.2, 0, [], "", "", True)
        duco_cobot.movel(target, 0.2, 0.2, 0, [], "", "", True)
        robotiq(1, 1, 170, 0, 10, 1)
        duco_cobot.tcp_move([0, 0, 0.1, 0, 0, 0], 0.03, 0.03, 0, True)

        # 夹爪闭合
        robotiq(1, 1, 250, 0, 100, 1)

        duco_cobot.fc_config([False, True, False, False, False, False], [0, -20, 0, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)

        duco_cobot.fc_start()
        #
        duco_cobot.fc_wait_pos([0, 0, dist, 0, 0, 0], [0, 0, 0.02, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, -20, 0, 0, 0, 0], [0, 2, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 2])
        # duco_cobot.fc_wait_logic([0, 0, 1])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        print("forcemove stop")
        sleep(1)

        # 放试管
        # robotiq(1, 1, 150, 0, 10, 1)
        duco_cobot.movel(middle, 0.2, 0.2, 0, [], "", "", True)

    def close_door(self):
        robotiq(1, 1, 255, 0, 100, 50)
        duco_cobot.movel(self.close_door1, 0.3, 0.3, 0, [], "", "", True)
        duco_cobot.tcp_move([0, 0, 0.16, 0, 0, 0], 0.2, 0.2, 0, True)
        duco_cobot.movel(self.close_door3, 0.2, 0.2, 0, [], "", "", True)
        # 向下按压
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, -40, 0, 0, 0],
                             [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.05, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
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
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, 30, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_wait_pos([0, 0, 0.15, 0, 0, 0], [0, 0, 0.01, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, 10, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 0])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()

    def defult(self):
        print("开始离心机操作")
        # 旋转矩阵
        self.key_open = np.array([[0.88812738, -0.45794297, -0.03896145, 0.06287588],
                                  [0.41193665, 0.83075479, -0.37437238, 0.22213659],
                                  [0.20380861, 0.31644071, 0.92645957, 0.04626311],
                                  [0, 0, 0, 1]])
        self.key_start = np.array([])
        self.axis = np.array([[0.02845407, -0.99952615, 0.011741, -0.10179548],
                              [0.9994615, 0.02864034, 0.01601378, -0.17848992],
                              [-0.01634246, 0.01127902, 0.99980283, 0.13430834],
                              [0., 0., 0., 1.]])
        self.getA = np.array([[0.08670534, -0.85263, 0.51527106, -0.28070549],
                              [0.99620706, 0.07040118, -0.05113874, -0.15647593],
                              [0.00732673, 0.51775067, 0.85550018, 0.04193006],
                              [0., 0., 0., 1.]])
        self.getB = np.array([[-0.79930289, -0.33028191, -0.50202464, 0.10769505],
                              [0.60076288, -0.41958753, -0.68046327, 0.10479884],
                              [0.01410143, -0.84549403, 0.53379864, 0.1884626],
                              [0., 0., 0., 1.]])
        self.putA = self.getA
        self.putB = np.array([[-0.79898651, -0.33113412, -0.50196687, 0.00604547],
                              [0.60115704, -0.41873701, -0.68063905, -0.02848487],
                              [0.01519071, -0.84558235, 0.53362884, 0.11008111],
                              [0., 0., 0., 1.]])
        # self.put_middleA = np.array([[0.99254937, -0.12135759, 0.01086691, -0.10877843],
        #                              [0.12099719, 0.99222159, 0.02925741, -0.06116523],
        #                              [-0.01433299, -0.02772456, 0.99951284, -0.1318585],
        #                              [0., 0., 0., 1., ]])
        self.put_middleB = np.array([[-0.99842942, -0.01541606, -0.05386133, -0.10597664],
                                     [0.05410575, -0.01590283, -0.99840857, 0.18961635],
                                     [0.01453498, -0.99975469, 0.01671195, 0.07029574],
                                     [0., 0., 0., 1.]])
        # self.get_middleA = np.array([[0.99254937, -0.12135759, 0.01086691, -0.10712437],
        #                              [0.12099719, 0.99222159, 0.02925741, -0.05977473],
        #                              [-0.01433299, -0.02772456, 0.99951284, -0.03188185],
        #                              [0., 0., 0., 1.]])
        # self.get_middleB = np.array([[0.99782771, 0.02664519, 0.06024854, -0.08751736],
        #                              [0.0397348, 0.4860373, -0.8730343, 0.08593098],
        #                              [-0.0525452, 0.87353178, 0.48392275, 0.00844932],
        #                              [0., 0., 0., 1.]])
        self.watch_aruco_pose = np.array([[0.99864272, -0.0507411, 0.01175022, -0.09899948],
                                          [0.05055016, 0.99859313, 0.01601386, -0.06431557],
                                          [-0.01254625, -0.01539815, 0.99980272, 0.1690991],
                                          [0., 0., 0., 1.]])
        self.close_door1 = np.array([[0.99907925, -0.04256267, -0.00539094, -0.15217543],
                                     [0.0307517, 0.79806095, -0.60179154, -0.09484748],
                                     [0.02991616, 0.60107166, 0.79863501, -0.36161365],
                                     [0., 0., 0., 1.]])
        self.close_door3 = np.array([[0.99959752, -0.02304766, 0.01654071, -0.14592057],
                                     [0.022821, 0.99964482, 0.01376344, -0.14525979],
                                     [-0.01685205, -0.01338042, 0.99976846, -0.13248836],
                                     [0., 0., 0., 1.]])

        self.pose_aruco2 = []
        self.put_middleA = []
        self.get_middleA = []
        self.get_middleB = []
        self.mark2camera = []
        # 过渡矩阵求的示教点
        # self.key_start = get_t(self.pose_aruco2, self.key_start)


class seperation(object):
    def __init__(self):
        self.middle = [1.7606562376022339, 0.35980868339538574, -1.9709006547927856, -1.5158488750457764,
                       -1.4029709100723267, 2.3568732738494873]


class drying(object):
    def __init__(self):
        print("开始烘干机操作")
        # 参数
        self.handle_offset = 0.12  # 路点到门把手距离
        self.handleforce = 60
        # 过渡矩阵
        self.openpower = np.array([[0.99915028, 0.00958035, 0.0400866, -0.64245626],
                                   [-0.00942595, 0.99994742, -0.00403894, 0.33302306],
                                   [-0.04012319, 0.00365765, 0.99918805, -0.00519725],
                                   [0., 0., 0., 1.]])
        self.handle = np.array([[0.99916037, 0.00723318, 0.04032669, -0.54900205],
                                [-0.00696292, 0.99995238, -0.00683811, 0.17952823],
                                [-0.04037423, 0.00655158, 0.99916315, -0.06888135],
                                [0., 0., 0., 1.]])
        self.open_waypoint1 = np.array([[0.73507608, 0.57945417, 0.35198867, -0.66300158],
                                        [-0.66570147, 0.51848966, 0.53666566, 0.02048671],
                                        [0.12847067, -0.62880946, 0.76687284, 0.060322],
                                        [0., 0., 0., 1.]])
        self.open_waypoint2 = np.array([[0.73506929, 0.57946094, 0.3519917, -0.51573811],
                                        [-0.66570687, 0.51847575, 0.5366724, 0.0295821],
                                        [0.12848153, -0.62881469, 0.76686673, -0.24252797],
                                        [0., 0., 0., 1.]])
        self.open_waypoint3 = np.array([[0.86893659, 0.48914397, -0.07541473, -0.79578978],
                                        [0.02202236, 0.11401251, 0.9932352, -0.30375685],
                                        [0.49443323, -0.86471922, 0.08829753, -0.07942949],
                                        [0., 0., 0., 1.]])
        self.open_waypoint4 = np.array([[0.8584633, 0.51282461, 0.00718869, -0.55643852],
                                        [-0.00938296, 0.00168979, 0.99995455, -0.36641834],
                                        [0.51278916, -0.85849174, 0.00626244, 0.12408332],
                                        [0., 0., 0., 1.]])
        self.open_waypoint5 = np.array([[0.85845333, 0.51284122, 0.00719491, 0.23259358],
                                        [-0.00937571, 0.00166529, 0.99995466, -0.36989539],
                                        [0.51280599, -0.85848186, 0.00623783, -0.22511512],
                                        [0., 0., 0., 1.]])
        self.close_waypoint1 = np.array([[0.66370055, -0.5625204, 0.4930237, 0.2223112],
                                         [-0.05488512, 0.62072343, 0.78210616, -0.56891528],
                                         [-0.74598204, -0.54614396, 0.38110049, -0.35505039],
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
        self.jia_j = [2.4160735607147217, -0.5137933492660522, -1.753578782081604, -0.877329409122467,
                      0.8465519547462463, 0.7901036143302917]
        self.jia_move = [0.22035756707191467, -0.5597630143165588, 0.2058982104063034, -1.0989407300949097,
                         -1.5707582235336304, 1.0976780652999878]
        self.waypoint2_j = [2.3617489337921143, -0.2606026530265808, -1.882265329360962, -1.00188148021698,
                            0.7927427887916565, 0.7899118661880493]
        self.waypoint3_j = [0.621639609336853, 0.4628130793571472, -2.2814958095550537, -1.2691535949707031,
                            -0.8961220979690552, 0.764840841293335]
        self.jia_put = np.array([[-0.01233997, 0.99985521, 0.01171665, -0.18250201],
                                 [-0.99990167, -0.01226078, -0.00680684, 0.13653012],
                                 [-0.0066622, -0.01179949, 0.99990819, 0.2687481],
                                 [0., 0., 0., 1.]])

        self.jia_close = [0.2792383134365082, 0.13230015337467194, -1.518208622932434, -0.18123768270015717,
                          1.560260534286499, -0.630290687084198]
        self.tubejia_getp = []

        self.watch_aruco_j = []
        self.pose_aruco2 = []

    def point(self):
        # 移动至识别aruco姿态
        # robotiq(1, 1, 255, 0, 100, 10)
        duco_cobot.movej([-0.02238067239522934, 0.3221063017845154, -2.4003913402557373, 2.0775609016418457,
                          1.5958776473999023, -0.7698469758033752], 20, 10, 0, True)
        self.watch_aruco_j = [0.07509901374578476, 0.07366134226322174, -2.2143123149871826, 2.140777826309204,
                              1.4934604167938232, -0.7687444686889648]
        duco_cobot.movej(self.watch_aruco_j, 20, 10, 0, True)
        # 定位
        aruco.mark()
        sleep(1)
        joint = duco_cobot.get_actual_joints_position()
        print("joint", joint)
        self.pose_aruco2 = duco_cobot.get_tcp_pose()

        self.openpower = get_t(self.pose_aruco2, self.openpower)
        self.handle = get_t(self.pose_aruco2, self.handle)
        self.open_waypoint1 = get_t(self.pose_aruco2, self.open_waypoint1)
        self.open_waypoint2 = get_t(self.pose_aruco2, self.open_waypoint2)
        self.open_waypoint3 = get_t(self.pose_aruco2, self.open_waypoint3)
        self.open_waypoint4 = get_t(self.pose_aruco2, self.open_waypoint4)
        self.open_waypoint5 = get_t(self.pose_aruco2, self.open_waypoint5)
        self.close_waypoint1 = get_t(self.pose_aruco2, self.close_waypoint1)
        self.close_waypoint2 = get_t(self.pose_aruco2, self.close_waypoint2)
        self.close_waypoint3 = get_t(self.pose_aruco2, self.close_waypoint3)

        self.jia_put = get_t(self.pose_aruco2, self.jia_put)

    def open_door(self):
        print("烘干机准备开门")
        # 移动至门把手
        robotiq(1, 1, 0, 0, 100, 10)
        duco_cobot.movel(self.handle, 0.2, 0.2, 0, [], "", "", True)
        duco_cobot.tcp_move([0, 0, self.handle_offset, 0, 0, 0], 0.2, 0.2, 0, True)
        # 合并夹爪，抓住门把手
        robotiq(1, 1, 250, 0, 100, 170)
        sleep(1)
        # 力控开门把手
        duco_cobot.fc_config([True, True, True, False, False, False], [0, 0, 90, 0, 0, 0],
                             [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0.06, 0, 0, 0, 0, 0], [0.01, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, 200, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        print("forcemove stop， 门把手已打开")
        sleep(0.5)
        # 张开夹爪
        duco_cobot.fc_config([True, True, True, False, False, False], [0, 0, 0, 0, 0, 0],
                             [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        robotiq(1, 1, 0, 0, 100, 10)
        sleep(1)
        duco_cobot.fc_stop()
        # 移动至个路点
        duco_cobot.tcp_move([0, 0, -0.1, 0, 0, 0], 0.05, 0.05, 0, True)
        robotiq(1, 1, 255, 0, 100, 10)
        duco_cobot.movel(self.open_waypoint1, 0.2, 0.2, 0, [], "", "", True)
        duco_cobot.movel(self.open_waypoint2, 0.2, 0.2, 0, [], "", "", True)
        duco_cobot.tcp_move([0, 0, -0.12, 0, 0, 0], 0.2, 0.2, 0, True)
        duco_cobot.movel(self.open_waypoint3, 0.2, 0.2, 0, [], "", "", True)
        duco_cobot.movel(self.open_waypoint4, 0.2, 0.2, 0, [], "", "", True)
        duco_cobot.movel(self.open_waypoint5, 0.2, 0.2, 0, [], "", "", True)
        print("烘干机已开门")

    def close_door(self):
        print("烘干机准备关门")
        robotiq(1, 1, 255, 0, 100, 1)
        duco_cobot.movel(self.close_waypoint1, 0.2, 0.2, 0, [], "", "", True)
        duco_cobot.tcp_move([0, 0, 0.2, 0, 0, 0], 0.2, 0.2, 0, True)
        duco_cobot.movel(self.close_waypoint2, 0.2, 0.2, 0, [], "", "", True)
        duco_cobot.movel(self.close_waypoint3, 0.2, 0.2, 0, [], "", "", True)
        # 力控关门
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, -60, 0, 0, 0],
                             [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, -58, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([0, 0, 1])
        duco_cobot.fc_move()
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, 20, 0, 0, 0],
                             [1000, 1000, 1000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.15, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_wait_pos([0.15, 0, 0, 0, 0, 0], [0.01, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 0])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        # 移动至门把手
        self.handle[2] += 0.005
        duco_cobot.movel(self.handle, 0.2, 0.2, 0, [], "", "", True)
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, -70, 0, 0, 0],
                             [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, -70, 0, 0, 0], [0, 0, 5, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([0, 0, 1])
        duco_cobot.fc_move()
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, 20, 0, 0, 0],
                             [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_wait_pos([0.08, 0, 0, 0, 0, 0], [0.01, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 0])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        print("烘干机已关门")

    def open_power(self):
        print("烘干机准备打开电源")
        duco_cobot.movel(self.openpower, 0.2, 0.2, 0, [], "", "", True)
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
        duco_cobot.fc_wait_pos([0.1, 0, 0, 0, 0, 0], [0.01, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 0])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        print("烘干机电源已打开")

    def tubejia_put(self):
        print("夹取试管架放入烘干机")
        # duco_cobot.movej(midpose, 3, 3, 0, True)
        duco_cobot.movej(self.waypoint1_j, 20, 10, 0, True)
        duco_cobot.movej(self.jia_j, 20, 10, 0, True)
        motor.motor_position(159000)
        motor.motor_position(0)
        sleep(2)
        print('移动至jia_j')
        robotiq(1, 1, 0, 0, 100, 10)
        print("开启力控")
        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, -20, 0, 0, 0],
                             [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([-0.11, 0, 0, 0, 0, 0], [0.006, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, -10, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        print('力控停止')
        duco_cobot.movel(self.jia_move, 0.2, 0.2, 0, [], "", "", True)
        print('movel移动至中心')
        robotiq(1, 1, 255, 0, 200, 200)
        sleep(1)
        duco_cobot.movej(self.waypoint2_j, 20, 10, 0, True)
        duco_cobot.movej(self.waypoint3_j, 20, 10, 0, True)
        duco_cobot.movel(self.jia_put, 0.2, 0.2, 0, [], "", "", True)
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
        robotiq(1, 1, 170, 0, 100, 1)
        duco_cobot.tcp_move([0, 0, -0.11, 0, 0, 0], 0.05, 0.05, 0, True)
        self.tubejia_getp = duco_cobot.get_tcp_pose()
        duco_cobot.movej(self.waypoint3_j, 20, 10, 0, True)
        sleep(2)
        robotiq(1, 1, 255, 0, 100, 1)
        duco_cobot.movej(self.jia_close, 20, 10, 0, True)

    def tubejia_get(self):
        duco_cobot.movej(self.jia_close, 20, 10, 0, True)
        duco_cobot.movej(self.waypoint3_j, 20, 10, 0, True)
        robotiq(1, 1, 170, 0, 100, 1)
        duco_cobot.movel(self.tubejia_getp, 0.2, 0.2, 0, [], "", "", True)

        duco_cobot.fc_config([False, False, True, False, False, False], [0, 0, -20, 0, 0, 0],
                             [1000, 1000, 2000, 57.29578, 57.29578, 57.29578],
                             [0.15, 0.15, 0.03, 1.047198, 1.047198, 1.047198], [0, 0, 0, 0, 0, 0], "default",
                             "default", 0)
        duco_cobot.fc_start()
        duco_cobot.fc_wait_pos([0.095, 0, 0, 0, 0, 0], [0.005, 0, 0, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_vel([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], False, 0, 5000)
        duco_cobot.fc_wait_ft([0, 0, -10, 0, 0, 0], [0, 0, 2, 0, 0, 0], False, 0, 500000)
        duco_cobot.fc_wait_logic([1, 0, 2])
        duco_cobot.fc_move()
        duco_cobot.fc_stop()
        robotiq(1, 1, 255, 0, 100, 1)

        duco_cobot.movel(self.jia_put, 0.2, 0.2, 0, [], "", "", True)
        duco_cobot.movej(self.waypoint3_j, 20, 10, 0, True)
        duco_cobot.movej(self.waypoint2_j, 20, 10, 0, True)
        duco_cobot.movel(self.jia_move, 0.2, 0.2, 0, [], "", "", True)
        sleep(1)
        robotiq(1, 1, 0, 0, 100, 1)
        duco_cobot.tcp_move([0, 0, -0.11, 0, 0, 0], 0.05, 0.05, 0, True)
        motor.motor_position(155000)
        robotiq(1, 1, 0, 0, 100, 1)
        duco_cobot.movej(self.waypoint2_j, 20, 10, 0, True)
        duco_cobot.movej(self.waypoint3_j, 20, 10, 0, True)
        duco_cobot.movej(self.jia_close, 20, 10, 0, True)

    def defult(self):
        print("开始烘干机操作")
        # 参数
        self.handle_offset = 0.12  # 路点到门把手距离
        self.handleforce = 60
        # 过渡矩阵
        self.openpower = np.array([[0.99915028, 0.00958035, 0.0400866, -0.64245626],
                                   [-0.00942595, 0.99994742, -0.00403894, 0.33302306],
                                   [-0.04012319, 0.00365765, 0.99918805, -0.00519725],
                                   [0., 0., 0., 1.]])
        self.handle = np.array([[0.99916037, 0.00723318, 0.04032669, -0.54900205],
                                [-0.00696292, 0.99995238, -0.00683811, 0.17952823],
                                [-0.04037423, 0.00655158, 0.99916315, -0.06888135],
                                [0., 0., 0., 1.]])
        self.open_waypoint1 = np.array([[0.73507608, 0.57945417, 0.35198867, -0.66300158],
                                        [-0.66570147, 0.51848966, 0.53666566, 0.02048671],
                                        [0.12847067, -0.62880946, 0.76687284, 0.060322],
                                        [0., 0., 0., 1.]])
        self.open_waypoint2 = np.array([[0.73506929, 0.57946094, 0.3519917, -0.51573811],
                                        [-0.66570687, 0.51847575, 0.5366724, 0.0295821],
                                        [0.12848153, -0.62881469, 0.76686673, -0.24252797],
                                        [0., 0., 0., 1.]])
        self.open_waypoint3 = np.array([[0.86893659, 0.48914397, -0.07541473, -0.79578978],
                                        [0.02202236, 0.11401251, 0.9932352, -0.30375685],
                                        [0.49443323, -0.86471922, 0.08829753, -0.07942949],
                                        [0., 0., 0., 1.]])
        self.open_waypoint4 = np.array([[0.8584633, 0.51282461, 0.00718869, -0.55643852],
                                        [-0.00938296, 0.00168979, 0.99995455, -0.36641834],
                                        [0.51278916, -0.85849174, 0.00626244, 0.12408332],
                                        [0., 0., 0., 1.]])
        self.open_waypoint5 = np.array([[0.85845333, 0.51284122, 0.00719491, 0.23259358],
                                        [-0.00937571, 0.00166529, 0.99995466, -0.36989539],
                                        [0.51280599, -0.85848186, 0.00623783, -0.22511512],
                                        [0., 0., 0., 1.]])
        self.close_waypoint1 = np.array([[0.66370055, -0.5625204, 0.4930237, 0.2223112],
                                         [-0.05488512, 0.62072343, 0.78210616, -0.56891528],
                                         [-0.74598204, -0.54614396, 0.38110049, -0.35505039],
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
        self.jia_j = [2.4160735607147217, -0.5137933492660522, -1.753578782081604, -0.877329409122467,
                      0.8465519547462463, 0.7901036143302917]
        self.jia_move = [0.22035756707191467, -0.5597630143165588, 0.2058982104063034, -1.0989407300949097,
                         -1.5707582235336304, 1.0976780652999878]
        self.waypoint2_j = [2.3617489337921143, -0.2606026530265808, -1.882265329360962, -1.00188148021698,
                            0.7927427887916565, 0.7899118661880493]
        self.waypoint3_j = [0.621639609336853, 0.4628130793571472, -2.2814958095550537, -1.2691535949707031,
                            -0.8961220979690552, 0.764840841293335]
        self.jia_put = np.array([[-0.01233997, 0.99985521, 0.01171665, -0.18250201],
                                 [-0.99990167, -0.01226078, -0.00680684, 0.13653012],
                                 [-0.0066622, -0.01179949, 0.99990819, 0.2687481],
                                 [0., 0., 0., 1.]])

        self.jia_close = [0.2792383134365082, 0.13230015337467194, -1.518208622932434, -0.18123768270015717,
                          1.560260534286499, -0.630290687084198]
        self.tubejia_getp = []

        self.watch_aruco_j = []
        self.pose_aruco2 = []


class prilling(object):
    def __init__(self):
        self.a = 1


motor = motor()
tube = testtube()
# if __name__ == '__main__':
#     # 初始化节点
#     rospy.init_node('robot')
#     try:
#         tube.lidget()
#         tube.lidput()
#         # duco_cobot.movej(midpose, 10, 10, 0, True)
#         # getsample = getsample()
#         # getsample.point()
#         # getsample.get_and_put()
#         #         # 机械臂心跳
#         #         thd_B = threading.Thread(target=hearthread_fun)
#         #         thd_B.daemon = True
#         #         thd_B.start()
#         #         # 激活夹爪
#         #         robotiq(0, 1, 120, 0, 50, 1)
#         #         robotiq(1, 1, 255, 0, 50, 1)
#         #         # 创建对象
#
#         #         # 离心机操作
#         #         cen = centrifuge()
#         #         cen.open()
#         #         cen.put()
#         #         cen.close_door()
#         #         cen.open()
#         #         cen.get()
#         #         cen.close_door()
#         #         duco_cobot.movej(midpose, 5, 5, 0, True)
#
#         #         # 烘干机操作
#         #         duco_cobot.movej(midpose, 5, 5, 0, True)
#         #         dry = drying()
#         #         dry.open_door()
#         #         dry.tubejia_put()
#         #         dry.close_door()
#         #         dry.open_power()
#         #         dry.open_door()
#         #         dry.tubejia_get()
#         #         dry.close_door()
#         #         duco_cobot.movej(midpose, 5, 5, 0, True)
#
#         # Close!
#         duco_cobot.close()
#     except Thrift.TException as tx:
#         print('%s' % tx.message)
#
#     except Exception as e:
#         print(e)
