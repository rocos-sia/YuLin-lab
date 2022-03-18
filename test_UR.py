import rospy
from time import sleep
import numpy as np
import math

#UR机械臂
from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive
#robotiq
import roslib
from torch import true_divide; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg

#自定义immport
from Translate import T
from aruco import rtde_r,rtde_c
from aruco import aruco


def move_endtcp(pose,axis,offset):
    """
    沿TCP移动
    pose: 当前位姿
    axis: [0,0,0,0,0,0]
    offset: [0,0,0,0,0,0]
    """
    #增量判断
    #平移判断
    pose_move=[0.0]*6
    if axis[0]==1:
        pose_move[0]=offset[0]
    if axis[1]==1:
        pose_move[1]=offset[1]
    if axis[2]==1:
        pose_move[2]=offset[2]
    #旋转沿TCP旋转
    if axis[3]==1:
        pose_move[3]=offset[3]/180.0*math.pi
    if axis[4]==1:
        pose_move[4]=offset[4]/180.0*math.pi
    if axis[5]==1:
        pose_move[5]=offset[5]/180.0*math.pi
    #得到增量的pose相对于末端tcp的坐标
    #转换为相对于末端T矩阵
    #T_move为理想位姿相对于末端坐标系的坐标
    #T_pose为末端坐标系相对于基座坐标系的
    T_move=T.rot2T(pose_move)
    T_pose=T.rot2T(pose)
    #矩阵乘法需要用np.dot()
    T_target=np.dot(T_pose,T_move)
    #从T_target到pose_target
    pose_target=T.T2rot(T_target)
    return pose_target

def robotiq(ACT,GTO,PR,ATR,SP,FR):
    """
    rACT: 复位:0，激活:1
    rGTO: 1
    rPR:  位置
    rATR: 0
    rSP:  速度
    rFR:  力
    """

    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
    command = outputMsg.Robotiq2FGripper_robot_output()
    for  i in range(2):
        command = outputMsg.Robotiq2FGripper_robot_output()
        command.rACT = ACT
        command.rGTO = GTO
        command.rPR = PR
        command.rATR = ATR
        command.rSP  = SP
        command.rFR  = FR
        pub.publish(command)
        rospy.sleep(0.1)
    rospy.sleep(3)

def cricle_put(pose_circle,pose_edge,offset,flag):
    """
    pose_circle: 圆心轴的位姿
    pose_edge:   放试管起始点
    offset:      旋转角度
    flag:        旋转半圆标志位
    """

    pose_middle=[-0.62625,-0.096,0.64434,0.224,3.253,0.038]

    rtde_c.moveL(pose_edge,0.2,0.1)
    testtude_force=8                #力判断
    testtude_dist=0.025             #位置判断
    offset=offset*0.0175            #弧度转角度
    offset_sum=0                    #初始角度累加
    goal2baselink=T.rot2T(pose_circle)      #目标相对于基座的T
    eelink2baselink=T.rot2T(pose_edge)          #当前位姿转换成T
    eelink2goal=np.dot(np.linalg.inv(goal2baselink),eelink2baselink)   #goal2baselink*eelink2goal=eelink2baselink
    #pose到欧拉角
    P,M=T.T2euler_2(eelink2goal)         #旋转平移矩阵

    radius=math.sqrt(math.pow(P[0],2)+math.pow(P[1],2))   #得R
    angle=math.atan2(P[1],P[0])                           #的rad
    
    for i in range(6):
        
        tran=[radius*math.cos(angle+offset_sum),radius*math.sin(angle+offset_sum),P[2]]
        euler=[M[0],M[1],M[2]+offset_sum]
        rot=T.euler2rot(euler)
        eelink2goal=[tran[0],tran[1],tran[2],rot[0],rot[1],rot[2]]   #转动后目标相对于基座的T
        eelink2goal=T.rot2T(eelink2goal)
        target=np.dot(goal2baselink,eelink2goal)
        target=T.T2rot(target)       #目标位姿
        ret=rtde_c.moveL(target,0.2,0.1)
        print("move:",ret)
        sleep(1)
        if ret==False:
            offset_sum=offset_sum+offset
            i=i-1
        else:
            task_frame=rtde_r.getActualTCPPose()                      #the force frame relative to the base frame
            if flag==0:
                selection_vctor=[0,0,1,0,0,0]                         #A 6d vector of 0s and 1s.
                wrench=[0,0,testtude_force+2,0,0,0]                   #The forces/torques the robot will apply to its environment
            else:
                selection_vctor=[0,1,0,0,0,0]                         #A 6d vector of 0s and 1s.
                wrench=[0,testtude_force+2,0,0,0,0]                   #The forces/torques the robot will apply to its environment

            force_type=2                                              #The force frame is not transformed
            limits=[0.1,0.1,0.1,0.1,0.1,0.1]   

            rtde_c.forceMode(task_frame,selection_vctor,wrench,force_type,limits)
            sleep(0.2)
            pose_edge=task_frame

            #力和位置判断
            while (abs(rtde_r.getActualTCPForce()[2]) < testtude_force) and (abs(abs(rtde_r.getActualTCPPose()[2])-abs(pose_edge[2]))<=testtude_dist):
                    print("moving")
                    sleep(0.05)
            print("stop start")
            rtde_c.forceModeStop()    #停止forcemode
            print("stop end")
            sleep(1)

            #该点成功放入
            if abs(rtde_r.getActualTCPPose()[2]-pose_edge[2])>=testtude_dist-0.005:   
                print("试管放入成功")
                print("robotiq open")
                robotiq(1, 1, 150, 0, 10, 1)
                sleep(2)  
                rtde_c.moveL(pose_edge,0.2,0.1)
                rtde_c.moveL(pose_middle,0.2,0.1)
                sleep(2)
                offset=60*0.0175 
                robotiq(1, 1, 230, 0, 10, 1)
                sleep(3)
            else:
                print("试管没有放入，继续旋转到下一个角度")
                rtde_c.moveL(pose_edge,0.2,0.1)
        offset_sum=offset_sum-offset

if __name__ == '__main__':

    #初始化节点
    rospy.init_node('move')

    #ur连接
    # rtde_r = rtde_receive.RTDEReceiveInterface("192.168.3.101")
    # rtde_c = RTDEControl("192.168.3.101", RTDEControl.FLAG_USE_EXT_UR_CAP)

    #定位
    aruco.mark()
    sleep(1)

#测试
    #二维码重复定位测试
    # translate=[-0.09187276476113676, 0.09429355196217293, 0.021041857593783464, 0.058704839558895286, -0.03245640438618253, 0.18780223780667088]
    # aruco_pose=rtde_r.getActualTCPPose()
    # goalpose=get_t(aruco_pose, translate)

    # rtde_c.moveJ_IK(goalpose,0.2,0.1)

    #离心机示教点测试
    # poseA_1=[-0.63299,-0.19188,0.55257,0.110,-2.892,-0.699]
    # poseA_2=[-0.60522,0.00966,0.55396,0.413,3.283,-0.737]
    # poseA_3=[-0.50434,-0.09773,0.43231,2.415,2.727,-1.333]

    # poseB_1=[-0.60075,-0.17957,0.55281,0.171,3.412,0.664]
    # poseB_2=[-0.63603,0.03455,0.54602,0.383,3.113,-0.848]
    # poseB_3=[-0.50751,-0.09363,0.43240,2.485,2.727,-1.244]

    # poseC_1=[-0.56696,-0.14615,0.55137,0.719,-2.636,-0.527]
    # poseC_2=[-0.66065,0.04579,0.54429,0.516,2.947,-0.856]
    # poseC_3=[-0.53537,-0.01494,0.44305,1.903,2.714,-1.339]

    # pose_middle=[-0.62625,-0.096,0.64434,0.224,3.253,0.038]

    #标志位
    # flagABC=0  #试管ABC点标志位
    
    #放试管
    # put_testtube(poseA_1,poseB_1,poseC_1,pose_middle)
    # put_testtube1(poseA_2, poseB_2,poseC_2,pose_middle)
    # put_testtube3(poseA_3, poseB_3,poseC_3,pose_middle)

    #画圆测试
    # rot_pose=[-0.67607,-0.08991,0.48074,2.304,2.161,0.013]
    # poseA_1=[-0.63299,-0.19188,0.55257,0.110,-2.892,-0.699]
    # cricle_put(rot_pose, poseA_1, 20,0)

    #报错退出
    # assert True



    
    








