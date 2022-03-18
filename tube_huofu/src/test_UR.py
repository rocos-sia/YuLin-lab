import rospy
from time import sleep
import numpy as np
from Translate import T
#UR机械臂
from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive
#robotiq
import roslib
from torch import true_divide; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
import math

#自定义immport
from aruco import rtde_r,rtde_c
from aruco import aruco

def move_endtcp(pose,axis,offset):
    #从当前点pose->T_pose

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

def put_testtube(poseA,poseB,poseC,pose_middle):
    #思路AB两点，各对应一套动作
    #移动到A点，然后forcemode下沿TCP末端Z方向移动
    #位置判断
    #   if机械臂末端受力大于3N转向B点，
    #   如果位置达到要求，直接松开夹爪

        #变量
        testtude_force=10                       #试管力判断大小
        testtude_dist=0.025                      #试管移动位移大小

        #标志位
        global flagABC        
        #移动至中间点
        rtde_c.moveL(pose_middle,0.1,0.1)  
        #移动至A点
        rtde_c.moveL(poseA,0.1,0.1)  

        #设置foecemode
        task_frameA=rtde_r.getActualTCPPose()    #the force frame relative to the base frame
        selection_vctor=[0,0,1,0,0,0]            #A 6d vector of 0s and 1s.
        wrench=[0,0,testtude_force+4,0,0,0]      #The forces/torques the robot will apply to its environment
        force_type=2                             #The force frame is not transformed
        limits=[0.1,0.1,0.1,0.1,0.1,0.1]           
        rtde_c.forceMode(task_frameA,selection_vctor,wrench,force_type,limits)
        sleep(0.2)

        #力和位置判断
        while (abs(rtde_r.getActualTCPForce()[2]) < testtude_force) and (abs(abs(rtde_r.getActualTCPPose()[2])-abs(poseA[2]))<=testtude_dist):
                print("moving")
                sleep(0.05)
        print("stop start")
        rtde_c.forceModeStop()    #停止forcemode
        print("stop end")
        sleep(1)

        #A点成功放入
        if abs(rtde_r.getActualTCPPose()[2]-poseA[2])>=testtude_dist-0.005:   
            print("试管A点放入成功")
            print("robotiq open")
            robotiq(1, 1, 150, 0, 10, 1)
            sleep(2)  
            rtde_c.moveL(poseA,0.1,0.1)
            sleep(0.1)
            flagABC=0

        #A点未放入，进行B点放入
        else :
            #移动到B点
            rtde_c.moveL(poseA,0.1,0.1)
            rtde_c.moveL(poseB,0.1,0.1)            

            #设这B点forcemode
            task_frameB=rtde_r.getActualTCPPose()
            rtde_c.forceMode(task_frameB,selection_vctor,wrench,force_type,limits)
            sleep(0.2)

            #力判断和位置判断
            while (abs(rtde_r.getActualTCPForce()[2]) < testtude_force) and (abs(abs(rtde_r.getActualTCPPose()[2])-abs(poseB[2]))<=testtude_dist):
                print("moveing")
                sleep(0.05)
            print("stop start")
            rtde_c.forceModeStop()
            print("stop end")
            sleep(1)
            #B点放入成功
            if abs(rtde_r.getActualTCPPose()[2]-poseB[2])>=testtude_dist-0.005:
                print("试管B点放入成功")
                print("robotiq open")
                robotiq(1, 1, 150, 0, 10, 1)
                sleep(2)
                rtde_c.moveL(poseB,0.1,0.1)
                sleep(0.1)
                flagABC=1
            else:
                rtde_c.moveL(poseB,0.1,0.1)
                rtde_c.moveL(poseC,0.1,0.1)            

                #设这B点forcemode
                task_frameC=rtde_r.getActualTCPPose()
                wrench=[0,0,testtude_force+4,0,0,0]      #The forces/torques the robot will apply to its environment
                rtde_c.forceMode(task_frameC,selection_vctor,wrench,force_type,limits)
                sleep(0.2)

                #力判断和位置判断
                while (abs(rtde_r.getActualTCPForce()[2]) < testtude_force) and (abs(abs(rtde_r.getActualTCPPose()[2])-abs(poseC[2]))<=testtude_dist):
                    print("moveing")
                    sleep(0.05)
                print("stop start")
                rtde_c.forceModeStop()
                print("stop end")
                sleep(1)
                #B点放入成功
                if abs(rtde_r.getActualTCPPose()[2]-poseC[2])>=testtude_dist-0.005:
                    print("试管C点放入成功")
                    print("robotiq open")
                    robotiq(1, 1, 150, 0, 10, 1)
                    sleep(2)
                    rtde_c.moveL(poseC,0.1,0.1)
                    sleep(0.1)
                    flagABC=2
                else:
                    rospy.signal_shutdown("error")
        rtde_c.moveL(pose_middle,0.1,0.1)  
        robotiq(1, 1, 230, 0, 10, 5) 
        sleep(2)
               
def output_testtube():
    TCP_offset=0.1

def open_door():

    handle_dist=0.02                                                 #门把手移动位移        
    middle_pose1=[0.305,-0.288,0.445,1.117,1.860,-0.880]             #开门的中间示教点1
    middle_pose2=[0.392,-0.515,0.492,1.485,1.682,-1.659]             #开门的中间示教点2
    middle_pose3=[]                                                  #开门的中间示教点3
    end_pose=[-0.201,-0.385,0.552,2.245,0.519,-1.924]                #开门的示教点终点
    handle_w=[]                                                      #门把手路点
    handle=[0.2991,-0.70727,0.39836,1.429,1.617,-0.874]              #门把手位置
    handleforce=30                                                   #门把手的力
    axis=[-0.378,-0.77357-0.255,0.36746,0,0,0]                       #门的轴心
    
    #移动至门把手
    robotiq(1, 1, 100, 0, 10, 10)
    print("移动至门把手")
    rtde_c.moveL(handle_w,0.2,0.1)
    rtde_c.moveL(handle,0.2,0.1)

    #设置foecemode
    task_frameA=rtde_r.getActualTCPPose()     #the force frame relative to the base frame
    selection_vctor=[1,1,1,1,1,1]            #A 6d vector of 0s and 1s.
    wrench=[0,0,0,0,0,0]                    #The forces/torques the robot will apply to its environment
    force_type=2                             #The force frame is not transformed
    limits=[0.1,0.1,0.1,0.1,0.1,0.1]           
    rtde_c.forceMode(task_frameA,selection_vctor,wrench,force_type,limits)
    sleep(0.2)

    #合并夹爪，抓住门把手
    robotiq(1, 1, 200, 0, 10, 170)

    #拉门把手
    print("拉门把手，请输入")
    input()
    task_frameB=rtde_r.getActualTCPPose()  
    wrench=[0,0,handleforce,0,0,0]
    rtde_c.forceMode(task_frameB,selection_vctor,wrench,force_type,limits)
    pose_now=task_frameB
    
    #位置判断
    while abs(rtde_r.getActualTCPPose()[2])-abs(pose_now)<handle_dist:
        sleep(0.1)
    rtde_c.forceModeStop()             #forcemode停止
    sleep(1)
    task_frameC=rtde_r.getActualTCPPose()  
    wrench=[0,0,0,0,0,0]
    rtde_c.forceMode(task_frameC,selection_vctor,wrench,force_type,limits)
    robotiq(1, 1, 162, 0, 10, 50)     #适当松开夹爪

    #画圆开门
    print("画圆推门")
    input()
    offset=10*0.0175   #弧度转角度
    offset_sum=0       #初始角度累加
    goal2baselink=T.rot2T(axis)     #目标相对于基座的T
    eelink2baselink=rtde_r.getActualTCPPose()   #获得当前位姿
    eelink2baselink=T.rot2T(eelink2baselink)    #当前位姿转换成T
    goal2eelink=np.dot(np.linalg.inv(goal2baselink),eelink2baselink)  #goal2baselink*eelink2goal=eelink2baselink
    P,M=T.T2rot_2(goal2eelink)   #旋转平移矩阵

    radius=math.sqrt(math.pow(P[0],2)+math.pow(P[1],2))   #得R
    angle=math.atan2(P[1],P[0])                           #的rad

    for i in range(2):
        offset_sum=offset_sum+offset
        tran=[radius*math.cos(angle+offset_sum),radius*math.sin(angle+offset_sum),P[2]]
        rot=[M[0],M[1],M[2]+offset_sum]
        eelink2goal=[tran[0],tran[1],tran[2],rot[0],rot[1],rot[2]]
        eelink2goal=T.rot2T(eelink2goal)
        target=np.dot(goal2baselink,eelink2goal)
        target=T.T2rot(target)       #目标位姿

        ret=rtde_c.moveL(target,0.1,0.1)
        print("move:",ret)
        if ret==False:
            offset_sum=offset_sum-offset
        sleep(0.5)
    sleep(1)

    print("示教点1到达，张开夹爪请输入")
    input()
    #张开夹爪
    robotiq(1, 1, 0, 0, 10, 10)
    sleep(2)
    #向下移动10cm
    pose_now=rtde_r.getActualTCPPose()  
    pose_now[2]=pose_now[2]-0.1
    rtde_c.moveL(pose_now,0.1,0.1)
    #合并夹爪
    robotiq(1, 1, 255, 0, 10, 10)
    sleep(1)
    #移至中间点推门
    print("移动至中间点，请输入")
    input()
    rtde_c.moveJ_IK(middle_pose1,0.1,0.1)
    sleep(0.5)
    rtde_c.moveJ_IK(middle_pose2,0.1,0.1)
    sleep(0.5)
    #最后的推门点
    rtde_c.moveJ_IK(end_pose,0.1,0.1)
    sleep(0.5)

def close_door():

    #示教一个点画圆
    axis=[-0.378,-0.77357-0.255,0.36746,0.164,-2.320,2.296]        #门的轴心
    door=[]                         #门
    handle=[]                       #门把手
    maxforce_door=15                #推门的最大力
    maxforce_handle=40              #推门把手的最大力
    offset=10*0.0175                #弧度转角度
    offset_sum=0                    #初始角度累加

    goal2baselink=T.rot2T(axis)                 #目标相对于基座的T
    eelink2baselink=rtde_r.getActualTCPPose()   #获得当前位姿
    eelink2baselink=T.rot2T(eelink2baselink)    #当前位姿转换成T
    goal2eelink=np.dot(np.linalg.inv(goal2baselink),eelink2baselink)  #goal2baselink*eelink2goal=eelink2baselink
    P,M=T.T2rot_2(goal2eelink)                  #旋转平移矩阵

    radius=math.sqrt(math.pow(P[0],2)+math.pow(P[1],2))   #得R
    angle=math.atan2(P[1],P[0])                           #的rad

    for i in range(8):
        offset_sum=offset_sum+offset
        tran=[radius*math.cos(angle+offset_sum),radius*math.sin(angle+offset_sum),P[2]]
        rot=[M[0],M[1],M[2]]                               #末端姿态不随之改变
        eelink2goal=[tran[0],tran[1],tran[2],rot[0],rot[1],rot[2]]
        eelink2goal=T.rot2T(eelink2goal)
        target=np.dot(goal2baselink,eelink2goal)
        target=T.T2rot(target)                             #目标位姿
        ret=rtde_c.moveL(target,0.1,0.1,True)
        sleep(1)              

    #移动至门
    rtde_c.moveL(door,0.2,0.1)

    #开启forcemode
    task_frameA=rtde_r.getActualTCPPose()     #the force frame relative to the base frame
    selection_vctor=[1,1,1,1,1,1]            #A 6d vector of 0s and 1s.
    wrench=[0,0,maxforce_door+5,0,0,0]                    #The forces/torques the robot will apply to its environment
    force_type=2                             #The force frame is not transformed
    limits=[0.1,0.1,0.1,0.1,0.1,0.1]           
    rtde_c.forceMode(task_frameA,selection_vctor,wrench,force_type,limits)
    sleep(0.2)

    #力判断门关好  注意获得的力是基于什么坐标系的
    while abs(rtde_r.getActualTCPForce()[0]) < maxforce_door:
        print("pushing")
    print("stop start")
    rtde_c.forceModeStop()                   #forcemode停止
    print("stop end")
    print("push over")
    sleep(1)

    #移动至门把手
    rtde_c.moveL(handle,0.1,0.1,True)                    #示教推门把手的点door
    wrench=[0,0,maxforce_handle+5,0,0,0]                    #The forces/torques the robot will apply to its environment
    rtde_c.forceMode(task_frameA,selection_vctor,wrench,force_type,limits)
    
    #最好位置判断 注意获得的力是基于什么坐标系的
    while abs(rtde_r.getActualTCPForce()[0]) < maxforce_handle:
        print("pushing")
    print("stop start")
    rtde_c.forceModeStop()
    print("stop end")
    print("push over")
    sleep(0.05)

def key():

    keyforce=10                             #按键力大小
    #移动至按键上方路点
    keyup_pose=get_t(pose1, pose2)
    rtde_c.moveL(keyup_pose,0.2,0.1)

    #开启forcemode
    task_frame=rtde_r.getActualTCPPose()     #the force frame relative to the base frame
    selection_vctor=[1,1,1,1,1,1]            #A 6d vector of 0s and 1s.
    wrench=[0,0,keyforce,0,0,0]                    #The forces/torques the robot will apply to its environment
    force_type=2                             #The force frame is not transformed
    limits=[0.1,0.1,0.1,0.1,0.1,0.1]           
    rtde_c.forceMode(task_frame,selection_vctor,wrench,force_type,limits)
    sleep(0.2)

    #受力判断，停止
    while abs(rtde_r.getActualTCPForce()[2]) < 3:
        print("moving")
        sleep(0.05)
    print("stop start")
    rtde_c.forceModeStop()    #停止forcemode
    print("stop end")
    sleep(1)

def cricle(rot_pose,eelink2baselink,offset):

    offset=offset*0.0175            #弧度转角度
    offset_sum=0                    #初始角度累加
    goal2baselink=T.rot2T(rot_pose)      #目标相对于基座的T
    eelink2baselink=T.rot2T(eelink2baselink)          #当前位姿转换成T
    eelink2goal=np.dot(np.linalg.inv(goal2baselink),eelink2baselink)   #goal2baselink*eelink2goal=eelink2baselink
    #pose到欧拉角
    P,M=T.T2rot_2(eelink2goal)         #旋转平移矩阵

    radius=math.sqrt(math.pow(P[0],2)+math.pow(P[1],2))   #得R
    angle=math.atan2(P[1],P[0])                           #的rad

    
    for i in range(20):
        offset_sum=offset_sum+offset
        tran=[radius*math.cos(angle+offset_sum),radius*math.sin(angle+offset_sum),P[2]]
        rot=[M[0],M[1],M[2]+offset_sum]
        eelink2goal=[tran[0],tran[1],tran[2],rot[0],rot[1],rot[2]]   #转动后目标相对于基座的T
        eelink2goal=T.rot2T(eelink2goal)
        target=np.dot(goal2baselink,eelink2goal)
        target=T.T2rot(target)       #目标位姿
        ret=rtde_c.moveL(target,0.2,0.1)
        print("move:",ret)
        if ret==False:
            offset_sum=offset_sum-offset
        sleep(1)
    return target

def get_T(pose1,pose2):
    #UR 得到过渡矩阵
    pose1_T=T.rot2T(pose1)      #视觉定位点位姿
    pose2_T=T.rot2T(pose2)       #pose2是目标点
    transition_T=np.dot(np.linalg.inv(pose1_T), pose2_T)
    transition=T.T2rot(transition_T)
    return transition

def get_t(pose1,pose2):
    #UR 相对于二维码的姿态
    pose1_T=T.rot2T(pose1)      #视觉定位点位姿
    pose2_T=T.rot2T(pose2)       #过渡矩阵
    transition_T=np.dot(pose1_T, pose2_T)
    transition=T.T2rot(transition_T)
    return transition

def put_testtube1(poseA,poseB,poseC,pose_middle):

    global flagABC
    if flagABC==1:
        poseA=poseB
    elif flagABC==2:
        poseA=poseC
    else:
        poseA=poseA

     #变量
    testtude_force=8                        #试管力判断大小
    testtude_dist=0.025                      #试管移动位移大小

    #标志位
    flagABC=0        
    #移动至A点
    rtde_c.moveL(poseA,0.1,0.1)  

    #设置foecemode
    task_frameA=rtde_r.getActualTCPPose()    #the force frame relative to the base frame
    selection_vctor=[0,0,1,0,0,0]            #A 6d vector of 0s and 1s.
    wrench=[0,0,testtude_force+4,0,0,0]      #The forces/torques the robot will apply to its environment
    force_type=2                             #The force frame is not transformed
    limits=[0.1,0.1,0.1,0.1,0.1,0.1]   

    # if flagABC==2:
    #      wrench=[0,testtude_force+4,0,0,0,0] 

    rtde_c.forceMode(task_frameA,selection_vctor,wrench,force_type,limits)
    sleep(0.2)

    #力和位置判断
    while (abs(rtde_r.getActualTCPForce()[2]) < testtude_force) and (abs(abs(rtde_r.getActualTCPPose()[2])-abs(poseA[2]))<=testtude_dist):
            print("moving")
            sleep(0.05)
    print("stop start")
    rtde_c.forceModeStop()    #停止forcemode
    print("stop end")
    sleep(1)

    #A点成功放入
    if abs(rtde_r.getActualTCPPose()[2]-poseA[2])>=testtude_dist-0.005:   
        print("试管A点放入成功")
        print("robotiq open")
        robotiq(1, 1, 150, 0, 10, 1)
        sleep(2)  
        rtde_c.moveL(poseA,0.1,0.1)
        rtde_c.moveL(pose_middle,0.1,0.1)  
        sleep(0.1)
        flagABC=0
    else:
        print("error")
    rtde_c.moveL(pose_middle,0.1,0.1)  
    robotiq(1, 1, 230, 0, 10, 40) 
    sleep(2)

def put_testtube2(poseA,poseB,poseC,pose_middle):
    
    global flagABC
    if flagABC==1:
        poseA=poseB
    elif flagABC==2:
        poseA=poseC
    else:
        poseA=poseA

     #变量
    testtude_force=8                        #试管力判断大小
    testtude_dist=0.025                      #试管移动位移大小

    #标志位
    flagABC=0        
    #移动至A点
    rtde_c.moveL(poseA,0.1,0.1)  

    #设置foecemode
    task_frameA=rtde_r.getActualTCPPose()    #the force frame relative to the base frame
    selection_vctor=[0,1,0,0,0,0]            #A 6d vector of 0s and 1s.
    wrench=[0,testtude_force+4,0,0,0,0]      #The forces/torques the robot will apply to its environment
    force_type=2                             #The force frame is not transformed
    limits=[0.1,0.1,0.1,0.1,0.1,0.1]   

    rtde_c.forceMode(task_frameA,selection_vctor,wrench,force_type,limits)
    sleep(0.2)

    #力和位置判断
    while (abs(rtde_r.getActualTCPForce()[2]) < testtude_force) and (abs(abs(rtde_r.getActualTCPPose()[2])-abs(poseA[2]))<=testtude_dist):
            print("moving")
            sleep(0.05)
    print("stop start")
    rtde_c.forceModeStop()    #停止forcemode
    print("stop end")
    sleep(1)

    #A点成功放入
    if abs(rtde_r.getActualTCPPose()[2]-poseA[2])>=testtude_dist-0.005:   
        print("试管A点放入成功")
        print("robotiq open")
        robotiq(1, 1, 150, 0, 10, 1)
        sleep(2)  
        rtde_c.moveL(poseA,0.1,0.1)
        rtde_c.moveL(pose_middle,0.1,0.1)  
        sleep(0.1)
        flagABC=0
    else:
        print("error")
    rtde_c.moveL(pose_middle,0.1,0.1) 
    sleep(1)

def put_testtube3(pose_circle,pose_edge,offset):
    # rtde_c.moveL(pose_circle,0.1,0.1)
    # rtde_c.moveL(pose_edge,0.1,0.1)
    testtude_force=8
    testtude_dist=0.025
    offset=offset*0.0175            #弧度转角度
    offset_sum=0                    #初始角度累加
    goal2baselink=T.rot2T(pose_circle)      #目标相对于基座的T
    eelink2baselink=T.rot2T(eelink2baselink)          #当前位姿转换成T
    eelink2goal=np.dot(np.linalg.inv(goal2baselink),eelink2baselink)   #goal2baselink*eelink2goal=eelink2baselink
    #pose到欧拉角
    P,M=T.T2euler_2(eelink2goal)         #旋转平移矩阵

    radius=math.sqrt(math.pow(P[0],2)+math.pow(P[1],2))   #得R
    angle=math.atan2(P[1],P[0])                           #的rad
    for i in range(20):
        offset_sum=offset_sum+offset
        tran=[radius*math.cos(angle+offset_sum),radius*math.sin(angle+offset_sum),P[2]]
        euler=[M[0],M[1],M[2]+offset_sum]
        rot=T.euler2rot(euler)
        eelink2goal=[tran[0],tran[1],tran[2],rot[0],rot[1],rot[2]]   #转动后目标相对于基座的T
        eelink2goal=T.rot2T(eelink2goal)
        target=np.dot(goal2baselink,eelink2goal)
        target=T.T2rot(target)       #目标位姿
        ret=rtde_c.moveL(target,0.2,0.1)
        print("move:",ret)
        if ret==False:
            offset_sum=offset_sum-offset
        else:
            sleep(1)
            task_frame=rtde_r.getActualTCPPose()    #the force frame relative to the base frame
            selection_vctor=[0,0,1,0,0,0]            #A 6d vector of 0s and 1s.
            wrench=[0,0,10,0,0,0]      #The forces/torques the robot will apply to its environment
            force_type=2                             #The force frame is not transformed
            limits=[0.1,0.1,0.1,0.1,0.1,0.1]   

            # if flagABC==2:
            #      wrench=[0,testtude_force+4,0,0,0,0] 

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

            #A点成功放入
            if abs(rtde_r.getActualTCPPose()[2]-pose_edge[2])>=testtude_dist-0.005:   
                print("试管放入成功")
                print("robotiq open")
                robotiq(1, 1, 150, 0, 10, 1)
                sleep(2)  
                rtde_c.moveL(poseA,0.1,0.1)
                sleep(0.1)
                break
                
            else:
                print("试管没有放入，继续旋转到下一个角度")
                continue

    print("试管已经放入")
    sleep(3)
    robotiq(1, 1, 230, 0, 10, 1)
    return target

if __name__ == '__main__':

    #初始化节点
    rospy.init_node('move')

    #ur连接
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.3.101")
    rtde_c = RTDEControl("192.168.3.101", RTDEControl.FLAG_USE_EXT_UR_CAP)

    #定位
    # aruco.mark()
    # sleep(1)

    #示教点
    #离心机示教点
    poseA_1=[-0.63299,-0.19188,0.55257,0.110,-2.892,-0.699]
    poseA_2=[-0.60522,0.00966,0.55396,0.413,3.283,-0.737]
    poseA_3=[-0.50434,-0.09773,0.43231,2.415,2.727,-1.333]

    poseB_1=[-0.60075,-0.17957,0.55281,0.171,3.412,0.664]
    poseB_2=[-0.63603,0.03455,0.54602,0.383,3.113,-0.848]
    poseB_3=[-0.50751,-0.09363,0.43240,2.485,2.727,-1.244]

    poseC_1=[-0.56696,-0.14615,0.55137,0.719,-2.636,-0.527]
    poseC_2=[-0.66065,0.04579,0.54429,0.516,2.947,-0.856]
    poseC_3=[-0.53537,-0.01494,0.44305,1.903,2.714,-1.339]

    pose_middle=[-0.62625,-0.096,0.64434,0.224,3.253,0.038]

    #标志位
    flagABC=0  #试管AB点标志位
    
    #放试管
    # put_testtube(poseA_1,poseB_1,poseC_1,pose_middle)
    # put_testtube1(poseA_2, poseB_2,poseC_2,pose_middle)
    # put_testtube3(poseA_3, poseB_3,poseC_3,pose_middle)

#测试
    #二维码重复定位测试
    # translate=[-0.09187276476113676, 0.09429355196217293, 0.021041857593783464, 0.058704839558895286, -0.03245640438618253, 0.18780223780667088]
    # aruco_pose=rtde_r.getActualTCPPose()
    # goalpose=get_t(aruco_pose, translate)

    # rtde_c.moveJ_IK(goalpose,0.2,0.1)

    #画圆测试
    rot_pose=[0.1207,0.4757,0.297,3.142,0,0]
    rtde_c.moveL(rot_pose,0.2,0.1)
    sleep(8)
    pose=[0.09982,0.47513,0.2992,3.119,0,-0.330]
    rtde_c.moveL(pose,0.2,0.1)
    sleep(5)
    cricle(rot_pose, pose, 20)



    
    








