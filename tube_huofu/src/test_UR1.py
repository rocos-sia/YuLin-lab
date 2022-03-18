
import rospy
from time import sleep
import numpy as np
from Translate import T
import math
#UR机械臂
from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive
#from UR_robot import UR_robot as UR
from numpy.linalg import inv
import roslib
# from torch import true_divide; roslib.load_manifest('robotiq_2f_gripper_control')
# from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
#注释：ur从旋转矢量到T矩阵，新松是从欧拉角到T矩阵
#该程序为ur的处理
#得到从点到基座得T矩阵
def rv_T(pose_teach):
    r=R.from_rotvec([pose_teach[3],pose_teach[4],pose_teach[5]])
    rotation=r.as_matrix()
    translation=np.array([pose_teach[0],pose_teach[1],pose_teach[2]])
    one=np.array([0,0,0,1])
    t = np.concatenate(
        [rotation.reshape(3, 3), translation.reshape(3, 1)], axis=1)
    T_teach_base= np.concatenate([t.reshape(3, 4), one.reshape(1, 4)], axis=0)
    return T_teach_base
#得到示教点相对于二维码的相对T矩阵变换
# def teach_aru(T_teach,T_aru):
#     T_inv=inv(T_aru)
#     T_relative=np.dot(T_inv,T_teach)
#     return T_relative
#当机械臂运动确定二维码的位姿可以得到二维码相对于基座的T_aru矩阵
#T_aru*T_relative =T_end_base
#获得末端相对于基座的T矩阵
def T_rv(T_end_base):
    end_rotation=T_end_base[0:3,0:3]
    end_translation=T_end_base[0:3,3]
    r=R.from_matrix(end_rotation)
    rv=r.as_rotvec()
    rv_end=[end_translation[0],end_translation[1],end_translation[2],rv[0],rv[1],rv[2]]
    return rv_end
#函数参数：T_aruco为二维码相对于基座的T矩阵，goal为目标点相对于基座的位姿
#函数功能：求取过渡矩阵
def goal_aruco(T_aruco,goal):
    T_goal=rv_T(goal)
    T_inv=inv(T_aruco)
    T_transition=np.dot(T_inv,T_goal)
    return T_transition

#知道过渡矩阵：下次直接输入T_aruco得到当前基座坐标系的pose
#求取目标点
def goal_pose(T_aruco,T_transition):
    T_goal=np.dot(T_aruco,T_transition)
    pose_goal=T_rv(T_goal)
    return pose_goal

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

def put_testtube_endforce(pose_A,pose_B,pose_C,pose_middle,wrench):
    #思路AB两点
    #移动到A点，然后沿TCP末端Z方向移动
    #位置判断
    #   if机械臂末端受力大于3N转向B点，
    #   如果位置达到要求，直接松开夹爪
        TCP_offset=0.03                #移动TCP_z轴距离
        
        #中间点
        rtde_c.moveL(pose_middle,0.1,0.1)  
        #A点
        rtde_c.moveL(pose_A,0.1,0.1)            
        task_frame=rtde_r.getActualTCPPose()
        selection_vctor=[1,1,1,1,1,1]
        # wrench=[0,0,8,0,0,0]
        force_type=2
        limits=[1,1,1,1,1,1]
        # target=move_endtcp(pose_A,[0,0,1,0,0,0],[0,0,TCP_offset,0,0,0])  #沿tcp移动TCP_offset距离

        # rtde_c.moveL(target,0.02,0.01,True)
        rtde_c.forceMode(task_frame,selection_vctor,wrench,force_type,limits)
        sleep(0.2)

        while (abs(rtde_r.getActualTCPForce()[2]) < 5) and (abs(abs(rtde_r.getActualTCPPose()[2])-abs(pose_A[2]))<=0.02):
                print("1")
                sleep(0.1)
        print("stopL start")
        rtde_c.forceModeStop()
        print("stopL end")
        sleep(1)

        if abs(rtde_r.getActualTCPPose()[2]-pose_A[2])>=0.02:
            print("插管成功，打开夹爪输入")
            # robotiq(1, 1, 150, 0, 10, 1)
            print("robotiq open")
            sleep(2)
            rtde_c.moveL(pose_A,0.1,0.1)
            rtde_c.moveL(pose_middle,0.1,0.1)
            sleep(0.1)
        else:
            #走到B点开始进行判断运行
            rtde_c.moveL(pose_A,0.1,0.1)
            rtde_c.moveL(pose_B,0.1,0.1)            

            # target=move_endtcp(pose_B,[0,0,1,0,0,0],[0,0,TCP_offset,0,0,0])  #沿tcp移动TCP_offset距离

            # rtde_c.moveL(target,0.02,0.01,True)
            task_frame1=rtde_r.getActualTCPPose()
            rtde_c.forceMode(task_frame1,selection_vctor,wrench,force_type,limits)
            sleep(0.2)
            while (abs(rtde_r.getActualTCPForce()[2]) < 5) and (abs(abs(rtde_r.getActualTCPPose()[2])-abs(pose_B[2]))<=0.02):
                print("2")
                sleep(0.01)

            print("stopL start")
            # 
            rtde_c.forceModeStop()
            print("stopL end")
            sleep(1)
            if abs(rtde_r.getActualTCPPose()[2]-pose_B[2])>=0.02:
                print("插管成功，打开夹爪输入")
            # robotiq(1, 1, 150, 0, 10, 1)
                print("robotiq open")
                sleep(2)
                rtde_c.moveL(pose_B,0.1,0.1)
                rtde_c.moveL(pose_middle,0.1,0.1)
                sleep(0.1)
            else:
                rtde_c.moveL(pose_B,0.1,0.1)
                rtde_c.moveL(pose_C,0.1,0.1)            

                # target=move_endtcp(pose_B,[0,0,1,0,0,0],[0,0,TCP_offset,0,0,0])  #沿tcp移动TCP_offset距离

                # rtde_c.moveL(target,0.02,0.01,True)
                task_frame2=rtde_r.getActualTCPPose()
                rtde_c.forceMode(task_frame2,selection_vctor,wrench,force_type,limits)
                sleep(0.2)
                while (abs(rtde_r.getActualTCPForce()[2]) < 5) and (abs(abs(rtde_r.getActualTCPPose()[2])-abs(pose_C[2]))<=0.02):
                    print("2")
                    sleep(0.01)

                print("stopL start")
                # 
                rtde_c.forceModeStop()
                print("stopL end")
                sleep(1)
                if abs(rtde_r.getActualTCPPose()[2]-pose_C[2])>=0.02:
                    print("插管成功，打开夹爪输入")
                # robotiq(1, 1, 150, 0, 10, 1)
                    print("robotiq open")
                    sleep(2)
                    rtde_c.moveL(pose_C,0.1,0.1)
                    rtde_c.moveL(pose_middle,0.1,0.1)
                    sleep(0.1)
"""def put_testtube_endtcp(pose_A,pose_B,pose_C,pose_middle):
    #思路AB两点
    #移动到A点，然后沿TCP末端Z方向移动
    #位置判断
    #   if机械臂末端受力大于3N转向B点，
    #   如果位置达到要求，直接松开夹爪
        TCP_offset=0.03                #移动TCP_z轴距离
        
        # task_frame=rtde_r.getActualTCPPose()
        # selection_vctor=[0,0,1,0,0,0]
        # wrench=[0,0,8,0,0,0]
        # force_type=2
        # limits=[1,1,1,1,1,1]
        #A点
        rtde_c.moveL(pose_A,0.1,0.1)            
        task_frame=rtde_r.getActualTCPPose()
        selection_vctor=[0,1,0,0,0,0]
        wrench=[0,8,0,0,0,0]
        force_type=2
        limits=[1,1,1,1,1,1]
        # target=move_endtcp(pose_A,[0,1,0,0,0,0],[0,TCP_offset,0,0,0,0])  #沿tcp移动TCP_offset距离

        # rtde_c.moveL(target,0.02,0.01,True)
        rtde_c.forceMode(task_frame,selection_vctor,wrench,force_type,limits)
        sleep(0.2)

        while (abs(rtde_r.getActualTCPForce()[2]) < 5) and (abs(abs(rtde_r.getActualTCPPose()[2])-abs(pose_A[2]))<=0.02):
                print("1")
                sleep(0.1)
        print("stopL start")
        rtde_c.forceModeStop()
        print("stopL end")
        sleep(1)

        if abs(rtde_r.getActualTCPPose()[1]-pose_A[1])>=0.02:
            print("插管成功，打开夹爪输入")
            # robotiq(1, 1, 150, 0, 10, 1)
            print("robotiq open")
            sleep(2)
            rtde_c.moveL(pose_A,0.1,0.1)
            rtde_c.moveL(pose_middle,0.1,0.1)
            sleep(0.1)
        else:
            #走到B点开始进行判断运行
            rtde_c.moveL(pose_A,0.1,0.1)
            rtde_c.moveL(pose_B,0.1,0.1)            

            # target=move_endtcp(pose_B,[0,1,0,0,0,0],[0,TCP_offset,0,0,0,0])  #沿tcp移动TCP_offset距离

            # rtde_c.moveL(target,0.02,0.01,True)
            task_frame1=rtde_r.getActualTCPPose()
            rtde_c.forceMode(task_frame1,selection_vctor,wrench,force_type,limits)
            sleep(0.2)
            while (abs(rtde_r.getActualTCPForce()[2]) < 5) and (abs(abs(rtde_r.getActualTCPPose()[2])-abs(pose_B[2]))<=0.02):
                print("2")
                sleep(0.01)

            print("stopL start")
            # 
            rtde_c.forceModeStop()
            print("stopL end")
            sleep(1)
            if abs(rtde_r.getActualTCPPose()[2]-pose_B[2])>=0.02:
                print("插管成功，打开夹爪输入")
            # robotiq(1, 1, 150, 0, 10, 1)
                print("robotiq open")
                sleep(2)
                rtde_c.moveL(pose_B,0.1,0.1)
                rtde_c.moveL(pose_middle,0.1,0.1)
                sleep(0.1)
            else:
                rtde_c.moveL(pose_B,0.1,0.1)
                rtde_c.moveL(pose_C,0.1,0.1)            
                task_frame2=rtde_r.getActualTCPPose()
                # target=move_endtcp(pose_C,[0,1,0,0,0,0],[0,TCP_offset,0,0,0,0])  #沿tcp移动TCP_offset距离

                # rtde_c.moveL(target,0.02,0.01,True)
                task_frame1=rtde_r.getActualTCPPose()
                rtde_c.forceMode(task_frame2,selection_vctor,wrench,force_type,limits)
                sleep(0.2)
                while (abs(rtde_r.getActualTCPForce()[2]) < 5) and (abs(abs(rtde_r.getActualTCPPose()[2])-abs(pose_C[2]))<=0.02):
                    print("2")
                    sleep(0.01)

                print("stopL start")
                # 
                rtde_c.forceModeStop()
                print("stopL end")
                sleep(1)
                if abs(rtde_r.getActualTCPPose()[1]-pose_C[2])>=0.02:
                    print("插管成功，打开夹爪输入")
                # robotiq(1, 1, 150, 0, 10, 1)
                    print("robotiq open")
                    sleep(2)
                    rtde_c.moveL(pose_C,0.1,0.1)
                    rtde_c.moveL(pose_middle,0.1,0.1)
                    sleep(0.1)"""
                

    
    #思路2
    #采用力模式
    # 移动到A点
    # 开始力模式
    #               

# def output_testtube():
#     TCP_offset=0.1

# def open_door():
#     handle_dist=0.058                     #门把手移动位移        
#     # pull_force=0                        #推门力的大小
#     middle_pose1=[0.305,-0.288,0.445,1.117,1.860,-0.880]                        #开门的中间示教点1
#     middle_pose2=[0.392,-0.515,0.492,1.485,1.682,-1.659]                        #开门的中间示教点2
#     middle_pose3=[]                        #开门的中间示教点3
#     end_pose=[-0.201,-0.385,0.552,2.245,0.519,-1.924]                            #开门的示教点终点
#     handle_q=[]                        #门把手路点
#     handle=[0.2991,-0.70727,0.39836,1.429,1.617,-0.874]                           #门把手位置
#     axis_pose=[-0.378,-0.77357-0.255,0.36746,0,0,0]                  #门的轴心
    
#     #开始程序
#     robotiq(1, 1, 50, 0, 10, 10)
#     #rtde_c.moveL(handle_q,0.1,0.1)
#     print("移动至门把手")
#     rtde_c.moveL(handle,0.1,0.1)     #移动至门把手位置
#     sleep(0.5)
    
#     print("抓住把手，关闭夹爪请输入")
#     input()
#     robotiq(1, 1, 200, 0, 10, 170)     #闭合夹爪

#     #是否开启forcemode,实验后再定
#     print("拉门把手，请输入")
#     input()
#     pose_now=rtde_r.getActualTCPPose()  
#     target_door=move_endtcp(pose_now,[0,0,1,0,0,0],[0,0,-handle_dist,0,0,0])  
#     ret=rtde_c.moveL(target_door,0.05,0.05)           #阻塞，沿tcp移动
#     print("ret",ret)
#     sleep(3)

#     print("适当松开夹爪")
#     input()
#     robotiq(1, 1, 162, 0, 10, 170)     #闭合夹爪
#     print("画圆推门")
#     input()

#     #画圆推门
#     offset=10*0.0175   #弧度转角度
#     offset_sum=0       #初始角度累加
#     goal2baselink=T.rot2T(axis_pose)     #目标相对于基座的T
#     eelink2baselink=rtde_r.getActualTCPPose()   #获得当前位姿
#     eelink2baselink=T.rot2T(eelink2baselink)    #当前位姿转换成T
#     goal2eelink=np.dot(np.linalg.inv(goal2baselink),eelink2baselink)  #goal2baselink*eelink2goal=eelink2baselink
#     P,M=T.T2rot_2(goal2eelink)   #旋转平移矩阵

#     radius=math.sqrt(math.pow(P[0],2)+math.pow(P[1],2))   #得R
#     angle=math.atan2(P[1],P[0])                           #的rad

#     for i in range(2):
#         offset_sum=offset_sum+offset
#         tran=[radius*math.cos(angle+offset_sum),radius*math.sin(angle+offset_sum),P[2]]
#         rot=[M[0],M[1],M[2]+offset_sum]
#         eelink2goal=[tran[0],tran[1],tran[2],rot[0],rot[1],rot[2]]
#         eelink2goal=T.rot2T(eelink2goal)
#         target=np.dot(goal2baselink,eelink2goal)
#         target=T.T2rot(target)       #目标位姿

#         ret=rtde_c.moveL(target,0.1,0.1)
#         print("move:",ret)
#         if ret==False:
#             offset_sum=offset_sum-offset
#         sleep(0.5)
#     sleep(1)

#     print("示教点1到达，张开夹爪请输入")

#     input()
#     #张开夹爪
#     robotiq(1, 1, 0, 0, 10, 10)
#     sleep(2)
#     pose_now=rtde_r.getActualTCPPose()  
#     pose_now[2]=pose_now[2]-0.1
#     rtde_c.moveL(pose_now,0.1,0.1)
#     robotiq(1, 1, 255, 0, 10, 10)
#     sleep(1)
#     print("移动至中间点，请输入")
#     input()
#     rtde_c.moveJ_IK(middle_pose1,0.1,0.1)
#     sleep(0.5)
#     rtde_c.moveJ_IK(middle_pose2,0.1,0.1)
#     sleep(0.5)
#     rtde_c.moveJ_IK(end_pose,0.1,0.1)
#     sleep(0.5)

#     #后续在说

# def close_door():

#     #示教一个点画圆
#     axis_pose=[-0.378,-0.77357-0.255,0.36746,0.164,-2.320,2.296]        #门的轴心
#     door_pose=[]        #门把手
#     max_force=40           #推门的最大力
#     max_force_2=40         #推门把手的最大力
#     offset=10*0.0175   #弧度转角度
#     offset_sum=0       #初始角度累加

#     goal2baselink=T.rot2T(pose)     #目标相对于基座的T
#     eelink2baselink=rtde_r.getActualTCPPose()   #获得当前位姿
#     eelink2baselink=T.rot2T(eelink2baselink)    #当前位姿转换成T
#     goal2eelink=np.dot(np.linalg.inv(goal2baselink),eelink2baselink)  #goal2baselink*eelink2goal=eelink2baselink
#     P,M=T.T2rot_2(goal2eelink)   #旋转平移矩阵

#     radius=math.sqrt(math.pow(P[0],2)+math.pow(P[1],2))   #得R
#     angle=math.atan2(P[1],P[0])                           #的rad

#     for i in range(9):
#         offset_sum=offset_sum+offset
#         tran=[radius*math.cos(angle+offset_sum),radius*math.sin(angle+offset_sum),P[2]]
#         rot=[M[0],M[1],M[2]]                                #末端姿态不随之改变
#         eelink2goal=[tran[0],tran[1],tran[2],rot[0],rot[1],rot[2]]
#         eelink2goal=T.rot2T(eelink2goal)
#         target=np.dot(goal2baselink,eelink2goal)
#         target=T.T2rot(target)       #目标位姿

#         ret=rtde_c.moveL(target,0.1,0.1,True)           #沿tcp z移动0.08

#         while abs(rtde_r.getActualTCPForce()[2]) < max_force:
#             print("push")
#         print("stopL start")
#         rtde_c.stopL(0.5)
#         print("stopL end")
#         print("push over")
#     sleep(1)

#     rtde_c.moveL(door_pose,0.1,0.1,True)  #示教推门把手的点door_pose

#     target=move_endtcp(door_pose,[0,0,1,0,0,0],[0,0,0.08,0,0,0])  
#     ret=rtde_c.moveL(target,0.1,0.1,True)           #沿tcp z移动0.08

#     while abs(rtde_r.getActualTCPForce()[2]) < max_force_2:
#         print("push")
#     print("stopL start")
#     rtde_c.stopL(0.5)
#     print("stopL end")
#     print("push over")
#     sleep(0.05)

# def cricle(rot_pose,eelink2baselink,offset):

#     TCP_offset=0.04                 #移动TCP_z轴距离
#     offset=offset*0.0175            #弧度转角度
#     # offset_sum=0                    #初始角度累加
#     goal2baselink=T.rot2T(rot_pose)      #目标相对于基座的T
#     eelink2baselink=T.rot2T(eelink2baselink)          #当前位姿转换成T
#     goal2eelink=np.dot(np.linalg.inv(goal2baselink),eelink2baselink)   #goal2baselink*eelink2goal=eelink2baselink
#     P,M=T.T2rot_2(goal2eelink)         #旋转平移矩阵

#     radius=math.sqrt(math.pow(P[0],2)+math.pow(P[1],2))   #得R
#     angle=math.atan2(P[1],P[0])                           #的rad

#     for i in range(6):
#     # offset_sum=offset_sum+offset
#     tran=[radius*math.cos(angle+offset),radius*math.sin(angle+offset),P[2]]
#     rot=[M[0],M[1],M[2]+offset]
#     eelink2goal=[tran[0],tran[1],tran[2],rot[0],rot[1],rot[2]]   #转动后目标相对于基座的T
#     eelink2goal=T.rot2T(eelink2goal)
#     target=np.dot(goal2baselink,eelink2goal)
#     target=T.T2rot(target)       #目标位姿
#     return target


if __name__ == '__main__':

    #初始化节点
    # rospy.init_node('move')

    #ur连接
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.3.101")
    rtde_c = RTDEControl("192.168.3.101", RTDEControl.FLAG_USE_EXT_UR_CAP)

    # pose_middle=[-0.49552,-0.13954,0.61938,3.170,1.390,-0.846]

    # pose_A=[-0.58938,-0.203,0.55649,3.455,1.116,-0.052]
    # pose_B=[-0.54533,-0.17609,0.54937,3.420,0.812,-0.514]
    # pose_C=[-0.57171,0.00234,0.55502,1.104,2.964,-0.772]
    # pose_D=[-0.57427,0.00454,0.55096,1.096,2.902,-0.796]
    
    #第一次求取过渡矩阵
    #T_aruco第一次示教的二维码位姿，goal为第一次示教的目标姿态
    #获得过渡矩阵
    """
    T_transition=goal_aruco(T_aruco, goal)
    #第二次求取目标点位姿,T_aruco为识别出来的二维码的位姿
    pose_goal=goal_pose(T_aruco, T_transition)"""
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

    wrench=[0,0,8,0,0,0]
    wrench1=[0,8,0,0,0,0]
    put_testtube_endforce(poseA_1,poseB_1,poseC_1,pose_middle,wrench)
    put_testtube_endforce(poseA_2,poseB_2,poseC_2,pose_middle,wrench)
    put_testtube_endtcp(poseA_3, poseB_3, poseC_3, pose_middle,wrench1)


    # rot_pose=[-0.75588,-0.0877,0.47653,3.107,0,0]   #圆的中心姿态
    # put_testtube(rot_pose,10)









