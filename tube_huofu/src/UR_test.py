#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
from rtde_control import RTDEControlInterface as RTDEControl
import rtde_control
import rtde_receive
# import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
# from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
# from Translate import T
import time
import threading
from Translate import  T
flag=0   ##求平均值累加次数
move_flag=0
id=1
pose_flag=Pose()  #定位移动次数
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.3.101")
rtde_c = RTDEControl("192.168.3.101", RTDEControl.FLAG_USE_EXT_UR_CAP)
def callback(data):
    global flag
    global pose_flag
    global move_flag
    
    pose_flag.position.x=pose_flag.position.x+data.pose.position.x
    pose_flag.position.y=pose_flag.position.y+data.pose.position.y
    pose_flag.position.z=pose_flag.position.z+data.pose.position.z
    pose_flag.orientation.x=pose_flag.orientation.x+data.pose.orientation.x
    pose_flag.orientation.y=pose_flag.orientation.y+data.pose.orientation.y
    pose_flag.orientation.z=pose_flag.orientation.z+data.pose.orientation.z
    pose_flag.orientation.w=pose_flag.orientation.w+data.pose.orientation.w
    flag=flag+1
    if(flag==30):
        #mark旋转平移矩阵
        
        print("flag=",flag)
        data_t = T.quat2T(pose_flag.position.x/30, pose_flag.position.y/30,pose_flag.position.z/30, pose_flag.orientation.x/30,pose_flag.orientation.y/30, pose_flag.orientation.z/30,pose_flag.orientation.w/30)
    
        #current_pose旋转平移矩阵
        get_pose = rtde_r.getActualTCPPose()
        # get_pose=[-0.451,0.292,0.295,3.136,-0.089,0.104]
        eelink2baselink=T.rot2T(get_pose)

        #cammer2eelink旋转平移矩阵
        cammer2eelink = np.array([[0.999894837721547,   0.0122271526577241,	  0.00779809179739681,	-0.0306775860527160],
        [-0.0121858291648088,	0.999911571799960,	  -0.00532486131673999,	-0.0896921720292136],
        [-0.00786251011836007,	0.00522927512768447,    0.999955416814289,	0.0116530639577847],
        [0,	0,	0,	1]])

        #设置目标mark2cammer
        mark2cammer = T.quat2T(0,0,0.4,1,0,0,0)

        #移动机械臂
        eelink2baselink_2=np.dot(np.dot(np.dot(np.dot(eelink2baselink,cammer2eelink),data_t),np.linalg.inv(mark2cammer)),np.linalg.inv(cammer2eelink))
        send_command=T.T2rot(eelink2baselink_2)
        print("send_command=",[send_command[0],send_command[1], send_command[2],send_command[3], send_command[4], send_command[5]])
        rtde_c.moveJ_IK([send_command[0],send_command[1], send_command[2],send_command[3], send_command[4], send_command[5]], 0.5, 0.3)
        
        #标志位
        flag=0
        pose_flag.position.x=0
        pose_flag.position.y=0
        pose_flag.position.z=0
        pose_flag.orientation.x=0
        pose_flag.orientation.y=0
        pose_flag.orientation.z=0
        pose_flag.orientation.w=0
        move_flag=move_flag+1
        print("move_flag=",move_flag)

        
       

def mark():

    # 初始化ROS节点
    # rospy.init_node('marker_move')
    
    #标志位
    ra=0

    print("start callback")
    
    while ra<150:
        data=rospy.wait_for_message("aruco_single/pose", PoseStamped)
        callback(data)
        ra+=1
   