#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
from geometry_msgs.msg import PoseStamped,Pose
import numpy as np
from rtde_control import RTDEControlInterface as RTDEControl
import rtde_control
import rtde_receive
from Translate import T
import math
from time import sleep

#连接UR
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.3.101")
rtde_c = RTDEControl("192.168.3.101", RTDEControl.FLAG_USE_EXT_UR_CAP)

class aruco:

    #标志位
    mark_flag=0       #累加标志位
    pose_flag=Pose()  #累加位姿
    div=90

    def mark_callback(data):
        
        aruco.pose_flag.position.x=aruco.pose_flag.position.x+data.pose.position.x
        aruco.pose_flag.position.y=aruco.pose_flag.position.y+data.pose.position.y
        aruco.pose_flag.position.z=aruco.pose_flag.position.z+data.pose.position.z
        aruco.pose_flag.orientation.x=aruco.pose_flag.orientation.x+data.pose.orientation.x
        aruco.pose_flag.orientation.y=aruco.pose_flag.orientation.y+data.pose.orientation.y
        aruco.pose_flag.orientation.z=aruco.pose_flag.orientation.z+data.pose.orientation.z
        aruco.pose_flag.orientation.w=aruco.pose_flag.orientation.w+data.pose.orientation.w
        aruco.mark_flag=aruco.mark_flag+1

        if(aruco.mark_flag==aruco.div):
            #mark旋转平移矩阵

            data_t = T.quat2T(aruco.pose_flag.position.x/aruco.div, aruco.pose_flag.position.y/aruco.div,aruco.pose_flag.position.z/aruco.div,
             aruco.pose_flag.orientation.x/aruco.div,aruco.pose_flag.orientation.y/aruco.div, aruco.pose_flag.orientation.z/aruco.div,aruco.pose_flag.orientation.w/aruco.div)
        
            #current_pose旋转平移矩阵
            get_pose = rtde_r.getActualTCPPose()
            eelink2baselink=T.rot2T(get_pose)

            #cammer2eelink旋转平移矩阵
            cammer2eelink = np.array([[-0.167184365100458,	-0.945608083445101,	-0.279060460454787,	200.134481114873],
            [0.958025425895036,	-0.0889515978419707,	-0.272534211760746,	11.6377705717887],
            [0.232887679804359,	-0.312910475639039,	0.920787903282135,	298.048028288196],
            [0,	0,	0,	1]])

            #设置目标mark2cammer
            mark2cammer = T.rot2T([0,0,0.3,math.pi,0,0])

            #移动机械臂
            eelink2baselink_2=np.dot(np.dot(np.dot(np.dot(eelink2baselink,cammer2eelink),data_t),np.linalg.inv(mark2cammer)),np.linalg.inv(cammer2eelink))
            send_command=T.T2rot(eelink2baselink_2)
            print("send_command=",[send_command[0],send_command[1], send_command[2],send_command[3], send_command[4], send_command[5]])
            rtde_c.moveJ_IK([send_command[0],send_command[1], send_command[2],send_command[3], send_command[4], send_command[5]], 0.5, 0.3)
            sleep(0.5)
            #标志位
            aruco.mark_flag=0
            aruco.pose_flag.position.x=0
            aruco.pose_flag.position.y=0
            aruco.pose_flag.position.z=0
            aruco.pose_flag.orientation.x=0
            aruco.pose_flag.orientation.y=0
            aruco.pose_flag.orientation.z=0
            aruco.pose_flag.orientation.w=0

    def mark():

        #初始化节点
        # rospy.init_node('move')
        print("listening mark")

        #订阅话题，矫正5次
        i=0
        while i<540:
            data=rospy.wait_for_message("aruco_single/pose", PoseStamped)
            aruco.mark_callback(data)
            i=i+1
        # return data.id
           




