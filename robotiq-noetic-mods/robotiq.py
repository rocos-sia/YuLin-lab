#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from time import sleep

def genCommand(ACT,GTO,PR,ATR,SP,FR):
    command = outputMsg.Robotiq2FGripper_robot_output();
    command.rACT = ACT
    command.rGTO = GTO
    command.rPR = PR
    command.rATR = ATR
    command.rSP  = SP
    command.rFR  = FR
    return command

if __name__ == "__main__":
        
        # 初始化ROS节点
    rospy.init_node('Robotiq') 
  
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
    command = outputMsg.Robotiq2FGripper_robot_output()
    for  i in range(2):
        command = genCommand(0,0,0,0,0,0)
        pub.publish(command)
        rospy.sleep(0.1)
    rospy.sleep(2)
    for  i in range(2):
        command = genCommand(1,1,0,0,20,1)
        pub.publish(command)
        rospy.sleep(0.1)
    rospy.loginfo("robotiq active")
    while 1:
        PR=int(input())
        for  i in range(2):
            command = genCommand(1,1,PR,0,20,1)
            pub.publish(command)
            rospy.sleep(0.1)

    

