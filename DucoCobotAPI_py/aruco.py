import math
import sys
from time import sleep

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Pose

import Translate as T
sys.path.append('gen_py')
sys.path.append('lib')
##更改：
##
from DucoCobot import DucoCobot

# Connect!
ip = '192.168.1.10'
reu = duco_cobot = DucoCobot(ip, 7003)
print("~~~~~~~~~`",reu)
duco_cobot.open()
duco_cobot.power_on(True)
duco_cobot.enable(True)



class aruco:
    def __init__(self):
        # 标志位
        self.mark_flag = 0  # 识别累加标志位
        self.pose_flag = Pose()  # 累加位姿
        self.div = 100  # 识别100次动一次
        self.cout = 0  # 机械臂移动次数累加标志位
        self.end = 5  # 机械臂总的移动次数

        # test标志位
        self.mark_num = 0  # 累加标志位
        self.pose_num = Pose()  # 累加位姿
        self.num = 80  # 识别次数
        self.mark2camera = []  # 返回值

    def mark_callback(self, data):
        # 累加
        self.pose_flag.position.x = self.pose_flag.position.x + data.pose.position.x
        self.pose_flag.position.y = self.pose_flag.position.y + data.pose.position.y
        self.pose_flag.position.z = self.pose_flag.position.z + data.pose.position.z
        self.pose_flag.orientation.x = self.pose_flag.orientation.x + data.pose.orientation.x
        self.pose_flag.orientation.y = self.pose_flag.orientation.y + data.pose.orientation.y
        self.pose_flag.orientation.z = self.pose_flag.orientation.z + data.pose.orientation.z
        self.pose_flag.orientation.w = self.pose_flag.orientation.w + data.pose.orientation.w
        self.mark_flag = self.mark_flag + 1

        # 计算并移动机械臂
        if self.mark_flag == self.div:
            # mark旋转平移矩阵
            data_t = T.quat2T(self.pose_flag.position.x / self.div, self.pose_flag.position.y / self.div,
                              self.pose_flag.position.z / self.div, self.pose_flag.orientation.x / self.div,
                              self.pose_flag.orientation.y / self.div, self.pose_flag.orientation.z / self.div,
                              self.pose_flag.orientation.w / self.div)
            get_pose = duco_cobot.get_tcp_pose()
            eelink2baselink = T.rpy2T(get_pose)
            cammer2eelink = T.euler2T([0.02, -0.09, 0.03, 0, 0, math.pi])
            mark2cammer = T.rot2T([0, 0, 0.30, math.pi, 0, 0])
            # 计算机械臂移动距离
            eelink2baselink_2 = np.dot(np.dot(np.dot(np.dot(eelink2baselink, cammer2eelink),
                                                     data_t), np.linalg.inv(mark2cammer)), np.linalg.inv(cammer2eelink))
            send_command = T.T2rpy(eelink2baselink_2)

            print("send_command=", [send_command[0], send_command[1], send_command[2],
                                    send_command[3], send_command[4], send_command[5]])
            # 移动机械臂
            duco_cobot.movel([send_command[0], send_command[1], send_command[2],
                              send_command[3], send_command[4], send_command[5]], 0.2, 0.2, 0, [], "", "", True)

            # 标志位清零
            self.mark_flag = 0
            self.pose_flag.position.x = 0
            self.pose_flag.position.y = 0
            self.pose_flag.position.z = 0
            self.pose_flag.orientation.x = 0
            self.pose_flag.orientation.y = 0
            self.pose_flag.orientation.z = 0
            self.pose_flag.orientation.w = 0
            sleep(0.5)

            self.cout += 1

        if self.cout == self.end:
            self.cout = 0
            return 1
        else:
            return 0

    def mark(self, aruco_id):

        print("recognize aruco")
        # 订阅话题，矫正4次
        while True:
            data = rospy.wait_for_message("aruco_single/pose", PoseStamped)
            # print(data.header.stamp.secs)
            # print(type(data.header.stamp.secs))
            if data.header.stamp.secs == aruco_id:
                # print(data.header.stamp.secs)
                # print(type(data.header.stamp.secs))
                ret = self.mark_callback(data)
                if ret == 1:
                    print("二维码识别完成")
                    break
            else:
                print("识别到错误的aruco码")
                sleep(0.2)

    # test
    def test_callback(self, data):
        self.pose_num.orientation.x = self.pose_num.orientation.x + data.pose.orientation.x
        self.pose_num.orientation.y = self.pose_num.orientation.y + data.pose.orientation.y
        self.pose_num.orientation.z = self.pose_num.orientation.z + data.pose.orientation.z
        self.pose_num.orientation.w = self.pose_num.orientation.w + data.pose.orientation.w
        self.mark_num = self.mark_num + 1

        if self.mark_num == self.num:
            # mark旋转平移矩阵
            mark2camera = T.quat2euler([self.pose_num.orientation.x / self.num, self.pose_num.orientation.y / self.num,
                                        self.pose_num.orientation.z / self.num, self.pose_num.orientation.w / self.num])
            print("mark2camera", mark2camera)
            self.mark_num = 0
            self.pose_num.orientation.x = 0
            self.pose_num.orientation.y = 0
            self.pose_num.orientation.z = 0
            self.pose_num.orientation.w = 0
            return mark2camera
        return []

    def test_mark(self, aruco_id):
        print("recognize aruco")
        # 识别固定次数
        while True:
            data = rospy.wait_for_message("aruco_single/pose", PoseStamped)
            if data.header.stamp.secs == aruco_id:
                self.mark2camera = self.test_callback(data)
                if len(self.mark2camera) != 0:
                    print("二维码识别完成")
                    break
            else:
                print("识别到错误的aruco码")
                sleep(0.2)
        return self.mark2camera

if __name__ == '__main__':
    rospy.init_node("aruco")
    aruco=aruco()
    aruco.mark(582)

        