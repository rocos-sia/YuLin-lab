#! /usr/bin/env python
import threading
import time
from socket import *
import sys
import actionlib
from action_tset1.msg import *
import logging
from datetime import datetime
from asyncio.log import logger
##
sys.path.append('/home/sia/YuLin_lab/src/DucoCobotAPI_py/gen_py')
sys.path.append('/home/sia/YuLin_lab/src/DucoCobotAPI_py/lib')
sys.path.append('/home/sia/YuLin_lab/src/DucoCobotAPI_py')

from lab import *
# 后期全局变量可以放置到元组中
"""
1.导包
2.初始化ROS节点
3.单独封装一个类
4.类中创建action服务端对象；
5.处理请求，（1，解析目标值，发送连续反馈，响应最终结果）---回调函数
6.spin()
"""

##
HOST = '127.0.0.1'

PORT = 60000

BUFFSIZE = 2048
ADDR1 = (HOST, PORT)
tt = socket(AF_INET, SOCK_STREAM)
tt.setsockopt(SOL_SOCKET,SO_REUSEADDR,1)
tt.bind(ADDR1)
tt.listen(3)
print(1)
ttClient, ttaddr = tt.accept()
print(ttClient)



if __name__ == "__main__":
    sleep(5)
    data = "13"
    ttClient.send(data.encode())
    plat = ttClient.recv(BUFFSIZE).decode()
    # data = "21"
    # ttClient.send(data.encode())
    # plat = ttClient.recv(BUFFSIZE).decode()