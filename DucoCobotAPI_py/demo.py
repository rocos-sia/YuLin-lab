import sys
import time
import threading
from DucoCobot import DucoCobot
import sys
import threading
import time
import logging
from DucoCobot import DucoCobot
import datetime
sys.path.append('gen_py')
sys.path.append('lib')
from thrift import Thrift

if __name__ == "__main__":
    ##日志记录
    logger_debug=logging.getLogger()
    logger_debug.setLevel(logging.DEBUG)  
    logger_deugFile = (
                "/home/sia/YuLin_lab/src/action_tset1/"
                + str(datetime.datetime.now().date().isoformat())
                + "debug.log"
            )
    fh_debug=logging.FileHandler(logger_deugFile, mode="a", encoding="utf-8")
    fh_debug.setLevel(logging.DEBUG)
    format = logging.Formatter(
                "%(asctime)s %(filename)s[line:%(lineno)d] %(levelname)s %(message)s"
            )
    fh_debug.setFormatter(format)
    logger_debug.addHandler(fh_debug)
    ##机械臂状态启动检测
    ip = '192.168.1.10'
    stopheartthread = False
    i=0
    while(i<5):
        try:
            print("connect the arm")
            duco_cobot = DucoCobot(ip, 7003)
            duco_cobot.open()
            duco_cobot.power_on(True)
            duco_cobot.enable(True)
            i=i+1
            array=duco_cobot.get_robot_state()
            if array[0]==6:
                print("机械臂状态正确，连接成功")
                logger_debug.debug("机械臂状态正确，连接成功")
                break
            
        except:
            logger_debug.debug("wait for a moment")
            time.sleep(5)
    if (i<5 ):  
        logger_debug.debug("continue ") 
    elif(i==5):
        logger_debug.debug("5次读取状态失败，需要人工干预")      
        assert False
    else:
        logger_debug.debug("意外情况产生，日志记录")      
        assert False

# def hearthread_fun():
#     duco_heartbeat = DucoCobot(ip, 7003)

#     if duco_heartbeat.open() == 0:
#         while not stopheartthread:
#             duco_heartbeat.rpc_heartbeat()
#             time.sleep(0.4)

#         duco_heartbeat.close()
#
#
# def thread_fun():
#     duco_cobot = DucoCobot(ip, 7003)
#     # Connect!
#     duco_cobot.open()
#     while not stopheartthread:
#         tcp_pose = duco_cobot.get_actual_joints_position()
#         # print("pos: ", tcp_pose)
#         time.sleep(1)
#
#     duco_cobot.close()


# def main():
#     # _thread.start_new_thread(thread_fun,())
#     # thd_A = threading.Thread(target=thread_fun)
#     # thd_A.start()
#     thd_B = threading.Thread(target=hearthread_fun)
#     thd_B.daemon = True
#     thd_B.start()

#     duco_cobot = DucoCobot(ip, 7003)

#     # op = Op()

#     # Connect!
#     rlt = duco_cobot.open()
#     print("open:", rlt)
#     rlt = duco_cobot.power_on(True)
#     print("power_on:", rlt)
#     rlt = duco_cobot.enable(True)
#     print("enable:", rlt)

#     ret = duco_cobot.movej([1.6679942607879639, 0.13228817284107208, -1.5182205438613892, -0.1812257021665573,
#      1.560260534286499, -0.630290687084198], 5, 5, 0, True)
#     time.sleep(2)
#     ret = duco_cobot.movej([1.7878724336624146, -0.264102041721344, -1.4782891273498535, 1.6719069480895996,
#      -0.03256284445524216, 2.3607442378997803], 5, 5, 0, True)


#     print("movej ", ret)

#     # filepath = "/home/ubuntu/tst.txt"
#     # track_list = []
#     #
#     # with open(filepath, 'r') as infile:
#     #     for line in infile:
#     #         str_list = line.split(" ")
#     #         num_list = [float(x) for x in str_list]
#     #         for i in range(3):
#     #             num_list[i] = num_list[i] / 1000
#     #             num_list[i + 3] = num_list[i + 3] * 3.1415926 / 180
#     #         track_list.append(num_list)
#     # infile.close()
#     # rlt = duco_cobot.trackClearQueue()
#     # print("clear queue: ", rlt)
#     # rlt = duco_cobot.getQueueSize()
#     # print("queue size: ", rlt)
#     # rlt = duco_cobot.trackEnqueue(track_list, True)
#     # print("enqueue: ", rlt)
#     # rlt = duco_cobot.getQueueSize()
#     # print("queue size: ", rlt)
#     # rlt = duco_cobot.trackEnqueue(track_list, True)
#     # print("enqueue: ", rlt)
#     # rlt = duco_cobot.getQueueSize()
#     # print("queue size: ", rlt)

#     # rlt = duco_cobot.getQueueSize()
#     # print("queue size: ", rlt)

#     # rlt = duco_cobot.trackCartMotion(0.08, 1, False, "default", "default")
#     # print("Cart motion: ", rlt)

#     # rlt = duco_cobot.trackJointMotion(0.08, 1, False)
#     # print("Joint motion: ", rlt)

#     # rlt = duco_cobot.run_program("eee.jspf", True)
#     # print("run:", rlt)
#     # Close!
#     # input()
#     # stopheartthread = True

#     rlt = duco_cobot.close()
#     print("close:", rlt)


# if __name__ == '__main__':
#     try:
#         main()
#     except Thrift.TException as tx:
#         print('%s' % tx.message)
