import sys
import time
import threading
from DucoCobot import DucoCobot
import sys
import threading
import time

from DucoCobot import DucoCobot

sys.path.append('gen_py')
sys.path.append('lib')
from thrift import Thrift

ip = '192.168.1.10'
stopheartthread = False


try:
    print("connect the arm")
    duco_cobot = DucoCobot(ip, 7003)
    duco_cobot.open()
    duco_cobot.power_on(True)
    duco_cobot.enable(True)
    # time.sleep(5)
    # array=duco_cobot.get_robot_state()
    # if array[0]==6:
    #     break
    print("connect success")
except:
    print("wait for a moment")
    time.sleep(5)
      
print("continue ")       

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
