#! /usr/bin/env python

# -*- encoding: utf-8 -*-
 #客户端

from socket import *

HOST ='127.0.0.1'

PORT = 50000

BUFFSIZE=2048

ADDR = (HOST,PORT)

Client = socket(AF_INET,SOCK_STREAM)

Client.connect(ADDR)

while True:
    print("请输入指令")
    data =input("<<")
    Client.send(data.encode())
    data = Client.recv(BUFFSIZE).decode()
    
    if not data:
        break
    print("发出的指令")
    print(data)
Client.close()


# switch={502:oo,#测试
#             503:doMsg,
#             504:sub_topic.doMsg1
#             }
# choice=np.array([502,503,504])
# for i in range(2):
#     switch.get(choice[i],default)()