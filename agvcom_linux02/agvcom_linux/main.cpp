#include "widget.h"
#include <QApplication>
#include <qdir.h>

#include <arpa/inet.h>
#include <errno.h>
#include <iostream>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#define MAXLINE 102400
QString PX;
QString PY;
QString Battery;
QString Task;
QString ID;
QString Robot;
int main(int argc, char *argv[])
{
//    QApplication a(argc, argv);

    std::string AGVIp;
    xNETDRIVER::LINK_PARAM m_LinkParam;
    AGVIp = "192.168.224.78";//IPconfig.getIP();
        memset(&m_LinkParam, 0, sizeof(xNETDRIVER::LINK_PARAM));
        m_LinkParam.DstAddr.SetIP(3232292942);
        m_LinkParam.LocalPort = xNETDRIVER::LOCAL_PORT_UDP;
        m_LinkParam.srcObjID = 0x92;
        m_LinkParam.dstObjID = 0x92;
        m_LinkParam.byDstLinkID = 0x14;

        // 建立AGV通讯
        xNETDRIVER::LINK_PARAM LinkParam;
        memset(&LinkParam, 0, sizeof(xNETDRIVER::LINK_PARAM));
        LinkParam.DstAddr.SetIP(3232292942);
        LinkParam.LocalPort = xNETDRIVER::LOCAL_PORT_UDP;
        LinkParam.srcObjID = 0x80;
        LinkParam.dstObjID = 0x80;
        LinkParam.byDstLinkID = 0x10;
//        DEFINE_STRUCT_AND_INIT(xNETDRIVER::ROBOT_STATE_RECV_FMT, robot_state);
//        int result_robot_state = xNETDRIVER::GetRobotState(&m_LinkParam, &robot_state);
//        qDebug()<<robot_state.byCharge;
        int sockfd;
        char buff_res[4096];
        std::string sendline;
        struct sockaddr_in servaddr;
        int count = 0;

        memset(&servaddr, 0, sizeof(servaddr)); //清空
        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(60000);
        servaddr.sin_addr.s_addr = inet_addr("127.0.0.1"); //服务器IP地址

        if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
        {
            printf("create socket error: %s(errno: %d)\n", strerror(errno), errno);
            return 0;
        }

        if (connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
        {
            printf("connect error: %s(errno: %d)\n", strerror(errno), errno);
            return 0;
        }

        char recv_buf[MAXLINE];
        double *jot_pos_ptr{nullptr};


        // //** 发送消息 **//
  while(1)
  {
      memset(recv_buf, 0, sizeof(recv_buf)); //清空
        ssize_t rr=recv(sockfd, recv_buf, MAXLINE, 0);
//        qDebug()<<recv(sockfd, recv_buf, MAXLINE, 0);
        if (recv_buf[0]=='0')
        {
            qDebug()<<"2";
            QString a;
            DEFINE_STRUCT_AND_INIT(xNETDRIVER::ROBOT_STATE_RECV_FMT, robot_state);
            int result_robot_state = xNETDRIVER::GetRobotState(&m_LinkParam, &robot_state);
//            qDebug()<< result_robot_state<<robot_state.dPointX<< robot_state.dPointY<<robot_state.byRelocState<<robot_state.byTaskState<<robot_state.dwGoalID;
             qDebug()<< robot_state.dPointX<< robot_state.dPointY<<robot_state.dBattery<<robot_state.byTaskState<<robot_state.dwGoalID;
            //X坐标，Y坐标，电池当前的电量（0-1），当前导航状态（0：无到导航点任务，1：等待，2：正在前往导航点，3：暂停，4：完成，5：失败，6：退出），当前目标点id
             result_robot_state=12;

            PX = QString::number(robot_state.dPointX, 10, 4);
            PY = QString::number(robot_state.dPointY, 10, 4);
            Battery = QString::number(robot_state.dBattery,10, 4);
            Task = QString::number(robot_state.byTaskState,10);
            ID = QString::number(robot_state.dwGoalID,10);

            Robot = QString("Robot_State,%1,%2,%3,%4,%5").arg(PX).arg(PY).arg(Battery).arg(Task).arg(ID);
            QString text= Robot;

            sendline=Robot.toStdString();
            int ret=send( sockfd, sendline.c_str( ), sendline.length( ), 0 ) ;
        }
        if (recv_buf[0]=='1')
        {
               qDebug()<<"first";
              int xxx = (int)(recv_buf[1] - '0');
               //int id = ui->lineEdit->text().toInt();
               qDebug()<<xxx;
              int id = xxx;
               DEFINE_STRUCT_AND_INIT(xNETDRIVER::AUTO_MANUAL_CHANGE_SEND_FMT, auto_manual_change);//切换模式接口
               auto_manual_change.byMode = 1;
               int result_auto_manual_change = xNETDRIVER::AutoManualChange(&m_LinkParam, &auto_manual_change);

               DEFINE_STRUCT_AND_INIT(xNETDRIVER::SEND_WAY_POINT_SEND_FMT, send_way_point);//自动导航接口
                // 目标点位PointName，需要转换为字符串
                char targetArr[5];
               sprintf(targetArr,"%d",id);
                 //_itoa_s(id,targetArr,5,10);//整数转字符串
               for (size_t i = 0; i < 5; i++)
               {
                 send_way_point.PointName[i] = targetArr[i];
               }
                int result_send_way_point = xNETDRIVER::SendWayPoint(&m_LinkParam, &send_way_point);

                DEFINE_STRUCT_AND_INIT(xNETDRIVER::ROBOT_STATE_RECV_FMT, robot_state);
                int result_robot_state = xNETDRIVER::GetRobotState(&m_LinkParam, &robot_state);
                while(robot_state.byTaskState!=4)
                {
                   result_robot_state = xNETDRIVER::GetRobotState(&m_LinkParam, &robot_state);
                   qDebug()<<"wait";
                   sleep(3);
                }
                sendline="00";
                int ret=send( sockfd, sendline.c_str( ), sendline.length( ), 0 ) ;
                qDebug()<<result_send_way_point;

                qDebug()<<"end";
        }

        //充电操作需要先发送指令到充电点位，然后在执行此操作
//        std::string command = ui->lineEdit_3->text().tostring();
        if (recv_buf[0]=='2')
         {
            std::string command(1,recv_buf[1]);
            if (command == "1" ||command == "2")
                {
                    //充电指令： 1-开始，2-结束
                    typedef struct
                    {
                        unsigned char name[16];
                        unsigned char value[256];
                    }TAG_DATA;
                    int tag_data_size = sizeof(TAG_DATA);
                    TAG_DATA send_data;
                    char *charge = "battery_command";
                    unsigned short comm = atoi(command.c_str());
                    memset(&send_data, 0, tag_data_size);
                    int nNameLen = strlen(charge);
                    memcpy((char *)send_data.name, charge, nNameLen > 16 ? 16 : nNameLen);
                    memcpy(send_data.value, (unsigned char *)&comm, 2);
                    //写变量
                    xNETDRIVER::RESPONSE ret = SendRequestAndWaitForRecv(&LinkParam, (xNETDRIVER::SERVICE)0x10, (xNETDRIVER::COMMAND)0x01, &send_data, tag_data_size, NULL, 0, 2000);
                    if (ret != xNETDRIVER::XNET_SUCCESS)
                    {
                        qDebug()<<"error";
                        //异常记录
                    }
                    sleep(6);
                    DEFINE_STRUCT_AND_INIT(xNETDRIVER::ROBOT_STATE_RECV_FMT, robot_state);
                    int result_robot_state = xNETDRIVER::GetRobotState(&m_LinkParam, &robot_state);
                    while(robot_state.byCharge!=0)
                    {
                       result_robot_state = xNETDRIVER::GetRobotState(&m_LinkParam, &robot_state);
                       qDebug()<<"charging";
                       sleep(20);
                    }
                    sendline="00";
                    send( sockfd, sendline.c_str( ), sendline.length( ), 0 ) ;
                }
        }


    }

}
