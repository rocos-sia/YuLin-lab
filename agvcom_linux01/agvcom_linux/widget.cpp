#include "widget.h"
#include "ui_widget.h"
#include <QDebug>
#include <vector>
#include <QValidator>

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);
    QRegExp regx("[1-9][0-9]+$");
    QValidator *validator = new QRegExpValidator(regx, ui->lineEdit );
    ui->lineEdit->setValidator(validator);
    ui->lineEdit_2->setText(QString::fromStdString("192.168.224.78"));
    this->init();
}

void Widget::init()
{
    AGVIp = "192.168.224.78";//IPconfig.getIP();
    memset(&m_LinkParam, 0, sizeof(xNETDRIVER::LINK_PARAM));
    m_LinkParam.DstAddr.SetIP(iptoint(AGVIp));
    m_LinkParam.LocalPort = xNETDRIVER::LOCAL_PORT_UDP;
    m_LinkParam.srcObjID = 0x92;
    m_LinkParam.dstObjID = 0x92;
    m_LinkParam.byDstLinkID = 0x14;
}

int Widget::iptoint(std::string ip)//ip地址转换为int整数
{
    std::vector<std::string> ret;
    size_t flag = 0;
    size_t start = 0, index = ip.find(".", 0);
    while (index != ip.npos)
    {
        if (start != index) {
            ret.push_back(ip.substr(start, index - start));
            flag = index;
        }
        start = index + 1;
        index = ip.find(".", start);
    }
    if (!ip.substr(start).empty())
        ret.push_back(ip.substr(start));
    unsigned char field[4];//
    for(int i = 0;i< ret.size();i++)
    {
        field[i] = atoi(ret.at(i).c_str());
    }
    return (unsigned int)(
        ((unsigned int)(field[0]) << 24) +
        ((unsigned int)(field[1]) << 16) +
        ((unsigned int)(field[2]) << 8) +
        ((unsigned int)(field[3]))
        );
}

Widget::~Widget()
{
    delete ui;
}


void Widget::on_pushButton_clicked()//状态信息，其他信息详见ROBOT_STATE_RECV_FMT结构体定义
{
    DEFINE_STRUCT_AND_INIT(xNETDRIVER::ROBOT_STATE_RECV_FMT, robot_state);
    int result_robot_state = xNETDRIVER::GetRobotState(&m_LinkParam, &robot_state);
    qDebug()<< result_robot_state<<robot_state.dPointX<< robot_state.dPointY<<robot_state.byRelocState<<robot_state.byTaskState<<robot_state.dwGoalID;
}

void Widget::on_pushButton_2_clicked()//开始导航,请先确定切换到自动模式，包括示教器切换到自动模式
{
    int id = ui->lineEdit->text().toInt();

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
    qDebug()<<result_send_way_point;
}
//充电操作需要先发送指令到充电点位，然后在执行此操作


//void Widget::on_lineEdit_cursorPositionChanged(int arg1, int arg2)
//{

//}
