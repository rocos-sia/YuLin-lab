#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <settings.h>
#include <xnetdriver/xNetDriver.h>
#include <xnetdriver/xNetRobotAcs.h>
#include <string>
#include <stdlib.h>

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    void init();
    int iptoint(std::string ip);
    ~Widget();

private slots:
    void on_pushButton_clicked();//获取机器人状态

    void on_pushButton_2_clicked();

    //void on_lineEdit_cursorPositionChanged(int arg1, int arg2);

private:
    Ui::Widget *ui;
    xNETDRIVER::LINK_PARAM m_LinkParam;
    Settings IPconfig;
    std::string AGVIp;
};
#endif // WIDGET_H
