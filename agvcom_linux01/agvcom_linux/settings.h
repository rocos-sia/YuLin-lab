#ifndef SETTINGS_H
#define SETTINGS_H

#include <QString>
#include <QSettings>
#include <QDebug>
#include <QFile>

class Settings
{
public:
    Settings();
    QSettings *settings;
    int loadSettings();
    int saveSettings();
    int createDefaultSettings();
    std::string getIP();


private:
    QString AGVIpAddressDef="192.168.224.231";


    QString settingsFileName = "agvconfig.ini";
    QString AGVIpAddress;
};

#endif // SETTINGS_H
