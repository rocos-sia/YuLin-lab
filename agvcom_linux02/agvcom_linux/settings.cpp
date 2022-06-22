#include "settings.h"

Settings::Settings()
{
    QFile settFile(settingsFileName);
    int res =-1;
    /* If settings file doesn't exist in the default directory
     * create new one with default value Or select one
     **/
    settings = new QSettings(settingsFileName, QSettings::IniFormat);
    if (!settFile.exists())
    {
        createDefaultSettings();
        qDebug() << "Creating default setting in: " << "done " << res;
    }
    /* Load settings from file
     **/
    res=loadSettings();
    qDebug() << "Loading setting from: " << "done " << res;
}

int Settings::loadSettings()
{
    settings->beginGroup("AGV");
    AGVIpAddress = settings->value("IPAddress").toString();
    settings->endGroup();
    return 0;
}

int Settings::saveSettings()
{
    settings->beginGroup("AGV");
    settings->setValue("IPAddress", AGVIpAddress);
    settings->endGroup();
    return 0;
}

int Settings::createDefaultSettings()
{
    settings->beginGroup("AGV");
    settings->setValue("IPAddress", AGVIpAddressDef);
    settings->endGroup();
    return 0;
}

std::string Settings::getIP()
{
    std::string ip = AGVIpAddress.toStdString();
    return ip;
}
